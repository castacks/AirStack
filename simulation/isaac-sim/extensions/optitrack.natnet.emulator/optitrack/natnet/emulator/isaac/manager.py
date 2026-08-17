# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""
``NatNetServerManager`` detects interface prims, samples poses from the stage,
and owns a **single** server instance it can start and stop. 
On each enable it builds a MODELDEF catalog from the config and constructs a
fresh server via an injectable factory.
"""

from __future__ import annotations

from .catalog import build_catalog, find_duplicate_targets
from .config import DEFAULT_UP_AXIS, NatNetInterfaceConfig
from .frames import BodySample, apply_pose_noise, build_frame, to_motive_pose
from .usd_bindings import find_interfaces, read_interface, read_world_pose, resolve_targets


def _catalog_signature(config: NatNetInterfaceConfig):
    """Identity of the catalog (body id/name set) — changes trigger a MODELDEF refresh."""
    return tuple((b.streaming_id, b.rigid_body_name) for b in config.bodies)


def _parse_version(version_str: str) -> tuple[int, int, int, int]:
    try:
        parts = tuple(int(x) for x in str(version_str).split("."))
    except ValueError:
        parts = ()
    return (parts + (0, 0, 0, 0))[:4]


def default_server_factory(config: NatNetInterfaceConfig):
    """Construct (but do not start) a ``NatNetUnicastServer`` from a config."""
    from ..server import NatNetUnicastServer, TransmissionType

    if config.mode != "unicast":
        raise NotImplementedError(
            f"mode {config.mode!r} is not supported yet (unicast only)"
        )
    server = NatNetUnicastServer(
        local_interface=config.server_ip,
        transmission_type=TransmissionType.UNICAST,
        multicast_address=None,
        command_port=config.command_port,
        data_port=config.data_port,
    )
    server.publish_rate = config.publish_rate
    server.natnet_version = _parse_version(config.natnet_version)
    return server


def format_interface(prim_path: str, cfg: NatNetInterfaceConfig) -> str:
    """Render a human-readable multi-line summary of one interface config."""
    lines = [f"[natnet] Interface @ {prim_path}"]
    lines.append(f"  serverEnabled : {cfg.server_enabled}")
    lines.append(f"  serverIp      : {cfg.server_ip}")
    lines.append(f"  mode          : {cfg.mode}")
    if cfg.mode == "multicast":
        lines.append(f"  multicastAddr : {cfg.multicast_addr}")
    lines.append(f"  commandPort   : {cfg.command_port}")
    lines.append(f"  dataPort      : {cfg.data_port}")
    lines.append(f"  publishRate   : {cfg.publish_rate}")
    lines.append(f"  natnetVersion : {cfg.natnet_version}")
    lines.append(f"  upAxis        : {cfg.up_axis}")
    lines.append(f"  poseNoise     : enabled={cfg.pose_noise_enabled}")
    lines.append(
        f"                  std={cfg.pose_noise_std_meters} m, rot={cfg.pose_noise_rotation_deg} deg"
    )
    if cfg.bodies:
        lines.append(f"  bodies ({len(cfg.bodies)}):")
        for b in cfg.bodies:
            target = b.target_prim or "<no target>"
            lines.append(
                f"    - {b.rigid_body_name} (id={b.streaming_id}, parent={b.parent_id}) -> {target}"
            )
    else:
        lines.append("  bodies        : (none)")
    return "\n".join(lines)


class NatNetServerManager:
    """Detects interface prims, prints config, and owns one server instance."""

    def __init__(self, server_factory=None):
        self._stage_event_sub = None
        self._usd_listener = None
        self._scan_tick_sub = None
        self._scan_pending = False
        self._timeline_sub = None
        self._server = None
        self._server_factory = server_factory or default_server_factory
        # Sampling state. A NatNet prim edit sets ``_needs_resync``; the next physics
        # sample re-reads the catalog/targets and clears it.
        self._needs_resync = False
        self._sample_cache: list = []
        self._frame_counter = 0
        self._catalog_signature = None
        self._physx_sub = None
        # Streamed up-axis, re-read on every resync. See frames.to_motive_pose.
        self._up_axis = DEFAULT_UP_AXIS
        # Pose noise.
        self._pose_noise_enabled = False
        self._pose_noise_std_meters = 0.0
        self._pose_noise_rotation_deg = 0.0

    # --- lifecycle -------------------------------------------------------------

    def on_startup(self):
        import omni.usd

        usd_context = omni.usd.get_context()
        self._stage_event_sub = usd_context.get_stage_event_stream().create_subscription_to_pop(
            self._on_stage_event, name="natnet_manager_stage_events"
        )
        self._register_usd_listener()
        self._subscribe_physics()
        self._subscribe_timeline()
        print("[natnet] NatNetServerManager initialized")
        self.scan_and_print()

    def on_shutdown(self):
        self.stop_server()
        self._physx_sub = None
        self._timeline_sub = None
        self._stage_event_sub = None
        self._scan_tick_sub = None
        self._scan_pending = False
        self._revoke_usd_listener()

    def _subscribe_physics(self):
        # Sample + enqueue poses on every physics step (only fires while playing).
        try:
            import omni.physx

            self._physx_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(
                self._on_physics_step
            )
        except Exception as exc:  # Kit/physx only
            print(f"[natnet] Physics step subscription unavailable: {exc}")
            self._physx_sub = None

    def _on_physics_step(self, _dt):
        if self._server is not None:
            self.sample_once()

    # --- timeline-driven lifecycle ---------------------------------------------

    def _subscribe_timeline(self):
        """Bind the server's lifetime to the sim: Play starts it, Stop shuts it down.

        Play builds the server from the prim, so ``serverIp``/ports/``mode`` — bound
        into the socket at construction — pick up whatever is authored at that point.
        Body and noise edits need no rebuild; ``_resync`` re-reads them while running.
        """
        try:
            import omni.timeline

            self._timeline_sub = (
                omni.timeline.get_timeline_interface()
                .get_timeline_event_stream()
                .create_subscription_to_pop(self._on_timeline_event)
            )
        except Exception as exc:  # Kit only
            print(f"[natnet] Timeline subscription unavailable: {exc}")
            self._timeline_sub = None

    def _on_timeline_event(self, event):
        import omni.timeline

        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            self._start_for_play()
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            self.stop_server()

    def _start_for_play(self):
        """Start from the stage on Play, honouring the prim's ``serverEnabled``."""
        if self.is_running:
            return
        stage = self._get_stage()
        if stage is None:
            return
        interfaces = find_interfaces(stage)
        if not interfaces:
            return
        config = read_interface(interfaces[0])
        if not config.server_enabled:
            print("[natnet] Play: serverEnabled is false — not starting.")
            return
        self.log_target_diagnostics(config)
        self.start_server(config)

    # --- scanning --------------------------------------------------------------

    def scan_and_print(self, *_):
        """Find every interface prim and print its parsed config."""
        stage = self._get_stage()
        if stage is None:
            return
        interfaces = find_interfaces(stage)
        if not interfaces:
            print("[natnet] Scan: no NatNetInterface prims on stage.")
            return
        print(f"[natnet] Scan: {len(interfaces)} interface(s) detected.")
        for prim in interfaces:
            cfg = read_interface(prim)
            print(format_interface(prim.GetPath().pathString, cfg))

    # --- server lifecycle (single instance; USD-free, factory-injectable) ------

    @property
    def is_running(self) -> bool:
        return self._server is not None

    @property
    def server(self):
        return self._server

    def start_server(self, config: NatNetInterfaceConfig) -> bool:
        """Build the catalog, construct a fresh server, and start it — once.

        Idempotent: if a server is already running this is a no-op returning False.
        Returns True when a new server was created and started.
        """
        if self._server is not None:
            print("[natnet] start_server ignored: a server is already running.")
            return False
        catalog = build_catalog(config)
        server = self._server_factory(config)
        server.set_model_def_payload(catalog.pack())
        # Pump frames from the physics-step thread; the server's own background timer
        # is starved by Kit's render/physics main loop.
        if hasattr(server, "auto_stream"):
            server.auto_stream = False
        server.start()
        self._server = server
        # Build the prim->pose cache from the live stage on the first sampled frame.
        self._needs_resync = True
        self._frame_counter = 0
        # None so the first resync reports "changed" and the first frame flags
        # model_list_changed, prompting the client to read MODELDEF.
        self._catalog_signature = None
        print(
            f"[natnet] Server started on {config.server_ip} "
            f"(cmd {config.command_port} / data {config.data_port}) "
            f"with {len(config.bodies)} body(ies)."
        )
        return True

    def stop_server(self) -> bool:
        """Shut down the running server (fresh instance is built on next start).

        Idempotent: returns False if nothing was running.
        """
        if self._server is None:
            return False
        try:
            self._server.shutdown()
        finally:
            self._server = None
            self._sample_cache = []
            self._needs_resync = False
        print("[natnet] Server stopped.")
        return True

    def toggle_server(self, config: NatNetInterfaceConfig) -> bool:
        """Start if stopped, stop if running. Returns the resulting running state."""
        if self.is_running:
            self.stop_server()
        else:
            self.start_server(config)
        return self.is_running

    def apply_enabled(self, config: NatNetInterfaceConfig) -> None:
        """Reconcile running state to ``config.server_enabled`` (start/stop)."""
        if config.server_enabled and not self.is_running:
            self.start_server(config)
        elif not config.server_enabled and self.is_running:
            self.stop_server()

    def log_target_diagnostics(self, config: NatNetInterfaceConfig) -> None:
        """Warn about missing target prims and duplicate targets (best-effort)."""
        stage = self._get_stage()
        if stage is not None:
            _existing, missing = resolve_targets(stage, config)
            for body in missing:
                print(
                    f"[natnet] WARNING: body '{body.rigid_body_name}' target prim "
                    f"missing or empty: {body.target_prim or '<empty>'}"
                )
        for path in find_duplicate_targets(config):
            print(f"[natnet] WARNING: multiple bodies target the same prim: {path}")

    # --- scripting entry point -------------------------------------------------

    def start_from_stage(self) -> bool:
        """Find the interface prim on the current stage, read it, and start.

        Convenience for scripts/Pegasus launchers: author the prim (see
        ``author_interface``) then call this. Returns False if nothing to start.
        """
        stage = self._get_stage()
        if stage is None:
            print("[natnet] start_from_stage: no active stage.")
            return False
        interfaces = find_interfaces(stage)
        if not interfaces:
            print("[natnet] start_from_stage: no NatNetInterface prim found.")
            return False
        config = read_interface(interfaces[0])
        self.log_target_diagnostics(config)
        return self.start_server(config)

    # --- pose sampling + dynamic catalog (the data-enqueue path) ---------------

    def mark_dirty(self) -> None:
        """Flag that the on-stage config changed; next sample re-reads the catalog."""
        self._needs_resync = True

    def _resync(self, stage) -> bool:
        """Re-read the interface config, rebuild the catalog, and re-resolve targets.

        Returns True if the catalog (body id/name set) actually changed, so the next
        frame can flag ``model_list_changed`` and the client re-requests MODELDEF.
        """
        interfaces = find_interfaces(stage)
        if not interfaces:
            self._sample_cache = []
            return False
        config = read_interface(interfaces[0])
        self._up_axis = config.up_axis
        self._pose_noise_enabled = config.pose_noise_enabled
        self._pose_noise_std_meters = config.pose_noise_std_meters
        self._pose_noise_rotation_deg = config.pose_noise_rotation_deg
        if self._server is not None:
            self._server.set_model_def_payload(build_catalog(config).pack())
        # Cache target paths, not prim handles, so bodies whose target prim appears
        # after the server starts begin streaming as soon as it exists.
        self._sample_cache = [
            (body.streaming_id, body.rigid_body_name, body.target_prim)
            for body in config.bodies
        ]
        signature = _catalog_signature(config)
        changed = signature != self._catalog_signature
        self._catalog_signature = signature
        return changed

    def sample_once(self, stage=None):
        """Sample every body's USD world pose and enqueue one frame to the server.

        Resyncs the catalog first if the config is dirty (so bodies added/removed
        live are picked up). Returns the enqueued frame (or None if nothing to do).
        """
        if self._server is None:
            return None
        if stage is None:
            stage = self._get_stage()
        if stage is None:
            return None

        model_changed = False
        if self._needs_resync:
            model_changed = self._resync(stage)
            self._needs_resync = False

        samples = []
        for streaming_id, _name, target_path in self._sample_cache:
            prim = stage.GetPrimAtPath(target_path) if target_path else None
            pose = read_world_pose(prim) if prim is not None else None
            if pose is None:
                samples.append(BodySample.lost(streaming_id))
            else:
                position, orientation = to_motive_pose(*pose, up_axis=self._up_axis)
                if self._pose_noise_enabled:
                    position, orientation = apply_pose_noise(position, orientation, self._pose_noise_std_meters, self._pose_noise_rotation_deg)
                samples.append(BodySample(streaming_id, position, orientation, valid=True))

        frame = build_frame(
            self._frame_counter, samples, model_list_changed=model_changed
        )
        self._frame_counter += 1
        self._server.enqueue_mocap_data(frame)
        # Send synchronously from this (physics-step) thread.
        flush_mocap_data = getattr(self._server, "flush_mocap_data", None)
        if callable(flush_mocap_data):
            flush_mocap_data()
        return frame

    # --- stage / USD notifications --------------------------------------------

    def _get_stage(self):
        import omni.usd

        return omni.usd.get_context().get_stage()

    def _on_stage_event(self, event):
        import omni.usd

        if event.type == int(omni.usd.StageEventType.OPENED):
            self._register_usd_listener()
            self.scan_and_print()

    def _register_usd_listener(self):
        from pxr import Tf, Usd

        stage = self._get_stage()
        if stage is None:
            return
        self._revoke_usd_listener()
        self._usd_listener = Tf.Notice.Register(
            Usd.Notice.ObjectsChanged, self._on_objects_changed, stage
        )

    def _revoke_usd_listener(self):
        if self._usd_listener is not None:
            self._usd_listener.Revoke()
            self._usd_listener = None

    def _on_objects_changed(self, notice, sender):
        # Only re-scan when something NatNet-related changed
        try:
            paths = list(notice.GetResyncedPaths()) + list(notice.GetChangedInfoOnlyPaths())
        except Exception:
            paths = []
        if any(("NatNetInterface" in str(p)) or ("natnet:" in str(p)) for p in paths):
            # A NatNet prim changed: mark the sampler dirty so the next physics step re-reads the catalog.
            self._needs_resync = True
            # Debounce author_interface() calls into one scan on the next update tick.
            self._request_scan()

    def _request_scan(self):
        if self._scan_pending:
            return
        self._scan_pending = True
        import omni.kit.app

        self._scan_tick_sub = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(self._on_scan_tick, name="natnet_manager_scan_tick")
        )

    def _on_scan_tick(self, _event):
        self._scan_pending = False
        self._scan_tick_sub = None
        self.scan_and_print()
