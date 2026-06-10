# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Commit 2 — detection + parameter read.

``NatNetServerManager`` makes the extension *aware* of interface prims without
acting on them yet: it scans the stage, reads each interface's config, and prints
the parsed parameters. It re-scans when the stage is opened and when a NatNet prim
changes (``Usd.Notice.ObjectsChanged``). No server, no pose sampling — those come
in Commits 3 and 4.

``format_interface`` is a pure function (no USD) so it's trivially testable. All
``pxr`` / ``omni`` imports are lazy so importing this module stays hermetic.
"""

from __future__ import annotations

from .config import NatNetInterfaceConfig
from .usd_bindings import find_interfaces, read_interface


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
    """Detects interface prims and prints their parameters on stage/USD changes."""

    def __init__(self):
        self._stage_event_sub = None
        self._usd_listener = None
        self._scan_tick_sub = None
        self._scan_pending = False

    # --- lifecycle -------------------------------------------------------------

    def on_startup(self):
        import omni.usd

        usd_context = omni.usd.get_context()
        self._stage_event_sub = usd_context.get_stage_event_stream().create_subscription_to_pop(
            self._on_stage_event, name="natnet_manager_stage_events"
        )
        self._register_usd_listener()
        print("[natnet] NatNetServerManager initialized")
        self.scan_and_print()

    def on_shutdown(self):
        self._stage_event_sub = None
        self._scan_tick_sub = None
        self._scan_pending = False
        self._revoke_usd_listener()

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
        # Only re-scan when something NatNet-related changed, so we don't spam the
        # console on every transform update while the sim is playing.
        try:
            paths = list(notice.GetResyncedPaths()) + list(notice.GetChangedInfoOnlyPaths())
        except Exception:  # pragma: no cover - defensive
            paths = []
        if any(("NatNetInterface" in str(p)) or ("natnet:" in str(p)) for p in paths):
            # A single author_interface() (Create/Save) emits many notices — one per
            # attribute/relationship op. Debounce them into one scan on the next
            # update tick so we print the final state once, not once per op.
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
