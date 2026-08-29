#!/usr/bin/env python
"""Shared Pegasus launch-script infrastructure.

Every AirStack Isaac Sim launch script follows the same skeleton: create the
``SimulationApp`` (honoring livestream/headless env vars), enable the required
Kit extensions, build a Pegasus world, load and prepare an environment, spawn
PX4 multirotors with their sensor subgraphs, and step the timeline. This module
owns that skeleton once; the scripts in this directory reduce to scenario
declarations (environment URL, drone configs, sensor toggles) plus optional
hooks.

Import-order contract (Kit requires ``SimulationApp`` to exist before any
``omni.*`` / ``pegasus.*`` import), which every launch script must follow:

    from pegasus_app import create_simulation_app
    simulation_app = create_simulation_app()          # FIRST
    from pegasus.simulator.params import SIMULATION_ENVIRONMENTS   # now safe
    from pegasus_app import PegasusApp

This module therefore imports nothing from omni/pegasus at module scope —
only inside functions that run after ``create_simulation_app()``.

Env vars honored by ``create_simulation_app`` (uniform across all scripts):
 - ``ISAAC_SIM_LIVESTREAM``: headless + WebRTC livestream (UI rendered into
   the stream); ``ISAAC_SIM_LIVESTREAM_UDP_PORT`` pins the media port.
 - ``ISAAC_SIM_HEADLESS``: plain headless (no livestream).

Env vars honored by ``PegasusApp``:
 - ``PLAY_SIM_ON_START`` (default true): autoplay the timeline after setup.
 - ``ISAAC_SIM_FOLLOW_CAM`` (default ``1``): domain id of the drone the
   viewport follow-camera tracks; ``off``/``none``/``0`` disables it. The
   follow-cam (``/World/follow_cam``) chases the drone with smoothing and
   keeps it centered — the stock perspective camera often starts inside
   geometry on cm-authored stages (black viewport), so the viewport is
   switched to the follow-cam at startup.
 - ``ISAAC_SIM_FOLLOW_CAM_OFFSET`` (default ``-5,-5,2.5``): world-frame
   ``x,y,z`` offset (meters) from the tracked drone to the camera.
"""

import os
import sys
import time
import asyncio

import carb

# Make sibling helpers (gps_utils) and ../utils (scene_prep) importable for
# this module and for the launch scripts that import it.
_LAUNCH_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _LAUNCH_SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_SCRIPTS_DIR)
_UTILS_DIR = os.path.normpath(os.path.join(_LAUNCH_SCRIPTS_DIR, "..", "utils"))
if _UTILS_DIR not in sys.path:
    sys.path.insert(0, _UTILS_DIR)

# The single copy of the Pegasus Iris asset path (previously repeated in every
# launch script).
DEFAULT_DRONE_USD = (
    "~/.local/share/ov/data/documents/Kit/shared/exts/pegasus.simulator/"
    "pegasus/simulator/assets/Robots/Iris/iris.usd"
)

# Kit extensions every AirStack Pegasus scene needs. ``isaacsim.core.nodes``
# is required by the sensor OmniGraphs; harmless where unused.
REQUIRED_EXTENSIONS = [
    "omni.graph.core",                  # Core runtime for OmniGraph engine
    "omni.graph.action",                # Action Graph framework
    "omni.graph.action_nodes",          # Built-in Action Graph node library
    "isaacsim.core.nodes",              # Isaac Sim compute nodes (sensors)
    "omni.graph.ui",                    # UI scaffolding for graph tools
    "omni.graph.visualization.nodes",   # Visualization helper nodes
    "omni.graph.scriptnode",            # Python script node support
    "omni.graph.window.action",         # Action Graph editor window
    "omni.graph.window.generic",        # Generic graph UI tools
    "omni.graph.ui_nodes",              # UI node building helpers
    "pegasus.simulator",
]

# Set by create_simulation_app(); PegasusApp.run() steps it.
SIMULATION_APP = None


def create_simulation_app(launch_config=None):
    """Create the ``SimulationApp``, honoring livestream/headless env vars.

    Must be the first Isaac call in every launch script — before importing any
    ``omni.*`` or ``pegasus.*`` module. Pass ``launch_config`` to override the
    derived configuration entirely.

    When ``ISAAC_SIM_LIVESTREAM=true``, mirrors the NVIDIA reference config from
    /isaac-sim/standalone_examples/api/isaacsim.simulation_app/livestream.py (in-image)
    so the Kit GUI (menu bar, toolbar, viewport, status bar) actually gets
    rendered into the WebRTC stream instead of just the bare 3D viewport.
    Key field: ``hide_ui: False`` — SimulationApp's default when ``headless=True``
    is to also hide the UI; the livestream reference opts back into showing it.
    ``display_options=3286`` is the same bitmask the reference uses to keep the
    default grid + axes visible at scene start.
    """
    global SIMULATION_APP
    from isaacsim import SimulationApp

    livestream = os.environ.get("ISAAC_SIM_LIVESTREAM", "").lower() == "true"
    headless = os.environ.get("ISAAC_SIM_HEADLESS", "false").lower() == "true"

    if launch_config is None:
        if livestream:
            launch_config = {
                "width": 1280,
                "height": 720,
                "window_width": 1920,
                "window_height": 1080,
                "headless": True,
                "hide_ui": False,
                "renderer": "RaytracedLighting",
                "display_options": 3286,
            }
        else:
            launch_config = {"headless": headless}

    app = SimulationApp(launch_config=launch_config)

    if livestream:
        from isaacsim.core.utils.extensions import enable_extension

        app.set_setting("/app/window/drawMouse", True)
        app.set_setting("/app/livestream/enabled", True)

        # Pin the UDP media port so it stays inside the narrow set of ports we
        # publish from this container and that `airstack osmo:webrtc` forwards.
        #
        # Kit 107's WebRTC livestream picks a UDP media port dynamically. The
        # documented `omni.services.livestream.nvcf` defaults were
        # minHostPort=47998 / maxHostPort=48020 / fixedHostPort=0, but the
        # actual Kit binary ignored that range on airstack-dev-13 and bound to
        # UDP 49042 — outside both the Compose-published port range AND the
        # default osmo `--udp` forward (47995-48012,49000-49007). Result:
        # signaling worked (TCP 49100), the WebRTC Streaming Client window
        # opened, but every media packet was dropped → black viewport +
        # the `NVST_CCE_DISCONNECTED when m_connectionCount 0 != 1` underflow
        # storm in the Kit log.
        #
        # Set all three settings so whichever code path the plugin reads, it
        # lands on UDP 49099. The value of 49099 is picked as one-off from the
        # 49100 signaling port — same range, easy to remember, and TCP/UDP can
        # coexist on the same number if anyone later wants a single port.
        port = int(os.environ.get("ISAAC_SIM_LIVESTREAM_UDP_PORT", "49099"))
        app.set_setting("/app/livestream/fixedHostPort", port)
        app.set_setting("/app/livestream/minHostPort", port)
        app.set_setting("/app/livestream/maxHostPort", port)

        enable_extension("omni.kit.livestream.webrtc")

    SIMULATION_APP = app
    return app


def enable_airstack_extensions(extra=()):
    """Enable the required Kit extensions (idempotent)."""
    import omni.kit.app

    ext_manager = omni.kit.app.get_app().get_extension_manager()
    for ext in list(REQUIRED_EXTENSIONS) + list(extra):
        if not ext_manager.is_extension_enabled(ext):
            # Immediate enable is more robust across Kit versions; fall back
            # to deferred enable where unavailable.
            try:
                ext_manager.set_extension_enabled_immediate(ext, True)
            except Exception:
                ext_manager.set_extension_enabled(ext, True)


def wait_for_stage(stage, timeout_s: float = 10.0):
    """Pump the Kit app loop until /World has content (scene fully loaded)."""
    import omni.kit.app

    for _ in range(int(timeout_s / 0.1)):
        omni.kit.app.get_app().update()
        world_prim = stage.GetPrimAtPath("/World")
        if world_prim.IsValid():
            non_physics = [c for c in world_prim.GetChildren() if c.GetName() != "PhysicsScene"]
            if non_physics:
                return True
        time.sleep(0.1)
    return False


def resolve_spawn_center_from_env(default=(0.0, 0.0)):
    """Parse ISAAC_SIM_SPAWN_XY ("x,y" in meters) — where the spawn row is
    centered. Useful for scenes whose origin is cluttered or unlit."""
    raw = os.environ.get("ISAAC_SIM_SPAWN_XY", "").strip()
    if not raw:
        return default
    try:
        x, y = (float(v) for v in raw.split(","))
        return (x, y)
    except ValueError:
        carb.log_warn(f"ISAAC_SIM_SPAWN_XY='{raw}' is not 'x,y' — using {default}.")
        return default


def row_spawn_configs(num_robots, spacing_m=2.0, z_m=0.07, center_xy=(0.0, 0.0)):
    """Drone configs in a row along X, centered on ``center_xy``."""
    cx, cy = center_xy
    configs = []
    for i in range(1, num_robots + 1):
        init_x = cx + spacing_m * (i - 1) - spacing_m * (num_robots - 1) / 2.0
        configs.append({"domain_id": i, "x_m": init_x, "y_m": cy, "z_m": z_m})
    return configs


def resolve_scene_from_env(simulation_environments,
                           default_key="Default Environment"):
    """Resolve the ISAAC_SIM_SCENE / ISAAC_SIM_STAGE_SCALE env vars set by
    `airstack up --scene <shortname>` (simulation/scenes.yaml).

    Returns ``(env_url, stage_scale)``. ISAAC_SIM_SCENE may be a Pegasus
    ``SIMULATION_ENVIRONMENTS`` key, or a USD reference (``omniverse://`` /
    ``https://`` URL or ``*.usd*`` path) used verbatim. Unset → ``default_key``.
    """
    scene = os.environ.get("ISAAC_SIM_SCENE", "").strip()
    scale = float(os.environ.get("ISAAC_SIM_STAGE_SCALE") or 1.0)
    if not scene:
        return simulation_environments[default_key], scale
    if scene in simulation_environments:
        return simulation_environments[scene], scale
    if "://" in scene or scene.endswith((".usd", ".usda", ".usdc", ".usdz")):
        return scene, scale
    raise ValueError(
        f"ISAAC_SIM_SCENE '{scene}' is neither a SIMULATION_ENVIRONMENTS key "
        f"nor a USD reference (keys: {', '.join(sorted(simulation_environments))})"
    )


FOLLOW_CAM_PATH = "/World/follow_cam"


def resolve_follow_cam_from_env():
    """Parse ISAAC_SIM_FOLLOW_CAM / ISAAC_SIM_FOLLOW_CAM_OFFSET.

    Returns ``(domain_id, offset_xyz)``; ``domain_id`` is ``None`` when the
    follow-cam is disabled.
    """
    raw = os.environ.get("ISAAC_SIM_FOLLOW_CAM", "").strip().lower()
    if raw in ("off", "false", "none", "0"):
        return None, None
    try:
        target = int(raw) if raw else 1
    except ValueError:
        carb.log_warn(
            f"ISAAC_SIM_FOLLOW_CAM='{raw}' is not a domain id — following drone 1."
        )
        target = 1
    offset = (-5.0, -5.0, 2.5)
    off_raw = os.environ.get("ISAAC_SIM_FOLLOW_CAM_OFFSET", "").strip()
    if off_raw:
        try:
            x, y, z = (float(v) for v in off_raw.split(","))
            offset = (x, y, z)
        except ValueError:
            carb.log_warn(
                f"ISAAC_SIM_FOLLOW_CAM_OFFSET='{off_raw}' is not 'x,y,z' — "
                f"using default {offset}."
            )
    return target, offset


class PegasusApp:
    """Base Pegasus launch app: world + environment + scene prep + drones.

    Scenario scripts either instantiate this directly with config kwargs or
    subclass it and override the hooks:

    - ``pre_scene_prep(stage)``  — after the environment loads, before
      scale/colliders (e.g. referencing extra root prims).
    - ``post_scene_prep(stage)`` — after scene prep, before drones spawn
      (e.g. authoring an overhead map camera).
    - ``post_spawn(stage)``      — after all drones spawn (e.g. authoring
      extra scene-level prims such as a mocap interface).

    Each entry in ``drone_configs`` is a dict:
      ``domain_id`` (required) — ROS 2 domain; also the default vehicle_id
        (MAVLink port = 14540 + vehicle_id).
      ``x_m``/``y_m``/``z_m`` — spawn position in meters (default 0/0/0.07).
      ``orient`` — quaternion [x, y, z, w] (default identity).
      ``prim`` — drone root prim (default ``/World/drone{i}/base_link``).
      ``node_name`` — Pegasus OmniGraph node name (default ``PX4Multirotor_{i}``).
      ``camera`` — per-drone camera override (default: app-level ``enable_camera``).
      ``lidar`` — per-drone lidar override (default: app-level ``enable_lidar``).
      ``lidar_min_range`` — per-drone min range (default: app-level value).
    """

    def __init__(
        self,
        *,
        env_url,
        drone_configs=(),
        drone_usd=DEFAULT_DRONE_USD,
        stage_scale=1.0,
        save_scene_to=None,
        enable_camera=True,
        camera_offset=(0.2, 0.0, -0.05),
        camera_rotation_offset=(0.0, 0.0, 0.0),
        enable_lidar=True,
        lidar_config="ouster_os1",
        lidar_topic_name="point_cloud_raw",
        lidar_offset=(0.0, 0.0, 0.025),
        lidar_rotation_offset=(0.0, 0.0, 0.0),
        lidar_min_range=0.75,
        dome_light=True,
        world_gps_origin=None,
        scale_spawn_positions=False,
        extra_extensions=(),
    ):
        import omni.kit.app
        import omni.timeline
        import omni.usd
        from omni.isaac.core.world import World
        from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

        self.env_url = env_url
        self.drone_configs = list(drone_configs)
        self.drone_usd = drone_usd
        self.stage_scale = stage_scale
        self.save_scene_to = save_scene_to
        self.enable_camera = enable_camera
        self.camera_offset = list(camera_offset)
        self.camera_rotation_offset = list(camera_rotation_offset)
        self.enable_lidar = enable_lidar
        self.lidar_config = lidar_config
        self.lidar_topic_name = lidar_topic_name
        self.lidar_offset = list(lidar_offset)
        self.lidar_rotation_offset = list(lidar_rotation_offset)
        self.lidar_min_range = lidar_min_range
        self.dome_light = dome_light
        self.stop_sim = False

        # GPS origins must be written before the PX4 SITL subprocesses start
        # (robot containers read them during their own bring-up).
        #
        # Multi-drone default (audit H4): without per-drone PX4_HOME_* values,
        # every PX4 SITL boots with the same GPS home, so the GCS map renders
        # the whole fleet stacked at one coordinate. When the caller didn't
        # pick a world origin, multi-drone spawns are anchored at
        # gps_utils.DEFAULT_WORLD_ORIGIN (Lisbon — the same anchor the Pegasus
        # configs.yaml default uses, so the map doesn't move cities).
        # Single-drone spawns (len == 1) are deliberately left untouched: their
        # GPS home comes from the PX4/Pegasus defaults and that behavior is
        # machine-validated — only opt in via an explicit world_gps_origin.
        if world_gps_origin is None and len(self.drone_configs) > 1:
            from gps_utils import DEFAULT_WORLD_ORIGIN

            world_gps_origin = DEFAULT_WORLD_ORIGIN
        if world_gps_origin is not None:
            from gps_utils import set_gps_origins

            set_gps_origins(self.drone_configs, world_origin=world_gps_origin)

        enable_airstack_extensions(extra_extensions)

        self.timeline = omni.timeline.get_timeline_interface()

        # Start Pegasus interface + world
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Keep the timeline stopped throughout setup so that OmniGraph's
        # OnPlaybackTick never fires.
        self.timeline.stop()

        self.pg.load_environment(self.env_url)

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage failed to load")

        if not wait_for_stage(stage):
            carb.log_warn("Stage load timed out — continuing anyway.")

        self.pre_scene_prep(stage)

        # ----- Scene preparation -----
        from scene_prep import scale_stage_prim, add_colliders, add_dome_light

        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            # Scale /World/stage if the asset uses non-metric units (e.g. cm).
            scale_stage_prim(stage, "/World/stage", self.stage_scale)

            # Apply CollisionAPI to every mesh so physics works correctly
            add_colliders(stage_prim)

            # Let the app process the transform and collision changes
            for _ in range(10):
                omni.kit.app.get_app().update()
        else:
            carb.log_warn("/World/stage not found — skipping scale and collision.")

        # ISAAC_SIM_LIGHT_BOOST=<factor>: multiply the scene's own lights
        # (e.g. office/hospital ceiling lights, which are authored very dim).
        boost_raw = os.environ.get("ISAAC_SIM_LIGHT_BOOST", "").strip()
        if boost_raw:
            from scene_prep import boost_scene_lights

            try:
                n = boost_scene_lights(stage, float(boost_raw))
                carb.log_warn(f"[light_boost] boosted {n} scene lights x{boost_raw}")
            except ValueError:
                carb.log_warn(f"ISAAC_SIM_LIGHT_BOOST='{boost_raw}' is not a number — ignored.")

        # Dome light for uniform illumination: True → defaults, dict → kwargs.
        # ISAAC_SIM_DOME_LIGHT="intensity[,exposure]" overrides either source —
        # useful for dim indoor scenes (office/hospital corners).
        if self.dome_light:
            kwargs = self.dome_light if isinstance(self.dome_light, dict) else {}
            raw = os.environ.get("ISAAC_SIM_DOME_LIGHT", "").strip()
            if raw:
                try:
                    parts = [float(v) for v in raw.split(",")]
                    kwargs["intensity"] = parts[0]
                    if len(parts) > 1:
                        kwargs["exposure"] = parts[1]
                except ValueError:
                    carb.log_warn(
                        f"ISAAC_SIM_DOME_LIGHT='{raw}' is not 'intensity[,exposure]' — ignored."
                    )
            add_dome_light(stage, **kwargs)

        self._maybe_export_scene()

        self.post_scene_prep(stage)

        # Positions are authored in meters; scenes with non-metric stage units
        # opt into converting spawn positions into stage units.
        self._position_scale = 1.0
        if scale_spawn_positions:
            from scene_prep import get_stage_meters_per_unit

            _, self._position_scale = get_stage_meters_per_unit(stage)

        # ----- Spawn drone OmniGraphs -----
        # This only creates the graph topology. The actual drones + PX4
        # backends are created by compute_base on the first Play tick.
        for cfg in self.drone_configs:
            self.spawn_drone(cfg)

        self.post_spawn(stage)

        self._setup_follow_cam(stage)

        self.play_on_start = os.environ.get("PLAY_SIM_ON_START", "true").lower() == "true"

    # --- Hooks (default no-ops) ---

    def pre_scene_prep(self, stage):
        pass

    def post_scene_prep(self, stage):
        pass

    def post_spawn(self, stage):
        pass

    # --- Building blocks ---

    def _maybe_export_scene(self):
        """Optionally save the prepared scene as a self-contained USD package.

        The Collector copies all Nucleus-hosted textures and MDLs locally.
        """
        if not self.save_scene_to:
            return
        import tempfile

        import omni.usd
        from scene_prep import save_scene_as_contained_usd

        tmp_usd = os.path.join(tempfile.gettempdir(), "prepared_scene.usd")
        success, error = asyncio.get_event_loop().run_until_complete(
            omni.usd.get_context().export_as_stage_async(tmp_usd)
        )
        if success:
            os.makedirs(self.save_scene_to, exist_ok=True)
            save_scene_as_contained_usd(tmp_usd, self.save_scene_to)
            os.remove(tmp_usd)
        else:
            carb.log_error(f"Scene export failed: {error}")

    def spawn_drone(self, cfg):
        """Spawn one drone (PX4 multirotor node + camera + optional lidar)."""
        from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node
        from pegasus.simulator.ogn.api.spawn_zed_camera import add_zed_stereo_camera_subgraph
        from pegasus.simulator.ogn.api.spawn_rtx_lidar import add_rtx_lidar_subgraph

        i = cfg["domain_id"]
        vehicle_id = cfg.get("vehicle_id", i)
        robot_name = cfg.get("robot_name", f"robot_{i}")
        drone_prim = cfg.get("prim", f"/World/drone{i}/base_link")
        node_name = cfg.get("node_name", f"PX4Multirotor_{i}")
        s = self._position_scale
        init_pos = [cfg.get("x_m", 0.0) * s, cfg.get("y_m", 0.0) * s, cfg.get("z_m", 0.07) * s]
        init_orient = cfg.get("orient", [0.0, 0.0, 0.0, 1.0])

        graph_handle = spawn_px4_multirotor_node(
            pegasus_node_name=node_name,
            drone_prim=drone_prim,
            robot_name=robot_name,
            vehicle_id=vehicle_id,  # MAVLink port = 14540 + vehicle_id
            domain_id=i,            # ROS 2 domain ID — match vehicle_id by convention
            usd_file=self.drone_usd,
            init_pos=init_pos,
            init_orient=init_orient,
        )

        if cfg.get("camera", self.enable_camera):
            add_zed_stereo_camera_subgraph(
                parent_graph_handle=graph_handle,
                drone_prim=drone_prim,
                robot_name=robot_name,
                camera_name="ZEDCamera",
                camera_offset=list(cfg.get("camera_offset", self.camera_offset)),
                camera_rotation_offset=list(self.camera_rotation_offset),
            )

        if cfg.get("lidar", self.enable_lidar):
            add_rtx_lidar_subgraph(
                parent_graph_handle=graph_handle,
                drone_prim=drone_prim,
                robot_name=robot_name,
                lidar_config=self.lidar_config,
                lidar_topic_name=self.lidar_topic_name,
                lidar_offset=self.lidar_offset,
                lidar_rotation_offset=self.lidar_rotation_offset,
                min_range=cfg.get("lidar_min_range", self.lidar_min_range),
            )

        return graph_handle

    # --- Follow camera -------------------------------------------------

    def _setup_follow_cam(self, stage):
        """Author the follow-camera and switch the viewport to it.

        The stock perspective camera often starts inside geometry on
        cm-authored stages (black viewport), so a chase camera that tracks a
        drone is both the fix and a nicer default view. The tracked drone
        prim only materializes on the first Play tick, so the per-frame
        update tolerates a missing target until then.
        """
        self._follow_target_path = None
        self._follow_cam_pos = None
        self._follow_look = None
        self._follow_vehicle_logged = False

        target, offset = resolve_follow_cam_from_env()
        if target is None or not self.drone_configs:
            return

        cfg = next(
            (c for c in self.drone_configs if c["domain_id"] == target),
            self.drone_configs[0],
        )
        if cfg["domain_id"] != target:
            carb.log_warn(
                f"ISAAC_SIM_FOLLOW_CAM={target} has no matching drone — "
                f"following drone {cfg['domain_id']}."
            )
        i = cfg["domain_id"]
        self._follow_target_path = cfg.get("prim", f"/World/drone{i}/base_link")

        from pxr import Gf, UsdGeom

        self._follow_offset = Gf.Vec3d(*offset)
        cam = UsdGeom.Camera.Define(stage, FOLLOW_CAM_PATH)
        cam.GetFocalLengthAttr().Set(16.0)
        # Generous range: cm-scaled stages have geometry both very near and
        # (pre-scale) very far from the camera.
        cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.1, 1.0e6))

        # ISAAC_SIM_FOLLOW_CAM_LIGHT=<intensity>: headlight riding the camera,
        # for scenes too dark to film (interiors the dome light can't reach).
        light_raw = os.environ.get("ISAAC_SIM_FOLLOW_CAM_LIGHT", "").strip()
        if light_raw:
            from pxr import UsdLux

            try:
                headlight = UsdLux.SphereLight.Define(
                    stage, FOLLOW_CAM_PATH + "/headlight"
                )
                headlight.GetIntensityAttr().Set(float(light_raw))
                headlight.GetRadiusAttr().Set(0.05)
                headlight.CreateNormalizeAttr().Set(True)
            except ValueError:
                carb.log_warn(
                    f"ISAAC_SIM_FOLLOW_CAM_LIGHT='{light_raw}' is not a number — ignored."
                )

        # Frame the spawn point immediately so the viewport is never black
        # while the sim is still paused.
        s = self._position_scale
        spawn = Gf.Vec3d(
            cfg.get("x_m", 0.0) * s, cfg.get("y_m", 0.0) * s, cfg.get("z_m", 0.07) * s
        )
        self._follow_cam_pos = spawn + self._follow_offset
        self._follow_look = spawn
        self._author_follow_cam()

        try:
            from omni.kit.viewport.utility import get_active_viewport

            viewport = get_active_viewport()
            if viewport is not None:
                viewport.camera_path = FOLLOW_CAM_PATH
        except Exception as exc:  # headless variants may have no viewport
            carb.log_warn(f"Could not switch viewport to follow cam: {exc}")

    def _author_follow_cam(self):
        from isaacsim.core.utils.viewports import set_camera_view

        set_camera_view(
            eye=self._follow_cam_pos,
            target=self._follow_look,
            camera_prim_path=FOLLOW_CAM_PATH,
        )

    def _follow_target_position(self):
        """Live world position of the tracked drone, or ``None`` pre-spawn.

        PhysX (fabric) does not write simulated poses back to USD, so the
        authoritative source is the Pegasus vehicle state; the USD transform
        only covers the pre-play window before the vehicle registers.
        """
        from pxr import Gf

        # OGN-spawned vehicles register under the full drone prim path
        # (e.g. "/World/drone2/base_link"); UI-spawned ones may use the parent.
        target_path = self._follow_target_path.rstrip("/")
        parent = target_path.rsplit("/", 1)[0]
        try:
            from pegasus.simulator.logic.vehicle_manager import VehicleManager

            for stage_prefix, vehicle in VehicleManager.get_vehicle_manager().vehicles.items():
                sp = stage_prefix.rstrip("/")
                if sp in (target_path, parent) or sp.startswith(parent + "/"):
                    if not self._follow_vehicle_logged:
                        self._follow_vehicle_logged = True
                        carb.log_warn(f"[follow_cam] tracking Pegasus vehicle '{sp}'")
                    p = vehicle.state.position
                    return Gf.Vec3d(float(p[0]), float(p[1]), float(p[2]))
        except Exception:
            pass

        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return None
        prim = stage.GetPrimAtPath(self._follow_target_path)
        if not prim.IsValid():
            return None  # drone spawns on the first Play tick
        try:
            return omni.usd.get_world_transform_matrix(prim).ExtractTranslation()
        except Exception:
            return None

    def _update_follow_cam(self):
        if not self._follow_target_path:
            return
        from pxr import Gf

        target = self._follow_target_position()
        if target is None:
            return
        desired = target + self._follow_offset
        # Exponential smoothing keeps the chase fluid through sharp maneuvers.
        alpha = 0.08
        self._follow_cam_pos += (desired - self._follow_cam_pos) * alpha
        self._follow_look += (Gf.Vec3d(target) - self._follow_look) * alpha
        self._author_follow_cam()

    def run(self):
        """Play (unless PLAY_SIM_ON_START=false) and step until closed."""
        import omni.kit.app
        from omni.isaac.core.world import World

        if self.play_on_start:
            self.timeline.play()
        else:
            self.timeline.stop()

        app = omni.kit.app.get_app()
        while SIMULATION_APP.is_running() and not self.stop_sim:
            self._update_follow_cam()
            # File → Save re-opens the stage, which invalidates the World.
            # Fall back to app.update() until the extension re-creates it.
            world = World.instance()
            if world is not None and hasattr(world, "_scene"):
                world.step(render=True)
                if world is not self.world:
                    self.world = world
                    self.pg._world = world
            else:
                app.update()

        carb.log_warn("Closing simulation.")
        self.timeline.stop()
        SIMULATION_APP.close()
