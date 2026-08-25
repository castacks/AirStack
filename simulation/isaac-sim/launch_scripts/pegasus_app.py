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


def row_spawn_configs(num_robots, spacing_m=2.0, z_m=0.07):
    """Drone configs in a row along X, centered near the origin: -2, 0, 2, …"""
    configs = []
    for i in range(1, num_robots + 1):
        init_x = spacing_m * (i - 1) - spacing_m * (num_robots - 1) / 2.0
        configs.append({"domain_id": i, "x_m": init_x, "y_m": 0.0, "z_m": z_m})
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

        # Dome light for uniform illumination: True → defaults, dict → kwargs.
        if self.dome_light:
            kwargs = self.dome_light if isinstance(self.dome_light, dict) else {}
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
