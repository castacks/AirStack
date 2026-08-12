#!/usr/bin/env python
"""
Single-drone PX4 Pegasus launcher with OptiTrack NatNet mocap streaming.

Same scene prep and sensor stack as ``example_one_px4_pegasus_launch_script.py``, plus
a Motive-compatible NatNet server that always streams:

 - ``Drone`` (id 1) from the Pegasus ``body`` prim under ``/World/base_link``
 - ``Target`` (id 100) from a static ``/World/target`` prim

Pair with robot-side ``LAUNCH_NATNET=true`` and a matching ``natnet_config.yaml``
profile. To consume the target on the robot, add a Target body to the profile
(see the commented scaffolding in ``natnet_config.yaml``).

Override rigid-body names with ``NATNET_BODY_NAME`` / ``NATNET_TARGET_NAME``.
"""

import os
import sys
import time
import asyncio

import carb
from isaacsim import SimulationApp

_LIVESTREAM = os.environ.get("ISAAC_SIM_LIVESTREAM", "").lower() == "true"

if _LIVESTREAM:
    _SIM_APP_CONFIG = {
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
    _SIM_APP_CONFIG = {"headless": False}

simulation_app = SimulationApp(launch_config=_SIM_APP_CONFIG)

if _LIVESTREAM:
    from isaacsim.core.utils.extensions import enable_extension
    simulation_app.set_setting("/app/window/drawMouse", True)
    simulation_app.set_setting("/app/livestream/enabled", True)
    LIVESTREAM_UDP_PORT = int(os.environ.get("ISAAC_SIM_LIVESTREAM_UDP_PORT", "49099"))
    simulation_app.set_setting("/app/livestream/fixedHostPort", LIVESTREAM_UDP_PORT)
    simulation_app.set_setting("/app/livestream/minHostPort", LIVESTREAM_UDP_PORT)
    simulation_app.set_setting("/app/livestream/maxHostPort", LIVESTREAM_UDP_PORT)
    enable_extension("omni.kit.livestream.webrtc")

import omni.kit.app
import omni.timeline
import omni.usd

from omni.isaac.core.world import World

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node
from pegasus.simulator.ogn.api.spawn_zed_camera import add_zed_stereo_camera_subgraph
from pegasus.simulator.ogn.api.spawn_rtx_lidar import add_rtx_lidar_subgraph

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "utils")))
from scene_prep import scale_stage_prim, add_colliders, add_dome_light, save_scene_as_contained_usd

# gps_utils lives in this launch_scripts directory.
_LAUNCH_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _LAUNCH_SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_SCRIPTS_DIR)
from gps_utils import set_gps_origins, DEFAULT_WORLD_ORIGIN

from optitrack.natnet.emulator.isaac import (
    DEFAULT_TARGET_PATH,
    DEFAULT_TARGET_POSITION,
    DEFAULT_TARGET_STREAMING_ID,
    author_static_target,
    start_drone_natnet_server,
)

# --------------------- CONFIGURATION ---------------------
ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]
STAGE_SCALE = 1.0
SAVE_SCENE_TO = None
DRONE_USD = "~/.local/share/ov/data/documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd"

# GPS world anchor: what world (0, 0, 0) maps to in real GPS coordinates. Must
# match the GCS origin (gcs_visualizer/gcs_utils.py) and the robot's
# natnet_ros2 mavros_gp_origin.yaml. In vision/mocap mode the robot-side
# mavros_gp_origin node is the authoritative datum; this keeps PX4's SITL home
# consistent with it so the two never disagree.
WORLD_GPS_ORIGIN = DEFAULT_WORLD_ORIGIN

# Single drone spawned at the world origin. domain_id / spawn must match the
# spawn_px4_multirotor_node call below.
DRONE_CONFIGS = [
    {"domain_id": 1, "x_m": 0.0, "y_m": 0.0, "z_m": 0.07},
]

NATNET_BODY_NAME = os.environ.get("NATNET_BODY_NAME", "Drone")
# Streaming id the emulator advertises for the drone. The NatNet client filters incoming
# frames by NUMERIC id, so this must match the body id in natnet_config.yaml, which reads
# the same env var — a mismatch yields a connected client that never publishes. Both
# default to 1, so the sim path works unconfigured.
NATNET_BODY_ID = int(os.environ.get("NATNET_BODY_ID", "1"))
NATNET_TARGET_NAME = os.environ.get("NATNET_TARGET_NAME", "Target")

_NATNET_SERVER_KWARGS = {
    "pose_noise_enabled": True,
    "pose_noise_std_meters": 0.0005,
    "pose_noise_rotation_deg": 0.05,
}
# ---------------------------------------------------------


ext_manager = omni.kit.app.get_app().get_extension_manager()
for ext in [
    "omni.graph.core",
    "omni.graph.action",
    "omni.graph.action_nodes",
    "isaacsim.core.nodes",
    "omni.graph.ui",
    "omni.graph.visualization.nodes",
    "omni.graph.scriptnode",
    "omni.graph.window.action",
    "omni.graph.window.generic",
    "omni.graph.ui_nodes",
    "pegasus.simulator",
]:
    if not ext_manager.is_extension_enabled(ext):
        ext_manager.set_extension_enabled_immediate(ext, True)


def wait_for_stage(stage, timeout_s: float = 10.0):
    for _ in range(int(timeout_s / 0.1)):
        omni.kit.app.get_app().update()
        world_prim = stage.GetPrimAtPath("/World")
        if world_prim.IsValid():
            non_physics = [c for c in world_prim.GetChildren() if c.GetName() != "PhysicsScene"]
            if non_physics:
                return True
        time.sleep(0.1)
    return False


class PegasusApp:

    def __init__(self):
        # Write GPS home before spawning so the drone's global position shares
        # the GCS datum. Must run before the PX4 SITL subprocess starts.
        set_gps_origins(DRONE_CONFIGS, world_origin=WORLD_GPS_ORIGIN)

        self.timeline = omni.timeline.get_timeline_interface()
        self.natnet_manager = None

        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world
        self.timeline.stop()

        self.pg.load_environment(ENV_URL)

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage failed to load")

        if not wait_for_stage(stage):
            carb.log_warn("Stage load timed out — continuing anyway.")

        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            scale_stage_prim(stage, "/World/stage", STAGE_SCALE)
            add_colliders(stage_prim)
            for _ in range(10):
                omni.kit.app.get_app().update()
        else:
            carb.log_warn("/World/stage not found — skipping scale and collision.")

        add_dome_light(stage)

        if SAVE_SCENE_TO:
            import tempfile
            tmp_usd = os.path.join(tempfile.gettempdir(), "prepared_scene.usd")
            success, error = asyncio.get_event_loop().run_until_complete(
                omni.usd.get_context().export_as_stage_async(tmp_usd)
            )
            if success:
                os.makedirs(SAVE_SCENE_TO, exist_ok=True)
                save_scene_as_contained_usd(tmp_usd, SAVE_SCENE_TO)
                os.remove(tmp_usd)
            else:
                carb.log_error(f"Scene export failed: {error}")

        graph_handle = spawn_px4_multirotor_node(
            pegasus_node_name="PX4Multirotor",
            drone_prim="/World/base_link",
            robot_name="robot_1",
            vehicle_id=1,
            domain_id=1,
            usd_file=DRONE_USD,
            init_pos=[0.0, 0.0, 0.07],
            init_orient=[0.0, 0.0, 0.0, 1.0],
        )

        add_zed_stereo_camera_subgraph(
            parent_graph_handle=graph_handle,
            drone_prim="/World/base_link",
            robot_name="robot_1",
            camera_name="ZEDCamera",
            camera_offset=[0.2, 0.0, -0.05],
            camera_rotation_offset=[0.0, 0.0, 0.0],
        )

        add_rtx_lidar_subgraph(
            parent_graph_handle=graph_handle,
            drone_prim="/World/base_link",
            robot_name="robot_1",
            lidar_config="ouster_os1",
            lidar_topic_name="point_cloud_raw",
            lidar_offset=[0.0, 0.0, 0.025],
            lidar_rotation_offset=[0.0, 0.0, 0.0],
            min_range=0.75,
        )

        self._setup_natnet(stage)
        self.play_on_start = os.environ.get("PLAY_SIM_ON_START", "true").lower() == "true"

    def _setup_natnet(self, stage):
        """Author the NatNet interface prim (drone + static target) and start the server."""
        try:
            author_static_target(stage, DEFAULT_TARGET_PATH, DEFAULT_TARGET_POSITION)
            bodies = [
                (NATNET_BODY_NAME, NATNET_BODY_ID, "/World/base_link/body"),
                (NATNET_TARGET_NAME, DEFAULT_TARGET_STREAMING_ID, DEFAULT_TARGET_PATH),
            ]
            self.natnet_manager = start_drone_natnet_server(
                stage, bodies, **_NATNET_SERVER_KWARGS
            )
            carb.log_warn(
                f"[natnet] Emulator started: '{NATNET_BODY_NAME}' (-> /World/base_link/body), "
                f"'{NATNET_TARGET_NAME}' (-> {DEFAULT_TARGET_PATH})."
            )
        except Exception as exc:  # noqa: BLE001 - never let NatNet kill the sim
            carb.log_error(f"[natnet] Failed to start emulator: {exc}")
            self.natnet_manager = None

    def run(self):
        if self.play_on_start:
            self.timeline.play()
        else:
            self.timeline.stop()

        app = omni.kit.app.get_app()
        while simulation_app.is_running():
            world = World.instance()
            if world is not None and hasattr(world, '_scene'):
                world.step(render=True)
                if world is not self.world:
                    self.world = world
                    self.pg._world = world
            else:
                app.update()

        carb.log_warn("Closing simulation.")
        if self.natnet_manager is not None:
            self.natnet_manager.on_shutdown()
        self.timeline.stop()
        simulation_app.close()


def main():
    PegasusApp().run()


if __name__ == "__main__":
    main()
