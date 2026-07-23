#!/usr/bin/env python
"""
Multi-drone PX4 Pegasus launcher with OptiTrack NatNet mocap streaming.

Same scene prep and sensor stack as ``example_multi_px4_pegasus_launch_script.py``,
plus a Motive-compatible NatNet server that always streams one rigid body per drone
and a shared static ``Target`` (id 100) at ``/World/target``.

Body naming:
 - ``NUM_ROBOTS=1``: drone body ``Drone`` (id 1)
 - ``NUM_ROBOTS>1``: ``Drone1``, ``Drone2``, … (ids 1..N)

Intended multi-robot profile pairing (see ``natnet_config.yaml`` commented scaffolding):
 - ``robot_1``: tracks its drone + the shared ``Target``
 - ``robot_2``: tracks its drone + the shared ``Target``
 - ``robot_3``: tracks its drone only (no Target in profile)

Set ``NUM_ROBOTS=3`` on both sim and robot stacks; each container picks its profile
via ``ROBOT_NAME``.

Env:
 - ``NUM_ROBOTS`` (default 1)
 - ``ENABLE_LIDAR`` (default false)
 - ``PLAY_SIM_ON_START`` (default true)
 - ``NATNET_BODY_NAME`` / ``NATNET_TARGET_NAME`` (optional name overrides)
"""

import asyncio
import os
import sys
import time

import carb
from isaacsim import SimulationApp

_headless = os.environ.get("ISAAC_SIM_HEADLESS", "false").lower() == "true"
simulation_app = SimulationApp({"headless": _headless})

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

NUM_ROBOTS = int(os.environ.get("NUM_ROBOTS", "1"))
ENABLE_LIDAR = os.environ.get("ENABLE_LIDAR", "false").lower() == "true"
NATNET_BODY_NAME = os.environ.get("NATNET_BODY_NAME", "Drone")
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


def _drone_body_name(index: int) -> str:
    """Single agent uses bare ``Drone``; multi uses ``Drone1``, ``Drone2``, …"""
    return NATNET_BODY_NAME if NUM_ROBOTS == 1 else f"{NATNET_BODY_NAME}{index}"


def spawn_drone(index: int):
    robot_name = f"robot_{index}"
    drone_prim = f"/World/drone{index}/base_link"
    init_x = 2.0 * (index - 1) - 2.0 * (NUM_ROBOTS - 1) / 2.0

    graph_handle = spawn_px4_multirotor_node(
        pegasus_node_name=f"PX4Multirotor_{index}",
        drone_prim=drone_prim,
        robot_name=robot_name,
        vehicle_id=index,
        domain_id=index,
        usd_file=DRONE_USD,
        init_pos=[init_x, 0.0, 0.07],
        init_orient=[0.0, 0.0, 0.0, 1.0],
    )

    add_zed_stereo_camera_subgraph(
        parent_graph_handle=graph_handle,
        drone_prim=drone_prim,
        robot_name=robot_name,
        camera_name="ZEDCamera",
        camera_offset=[0.2, 0.0, -0.05],
        camera_rotation_offset=[0.0, 0.0, 0.0],
    )

    if ENABLE_LIDAR:
        add_rtx_lidar_subgraph(
            parent_graph_handle=graph_handle,
            drone_prim=drone_prim,
            robot_name=robot_name,
            lidar_config="ouster_os1",
            lidar_topic_name="point_cloud_raw",
            lidar_offset=[0.0, 0.0, 0.025],
            lidar_rotation_offset=[0.0, 0.0, 0.0],
            min_range=0.75,
        )


class PegasusApp:

    def __init__(self):
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

        print(f"[example_multi_natnet] Spawning {NUM_ROBOTS} drone(s), lidar={'on' if ENABLE_LIDAR else 'off'}")
        for i in range(1, NUM_ROBOTS + 1):
            spawn_drone(i)

        self._setup_natnet(stage)
        self.play_on_start = os.environ.get("PLAY_SIM_ON_START", "true").lower() == "true"

    def _setup_natnet(self, stage):
        """Author NatNet bodies: one per drone plus one shared static target."""
        try:
            author_static_target(stage, DEFAULT_TARGET_PATH, DEFAULT_TARGET_POSITION)
            bodies = [
                (_drone_body_name(i), i, f"/World/drone{i}/base_link/body")
                for i in range(1, NUM_ROBOTS + 1)
            ]
            bodies.append((NATNET_TARGET_NAME, DEFAULT_TARGET_STREAMING_ID, DEFAULT_TARGET_PATH))

            self.natnet_manager = start_drone_natnet_server(
                stage, bodies, **_NATNET_SERVER_KWARGS
            )
            carb.log_warn(
                f"[natnet] Emulator started with {NUM_ROBOTS} drone body(ies) "
                f"and shared target '{NATNET_TARGET_NAME}' (robot_1/robot_2 subscribe via "
                f"natnet_config; robot_3 omits Target)."
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
