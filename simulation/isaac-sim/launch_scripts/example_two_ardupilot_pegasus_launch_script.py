#!/usr/bin/env python
"""
Example two-drone ArduPilot launcher with scene preparation.

Analog to example_two_px4_pegasus_launch_script.py.
"""

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import asyncio
import os
import sys
import time

import omni.kit.app
import omni.timeline
import omni.usd

from omni.isaac.core.world import World

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.ogn.api.spawn_multirotor import spawn_ardupilot_multirotor_node
from pegasus.simulator.ogn.api.spawn_zed_camera import add_zed_stereo_camera_subgraph
from pegasus.simulator.ogn.api.spawn_ouster_lidar import add_ouster_lidar_subgraph

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "utils")))
from scene_prep import scale_stage_prim, add_colliders, add_dome_light, save_scene_as_contained_usd


ENV_URL = SIMULATION_ENVIRONMENTS["Curved Gridroom"]
STAGE_SCALE = 1.0
SAVE_SCENE_TO = None
DRONE_USD = "~/.local/share/ov/data/documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd"
ARDUPILOT_DIR = os.path.expanduser(os.environ.get("ARDUPILOT_DIR", "/isaac-sim/ardupilot"))

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
        self.timeline = omni.timeline.get_timeline_interface()
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

        graph_handle1 = spawn_ardupilot_multirotor_node(
            pegasus_node_name="ArduPilotMultirotor_1",
            drone_prim="/World/drone1/base_link",
            robot_name="robot_1",
            vehicle_id=1,
            domain_id=1,
            usd_file=DRONE_USD,
            ardupilot_dir=ARDUPILOT_DIR,
            init_pos=[2.0, 0.0, 0.07],
            init_orient=[0.0, 0.0, 0.0, 1.0],
        )

        add_zed_stereo_camera_subgraph(
            parent_graph_handle=graph_handle1,
            drone_prim="/World/drone1/base_link",
            robot_name="robot_1",
            camera_name="ZEDCamera",
            camera_offset=[0.2, 0.0, -0.05],
            camera_rotation_offset=[0.0, 0.0, 0.0],
        )

        add_ouster_lidar_subgraph(
            parent_graph_handle=graph_handle1,
            drone_prim="/World/drone1/base_link",
            robot_name="robot_1",
            lidar_name="OS1_REV6_128_10hz___512_resolution",
            lidar_offset=[0.0, 0.0, 0.025],
            lidar_rotation_offset=[0.0, 0.0, 0.0],
            lidar_min_range=0.75,
        )

        graph_handle2 = spawn_ardupilot_multirotor_node(
            pegasus_node_name="ArduPilotMultirotor_2",
            drone_prim="/World/drone2/base_link",
            robot_name="robot_2",
            vehicle_id=2,
            domain_id=2,
            usd_file=DRONE_USD,
            ardupilot_dir=ARDUPILOT_DIR,
            init_pos=[0.0, 0.0, 0.07],
            init_orient=[0.0, 0.0, 0.0, 1.0],
        )

        add_zed_stereo_camera_subgraph(
            parent_graph_handle=graph_handle2,
            drone_prim="/World/drone2/base_link",
            robot_name="robot_2",
            camera_name="ZEDCamera",
            camera_offset=[0.2, 0.0, -0.05],
            camera_rotation_offset=[0.0, 0.0, 0.0],
        )

        add_ouster_lidar_subgraph(
            parent_graph_handle=graph_handle2,
            drone_prim="/World/drone2/base_link",
            robot_name="robot_2",
            lidar_name="OS1_REV6_128_10hz___512_resolution",
            lidar_offset=[0.0, 0.0, 0.025],
            lidar_rotation_offset=[0.0, 0.0, 0.0],
            lidar_min_range=0.75,
        )

        self.play_on_start = os.environ.get("PLAY_SIM_ON_START", "true").lower() == "true"

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
        self.timeline.stop()
        simulation_app.close()


def main():
    PegasusApp().run()


if __name__ == "__main__":
    main()
