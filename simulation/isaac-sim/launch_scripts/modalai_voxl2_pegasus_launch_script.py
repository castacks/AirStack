#!/usr/bin/env python3
"""
ModalAI VOXL2 simulation launcher.

Spawns a single drone in Isaac Sim using the Pegasus PX4 backend.
The Iris USD is used as a visual placeholder until a VOXL2-specific
USD model is available.

On the ROS side, run modalai_interface.launch.xml (not px4_interface).
In simulation, also run the sim_qvio_bridge node so that modalai_interface
receives odometry on the /qvio topic just like real VOXL2 hardware would
publish it via voxl-mpa-to-ros2.
"""

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import os
import sys
import time

import omni.kit.app
import omni.timeline
import omni.usd

from omni.isaac.core.world import World

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node
from pegasus.simulator.ogn.api.spawn_zed_camera import add_zed_stereo_camera_subgraph
from pegasus.simulator.ogn.api.spawn_ouster_lidar import add_ouster_lidar_subgraph

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "utils")))
from scene_prep import scale_stage_prim, add_colliders, add_dome_light


# --------------------- CONFIGURATION ---------------------
ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]

STAGE_SCALE = 1.0

# Iris USD as visual placeholder for the VOXL2 drone.
# Replace with a VOXL2-specific USD when one is available.
DRONE_USD = "~/.local/share/ov/data/documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd"

ROBOT_NAME  = os.environ.get("ROBOT_NAME", "robot_1")
VEHICLE_ID  = int(os.environ.get("VEHICLE_ID", "1"))
DOMAIN_ID   = int(os.environ.get("ROS_DOMAIN_ID", str(VEHICLE_ID)))
# ---------------------------------------------------------


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

        # Spawn the VOXL2 drone (Iris as visual placeholder).
        graph_handle = spawn_px4_multirotor_node(
            pegasus_node_name="VOXL2Multirotor",
            drone_prim="/World/base_link",
            robot_name=ROBOT_NAME,
            vehicle_id=VEHICLE_ID,
            domain_id=DOMAIN_ID,
            usd_file=DRONE_USD,
            init_pos=[0.0, 0.0, 0.07],
            init_orient=[0.0, 0.0, 0.0, 1.0],
        )

        add_zed_stereo_camera_subgraph(
            parent_graph_handle=graph_handle,
            drone_prim="/World/base_link",
            robot_name=ROBOT_NAME,
            camera_name="ZEDCamera",
            camera_offset=[0.2, 0.0, -0.05],
            camera_rotation_offset=[0.0, 0.0, 0.0],
        )

        add_ouster_lidar_subgraph(
            parent_graph_handle=graph_handle,
            drone_prim="/World/base_link",
            robot_name=ROBOT_NAME,
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
    pg_app = PegasusApp()
    pg_app.run()


if __name__ == "__main__":
    main()
