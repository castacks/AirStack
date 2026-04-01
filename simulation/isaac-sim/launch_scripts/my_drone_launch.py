#!/usr/bin/env python
"""
Custom multi-drone PX4 launcher.

Loads an environment (Nucleus URL or local path) and spawns
PX4 drones with ZED camera and Ouster lidar.
"""

import carb
from isaacsim import SimulationApp

# Must be created before any omni imports
simulation_app = SimulationApp({"headless": False})

import os
import sys
import time

import omni.kit.app
import omni.timeline
import omni.usd

from omni.isaac.core.world import World

# Pegasus imports
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node
from pegasus.simulator.ogn.api.spawn_zed_camera import add_zed_stereo_camera_subgraph
from pegasus.simulator.ogn.api.spawn_ouster_lidar import add_ouster_lidar_subgraph

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "utils")))
from scene_prep import scale_stage_prim, add_colliders, add_dome_light


# --------------------- CONFIGURATION ---------------------
ENV_URL = "omniverse://airlab-nucleus.andrew.cmu.edu:443/Library/Stages/ConstructionSite/ConstructionSite.stage.usd"

# Scale applied to /World/stage. 0.01 converts cm→m for Nucleus assets.
# Set to 1.0 if the environment is already in meters.
STAGE_SCALE = 0.01

DRONE_USD = "~/.local/share/ov/data/documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd"
# ---------------------------------------------------------


# Enable required extensions
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


def wait_for_stage(stage, timeout_s: float = 30.0):
    """Pump the Kit app loop until /World has content (scene fully loaded)."""
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

        # Start Pegasus interface + world
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Keep timeline stopped during setup to prevent OmniGraph ticks
        self.timeline.stop()

        self.pg.load_environment(ENV_URL)

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage failed to load")

        if not wait_for_stage(stage):
            carb.log_warn("Stage load timed out — continuing anyway.")

        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            if STAGE_SCALE != 1.0:
                scale_stage_prim(stage, "/World/stage", STAGE_SCALE)
            add_colliders(stage_prim)
            omni.kit.app.get_app().update()

        add_dome_light(stage)

        # ----- Spawn drone 1 -----

        graph_handle1 = spawn_px4_multirotor_node(
            pegasus_node_name="PX4Multirotor_1",
            drone_prim="/World/drone1/base_link",
            robot_name="robot_1",
            vehicle_id=1,
            domain_id=1,
            usd_file=DRONE_USD,
            init_pos=[0.0, 0.0, 0.07],
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

        # add_ouster_lidar_subgraph(
        #     parent_graph_handle=graph_handle1,
        #     drone_prim="/World/drone1/base_link",
        #     robot_name="robot_1",
        #     lidar_name="OS1_REV6_128_10hz___512_resolution",
        #     lidar_offset=[0.0, 0.0, 0.025],
        #     lidar_rotation_offset=[0.0, 0.0, 0.0],
        #     lidar_min_range=0.75,
        # )

        # ----- Spawn drone 2 -----

        graph_handle2 = spawn_px4_multirotor_node(
            pegasus_node_name="PX4Multirotor_2",
            drone_prim="/World/drone2/base_link",
            robot_name="robot_2",
            vehicle_id=2,
            domain_id=2,
            usd_file=DRONE_USD,
            init_pos=[3.0, 0.0, 0.07],
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

        # add_ouster_lidar_subgraph(
        #     parent_graph_handle=graph_handle2,
        #     drone_prim="/World/drone2/base_link",
        #     robot_name="robot_2",
        #     lidar_name="OS1_REV6_128_10hz___512_resolution",
        #     lidar_offset=[0.0, 0.0, 0.025],
        #     lidar_rotation_offset=[0.0, 0.0, 0.0],
        #     lidar_min_range=0.75,
        # )

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
