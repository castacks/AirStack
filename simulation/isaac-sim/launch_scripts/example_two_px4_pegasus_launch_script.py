#!/usr/bin/env python
"""
Example two-drone PX4 launcher with scene preparation.

Demonstrates:
 - Loading a Pegasus world with an environment
 - Scaling the environment prim and adding collision geometry
 - Adding a dome light
 - Spawning two PX4 multirotors each with a ZED camera and Ouster lidar
 - Optionally saving the prepared scene as a self-contained USD
"""

import carb
from isaacsim import SimulationApp

# Must be created before any omni imports
simulation_app = SimulationApp({"headless": False})

import asyncio
import os
import sys
import time

import omni.kit.app
import omni.timeline
import omni.usd

from omni.isaac.core.world import World

# Pegasus imports
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node
from pegasus.simulator.ogn.api.spawn_zed_camera import add_zed_stereo_camera_subgraph
from pegasus.simulator.ogn.api.spawn_ouster_lidar import add_ouster_lidar_subgraph

# Load scene-prep utilities via explicit file path (robust to Isaac Sim's script runner)
import importlib.util as _ilu
_scene_prep_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "utils", "scene_prep.py")
_spec = _ilu.spec_from_file_location("scene_prep", os.path.normpath(_scene_prep_path))
_scene_prep = _ilu.module_from_spec(_spec)
_spec.loader.exec_module(_scene_prep)
scale_stage_prim            = _scene_prep.scale_stage_prim
add_colliders               = _scene_prep.add_colliders
add_dome_light              = _scene_prep.add_dome_light
save_scene_as_contained_usd = _scene_prep.save_scene_as_contained_usd


# --------------------- CONFIGURATION ---------------------
# Environment to load. Swap this URL/key for any other scene.
ENV_URL = SIMULATION_ENVIRONMENTS["Curved Gridroom"]

# Scale applied to /World/stage. 0.01 converts cm→m for Nucleus assets.
# Set to 1.0 if the environment is already in meters.
STAGE_SCALE = 1.0

# Set to a directory path to export a self-contained USD after scene prep.
# Set to None to skip saving.
SAVE_SCENE_TO = None  # e.g. os.path.expanduser("~/AirStack/my_scene/")

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


def wait_for_stage(stage, timeout_s: float = 10.0):
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

        # Load environment
        self.pg.load_environment(ENV_URL)

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage failed to load")

        # Wait for the environment to finish loading before modifying it
        if not wait_for_stage(stage):
            carb.log_warn("Stage load timed out — continuing anyway.")

        # ----- Scene preparation -----

        # Scale /World/stage if the asset uses non-metric units (e.g. cm).
        # Remove or set STAGE_SCALE=1.0 if the environment is already in meters.
        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            scale_stage_prim(stage, "/World/stage", STAGE_SCALE)

            # Apply CollisionAPI to every mesh so physics works correctly
            add_colliders(stage_prim)

            # Let the app process the transform and collision changes
            for _ in range(10):
                omni.kit.app.get_app().update()
        else:
            carb.log_warn("/World/stage not found — skipping scale and collision.")

        # Add a dome light for uniform scene illumination.
        # Pass intensity/exposure kwargs to override defaults defined in scene_prep.
        add_dome_light(stage)

        # Optionally save the prepared scene as a self-contained USD package.
        # The Collector copies all Nucleus-hosted textures and MDLs locally.
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

        # ----- Spawn drones -----

        ####################################################################################################
        # Spawn vehicle 1
        ####################################################################################################
        graph_handle1 = spawn_px4_multirotor_node(
            pegasus_node_name="PX4Multirotor_1",
            drone_prim="/World/drone1/base_link",
            robot_name="robot_1",
            vehicle_id=1,   # MAVLink port = 14540 + vehicle_id
            domain_id=1,    # ROS 2 domain ID — match vehicle_id by convention
            usd_file=DRONE_USD,
            init_pos=[2.0, 0.0, 0.07],
            init_orient=[0.0, 0.0, 0.0, 1.0],
        )

        # Add a ZED stereo camera subgraph to drone 1
        add_zed_stereo_camera_subgraph(
            parent_graph_handle=graph_handle1,
            drone_prim="/World/drone1/base_link",
            robot_name="robot_1",
            camera_name="ZEDCamera",
            camera_offset=[0.2, 0.0, -0.05],        # X, Y, Z offset from base_link
            camera_rotation_offset=[0.0, 0.0, 0.0],  # roll, pitch, yaw in degrees
        )

        # Add an Ouster lidar subgraph to drone 1
        add_ouster_lidar_subgraph(
            parent_graph_handle=graph_handle1,
            drone_prim="/World/drone1/base_link",
            robot_name="robot_1",
            lidar_name="OS1_REV6_128_10hz___512_resolution",
            lidar_offset=[0.0, 0.0, 0.025],          # X, Y, Z offset from base_link
            lidar_rotation_offset=[0.0, 0.0, 0.0],
            lidar_min_range=0.75,                     # avoid propeller hits
        )

        ####################################################################################################
        # Spawn vehicle 2
        ####################################################################################################
        graph_handle2 = spawn_px4_multirotor_node(
            pegasus_node_name="PX4Multirotor_2",
            drone_prim="/World/drone2/base_link",
            robot_name="robot_2",
            vehicle_id=2,   # MAVLink port = 14540 + vehicle_id
            domain_id=2,    # ROS 2 domain ID — match vehicle_id by convention
            usd_file=DRONE_USD,
            init_pos=[0.0, 0.0, 0.07],
            init_orient=[0.0, 0.0, 0.0, 1.0],
        )

        # Add a ZED stereo camera subgraph to drone 2
        add_zed_stereo_camera_subgraph(
            parent_graph_handle=graph_handle2,
            drone_prim="/World/drone2/base_link",
            robot_name="robot_2",
            camera_name="ZEDCamera",
            camera_offset=[0.2, 0.0, -0.05],
            camera_rotation_offset=[0.0, 0.0, 0.0],
        )

        # Add an Ouster lidar subgraph to drone 2
        add_ouster_lidar_subgraph(
            parent_graph_handle=graph_handle2,
            drone_prim="/World/drone2/base_link",
            robot_name="robot_2",
            lidar_name="OS1_REV6_128_10hz___512_resolution",
            lidar_offset=[0.0, 0.0, 0.025],
            lidar_rotation_offset=[0.0, 0.0, 0.0],
            lidar_min_range=0.75,
        )

        # Reset physics/articulations before playing
        self.world.reset()
        self.stop_sim = False

    def run(self):
        self.timeline.play()

        while simulation_app.is_running() and not self.stop_sim:
            try:
                self.world.step(render=True)
            except Exception as e:
                carb.log_error(f"Simulation step error: {e}")
                break

        carb.log_warn("Closing simulation.")
        self.timeline.stop()
        simulation_app.close()


def main():
    pg_app = PegasusApp()
    pg_app.run()


if __name__ == "__main__":
    main()
