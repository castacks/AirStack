#!/usr/bin/env python
"""
Multi-drone PX4 Pegasus launcher, parametrized by NUM_ROBOTS.

Env:
 - NUM_ROBOTS (default 1): how many drones to spawn
 - ENABLE_LIDAR (default false): attach an Ouster lidar to each drone
 - PLAY_SIM_ON_START (default true): autoplay timeline
"""

import asyncio
import os
import sys
import time

import carb
from isaacsim import SimulationApp

# Must be created before any omni imports
_headless = os.environ.get("ISAAC_SIM_HEADLESS", "false").lower() == "true"
simulation_app = SimulationApp({"headless": _headless})

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

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "utils")))
from scene_prep import scale_stage_prim, add_colliders, add_dome_light, save_scene_as_contained_usd


# --------------------- CONFIGURATION ---------------------
ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]
STAGE_SCALE = 1.0
SAVE_SCENE_TO = None
DRONE_USD = "~/.local/share/ov/data/documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd"

NUM_ROBOTS = int(os.environ.get("NUM_ROBOTS", "1"))
ENABLE_LIDAR = os.environ.get("ENABLE_LIDAR", "false").lower() == "true"
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


def spawn_drone(index: int):
    """Spawn drone with vehicle_id=index (1-based), plus camera and optional lidar."""
    robot_name = f"robot_{index}"
    drone_prim = f"/World/drone{index}/base_link"
    # Spread drones along X: -2, 0, 2, 4, ... centered near origin for small counts
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
        add_ouster_lidar_subgraph(
            parent_graph_handle=graph_handle,
            drone_prim=drone_prim,
            robot_name=robot_name,
            lidar_name="OS1_REV6_128_10hz___512_resolution",
            lidar_offset=[0.0, 0.0, 0.025],
            lidar_rotation_offset=[0.0, 0.0, 0.0],
            lidar_min_range=0.75,
        )


class PegasusApp:

    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Keep the timeline stopped throughout setup so OmniGraph doesn't tick early.
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

        print(f"[example_multi] Spawning {NUM_ROBOTS} drone(s), lidar={'on' if ENABLE_LIDAR else 'off'}")
        for i in range(1, NUM_ROBOTS + 1):
            spawn_drone(i)

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
