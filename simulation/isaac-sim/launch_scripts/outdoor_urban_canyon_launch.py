#!/usr/bin/env python
"""
Outdoor Urban Canyon launch script for AirStack Isaac Sim.

Creates a four-building canyon block, spawns the PX4 multirotor, and runs the
full GPS degradation pipeline (Walker-delta constellation + PhysX raycasting)
inside the simulation loop.

GPS degradation is driven by GPSDegradationNode.physics_step() called each
physics tick, so raycasts execute in the physics thread with the scene active.
"""

import os
import sys
import time

# SimulationApp must be the very first Isaac Sim object created.
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

# All omni/isaacsim imports come after SimulationApp.
import omni.usd
from omni.timeline import get_timeline_interface
from omni.isaac.core.world import World
from pxr import UsdPhysics
import isaacsim.core.utils.prims as prim_utils

from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node

import rclpy

# GPSDegradationNode imports the staging package, which needs Isaac Sim active.
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from gps_degradation_node import GPSDegradationNode


def apply_collision_api(stage, prim_path: str) -> None:
    """Apply PhysicsCollisionAPI to a prim so PhysX raycasts hit it."""
    prim = stage.GetPrimAtPath(prim_path)
    if prim.IsValid() and not prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(prim)


class UrbanCanyonApp:
    def __init__(self):
        self.timeline = get_timeline_interface()
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        self._stage = omni.usd.get_context().get_stage()

        self.create_scene()
        self.spawn_drone()

        rclpy.init()
        self.gps_node = GPSDegradationNode(vehicle_id=1, scenario="urban")

    def create_scene(self) -> None:
        """Build a four-building urban canyon block with collision meshes."""
        prim_utils.create_prim(
            "/World/GroundPlane",
            "Plane",
            position=(0.0, 0.0, 0.0),
            scale=(50.0, 50.0, 1.0),
            attributes={"inputs:displayColor": (0.3, 0.3, 0.3)},
        )
        apply_collision_api(self._stage, "/World/GroundPlane")

        buildings = [
            {"name": "BuildingA", "position": (5.0,  5.0,  7.5), "scale": (4.0, 4.0, 15.0)},
            {"name": "BuildingB", "position": (5.0,  15.0, 7.5), "scale": (4.0, 4.0, 15.0)},
            {"name": "BuildingC", "position": (15.0, 5.0,  7.5), "scale": (4.0, 4.0, 15.0)},
            {"name": "BuildingD", "position": (15.0, 15.0, 7.5), "scale": (4.0, 4.0, 15.0)},
        ]

        for b in buildings:
            path = f"/World/{b['name']}"
            prim_utils.create_prim(
                path,
                "Cube",
                position=b["position"],
                scale=b["scale"],
                attributes={"inputs:displayColor": (0.5, 0.5, 0.5)},
            )
            # PhysicsCollisionAPI is required for raycasts to hit these prims.
            apply_collision_api(self._stage, path)

    def spawn_drone(self) -> None:
        spawn_px4_multirotor_node(
            pegasus_node_name="PX4Multirotor",
            drone_prim="/World/robot_1",
            robot_name="robot_1",
            vehicle_id=1,
            domain_id=100,
            init_pos=[10.0, 10.0, 0.5],   # start inside the canyon
            init_orient=[0.0, 0.0, 0.0, 1.0],
        )

    def run(self) -> None:
        self.timeline.play()
        try:
            while simulation_app.is_running():
                self.world.step(render=False)

                sim_time_s = float(self.world.current_time)

                # PhysX raycasts execute here, in the physics thread, with
                # the scene active — this is the required calling context.
                self.gps_node.physics_step(sim_time_s)

                # Drain incoming ROS 2 messages (non-blocking).
                rclpy.spin_once(self.gps_node, timeout_sec=0.0)

        finally:
            self.gps_node.destroy_node()
            rclpy.shutdown()
            simulation_app.close()


def main() -> None:
    app = UrbanCanyonApp()
    app.run()


if __name__ == "__main__":
    main()
