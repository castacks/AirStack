#!/usr/bin/env python
"""
Minimal PegasusApp launcher that:
 - Starts Isaac Sim
 - Enables required extensions
 - Creates a Pegasus world
 - Starts the timeline and steps until closed
"""

import carb
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment (Must start this before importing omni modules)
simulation_app = SimulationApp({"headless": False})

import rclpy
print(f"[Launcher] SUCCESS: rclpy imported from {rclpy.__file__}")

import omni.kit.app
import omni.timeline
from omni.isaac.core.world import World

# Pegasus imports
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node
from pegasus.simulator.ogn.api.spawn_zed_camera import add_zed_stereo_camera_subgraph
from pegasus.simulator.ogn.api.spawn_ouster_lidar import add_ouster_lidar_subgraph
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from scipy.spatial.transform import Rotation
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
import numpy as np
import os
import subprocess
import threading
import signal
import atexit
import time


# Explicitly enable required extensions
ext_manager = omni.kit.app.get_app().get_extension_manager()

for ext in [
    "airlab.airstack",
    "omni.graph.core",                  # Core runtime for OmniGraph engine
    "omni.graph.action",                # Action Graph framework
    "omni.graph.action_nodes",          # Built-in Action Graph node library
    "isaacsim.core.nodes",              # Core helper nodes for OmniGraph
    "omni.graph.ui",                    # UI scaffolding for graph tools
    "omni.graph.visualization.nodes",   # Visualization helper nodes
    "omni.graph.scriptnode",            # Python script node support
    "omni.graph.window.action",         # Action Graph editor window
    "omni.graph.window.generic",        # Generic graph UI tools
    "omni.graph.ui_nodes",              # UI node building helpers
    "pegasus.simulator",
]:
    if not ext_manager.is_extension_enabled(ext):
        print(f"[Launcher] Enabling extension: {ext}")
        # Try both methods for robustness in different Kit versions
        ext_manager.set_extension_enabled_immediate(ext, True)
        print(f"[Launcher] Successfully enabled extension: {ext} via immediate method")        
    else:
        print(f"[Launcher] Extension already enabled: {ext}")


class PegasusApp:
    """
    Minimal PegasusApp: just loads a Pegasus world and steps simulation
    """

    def __init__(self):
        # Timeline for controlling play/stop
        self.timeline = omni.timeline.get_timeline_interface()

        # Start Pegasus interface + world
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Load default environment. You can replace with url or file path of any desired environment.
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

        # # Spawn a PX4 multirotor drone with a specified vehicle ID and domain ID
        # # PX4 udp port = 14540 + (vehicle_id)
        # # Domain ID is for ROS2 domain communication. As of now, it should match the vehicle id by convention. 
        graph_handle = spawn_px4_multirotor_node(
            pegasus_node_name="PX4Multirotor",
            drone_prim="/World/drone/base_link",
            vehicle_id=1,
            domain_id=1,
            usd_file="/root/Documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd",
            init_pos=[0.0, 0.0, 0.07],
            init_orient=[0.0, 0.0, 0.0, 1.0],
        )

        # Add a ZED stereo camera (with an associated subgraph) to the drone
        add_zed_stereo_camera_subgraph(
            parent_graph_handle=graph_handle,
            drone_prim="/World/drone/base_link",
            camera_name="ZEDCamera",
            camera_offset = [0.2, 0.0, -0.05], # X, Y, Z offset from drone base_link
            camera_rotation_offset = [0.0, 0.0, 0.0], # Rotation in degrees (roll, pitch, yaw)
        )

        # Add an Ouster lidar (with an associated subgraph) to the drone
        add_ouster_lidar_subgraph(
            parent_graph_handle=graph_handle,
            drone_prim="/World/drone/base_link",
            lidar_name="OS1_REV6_128_10hz___512_resolution",
            lidar_offset = [0.0, 0.0, 0.15], # X, Y, Z offset from drone base_link
            lidar_rotation_offset = [0.0, 0.0, 0.0], # Rotation in degrees (roll, pitch, yaw)
        )
        
        # Reset so physics/articulations are ready
        self.world.reset()

        self.stop_sim = False


    def run(self):
        # Start sim timeline if configured to do so
        if os.getenv("PLAY_SIM_ON_START", "true").lower() == "true":
            self.timeline.play()

        # Main loop
        while simulation_app.is_running() and not self.stop_sim:
            # If the simulation is playing, step the world effectively (physics + render)
            
            if self.timeline.is_playing():
                self.world.step(render=True)
                    
            else:
                # If paused/stopped, just update the app/rendering without forcing physics steps
                simulation_app.update()

        # Cleanup
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    pg_app = PegasusApp()
    pg_app.run()


if __name__ == "__main__":
    main()