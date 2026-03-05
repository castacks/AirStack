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
import omni.ui
from omni.isaac.core.world import World
from datetime import datetime
from pxr import UsdLux

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

        # Start Pegasus interface + world
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Load default environment. You can replace with url or file path of any desired environment.
        # self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Default Environment"])
        # self.pg.load_environment("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/DSTA/Theme2/dontfoolmetwice/full_warehouse.usd")
        
        # Timeline for controlling play/stop
        self.timeline = omni.timeline.get_timeline_interface()
        self.timeline.stop()

        # Add a distant light to the scene
        self.add_distant_light()
        
        # # Spawn a PX4 multirotor drone with a specified vehicle ID and domain ID
        # # PX4 udp port = 14540 + (vehicle_id)
        # # Domain ID is for ROS2 domain communication. As of now, it should match the vehicle id by convention. 
        graph_handle = spawn_px4_multirotor_node(
            pegasus_node_name="PX4Multirotor",
            drone_prim="/World/base_link",
            vehicle_id=1,
            domain_id=1,
            usd_file="~/.local/share/ov/data/documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd",
            init_pos=[0.0, 0.0, 0.07],
            init_orient=[0.0, 0.0, 0.0, 1.0],
        )

        # Add a ZED stereo camera (with an associated subgraph) to the drone
        add_zed_stereo_camera_subgraph(
            parent_graph_handle=graph_handle,
            drone_prim="/World/base_link",
            camera_name="ZEDCamera",
            camera_offset = [0.2, 0.0, -0.05], # X, Y, Z offset from drone base_link
            camera_rotation_offset = [0.0, 0.0, 0.0], # Rotation in degrees (roll, pitch, yaw)
        )

        # Add an Ouster lidar (with an associated subgraph) to the drone
        add_ouster_lidar_subgraph(
            parent_graph_handle=graph_handle,
            drone_prim="/World/base_link",
            lidar_name="OS1_REV6_128_10hz___512_resolution",
            lidar_offset = [0.0, 0.0, 0.025], # X, Y, Z offset from drone base_link
            lidar_rotation_offset = [0.0, 0.0, 0.0], # Rotation in degrees (roll, pitch, yaw)
            lidar_min_range = 0.75, # Minimum detection range (m) to avoid propeller hits
        )

        self.play_sim_on_start = os.getenv("PLAY_SIM_ON_START", "false").lower() == "true"
        self.sim_started = False
        if self.play_sim_on_start:
            # Reset so physics/articulations are ready
            self.world.reset()
            self.timeline.play()
            self.sim_started = True
        
        self.stop_sim = False

    def add_distant_light(self):
        """Add a distant light to the scene for better lighting"""
        distant_light = UsdLux.DistantLight.Define(self.world.scene.stage, "/World/DistantLight")
        distant_light.CreateIntensityAttr(1000)

    def run(self):
        # Main loop
        while simulation_app.is_running() and not self.stop_sim:
            
            if self.timeline.is_playing():
                if not self.sim_started:
                    self.world.reset() # Reset so physics/articulations are ready
                    self.sim_started = True

                self.world.step(render=True)
            else:
                # If paused, just update the app (render UI, handle inputs) 
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