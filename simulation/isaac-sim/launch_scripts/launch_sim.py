#!/usr/bin/env python
"""
| File: 1_px4_single_vehicle.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: This files serves as an example on how to build an app that makes use of the Pegasus API to run a simulation with a single vehicle, controlled using the MAVLink control backend.
"""

# Imports to start Isaac Sim from this script
import carb
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
simulation_app = SimulationApp({
    "headless": False,
    "renderer": "RayTracedLighting",  # optional
    "use_exts": True
})

from omni.isaac.core.utils.extensions import enable_extension

# -----------------------------------
# The actual script should start here
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
# Auxiliary scipy and numpy modules
import os.path
from scipy.spatial.transform import Rotation

from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig

# mavlink_cfg = PX4MavlinkBackendConfig({
#   "vehicle_id": 0,                  # Unique ID for multi-vehicle systems
#   "connection_type": "tcpin",      # Options: "tcpin", "udp", "udpin", etc.
#   "connection_ip": "localhost",    # Host IP for MAVLink link
#   "connection_baseport": 4560,     # Base port (actual = base + vehicle_id)
#   "px4_autolaunch": True,          # Whether to auto-start PX4 SITL
#   "px4_dir": "PegasusInterface().px4_path",    # Path to PX4 source
#   "px4_vehicle_model": "iris",     # PX4 airframe
#   "enable_lockstep": True,         # Synchronize sim and PX4 steps
#   "num_rotors": 4,                 # Vehicle rotor count
#   "input_offset": [0.0]*4,         # RC input calibration
#   "input_scaling": [1000.0]*4,     # RC input gain
#   "zero_position_armed": [100.0]*4,# Neutral input when armed
#   "update_rate": 250.0,            # MAVLink sensor streaming Hz
# })

# config = MultirotorConfig()
# config.backends = [PX4MavlinkBackend(mavlink_cfg)]

class PegasusApp:
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self):
        """
        Method that initializes the PegasusApp and is used to setup the simulation environment.
        """

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics, 
        # spawning asset primitives, etc.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Launch one of the worlds provided by NVIDIA
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

        # Initialize rclpy and create a ROS node
        rclpy.init()
        self.node = Node("pegasus_app_node")

        # Create a publisher on a new topic 'dummy_topic'
        self.dummy_publisher = self.node.create_publisher(String, "dummy_topic", 10)

        # Create the vehicle
        # Try to spawn the selected robot in the world to the specified namespace
        config_multirotor = MultirotorConfig()
        # Create the multirotor configuration
        mavlink_config = PX4MavlinkBackendConfig({
            "vehicle_id": 0,
            "connection_type": "tcpin",
            "connection_ip": "localhost",
            # The actual port that gets used = "connection_baseport" + "vehicle_id"
            "connection_baseport": 4560,
            "enable_lockstep": True,
            "num_rotors": 4,
            "input_offset": [0.0, 0.0, 0.0, 0.0],
            "input_scaling": [1000.0, 1000.0, 1000.0, 1000.0],
            "zero_position_armed": [100.0, 100.0, 100.0, 100.0],
            "update_rate": 250.0,

            # Settings for automatically launching PX4
            # If px4_autolaunch==False, then "px4_dir" and "px4_vehicle_model" are unused
            "px4_autolaunch": True,
            "px4_dir": self.pg.px4_path,
            "px4_vehicle_model": self.pg.px4_default_airframe, # "iris",
        })
        
        config_multirotor.backends = [PX4MavlinkBackend(mavlink_config)]

        Multirotor(
            "/World/quadrotor",
            ROBOTS['Iris'],
            0,
            [0.0, 0.0, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()

        # Auxiliar variable for the timeline callback example
        self.stop_sim = False

    def run(self):
        self.timeline.play()

        count = 0
        while simulation_app.is_running() and not self.stop_sim:
            # Publish dummy message
            msg = String()
            msg.data = f"Hello from PegasusApp count {count}"
            self.dummy_publisher.publish(msg)
            count += 1

            # Allow rclpy to process any ROS messages (even if none subscribed)
            rclpy.spin_once(self.node, timeout_sec=0)

            self.world.step(render=True)

        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()
        # Shutdown rclpy when done
        self.node.destroy_node()
        rclpy.shutdown()

import sys

import threading
from mavros_util import run_mavros_launch_nonblocking, wait_for_mavlink

def main():

    # Instantiate the template app
    pg_app = PegasusApp()

    if (len(sys.argv) >= 1): # Only if extensions are present
        for ext in sys.argv[1].split(","):
            enable_extension(ext)
    
    mavros_process = run_mavros_launch_nonblocking() # Start MAVROS launch 


    # Run the application loop
    pg_app.run()

if __name__ == "__main__":
    main()
