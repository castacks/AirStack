#!/usr/bin/env python
"""
| File: launch_sim.py
| Author: John Liu johnliu@andrew.cmu.edu
| Date: 2025-07-18
| License: BSD-3-Clause. Copyright (c) 2024, AirStack. All rights reserved.
| Description: Launch script for AirStack integration with Pegasus Simulator to run a simulation with
| a single vehicle controlled using MAVLink, including stereo camera sensors as specified in AirStack config.
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
import omni.usd

# -----------------------------------
# The actual script should start here
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, CameraInfo
import yaml
import os
import sys
import subprocess
import atexit
import signal
import threading

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS, NVIDIA_SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.graphical_sensors.lidar import Lidar
# Auxiliary scipy and numpy modules
import numpy as np
from scipy.spatial.transform import Rotation

class AirStackPegasusApp:
    """
    AirStack integration class for Pegasus Simulator. 
    Spawns a drone with stereo cameras matching AirStack camera configuration.
    """

    def __init__(self, environment=SIMULATION_ENVIRONMENTS["Curved Gridroom"]):
        """
        Method that initializes the AirStackPegasusApp and sets up the simulation environment.
        """

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics, 
        # spawning asset primitives, etc.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Load Isaac Sim configuration first to get scene info
        self.isaac_sim_config = self.load_isaac_sim_config()

        # ALWAYS load a Pegasus environment first to ensure proper World initialization
        # This is critical for the World object to have all required attributes like _scene
        carb.log_info("Initializing Pegasus environment for proper World setup...")
        if (environment in NVIDIA_SIMULATION_ENVIRONMENTS):
            self.pg.load_environment(SIMULATION_ENVIRONMENTS[environment])
        else:
            self.pg.load_environment(environment)

        # After Pegasus environment is loaded, check if we should override with custom Omniverse scene
        # custom_scene = self.isaac_sim_config.get('scene')
        # self.pg.load_environment(custom_scene)
        # if custom_scene and (custom_scene.startswith('omniverse://') or custom_scene.endswith('.usd')):
        #     carb.log_info(f"Loading custom Omniverse scene to override environment: {custom_scene}")
        #     if self.load_omniverse_scene(custom_scene):
        #         carb.log_info("Custom Omniverse scene loaded successfully")
        #         self.using_custom_scene = True
        #     else:
        #         carb.log_warn("Failed to load custom scene, using default Pegasus environment")
        #         self.using_custom_scene = False
        # else:
        #     # Using default Pegasus environment
        #     carb.log_info("Using default Pegasus environment")
        #     self.using_custom_scene = False

        # Initialize rclpy and create a ROS node
        rclpy.init()
        self.node = Node("airstack_pegasus_app_node")
        
        # Add clock publisher for simulation time synchronization
        self.clock_pub = self.node.create_publisher(
            Clock, 
            '/clock', 
            10
        )
        
        # Create timer to publish simulation time
        self.clock_timer = self.node.create_timer(0.01, self.publish_clock)  # Publish at 100Hz

        # Load camera configuration from AirStack config
        self.camera_config = self.load_camera_config()
        
        # Extract robot configuration from environment
        self.robot_config = self.get_robot_config()

        # Create the vehicle with sensors
        # The world should now be properly initialized by Pegasus
        carb.log_info("Creating vehicle...")
        carb.log_info(f"World object type: {type(self.world)}")
        carb.log_info(f"World has _scene attribute: {hasattr(self.world, '_scene')}")
        if hasattr(self.world, '_scene'):
            carb.log_info(f"World._scene value: {self.world._scene}")
        
        self.create_vehicle()

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        # This works for both Pegasus environments and custom Omniverse scenes
        carb.log_info("Resetting simulation world...")
        self.world.reset()

        # Start domain bridge to bridge topics from simulation domain (100) to robot domain (1)
        carb.log_info("Starting domain bridge...")
        self.domain_bridge_process = self.start_domain_bridge()
        
        # Register cleanup handler for domain bridge
        atexit.register(self.cleanup_domain_bridge)

        # If using custom scene, ensure everything is properly initialized
        if hasattr(self, 'using_custom_scene') and self.using_custom_scene:
            carb.log_info("Custom Omniverse scene is ready for simulation")

        # Auxiliar variable for the timeline callback example
        self.stop_sim = False

    def load_omniverse_scene(self, scene_url):
        """
        Load an Omniverse USD scene to override the default environment.
        This assumes the World is already properly initialized by Pegasus.
        """
        try:
            carb.log_info(f"Loading Omniverse scene: {scene_url}")
            
            # Get USD context - this is the proper way to access USD functionality
            usd_context = omni.usd.get_context()
            
            # Open the stage synchronously to replace the current scene
            # This will override the Pegasus environment visually
            result = usd_context.open_stage(scene_url)
            
            if result:
                carb.log_info("Omniverse scene loaded successfully")
                
                # Verify stage is valid
                stage = usd_context.get_stage()
                if stage:
                    carb.log_info("USD stage is valid")
                    
                    # Since Pegasus environment was loaded first, physics should already be set up
                    # We may need to refresh physics for the new scene
                    try:
                        from omni.isaac.core.utils.physics import setup_physics_scene
                        setup_physics_scene()
                        carb.log_info("Physics scene refreshed for custom scene")
                    except Exception as e:
                        carb.log_warn(f"Could not refresh physics scene: {e}")
                    
                    return True
                else:
                    carb.log_error("USD stage is invalid after loading")
                    return False
            else:
                carb.log_error(f"Failed to open Omniverse stage: {scene_url}")
                return False
                
        except Exception as e:
            carb.log_error(f"Exception while loading Omniverse scene: {e}")
            return False

    def get_robot_config(self):
        """
        Extract robot configuration from environment variables and defaults.
        Robot-specific config will be moved to a separate robot config file later.
        """
        robot_name = os.environ.get('ROBOT_NAME', 'robot_1')
        
        # Extract robot number from robot name (e.g., 'robot_1' -> 1)
        try:
            if robot_name.startswith('robot_'):
                robot_number = int(robot_name.split('_')[1])
            else:
                robot_number = 1
        except (IndexError, ValueError):
            robot_number = 1
            
        # Set ROS Domain ID to match the robot number (as per AirStack convention)
        # Robot domain is 1, simulation domain is 100 (bridged via domain_bridge)
        ros_domain_id = robot_number
        
        # Use default robot spawn configuration (will be moved to separate robot config file later)
        spawn_position = [0.0, 0.0, 1.0]
        spawn_orientation = [0.0, 0.0, 0.0]
        
        carb.log_info(f"Robot configuration: name={robot_name}, number={robot_number}, domain_id={ros_domain_id}")
        carb.log_info(f"Spawn position: {spawn_position}, orientation: {spawn_orientation}")
        
        return {
            'name': robot_name,
            'number': robot_number, 
            'domain_id': ros_domain_id,
            'spawn_position': spawn_position,
            'spawn_orientation': spawn_orientation
        }

    def get_spawn_position_for_scene(self):
        """
        Get appropriate spawn position based on the scene type.
        """
        if hasattr(self, 'using_custom_scene') and self.using_custom_scene:
            # For fire academy or other custom scenes, spawn at safe height
            return [0.0, 0.0, 5.0]  # 5 meters above ground
        else:
            # Use default spawn position for Pegasus environments  
            return [0.0, 0.0, 0.07]  # Just above ground

    def load_isaac_sim_config(self):
        """
        Load Isaac Sim configuration from config file
        """
        config_path = os.environ.get('ISAAC_SIM_CONFIG_FILE', '/config/isaac_sim_config.yaml')
        
        try:
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
                carb.log_info(f"Loaded Isaac Sim config from {config_path}")
                return config.get('isaac_sim', {})
        except Exception as e:
            carb.log_warn(f"Could not load Isaac Sim config from {config_path}: {e}")
            # Return default Isaac Sim config only
            return {
                'enabled_extensions': ['airlab.airstack', 'pegasus.simulator'],
                'scene': '/isaac-sim/kit/exts/pegasus.simulator/pegasus/simulator/assets/Worlds/Base.usd',
                'play_on_start': True
            }

    def load_camera_config(self):
        """
        Load camera configuration from AirStack camera_config.yaml
        """
        # Path to AirStack camera configuration from environment variable
        config_path = os.environ.get('CAMERA_CONFIG_FILE', '/config/camera_config.yaml')
        
        try:
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
                return config
        except Exception as e:
            carb.log_warn(f"Could not load camera config from {config_path}: {e}")
            # Return default stereo config
            return {
                'base_link_frame_id': 'base_link',
                'camera_list': [{
                    'camera_name': 'front_stereo',
                    'camera_type': 'stereo',
                    'camera_info_sub_topic': 'camera_info',
                    'left_camera_frame_id': 'CameraLeft',
                    'right_camera_frame_id': 'CameraRight'
                }]
            }

    def create_vehicle(self):
        """
        Create the multirotor vehicle with stereo camera sensors based on AirStack configuration.
        """
        # Create the multirotor configuration
        config_multirotor = MultirotorConfig()
        
        # Create the MAVLink backend configuration
        # This will connect to MAVROS running on the robot side
        mavlink_config = PX4MavlinkBackendConfig({
            "vehicle_id": self.robot_config['number'] - 1,  # Vehicle ID is 0-indexed for PX4/MAVLink
            "px4_autolaunch": True,
            "px4_dir": self.pg.px4_path,
            "px4_vehicle_model": self.pg.px4_default_airframe,
        })
        
        # Set up ROS2 backend for sensor publishing with correct domain ID (simulation domain)
        ros2_config = {
            "namespace": "robot_",  # Namespace prefix - will be combined with vehicle_id to form "robot_1/", "robot_2/", etc.
            "ros_domain_id": 100,             # Simulation domain - bridged to robot domain (1) by domain_bridge
            "pub_sensors": True,           # Publish basic sensors (IMU, etc.)
            "pub_graphical_sensors": True, # Publish camera and lidar data
            "pub_state": True,            # Publish vehicle state
            "sub_control": False,         # Don't subscribe to control (handled by MAVLink)
        }
        
        # Configure backends - MAVLink for control, ROS2 for sensor data
        config_multirotor.backends = [
            PX4MavlinkBackend(mavlink_config),
            ROS2Backend(vehicle_id=self.robot_config['number'], config=ros2_config)  # Use 1-indexed vehicle_id
        ]

        # Create stereo camera sensors based on AirStack configuration
        sensors = []
        
        # Add cameras
        for cam_config in self.camera_config.get('camera_list', []):
            if cam_config.get('camera_type') == 'stereo':
                # Create left camera
                left_camera = MonocularCamera(
                    camera_name=f"{cam_config['camera_name']}/left",
                    config={
                        "position": np.array([0.30, -0.05, 0.0]),  # Left camera position
                        "orientation": np.array([0.0, 0.0, 0.0]), # Forward facing
                        "resolution": (1920, 1200),               # High resolution for stereo
                        "frequency": 30,                          # 30 fps
                        "depth": True,                           # Enable depth
                        "intrinsics": np.array([
                            [958.8, 0.0, 960.0], 
                            [0.0, 958.8, 600.0], 
                            [0.0, 0.0, 1.0]
                        ]),
                    }
                )
                
                # Create right camera  
                right_camera = MonocularCamera(
                    camera_name=f"{cam_config['camera_name']}/right", 
                    config={
                        "position": np.array([0.30, 0.05, 0.0]),   # Right camera position (baseline = 0.1m)
                        "orientation": np.array([0.0, 0.0, 0.0]),  # Forward facing
                        "resolution": (1920, 1200),               # Same resolution as left
                        "frequency": 30,                          # Same frequency as left
                        "depth": True,                           # Enable depth
                        "intrinsics": np.array([
                            [958.8, 0.0, 960.0], 
                            [0.0, 958.8, 600.0], 
                            [0.0, 0.0, 1.0]
                        ]),
                    }
                )
                
                sensors.extend([left_camera, right_camera])
        
        # Add LiDAR sensors
        for lidar_config in self.camera_config.get('lidar_list', []):
            if lidar_config.get('lidar_type') == 'RTX_lidar':
                # Create LiDAR sensor
                lidar = Lidar(
                    lidar_name=f"sensors/{lidar_config['lidar_name']}",
                    config={
                        "position": np.array(lidar_config.get('position', [0.0, 0.0, 0.15])),
                        "orientation": np.array(lidar_config.get('orientation', [0.0, 0.0, 0.0])),
                        "sensor_configuration": lidar_config.get('sensor_configuration', 'Velodyne_VLS128'),
                        "frequency": lidar_config.get('frequency', 60.0),
                        "show_render": lidar_config.get('show_render', False),
                    }
                )
                
                sensors.append(lidar)
                carb.log_info(f"Added LiDAR sensor: {lidar_config['lidar_name']}")
        
        # Assign sensors to the multirotor configuration
        config_multirotor.graphical_sensors = sensors

        # Create the multirotor vehicle
        spawn_position = self.get_spawn_position_for_scene()
        carb.log_info(f"Spawning vehicle at position: {spawn_position}")
        
        Multirotor(
            "/World/quadrotor",
            ROBOTS['Iris'],
            self.robot_config['number'],  # Vehicle ID should be 1-indexed to match ROS2 backend
            spawn_position,
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

    def publish_clock(self):
        """
        Publish the simulation clock to the /clock topic.
        """
        try:
            # Get the current simulation time
            if hasattr(self.world, 'current_time'):
                sim_time = self.world.current_time
            elif hasattr(self.world, 'get_time'):
                sim_time = self.world.get_time()
            else:
                sim_time = self.world.get_physics_dt() * self.world.current_time_step_index

            # Create and publish the clock message
            clock_msg = Clock()
            clock_msg.clock.sec = int(sim_time)
            clock_msg.clock.nanosec = int((sim_time - int(sim_time)) * 1e9)
            self.clock_pub.publish(clock_msg)
            
        except Exception as e:
            carb.log_error(f"Error publishing clock: {e}")

    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        self.timeline.play()

        # Log the configuration
        carb.log_info("=" * 60)
        carb.log_info("AirStack Pegasus Simulator Started")
        carb.log_info("=" * 60)
        carb.log_info(f"Robot: {self.robot_config['name']}")
        carb.log_info(f"ROS Domain ID: {self.robot_config['domain_id']} (Simulation in domain 100)")
        carb.log_info(f"Vehicle ID: {self.robot_config['number'] - 1}")
        carb.log_info(f"Domain Bridge: {'Running' if self.domain_bridge_process else 'Failed to start'}")
        
        # Show environment type
        if hasattr(self, 'using_custom_scene') and self.using_custom_scene:
            scene_url = self.isaac_sim_config.get('scene', 'Unknown')
            carb.log_info(f"Environment: Custom Omniverse Scene")
            carb.log_info(f"Scene URL: {scene_url}")
        else:
            carb.log_info(f"Environment: Default Pegasus Environment")
            
        spawn_pos = self.get_spawn_position_for_scene()
        carb.log_info(f"Spawn Position: {spawn_pos}")
        carb.log_info(f"Cameras: {len(self.camera_config.get('camera_list', []))} system(s)")
        carb.log_info(f"LiDAR: {len(self.camera_config.get('lidar_list', []))} sensor(s)")
        carb.log_info(f"MAVLink: UDP port 14540")
        carb.log_info(f"Network: 172.31.0.200")
        carb.log_info("=" * 60)

        # Main simulation loop
        while simulation_app.is_running() and not self.stop_sim:
            # Let the simulation run
            self.world.step(render=True)
            
            # Spin ROS nodes to process timers and callbacks
            rclpy.spin_once(self.node, timeout_sec=0.0)
        
        # Cleanup and stop
        carb.log_warn("AirStack Pegasus Simulation App is closing.")
        
        # Stop domain bridge first
        self.cleanup_domain_bridge()
        
        # Stop simulation
        self.timeline.stop()
        simulation_app.close()

    def start_domain_bridge(self):
        """
        Start the ROS 2 domain bridge to bridge topics from simulation domain (100) to robot domain (1).
        """
        try:
            # Path to the domain bridge configuration file (mounted in docker-compose.yaml)
            bridge_config_path = "/sim_to_robot_bridge.yaml"
            
            # Verify the config file exists
            if not os.path.exists(bridge_config_path):
                carb.log_error(f"Domain bridge config file not found: {bridge_config_path}")
                return None
            
            # Set up environment for domain bridge process
            bridge_env = os.environ.copy()
            bridge_env['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
            
            # Start domain bridge process using ros2 run command
            cmd = ['ros2', 'run', 'domain_bridge', 'domain_bridge', bridge_config_path]
            carb.log_info(f"Starting domain bridge with command: {' '.join(cmd)}")
            
            process = subprocess.Popen(
                cmd,
                env=bridge_env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                preexec_fn=os.setsid  # Create new process group for clean shutdown
            )
            
            # Start a thread to monitor domain bridge output
            def monitor_domain_bridge():
                try:
                    for line in process.stdout:
                        carb.log_info(f"[domain_bridge] {line.strip()}")
                except:
                    pass
                    
            monitor_thread = threading.Thread(target=monitor_domain_bridge, daemon=True)
            monitor_thread.start()
            
            carb.log_info(f"Domain bridge started with PID: {process.pid}")
            return process
            
        except Exception as e:
            carb.log_error(f"Failed to start domain bridge: {e}")
            return None

    def cleanup_domain_bridge(self):
        """
        Clean up the domain bridge process on exit.
        """
        if hasattr(self, 'domain_bridge_process') and self.domain_bridge_process:
            try:
                carb.log_info("Stopping domain bridge...")
                
                # Send SIGTERM to the process group to ensure clean shutdown
                os.killpg(os.getpgid(self.domain_bridge_process.pid), signal.SIGTERM)
                
                # Wait for process to terminate gracefully
                try:
                    self.domain_bridge_process.wait(timeout=5)
                    carb.log_info("Domain bridge stopped gracefully")
                except subprocess.TimeoutExpired:
                    # Force kill if it doesn't stop gracefully
                    carb.log_warn("Domain bridge didn't stop gracefully, force killing...")
                    os.killpg(os.getpgid(self.domain_bridge_process.pid), signal.SIGKILL)
                    self.domain_bridge_process.wait()
                    carb.log_info("Domain bridge force killed")
                    
            except Exception as e:
                carb.log_error(f"Error stopping domain bridge: {e}")

import sys
import threading

def main():
    # Load Isaac Sim configuration to get enabled extensions
    config_path = os.environ.get('ISAAC_SIM_CONFIG_FILE', '/config/isaac_sim_config.yaml')
    
    try:
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
            isaac_sim_config = config.get('isaac_sim', {})
            enabled_extensions = isaac_sim_config.get('enabled_extensions', ['airlab.airstack', 'pegasus.simulator'])
            
            # Get scene from isaac_sim config
            scene = isaac_sim_config.get('scene', SIMULATION_ENVIRONMENTS["Curved Gridroom"])
            
            # For Omniverse URLs, we'll handle them in the AirStackPegasusApp
            # Use default environment here as fallback
            if isinstance(scene, str) and scene.startswith('omniverse://'):
                environment = SIMULATION_ENVIRONMENTS["Curved Gridroom"]
                print("################################################")
                print(f"Custom Omniverse scene configured: {scene}")
                print("################################################")
            else:
                environment = scene
                print("################################################")
                print(f"Using simulation environment: {environment}")
                print("################################################")

    except Exception as e:
        carb.log_warn(f"Could not load Isaac Sim config from {config_path}: {e}, using default extensions")
        enabled_extensions = ['airlab.airstack', 'pegasus.simulator']
        environment = SIMULATION_ENVIRONMENTS["Curved Gridroom"]
    
    # Enable the extensions from config
    for ext in enabled_extensions:
        carb.log_info(f"Enabling extension: {ext}")
        enable_extension(ext)

    # Instantiate the AirStack Pegasus app
    pg_app = AirStackPegasusApp(environment)

    # Run the application loop
    pg_app.run()

if __name__ == "__main__":
    main()
