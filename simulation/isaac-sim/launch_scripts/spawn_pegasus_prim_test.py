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

# Start Isaac Sim's simulation environment
simulation_app = SimulationApp({"headless": False})

# -----------------------------------
# The actual script starts here
# -----------------------------------
import omni.kit.app
import omni.timeline
from omni.isaac.core.world import World

# Pegasus imports
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node
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


# Explicitly enable required extensions
ext_manager = omni.kit.app.get_app().get_extension_manager()
for ext in [
    # "airlab.airstack",
    "omni.physx.forcefields",
    "omni.graph.core",                  # Core runtime for OmniGraph engine
    "omni.graph.action",                # Action Graph framework
    "omni.graph.action_nodes",          # Built-in Action Graph node library
    "omni.graph.ui",                    # UI scaffolding for graph tools
    "omni.graph.visualization.nodes",   # Visualization helper nodes
    "omni.graph.scriptnode",            # Python script node support
    "omni.graph.window.action",         # Action Graph editor window
    "omni.graph.window.generic",        # Generic graph UI tools
    "omni.graph.ui_nodes",              # UI node building helpers
    "airlab.pegasus",                   # Airlab extension Pegasus core extension
    "pegasus.simulator",
]:
    if not ext_manager.is_extension_enabled(ext):
        ext_manager.set_extension_enabled(ext, True)




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

        # Load default environment
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        
        import omni.graph.core as og

        print("="*60)
        print("Registered OmniGraph Nodes:")
        print("="*60)

        # Iterate all registered node types
        for node_name in og.get_registered_nodes():
            if "Pegasus" in node_name or "pegasus" in node_name or "Ascent" in node_name or "ascent" in node_name:
                print(f" - {node_name}")

        # import omni.graph.core as og
        # print(og.get_node_type("action"))
        spawn_px4_multirotor_node(
            node_name="PX4Multirotor",
            drone_prim="/World/Quadrotor",
            vehicle_id=0,
            usd_file="/root/Documents/Kit/shared/exts/pegasus.simulator/pegasus/simulator/assets/Robots/Iris/iris.usd",
        )
        
        #########################################################################################
        # Create the vehicle using the old method (directly through Pegasus API)
        #########################################################################################
        # Create the vehicle
        # Try to spawn the selected robot in the world to the specified namespace
        # config_multirotor = MultirotorConfig()
        # # Create the multirotor configuration
        # mavlink_config = PX4MavlinkBackendConfig({
        #     "vehicle_id": 0,
        #     "px4_autolaunch": True,
        #     "px4_dir": self.pg.px4_path,
        #     "px4_vehicle_model": self.pg.px4_default_airframe # CHANGE this line to 'iris' if using PX4 version bellow v1.14
        # })
        # # Set up ROS2 backend for sensor publishing with domain configuration based on flag
        # ros2_config = {
        #     "namespace": "robot_",  # Namespace prefix - will be combined with vehicle_id to form "robot_1/", "robot_2/", etc.
        #     "ros_domain_id": 100,
        #     "pub_sensors": True,  # Publish basic sensors (IMU, etc.)
        #     "pub_graphical_sensors": True,  # Publish camera and lidar data
        #     "pub_state": True,  # Publish vehicle state
        #     "sub_control": False,  # Don't subscribe to control (handled by MAVLink)
        #     "use_sim_time": True,  # Use simulation time published on /clock topic
        #     "pub_tf": False,  # Enable TF publishing for dynamic map→robot_1_base_link transform
        # }

        # # Configure backends - MAVLink for control, ROS2 for sensor data
        # config_multirotor.backends = [
        #     PX4MavlinkBackend(mavlink_config),
        #     ROS2Backend(
        #         vehicle_id=1, config=ros2_config
        #     ),  # Use 1-indexed vehicle_id
        # ]
        # sensors = []

        # left_camera = MonocularCamera(
        #     camera_name=f"stereo/left",
        #     config={
        #         "position": np.array(
        #             [0.30, -0.05, 0.0]
        #         ),  # Left camera position
        #         "orientation": np.array([0.0, 0.0, 0.0]),  # Forward facing
        #         "resolution": (1920, 1200),  # High resolution for stereo
        #         "frequency": 30,  # 30 fps
        #         "depth": True,  # Enable depth
        #         "intrinsics": np.array(
        #             [[958.8, 0.0, 960.0], [0.0, 958.8, 600.0], [0.0, 0.0, 1.0]]
        #         ),
        #     },
        # )

        # # Create right camera
        # right_camera = MonocularCamera(
        #     camera_name=f"stereo/right",
        #     config={
        #         "position": np.array(
        #             [0.30, 0.05, 0.0]
        #         ),  # Right camera position (baseline = 0.1m)
        #         "orientation": np.array([0.0, 0.0, 0.0]),  # Forward facing
        #         "resolution": (1920, 1200),  # Same resolution as left
        #         "frequency": 30,  # Same frequency as left
        #         "depth": True,  # Enable depth
        #         "intrinsics": np.array(
        #             [[958.8, 0.0, 960.0], [0.0, 958.8, 600.0], [0.0, 0.0, 1.0]]
        #         ),
        #     },
        # )

        # sensors.extend([left_camera, right_camera])

        # config_multirotor.graphical_sensors = sensors

        # Multirotor(
        #     "/World/quadrotor",
        #     ROBOTS['Iris'],
        #     0,
        #     [0.0, 0.0, 0.07],
        #     Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
        #     config=config_multirotor,
        # )
        #########################################################################################

        print("Spawned PX4 multirotor node")
        print("!" * 60)

        # Reset so physics/articulations are ready
        self.world.reset()

        self.stop_sim = False

        # self.domain_bridge_process = self.start_domain_bridge()
        # atexit.register(self.cleanup_domain_bridge)

    def run(self):
        # Start sim timeline
        self.timeline.play()

        # Main loop
        while simulation_app.is_running() and not self.stop_sim:
            self.world.step(render=True)

        # Cleanup
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()
        self.cleanup_domain_bridge()

    def start_domain_bridge(self):
        """
        Start the ROS 2 domain bridge to bridge topics from simulation domain (100) to robot domain (1).
        """
        try:
            # Path to the domain bridge configuration file (mounted in docker-compose.yaml)
            bridge_config_path = "/sim_to_robot_bridge.yaml"

            # Verify the config file exists
            if not os.path.exists(bridge_config_path):
                carb.log_error(
                    f"Domain bridge config file not found: {bridge_config_path}"
                )
                return None

            # Set up environment for domain bridge process
            bridge_env = os.environ.copy()
            bridge_env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"

            # Start domain bridge process using ros2 run command
            cmd = ["ros2", "run", "domain_bridge", "domain_bridge", bridge_config_path]
            carb.log_info(f"Starting domain bridge with command: {' '.join(cmd)}")

            process = subprocess.Popen(
                cmd,
                env=bridge_env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                preexec_fn=os.setsid,  # Create new process group for clean shutdown
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
        if hasattr(self, "domain_bridge_process") and self.domain_bridge_process:
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
                    carb.log_warn(
                        "Domain bridge didn't stop gracefully, force killing..."
                    )
                    os.killpg(
                        os.getpgid(self.domain_bridge_process.pid), signal.SIGKILL
                    )
                    self.domain_bridge_process.wait()
                    carb.log_info("Domain bridge force killed")

            except Exception as e:
                carb.log_error(f"Error stopping domain bridge: {e}")

def main():
    pg_app = PegasusApp()
    pg_app.run()


if __name__ == "__main__":
    main()
