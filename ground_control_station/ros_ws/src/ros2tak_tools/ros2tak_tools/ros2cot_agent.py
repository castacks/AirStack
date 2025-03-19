#!/usr/bin/env python3

"""
ROS 2 GPS to CoT Event Publisher

Author: Aditya Rauniyar (rauniyar@cmu.edu)

This script acts as a ROS 2 node that subscribes to GPS data from multiple robots
and converts that data into Cursor-On-Target (CoT) events. The CoT events are then sent 
over MQTT to a designated topic. The configuration for the MQTT connection,
as well as the robot streaming configuration, is loaded from a YAML configuration file.

The script now supports publishing GPS data at a specified frequency for each robot.

Usage:
    1. Ensure you have Python 3.x installed with the necessary packages:
       pip install rclpy sensor_msgs paho-mqtt pytak pyyaml

    2. Create a YAML configuration file (e.g., config.yaml) with the following structure:
       project:
         name: test
       logging:
         level: 'INFO'  # Logging level for the services. Options: DEBUG, INFO, WARNING, ERROR, CRITICAL.
       gps_streaming:
         - name: 'drone1'
           type: 'uav'
           topicname: '/robot_1/interface/mavros/global_position/global'
           frequency: 1  # Frequency in Hz
         - name: 'drone2'
           type: 'uav'
           topicname: '/robot_2/interface/mavros/global_position/global'
           frequency: 1  # Frequency in Hz
       mqtt:
         host: '127.0.0.1'
         port: 1883
         username: 'user'
         password: 'pass'
       services:
         host: '127.0.0.1'
         mediator:
           ros2cot_agent:
             topic_name: 'ros2cot/events'

    3. Run the script with the following command, specifying the path to the config file:
       ros2 run your_package your_script --ros-args -p config_file_path:=config.yaml -p creds_dir:=/path/to/creds
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import paho.mqtt.client as mqtt
import pytak
import socket
import yaml
import logging
import sys
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from threading import Lock
from tak_helper.create_cot_msgs import create_gps_COT


def load_config(file_path):
    """Load configuration from a YAML file."""
    with open(file_path, "r") as f:
        return yaml.safe_load(f)


class RobotGPSData:
    """Class to store the latest GPS data for a robot."""

    def __init__(self, robot_name, robot_type, frequency):
        self.robot_name = robot_name
        self.robot_type = robot_type
        self.frequency = frequency
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.last_update_time = 0.0
        self.last_publish_time = 0.0
        self.has_new_data = False
        self.lock = Lock()  # For thread safety

    def update_data(self, latitude, longitude, altitude):
        """Update the GPS data for this robot."""
        with self.lock:
            self.latitude = latitude
            self.longitude = longitude
            self.altitude = altitude
            self.last_update_time = time.time()
            self.has_new_data = True

    def should_publish(self):
        """Check if it's time to publish data based on frequency."""
        current_time = time.time()
        # Only publish if:
        # 1. We have new data since the last publish
        # 2. The publish interval has elapsed (1/frequency seconds)
        with self.lock:
            if not self.has_new_data:
                return False

            time_since_last_publish = current_time - self.last_publish_time
            should_publish = time_since_last_publish >= (1.0 / self.frequency)

            if should_publish:
                # Update the last publish time and mark data as published
                self.last_publish_time = current_time
                self.has_new_data = False

            return should_publish

    def get_data(self):
        """Get the current GPS data."""
        with self.lock:
            return {
                "latitude": self.latitude,
                "longitude": self.longitude,
                "altitude": self.altitude
            }


class ROS2COTPublisher(Node):
    def __init__(self):
        super().__init__("ros2cot_publisher")
        self.subscribers = []
        self.robot_data = {}  # Dictionary to store the latest data for each robot

        # Read config from config filename from the ros2 parameters
        self.declare_parameter("config_file_path", "")
        self.config_filepath = self.get_parameter("config_file_path").get_parameter_value().string_value

        # Initialize a basic logger before loading the config
        self.logger = logging.getLogger("ROS2COT")

        self.get_logger().info(f"Loading configuration from {self.config_filepath}")

        # Load the configuration
        try:
            config = load_config(self.config_filepath)

            # Setup logger based on config
            log_level = config.get('logging', {}).get('level', 'INFO')
            self.logger = setup_logger(self, log_level)
            self.logger.info(f"Logger configured with level: {log_level}")

        except Exception as e:
            self.get_logger().error(f"Failed to load configuration: {e}")
            sys.exit(1)

        # Read the credentials dir from the ros2 parameters
        self.declare_parameter("creds_dir", "")
        self.creds_dir = self.get_parameter("creds_dir").get_parameter_value().string_value
        self.logger.info(f"Loading credentials from {self.creds_dir}")

        # Get host and port from the config
        self.host = config["services"]["host"]
        self.project_name = config["project"]["name"]

        # Get GPS streaming configuration
        self.gps_streaming = config.get("gps_streaming", [])

        if not self.gps_streaming:
            self.logger.warning("No GPS streaming configurations found in config file")

        # MQTT related configs
        try:
            self.mqtt_broker = config["mqtt"]["host"]
            self.mqtt_port = config["mqtt"]["port"]
            self.mqtt_username = config["mqtt"]["username"]
            self.mqtt_pwd = config["mqtt"]["password"]
            self.mqtt_topicname = config["services"]["mediator"]["ros2cot_agent"]["topic_name"]

            self.logger.info(
                f"MQTT CONFIG: Broker={self.mqtt_broker}, Port={self.mqtt_port}, Topic={self.mqtt_topicname}")
        except KeyError as e:
            self.logger.error(f"Missing required MQTT configuration key: {e}")
            sys.exit(1)

        # Setting MQTT
        self.mqtt_client = mqtt.Client()
        # Set the username and password
        self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_pwd)
        try:
            self.logger.info(f"Attempting to connect to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}")
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, keepalive=65535)
            self.mqtt_client.loop_start()  # Start MQTT loop in background thread
            self.logger.info(f"Connected to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}")
        except Exception as e:
            self.logger.error(f"Failed to connect to MQTT broker: {e}")
            self.logger.error(f"Exception type: {type(e)}")

        self.logger.info(f"Starting ROS2COTPublisher for project: {self.project_name}")

        # Create a QoS profile that matches the publisher
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Match the publisher's BEST_EFFORT
            durability=DurabilityPolicy.VOLATILE,  # Match the publisher's VOLATILE
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscribe to GPS topics based on the configuration
        for robot_config in self.gps_streaming:
            robot_name = robot_config.get("name")
            robot_type = robot_config.get("type")
            topic_name = robot_config.get("topicname")
            frequency = robot_config.get("frequency", 1.0)  # Default to 1Hz if not specified

            if not robot_name or not topic_name:
                self.logger.warning(f"Skipping invalid robot config: {robot_config}")
                continue

            # Create a data structure to hold GPS data for this robot
            self.robot_data[robot_name] = RobotGPSData(robot_name, robot_type, frequency)

            subscriber = self.create_subscription(
                NavSatFix,
                topic_name,
                lambda msg, name=robot_name: self.gps_callback(msg, name),
                self.qos_profile
            )
            self.subscribers.append(subscriber)
            self.logger.info(f"Subscribed to GPS topic for {robot_name}: {topic_name}, publishing at {frequency} Hz")

        # Create a timer to check and publish data at regular intervals
        # Use the shortest interval possible (0.01 seconds) to check all robots
        self.publisher_timer = self.create_timer(0.01, self.publish_timer_callback)

    def gps_callback(self, msg, robot_name):
        """Callback for processing GPS data."""
        logger = logging.getLogger(f"ROS2COT.{robot_name}")

        latitude = msg.latitude
        longitude = msg.longitude
        altitude = msg.altitude

        # Log the received GPS data
        logger.debug(
            f"Received GPS data: Lat {latitude:.6f}, Lon {longitude:.6f}, Alt {altitude:.2f}"
        )

        # Update the stored data for this robot
        if robot_name in self.robot_data:
            self.robot_data[robot_name].update_data(latitude, longitude, altitude)
        else:
            logger.warning(f"Received data for unknown robot: {robot_name}")

    def publish_timer_callback(self):
        """Timer callback to check and publish data for all robots based on their frequency."""
        for robot_name, robot_data in self.robot_data.items():
            if robot_data.should_publish():
                # Get the current data
                data = robot_data.get_data()

                # Create a CoT event
                cot_event = create_gps_COT(
                    f"{self.project_name}_{robot_name}",
                    data["latitude"],
                    data["longitude"],
                    data["altitude"],
                    "COT_Event",
                    robot_data.robot_type
                )

                # Send the CoT event over MQTT
                self.send_cot_event_over_mqtt(cot_event, robot_name)

    def send_cot_event_over_mqtt(self, cot_event, robot_name=None):
        """Send CoT event over the MQTT network"""
        log_prefix = f"ROS2COT.MQTT.{robot_name}" if robot_name else "ROS2COT.MQTT"
        logger = logging.getLogger(log_prefix)

        try:
            self.mqtt_client.publish(self.mqtt_topicname, cot_event)
            logger.debug(f"CoT event published to topic '{self.mqtt_topicname}'")
        except Exception as e:
            logger.error(f"Failed to publish to MQTT: {e}")
            logger.error(f"Exception type: {type(e)}")

    def send_cot_event_over_network(self, cot_event, host=None, port=None):
        """Send CoT event over a TCP socket to the configured host."""
        logger = logging.getLogger("ROS2COT.TCP")

        # Use provided host/port or fall back to class attributes
        host = host or self.host
        port = port or getattr(self, 'port', None)

        if not port:
            logger.error("No port configured for TCP connection")
            return

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((host, port))
                s.sendall(cot_event)
                logger.info(f"Sent CoT event to {host}:{port}")
        except ConnectionRefusedError:
            logger.error(f"Connection to {host}:{port} refused. Ensure the server is running.")
        except Exception as e:
            logger.error(f"Failed to send CoT event via TCP: {e}")
            logger.error(f"Exception type: {type(e)}")


def main(args=None):
    # Initialize logger
    logger = logging.getLogger("ROS2COT.main")
    logger.info("Initializing ROS 2 Python client library")

    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    try:
        # Create an instance of the ROS2COTPublisher node
        logger.info("Creating ROS2COTPublisher node")
        gps_cot_publisher = ROS2COTPublisher()

        # Keep the node running to listen to incoming messages
        logger.info("Node initialized successfully, starting spin")
        rclpy.spin(gps_cot_publisher)
    except Exception as e:
        logger.critical(f"Fatal error occurred: {e}")
    finally:
        # Shutdown and cleanup
        logger.info("Shutting down node")
        if 'gps_cot_publisher' in locals():
            # Stop the MQTT client loop
            if hasattr(gps_cot_publisher, 'mqtt_client'):
                gps_cot_publisher.mqtt_client.loop_stop()
                gps_cot_publisher.mqtt_client.disconnect()
            gps_cot_publisher.destroy_node()
        rclpy.shutdown()
        logger.info("Node has shut down cleanly")


if __name__ == "__main__":
    # Basic logger setup before config is loaded
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(name)s - %(message)s'
    )
    logger = logging.getLogger("ROS2COT.startup")
    logger.info("Starting ROS 2 GPS to CoT Publisher")

    main()