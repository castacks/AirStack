#!/usr/bin/env python3
"""
ROS 2 ReIDPersonArray to CoT Event Publisher

This script acts as a ROS 2 node that subscribes to ReIDPersonArray data
and converts that data into Cursor-On-Target (CoT) events. The CoT events are then sent
over MQTT to a designated topic. The configuration for the MQTT connection is loaded
from a YAML configuration file.

Usage:
    1. Ensure you have Python 3.x installed with the necessary packages:
       pip install rclpy sensor_msgs geometry_msgs paho-mqtt pytak pyyaml
    2. Create a YAML configuration file (e.g., config.yaml) with the following structure:
       project:
         name: test
       logging:
         level: 'INFO'  # Logging level for the services. Options: DEBUG, INFO, WARNING, ERROR, CRITICAL.
       mqtt:
         host: '127.0.0.1'
         port: 1883
         username: 'user'
         password: 'pass'
       services:
         host: '127.0.0.1'
         mediator:
           reidperson2cot_agent:
             reid_topic_name: '/reid/person'
             topic_name: 'to_tak'
    3. Run the script with the following command, specifying the path to the config file:
       ros2 run your_package reid_to_cot_publisher --ros-args -p config_file_path:=config.yaml -p creds_dir:=/path/to/creds
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Quaternion
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
from tak_helper.logger import setup_logger

# Import the custom message types
from std_msgs.msg import Header
from humanflow_msgs.msg import ReIDPerson, ReIDPersonArray


def load_config(file_path):
    """Load configuration from a YAML file."""
    with open(file_path, "r") as f:
        return yaml.safe_load(f)


class PersonData:
    """Class to store the latest data for a detected person."""

    def __init__(self, person_id):
        self.person_id = person_id
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.heading = None  # Quaternion
        self.confidence = 0.0
        self.features = []
        self.keywords = []
        self.last_update_time = 0.0
        self.has_new_data = False
        self.lock = Lock()  # For thread safety

    def update_data(self, reid_person):
        """Update the person data."""
        with self.lock:
            self.latitude = reid_person.gps.latitude
            self.longitude = reid_person.gps.longitude
            self.altitude = reid_person.gps.altitude
            self.heading = reid_person.heading
            self.confidence = reid_person.confidence
            self.features = reid_person.reid_features
            self.keywords = reid_person.keyword_descriptors
            self.last_update_time = time.time()
            self.has_new_data = True

    def get_data(self):
        """Get the current person data."""
        with self.lock:
            self.has_new_data = False
            return {
                "latitude": self.latitude,
                "longitude": self.longitude,
                "altitude": self.altitude,
                "heading": self.heading,
                "confidence": self.confidence,
                "keywords": self.keywords
            }


class ReIDPersonArrayCOTPublisher(Node):
    def __init__(self):
        super().__init__("reidperson_array_cot_publisher")
        self.person_data = {}  # Dictionary to store the latest data for each person

        # Read config from config filename from the ros2 parameters
        self.declare_parameter("config_file_path", "")
        self.config_filepath = self.get_parameter("config_file_path").get_parameter_value().string_value

        # Initialize a basic logger before loading the config
        self.logger = logging.getLogger("ReIDPersonCOT")
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

        # MQTT related configs
        try:
            self.mqtt_broker = config["mqtt"]["host"]
            self.mqtt_port = config["mqtt"]["port"]
            self.mqtt_username = config["mqtt"]["username"]
            self.mqtt_pwd = config["mqtt"]["password"]

            # Get the reid topic name and the output topic name
            self.reid_topic_name = config["services"]["mediator"]["reidperson2cot_agent"]["reid_topic_name"]
            self.mqtt_topicname = config["services"]["mediator"]["reidperson2cot_agent"]["topic_name"]

            self.logger.info(
                f"MQTT CONFIG: Broker={self.mqtt_broker}, Port={self.mqtt_port}, Topic={self.mqtt_topicname}")
            self.logger.info(f"ReID Topic: {self.reid_topic_name}")
        except KeyError as e:
            self.logger.error(f"Missing required configuration key: {e}")
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

        self.logger.info(f"Starting ReIDPersonCOTPublisher for project: {self.project_name}")

        # Create a QoS profile
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscribe to ReIDPersonArray topic
        self.person_array_subscriber = self.create_subscription(
            ReIDPersonArray,
            self.reid_topic_name,
            self.person_array_callback,
            self.qos_profile
        )

        self.logger.info(f"Subscribed to ReIDPersonArray topic: {self.reid_topic_name}")

        # Create a timer to publish data at regular intervals (10Hz)
        self.publisher_timer = self.create_timer(0.1, self.publish_timer_callback)

    def person_array_callback(self, msg):
        """Callback for processing a ReIDPersonArray message."""
        logger = logging.getLogger("ReIDPersonCOT.PersonArray")
        logger.debug(f"Received ReIDPersonArray with {len(msg.peoplemap)} people")

        # Corrected to use the proper field name 'people' from the message definition
        for person in msg.peoplemap:
            person_id = person.global_id

            # Store or update the person data
            if person_id not in self.person_data:
                self.person_data[person_id] = PersonData(person_id)

            self.person_data[person_id].update_data(person)

    def publish_timer_callback(self):
        """Timer callback to check and publish data for all detected persons."""
        # Get a copy of the keys to avoid modification during iteration
        person_ids = list(self.person_data.keys())

        for person_id in person_ids:
            person_data_obj = self.person_data[person_id]

            if person_data_obj.has_new_data:
                # Get the current data
                data = person_data_obj.get_data()

                # Create callsign using the format "person_<global_id>"
                callsign = f"person_{person_id}"

                # Create a CoT event
                cot_event = self.create_person_cot(
                    callsign,
                    data["latitude"],
                    data["longitude"],
                    data["altitude"],
                    data["confidence"],
                    person_id
                )

                # Send the CoT event over MQTT
                self.send_cot_event_over_mqtt(cot_event, f"person_{person_id}")

    def create_person_cot(self, callsign, latitude, longitude, altitude, confidence, person_id):
        """Create a CoT event for a person with ReID data."""
        # Use the existing CoT creation function but add additional details for person
        cot_event = create_gps_COT(
            callsign,
            latitude,
            longitude,
            altitude,
            "ReIDPerson",
            "person"  # type
        )

        # Here you could enhance the CoT message with additional person-specific details
        # For example, adding confidence level or ReID features to the detail section
        # This would require modifying the create_gps_COT function or creating a new one

        return cot_event

    def send_cot_event_over_mqtt(self, cot_event, person_id=None):
        """Send CoT event over the MQTT network"""
        log_prefix = f"ReIDPersonCOT.MQTT.{person_id}" if person_id else "ReIDPersonCOT.MQTT"
        logger = logging.getLogger(log_prefix)
        try:
            self.mqtt_client.publish(self.mqtt_topicname, cot_event)
            logger.debug(f"CoT event published to topic '{self.mqtt_topicname}'")
        except Exception as e:
            logger.error(f"Failed to publish to MQTT: {e}")
            logger.error(f"Exception type: {type(e)}")


def main(args=None):
    # Initialize logger
    logger = logging.getLogger("ReIDPersonCOT.main")
    logger.info("Initializing ROS 2 Python client library")

    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    try:
        # Create an instance of the ReIDPersonArrayCOTPublisher node
        logger.info("Creating ReIDPersonArrayCOTPublisher node")
        reid_cot_publisher = ReIDPersonArrayCOTPublisher()

        # Keep the node running to listen to incoming messages
        logger.info("Node initialized successfully, starting spin")
        rclpy.spin(reid_cot_publisher)
    except Exception as e:
        logger.critical(f"Fatal error occurred: {e}")
    finally:
        # Shutdown and cleanup
        logger.info("Shutting down node")
        if 'reid_cot_publisher' in locals():
            # Stop the MQTT client loop
            if hasattr(reid_cot_publisher, 'mqtt_client'):
                reid_cot_publisher.mqtt_client.loop_stop()
                reid_cot_publisher.mqtt_client.disconnect()
            reid_cot_publisher.destroy_node()
        rclpy.shutdown()
        logger.info("Node has shut down cleanly")


if __name__ == "__main__":
    # Basic logger setup before config is loaded
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(name)s - %(message)s'
    )
    logger = logging.getLogger("ReIDPersonCOT.startup")
    logger.info("Starting ROS 2 ReIDPersonArray to CoT Publisher")
    main()