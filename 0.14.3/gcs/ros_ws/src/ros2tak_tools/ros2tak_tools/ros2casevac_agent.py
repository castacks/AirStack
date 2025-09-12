#!/usr/bin/env python3

"""
ROS 2 CASEVAC Agent

Subscribes to 2 topics that has casualty meta data and image data for the casualty.

Author: Aditya Rauniyar (rauniyar@cmu.edu)

Usage:
    1. Run the script with the following command, specifying the path to the config file:
       ros2 run your_package ros2casevac_agent --ros-args -p config_file_path:=config.yaml -p creds_dir:=/path/to/creds
"""

import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import argparse
import yaml
import logging
import sys
from straps_msgs.msg import CasualtyMeta, Injury, Critical, Vitals
from tak_helper.Casualty import CasualtyCOT, create_casualty_id
import pytak
from tak_helper.logger import setup_logger


def load_config(file_path):
    """Load configuration from a YAML file."""
    with open(file_path, "r") as f:
        return yaml.safe_load(f)


############################################################################################################
"""
Global dictionary to store the casualty meta data.
"""
CASUALTY_META_DATA = {}
############################################################################################################


class ROS2COTPublisher(Node):
    def __init__(self):
        super().__init__("ros2casevac_agent")
        self.subscribers = []

        # Read config from config filename from the ros2 parameters
        self.declare_parameter("config_file_path", "")
        self.config_filepath = self.get_parameter("config_file_path").get_parameter_value().string_value

        # Initialize a basic logger before loading the config
        self.logger = logging.getLogger("ROS2CASEVAC")

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
            self.mqtt_topicname = config["services"]["mediator"]["ros2casevac_agent"]["topic_name"]

            self.ros_casualty_meta_topic_name = config["services"]["mediator"]["ros2casevac_agent"]["ros_casualty_meta_topic_name"]
            self.ros_casualty_image_topic_name = config["services"]["mediator"]["ros2casevac_agent"]["ros_casualty_image_topic_name"]

            self.logger.info(
                f"MQTT CONFIG: Broker={self.mqtt_broker}, Port={self.mqtt_port}, Topic={self.mqtt_topicname}")
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

        self.logger.info(f"Starting ROS2CASEVAC_AGENT for project: {self.project_name}")

        # Subscribe to the casualty meta data topic with msg type CasualtyMeta
        self.casualty_meta_subscriber = self.create_subscription(
            CasualtyMeta,
            self.ros_casualty_meta_topic_name,
            self.casualty_meta_callback,
            10,
        )
        self.logger.info(f"Subscribed to {self.ros_casualty_meta_topic_name} topic")

    def casualty_meta_callback(self, msg):
        """Callback for the casualty meta data subscriber"""
        global CASUALTY_META_DATA

        self.logger.info(f"Received CasualtyMeta message: {msg}")

        # get the casualty id from the message
        casualty_id = create_casualty_id(msg.casualty_id)

        if casualty_id not in CASUALTY_META_DATA:
            # create a new CasualtyCOT object
            CASUALTY_META_DATA[casualty_id] = CasualtyCOT(msg.casualty_id)
            self.logger.info(f"Created new CasualtyCOT object for casualty: {casualty_id}")

        # update the CasualtyCOT object with the new data
        CASUALTY_META_DATA[casualty_id].update_casualty_metadata(msg)
        self.logger.info(f"Updated CasualtyCOT object for casualty: {casualty_id}")

        # send the updated CoT event over MQTT if the GPS data is available
        if CASUALTY_META_DATA[casualty_id].gps.status:
            self.send_cot_event_over_mqtt(
                CASUALTY_META_DATA[casualty_id].generate_cot_event(),
                casualty_id
            )
            self.logger.info(f"Sent CoT event for casualty: {casualty_id}")

    def send_cot_event_over_mqtt(self, cot_event, casualty_id=None):
        """Send CoT event over the MQTT network"""
        log_prefix = f"ROS2CASEVAC.MQTT.{casualty_id}" if casualty_id else "ROS2CASEVAC.MQTT"
        logger = logging.getLogger(log_prefix)

        try:
            self.mqtt_client.publish(self.mqtt_topicname, cot_event)
            logger.debug(f"CoT event published to topic '{self.mqtt_topicname}'")
        except Exception as e:
            logger.error(f"Failed to publish to MQTT: {e}")
            logger.error(f"Exception type: {type(e)}")


def main(args=None):
    # Initialize logger
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(name)s - %(message)s'
    )
    logger = logging.getLogger("ROS2CASEVAC.main")
    logger.info("Initializing ROS 2 Python client library")

    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    try:
        # Create an instance of the ROS2COTPublisher node
        logger.info("Creating ROS2CASEVAC_AGENT node")
        casevac_agent = ROS2COTPublisher()

        # Keep the node running to listen to incoming messages
        logger.info("Node initialized successfully, starting spin")
        rclpy.spin(casevac_agent)
    except Exception as e:
        logger.critical(f"Fatal error occurred: {e}")
    finally:
        # Shutdown and cleanup
        logger.info("Shutting down node")
        if 'casevac_agent' in locals():
            # Stop the MQTT client loop
            if hasattr(casevac_agent, 'mqtt_client'):
                casevac_agent.mqtt_client.loop_stop()
                casevac_agent.mqtt_client.disconnect()
            casevac_agent.destroy_node()
        rclpy.shutdown()
        logger.info("Node has shut down cleanly")


if __name__ == "__main__":
    # Basic logger setup before config is loaded
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(name)s - %(message)s'
    )
    logger = logging.getLogger("ROS2CASEVAC.startup")
    logger.info("Starting ROS 2 CASEVAC Agent")

    main()