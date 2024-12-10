#!/usr/bin/env python3

"""
ROS 2 GPS to CoT Event Publisher

Author: Aditya Rauniyar (rauniyar@cmu.edu)

This script acts as a ROS 2 node that subscribes to GPS data from multiple robots
and converts that data into Cursor-On-Target (CoT) events. The CoT events are then sent 
over a TCP socket to a designated server. The configuration for the host and port, as 
well as the number of robots to subscribe to, is loaded from a YAML configuration file.

Usage:
    1. Ensure you have Python 3.x installed with the necessary packages:
       pip install rclpy sensor_msgs pytak pyyaml

    2. Create a YAML configuration file (e.g., config.yaml) with the following structure:
       project:
         name: test
       robot:
         count: 3
         gps_topicname: '/iphone{n}/gps'  # Pattern for topic names
       tak_server:
         cot_url: {Enter the URL of the CoT server}
         pytak_tls_client_cert: '/path/to/cert.pem'
         pytak_tls_client_key: '/path/to/key.key'
       services:
         host: '127.0.0.1'
         publisher:
           tak_publisher:
             port: 10000
         mediator:
           ros2cot_agent:
             port: 10000

    3. Run the script with the following command, specifying the path to the config file:
       python your_script.py --config config.yaml

    4. The script will listen for incoming GPS messages and send CoT events to the 
       configured server.

Note:
    Ensure the server receiving the CoT events is running and listening on the specified port.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import paho.mqtt.client as mqtt
import pytak
import xml.etree.ElementTree as ET
import argparse
import socket
import yaml


def load_config(file_path):
    """Load configuration from a YAML file."""
    with open(file_path, "r") as f:
        return yaml.safe_load(f)


def create_COT_pos_event(uuid, latitude, longitude, altitude):
    """Create a CoT event based on the GPS data."""
    root = ET.Element("event")
    root.set("version", "2.0")
    root.set("type", "a-f-G")
    root.set("uid", uuid)  # Use topic name as UID for identification
    root.set("how", "m-g")
    root.set("time", pytak.cot_time())
    root.set("start", pytak.cot_time())
    root.set("stale", pytak.cot_time(3600))

    pt_attr = {
        "lat": str(latitude),
        "lon": str(longitude),
        "hae": str(altitude),
        "ce": "10",
        "le": "10",
    }

    ET.SubElement(root, "point", attrib=pt_attr)

    return ET.tostring(root, encoding="utf-8")


class ROS2COTPublisher(Node):
    def __init__(self, config):
        super().__init__("ros2cot_publisher")
        self.subscribers = []

        # Get host and port from the config
        self.host = config["services"]["host"]
        self.robots_count = config["robot"]["count"]
        self.gps_topicname_pattern = config["robot"]["gps_topicname"]
        self.project_name = config["project"]["name"]

        # MQTT related configs
        self.mqtt_broker = config["mqtt"]["host"]
        self.mqtt_port = config["mqtt"]["port"]
        self.mqtt_username = config["mqtt"]["username"]
        self.mqtt_pwd = config["mqtt"]["password"]
        self.mqtt_topicname = config["services"]["mediator"]["ros2cot_agent"][
            "topic_name"
        ]

        # Setting MQTT
        self.mqtt_client = mqtt.Client()
        # Set the username and password
        self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_pwd)
        try:
            print(f"Trying to connect to {self.mqtt_broker}:{self.mqtt_port}")
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, keepalive=65535)
            self.get_logger().info(
                f"Connected to MQTT ({self.mqtt_broker}:{self.mqtt_port})"
            )
        except Exception as e:
            print(f"Failed to connect or publish: {e}")

        self.get_logger().info(
            f"Starting ROS2COTPublisher for project: {self.project_name}"
        )
        self.get_logger().info(
            f"Subscribing to {self.robots_count} robot(s) on topics matching: {self.gps_topicname_pattern}"
        )

        # Subscribe to GPS topics based on the pattern
        for i in range(1, self.robots_count + 1):
            topic_name = self.gps_topicname_pattern.format(n=i)
            subscriber = self.create_subscription(
                NavSatFix,
                topic_name,
                lambda msg, topic_name=topic_name: self.gps_callback(msg, f"/robot{i}"),
                10,  # QoS depth
            )
            self.subscribers.append(subscriber)
            self.get_logger().info(f"Subscribed to GPS topic: {topic_name}")

    def gps_callback(self, msg, topic_name):
        """Callback for processing GPS data."""
        latitude = msg.latitude
        longitude = msg.longitude
        altitude = msg.altitude

        # Log the received GPS data
        self.get_logger().info(
            f"Received GPS from {topic_name}: Lat {latitude}, Lon {longitude}, Alt {altitude}"
        )

        # Create a CoT event from the GPS data
        cot_event = create_COT_pos_event(
            f"{self.project_name}{topic_name}", latitude, longitude, altitude
        )

        # Send the CoT event over MQTT
        self.send_cot_event_over_mqtt(cot_event)

    def send_cot_event_over_mqtt(self, cot_event):
        """Send CoT event over the MQTT network"""
        try:
            self.mqtt_client.publish(self.mqtt_topicname, cot_event)
            self.get_logger().info(
                f"Message '{cot_event}' published to topic '{self.mqtt_topicname}'"
            )
        except:
            self.get_logger().error(f"Failed to publish.")

    def send_cot_event_over_network(self, cot_event):
        """Send CoT event over a TCP socket to the configured host."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.bind((self.host, self.port))
                s.connect((self.host, self.port))
                s.sendall(cot_event)
                self.get_logger().info(f"Sent CoT event to {self.host}:{self.port}")
        except ConnectionRefusedError:
            self.get_logger().error(
                f"Connection to {self.host}:{self.port} refused. Ensure the server is running."
            )
        except Exception as e:
            self.get_logger().error(f"Failed to send CoT event: {e}")


def main(args=None):
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Use argparse to handle command line arguments
    parser = argparse.ArgumentParser(description="ROS to CoT Publisher Node")
    parser.add_argument(
        "--config", type=str, required=True, help="Path to the config YAML file."
    )

    # Parse the arguments
    input_args = parser.parse_args()

    # Load configuration
    config = load_config(input_args.config)

    # Create an instance of the ROS2COTPublisher node with the provided configuration
    gps_cot_publisher = ROS2COTPublisher(config)

    # Keep the node running to listen to incoming messages
    rclpy.spin(gps_cot_publisher)

    # Shutdown and cleanup
    gps_cot_publisher.destroy_node()
    rclpy.shutdown()
    print("Node has shut down cleanly.")


if __name__ == "__main__":
    main()
