#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
import paho.mqtt.client as mqtt
from airstack_msgs.msg import SearchMissionRequest  # Import the custom message
from std_msgs.msg import Header
from geometry_msgs.msg import Polygon
from rclpy.qos import QoSProfile

class Cot2Planner(Node):
    def __init__(self, config_file):
        super().__init__('cot2planner')

        # Load configuration
        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)

        # MQTT Configurations
        mqtt_config = config['mqtt']
        self.mqtt_broker = mqtt_config['host']
        self.mqtt_port = int(mqtt_config['port'])
        self.mqtt_topicname = mqtt_config['topicname']
        self.mqtt_username = mqtt_config['username']
        self.mqtt_password = mqtt_config['password']

        # ROS Configurations
        self.ros_topic = config['ros_topic']  # Topic to publish to

        # ROS 2 Publisher
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(SearchMissionRequest, self.ros_topic, qos_profile)

        # Initialize MQTT Client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)
        self.mqtt_client.on_message = self._on_mqtt_message

        # Connect to MQTT broker and start loop
        self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port)
        self.mqtt_client.subscribe(self.mqtt_topicname)
        self.mqtt_client.loop_start()

        self.get_logger().info(f"Subscribed to MQTT topic '{self.mqtt_topicname}' on broker '{self.mqtt_broker}:{self.mqtt_port}'")

    def _on_mqtt_message(self, client, userdata, msg):
        """Callback for incoming MQTT messages."""
        try:
            # Assuming message payload is in JSON format and matches SearchMissionRequest structure
            data = yaml.safe_load(msg.payload)
            message = self._create_ros_message(data)
            self.publisher.publish(message)
            self.get_logger().info(f"Published SearchMissionRequest to '{self.ros_topic}'")
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")

    def _create_ros_message(self, data):
        """Create and populate the SearchMissionRequest message from MQTT data."""
        message = SearchMissionRequest()
        
        # Set fields based on incoming data
        message.header = Header()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'

        # Populate search_bounds, search_priors, keep_out_zones here based on data format
        # Example (ensure your incoming data structure matches this):
        message.search_bounds = Polygon()  # Populate with points if available
        # Example for search_priors and keep_out_zones: 
        # message.search_priors = [...] # Fill in details from data
        # message.keep_out_zones = [...] # Fill in details from data

        return message

    def destroy_node(self):
        self.mqtt_client.loop_stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description="COT to Planner")
    parser.add_argument('--config', type=str, required=True, help='Path to the config YAML file.')
    args = parser.parse_args()

    cot2planner = Cot2Planner(args.config)
    rclpy.spin(cot2planner)
    cot2planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
