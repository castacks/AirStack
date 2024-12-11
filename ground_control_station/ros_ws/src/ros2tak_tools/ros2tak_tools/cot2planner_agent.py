#!/usr/bin/env python3

"""
Description:
This script listens to the MQTT messages from the TAK server and converts them to ROS 2 messages.
Mainly used for Global Planner for getting Search Bounds, Search Priors, and Keep Out Zones.

Usage:
ros2 run ros2tak_tools cot2planner_agent --config <config_file>

Author:
Aditya Rauniyar <rauniyar@cmu.edu> (2024)

"""

import rclpy
from rclpy.node import Node
import xml.etree.ElementTree as ET
import paho.mqtt.client as mqtt
from airstack_msgs.msg import SearchMissionRequest, SearchPrior, KeepOutZone  # Import the custom message
from std_msgs.msg import Header
from geometry_msgs.msg import Polygon, Point32
from rclpy.qos import QoSProfile
import yaml
from enum import Enum
import re

# Constants and Enums
DEFAULT_FRAME_ID = 'map'

# Define event types using Enum for better type safety
class EventType(Enum):
    POLYGON = "u-d-f"
    ROUTE = "b-m-r"
    POINT = "u-d-c-p"
    CIRCLE = "u-d-c-c"

def extract_polygon_points(root):
    """Extract points for the polygon based on link coordinates."""
    points32 = []

    # Find all <link> elements and extract their 'point' attribute
    for link in root.findall('.//link'):
        point_str = link.attrib.get('point')
        # Check if point attribute exists and is non-empty.
        # WinTAK sometimes generates blank points, ATAK is fine.
        if point_str:
            try:
                # Split the single point into latitude and longitude
                lat, lon = map(float, point_str.split(','))

                # Create a Point32 object with lat and lon, set z=0.0 (altitude is not used here)
                point = Point32()
                point.x = lat
                point.y = lon
                point.z = 0.0  # Default z value (altitude not available)

                # Add the point to the list
                points32.append(point)
            except ValueError as e:
                print(f"Warning: Skipping invalid point: {point_str} - Error: {e}")

    return points32

def extract_value_priority(callsign):
    """Extract value and priority from a callsign like 'planner_sp_v0.2_p3.4'."""
    match = re.search(r'v([0-9.]+)_p([0-9.]+)', callsign)
    if match:
        value = float(match.group(1))
        priority = float(match.group(2))
        return value, priority
    return 0.0, 1.0  # Default values if no match


def _process_polygons(root):
    """Process XML for 'u-d-f' type and convert to SearchBound and SearchPrior."""
    points32 = extract_polygon_points(root)
    # Create ROS message
    polygon = Polygon()

    # Set the points for the polygon
    polygon.points = points32

    return polygon


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
        self.mqtt_username = mqtt_config['username']
        self.mqtt_password = mqtt_config['password']

        # ROS Configurations
        # Reading the ros_topic_name for the planner under filter_messages -> planner
        planner_config = next(
            item for item in config['services']['subscriber']['tak_subscriber']['filter_messages']
            if item['name'] == 'planner'
        )
        self.ros_topic = planner_config['ros_topic_name']  # Read the ros_topic_name dynamically
        self.mqtt_topicname = planner_config['mqtt_topic_name']  # Read the MQTT topic name for planner events

        # ROS 2 Publisher
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(SearchMissionRequest, self.ros_topic, qos_profile)

        # Initialize MQTT Client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)
        self.mqtt_client.on_message = self._on_mqtt_message

        # Connect to MQTT broker and start loop
        self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, keepalive=65535)
        self.mqtt_client.subscribe(self.mqtt_topicname)  # Subscribe to the dynamic planner topic
        self.mqtt_client.loop_start()

        # Planner message request to be sent
        self.plan_msg_request = self._initialize_ros_message_header(SearchMissionRequest(), frame_id=DEFAULT_FRAME_ID)

        self.get_logger().info(f"Subscribed to MQTT topic '{self.mqtt_topicname}' on broker '{self.mqtt_broker}:{self.mqtt_port}'")

    def _initialize_ros_message_header(self, message, frame_id=DEFAULT_FRAME_ID):
        """Initialize the ROS message with header info."""
        message.header = Header()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = frame_id
        return message

    def _on_mqtt_message(self, client, userdata, msg):
        """Callback for incoming MQTT messages."""
        try:
            # Parse the XML message
            root = ET.fromstring(msg.payload.decode('utf-8'))

            # Extract event type from the XML
            event_type = root.attrib.get('type')

            # Extract the callsign from the XML
            callsign = root.find('.//contact').attrib.get('callsign', '')

            # Handle event types using Enum values
            if event_type == EventType.POLYGON.value or event_type == EventType.ROUTE.value or event_type == EventType.POINT.value:
                polygon = _process_polygons(root)

                # Check if the callsign contains 'sb' for search bounds
                if 'sb' in callsign.lower():
                    self.plan_msg_request.search_bounds = polygon
                    self.get_logger().info(f"Added SearchBounds with callsign '{callsign}' to the SearchMissionRequest")

                event_shape_type = "polygon" if event_type == EventType.POLYGON.value else \
                    "route" if event_type == EventType.ROUTE.value else "point"

                # Process as search prior
                self._process_search_priors(polygon, call_sign=callsign, shape_type=event_shape_type)

            elif event_type == EventType.CIRCLE.value:
                self._process_keep_out_zones(root)
                self.get_logger().info(f"Added KeepOutZones with callsign '{callsign}' to the SearchMissionRequest")
            else:
                self.get_logger().info(f"Event type '{event_type}' not supported")

            # Print the current self.plan_msg_request
            # self.get_logger().info(f"Current SearchMissionRequest: {self.plan_msg_request}")

            # Check if the xml contains word "end" in the remarks field mostly the remarks field is kept empty
            remarks = root.find('.//remarks').text
            if remarks and 'end' in remarks.lower():
                # Publish the SearchMissionRequest message
                self.publisher.publish(self.plan_msg_request)
                self.get_logger().info(f"Published SearchMissionRequest to '{self.ros_topic}'")
                # Reset the plan_msg_request for next mission
                self.plan_msg_request = self._initialize_ros_message_header(SearchMissionRequest(), frame_id=DEFAULT_FRAME_ID)

        except (ET.ParseError, mqtt.MQTTException) as e:
            self.get_logger().error(f"Error processing MQTT message: {e}")

    def _process_search_priors(self, polygon, call_sign="", shape_type="polygon"):
        """Process XML for 'b-m-r' type and convert to SearchBound and SearchPrior."""

        # Extract the value and priority from the callsign
        value, priority = extract_value_priority(call_sign)

        # Create a SearchPrior message
        search_prior = self._initialize_ros_message_header(SearchPrior(), frame_id=DEFAULT_FRAME_ID)

        # Set the prior type as polygon
        if shape_type == "polygon":
            search_prior.grid_prior_type = SearchPrior.POLYGON_PRIOR
        elif shape_type == "route":
            search_prior.grid_prior_type = SearchPrior.LINE_SEG_PRIOR
        elif shape_type == "point":
            search_prior.grid_prior_type = SearchPrior.POINT_PRIOR

        # set polygon, value, and priority
        search_prior.points_list = polygon
        search_prior.value = [value]
        search_prior.priority = [priority]

        # Update the plan_msg_request with the search_prior
        self.plan_msg_request.search_priors.append(search_prior)
        self.get_logger().info(f"Added SearchPriors to the SearchMissionRequest with callSign={call_sign}, "
                               f"type={shape_type}, value={value}, priority={priority}")


    def _process_keep_out_zones(self, root):
        """Process XML for 'u-d-c-c' type and convert to KeepOutZone."""
        # Extract data from XML
        point = root.find('.//point')
        if point is not None:
            lat = float(point.attrib.get('lat', 0.0))
            lon = float(point.attrib.get('lon', 0.0))
        else:
            lat = lon = 0.0

        ellipse = root.find('.//ellipse')
        major_ellipse = float(ellipse.attrib.get('major', 0.0)) if ellipse is not None else 0.0

        # Create ROS message
        message = self._initialize_ros_message_header(KeepOutZone(), frame_id=DEFAULT_FRAME_ID)

        # Set the x, y, z_min, z_max, and radius fields
        message.x = lat
        message.y = lon
        message.z_min = 0.0
        message.z_max = 0.0
        message.radius = major_ellipse  # Radius is the major ellipse axis in meters

        # Append the KeepOutZone message to the plan_msg_request
        self.plan_msg_request.keep_out_zones.append(message)

    def destroy_node(self):
        """Stop MQTT loop and destroy the node."""
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()  # Explicit disconnect
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    import argparse
    parser = argparse.ArgumentParser(description="COT to Planner")
    parser.add_argument('--config', type=str, required=True, help='Path to the config YAML file.')
    args = parser.parse_args()

    cot2planner = Cot2Planner(args.config)
    rclpy.spin(cot2planner)
    cot2planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
