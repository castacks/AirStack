import os
import xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt
from tak_helper.create_cot_msgs import create_chat_COT, create_gps_COT, create_polygon_COT
from tak_helper.logger import setup_logger
from airstack_msgs.msg import TextQueryResponse
import yaml
import uuid
import logging
import sys
from threading import Lock
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

HELP_MESSAGE = (
    "Welcome to Fleet Control!\n\n"
    "Available commands:\n"
    "1. **robot {robot_name} find {area}**\n"
    "   - Instruct a robot to locate a specified area.\n"
    "     - `robot_name` should be a single word.\n"
    "2. **help**\n"
    "   - Display this help message."
)

# Shape of the CoT message:
POLYGON = "u-d-f"


class AiAgent(Node):
    def __init__(self):
        super().__init__('gcs_ai_agent')

        # Initialize a basic logger before loading the config
        self.logger = logging.getLogger("GCSAIAgent")

        # Read config from config filename from the ros2 parameters
        self.declare_parameter("config_file_path", "")
        self.config_filepath = self.get_parameter("config_file_path").get_parameter_value().string_value

        # Read credentials directory (for compatibility with ros2_cot_agent)
        self.declare_parameter("creds_dir", "")
        self.creds_dir = self.get_parameter("creds_dir").get_parameter_value().string_value

        self.get_logger().info(f"Loading configuration from {self.config_filepath}")
        self.get_logger().info(f"Credentials directory: {self.creds_dir}")

        # Load the configuration
        try:
            with open(self.config_filepath, 'r') as file:
                config = yaml.safe_load(file)

            # Setup logger based on config
            log_level = config.get('logging', {}).get('level', 'INFO')
            self.logger = setup_logger(self, log_level)
            self.logger.info(f"Logger configured with level: {log_level}")

        except Exception as e:
            self.get_logger().error(f"Failed to load configuration: {e}")
            sys.exit(1)

        # Read environment variables for configuration
        self.project_name = config.get("project", {}).get("name", os.getenv("PROJECT_NAME", "airlab"))
        self.ai_agent_name = os.getenv("AI_AGENT_NAME", "aerolens.ai")

        # MQTT Configurations
        mqtt_config = config['mqtt']
        self.mqtt_broker = mqtt_config.get('host', "localhost")
        self.mqtt_port = int(mqtt_config['port'])
        self.mqtt_username = mqtt_config['username']
        self.mqtt_pwd = mqtt_config['password']
        self.mqtt2tak_topic = config['services']['publisher']['tak_publisher']['topic_name']
        self.mqtt_subscribe_topic = config["services"]["mediator"]["chat2ros_agent"]["mqtt_subcribe_topic"]

        # ROS Configurations
        self.ros_robot_query_txt_topic = config["services"]["mediator"]["chat2ros_agent"]["ros_query_text_topic"]
        self.ros_robot_query_response_topic = config["services"]["mediator"]["chat2ros_agent"][
            "ros_query_response_topic"]
        self.robot_publisher = {}

        # Set up MQTT client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_pwd)
        self.mqtt_client.on_message = self._on_mqtt_message

        # Create a QoS profile for ROS subscriptions
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Connect to MQTT broker and subscribe to topic
        try:
            self.logger.info(f"Connecting to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}")
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, keepalive=65535)
            self.mqtt_client.subscribe(self.mqtt_subscribe_topic)
            self.mqtt_client.loop_start()
            self.logger.info(
                f"Connected and subscribed to MQTT topic '{self.mqtt_subscribe_topic}' on broker {self.mqtt_broker}:{self.mqtt_port}")
        except Exception as e:
            self.logger.error(f"Failed to connect or subscribe to MQTT: {e}")
            self.logger.error(f"Exception type: {type(e)}")

    def _get_robot_text_query(self, query):
        return f"{query}"

    def _on_mqtt_message(self, client, userdata, msg):
        # Parse the XML message
        try:
            root = ET.fromstring(msg.payload.decode('utf-8'))
            remarks_tag = root.find(".//remarks")
            remarks = remarks_tag.text.lower()
            self.logger.info(f"Received message: {remarks}")
            self.process_remarks(remarks)
        # Capture NoneType error
        except AttributeError as e:
            self.logger.warning(f"Failed to parse message: {e}")
        except Exception as e:
            self.logger.error(f"Failed to process message: {e}")
            self.logger.error(f"Exception type: {type(e)}")

    def process_remarks(self, remarks):
        """Process the remarks and act accordingly."""
        if remarks.startswith("robot"):
            # Example: "robot {robot_name} find {area}"
            parts = remarks.split(" ")
            if len(parts) >= 4 and parts[2] == "find":
                robot_name = parts[1]
                area = " ".join(parts[3:])
                self.publish_txt_query_to_robot(robot_name, area)
            else:
                self.logger.warning(f"Invalid robot command format: {remarks}")
                help_cot_message = create_chat_COT(uuid=str(uuid.uuid4()), callsign=self.ai_agent_name,
                                                   message=HELP_MESSAGE)
                self.send_message_to_TAK(cot_message=help_cot_message)
        elif remarks.lower() == "help":
            help_cot_message = create_chat_COT(uuid=str(uuid.uuid4()), callsign=self.ai_agent_name,
                                               message=HELP_MESSAGE)
            self.send_message_to_TAK(cot_message=help_cot_message)
        else:
            self.logger.info(f"Unrecognized command: {remarks}")

    def publish_txt_query_to_robot(self, robot_name, area):
        """Publish the area to the ROS robot's query topic."""
        message = self._get_robot_text_query(area)
        msg = String()
        msg.data = message

        request_topic_name = f"/{robot_name}{self.ros_robot_query_txt_topic}"
        response_topic_name = f"/{robot_name}{self.ros_robot_query_response_topic}"

        if robot_name not in self.robot_publisher:
            self.robot_publisher[robot_name] = self.create_publisher(String, request_topic_name, 10)
        self.robot_publisher[robot_name].publish(msg)
        self.logger.info(f"Sent command to {request_topic_name} to find {area}")

        # Send a chat message
        message = f"Sent command to {robot_name} to find '{area}'"
        cot_message = create_chat_COT(uuid=str(uuid.uuid4()), callsign=self.ai_agent_name, message=message)
        self.send_message_to_TAK(cot_message=cot_message)

        # Create a subscriber to listen for the response
        self.create_subscription(
            TextQueryResponse,
            response_topic_name,
            lambda msg: self._on_robot_response(msg, robot_name),
            self.qos_profile
        )

    def _on_robot_response(self, msg, robot_name):
        """Callback for processing robot response and sending CoT messages."""
        log_prefix = f"GCSAIAgent.{robot_name}"
        robot_logger = logging.getLogger(log_prefix)

        # Extracting the header information
        header = msg.header
        robot_logger.info(f"Header information: seq={header.seq}, stamp={header.stamp}, frame_id={header.frame_id}")

        # Extract the tag name
        tag_name = msg.tag_name
        robot_logger.info(f"Tag name: {tag_name}")

        # Extracting the geofence data (which is an array of NavSatFix)
        geofence_data = msg.geofence
        geofence_info = ""

        # Create a list of GPS points
        gps_points = []

        for i, gps_fix in enumerate(geofence_data):
            # Create a list of GPS points with {"lat": str, "lon": str, "hae": str}
            gps_point = (gps_fix.latitude, gps_fix.longitude)
            gps_points.append(gps_point)
            geofence_info += f"Point {i}: lat={gps_fix.latitude}, lon={gps_fix.longitude}\n"

        polygon_cot_message = create_polygon_COT(uuid=tag_name, callsign=self.ai_agent_name, gps_coordinates=gps_points)
        robot_logger.info(f"Geofence data:\n{geofence_info}")

        # Send the polygon CoT message
        self.send_message_to_TAK(polygon_cot_message)

        # Send confirmation chat message
        cot_chat_message = create_chat_COT(
            uuid=str(uuid.uuid4()),
            callsign=self.ai_agent_name,
            message=f"Response received! Please check the geofence data for {tag_name} near {robot_name}"
        )
        self.send_message_to_TAK(cot_chat_message)

    def send_message_to_TAK(self, cot_message):
        """Send a message to the TAK topic."""
        mqtt_logger = logging.getLogger("GCSAIAgent.MQTT")
        try:
            self.mqtt_client.publish(self.mqtt2tak_topic, cot_message)
            mqtt_logger.debug(f"Sent message to topic {self.mqtt2tak_topic}")
        except Exception as e:
            mqtt_logger.error(f"Failed to send message: {e}")
            mqtt_logger.error(f"Exception type: {type(e)}")

    def destroy_node(self):
        """Stop MQTT loop and destroy the node."""
        self.logger.info("Shutting down GCS AI Agent")
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()  # Explicit disconnect
        self.logger.info("MQTT client disconnected")
        super().destroy_node()


def main(args=None):
    # Basic logger setup before config is loaded
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(name)s - %(message)s'
    )
    startup_logger = logging.getLogger("GCSAIAgent.startup")
    startup_logger.info("Starting GCS AI Agent")

    rclpy.init(args=args)

    try:
        # Create the AI agent node
        startup_logger.info("Creating AI Agent node")
        ai_agent = AiAgent()

        # Spin the node
        startup_logger.info("Node initialized successfully, starting spin")
        rclpy.spin(ai_agent)
    except Exception as e:
        startup_logger.critical(f"Fatal error occurred: {e}")
    finally:
        # Shutdown and cleanup
        startup_logger.info("Shutting down node")
        if 'ai_agent' in locals():
            ai_agent.destroy_node()
        rclpy.shutdown()
        startup_logger.info("Node has shut down cleanly")


if __name__ == "__main__":
    main()