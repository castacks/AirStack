#!/usr/bin/env python3

"""
ROS 2 CASEVAC Agent

Subscribes to 2 topics that has casualty meta data and image data for the casualty.

Author: Aditya Rauniyar (rauniyar@cmu.edu)

"""

import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import argparse
import yaml
from straps_msgs.msg import CasualtyMeta, Injury, Critical, Vitals
from xml.etree.ElementTree import Element, SubElement, tostring
from datetime import datetime
import pytak
from enum import Enum


"""
Enums to represent the different types of injuries and vitals.
"""


class TraumaType(Enum):
    TRAUMA_HEAD = Injury.TRAUMA_HEAD
    TRAUMA_TORSO = Injury.TRAUMA_TORSO
    TRAUMA_LOWER_EXT = Injury.TRAUMA_LOWER_EXT
    TRAUMA_UPPER_EXT = Injury.TRAUMA_UPPER_EXT
    ALERTNESS_OCULAR = Injury.ALERTNESS_OCULAR
    ALERTNESS_VERBAL = Injury.ALERTNESS_VERBAL
    ALERTNESS_MOTOR = Injury.ALERTNESS_MOTOR


class TraumaSeverity(Enum):
    NORMAL = Injury.TRAUMA_NORMAL
    WOUND = Injury.TRAUMA_WOUND
    AMPUTATION = Injury.TRAUMA_AMPUTATION


class OcularAlertness(Enum):
    OPEN = Injury.OCULAR_OPEN
    CLOSED = Injury.OCULAR_CLOSED
    NOT_TESTABLE = Injury.OCULAR_NOT_TESTABLE


class AlertnessLevel(Enum):
    NORMAL = Injury.ALERTNESS_NORMAL
    ABNORMAL = Injury.ALERTNESS_ABNORMAL
    ABSENT = Injury.ALERTNESS_ABSENT
    NOT_TESTABLE = Injury.ALERTNESS_NOT_TESTABLE


class VitalType(Enum):
    HEART_RATE = Vitals.HEART_RATE
    RESPIRATORY_RATE = Vitals.RESPIRATORY_RATE


class ConditionType(Enum):
    SEVERE_HEMORRHAGE = Critical.SEVERE_HEMORRHAGE
    RESPIRATORY_DISTRESS = Critical.RESPIRATORY_DISTRESS


class ConditionStatus(Enum):
    ABSENT = Critical.ABSENT
    PRESENT = Critical.PRESENT


"""
Functions to validate the type and value of the injury.
"""


def is_valid_type_injury_value(trauma_type, value):
    """Validates that the value matches the type based on the rules."""
    if trauma_type in [TraumaType.TRAUMA_HEAD, TraumaType.TRAUMA_TORSO]:
        # TRAUMA_HEAD and TRAUMA_TORSO should have values NORMAL or WOUND
        return value in [TraumaSeverity.NORMAL, TraumaSeverity.WOUND]

    elif trauma_type in [TraumaType.TRAUMA_LOWER_EXT, TraumaType.TRAUMA_UPPER_EXT]:
        # TRAUMA_LOWER_EXT and TRAUMA_UPPER_EXT should have values NORMAL, WOUND, or AMPUTATION
        return value in [
            TraumaSeverity.NORMAL,
            TraumaSeverity.WOUND,
            TraumaSeverity.AMPUTATION,
        ]

    elif trauma_type == TraumaType.ALERTNESS_OCULAR:
        # ALERTNESS_OCULAR should have values OPEN, CLOSED, or NOT_TESTABLE
        return value in [
            OcularAlertness.OPEN,
            OcularAlertness.CLOSED,
            OcularAlertness.NOT_TESTABLE,
        ]

    elif trauma_type in [TraumaType.ALERTNESS_VERBAL, TraumaType.ALERTNESS_MOTOR]:
        # ALERTNESS_VERBAL and ALERTNESS_MOTOR should have values NORMAL, ABNORMAL, ABSENT, or NOT_TESTABLE
        return value in [
            AlertnessLevel.NORMAL,
            AlertnessLevel.ABNORMAL,
            AlertnessLevel.ABSENT,
            AlertnessLevel.NOT_TESTABLE,
        ]

    return False


# Function to create a unique casualty ID
def create_casualty_id(casualty_id: int):
    return f"casualty-{casualty_id}"


"""
Classes to represent a Casualty object with all the necessary information for triage.
"""


class GPSCOT:
    """
    GPS class to store the GPS coordinates of the casualty.
    """

    # Define the types
    status: bool
    latitude: float
    longitude: float
    altitude: float

    def __init__(self):
        self.status = False
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0

    def set_gps(self, latitude, longitude, altitude):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.status = True


class StatusCOT:
    """
    Base class to store the status information.
    """

    name: str
    status: bool  # True if diagnosis has been made, False otherwise
    system: str
    confidence: float

    def __init__(self, name="", system="", confidence=0.0):
        self.name = name
        self.status = False
        self.system = system
        self.confidence = confidence

    def __str__(self):
        return f"{self.name}({int(self.confidence * 100)}%)"


class VitalsCOT(StatusCOT):
    """
    Derived class to store vitals information.
    """

    vitalsType: VitalType
    value: float
    time_ago: float

    def __init__(
        self,
        vitals_name="",
        system="",
        type_=None,
        value=None,
        time_ago=None,
        confidence=0.0,
    ):
        super().__init__(vitals_name, system, confidence)
        self.vitalsType = type_
        self.value = value
        self.time_ago = time_ago

    def set_vitals(self, system, type_, value, time_ago, confidence):
        if type_ not in [VitalType.HEART_RATE, VitalType.RESPIRATORY_RATE]:
            raise ValueError("Type must be either HEART_RATE or RESPIRATORY_RATE")
        self.system = system
        self.vitalsType = type_
        self.value = value
        self.time_ago = time_ago
        self.confidence = confidence
        self.status = True

    def __str__(self):
        return (
            f"{self.name}({self.value}, {int(self.confidence * 100)}%)"
        )


class InjuryCOT(StatusCOT):
    """
    Derived class to store injury information.
    """

    injuryType: TraumaType
    

    def __init__(
        self, injury_name="", system="", type_=None, value=None, confidence=0.0
    ):
        super().__init__(injury_name, system, confidence)
        self.injuryType = type_
        self.value = value

    def set_status(self, system, type_, value, confidence):
        if not is_valid_type_injury_value(type_, value):
            raise ValueError(f"Invalid value for type {type_}: {value}")
        self.system = system
        self.injuryType = TraumaType(value=type_)
        
        # update the value based on type
        if type_ == TraumaType.TRAUMA_HEAD or type_ == TraumaType.TRAUMA_TORSO or type_ == TraumaType.TRAUMA_LOWER_EXT or type_ == TraumaType.TRAUMA_UPPER_EXT:
            self.value = TraumaSeverity(value=value)
        elif type_ == TraumaType.ALERTNESS_OCULAR:
            self.value = OcularAlertness(value=value)
        elif type_ == TraumaType.ALERTNESS_VERBAL or type_==TraumaType.ALERTNESS_MOTOR:
            self.value = AlertnessLevel(value=value)
        
        self.confidence = confidence
        self.status = True
    
    def __str__(self):
        return (
            f"{self.name}({self.value.name}, {int(self.confidence * 100)}%)"
        )


class CriticalCOT(StatusCOT):
    """
    Derived class to store critical condition information.
    """

    criticalType: ConditionType
    value: ConditionStatus

    def __init__(
        self, critical_name="", system="", type_=None, value=None, confidence=0.0
    ):
        super().__init__(critical_name, system, confidence)
        self.criticalType = type_
        self.value = value

    def set_status(self, system, type_, value, confidence):
        self.system = system
        self.criticalType = type_
        self.value = value
        self.confidence = confidence
        self.status = True


class CasualtyCOT:
    """
    Casualty class to store all the information of a casualty.
    """

    # Define the class types
    gps: GPSCOT
    stamp: str
    casualty_id: str
    # Critical conditions
    severe_hemorrhage: CriticalCOT
    respiratory_distress: CriticalCOT
    # Vitals
    heart_rate: VitalsCOT
    respiratory_rate: VitalsCOT
    # Injuries
    trauma_head: InjuryCOT
    trauma_torso: InjuryCOT
    trauma_lower_ext: InjuryCOT
    trauma_upper_ext: InjuryCOT
    # Alertness
    alertness_ocular: InjuryCOT
    alertness_verbal: InjuryCOT
    alertness_motor: InjuryCOT

    def __init__(self, casualty_id: int):
        self.stamp = pytak.cot_time()

        self.casualty_id = create_casualty_id(casualty_id)

        self.gps = GPSCOT()

        self.severe_hemorrhage = CriticalCOT("Severe Hemorrhage")
        self.respiratory_distress = CriticalCOT("Respiratory Distress")

        self.heart_rate = VitalsCOT("Heart Rate")
        self.respiratory_rate = VitalsCOT("Respiratory Rate")

        self.trauma_head = InjuryCOT("Trauma Head")
        self.trauma_torso = InjuryCOT("Trauma Torso")
        self.trauma_lower_ext = InjuryCOT("Trauma Lower Extremity")
        self.trauma_upper_ext = InjuryCOT("Trauma Upper Extremity")

        self.alertness_ocular = InjuryCOT("Alertness Ocular")
        self.alertness_verbal = InjuryCOT("Alertness Verbal")
        self.alertness_motor = InjuryCOT("Alertness Motor")

    # ZMIST Report Fields:
    # Z: Zap Number – A unique identifier for the casualty.
    # M: Mechanism of Injury – Describes how the injury occurred (e.g., explosion, gunshot wound).
    # I: Injuries Sustained – Specifies the injuries observed (e.g., right leg amputation).
    # S: Signs and Symptoms – Details vital signs and symptoms (e.g., massive hemorrhage, no radial pulse).
    # T: Treatments Rendered – Lists the medical interventions provided (e.g., tourniquet applied, pain medication administered).

    def get_zap_num(self):
        return f"{self.casualty_id}"

    def get_mechanism(self):
        return "Unknown"

    def get_injuries(self):
        """Returns the injuries sustained for preset status"""

        injuries = []
        # Trauma 
        if self.trauma_head.status and self.trauma_head.value != TraumaSeverity.NORMAL:
            injuries.append(str(self.trauma_head))
            print(f"trauma_head detected for {self.casualty_id}")
        if self.trauma_torso.status and self.trauma_torso.value != TraumaSeverity.NORMAL:
            injuries.append(str(self.trauma_torso))
            print(f"trauma_torso detected for {self.casualty_id}")
        if self.trauma_lower_ext.status and self.trauma_lower_ext.value != TraumaSeverity.NORMAL:
            injuries.append(str(self.trauma_lower_ext))
            print(f"trauma_lower_ext detected for {self.casualty_id}")
        if self.trauma_upper_ext.status and self.trauma_upper_ext.value != TraumaSeverity.NORMAL:
            injuries.append(str(self.trauma_upper_ext))
            print(f"trauma_upper_ext detected for {self.casualty_id}")
        # Alertness 
        if self.alertness_ocular.status and self.alertness_ocular.value != OcularAlertness.OPEN:
            injuries.append(str(self.alertness_ocular))            
            print(f"alertness_ocular detected for {self.casualty_id}")   
        if self.alertness_verbal.status and self.alertness_verbal.value != AlertnessLevel.NORMAL:
            injuries.append(str(self.alertness_verbal))            
            print(f"alertness_verbal detected for {self.casualty_id}")   
        if self.alertness_motor.status and self.alertness_motor.value != AlertnessLevel.NORMAL:
            injuries.append(str(self.alertness_motor))            
            print(f"alertness_motor detected for {self.casualty_id}")    
        print(f"Current injuries : {injuries}")
        return ", ".join(injuries)

    def get_signs_symptoms(self):
        """Returns the signs and symptoms for preset status"""

        signs = []
        if self.severe_hemorrhage.status and self.severe_hemorrhage.value == ConditionStatus.PRESENT:
            sev_hem_string = str(self.severe_hemorrhage)
            print(f"severe_hemorrhage: {self.severe_hemorrhage}")
            signs.append(sev_hem_string)
        if self.respiratory_distress.status and self.respiratory_distress.value == ConditionStatus.PRESENT:
            signs.append(str(self.respiratory_distress))
            print(f"Respiratory distress: {str(self.respiratory_distress)}")
        if self.heart_rate.status:
            signs.append(str(self.heart_rate))
            print(f"heart rate: {str(self.heart_rate)}")
        if self.respiratory_rate.status:
            signs.append(str(self.respiratory_rate))
            print(f"Respiratory rate: {str(self.respiratory_rate)}")
        print(f"Current signs: {signs}")
        return ", ".join(signs)

    def get_treatments(self):
        return "Unknown"

    def update_casualty_metadata(self, msg: CasualtyMeta):
        """Updates the casualty metadata with the message data."""

        # Update GPS coordinates
        if msg.valid_gps:
            self.gps.set_gps(msg.gps.latitude, msg.gps.longitude, msg.gps.altitude)

        # Update critical conditions
        if msg.valid_severe_hemorrhage:
            self.severe_hemorrhage.set_status(
                system="Circulatory",
                type_=ConditionType.SEVERE_HEMORRHAGE,
                value=ConditionStatus(msg.severe_hemorrhage.value),
                confidence=msg.severe_hemorrhage.confidence,
            )
        if msg.valid_respiratory_distress:
            self.respiratory_distress.set_status(
                system="Respiratory",
                type_=ConditionType.RESPIRATORY_DISTRESS,
                value=ConditionStatus(msg.respiratory_distress.value),
                confidence=msg.respiratory_distress.confidence,
            )

        # Update vitals
        if msg.valid_heart_rate:
            self.heart_rate.set_vitals(
                system="Cardiovascular",
                type_=VitalType.HEART_RATE,
                value=msg.heart_rate.value,
                time_ago=msg.heart_rate.time_ago,
                confidence=msg.heart_rate.confidence,
            )
        if msg.valid_respiratory_rate:
            self.respiratory_rate.set_vitals(
                system="Respiratory",
                type_=VitalType.RESPIRATORY_RATE,
                value=msg.respiratory_rate.value,
                time_ago=msg.respiratory_rate.time_ago,
                confidence=msg.respiratory_rate.confidence,
            )

        # Update injuries
        if msg.valid_trauma_head:
            self.trauma_head.set_status(
                system="Head",
                type_=TraumaType.TRAUMA_HEAD,
                value=TraumaSeverity(msg.trauma_head.value),
                confidence=msg.trauma_head.confidence,
            )
        if msg.valid_trauma_torso:
            self.trauma_torso.set_status(
                system="Torso",
                type_=TraumaType.TRAUMA_TORSO,
                value=TraumaSeverity(msg.trauma_torso.value),
                confidence=msg.trauma_torso.confidence,
            )
        if msg.valid_trauma_lower_ext:
            self.trauma_lower_ext.set_status(
                system="Lower Extremity",
                type_=TraumaType.TRAUMA_LOWER_EXT,
                value=TraumaSeverity(msg.trauma_lower_ext.value),
                confidence=msg.trauma_lower_ext.confidence,
            )
        if msg.valid_trauma_upper_ext:
            self.trauma_upper_ext.set_status(
                system="Upper Extremity",
                type_=TraumaType.TRAUMA_UPPER_EXT,
                value=TraumaSeverity(msg.trauma_upper_ext.value),
                confidence=msg.trauma_upper_ext.confidence,
            )

        # Update alertness levels
        if msg.valid_alertness_ocular:
            self.alertness_ocular.set_status(
                system="Neurological",
                type_=TraumaType.ALERTNESS_OCULAR,
                value=OcularAlertness(msg.alertness_ocular.value),
                confidence=msg.alertness_ocular.confidence,
            )
        if msg.valid_alertness_verbal:
            self.alertness_verbal.set_status(
                system="Neurological",
                type_=TraumaType.ALERTNESS_VERBAL,
                value=AlertnessLevel(msg.alertness_verbal.value),
                confidence=msg.alertness_verbal.confidence,
            )
        if msg.valid_alertness_motor:
            self.alertness_motor.set_status(
                system="Neurological",
                type_=TraumaType.ALERTNESS_MOTOR,
                value=AlertnessLevel(msg.alertness_motor.value),
                confidence=msg.alertness_motor.confidence,
            )

    def generate_cot_event(self):
        # Create root event element
        event = Element(
            "event",
            {
                "version": "2.0",
                "uid": self.casualty_id,
                "type": "b-r-f-h-c",
                "how": "h-g-i-g-o",
                "time": self.stamp,
                "start": self.stamp,
                "stale": pytak.cot_time(2400),
            },
        )

        # Create point element
        point = SubElement(
            event,
            "point",
            {
                "lat": f"{self.gps.latitude}",
                "lon": f"{self.gps.longitude}",
                "hae": "9999999.0",
                "ce": "9999999.0",
                "le": "9999999.0",
            },
        )

        # Create detail element
        detail = SubElement(event, "detail")

        # Add contact element
        contact = SubElement(detail, "contact", {"callsign": self.casualty_id})

        # Add link element
        link = SubElement(
            detail,
            "link",
            {
                "type": "a-f-G-U-C-I",
                "uid": "S-1-5-21-942292099-3747883346-3641641706-1000",
                "parent_callsign": self.casualty_id,
                "relation": "p-p",
                "production_time": self.stamp,
            },
        )

        # Add archive and status elements
        SubElement(detail, "archive")
        SubElement(detail, "status", {"readiness": "false"})
        SubElement(detail, "remarks")

        # Create _medevac_ element with nested zMistsMap and zMist
        medevac = SubElement(
            detail,
            "_medevac_",
            {
                "title": self.casualty_id.upper(),
                "casevac": "false",
                "freq": "0.0",
                "equipment_none": "true",
                "security": "0",
                "hlz_marking": "3",
                "terrain_none": "true",
                "obstacles": "None",
                "medline_remarks": "",
                "zone_prot_selection": "0",
            },
        )

        zMistsMap = SubElement(medevac, "zMistsMap")
        zMist = SubElement(
            zMistsMap,
            "zMist",
            {
                "z": self.get_zap_num(),
                "m": self.get_mechanism(),
                "i": self.get_injuries(),
                "s": self.get_signs_symptoms(),
                "t": self.get_treatments(),
                "title": "ZMIST1",
            },
        )

        # Add _flow-tags_ element
        flow_tags = SubElement(
            detail,
            "_flow-tags_",
            {
                "TAK-Server-f6edbf55ccfa4af1b4cfa2d7f177ea67": f"chiron-tak_subscriber: {datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%SZ')}"
            },
        )

        # Convert to a string
        return tostring(event, encoding="utf-8").decode("utf-8")


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
    def __init__(self, config):
        super().__init__("ros2casevac_agent")
        self.subscribers = []

        # Get host and port from the config
        self.host = config["services"]["host"]
        self.project_name = config["project"]["name"]

        # MQTT related configs
        self.mqtt_broker = config["mqtt"]["host"]
        self.mqtt_port = config["mqtt"]["port"]
        self.mqtt_username = config["mqtt"]["username"]
        self.mqtt_pwd = config["mqtt"]["password"]
        self.mqtt_topicname = config["services"]["mediator"]["ros2casevac_agent"][
            "topic_name"
        ]

        self.ros_casualty_meta_topic_name = config["services"]["mediator"][
            "ros2casevac_agent"
        ]["ros_casualty_meta_topic_name"]
        self.ros_casualty_image_topic_name = config["services"]["mediator"][
            "ros2casevac_agent"
        ]["ros_casualty_image_topic_name"]

        # Setting MQTT
        self.mqtt_client = mqtt.Client()
        # Set the username and password
        self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_pwd)
        try:
            print(f"Trying to connect to {self.mqtt_broker}:{self.mqtt_port}")
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, keepalive=65535)
            self.mqtt_client.loop_start()
            self.get_logger().info(
                f"Connected to MQTT ({self.mqtt_broker}:{self.mqtt_port})"
            )
        except Exception as e:
            print(f"Failed to connect or publish: {e}")

        self.get_logger().info(
            f"Starting ROS2CASEVAC_AGENT for project: {self.project_name}"
        )

        # Subscribe to the casualty meta data topic with msg type CasualtyMeta
        self.casualty_meta_subscriber = self.create_subscription(
            CasualtyMeta,
            self.ros_casualty_meta_topic_name,
            self.casualty_meta_callback,
            10,
        )
        self.get_logger().info(
            f"Subscribed to {self.ros_casualty_meta_topic_name} topic"
        )

    def casualty_meta_callback(self, msg):
        """Callback for the casualty meta data subscriber"""
        global CASUALTY_META_DATA

        self.get_logger().info(f"Received CasualtyMeta message: {msg}")

        # get the casualty id from the message
        casualty_id = create_casualty_id(msg.casualty_id)

        if casualty_id not in CASUALTY_META_DATA:
            # create a new CasualtyCOT object
            CASUALTY_META_DATA[casualty_id] = CasualtyCOT(msg.casualty_id)
            self.get_logger().info(
                f"Created new CasualtyCOT object for casualty: {casualty_id}"
            )

        # update the CasualtyCOT object with the new data
        CASUALTY_META_DATA[casualty_id].update_casualty_metadata(msg)
        self.get_logger().info(
            f"Updated CasualtyCOT object for casualty: {casualty_id}"
        )

        # send the updated CoT event over MQTT if the GPS data is available
        if CASUALTY_META_DATA[casualty_id].gps.status:
            self.send_cot_event_over_mqtt(
                CASUALTY_META_DATA[casualty_id].generate_cot_event()
            )
            self.get_logger().info(f"Sent CoT event for casualty: {casualty_id}")

    def send_cot_event_over_mqtt(self, cot_event):
        """Send CoT event over the MQTT network"""
        try:
            self.mqtt_client.publish(self.mqtt_topicname, cot_event)
            self.get_logger().info(
                f"Message '{cot_event}' published to topic '{self.mqtt_topicname}'"
            )
        except:
            self.get_logger().error(f"Failed to publish.")


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
