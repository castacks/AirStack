# ROS2 CASEVAC Agent

## Overview
ROS2 CASEVAC Agent is a service that bridges ROS2 casualty information to TAK (Tactical Assault Kit) systems using MQTT as the transport layer. The agent subscribes to ROS topics containing casualty metadata and images, converts this information into CoT (Cursor on Target) format, and publishes it to an MQTT broker for TAK systems to consume.

## Features
- Subscribes to ROS casualty metadata and image topics
- Converts casualty information into standard CoT format with ZMIST fields
  - Z: Zap Number - Unique casualty identifier
  - M: Mechanism of Injury
  - I: Injuries Sustained
  - S: Signs and Symptoms
  - T: Treatments Rendered
- Tracks multiple casualties simultaneously
- Handles various injury types and severity levels
- Transmits data over MQTT to TAK systems

## Installation

### Prerequisites
- ROS2 (tested with Foxy/Humble)
- Python 3.8+
- paho-mqtt
- PyTAK

### Dependencies
```bash
pip install paho-mqtt pytak pyyaml
```

## Configuration
Create a YAML configuration file with the following structure:

```yaml
project:
  name: your_project_name

services:
  host: your_host_ip
  mediator:
    ros2casevac_agent:
      topic_name: to_tak  # MQTT topic name for CoT messages
      ros_casualty_meta_topic_name: '/casualty/meta'  # ROS topic for casualty metadata
      ros_casualty_image_topic_name: '/casualty/image'  # ROS topic for casualty images

mqtt:
  host: mqtt_broker_ip
  port: mqtt_broker_port
  username: mqtt_username
  password: mqtt_password
```

## Usage
Run the agent with a configuration file:

```bash
ros2 run ros2tak_tools ros2casevac_agent --config path/to/your/config.yaml
```

## Message Types
The agent expects the following ROS message types:
- `airstack_msgs/CasualtyMeta`: Contains casualty metadata including:
  - GPS coordinates
  - Trauma assessments (head, torso, extremities)
  - Vital signs (heart rate, respiratory rate)
  - Critical conditions (hemorrhage, respiratory distress)
  - Alertness indicators (ocular, verbal, motor)

## How It Works
1. The agent subscribes to the ROS topic for casualty metadata
2. When new data is received, it updates an internal casualty tracking object
3. If GPS data is available, it generates a CoT event in XML format
4. The CoT event is published to the configured MQTT topic
5. TAK systems subscribed to the MQTT topic receive and display the casualty information

## Customization
The code supports several enum types for different injury categories:
- `TraumaType`: Different body regions (head, torso, extremities)
- `TraumaSeverity`: Levels of trauma (normal, wound, amputation)
- `OcularAlertness`: Eye response states
- `AlertnessLevel`: Verbal and motor response states
- `VitalType`: Types of vital signs
- `ConditionType`: Critical conditions
- `ConditionStatus`: Presence/absence of conditions

## Author
Aditya Rauniyar (rauniyar@cmu.edu)