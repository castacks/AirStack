#!/usr/bin/env python3

"""
COT Subscriber from TAK Server

Author: Aditya Rauniyar (rauniyar@cmu.edu)

Description:
This script acts as a receiver for Cursor-On-Target (CoT) messages. It handles incoming CoT data from a TAK server
and processes it according to the defined logic. The received messages are then sent to a specified
host IP and port defined in the configuration file.

Usage:
1. Ensure you have Python 3.x installed with the necessary packages:
   pip install pytak

2. Create a configuration file with the parameters specified.

3. Run the script:
   python your_script.py --config path/to/config.yaml
"""

import asyncio
import xml.etree.ElementTree as ET
import pytak
import argparse
import socket
import yaml
from configparser import ConfigParser
import logging
import paho.mqtt.client as mqtt

# Log levels: DEBUG, INFO, WARNING, ERROR, CRITICAL
LOG_LEVEL = "DEBUG"



def load_config(file_path):
    """Load configuration from a YAML file."""
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)


class MyReceiver(pytak.QueueWorker):
    """Defines how you will handle events from RX Queue."""

    def __init__(self, rx_queue, config, filter_names2topic):
        super().__init__(rx_queue, config["mycottool"])
        self.host = config["service"]["host"]

        # MQTT parameters
        self.mqtt_broker = config['mqtt']['host']
        self.mqtt_port = int(config['mqtt']['port'])
        self.mqtt_username = config['mqtt']['username']
        self.mqtt_pwd = config['mqtt']['password']

        self._logger.info(f"Sending data to {self.host}:{self.mqtt_port}")
        self.filter_names2topic = filter_names2topic
        self.total_filters = len(self.filter_names2topic)
        print(f"Filter messages number: {self.total_filters}")

        # Set up logging config to print debug message
        self._logger.setLevel(logging.DEBUG)

        # Set up MQTT client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_pwd)

        # Connect to MQTT broker and subscribe to topic
        try:
            self._logger.info(f"Connecting to {self.mqtt_broker}:{self.mqtt_port}")
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port)
            self._logger.info(f"Connected and subscribed to MQTT on broker {self.mqtt_broker}:{self.mqtt_port}")
        except Exception as e:
            self._logger.error(f"Failed to connect or subscribe to MQTT: {e}")


    async def handle_data(self, data):
        """Handle data from the receive queue."""
        self._logger.debug("Received:\n%s\n", data.decode())

        # Parse the CoT message to extract necessary fields
        try:
            root = ET.fromstring(data.decode())
            uuid = root.get("uid")

            self._logger.info(f"Recevied Message: {data}")
            mqtt_topic = self.should_send_message(root)

            # Add your filter conditions here
            if mqtt_topic:
                # Send received data to the specified host and port
                # self._logger.info("Sending data to %s:%s", self.host, self.port)
                await self.send_to_mqtt(data, mqtt_topic=mqtt_topic)
            else:
                # self._logger.info("Filtered out message with UID: %s", uuid)
                self._logger.debug(ET.tostring(root, encoding='unicode', method='xml'))

        except ET.ParseError as e:
            # self._logger.error("Failed to parse CoT message: %s", e)
            pass

    def should_send_message(self, root):
        """Determine whether to send the message based on filtering criteria."""
        # Iterate over the filter messages and check if their name exists in the XML
        for ii in range(self.total_filters):
            filter_name = self.filter_names2topic[ii]["name"]
            # Check if the filter name exists anywhere in the root element
            if self.is_message_relevant(root, filter_name):
                mqtt_topic = self.filter_names2topic[ii]["mqtt_topic"]
                return mqtt_topic

        return None

    def is_message_relevant(self, root, filter_name):
        """Helper function to check if a filter name exists in the XML string."""
        # Convert the XML to a string
        xml_string = ET.tostring(root, encoding='unicode', method='xml')

        self._logger.info(f"Checking with filter: {filter_name}")

        # Check if the filter name exists in the XML string
        if filter_name in xml_string:
            self._logger.debug("Found filter match in XML string: %s", filter_name)
            return True

        return False

    async def send_to_mqtt(self, data, mqtt_topic):
        """Send CoT event over the MQTT network"""
        try: 
            self.mqtt_client.publish(mqtt_topic, data)
            self._logger.info(f"Message published to topic {mqtt_topic}")
            self._logger.debug(f"Message: '{data}'")   
        except:
            self._logger.info(f"Failed to publish.")


    async def run(self):  # pylint: disable=arguments-differ
        """Read from the receive queue, put data onto handler."""
        while True:
            data = await self.queue.get()  # Get received CoT from rx_queue
            await self.handle_data(data)


async def async_main():
    parser = argparse.ArgumentParser(description="TAK Subscriber Script")
    parser.add_argument('--config', type=str, required=True, help='Path to the config YAML file.')
    args, unknown = parser.parse_known_args()

    # Load the YAML configuration
    with open(args.config, 'r') as file:
        config_data = yaml.safe_load(file)

    # Extract necessary parameters from the configuration
    cot_url = config_data['tak_server']['cot_url']
    pytak_tls_client_cert = config_data['tak_server']['pytak_tls_client_cert']
    pytak_tls_client_key = config_data['tak_server']['pytak_tls_client_key']
    host = config_data['services']['host']
    filter_messages = config_data['services']['subscriber']['tak_subscriber']['filter_messages']
    # Extract the filter name and corresponding mqtt topic_name
    message_name2topic = [{"name": msg['name'], "mqtt_topic": msg["mqtt_topic_name"]} for msg in filter_messages]

    # MQTT params
    mqtt_broker = config_data['mqtt']['host']
    # mqtt_broker = "localhost" # Uncomment if running from the host
    mqtt_port = config_data['mqtt']['port']
    mqtt_username = config_data['mqtt']['username']
    mqtt_pwd = config_data['mqtt']['password']

    # Setup config for pytak
    config = ConfigParser()
    config["mycottool"] = {
        "COT_URL": cot_url,
        "PYTAK_TLS_CLIENT_CERT": pytak_tls_client_cert,
        "PYTAK_TLS_CLIENT_KEY": pytak_tls_client_key,
        "PYTAK_TLS_CLIENT_PASSWORD": "atakatak",
        "PYTAK_TLS_DONT_VERIFY": "1"
    }
    
    config["service"] = {
        "host": host
    }

    config["mqtt"] = {
        "host": mqtt_broker,
        "port": mqtt_port,
        "username": mqtt_username,
        "password": mqtt_pwd
    }

    # Initialize worker queues and tasks.
    clitool = pytak.CLITool(config["mycottool"])
    await clitool.setup()

    clitool.add_task(MyReceiver(clitool.rx_queue, config, message_name2topic))
    # Start all tasks.
    await clitool.run()

def main():
    asyncio.run(async_main())

if __name__ == "__main__":
    main()
