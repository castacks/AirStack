#!/usr/bin/env python3

"""
TAK Publisher Script

Author: Aditya Rauniyar (rauniyar@cmu.edu)

This script sends Cursor-On-Target (CoT) events to a TAK server by reading COT messages
from a specified localhost TCP socket. The configuration for the TAK server and other
parameters is loaded from a YAML file provided as a command-line argument.

Usage:
    1. Ensure you have Python 3.x installed with the necessary packages:
       pip install asyncio pytak pyyaml

    2. Create a YAML configuration file (e.g., config.yaml) with the necessary parameters.

    3. Run the script with the following command:
       python your_script.py --config path/to/config.yaml
"""

import asyncio
import xml.etree.ElementTree as ET
import pytak
import socket
import multiprocessing
import argparse
import yaml
from configparser import ConfigParser
import paho.mqtt.client as mqtt

class MySender(pytak.QueueWorker):
    def __init__(self, tx_queue, config, event_loop):
        super().__init__(tx_queue, config["mycottool"])

        # MQTT parameters
        self.mqtt_broker = config['mqtt']['host']
        self.mqtt_port = int(config['mqtt']['port'])
        self.mqtt_username = config['mqtt']['username']
        self.mqtt_pwd = config['mqtt']['password']
        self.mqtt_topicname = config['mqtt']['topicname']
        
        # Capture the main event loop
        self.event_loop = event_loop

        # Set up MQTT client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_pwd)
        self.mqtt_client.on_message = self._on_message_sync  # Use sync wrapper

        # Connect to MQTT broker and subscribe to topic
        try:
            print(f"Connecting to {self.mqtt_broker}:{self.mqtt_port}")
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, keepalive=65535)
            self.mqtt_client.subscribe(self.mqtt_topicname)
            print(f"Connected and subscribed to MQTT topic '{self.mqtt_topicname}' on broker {self.mqtt_broker}:{self.mqtt_port}")
        except Exception as e:
            print(f"Failed to connect or subscribe to MQTT: {e}")

    def start_mqtt_loop(self):
        """Start MQTT loop in a separate thread."""
        self.mqtt_client.loop_start()

    def _on_message_sync(self, client, userdata, message):
        """Synchronous wrapper for MQTT on_message to run handle_data in the main event loop."""
        asyncio.run_coroutine_threadsafe(self.handle_data(client, userdata, message), self.event_loop)

    async def handle_data(self, client, userdata, message):
        """Handle incoming MQTT data and put it on the async queue."""
        event = message.payload
        await self.put_queue(event)
        print(f"Received message on '{self.mqtt_topicname}'")

    async def run(self, number_of_iterations=-1):
        self.start_mqtt_loop()
        try:
            while True:
                await asyncio.sleep(0.5)  # Keep the loop running
        except asyncio.CancelledError:
            self.mqtt_client.loop_stop()
            print("MQTT loop stopped.")

async def main(config):
    loop = asyncio.get_running_loop()  # Capture the main event loop
    clitool = pytak.CLITool(config["mycottool"])
    await clitool.setup()
    clitool.add_task(MySender(clitool.tx_queue, config, loop))  # Pass the loop
    await clitool.run()

def run_main_in_process(config):
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main(config))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="TAK Publisher Script")
    parser.add_argument('--config', type=str, required=True, help='Path to the config YAML file.')
    args = parser.parse_args()

    # Load the YAML configuration
    with open(args.config, 'r') as file:
        print(f"Loading configuration from {args.config}")
        config_data = yaml.safe_load(file)

    # Extract necessary parameters
    cot_url = config_data['tak_server']['cot_url']
    pytak_tls_client_cert = config_data['tak_server']['pytak_tls_client_cert']
    pytak_tls_client_key = config_data['tak_server']['pytak_tls_client_key']
    host = config_data['services']['host']

    # MQTT params
    mqtt_broker = config_data['mqtt']['host']
    mqtt_port = config_data['mqtt']['port']
    mqtt_username = config_data['mqtt']['username']
    mqtt_pwd = config_data['mqtt']['password']
    mqtt_topicname = config_data['services']['publisher']['tak_publisher']['topic_name']

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
        "host": host,
    }

    config["mqtt"] = {
        "host": mqtt_broker,
        "port": mqtt_port,
        "username": mqtt_username,
        "password": mqtt_pwd,
        "topicname": mqtt_topicname
    }

    # Start the asyncio event loop in a separate process
    process = multiprocessing.Process(target=run_main_in_process, args=(config,))
    process.start()

    print("Main() is now running in a separate process.")
    process.join()