#!/usr/bin/env python3

"""
TAK Publisher Script with Enhanced Debugging

Author: Aditya Rauniyar (rauniyar@cmu.edu)
"""

import asyncio
import xml.etree.ElementTree as ET
from pathlib import Path
import sys
import time

import pytak
import socket
import multiprocessing
import argparse
import yaml
from configparser import ConfigParser
import paho.mqtt.client as mqtt

# Force output to be unbuffered
print("SCRIPT STARTING - Print test", flush=True)


class MySender(pytak.QueueWorker):
    def __init__(self, tx_queue, config, event_loop):
        print("INIT: MySender class initialization started", flush=True)
        super().__init__(tx_queue, config["mycottool"])

        # MQTT parameters
        self.mqtt_broker = config['mqtt']['host']
        self.mqtt_port = int(config['mqtt']['port'])
        self.mqtt_username = config['mqtt']['username']
        self.mqtt_pwd = config['mqtt']['password']
        self.mqtt_topicname = config['mqtt']['topicname']

        # Capture the main event loop
        self.event_loop = event_loop

        # Print MQTT connection details
        print(f"DEBUG: MQTT config - Broker: {self.mqtt_broker}, Port: {self.mqtt_port}, Topic: {self.mqtt_topicname}",
              flush=True)

        # Set up MQTT client with explicit callbacks
        print("DEBUG: Creating MQTT client", flush=True)
        self.mqtt_client = mqtt.Client(client_id="tak_publisher_client", protocol=mqtt.MQTTv311)

        # Set callbacks
        print("DEBUG: Setting up MQTT callbacks", flush=True)
        self.mqtt_client.on_connect = self._on_connect
        self.mqtt_client.on_disconnect = self._on_disconnect
        self.mqtt_client.on_message = self._on_message_sync

        # Set credentials if provided
        if self.mqtt_username:
            print(f"DEBUG: Setting up MQTT credentials for user: {self.mqtt_username}", flush=True)
            self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_pwd)

        # Connect to MQTT broker
        print(f"DEBUG: Attempting to connect to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}", flush=True)
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, keepalive=60)
            print("DEBUG: MQTT connect() method completed without exception", flush=True)
        except Exception as e:
            print(f"ERROR: MQTT Connection failed with exception: {e}", flush=True)
            # Don't re-raise to allow the script to continue for debugging
            print(f"ERROR: Exception type: {type(e)}", flush=True)

        print("DEBUG: MySender initialization completed", flush=True)

    def _on_connect(self, client, userdata, flags, rc):
        """Callback for when the client connects to the broker"""
        connection_responses = {
            0: "Connection successful",
            1: "Connection refused - incorrect protocol version",
            2: "Connection refused - invalid client identifier",
            3: "Connection refused - server unavailable",
            4: "Connection refused - bad username or password",
            5: "Connection refused - not authorized"
        }

        status = connection_responses.get(rc, f"Unknown error code: {rc}")
        print(f"MQTT CONNECTION STATUS: {status} (code={rc})", flush=True)

        if rc == 0:
            print(f"SUCCESS: Connected to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}", flush=True)
            # Subscribe to topic
            result, mid = self.mqtt_client.subscribe(self.mqtt_topicname)
            print(f"MQTT Subscribe result: {result}, Message ID: {mid}", flush=True)
            if result == 0:
                print(f"SUCCESS: Subscribed to MQTT topic: {self.mqtt_topicname}", flush=True)
            else:
                print(f"ERROR: Failed to subscribe to MQTT topic: {self.mqtt_topicname}", flush=True)
        else:
            print(f"ERROR: MQTT connection failed: {status}", flush=True)

    def _on_disconnect(self, client, userdata, rc):
        """Callback for when the client disconnects from the broker"""
        if rc == 0:
            print("MQTT: Disconnected from broker cleanly", flush=True)
        else:
            print(f"MQTT: Unexpected disconnect with code {rc}", flush=True)

    def start_mqtt_loop(self):
        """Start MQTT loop in a separate thread."""
        print("DEBUG: Starting MQTT client loop", flush=True)
        self.mqtt_client.loop_start()
        print("DEBUG: MQTT loop started", flush=True)

    def _on_message_sync(self, client, userdata, message):
        """Synchronous wrapper for MQTT on_message to run handle_data in the main event loop."""
        print(f"MQTT: Message received on topic: {message.topic}", flush=True)
        asyncio.run_coroutine_threadsafe(self.handle_data(client, userdata, message), self.event_loop)

    async def handle_data(self, client, userdata, message):
        """Handle incoming MQTT data and put it on the async queue."""
        event = message.payload
        await self.put_queue(event)
        print(f"MQTT: Processed message from topic '{message.topic}' and queued for transmission", flush=True)

    async def run(self, number_of_iterations=-1):
        print("DEBUG: MySender.run() method started", flush=True)
        self.start_mqtt_loop()
        print("DEBUG: MQTT loop started, now waiting in run loop", flush=True)
        try:
            while True:
                await asyncio.sleep(10)  # Keep the loop running, check every 10 seconds
                print("DEBUG: Still running in the MySender.run() loop", flush=True)
        except asyncio.CancelledError:
            print("DEBUG: CancelledError caught, stopping MQTT loop", flush=True)
            self.mqtt_client.loop_stop()
            print("DEBUG: MQTT loop stopped", flush=True)
        except Exception as e:
            print(f"ERROR: Unexpected exception in MySender.run(): {e}", flush=True)
            raise


async def main(config):
    print("DEBUG: main() function started", flush=True)
    loop = asyncio.get_running_loop()  # Capture the main event loop
    clitool = pytak.CLITool(config["mycottool"])
    await clitool.setup()
    print("DEBUG: Adding MySender task to CLITool", flush=True)
    clitool.add_task(MySender(clitool.tx_queue, config, loop))  # Pass the loop
    print("DEBUG: Running CLITool", flush=True)
    await clitool.run()


def run_main_in_process(config):
    print("DEBUG: run_main_in_process() started", flush=True)
    loop = asyncio.get_event_loop()
    print("DEBUG: Event loop created, running main()", flush=True)
    loop.run_until_complete(main(config))


if __name__ == "__main__":
    print("STARTUP: Script main block executing", flush=True)
    parser = argparse.ArgumentParser(description="TAK Publisher Script")
    parser.add_argument('--config_file_path', type=str, required=True, help='Path to the config YAML file.')
    parser.add_argument('--creds_path', type=str, required=True, help='Path to the creds directory.')

    args = parser.parse_args()
    print(f"STARTUP: Args parsed - config_file_path: {args.config_file_path}, creds_path: {args.creds_path}",
          flush=True)

    # Load the YAML configuration
    try:
        with open(args.config_file_path, 'r') as file:
            print(f"STARTUP: Loading configuration from {args.config_file_path}", flush=True)
            config_data = yaml.safe_load(file)
            print("STARTUP: Configuration loaded successfully", flush=True)
    except Exception as e:
        print(f"ERROR: Failed to load configuration: {e}", flush=True)
        sys.exit(1)

    # Extract necessary parameters
    try:
        cot_url = config_data['tak_server']['cot_url']
        pytak_tls_client_cert = config_data['tak_server']['pytak_tls_client_cert']
        # Add the creds_path to the pytak_tls_client_cert
        pytak_tls_client_cert = Path(args.creds_path) / pytak_tls_client_cert
        pytak_tls_client_key = config_data['tak_server']['pytak_tls_client_key']
        # Add the creds_path to the pytak_tls_client_key
        pytak_tls_client_key = Path(args.creds_path) / pytak_tls_client_key
        host = config_data['services']['host']

        # MQTT params
        mqtt_broker = config_data['mqtt']['host']
        mqtt_port = config_data['mqtt']['port']
        mqtt_username = config_data['mqtt']['username']
        mqtt_pwd = config_data['mqtt']['password']
        mqtt_topicname = config_data['services']['publisher']['tak_publisher']['topic_name']

        print(f"MQTT CONFIG: Broker={mqtt_broker}, Port={mqtt_port}, Topic={mqtt_topicname}", flush=True)
    except KeyError as e:
        print(f"ERROR: Missing required configuration key: {e}", flush=True)
        sys.exit(1)

    # Setup config for pytak
    config = ConfigParser()
    config["mycottool"] = {
        "COT_URL": cot_url,
        "PYTAK_TLS_CLIENT_CERT": str(pytak_tls_client_cert),
        "PYTAK_TLS_CLIENT_KEY": str(pytak_tls_client_key),
        "PYTAK_TLS_CLIENT_PASSWORD": "atakatak",
        "PYTAK_TLS_DONT_VERIFY": "1"
    }
    config["service"] = {
        "host": host,
    }

    config["mqtt"] = {
        "host": mqtt_broker,
        "port": str(mqtt_port),  # Convert to string
        "username": mqtt_username,
        "password": mqtt_pwd or "",  # Handle empty password
        "topicname": mqtt_topicname
    }

    # Start the asyncio event loop in a separate process
    print("STARTUP: Starting TAK publisher process", flush=True)
    process = multiprocessing.Process(target=run_main_in_process, args=(config,))
    process.start()

    print("STARTUP: Main() is now running in a separate process", flush=True)
    process.join()
    print("SHUTDOWN: Process completed", flush=True)