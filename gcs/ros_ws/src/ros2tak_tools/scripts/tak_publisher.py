#!/usr/bin/env python3

"""
TAK Publisher Script with Enhanced Logging

Author: Aditya Rauniyar (rauniyar@cmu.edu)
"""

import asyncio
from pathlib import Path
import sys
import logging
import logging.handlers
import multiprocessing
import argparse
import yaml
from configparser import ConfigParser
import paho.mqtt.client as mqtt
import pytak


def setup_logger(log_level):
    """
    Set up the logger with appropriate log level and formatting.

    Args:
        log_level: The log level from config (DEBUG, INFO, WARNING, ERROR, CRITICAL)

    Returns:
        Configured logger object
    """
    # Convert string log level to logging constants
    level_map = {
        'DEBUG': logging.DEBUG,
        'INFO': logging.INFO,
        'WARNING': logging.WARNING,
        'ERROR': logging.ERROR,
        'CRITICAL': logging.CRITICAL
    }

    # Default to INFO if level not recognized
    numeric_level = level_map.get(log_level, logging.INFO)

    # Configure root logger
    logger = logging.getLogger()
    logger.setLevel(numeric_level)

    # Console handler with improved formatting
    console = logging.StreamHandler()
    console.setLevel(numeric_level)

    # Format: timestamp - level - component - message
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(name)s - %(message)s')
    console.setFormatter(formatter)

    # Add handler to logger
    logger.addHandler(console)

    return logger


class MySender(pytak.QueueWorker):
    def __init__(self, tx_queue, config, event_loop):
        self.logger = logging.getLogger("MySender")
        self.logger.debug("Initializing MySender class")
        super().__init__(tx_queue, config["mycottool"])

        # MQTT parameters
        self.mqtt_broker = config['mqtt']['host']
        self.mqtt_port = int(config['mqtt']['port'])
        self.mqtt_username = config['mqtt']['username']
        self.mqtt_pwd = config['mqtt']['password']
        self.mqtt_topicname = config['mqtt']['topicname']

        # Capture the main event loop
        self.event_loop = event_loop

        # Log MQTT connection details
        self.logger.info(
            f"MQTT config - Broker: {self.mqtt_broker}, Port: {self.mqtt_port}, Topic: {self.mqtt_topicname}")

        # Set up MQTT client with explicit callbacks
        self.logger.debug("Creating MQTT client")
        self.mqtt_client = mqtt.Client(client_id="tak_publisher_client", protocol=mqtt.MQTTv311)

        # Set callbacks
        self.logger.debug("Setting up MQTT callbacks")
        self.mqtt_client.on_connect = self._on_connect
        self.mqtt_client.on_disconnect = self._on_disconnect
        self.mqtt_client.on_message = self._on_message_sync

        # Set credentials if provided
        if self.mqtt_username:
            self.logger.debug(f"Setting up MQTT credentials for user: {self.mqtt_username}")
            self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_pwd)

        # Connect to MQTT broker
        self.logger.info(f"Attempting to connect to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}")
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, keepalive=60)
            self.logger.debug("MQTT connect() method completed without exception")
        except Exception as e:
            self.logger.error(f"MQTT Connection failed with exception: {e}")
            self.logger.error(f"Exception type: {type(e)}")

        self.logger.debug("MySender initialization completed")

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
        self.logger.info(f"MQTT CONNECTION STATUS: {status} (code={rc})")

        if rc == 0:
            self.logger.info(f"Connected to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}")
            # Subscribe to topic
            result, mid = self.mqtt_client.subscribe(self.mqtt_topicname)
            self.logger.debug(f"MQTT Subscribe result: {result}, Message ID: {mid}")
            if result == 0:
                self.logger.info(f"Subscribed to MQTT topic: {self.mqtt_topicname}")
            else:
                self.logger.error(f"Failed to subscribe to MQTT topic: {self.mqtt_topicname}")
        else:
            self.logger.error(f"MQTT connection failed: {status}")

    def _on_disconnect(self, client, userdata, rc):
        """Callback for when the client disconnects from the broker"""
        if rc == 0:
            self.logger.info("Disconnected from broker cleanly")
        else:
            self.logger.warning(f"Unexpected disconnect with code {rc}")

    def start_mqtt_loop(self):
        """Start MQTT loop in a separate thread."""
        self.logger.debug("Starting MQTT client loop")
        self.mqtt_client.loop_start()
        self.logger.debug("MQTT loop started")

    def _on_message_sync(self, client, userdata, message):
        """Synchronous wrapper for MQTT on_message to run handle_data in the main event loop."""
        self.logger.debug(f"Message received on topic: {message.topic}")
        asyncio.run_coroutine_threadsafe(self.handle_data(client, userdata, message), self.event_loop)

    async def handle_data(self, client, userdata, message):
        """Handle incoming MQTT data and put it on the async queue."""
        event = message.payload
        await self.put_queue(event)
        self.logger.debug(f"Processed message from topic '{message.topic}' and queued for transmission")

    async def run(self, number_of_iterations=-1):
        self.logger.debug("MySender.run() method started")
        self.start_mqtt_loop()
        self.logger.debug("MQTT loop started, now waiting in run loop")
        try:
            while True:
                await asyncio.sleep(10)  # Keep the loop running, check every 10 seconds
                self.logger.debug("Still running in the MySender.run() loop")
        except asyncio.CancelledError:
            self.logger.debug("CancelledError caught, stopping MQTT loop")
            self.mqtt_client.loop_stop()
            self.logger.debug("MQTT loop stopped")
        except Exception as e:
            self.logger.error(f"Unexpected exception in MySender.run(): {e}")
            raise


async def main(config):
    logger = logging.getLogger("main")
    logger.debug("main() function started")
    loop = asyncio.get_running_loop()  # Capture the main event loop
    clitool = pytak.CLITool(config["mycottool"])
    await clitool.setup()
    logger.debug("Adding MySender task to CLITool")
    clitool.add_task(MySender(clitool.tx_queue, config, loop))  # Pass the loop
    logger.debug("Running CLITool")
    await clitool.run()


def run_main_in_process(config):
    logger = logging.getLogger("process")
    logger.debug("run_main_in_process() started")
    loop = asyncio.get_event_loop()
    logger.debug("Event loop created, running main()")
    loop.run_until_complete(main(config))


if __name__ == "__main__":
    logger = logging.getLogger("startup")
    logger.info("Script main block executing")

    parser = argparse.ArgumentParser(description="TAK Publisher Script")
    parser.add_argument('--config_file_path', type=str, required=True, help='Path to the config YAML file.')
    parser.add_argument('--creds_path', type=str, required=True, help='Path to the creds directory.')

    args = parser.parse_args()
    logger.info(f"Args parsed - config_file_path: {args.config_file_path}, creds_path: {args.creds_path}")

    # Load the YAML configuration
    try:
        with open(args.config_file_path, 'r') as file:
            logger.info(f"Loading configuration from {args.config_file_path}")
            config_data = yaml.safe_load(file)
            logger.info("Configuration loaded successfully")

            # Setup logger based on config
            log_level = config_data.get('logging', {}).get('level', 'INFO')
            logger = setup_logger(log_level)
            logger.info(f"Logger configured with level: {log_level}")

    except Exception as e:
        logger.error(f"Failed to load configuration: {e}")
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

        logger.info(f"MQTT CONFIG: Broker={mqtt_broker}, Port={mqtt_port}, Topic={mqtt_topicname}")
    except KeyError as e:
        logger.error(f"Missing required configuration key: {e}")
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
    logger.info("Starting TAK publisher process")
    process = multiprocessing.Process(target=run_main_in_process, args=(config,))
    process.start()

    logger.info("Main() is now running in a separate process")
    process.join()
    logger.info("Process completed")