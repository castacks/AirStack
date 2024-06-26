import time
from pymavlink import mavutil
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("message", default="HEARTBEAT")

args = parser.parse_args()

# Connect to the MAVLink network
mavlink_connection = mavutil.mavlink_connection('udpin:localhost:14553')  # Adjust as necessary

message_type = args.message  # Replace with the specific message type you are interested in
message_interval = []  # To store time intervals between messages

last_timestamp = None

# Receive messages and log timestamps
while True:
    message = mavlink_connection.recv_match(type=message_type, blocking=True)
    if message:
        current_timestamp = time.time()
        if last_timestamp is not None:
            interval = current_timestamp - last_timestamp
            message_interval.append(interval)
            print(f"Interval: {interval} seconds")
        last_timestamp = current_timestamp

    # Calculate and print average rate every 10 messages
    if len(message_interval) >= 10:
        average_interval = sum(message_interval) / len(message_interval)
        message_rate = 1 / average_interval
        print(f"Average Rate: {message_rate} messages per second")
        message_interval = []  # Reset for next batch

