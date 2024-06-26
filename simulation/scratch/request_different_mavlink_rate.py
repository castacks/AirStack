from pymavlink import mavutil

# Connect to the MAVLink network
master = mavutil.mavlink_connection('udpin:localhost:14553')  # Adjust as necessary

# Wait for the first heartbeat 
# (to find the system and component IDs)
master.wait_heartbeat()

# Define message type and desired interval
message_id = mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED  # Replace with the desired message ID
interval_us = 1/60 * 1e6  # Interval in microseconds (1 second in this case)

# Send command to set message interval
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0,  # Confirmation
    message_id,  # The MAVLink message ID
    interval_us,  # Interval in microseconds
    0, 0, 0, 0, 0  # Unused parameters
)

print(f"Requested message interval for message ID {message_id} set to {interval_us / 1e6} seconds.")

# Continue receiving messages to see the effect
while True:
    message = master.recv_match(blocking=True)
    if message:
        print(message)

