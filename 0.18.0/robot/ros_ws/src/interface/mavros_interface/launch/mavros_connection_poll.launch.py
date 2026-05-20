#!/usr/bin/env python3

"""
ROS2 Python Launch file for MAVROS interface with connection polling.
This launch file continuously polls for a mavlink connection using pymavlink
to check for heartbeat messages and launches px4.launch once the 
connection is established.
"""

import time
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import AnyLaunchDescriptionSource

from pymavlink import mavutil


def wait_for_connection_and_launch(context):
    """Wait for mavlink connection and then launch px4.launch."""
    
    # Get the MAVROS FCU URL from launch configuration
    fcu_url = LaunchConfiguration('fcu_url').perform(context)
    
    if not fcu_url:
        print("ERROR: fcu_url launch argument is not set!")
        return []
    
    # Get launch configurations from context
    heartbeat_timeout = float(LaunchConfiguration('heartbeat_timeout').perform(context))

    mavlink_url = fcu_url.split("@")[0].replace("udp://", "udpin:")  + fcu_url.split("@")[1] if "@" in fcu_url else ""
    
    print(f"Starting mavlink heartbeat polling for: {mavlink_url}")
    if heartbeat_timeout < 0:
        print(f"Heartbeat timeout: infinite (will wait indefinitely)")
    else:
        print(f"Heartbeat timeout: {heartbeat_timeout} seconds")

    # Poll for heartbeat with single timeout loop
    start_time = time.time()
    print(f"Polling for mavlink heartbeat from: {mavlink_url}")

    connection = None
    tgt_system, tgt_component = None, None


    while heartbeat_timeout < 0 or (time.time() - start_time) < heartbeat_timeout:
        try:
            # Create connection once
            if connection is None:
                connection = mavutil.mavlink_connection(mavlink_url)
            
            msg = connection.recv_match(type='HEARTBEAT', blocking=False, timeout=1)

            if msg:
                # Check the message type to identify a heartbeat
                if msg.get_type() == 'HEARTBEAT':
                    print("Heartbeat message received!")
                    print(f"MAV_TYPE: {msg.type}, MAV_AUTOPILOT: {msg.autopilot}")

                    print("Requesting autopilot version...")
                    connection.mav.command_long_send(
                        connection.target_system,  # Target system
                        connection.target_component, # Target component
                        mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, # Command ID
                        0,  # Confirmation
                        0,  # Parameter 1
                        0,  # Parameter 2
                        0,  # Parameter 3
                        0,  # Parameter 4
                        0,  # Parameter 5
                        0,  # Parameter 6
                        0   # Parameter 7
                    )

                    # wait for a response and print it
                    msg = connection.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=10)
                    if msg:
                        print("Autopilot version message received!")
                        print(msg)
                    else:
                        print("No autopilot version message received within timeout.")
                        exit(1)

                    tgt_system = connection.target_system
                    tgt_component = connection.target_component
                    print(f"Target System: {tgt_system}, Target Component: {tgt_component}")

                    connection.close()
                    break
            print(f"Waiting for mavlink heartbeat from: {mavlink_url}")
        except Exception as e:
            print(f"Waiting for connection {time.time() - start_time:.1f}s")
        time.sleep(1)

    print(f"Heartbeat received! Launching MAVROS px4.launch on {fcu_url}")

    # Create the MAVROS launch action
    mavros_launch = [
        PushRosNamespace('interface'),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                FindPackageShare('mavros'), '/launch/px4.launch'
            ]),
            launch_arguments={
                'fcu_url': fcu_url,
                # "tgt_system": str(tgt_system),
                # "tgt_component": str(tgt_component)
            }.items()
        )
    ]
    
    return mavros_launch


def generate_launch_description():
    """Generate the launch description."""
    
    # Declare launch arguments
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='',
        description='FCU connection URL (e.g., udp://localhost:14540, tcp://localhost:5760, /dev/ttyUSB0:57600)'
    )
    
    heartbeat_timeout_arg = DeclareLaunchArgument(
        'heartbeat_timeout',
        default_value='-1.0',
        description='Timeout in seconds to wait for mavlink heartbeat (use negative value for infinite waiting)'
    )
    
    # Use OpaqueFunction to handle the connection polling and launch
    connection_and_launch = OpaqueFunction(function=wait_for_connection_and_launch)
    
    return LaunchDescription([
        # Launch arguments
        fcu_url_arg,
        heartbeat_timeout_arg,
        
        # Connection polling and MAVROS launch
        connection_and_launch,
    ])
