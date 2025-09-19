#!/usr/bin/env python3

"""
ROS2 Python Launch file for MAVROS interface with connection polling.
This launch file continuously polls for a mavlink connection using the 
fcu_url launch argument and launches px4.launch once the 
connection is established.
"""

import os
import time
import socket
import threading
from urllib.parse import urlparse
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import AnyLaunchDescriptionSource


class MavlinkConnectionChecker:
    """Class to handle mavlink connection checking."""
    
    def __init__(self):
        self.connection_established = False
        self.fcu_url = None
        self.check_interval = 1.0  # seconds
        
    def parse_fcu_url(self, fcu_url):
        """Parse the FCU URL to extract connection details."""
        if fcu_url.startswith('udp://'):
            # Parse UDP URL: udp://host:port or udp://:port@remote_host:remote_port
            if '@' in fcu_url:
                # Format: udp://:local_port@remote_host:remote_port
                local_part, remote_part = fcu_url.replace('udp://', '').split('@')
                if ':' in local_part:
                    local_host, local_port = local_part.split(':')
                    if not local_host:
                        local_host = 'localhost'
                else:
                    local_port = local_part
                    local_host = 'localhost'
                
                remote_host, remote_port = remote_part.split(':')
                return 'udp', remote_host, int(remote_port)
            else:
                # Format: udp://host:port
                url_parts = urlparse(fcu_url)
                return 'udp', url_parts.hostname, url_parts.port
                
        elif fcu_url.startswith('tcp://') or fcu_url.startswith('tcpin:'):
            # Parse TCP URL: tcp://host:port or tcpin:host:port
            if fcu_url.startswith('tcpin:'):
                parts = fcu_url.replace('tcpin:', '').split(':')
                host = parts[0] if parts[0] else 'localhost'
                port = int(parts[1]) if len(parts) > 1 else 5760
            else:
                url_parts = urlparse(fcu_url)
                host = url_parts.hostname or 'localhost'
                port = url_parts.port or 5760
            return 'tcp', host, port
            
        elif fcu_url.startswith('/dev/'):
            # Serial connection - assume it's always available
            return 'serial', fcu_url, None
            
        else:
            raise ValueError(f"Unsupported FCU URL format: {fcu_url}")
    
    def check_tcp_connection(self, host, port, timeout=1.0):
        """Check if a TCP connection can be established."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(timeout)
            result = sock.connect_ex((host, port))
            sock.close()
            return result == 0
        except Exception:
            return False
    
    def check_udp_connection(self, host, port, timeout=1.0):
        """Check if UDP port is reachable (basic check)."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(timeout)
            # Send a small test packet to see if port is reachable
            sock.sendto(b'test', (host, port))
            sock.close()
            return True
        except Exception:
            return False
    
    def check_serial_connection(self, device_path):
        """Check if serial device exists and is accessible."""
        return os.path.exists(device_path) and os.access(device_path, os.R_OK | os.W_OK)
    
    def check_connection(self, fcu_url):
        """Check if the mavlink connection is available."""
        try:
            conn_type, host_or_path, port = self.parse_fcu_url(fcu_url)
            
            if conn_type == 'tcp':
                return self.check_tcp_connection(host_or_path, port)
            elif conn_type == 'udp':
                return self.check_udp_connection(host_or_path, port)
            elif conn_type == 'serial':
                return self.check_serial_connection(host_or_path)
            else:
                return False
                
        except Exception as e:
            print(f"Error checking connection: {e}")
            return False
    
    def poll_connection(self, fcu_url):
        """Continuously poll for connection until it's established."""
        print(f"Polling for mavlink connection to: {fcu_url}")
        
        while not self.connection_established:
            if self.check_connection(fcu_url):
                print(f"Mavlink connection established to: {fcu_url}")
                self.connection_established = True
                self.fcu_url = fcu_url
                break
            else:
                print(f"Waiting for mavlink connection to: {fcu_url}")
                time.sleep(self.check_interval)


def wait_for_connection_and_launch(context):
    """Wait for mavlink connection and then launch px4.launch."""
    
    # Get the FCU URL from launch configuration
    fcu_url = LaunchConfiguration('fcu_url').perform(context)
    
    if not fcu_url:
        print("ERROR: fcu_url launch argument is not set!")
        return []
    
    # Get launch configurations from context
    max_wait_time = float(LaunchConfiguration('max_wait_time').perform(context))
    
    print(f"Starting mavlink connection polling for: {fcu_url}")
    print(f"Max wait time: {max_wait_time if max_wait_time >= 0 else 'infinite'} seconds")

    # Create connection checker and poll for connection
    checker = MavlinkConnectionChecker()
    
    # Run connection polling in a separate thread to avoid blocking the launch
    def poll_thread():
        checker.poll_connection(fcu_url)
    
    # Start polling thread
    polling_thread = threading.Thread(target=poll_thread, daemon=True)
    polling_thread.start()
    
    # Wait for connection to be established (with timeout)
    start_time = time.time()

    while not checker.connection_established and (max_wait_time < 0 or (time.time() - start_time) < max_wait_time):
        time.sleep(0.1)
    
    if not checker.connection_established:
        print(f"ERROR: Timeout waiting for mavlink connection to: {fcu_url}")
        return []
    
    print("Connection established! Launching MAVROS px4.launch...")
    
    # Create the MAVROS launch action
    mavros_launch = [
        PushRosNamespace('interface'),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                FindPackageShare('mavros'), '/launch/px4.launch'
            ]),
            launch_arguments={
                'fcu_url': fcu_url,
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
    
    polling_interval_arg = DeclareLaunchArgument(
        'polling_interval',
        default_value='1.0',
        description='Interval in seconds for polling mavlink connection'
    )
    
    max_wait_time_arg = DeclareLaunchArgument(
        'max_wait_time',
        default_value='-1.0',
        description='Maximum time in seconds to wait for mavlink connection (default is -1.0 for infinite polling)'
    )
    
    # Use OpaqueFunction to handle the connection polling and launch
    connection_and_launch = OpaqueFunction(function=wait_for_connection_and_launch)
    
    return LaunchDescription([
        # Launch arguments
        fcu_url_arg,
        polling_interval_arg,
        max_wait_time_arg,
        
        # Connection polling and MAVROS launch
        connection_and_launch,
    ])
