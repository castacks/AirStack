#!/usr/bin/env python3

"""
ROS2 Python Launch file for the interface system.
Converted from interface.launch.xml to enable programmatic calculation of mavlink FCU URL.
The FCU URL calculation follows the same logic as px4_mavlink_backend.py in Pegasus simulator.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource


def create_mavros_groups(context):
    """Function to create MAVROS launch groups with calculated FCU URLs."""
    
    # Get the robot_id from environment variable
    robot_id_str = os.environ.get('ROS_DOMAIN_ID', '0')
    robot_id = int(robot_id_str)
    
    # Calculate FCU URL based on robot_id (following px4_mavlink_backend.py logic)
    # Connection format: connection_type:connection_ip:connection_port
    # Where connection_port = connection_baseport + vehicle_id
    connection_type = "tcpin"
    connection_ip = "localhost" 
    connection_baseport = 4560
    sim_fcu_url = f"{connection_type}:{connection_ip}:{connection_baseport + robot_id}"
    
    # Real hardware FCU URL (unchanged from original)
    real_fcu_url = '/dev/ttyTHS4:115200'
    
    sim = LaunchConfiguration('sim')
    
    # MAVROS group for simulation
    mavros_sim_group = GroupAction(
        condition=IfCondition(sim),
        actions=[
            PushRosNamespace('interface'),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource([
                    FindPackageShare('mavros'), '/launch/px4.launch'
                ]),
                launch_arguments={
                    'fcu_url': sim_fcu_url,
                }.items()
            )
        ]
    )
    
    # MAVROS group for real hardware
    mavros_real_group = GroupAction(
        condition=UnlessCondition(sim),
        actions=[
            PushRosNamespace('interface'),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource([
                    FindPackageShare('mavros'), '/launch/px4.launch'
                ]),
                launch_arguments={
                    'fcu_url': real_fcu_url,
                }.items()
            )
        ]
    )
    
    return [mavros_sim_group, mavros_real_group]


def generate_launch_description():
    """Generate the launch description for the interface system."""
    
    # Declare launch arguments
    interface_odometry_in_topic_arg = DeclareLaunchArgument(
        'interface_odometry_in_topic',
        default_value=[EnvironmentVariable('ROBOT_NAME'), '/interface/mavros/local_position/odom'],
        description='Input odometry topic for the interface'
    )
    
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Whether running in simulation mode'
    )
    
    # Get launch configurations
    interface_odometry_in_topic = LaunchConfiguration('interface_odometry_in_topic')
    robot_name = EnvironmentVariable('ROBOT_NAME')
    
    # Robot interface node
    robot_interface_node = Node(
        package='robot_interface',
        executable='robot_interface_node',
        namespace='interface',
        output='screen',
        parameters=[{
            'interface': 'mavros_interface::MAVROSInterface',
            'is_ardupilot': False,
            'post_takeoff_command_delay_time': 10.0,
            'do_global_pose_command': False,
        }],
        remappings=[
            ('ardupilot_takeoff', [robot_name, '/takeoff_landing_planner/ardupilot_takeoff']),
            ('reset_integrators', [robot_name, '/control/reset_integrators']),
        ]
    )
    
    # Position setpoint publisher node
    position_setpoint_node = Node(
        package='mavros_interface',
        executable='position_setpoint_pub.py',
        namespace='interface',
        output='screen',
        parameters=[{
            'command_type': 2,
            'max_velocity': 3.0,
            'target_frame': 'base_link',
            'publish_goal': False,
        }]
    )
    
    # Odometry conversion node
    odometry_conversion_node = Node(
        package='robot_interface',
        namespace='odometry_conversion',
        executable='odometry_conversion',
        output='screen',
        parameters=[{
            'odom_input_qos_is_best_effort': True,
            'new_frame_id': 'map',
            'new_child_frame_id': 'base_link',
            'odometry_output_type': 1,
            'convert_odometry_to_transform': True,
            'convert_odometry_to_stabilized_transform': True,
            'restamp_now_post': False,
        }],
        remappings=[
            ('odometry_in', interface_odometry_in_topic),
            ('odometry_out', 'odometry'),
        ]
    )
    
    # Drone safety monitor node
    drone_safety_monitor_node = Node(
        package='drone_safety_monitor',
        executable='drone_safety_monitor',
        namespace='drone_safety_monitor',
        output='screen',
        parameters=[{
            'state_estimate_timeout': 1.0,
        }],
        remappings=[
            ('state_estimate', [robot_name, '/odometry_conversion/odometry']),
        ]
    )
    
    # Use OpaqueFunction to handle the dynamic MAVROS group creation
    mavros_opaque_function = OpaqueFunction(function=create_mavros_groups)
    
    # Create and return launch description
    return LaunchDescription([
        # Launch arguments
        interface_odometry_in_topic_arg,
        sim_arg,
        
        # MAVROS groups (dynamically created)
        mavros_opaque_function,
        
        # Nodes
        robot_interface_node,
        position_setpoint_node,
        odometry_conversion_node,
        drone_safety_monitor_node,
    ])
