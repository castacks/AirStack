#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value=os.environ.get('ROBOT_NAME', 'robot_1'),
        description='Name of the robot'
    )
    
    # Get the robot name
    robot_name = LaunchConfiguration('robot_name')
    
    # Path to the URDF file
    urdf_file = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'urdf',
        'robot.urdf.xacro'
    ])
    
    # Robot description command
    robot_description_content = Command([
        'xacro ', urdf_file, ' robot_name:=', robot_name
    ])
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'publish_frequency': 50.0,
            'use_sim_time': True  # Enable simulation time for proper TF synchronization
        }],
        output='screen'
    )
    
    return LaunchDescription([
        robot_name_arg,
        robot_state_publisher_node
    ])
