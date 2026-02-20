#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Setup function to handle conditional robot description logic."""
    # Get launch configurations
    robot_name = LaunchConfiguration('robot_name')
    robot_description_param = LaunchConfiguration('robot_description')
    urdf_file_path = LaunchConfiguration('urdf_file_path')
    publish_frequency = LaunchConfiguration('publish_frequency')
    ignore_timestamp = LaunchConfiguration('ignore_timestamp')
    frame_prefix = LaunchConfiguration('frame_prefix')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Get the actual value of robot_description parameter
    robot_description_value = robot_description_param.perform(context)
    
    # Get the actual urdf_file_path value for debugging
    urdf_file_path_value = urdf_file_path.perform(context)
    
    # Determine robot description source
    if robot_description_value:
        # Use provided robot description directly
        robot_description_content = robot_description_param
    else:
        # Generate from xacro file using parameterized path
        
        if os.path.isabs(urdf_file_path_value):
            # Use absolute path directly
            urdf_file = urdf_file_path_value
        elif '/' in urdf_file_path_value:
            # Path format: "package_name/urdf/file.urdf"
            # Split to get package name and the rest of the path
            parts = urdf_file_path_value.split('/', 1)
            package_name = parts[0]
            relative_path = parts[1]
            
            
            # Construct path using FindPackageShare with the package name
            urdf_file = PathJoinSubstitution([
                FindPackageShare(package_name),
                relative_path
            ])
        else:
            # Use relative path within robot_bringup package
            urdf_file = PathJoinSubstitution([
                FindPackageShare('robot_bringup'),
                'urdf',
                urdf_file_path
            ])
        
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
            'publish_frequency': publish_frequency,
            'ignore_timestamp': ignore_timestamp,
            'frame_prefix': frame_prefix,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    return [robot_state_publisher_node]


def generate_launch_description():
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value=os.environ.get('ROBOT_NAME', 'robot_1'),
        description='Name of the robot'
    )
    
    robot_description_arg = DeclareLaunchArgument(
        'robot_description',
        default_value='',
        description='The robot description in URDF form. If empty, will use the default URDF file.'
    )
    
    urdf_file_path_arg = DeclareLaunchArgument(
        'urdf_file_path',
        default_value='robot.urdf.xacro',
        description='Path to the URDF/xacro file. Can be relative to robot_bringup/urdf/ or an absolute path'
    )
    
    publish_frequency_arg = DeclareLaunchArgument(
        'publish_frequency',
        default_value='20.0',
        description='The maximum frequency at which non-static transforms will be published to /tf'
    )
    
    ignore_timestamp_arg = DeclareLaunchArgument(
        'ignore_timestamp',
        default_value='false',
        description='Whether to accept all joint states no matter what the timestamp'
    )
    
    frame_prefix_arg = DeclareLaunchArgument(
        'frame_prefix',
        default_value='',
        description='An arbitrary prefix to add to the published tf2 frames'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Enable simulation time for proper TF synchronization'
    )

    return LaunchDescription([
        robot_name_arg,
        robot_description_arg,
        urdf_file_path_arg,
        publish_frequency_arg,
        ignore_timestamp_arg,
        frame_prefix_arg,
        use_sim_time_arg,
        OpaqueFunction(function=launch_setup)
    ])