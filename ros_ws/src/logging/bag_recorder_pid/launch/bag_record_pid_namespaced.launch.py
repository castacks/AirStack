from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)

from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the savior_database package
    pkg_dir = get_package_share_directory('bag_record_pid')

    # Add robot_prefix launch argument
    robot_prefix_arg = DeclareLaunchArgument(
        'robot_prefix',
        default_value='robot1',
        description='Prefix/namespace for the robot'
    )

    # Declare all launch arguments
    cfg_path_arg = DeclareLaunchArgument(
        'cfg_path',
        default_value=os.path.join(pkg_dir, 'config', 'cfg.yaml'),
        description='Configuration file for bag record pid'
    )
    
    mcap_qos_dir_arg = DeclareLaunchArgument(
        'mcap_qos_dir',
        default_value=os.path.join(pkg_dir, 'config'),
        description='MCAP QoS directory'
    )
    
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/logging',
        description='Logging directory'
    )

    # Create a group action with namespace
    bag_record_group = GroupAction(
        actions=[
            # Push ROS namespace
            PushRosNamespace(LaunchConfiguration('robot_prefix')),
            # Include the bag_record_pid node launch file
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_dir, 'launch', 'bag_record_pid_node.launch.py')
                ),
                launch_arguments={
                    'cfg_path': LaunchConfiguration('cfg_path'),
                    'output_dir': LaunchConfiguration('output_dir'),
                    'mcap_qos_dir': LaunchConfiguration('mcap_qos_dir')
                }.items()
            )
        ]
    )

    return LaunchDescription([
        # Add robot_prefix argument
        robot_prefix_arg,
        # Add all argument declarations
        cfg_path_arg,
        mcap_qos_dir_arg,
        output_dir_arg,
        # Include the group containing the database node launch file
        bag_record_group
    ])
