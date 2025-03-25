from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('bag_record_pid')
    
    # Declare all launch arguments
    cfg_path_arg = DeclareLaunchArgument(
        'cfg_path',
        default_value=os.path.join(pkg_dir, 'config', 'cfg.yaml'),
        description='Configuration file for bag record pid'
    )
    
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/logging',
        description='Logging directory'
    )
    
    mcap_qos_dir_arg = DeclareLaunchArgument(
        'mcap_qos_dir',
        default_value=os.path.join(pkg_dir, 'config'),
        description='MCAP QoS directory'
    )

    # Create the node with launch configurations
    bag_record_node = Node(
        package='bag_record_pid',
        executable='bag_record_node',
        name='bag_record_pid',
        parameters=[{
            'cfg_path': LaunchConfiguration('cfg_path'),
            'output_dir': LaunchConfiguration('output_dir'),
            'mcap_qos_dir': LaunchConfiguration('mcap_qos_dir')
        }],
        output='screen'
    )

    return LaunchDescription([
        # Add all argument declarations
        cfg_path_arg,
        output_dir_arg,
        mcap_qos_dir_arg,
        # Add the node
        bag_record_node
    ])

