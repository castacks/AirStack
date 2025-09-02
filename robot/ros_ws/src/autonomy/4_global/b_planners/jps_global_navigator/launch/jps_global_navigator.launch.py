from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('jps_global_navigator')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'jps_params.yaml'),
        description='Path to the configuration file'
    )

    cost_map_topic_arg = DeclareLaunchArgument(
        'cost_map_topic',
        default_value='/cost_map',
        description='Cost map topic name'
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Odometry topic name'
    )

    global_plan_topic_arg = DeclareLaunchArgument(
        'global_plan_topic',
        default_value='/global_plan',
        description='global_plan topic name'
    )

    enable_visualization_arg = DeclareLaunchArgument(
        'enable_debug_visualization',
        default_value='true',
        description='Enable debug visualization'
    )

    # JPS Global Navigator node
    jps_navigator_node = Node(
        package='jps_global_navigator',
        executable='jps_global_navigator_node',
        name='jps_global_navigator',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'cost_map_topic': LaunchConfiguration('cost_map_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                "global_plan_topic": LaunchConfiguration('global_plan_topic'),
                'enable_debug_visualization': LaunchConfiguration('enable_debug_visualization'),
            }
        ],
        remappings=[
            ('cost_map', LaunchConfiguration('cost_map_topic')),
            ('odom', LaunchConfiguration('odom_topic')),
            ('global_plan', LaunchConfiguration('global_plan_topic')),
        ]
    )

    return LaunchDescription([
        config_file_arg,
        cost_map_topic_arg,
        odom_topic_arg,
        enable_visualization_arg,
        jps_navigator_node,
    ])
