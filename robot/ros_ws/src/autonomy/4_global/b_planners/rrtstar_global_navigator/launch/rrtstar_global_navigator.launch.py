#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    rrt_max_iterations_arg = DeclareLaunchArgument(
        'rrt_max_iterations',
        default_value='5000',
        description='Maximum iterations for RRT* algorithm'
    )
    
    rrt_step_size_arg = DeclareLaunchArgument(
        'rrt_step_size',
        default_value='0.5',
        description='Step size for RRT* algorithm'
    )
    
    rrt_goal_tolerance_arg = DeclareLaunchArgument(
        'rrt_goal_tolerance',
        default_value='3.0',
        description='Goal tolerance for RRT* algorithm'
    )
    
    rrt_rewire_radius_arg = DeclareLaunchArgument(
        'rrt_rewire_radius',
        default_value='1.0',
        description='Rewire radius for RRT* algorithm'
    )
    
    cost_map_topic_arg = DeclareLaunchArgument(
        'cost_map_topic',
        default_value='/cost_map',
        description='Topic name for cost map PointCloud2'
    )
    
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Topic name for odometry'
    )

    # Create the node
    rrtstar_global_navigator_node = Node(
        package='rrtstar_global_navigator',
        executable='rrtstar_global_navigator_node',
        name='rrtstar_global_navigator',
        output='screen',
        parameters=[{
            'rrt_max_iterations': LaunchConfiguration('rrt_max_iterations'),
            'rrt_step_size': LaunchConfiguration('rrt_step_size'),
            'rrt_goal_tolerance': LaunchConfiguration('rrt_goal_tolerance'),
            'rrt_rewire_radius': LaunchConfiguration('rrt_rewire_radius'),
            'cost_map_topic': LaunchConfiguration('cost_map_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
        }],
        remappings=[
            ('rrtstar_navigator', '/rrtstar_navigator'),
            ('global_plan', '/global_plan'),
        ]
    )

    return LaunchDescription([
        rrt_max_iterations_arg,
        rrt_step_size_arg,
        rrt_goal_tolerance_arg,
        rrt_rewire_radius_arg,
        cost_map_topic_arg,
        odom_topic_arg,
        rrtstar_global_navigator_node,
    ])