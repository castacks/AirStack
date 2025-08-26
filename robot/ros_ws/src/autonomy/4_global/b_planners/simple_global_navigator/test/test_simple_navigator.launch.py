#!/usr/bin/env python3

"""
Launch file to run the SimpleGlobalNavigator with tests.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('simple_global_navigator')
    
    # Navigator node
    navigator_node = Node(
        package='simple_global_navigator',
        executable='simple_global_navigator_node',
        name='simple_global_navigator',
        parameters=[{
            'max_planning_time': 10.0,
            'max_iterations': 1000,
            'step_size': 0.5,
            'goal_tolerance': 0.5,
            'publish_tree_markers': True,
            'tree_marker_lifetime': 10.0
        }],
        output='screen'
    )
    
    # Test node (delayed start to let navigator initialize)
    test_node = TimerAction(
        period=3.0,  # Wait 3 seconds before starting tests
        actions=[
            Node(
                package='simple_global_navigator',
                executable='run_simple_tests.py',
                name='simple_navigator_tester',
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        navigator_node,
        test_node
    ])