"""Launch one action_relay node per robot.

Reads NUM_ROBOTS from the environment (default 1).
Robot names are robot_1 .. robot_N with domain IDs 1 .. N.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    num_robots = int(os.environ.get('NUM_ROBOTS', '1'))

    nodes = []
    for i in range(1, num_robots + 1):
        nodes.append(Node(
            package='action_relay',
            executable='action_relay_node',
            name=f'action_relay_robot_{i}',
            output='screen',
            parameters=[{
                'robot_name': f'robot_{i}',
                'robot_domain': i,
            }],
        ))

    return LaunchDescription(nodes)
