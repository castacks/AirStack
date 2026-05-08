"""Launch one action_relay node per robot.

Defaults to robot_1..robot_N with domain IDs 1..N (NUM_ROBOTS env, default 1).
Override with ROBOT_RELAY_MAP="robot_1:1,robot_2:2,..." when the
robot_name -> domain mapping in default_robot_name_map.yaml has been customised.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def _parse_robots():
    override = os.environ.get('ROBOT_RELAY_MAP', '').strip()
    if override:
        out = []
        for entry in override.split(','):
            name, _, domain = entry.strip().partition(':')
            if not name or not domain:
                raise ValueError(f"ROBOT_RELAY_MAP entry '{entry}' must be name:domain")
            out.append((name, int(domain)))
        return out
    n = int(os.environ.get('NUM_ROBOTS', '1'))
    return [(f'robot_{i}', i) for i in range(1, n + 1)]


def generate_launch_description():
    nodes = []
    for name, domain in _parse_robots():
        nodes.append(Node(
            package='action_relay',
            executable='action_relay_node',
            name=f'action_relay_{name}',
            output='screen',
            parameters=[{'robot_name': name, 'robot_domain': domain}],
        ))
    return LaunchDescription(nodes)
