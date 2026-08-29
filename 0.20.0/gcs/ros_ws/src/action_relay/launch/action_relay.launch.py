"""Launch one action_relay node per robot.

Robot roster resolution (first match wins):

1. ROBOT_RELAY_MAP="robot_1:1,robot_2:2,..." — explicit override for custom
   robot_name -> domain mappings.
2. FLEET_CONFIG_FILE (set by ``airstack up --fleet <name>``) — the fleet file
   names the robots; ``network.domain_policy: auto`` assigns robot N (1-based
   file order) -> domain N, the same rule as tools/fleet/resolve_fleet.py.
3. Legacy fallback: NUM_ROBOTS (default 1) -> robot_1..robot_N with domain
   IDs 1..N, matching default_robot_name_map.yaml.
"""

import os
import sys

from launch import LaunchDescription
from launch_ros.actions import Node


def _fleet_robots(fleet_config_file):
    """(robot_name, domain) pairs from a fleet file (RFC #380).

    Prefers importing the canonical resolver from the checkout mounted at
    ``<root>/tools/fleet`` (the fleet file lives at
    ``<root>/config/fleets/<name>.yaml``, so ``<root>`` is derived from the
    file's own path — /root/AirStack inside the GCS container). If that mount
    is missing, falls back to reading the YAML directly with the same
    ``domain_policy: auto`` rule (robot N, 1-based file order -> domain N).
    """
    fleet_path = os.path.abspath(fleet_config_file)
    root = os.path.dirname(os.path.dirname(os.path.dirname(fleet_path)))
    tools_fleet = os.path.join(root, 'tools', 'fleet')
    if tools_fleet not in sys.path:
        sys.path.insert(0, tools_fleet)
    try:
        from resolve_fleet import load_fleet  # single source of fleet parsing
        fleet = load_fleet(fleet_path)
    except ImportError:
        # tools/fleet isn't mounted in this container — parse the YAML inline
        # (same schema, same auto domain rule; keep the two in sync).
        import yaml
        with open(fleet_path, encoding='utf-8') as f:
            fleet = yaml.safe_load(f) or {}
    robots = fleet.get('robots') or {}
    if not robots:
        raise ValueError(f'fleet file {fleet_path} has no robots:')
    return [(name, i) for i, name in enumerate(robots, start=1)]


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
    fleet_config_file = os.environ.get('FLEET_CONFIG_FILE', '').strip()
    if fleet_config_file and os.path.isfile(fleet_config_file):
        return _fleet_robots(fleet_config_file)
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
