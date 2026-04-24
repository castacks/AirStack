#!/usr/bin/env python3
"""
Resolve a Docker container name to a robot name and per-robot index using a YAML mapping config.

The YAML key is still spelled `domain_id` for backwards compatibility with existing
configs — under rmw_zenoh_cpp we don't use it as ROS_DOMAIN_ID anymore (Zenoh would
isolate robots if we did), but it's still a unique per-robot integer used for
MAVLink ports, bag filenames, etc.

Usage:
    ./resolve_robot_name.py <container_name> <config_file>

Example:
    ./resolve_robot_name.py docker-robot-1 container-robot-map.yaml

Output (stdout):
    ROBOT_NAME=robot_1
    ROBOT_INDEX=1
"""

import re
import sys
import yaml


def apply_groups(template: str, groups: tuple) -> str:
    """Replace {1}, {2}, ... placeholders in template with regex match groups."""
    result = template
    for i, group in enumerate(groups, start=1):
        result = result.replace(f"{{{i}}}", group)
    return result


def resolve_robot_name(container_name: str, config_path: str) -> tuple[str, str] | None:
    """Return (robot_name, domain_id) for the first matching rule, or None."""
    with open(config_path) as f:
        config = yaml.safe_load(f)

    mappings = config.get("mappings")
    if not mappings:
        print("Error: config file has no 'mappings' key.", file=sys.stderr)
        sys.exit(1)

    for rule in mappings:
        match = re.fullmatch(rule["pattern"], container_name)
        if match:
            robot_name = apply_groups(rule["robot"], match.groups())
            domain_id = apply_groups(str(rule.get("domain_id", "0")), match.groups())
            return robot_name, domain_id

    return None


def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <container_name> <config_file>", file=sys.stderr)
        sys.exit(1)

    container_name = sys.argv[1]
    config_path = sys.argv[2]

    try:
        result = resolve_robot_name(container_name, config_path)
    except FileNotFoundError:
        print(f"Error: config file '{config_path}' not found.", file=sys.stderr)
        sys.exit(1)
    except yaml.YAMLError as e:
        print(f"Error: failed to parse config file: {e}", file=sys.stderr)
        sys.exit(1)

    if result is None:
        print(f"Error: no match found for container '{container_name}'.", file=sys.stderr)
        sys.exit(1)

    robot_name, domain_id = result
    print(f"ROBOT_NAME={robot_name}")
    print(f"ROBOT_INDEX={domain_id}")


if __name__ == "__main__":
    main()