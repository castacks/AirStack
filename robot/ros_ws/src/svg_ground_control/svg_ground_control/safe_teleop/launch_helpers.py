"""Shared pieces for the launch files that start gamepad teleop.

Used by launch/teleop.launch.py (the normal way: teleop in its own terminal,
started and checked before the commander) and by launch/ground_control.launch.py
(``use_teleop:=true`` to bundle it with the commander). Both resolve the same
three things from the config YAML + launch arguments:

* which drone is hand-flown  (``drone:=`` or the first of ``teleop_drones``),
* which device flies it      (``teleop_controller:=`` or the config's
                              ``safe_teleop.teleop_controller``, else default),
* the nodes to start for it  (the profile's driver(s) + one ``safe_teleop``).
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

import yaml
from launch.actions import LogInfo
from launch_ros.actions import Node

from .controllers import DEFAULT_CONTROLLER, ControllerProfile, get_controller


def node_params(config_path: str, node_name: str) -> Dict[str, Any]:
    """``ros__parameters`` of one node block in the config YAML ({} if absent)."""
    try:
        with open(config_path) as f:
            doc = yaml.safe_load(f) or {}
    except OSError:
        return {}
    block = doc.get(node_name) or {}
    return block.get('ros__parameters') or {}


def name_list(value) -> List[str]:
    """A comma-separated string or a YAML list as a clean list of names."""
    if isinstance(value, (list, tuple)):
        return [str(v).strip() for v in value if str(v).strip()]
    return [n.strip() for n in str(value or '').split(',') if n.strip()]


def config_teleop_drones(config_path: str, override: str = '') -> List[str]:
    """``teleop_drones`` from the launch override, else the config's commander block."""
    return name_list(override or node_params(config_path, 'swarm_commander')
                     .get('teleop_drones', ''))


def resolve_controller(config_path: str, override: str = '') -> ControllerProfile:
    """The ControllerProfile for this run; KeyError names the supported ones."""
    name = (override.strip()
            or str(node_params(config_path, 'safe_teleop')
                   .get('teleop_controller', '')).strip()
            or DEFAULT_CONTROLLER)
    return get_controller(name)


def teleop_actions(config_path: str, drone: str, profile: ControllerProfile,
                   extra_teleop_params: Optional[Dict[str, Any]] = None,
                   others: Optional[List[str]] = None) -> list:
    """Driver node(s) + safe_teleop for ``drone`` using ``profile``.

    One physical device flies one drone. ``others`` are further teleop drones
    in the config that this launch is NOT driving; they are named in a log
    line so nobody waits for a second pad that was never started.
    """
    actions = [LogInfo(msg=f'teleop: {profile.name} ({profile.description}) '
                           f'-> {drone}')]
    if others:
        actions.append(LogInfo(
            msg=f'teleop: only {drone} gets the {profile.name} controller; '
                f'{", ".join(others)} need their own device (launch '
                f'teleop.launch.py again with drone:=<name>)'))
    for driver in profile.drivers:
        actions.append(Node(
            package=driver.package, executable=driver.executable,
            name=driver.name, output='screen',
            # Profile defaults first, so a matching block in the config
            # (e.g. joy_node: {ros__parameters: {device_id: 1}}) wins.
            parameters=[dict(driver.parameters), config_path],
        ))
    params: Dict[str, Any] = {'drone': drone, 'teleop_controller': profile.name}
    params.update(extra_teleop_params or {})
    actions.append(Node(
        package='svg_ground_control', executable='safe_teleop',
        name='safe_teleop', output='screen',
        parameters=[config_path, params],
    ))
    return actions
