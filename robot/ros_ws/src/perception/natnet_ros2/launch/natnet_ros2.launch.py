#!/usr/bin/env python3
"""Bring up the NatNet node from natnet_config.yaml; optionally the MAVROS bridge.

The config uses a custom ``natnet:`` schema (server settings + per-robot profiles),
so this launch file parses it, selects the profile matching ``ROBOT_NAME``, flattens
the body list into node parameters, and — when the robot's ``vision_pose`` block is
enabled — includes the MAVROS GP-origin + vision_pose_converter bridges.

natnet_ros2_node is a C++ executable that requires the OptiTrack NatNet SDK.
If the SDK was not installed (``airstack setup`` not run) and the workspace
has not been rebuilt, launching this file will raise a RuntimeError with
instructions. Set LAUNCH_NATNET=false in .env to disable OptiTrack entirely.
"""

from __future__ import annotations

import os
import re
from pathlib import Path
from typing import Any, cast

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Per-body covariance fallback when a body omits its own (sub-0.1 mm / sub-0.1 deg).
_DEFAULT_POSITION_COVARIANCE = [1.0e-6, 0.0, 0.0, 0.0, 1.0e-6, 0.0, 0.0, 0.0, 1.0e-6]
_DEFAULT_ORIENTATION_COVARIANCE = [3.0e-6, 0.0, 0.0, 0.0, 3.0e-6, 0.0, 0.0, 0.0, 3.0e-6]

_ENV_SUBST = re.compile(r"\$\(env\s+(\w+)(?:\s+([^)]*))?\)")


def _expand_env(value: Any) -> Any:
    """Expand ``$(env VAR default)`` tokens in a string using os.environ."""
    if not isinstance(value, str):
        return value

    def _replace(match: re.Match) -> str:
        var, default = match.group(1), match.group(2)
        return os.environ.get(var, default if default is not None else "")

    return _ENV_SUBST.sub(_replace, value)


def _load_natnet_config(config_path: str) -> dict:
    """Parse the ``natnet:`` block from the config YAML."""
    path = Path(config_path)
    if not path.is_file():
        return {}
    with path.open(encoding='utf-8') as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        return {}
    natnet = data.get('natnet', {})
    return cast(dict, natnet) if isinstance(natnet, dict) else {}


def _flatten_covariance(values: Any, fallback: list[float]) -> list[float]:
    """Coerce a 9-element covariance block to floats, falling back when absent."""
    if not isinstance(values, (list, tuple)) or len(values) == 0:
        return list(fallback)
    return [float(v) for v in values]


def _build_node_params(server: dict, profile: dict) -> dict:
    """Flatten the server block + a robot's body list into node parameters."""
    bodies = profile.get('bodies', []) or []

    params: dict[str, Any] = {
        'server_ip': str(_expand_env(server.get('server_ip', '172.31.0.200'))),
        'client_ip': str(_expand_env(server.get('client_ip', '0.0.0.0'))),
        'command_port': int(server.get('command_port', 1510)),
        'data_port': int(server.get('data_port', 1511)),
        'connection_type': str(server.get('connection_type', 'unicast')),
        'multicast_address': str(server.get('multicast_address', '239.255.42.99')),
        'frame_id': str(server.get('frame_id', 'world')),
        'debug': bool(server.get('debug', False)),
        'latency_sampling_warmup_s': float(server.get('latency_sampling_warmup_s', 5.0)),
        'latency_sampling_window_s': float(server.get('latency_sampling_window_s', 20.0)),
        'cube_orange_latency_ms': float(server.get('cube_orange_latency_ms', 5.0)),
    }

    body_names: list[str] = []
    body_ids: list[int] = []
    body_topics: list[str] = []
    body_pose: list[bool] = []
    body_pose_cov: list[bool] = []
    body_position_covariance: list[float] = []
    body_orientation_covariance: list[float] = []

    for body in bodies:
        body_names.append(str(body.get('rigid_body_name', '')))
        body_ids.append(int(body.get('id', -1)))
        body_topics.append(str(body.get('topic', '')))
        body_pose.append(bool(body.get('pose', True)))
        body_pose_cov.append(bool(body.get('pose_cov', True)))
        body_position_covariance.extend(
            _flatten_covariance(body.get('position_covariance'), _DEFAULT_POSITION_COVARIANCE)
        )
        body_orientation_covariance.extend(
            _flatten_covariance(body.get('orientation_covariance'), _DEFAULT_ORIENTATION_COVARIANCE)
        )

    params.update(
        {
            'body_names': body_names,
            'body_ids': body_ids,
            'body_topics': body_topics,
            'body_pose': body_pose,
            'body_pose_cov': body_pose_cov,
            'body_position_covariance': body_position_covariance,
            'body_orientation_covariance': body_orientation_covariance,
        }
    )
    return params


def _namespaced(robot_name: str, relative: str) -> str:
    """Namespace a relative topic under /{robot_name}/."""
    return '/' + robot_name + '/' + relative.lstrip('/')


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('natnet_ros2')
    default_natnet_yaml = os.path.join(pkg_share, 'config', 'natnet_config.yaml')
    default_vp_yaml = os.path.join(pkg_share, 'config', 'vision_pose_converter.yaml')
    default_gp_origin_yaml = os.path.join(pkg_share, 'config', 'mavros_gp_origin.yaml')

    config_file = LaunchConfiguration('config_file')
    vision_pose_config_file = LaunchConfiguration('vision_pose_config_file')
    gp_origin_config_file = LaunchConfiguration('gp_origin_config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    def launch_setup(context, *_args, **_kwargs):
        cfg_path = config_file.perform(context)
        vp_path = vision_pose_config_file.perform(context)
        gp_path = gp_origin_config_file.perform(context)
        ust = use_sim_time.perform(context)

        robot_name = os.environ.get('ROBOT_NAME', 'robot_1')
        natnet = _load_natnet_config(cfg_path)
        server = natnet.get('server', {}) if isinstance(natnet, dict) else {}
        robots = natnet.get('robots', {}) if isinstance(natnet, dict) else {}
        profile = robots.get(robot_name, {}) if isinstance(robots, dict) else {}

        if not profile:
            print(
                f"[natnet_ros2.launch] WARNING: no profile for ROBOT_NAME='{robot_name}' "
                f"in {cfg_path}; node will start with no tracked bodies."
            )

        node_params = _build_node_params(server, profile)
        # launch_ros / rclpy cannot infer the type of an empty-list parameter, so drop
        # any empty arrays; the node declares matching empty defaults and tracks nothing.
        node_params = {
            k: v for k, v in node_params.items() if not (isinstance(v, list) and len(v) == 0)
        }

        # pkg_share = <prefix>/share/natnet_ros2 → go up two levels to reach <prefix>,
        # then down into lib/natnet_ros2/ where colcon installs executables.
        node_path = Path(pkg_share).parent.parent / 'lib' / 'natnet_ros2' / 'natnet_ros2_node'
        if not node_path.exists():
            raise RuntimeError(
                'natnet_ros2_node executable not found — NatNet SDK is not installed.\n'
                "Run 'airstack setup' to download and install the OptiTrack NatNet SDK,\n"
                'then rebuild the workspace: bws --packages-select natnet_ros2\n'
                'Or set LAUNCH_NATNET=false in .env to disable OptiTrack.'
            )

        actions = [
            Node(
                package='natnet_ros2',
                executable='natnet_ros2_node',
                name='natnet_ros2_node',
                output='screen',
                parameters=[node_params],
            ),
        ]

        vision_pose = profile.get('vision_pose', {}) if isinstance(profile, dict) else {}
        if vision_pose.get('enabled', False):
            input_topic = _namespaced(
                robot_name, str(vision_pose.get('input_topic', 'perception/optitrack/drone/pose_cov'))
            )
            output_pose_topic = _namespaced(
                robot_name, str(vision_pose.get('output_pose_topic', 'interface/mavros/vision_pose/pose'))
            )
            output_pose_cov_topic = _namespaced(
                robot_name,
                str(vision_pose.get('output_pose_cov_topic', 'interface/mavros/vision_pose/pose_cov')),
            )

            actions.append(
                IncludeLaunchDescription(
                    FrontendLaunchDescriptionSource(
                        os.path.join(pkg_share, 'launch', 'mavros_gp_origin.launch.xml'),
                    ),
                    launch_arguments=[
                        ('config_file', gp_path),
                        ('use_sim_time', ust),
                    ],
                ),
            )
            actions.append(
                IncludeLaunchDescription(
                    FrontendLaunchDescriptionSource(
                        os.path.join(pkg_share, 'launch', 'vision_pose_converter.launch.xml'),
                    ),
                    launch_arguments=[
                        ('config_file', vp_path),
                        ('input_topic', input_topic),
                        ('output_pose_topic', output_pose_topic),
                        ('output_pose_cov_topic', output_pose_cov_topic),
                        ('use_sim_time', ust),
                    ],
                ),
            )
        return actions

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'config_file',
                default_value=default_natnet_yaml,
                description='NatNet config YAML (natnet: server + per-robot profiles). '
                'The robot profile selected by ROBOT_NAME drives bodies + MAVROS include.',
            ),
            DeclareLaunchArgument(
                'vision_pose_config_file',
                default_value=default_vp_yaml,
                description='vision_pose_converter parameter YAML (frame_id, canonical_quaternion).',
            ),
            DeclareLaunchArgument(
                'gp_origin_config_file',
                default_value=default_gp_origin_yaml,
                description='mavros_gp_origin parameter YAML.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Forwarded to MAVROS bridge launch files.',
            ),
            OpaqueFunction(function=launch_setup),
        ],
    )
