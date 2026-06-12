#!/usr/bin/env python3
"""Bring up NatNet node; optionally MAVROS bridge per natnet_config.yaml.

natnet_ros2_node is a C++ executable that requires the OptiTrack NatNet SDK.
If the SDK was not installed (``airstack setup`` not run) and the workspace
has not been rebuilt, launching this file will raise a RuntimeError with
instructions. Set LAUNCH_NATNET=false in .env to disable OptiTrack entirely.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import cast

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def _ros_params_from_file(config_path: str) -> dict:
    """Parse /** / ros__parameters block from a ROS 2 parameter YAML."""
    path = Path(config_path)
    if not path.is_file():
        return {}
    with path.open(encoding='utf-8') as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        return {}
    block = data.get('/**')
    if not isinstance(block, dict):
        return {}
    params = block.get('ros__parameters', {})
    return cast(dict, params) if isinstance(params, dict) else {}


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

        ros_params = _ros_params_from_file(cfg_path)
        publish_mavros = bool(ros_params.get('publish_to_mavros', False))
        body_name = str(ros_params.get('body_name', 'robot_1'))

        # pkg_share = <prefix>/share/natnet_ros2 → go up two levels to reach <prefix>,
        # then down into lib/natnet_ros2/ where colcon installs executables.
        pkg_share = get_package_share_directory('natnet_ros2')
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
                parameters=[ParameterFile(config_file, allow_substs=True)],
            ),
        ]

        if publish_mavros:
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
                        ('body_name', body_name),
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
                description='NatNet parameter YAML (/** ros__parameters). '
                'publish_to_mavros and body_name control MAVROS include.',
            ),
            DeclareLaunchArgument(
                'vision_pose_config_file',
                default_value=default_vp_yaml,
                description='vision_pose_converter parameter YAML.',
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
