#!/usr/bin/env python3
"""Bring up NatNet node; optionally MAVROS bridge per natnet_config.yaml."""

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

    config_file = LaunchConfiguration('config_file')
    vision_pose_config_file = LaunchConfiguration('vision_pose_config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    def launch_setup(context, *_args, **_kwargs):
        cfg_path = config_file.perform(context)
        vp_path = vision_pose_config_file.perform(context)
        ust = use_sim_time.perform(context)

        ros_params = _ros_params_from_file(cfg_path)
        publish_mavros = bool(ros_params.get('publish_to_mavros', False))
        body_name = str(ros_params.get('body_name', 'robot_1'))

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
                'use_sim_time',
                default_value='false',
                description='Forwarded to vision_pose_converter.launch.xml.',
            ),
            OpaqueFunction(function=launch_setup),
        ],
    )
