#!/usr/bin/env python3
"""ROS2 Python launch file for interface bringup.

Dynamically computes FCU URL and TGT_SYSTEM from environment variables:
    OFFBOARD_PORT = OFFBOARD_BASE_PORT + ROS_DOMAIN_ID
    ONBOARD_PORT  = ONBOARD_BASE_PORT  + ROS_DOMAIN_ID
    FCU_URL       = udp://:<OFFBOARD_PORT>@<SIM_IP>:<ONBOARD_PORT>
    TGT_SYSTEM    = 1 + ROS_DOMAIN_ID
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Compute FCU URL and TGT_SYSTEM, then return all launch actions."""

    # --- Dynamic port / URL calculation (mirrors robot .bashrc logic) -------
    # Use pre-set env vars if available, otherwise compute from base ports + domain id
    ros_domain_id = int(os.environ.get('ROS_DOMAIN_ID', '0'))

    if os.environ.get('FCU_URL'):
        fcu_url = os.environ['FCU_URL']
    else:
        offboard_base_port = int(os.environ.get('OFFBOARD_BASE_PORT', '14540'))
        onboard_base_port = int(os.environ.get('ONBOARD_BASE_PORT', '14580'))
        offboard_port = offboard_base_port + ros_domain_id
        onboard_port = onboard_base_port + ros_domain_id
        sim_ip = os.environ.get('SIM_IP', '172.31.0.200')
        fcu_url = f'udp://:{offboard_port}@{sim_ip}:{onboard_port}'

    tgt_system = os.environ.get('TGT_SYSTEM') or str(1 + ros_domain_id)

    # --- Other environment variables ----------------------------------------
    robot_name = os.environ.get('ROBOT_NAME', 'robot')
    sim_type = os.environ.get('SIM_TYPE', '')

    interface_odometry_in_topic = context.launch_configurations.get(
        'interface_odometry_in_topic',
        f'/{robot_name}/interface/mavros/local_position/odom',
    )

    actions = []

    # --- MAVROS (skipped in simple sim) -------------------------------------
    if sim_type != 'simple':
        mavros_group = GroupAction([
            PushRosNamespace('interface'),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource([
                    FindPackageShare('interface_bringup'),
                    '/launch/mavros_px4.launch.xml',
                ]),
                launch_arguments={
                    'fcu_url': fcu_url,
                    'tgt_system': tgt_system,
                }.items(),
            ),
        ])
        actions.append(mavros_group)

    # --- Interface between AirStack and MAVROS ------------------------------
    robot_interface_node = Node(
        package='robot_interface',
        executable='robot_interface_node',
        namespace='interface',
        output='screen',
        parameters=[{
            'interface': 'mavros_interface::MAVROSInterface',
            'is_ardupilot': False,
            'post_takeoff_command_delay_time': 10.0,
            'do_global_pose_command': False,
        }],
        remappings=[
            ('ardupilot_takeoff',
             f'/{robot_name}/takeoff_landing_planner/ardupilot_takeoff'),
            ('reset_integrators',
             f'/{robot_name}/control/reset_integrators'),
        ],
    )
    actions.append(robot_interface_node)

    # --- Position setpoints -------------------------------------------------
    position_setpoint_node = Node(
        package='mavros_interface',
        executable='position_setpoint_pub.py',
        namespace='interface',
        output='screen',
        parameters=[{
            'command_type': 0,
            'max_velocity': 3.0,
            'target_frame': 'base_link',
            'publish_goal': False,
        }],
    )
    actions.append(position_setpoint_node)

    # --- Odometry conversion ------------------------------------------------
    odometry_conversion_node = Node(
        package='robot_interface',
        executable='odometry_conversion',
        namespace='odometry_conversion',
        output='screen',
        parameters=[{
            'odom_input_qos_is_best_effort': True,
            'new_frame_id': 'map',
            'new_child_frame_id': 'base_link',
            'odometry_output_type': 2,
            'convert_odometry_to_transform': True,
            'convert_odometry_to_stabilized_transform': True,
            'restamp_now_post': False,
        }],
        remappings=[
            ('odometry_in', interface_odometry_in_topic),
            ('odometry_out', 'odometry'),
        ],
    )
    actions.append(odometry_conversion_node)

    # --- Drone safety monitor -----------------------------------------------
    drone_safety_monitor_node = Node(
        package='drone_safety_monitor',
        executable='drone_safety_monitor',
        namespace='drone_safety_monitor',
        output='screen',
        parameters=[{
            'state_estimate_timeout': 1.0,
        }],
        remappings=[
            ('state_estimate',
             f'/{robot_name}/odometry_conversion/odometry'),
        ],
    )
    actions.append(drone_safety_monitor_node)

    return actions


def generate_launch_description():
    robot_name = os.environ.get('ROBOT_NAME', 'robot')

    interface_odometry_in_topic_arg = DeclareLaunchArgument(
        'interface_odometry_in_topic',
        default_value=f'/{robot_name}/interface/mavros/local_position/odom',
        description='Input odometry topic remapped into the odometry_conversion node',
    )

    return LaunchDescription([
        interface_odometry_in_topic_arg,
        OpaqueFunction(function=launch_setup),
    ])
