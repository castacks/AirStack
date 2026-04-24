#!/usr/bin/env python3
"""ROS2 Python launch file for interface bringup.

Dynamically computes FCU URL and TGT_SYSTEM from environment variables:
    ROBOT_INDEX   = trailing integer of ROBOT_NAME (robot_1 -> 1), or env override
    OFFBOARD_PORT = OFFBOARD_BASE_PORT                   (fixed, per-netns)
    ONBOARD_PORT  = ONBOARD_BASE_PORT  + ROBOT_INDEX     (per-robot)
    FCU_URL       = udp://:<OFFBOARD_PORT>@<SIM_IP>:<ONBOARD_PORT>
    TGT_SYSTEM    = 1 + ROBOT_INDEX

Under rmw_zenoh_cpp every container is on ROS_DOMAIN_ID=0 (Zenoh's isolation model),
so per-robot uniqueness is carried by ROBOT_INDEX instead of ROS_DOMAIN_ID.
"""

import os
import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Compute FCU URL and TGT_SYSTEM, then return all launch actions."""

    # --- Per-robot port offset ---------------------------------------------
    # Historically derived from ROS_DOMAIN_ID, but under rmw_zenoh_cpp all
    # containers share ROS_DOMAIN_ID=0 for one shared graph. Derive the per-
    # robot offset from the trailing integer in ROBOT_NAME (e.g. "robot_1" -> 1,
    # "drone-alpha-3" -> 3). Set ROBOT_INDEX explicitly to override when the
    # robot name has no trailing number.
    robot_name_env = os.environ.get('ROBOT_NAME', '')
    explicit_index = os.environ.get('ROBOT_INDEX')
    if explicit_index is not None:
        robot_index = int(explicit_index)
    else:
        trailing_digits = re.search(r'(\d+)$', robot_name_env)
        if trailing_digits is None:
            raise RuntimeError(
                f"ROBOT_NAME='{robot_name_env}' has no trailing integer; "
                "set ROBOT_INDEX explicitly so each robot gets a unique MAVLink port."
            )
        robot_index = int(trailing_digits.group(1))

    if os.environ.get('FCU_URL'):
        fcu_url = os.environ['FCU_URL']
    else:
        # Target PX4's Normal-mode MAVLink endpoint (not Onboard mode):
        # - Onboard mode sends outbound to a HARDCODED 127.0.0.1 target, so replies
        #   never leave the sim container. We can't reach it cross-container.
        # - Normal mode learns the target IP from received heartbeats, so it works
        #   across any container topology without patching PX4's rc.mavlink.
        # Default Pegasus/PX4 SITL ports for Normal mode:
        #   PX4 local (where PX4 listens)  : 18570 + robot_index   (e.g. 18571, 18572)
        #   PX4 remote (where PX4 sends)   : 14550 (QGC default — mavros binds this)
        offboard_base_port = int(os.environ.get('OFFBOARD_BASE_PORT', '14550'))
        onboard_base_port = int(os.environ.get('ONBOARD_BASE_PORT', '18570'))
        # Local bind port: same for every robot (each mavros is in its own container netns).
        # If you ever run multiple mavros instances in ONE netns, add +robot_index here.
        offboard_port = offboard_base_port
        onboard_port = onboard_base_port + robot_index
        sim_ip = os.environ.get('SIM_IP', '172.31.0.200')
        fcu_url = f'udp://:{offboard_port}@{sim_ip}:{onboard_port}'

    tgt_system = os.environ.get('TGT_SYSTEM') or str(1 + robot_index)

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

    # NOTE: drone_safety_monitor is now launched from behavior_bringup

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
