#!/usr/bin/env python3
"""ROS2 Python launch file for interface bringup.

Dynamically computes FCU URL and TGT_SYSTEM from environment variables (unless FCU_URL is set):
    PX4:  FCU_URL = udp://:<OFFBOARD_PORT>@172.31.0.200:<ONBOARD_PORT>
    ArduPilot SITL: FCU_URL = tcp://172.31.0.200:<5760 + ROS_DOMAIN_ID * 10>
    TGT_SYSTEM = 1 + ROS_DOMAIN_ID (unless TGT_SYSTEM is set)

FLIGHT_STACK=px4 (default) uses mavros_px4.launch.xml; FLIGHT_STACK=ardupilot uses mavros_apm.launch.xml
and sets is_ardupilot on robot_interface_node.

map → base_link TF: ``odometry_conversion`` subscribes to ``interface_odometry_in_topic`` (default:
MAVROS ``local_position/odom``), overwrites frames to map/base_link, and broadcasts TF. To drive TF
from simulation ground truth instead, launch with ``interface_odometry_in_topic`` remapped to that
topic (and ensure QoS/frame ids match). Other sources (VIO, etc.) can use the same remapping pattern.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Compute FCU URL and TGT_SYSTEM, then return all launch actions."""

    # --- Dynamic port / URL calculation (mirrors robot .bashrc logic) -------
    # Use pre-set env vars if available, otherwise compute from base ports + domain id
    ros_domain_id = int(os.environ.get('ROS_DOMAIN_ID', '0'))

    flight_stack = os.environ.get('FLIGHT_STACK', 'px4').strip().lower()
    is_ardupilot = flight_stack in ('ardupilot', 'apm')

    if os.environ.get('FCU_URL'):
        fcu_url = os.environ['FCU_URL']
    elif is_ardupilot:
        # ArduCopter SITL exposes serial0 as tcp:0.0.0.0:<5760 + ROS_DOMAIN_ID * 10>:wait
        # inside the Isaac container (172.31.0.200).  MAVROS connects via TCP.
        serial0_port = 5760 + ros_domain_id * 10
        fcu_url = f'tcp://172.31.0.200:{serial0_port}'
    else:
        offboard_base_port = int(os.environ.get('OFFBOARD_BASE_PORT', '14540'))
        onboard_base_port = int(os.environ.get('ONBOARD_BASE_PORT', '14580'))
        offboard_port = offboard_base_port + ros_domain_id
        onboard_port = onboard_base_port + ros_domain_id
        fcu_url = f'udp://:{offboard_port}@172.31.0.200:{onboard_port}'

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
        mavros_launch = (
            'mavros_apm.launch.xml' if is_ardupilot else 'mavros_px4.launch.xml'
        )
        mavros_group = GroupAction([
            PushRosNamespace('interface'),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource([
                    FindPackageShare('interface_bringup'),
                    f'/launch/{mavros_launch}',
                ]),
                launch_arguments={
                    'fcu_url': fcu_url,
                    'tgt_system': tgt_system,
                }.items(),
            ),
        ])
        actions.append(mavros_group)

        # ArduPilot: MAVROS for APM on Jazzy does not auto-request MAVLink data
        # streams.  PX4 pushes everything by default, but ArduPilot needs explicit
        # SET_MESSAGE_INTERVAL for each message.  Without this, LOCAL_POSITION_NED
        # is never sent on serial0 → odometry_conversion gets nothing → no TF.
        # set_stream_rate (REQUEST_DATA_STREAM) does NOT reliably include
        # LOCAL_POSITION_NED on ArduCopter 4.x; set_message_interval works.
        if is_ardupilot:
            mavros_ns = f'/{robot_name}/interface/mavros'
            odom_topic = f'/{robot_name}/interface/mavros/local_position/odom'
            interval_svc = f'{mavros_ns}/set_message_interval'
            # ArduPilot ACKs SET_MESSAGE_INTERVAL even before EKF converges, but
            # silently ignores it until the message is available.  Retry in a loop
            # until local_position/odom actually starts publishing.
            # odom requires BOTH LOCAL_POSITION_NED (32) and ATTITUDE_QUATERNION (31).
            stream_requests = [
                # (MAVLink msg ID, rate Hz)
                (32, 10.0),   # LOCAL_POSITION_NED  → local_position/{pose,odom}
                (31, 10.0),   # ATTITUDE_QUATERNION → attitude in odom, imu/data
                (30, 10.0),   # ATTITUDE            → imu/data orientation
                (27, 10.0),   # RAW_IMU             → imu/data, imu/data_raw
                (33, 5.0),    # GLOBAL_POSITION_INT → global_position/{global,rel_alt}
                (74, 5.0),    # VFR_HUD             → altitude, airspeed, heading
                (1, 2.0),     # SYS_STATUS          → battery, sensors health
                (24, 5.0),    # GPS_RAW_INT         → GPS fix quality
                (29, 2.0),    # SCALED_PRESSURE     → barometer / imu/static_pressure
            ]
            svc_calls = ' '.join(
                f'ros2 service call {interval_svc} '
                f'mavros_msgs/srv/MessageInterval '
                f'"{{message_id: {mid}, message_rate: {rate}}}" > /dev/null 2>&1;'
                for mid, rate in stream_requests
            )
            request_script = (
                f'for attempt in $(seq 1 60); do '
                f'  {svc_calls} '
                f'  if timeout 3 ros2 topic hz {odom_topic} 2>&1 '
                f'    | grep -q "average rate"; then '
                f'    echo "[ArduPilot streams] local_position/odom active after attempt $attempt"; '
                f'    exit 0; '
                f'  fi; '
                f'  echo "[ArduPilot streams] attempt $attempt — odom not yet active, retrying in 5s"; '
                f'  sleep 5; '
                f'done; '
                f'echo "[ArduPilot streams] WARNING: odom still not active after 60 attempts"'
            )
            actions.append(
                TimerAction(
                    period=10.0,
                    actions=[
                        ExecuteProcess(
                            cmd=['bash', '-c', request_script],
                            output='screen',
                        ),
                    ],
                )
            )

    # --- Interface between AirStack and MAVROS ------------------------------
    robot_interface_node = Node(
        package='robot_interface',
        executable='robot_interface_node',
        namespace='interface',
        output='screen',
        parameters=[{
            'interface': 'mavros_interface::MAVROSInterface',
            'is_ardupilot': is_ardupilot,
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
            # True → BEST_EFFORT (MAVROS local_position/odom on Jazzy). Required for matching QoS.
            # False → RELIABLE (use for publishers that only offer reliable, e.g. some bag/replay setups).
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
        description=(
            'nav_msgs/Odometry source for odometry_conversion (map→base_link TF). '
            'Default: MAVROS local position. Override for sim ground-truth or other estimators.'
        ),
    )

    return LaunchDescription([
        interface_odometry_in_topic_arg,
        OpaqueFunction(function=launch_setup),
    ])
