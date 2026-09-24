"""Launch the SVG ground controller.

    # Simulation (MAVROS interfaces, no mocap), scenario from the config:
    ros2 launch svg_ground_control ground_control.launch.py

    # Override the scenario from the CLI:
    ros2 launch svg_ground_control ground_control.launch.py scenario:=squeeze

    # Hardware (px4_interface + mocap bridge):
    ros2 launch svg_ground_control ground_control.launch.py \
        config:=<path>/swarm_real.yaml use_mocap:=true

    # Foxglove: the bridge starts with the commander (ws://localhost:8765).
    # Open Studio on the host, or inside the container with:
    ros2 launch svg_ground_control ground_control.launch.py use_foxglove_studio:=true
    # Disable the bridge (e.g. one already running elsewhere):
    ros2 launch svg_ground_control ground_control.launch.py use_foxglove_bridge:=false

The keyboard teleop is NOT started here — it needs its own TTY:
    ros2 run svg_ground_control keyboard_teleop --ros-args -p drone:=drone_3
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    config = LaunchConfiguration('config')
    scenario = LaunchConfiguration('scenario').perform(context)
    foxglove_port = int(LaunchConfiguration('foxglove_port').perform(context))

    commander_params = [config]
    if scenario:
        commander_params.append({'scenario': scenario})

    return [
        Node(
            package='svg_ground_control',
            executable='swarm_commander',
            name='swarm_commander',
            output='screen',
            parameters=commander_params,
        ),
        Node(
            package='svg_ground_control',
            executable='mocap_bridge',
            name='mocap_bridge',
            output='screen',
            parameters=[config],
            condition=IfCondition(LaunchConfiguration('use_mocap')),
        ),
        # Onboard LED strips (real drones): green by default, red while the CBF
        # corrects a drone, per-drone /svg/<name>/set_led_color service. Talks
        # UDP to scripts/svg_led_daemon.py on each VOXL; harmless when no drone
        # answers (sim runs).
        Node(
            package='svg_ground_control',
            executable='led_controller',
            name='led_controller',
            output='screen',
            parameters=[config],
            condition=IfCondition(LaunchConfiguration('use_led')),
        ),
        # Foxglove bridge: serves every topic and service on this ROS domain to
        # Foxglove Studio over WebSocket. It lives here so the SVG Basestation
        # panel works as soon as ground control is up, with nothing started by
        # hand. robot-desktop runs on network_mode: host, so a Studio on the
        # host connects to ws://localhost:<foxglove_port> directly. The `gcs`
        # container is not involved (Docker bridge network, domain 0 — it
        # never sees the drone topics). package:// mesh assets for the drone
        # markers are served through the bridge's asset capability.
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            respawn=True,
            respawn_delay=1.0,
            parameters=[{
                'port': foxglove_port,
                'address': '0.0.0.0',
                'include_hidden': True,
                'send_buffer_limit': 10000000,
            }],
            condition=IfCondition(LaunchConfiguration('use_foxglove_bridge')),
        ),
        # Foxglove Studio INSIDE the container, pre-connected to the bridge.
        # Off by default (most runs use a Studio on the host). Needs the X
        # display docker-compose passes through; --no-sandbox because the
        # container runs as root. The SVG Basestation panel comes from this
        # package's foxglove/ directory (general AirStack panels from
        # gcs/foxglove_extensions), installed into ~/.foxglove-studio at
        # container start.
        ExecuteProcess(
            cmd=['foxglove-studio', '--no-sandbox',
                 'foxglove://open?ds=foxglove-websocket'
                 f'&ds.url=ws://localhost:{foxglove_port}'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_foxglove_studio')),
        ),
    ]


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare('svg_ground_control'), 'config', 'swarm_sim.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='Swarm parameter YAML (swarm_sim.yaml or swarm_real.yaml)'),
        DeclareLaunchArgument(
            'scenario', default_value='',
            description='Override the scenario from the config: hover, '
                        'random_walk, random_goals, head_on, antipodal, squeeze'),
        DeclareLaunchArgument(
            'use_mocap', default_value='true',
            description='Start the mocap bridge (hardware only)'),
        DeclareLaunchArgument(
            'use_led', default_value='true',
            description='Start the onboard-LED controller (UDP to svg_led_daemon on '
                        'each real drone; no-op without drones)'),
        DeclareLaunchArgument(
            'use_foxglove_bridge', default_value='true',
            description='Start foxglove_bridge (WebSocket for Foxglove Studio) '
                        'alongside the commander'),
        DeclareLaunchArgument(
            'foxglove_port', default_value='8765',
            description='foxglove_bridge WebSocket port'),
        DeclareLaunchArgument(
            'use_foxglove_studio', default_value='false',
            description='Also open Foxglove Studio inside the container, '
                        'pre-connected to the bridge (needs an X display)'),
        OpaqueFunction(function=launch_setup),
    ])
