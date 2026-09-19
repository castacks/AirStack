"""Launch the SVG ground controller.

    # Simulation (MAVROS interfaces, no mocap), scenario from the config:
    ros2 launch svg_ground_control ground_control.launch.py

    # Override the scenario from the CLI:
    ros2 launch svg_ground_control ground_control.launch.py scenario:=squeeze

    # Squeeze with the intruder hand-flown instead of scenario-driven:
    ros2 launch svg_ground_control ground_control.launch.py \
        scenario:=squeeze teleop_drones:=drone_3

    # Hardware (px4_interface + mocap bridge):
    ros2 launch svg_ground_control ground_control.launch.py \
        config:=<path>/swarm_real.yaml use_mocap:=true

Teleop is NOT started here by default. Start it first, in its own terminal,
and check the printed stick readings before bringing up the commander:
    ros2 launch svg_ground_control teleop.launch.py config:=<same config>
`use_teleop:=true` bundles the input-device driver and `safe_teleop` (for the
first drone in `teleop_drones`) into this launch instead; `use_teleop:=auto`
does so only when the run has teleop drones. The device is the
`teleop_controller` parameter (config `safe_teleop` block or
`teleop_controller:=`), an entry of `safe_teleop/controllers.py` (xbox_usb).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from svg_ground_control.safe_teleop.launch_helpers import (
    config_teleop_drones, resolve_controller, teleop_actions)


def teleop_nodes(context, config_path: str, teleop_drones: list) -> list:
    """Optionally bundle the input driver + safe_teleop (see teleop.launch.py)."""
    use_teleop = LaunchConfiguration('use_teleop').perform(context).lower()
    if use_teleop == 'false' or (use_teleop != 'true' and not teleop_drones):
        return []
    if not teleop_drones:
        return [LogInfo(msg='use_teleop:=true but no teleop drones are '
                            'configured; not starting teleop')]
    profile = resolve_controller(
        config_path, LaunchConfiguration('teleop_controller').perform(context))
    return teleop_actions(config_path, teleop_drones[0], profile,
                          others=teleop_drones[1:])


def launch_setup(context, *args, **kwargs):
    config = LaunchConfiguration('config')
    config_path = config.perform(context)
    scenario = LaunchConfiguration('scenario').perform(context)
    teleop_drones = LaunchConfiguration('teleop_drones').perform(context)

    commander_params = [config]
    if scenario:
        commander_params.append({'scenario': scenario})
    if teleop_drones:
        commander_params.append({'teleop_drones': teleop_drones})

    teleop_names = config_teleop_drones(config_path, teleop_drones)

    return teleop_nodes(context, config_path, teleop_names) + [
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
            'teleop_drones', default_value='',
            description='Override teleop_drones from the config: comma-'
                        'separated drone names flown by hand instead of by '
                        'the scenario, e.g. drone_3 to hand-fly the squeeze '
                        'intruder'),
        DeclareLaunchArgument(
            'use_mocap', default_value='false',
            description='Start the mocap bridge (hardware only)'),
        DeclareLaunchArgument(
            'use_teleop', default_value='false',
            description='Also start the input-device driver + safe_teleop for '
                        'the first teleop drone here: false (default: use '
                        'teleop.launch.py in its own terminal first), true, '
                        'or auto (only when the run has teleop drones)'),
        DeclareLaunchArgument(
            'teleop_controller', default_value='',
            description='Input device for teleop, an entry of '
                        'safe_teleop/controllers.py (xbox_usb). Empty = the '
                        "config's safe_teleop.teleop_controller, else xbox_usb"),
        DeclareLaunchArgument(
            'use_led', default_value='true',
            description='Start the onboard-LED controller (UDP to svg_led_daemon on '
                        'each real drone; no-op without drones)'),
        OpaqueFunction(function=launch_setup),
    ])
