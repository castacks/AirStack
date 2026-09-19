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

Teleop: whenever the run has teleop drones (the config's `teleop_drones` or
the `teleop_drones:=` override), the input-device driver and `safe_teleop`
are started here too. Which device is wired up is the `teleop_controller`
parameter — the config's `safe_teleop` block, or `teleop_controller:=` on
the launch line — resolved against `safe_teleop/controllers.py` (currently
`xbox_usb`). `use_teleop:=false` leaves both to be started by hand (as
scripts/svg_teleop.sh does, in its own tmux sessions).
"""

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from svg_ground_control.safe_teleop.controllers import (DEFAULT_CONTROLLER,
                                                        get_controller)


def _node_params(config_path: str, node_name: str) -> dict:
    """The ``ros__parameters`` of one node block in the config YAML ({} if absent)."""
    try:
        with open(config_path) as f:
            doc = yaml.safe_load(f) or {}
    except OSError:
        return {}
    block = doc.get(node_name) or {}
    return block.get('ros__parameters') or {}


def _name_list(value) -> list:
    if isinstance(value, (list, tuple)):
        return [str(v).strip() for v in value if str(v).strip()]
    return [n.strip() for n in str(value or '').split(',') if n.strip()]


def teleop_nodes(context, config_path: str, teleop_drones: list) -> list:
    """Input driver + safe_teleop for a run that hand-flies a drone.

    Registry-driven: the controller profile says which driver nodes make the
    device a sensor_msgs/Joy stream and what its axis map is. One physical
    device can only fly one drone, so safe_teleop is started for the FIRST
    teleop drone; extra ones need their own device and a hand-started node.
    """
    use_teleop = LaunchConfiguration('use_teleop').perform(context).lower()
    if use_teleop == 'false' or (use_teleop != 'true' and not teleop_drones):
        return []
    if not teleop_drones:
        return [LogInfo(msg='use_teleop:=true but no teleop drones are '
                            'configured; not starting teleop')]

    name = (LaunchConfiguration('teleop_controller').perform(context).strip()
            or str(_node_params(config_path, 'safe_teleop')
                   .get('teleop_controller', '')).strip()
            or DEFAULT_CONTROLLER)
    profile = get_controller(name)      # KeyError lists the supported names

    drone = teleop_drones[0]
    actions = [LogInfo(msg=f'teleop: {profile.name} ({profile.description}) '
                           f'-> {drone}')]
    if len(teleop_drones) > 1:
        actions.append(LogInfo(
            msg=f'teleop: only {drone} gets the {profile.name} controller; '
                f'{", ".join(teleop_drones[1:])} need their own device '
                f'(start safe_teleop by hand with -p drone:=<name>)'))
    for driver in profile.drivers:
        actions.append(Node(
            package=driver.package, executable=driver.executable,
            name=driver.name, output='screen',
            # Profile defaults first, so a matching block in the config
            # (e.g. joy_node: {ros__parameters: {device_id: 1}}) wins.
            parameters=[dict(driver.parameters), config_path],
        ))
    actions.append(Node(
        package='svg_ground_control', executable='safe_teleop',
        name='safe_teleop', output='screen',
        parameters=[config_path,
                    {'drone': drone, 'teleop_controller': profile.name}],
    ))
    return actions


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

    teleop_names = _name_list(
        teleop_drones or _node_params(config_path, 'swarm_commander')
        .get('teleop_drones', ''))

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
            'use_teleop', default_value='auto',
            description='Start the input-device driver + safe_teleop for the '
                        'first teleop drone: auto (when the run has teleop '
                        'drones), true, or false (start them by hand)'),
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
