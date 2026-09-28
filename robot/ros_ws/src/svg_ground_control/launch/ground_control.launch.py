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

    # Foxglove: the bridge starts with the commander (ws://localhost:8765).
    # Open Studio on the host, or inside the container with:
    ros2 launch svg_ground_control ground_control.launch.py use_foxglove_studio:=true
    # Disable the bridge (e.g. one already running elsewhere):
    ros2 launch svg_ground_control ground_control.launch.py use_foxglove_bridge:=false

    # Rosbag: the bag_recorder node starts with the commander; the Rosbag
    # switch in the Foxglove basestation panel (or
    # `ros2 service call /bag_recorder/record std_srvs/srv/SetBool "{data: true}"`)
    # starts/stops `ros2 bag record --all-topics` (mcap) at
    # bag_dir/<bag_prefix>_YYYYmmdd_HHMMSS, default ~/AirStack/robot/ros_ws/bags.
    # record_bag:=true records from launch to Ctrl-C (still switchable):
    ros2 launch svg_ground_control ground_control.launch.py record_bag:=true
    ros2 launch svg_ground_control ground_control.launch.py \
        record_bag:=true bag_dir:=/bags bag_prefix:=squeeze_c2

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
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            OpaqueFunction)
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


def bag_recorder(context) -> list:
    """The switchable rosbag recorder (svg_ground_control/bag_recorder.py).

    Always started (use_bag_recorder) so the Foxglove panel's Rosbag switch has
    something to talk to; it records nothing until switched on, unless
    `record_bag:=true`, which is the old whole-run recording: on from launch,
    off at Ctrl-C (which still writes metadata.yaml), and switchable in
    between. One mcap bag per start at <bag_dir>/<bag_prefix>_<YYYYmmdd_HHMMSS>;
    the default bag_dir is the bind-mounted workspace so the bag is on the
    host when the container goes away (experiment.md, "Recording rosbags").
    """
    if LaunchConfiguration('use_bag_recorder').perform(context).lower() != 'true':
        return []
    record_on_start = LaunchConfiguration('record_bag').perform(context).lower() == 'true'
    return [
        Node(
            package='svg_ground_control',
            executable='bag_recorder',
            name='bag_recorder',
            output='screen',
            parameters=[{
                'bag_dir': LaunchConfiguration('bag_dir'),
                'bag_prefix': LaunchConfiguration('bag_prefix'),
                'include_hidden': LaunchConfiguration('bag_include_hidden'),
                'record_on_start': record_on_start,
            }],
        ),
    ]


def launch_setup(context, *args, **kwargs):
    config = LaunchConfiguration('config')
    config_path = config.perform(context)
    scenario = LaunchConfiguration('scenario').perform(context)
    teleop_drones = LaunchConfiguration('teleop_drones').perform(context)
    foxglove_port = int(LaunchConfiguration('foxglove_port').perform(context))

    commander_params = [config]
    commander_params.append({'takeover_twins':
                             LaunchConfiguration('takeover').perform(context).lower()
                             in ('true', '1', 'yes')})
    if scenario:
        commander_params.append({'scenario': scenario})
    if teleop_drones:
        commander_params.append({'teleop_drones': teleop_drones})

    teleop_names = config_teleop_drones(config_path, teleop_drones)

    return teleop_nodes(context, config_path, teleop_names) + bag_recorder(context) + [
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
                        'random_walk, random_goals, head_on, antipodal, squeeze, '
                        'figure_eight, goal_sequence'),
        DeclareLaunchArgument(
            'teleop_drones', default_value='',
            description='Override teleop_drones from the config: comma-'
                        'separated drone names flown by hand instead of by '
                        'the scenario, e.g. drone_3 to hand-fly the squeeze '
                        'intruder'),
        DeclareLaunchArgument(
            'use_mocap', default_value='true',
            description='Start the mocap bridge (hardware only)'),
        DeclareLaunchArgument(
            'takeover', default_value='true',
            description='Kill any other swarm_commander process left in this '
                        'container by an earlier launch, unless it has a drone '
                        'in the air (then this one refuses takeoff/start)'),
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
        DeclareLaunchArgument(
            'use_bag_recorder', default_value='true',
            description='Start the bag_recorder node (the Rosbag switch in '
                        'the Foxglove panel / the /bag_recorder/record '
                        'service); records nothing until switched on'),
        DeclareLaunchArgument(
            'record_bag', default_value='false',
            description='Record every published topic on this ROS domain to '
                        'an mcap rosbag from launch (bag_recorder '
                        'record_on_start); the panel switch can still stop '
                        'and restart it; stops with the launch'),
        DeclareLaunchArgument(
            'bag_dir', default_value='~/AirStack/robot/ros_ws/bags',
            description='Directory the bag is written under (default: the '
                        'bind-mounted workspace, so it survives the container; '
                        'use /bags on the robot-l4t storage mount)'),
        DeclareLaunchArgument(
            'bag_prefix', default_value='svg',
            description='Bag folder name prefix: <bag_dir>/<bag_prefix>_'
                        '<YYYYmmdd_HHMMSS>'),
        DeclareLaunchArgument(
            'bag_include_hidden', default_value='false',
            description='Also record hidden topics (/_ and /rosout-style '
                        'internals) in the bag'),
        OpaqueFunction(function=launch_setup),
    ])
