"""Mode C bring-up: NatNet mocap + px4_interface + DiffAero velocity commander.

Does **not** start MicroXRCEAgent — run that in a separate terminal first:

    MicroXRCEAgent udp4 -p 8888 -v4

Optional ToF (§10.2): start ``tof_udp_bridge.py`` before ``tof_udp_stream`` on the VOXL.

Usage (robot container, ``sws``, ``ROS_DOMAIN_ID=1``, host network):

    ros2 launch svg_ground_control diffaero_real.launch.py

    # Override OptiTrack / Motive IPs (defaults match natnet_ros2.launch.py):
    ros2 launch svg_ground_control diffaero_real.launch.py \\
        natnet_server_ip:=192.168.123.199 natnet_client_ip:=192.168.123.134

    # Multi-drone px4_interface only (commander YAML still single-drone):
    ros2 launch svg_ground_control diffaero_real.launch.py drones:=drone_1,drone_2

Preflight before ``~/takeoff``:

    bash $(ros2 pkg prefix svg_ground_control)/share/svg_ground_control/scripts/preflight_diffaero_mode_c.sh drone_1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Policy presets — a short name maps to the parameter overlay applied on top of
# the base config YAML (diffaero_vel_real.yaml). Empty `policy:=` keeps whatever
# checkpoint_path the YAML declares (default: sha2c_vel_cmd_oa). Planar
# checkpoints emit a 2-D action; keep conservative first-flight velocity caps for
# real hardware (no max_vel_z — the planar wrapper ignores it).
CKPT = "/root/AirStack/robot/ros_ws/checkpoints/diffaero"
POLICIES = {
    "sha2c":       {"checkpoint_path": f"{CKPT}/sha2c_vel_cmd_oa/"},
    "planar_mlp":  {"checkpoint_path": f"{CKPT}/planar_mlp_sr0.96/",
                    "max_vel": 1.5, "max_vel_xy": 1.0},
    "planar_cnn":  {"checkpoint_path": f"{CKPT}/planar_cnn_sr0.97/",
                    "max_vel": 1.5, "max_vel_xy": 1.0},
    "planar_rcnn": {"checkpoint_path": f"{CKPT}/planar_rcnn_sr0.97/",
                    "max_vel": 1.5, "max_vel_xy": 1.0},
}


def launch_setup(context, *args, **kwargs):
    config = LaunchConfiguration('config')
    scenario = LaunchConfiguration('scenario').perform(context)
    policy = LaunchConfiguration('policy').perform(context)
    natnet_server = LaunchConfiguration('natnet_server_ip')
    natnet_client = LaunchConfiguration('natnet_client_ip')

    commander_params = [config]
    if scenario:
        commander_params.append({'scenario': scenario})
    if policy:
        if policy not in POLICIES:
            raise RuntimeError(
                f"Unknown policy '{policy}'. Valid: {', '.join(POLICIES)}")
        commander_params.append(dict(POLICIES[policy]))

    natnet_launch = PathJoinSubstitution(
        [FindPackageShare('natnet_ros2'), 'launch', 'natnet_ros2.launch.py'])

    real_interfaces_launch = PathJoinSubstitution(
        [FindPackageShare('svg_ground_control'), 'launch', 'real_interfaces.launch.py'])

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(natnet_launch),
            launch_arguments={
                'serverIP': natnet_server,
                'clientIP': natnet_client,
                'pub_rigid_body': 'true',
                'activate': 'true',
            }.items(),
            condition=IfCondition(LaunchConfiguration('start_natnet')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(real_interfaces_launch),
            launch_arguments={
                'drones': LaunchConfiguration('drones'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('start_interfaces')),
        ),
        Node(
            package='svg_ground_control',
            executable='mocap_bridge',
            name='mocap_bridge',
            output='screen',
            parameters=[config],
            condition=IfCondition(LaunchConfiguration('start_mocap_bridge')),
        ),
        Node(
            package='svg_ground_control',
            executable='diffaero_velocity_commander',
            name='diffaero_velocity_commander',
            output='screen',
            parameters=commander_params,
            condition=IfCondition(LaunchConfiguration('start_commander')),
        ),
    ]


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare('svg_ground_control'), 'config', 'diffaero_vel_real.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='Parameter YAML (commander + mocap_bridge sections)'),
        DeclareLaunchArgument(
            'scenario', default_value='',
            description='Override scenario: hover, goal, random_walk, ...'),
        DeclareLaunchArgument(
            'policy', default_value='',
            description='Policy preset (checkpoint + velocity caps) overlaid on '
                        'the config: sha2c, planar_mlp, planar_cnn, planar_rcnn. '
                        'Empty keeps the config YAML checkpoint.'),
        DeclareLaunchArgument(
            'drones', default_value='drone_1',
            description='Comma-separated names for real_interfaces (must match VOXL -n)'),
        DeclareLaunchArgument(
            'natnet_server_ip', default_value='192.168.50.5',
            description='OptiTrack Motive PC IP (NatNet server)'),
        DeclareLaunchArgument(
            'natnet_client_ip', default_value='192.168.50.2',
            description='This ground PC IP on the Motive/NatNet network'),
        DeclareLaunchArgument(
            'start_natnet', default_value='true',
            description='Launch natnet_ros2 node'),
        DeclareLaunchArgument(
            'start_interfaces', default_value='true',
            description='Launch px4_interface via real_interfaces.launch.py'),
        DeclareLaunchArgument(
            'start_mocap_bridge', default_value='true',
            description='Launch mocap_bridge (external vision → PX4)'),
        DeclareLaunchArgument(
            'start_commander', default_value='true',
            description='Launch diffaero_velocity_commander (IDLE until ~/takeoff)'),
        OpaqueFunction(function=launch_setup),
    ])
