"""Launch the DiffAero single-drone VELOCITY-command commander.

Sibling of diffaero_single.launch.py for a velocity-command policy
(dynamics=velocity_pointmass). See config/diffaero_vel_sim.yaml.

    # Simulation (default):
    ros2 launch svg_ground_control diffaero_velocity_single.launch.py

    # Override scenario:
    ros2 launch svg_ground_control diffaero_velocity_single.launch.py scenario:=goal

    # Hardware (px4_interface + mocap bridge):
    ros2 launch svg_ground_control diffaero_velocity_single.launch.py \\
        config:=<share>/config/diffaero_vel_real.yaml use_mocap:=true

    # Select a different trained policy (overlays checkpoint_path + velocity caps
    # on top of the config YAML). Valid names: see POLICIES below.
    ros2 launch svg_ground_control diffaero_velocity_single.launch.py policy:=planar_rcnn
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Policy presets — a short name maps to the parameter overlay applied on top of
# the base config YAML. Empty `policy:=` keeps whatever checkpoint_path the YAML
# declares (default: sha2c_vel_cmd_oa). Planar checkpoints emit a 2-D action and
# are slow indoor cruisers, so cap velocity conservatively (no max_vel_z — the
# planar wrapper ignores it).
CKPT = "/root/AirStack/robot/ros_ws/checkpoints/diffaero"
POLICIES = {
    "sha2c":       {"checkpoint_path": f"{CKPT}/sha2c_vel_cmd_oa/"},
    "planar_mlp":  {"checkpoint_path": f"{CKPT}/planar_mlp_sr0.96/",
                    "max_vel": 1.5, "max_vel_xy": 1.5},
    "planar_cnn":  {"checkpoint_path": f"{CKPT}/planar_cnn_sr0.97/",
                    "max_vel": 1.5, "max_vel_xy": 1.5},
    "planar_rcnn": {"checkpoint_path": f"{CKPT}/planar_rcnn_sr0.97/",
                    "max_vel": 1.5, "max_vel_xy": 1.5},
}


def launch_setup(context, *args, **kwargs):
    config = LaunchConfiguration('config')
    scenario = LaunchConfiguration('scenario').perform(context)
    policy = LaunchConfiguration('policy').perform(context)

    commander_params = [config]
    if scenario:
        commander_params.append({'scenario': scenario})
    if policy:
        if policy not in POLICIES:
            raise RuntimeError(
                f"Unknown policy '{policy}'. Valid: {', '.join(POLICIES)}")
        commander_params.append(dict(POLICIES[policy]))

    return [
        Node(
            package='svg_ground_control',
            executable='diffaero_velocity_commander',
            name='diffaero_velocity_commander',
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
    ]


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare('svg_ground_control'), 'config', 'diffaero_vel_sim.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='Parameter YAML for diffaero_velocity_commander'),
        DeclareLaunchArgument(
            'scenario', default_value='',
            description='Override scenario from config: hover, goal, random_walk, ...'),
        DeclareLaunchArgument(
            'policy', default_value='',
            description='Policy preset (checkpoint + velocity caps) overlaid on '
                        'the config: sha2c, planar_mlp, planar_cnn, planar_rcnn. '
                        'Empty keeps the config YAML checkpoint.'),
        DeclareLaunchArgument(
            'use_mocap', default_value='false',
            description='Start the mocap bridge (hardware only)'),
        OpaqueFunction(function=launch_setup),
    ])
