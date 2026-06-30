"""Launch the SVG ground controller.

    # Simulation (MAVROS interfaces, no mocap), scenario from the config:
    ros2 launch svg_ground_control ground_control.launch.py

    # Override the scenario from the CLI:
    ros2 launch svg_ground_control ground_control.launch.py scenario:=squeeze

    # Hardware (px4_interface + mocap bridge):
    ros2 launch svg_ground_control ground_control.launch.py \
        config:=<path>/swarm_real.yaml use_mocap:=true

The keyboard teleop is NOT started here — it needs its own TTY:
    ros2 run svg_ground_control keyboard_teleop --ros-args -p drone:=drone_3
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    config = LaunchConfiguration('config')
    scenario = LaunchConfiguration('scenario').perform(context)

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
            'use_mocap', default_value='True',
            description='Start the mocap bridge (hardware only)'),
        OpaqueFunction(function=launch_setup),
    ])
