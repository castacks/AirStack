"""Launch the SVG ground controller.

    # Simulation (MAVROS interfaces, no mocap):
    ros2 launch svg_ground_control ground_control.launch.py

    # Hardware (px4_interface + mocap bridge):
    ros2 launch svg_ground_control ground_control.launch.py \
        config:=<path>/swarm_real.yaml use_mocap:=true

The keyboard teleop is NOT started here — it needs its own TTY:
    ros2 run svg_ground_control keyboard_teleop
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare('svg_ground_control'), 'config', 'swarm_sim.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='Swarm parameter YAML (swarm_sim.yaml or swarm_real.yaml)'),
        DeclareLaunchArgument(
            'use_mocap', default_value='false',
            description='Start the mocap bridge (hardware only)'),

        Node(
            package='svg_ground_control',
            executable='swarm_commander',
            name='swarm_commander',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
        Node(
            package='svg_ground_control',
            executable='mocap_bridge',
            name='mocap_bridge',
            output='screen',
            parameters=[LaunchConfiguration('config')],
            condition=IfCondition(LaunchConfiguration('use_mocap')),
        ),
    ])
