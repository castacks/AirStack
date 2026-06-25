"""Launch the DiffAero single-drone commander.

    # Simulation (default — MAVROS interfaces, no mocap):
    ros2 launch svg_ground_control diffaero_single.launch.py

    # Point at a different config:
    ros2 launch svg_ground_control diffaero_single.launch.py \\
        config:=<path>/diffaero_sim.yaml

    # Hardware (px4_interface + mocap bridge):
    ros2 launch svg_ground_control diffaero_single.launch.py \\
        config:=<share>/config/diffaero_real.yaml use_mocap:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare('svg_ground_control'), 'config', 'diffaero_sim.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='Parameter YAML for the diffaero_commander node'),
        DeclareLaunchArgument(
            'use_mocap', default_value='false',
            description='Start the mocap bridge (hardware only)'),

        Node(
            package='svg_ground_control',
            executable='diffaero_commander',
            name='diffaero_commander',
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
