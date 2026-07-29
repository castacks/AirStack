"""Launch the SVG trajectory commander."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare('svg_ground_control'), 'config',
         'drone_soccer',
         'trajectory_commander_drone2_waypoints.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='Trajectory commander parameter YAML'),
        Node(
            package='svg_ground_control',
            executable='trajectory_commander',
            name='trajectory_commander',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
    ])
