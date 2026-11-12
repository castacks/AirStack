"""Launch the online policy-memory learner (pairs with policy_commander enable_policy_hot_swap:=true)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare('svg_ground_control'), 'config',
         'drone_soccer', 'online_memory_learner_drone3.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='Online memory learner parameter YAML'),
        Node(
            package='svg_ground_control',
            executable='online_memory_learner',
            name='online_memory_learner',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
    ])
