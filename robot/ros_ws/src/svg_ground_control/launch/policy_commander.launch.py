"""Launch the drone_soccer PPO policy commander (direct FMU pose backend)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    config = LaunchConfiguration('config')
    model_path = LaunchConfiguration('model_path').perform(context).strip()
    params = [config]
    if model_path:
        params.append({'model_path': model_path})
    return [
        Node(
            package='svg_ground_control',
            executable='policy_commander',
            name='policy_commander',
            output='screen',
            parameters=params,
        ),
    ]


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare('svg_ground_control'), 'config',
         'drone_soccer', 'policy_commander_drone4.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='Policy commander parameter YAML'),
        DeclareLaunchArgument(
            'model_path', default_value='',
            description='SB3 checkpoint .zip (overrides YAML when non-empty)'),
        OpaqueFunction(function=_launch_setup),
    ])
