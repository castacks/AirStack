"""Launch the drone_soccer PPO policy commander (direct FMU pose backend)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    config = LaunchConfiguration('config')
    model_path = LaunchConfiguration('model_path').perform(context).strip()
    policy_type = LaunchConfiguration('policy_type').perform(context).strip()
    namespace = LaunchConfiguration('namespace')
    debug_topic_prefix = LaunchConfiguration(
        'debug_topic_prefix').perform(context).strip()
    hot_swap = LaunchConfiguration(
        'enable_policy_hot_swap').perform(context).strip().lower()
    params = [config]
    if model_path:
        params.append({'model_path': model_path})
    if policy_type:
        params.append({'policy_type': policy_type})
    if debug_topic_prefix:
        params.append({'debug_topic_prefix': debug_topic_prefix})
    if hot_swap:
        params.append({'enable_policy_hot_swap': hot_swap == 'true'})
    return [
        Node(
            package='svg_ground_control',
            executable='policy_commander',
            name='policy_commander',
            namespace=namespace,
            output='screen',
            parameters=params,
        ),
    ]


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare('svg_ground_control'), 'config',
         'drone_soccer', 'policy_commander_drone3.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='Policy commander parameter YAML'),
        DeclareLaunchArgument(
            'model_path', default_value='',
            description='SB3 checkpoint .zip (overrides YAML when non-empty)'),
        DeclareLaunchArgument(
            'policy_type', default_value='',
            description='standard or residual (overrides YAML when non-empty)'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='ROS namespace used to isolate this commander'),
        DeclareLaunchArgument(
            'debug_topic_prefix', default_value='',
            description='Debug topic prefix override'),
        DeclareLaunchArgument(
            'enable_policy_hot_swap', default_value='',
            description='true: accept online policy-memory updates on '
                        '<debug_topic_prefix>/load_policy (overrides YAML when non-empty)'),
        OpaqueFunction(function=_launch_setup),
    ])
