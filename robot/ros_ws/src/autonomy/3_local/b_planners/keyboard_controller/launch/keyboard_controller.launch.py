from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, EnvironmentVariable, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
# (Yunwoo)
def generate_launch_description():
    env_robot_name = EnvironmentVariable('ROBOT_NAME', default_value='robot_0')
    robot_name = LaunchConfiguration('robot_name')
    return LaunchDescription([
        DeclareLaunchArgument('robot_name',default_value=env_robot_name, description="Robot Name from environment variable"),
        Node(
            package = "keyboard_controller",
            executable = 'keyboard_controller_node',
            name='keyboard_controller',
            output="screen",
            remappings=[
                (
                    'tracking_point', [TextSubstitution(text='/'),robot_name,TextSubstitution(text='/trajectory_controller/tracking_point')]
                ),
            ],
            ),
    ])

