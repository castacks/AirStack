from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mission_manager',
            # namespace='mission_manager',
            executable='mission_manager_node',
            output="screen",
            name='mission_manager_node',
        )
    ])