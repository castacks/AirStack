from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="mavros",
                node_namespace="mavros",
                node_executable="apm.launch",
                node_name="mavros_ardupilot",
            )
        ]
    )
