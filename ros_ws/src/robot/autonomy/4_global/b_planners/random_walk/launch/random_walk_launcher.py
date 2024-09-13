import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
            get_package_share_directory('random_walk_planner'),
            'config',
            'random_walk_config.yaml'
            )
    random_walk_planner = Node(
        package='random_walk_planner',
        executable='random_walk_planner',
        name='random_walk_planner',
        parameters = [config]
    )

    ld.add_action(random_walk_planner)

    return ld