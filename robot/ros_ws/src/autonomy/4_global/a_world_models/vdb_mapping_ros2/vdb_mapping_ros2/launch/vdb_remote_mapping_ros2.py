import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import SetParametersFromFile

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
            get_package_share_directory('vdb_mapping_ros2'),
            'config',
            'vdb_remote_params.yaml'
            )

    print(config)

    container = ComposableNodeContainer(
        name='Container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='vdb_mapping_ros2',
                plugin='vdb_mapping_ros2::vdb_mapping_ros2_component',
                name='vdb_remote_mapping',
                parameters=[config],
            )
        ],
        output='screen',
    )

    ld.add_action(container)

    return ld
