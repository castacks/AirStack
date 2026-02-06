import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Get the package share directory
    package_share_directory = get_package_share_directory('mission_manager')
    print(package_share_directory)

    # Define the path to the configuration file
    visualization_config_file = os.path.join(package_share_directory, 'config', 'grid_map.yaml')
    rviz_config_file = os.path.join(package_share_directory, 'config', 'mission_manager.rviz')

    config = os.path.join(
        package_share_directory,
        'config',
        'mission_manager.yaml'
        )
    
    mission_manager = Node(
        package='mission_manager',
        # namespace='mission_manager',
        executable='mission_manager_node',
        output="screen",
        name='mission_manager_node',
        parameters=[config]
    )

    # grid_map_visualization_node = Node(
    #     package='grid_map_visualization',
    #     executable='grid_map_visualization',
    #     name='grid_map_visualization',
    #     output='screen',
    #     parameters=[visualization_config_file]
    # )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    ld = LaunchDescription()

    ld.add_action(rviz2_node)
    # ld.add_action(grid_map_visualization_node)
    ld.add_action(mission_manager)

    return ld