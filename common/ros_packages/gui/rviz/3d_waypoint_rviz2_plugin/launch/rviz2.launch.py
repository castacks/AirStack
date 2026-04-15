"""
RViz2 launch file for waypoint plugin demonstration
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('waypoint_rviz2_plugin')
    
    rviz_config_file = os.path.join(pkg_share, 'config', 'waypoint_demo.rviz')
    
    use_rviz_config = os.path.exists(rviz_config_file)
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file] if use_rviz_config else [],
        output='screen'
    )

    return LaunchDescription([
        rviz_node
    ])
