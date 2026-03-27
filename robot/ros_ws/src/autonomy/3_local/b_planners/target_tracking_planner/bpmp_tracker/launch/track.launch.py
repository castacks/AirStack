# (Yunwoo) Launch file for bpmp_tracker and bpmp_predictor nodes
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('bpmp_tracker')
    
    # Config file paths
    visualization_param_file = os.path.join(pkg_share, 'config', 'VisualizationParam.yaml')
    planning_param_file = os.path.join(pkg_share, 'config', 'PlanningParam.yaml')
    predictor_param_file = os.path.join(pkg_share, 'config', 'Predictor.yaml')

    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot',
        description='Name of the robot for namespacing'
    )

    # BPMP Predictor Node (Yunwoo)
    bpmp_predictor_node = Node(
        package='bpmp_tracker',
        executable='bpmp_predictor_node',
        name='bpmp_predictor',
        namespace=LaunchConfiguration('robot_name'),
        output='screen',
        parameters=[
            predictor_param_file,
        ],
        remappings=[
            ('target_info', 'target_info'),
        ]
    )

    # BPMP Tracker Node
    bpmp_tracker_node = Node(
        package='bpmp_tracker',
        executable='bpmp_tracker_node',
        name='bpmp_tracker',
        namespace=LaunchConfiguration('robot_name'),
        output='screen',
        parameters=[
            visualization_param_file,
            planning_param_file,
        ],
        remappings=[
            ('ego_odometry', 'odometry'),
            ('target_info', 'target_info'),
        ]
    )

    return LaunchDescription([
        robot_name_arg,
        bpmp_predictor_node,
        bpmp_tracker_node,
    ])
