import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Assume the child launch files are in the same directory as this file
    current_dir = os.path.dirname(os.path.realpath(__file__))

    # Paths to the child launch files 43002058
    launch_file_path = os.path.join(current_dir, 'zed_camera.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
                'pose_cam_serial',
                default_value=TextSubstitution(text='41591402'),
                description='the camera serial number used for pose estimation'),
        DeclareLaunchArgument(
                'wire_cam_serial',
                default_value=TextSubstitution(text='43002058'),
                description='the camera serial number used for wire tracking'),
        DeclareLaunchArgument(
            'log_level',
            default_value=TextSubstitution(text='warn'),
            description='Logging level (e.g., debug, info, warn, error, fatal)'),
 
        # Wire camera launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file_path),
            launch_arguments={
                'namespace': 'robot_1/sensors',
                'camera_name': 'front_stereo',
                'camera_model': 'zedx',
                'serial_number': LaunchConfiguration('wire_cam_serial'),
               'config_path': os.path.join(get_package_share_directory('zed_wrapper'),'config','wire_common.yaml'),
            }.items()
        ),
    ])
