from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('bag_record_pid')

    # Declare launch arguments
    cfg_path_arg = DeclareLaunchArgument(
        'cfg_path',
        default_value=LaunchConfiguration('cfg_path', default=pkg_dir + '/config/cfg.yaml'),
        description='Configuration file for bag record pid'
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value=LaunchConfiguration('output_dir', default='/logging'),
        description='Logging directory'
    )
    
    mcap_qos_dir_arg = DeclareLaunchArgument(
        'mcap_qos_dir',
        default_value=LaunchConfiguration('mcap_qos_dir', default=pkg_dir + '/config'),
        description='MCAP QoS directory'
    )

    # Include the bag record node launch file
    bag_record_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('bag_record_pid'),
            '/launch/bag_record_pid_node.launch.py'
        ]),
        launch_arguments={
            'cfg_path': LaunchConfiguration('cfg_path'),
            'output_dir': LaunchConfiguration('output_dir'),
            'mcap_qos_dir': LaunchConfiguration('mcap_qos_dir')
        }.items()
    )

    return LaunchDescription([
        # Launch arguments
        cfg_path_arg,
        output_dir_arg,
        mcap_qos_dir_arg,
        # Include launch file
        bag_record_launch
    ])
