"""Launch the px4_interface stack for EVERY drone from one command (hardware).

    ros2 launch svg_ground_control real_interfaces.launch.py \
        drones:=drone_1,drone_2,drone_3

Includes drone_interface.launch.xml once per name — no more one terminal per
drone. Requires each drone's uXRCE-DDS client to be namespaced to match
(``uxrce_dds_client start -n drone_i`` on the VOXL).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    drones = LaunchConfiguration('drones').perform(context)
    names = [n.strip() for n in drones.split(',') if n.strip()]
    if not names:
        raise ValueError('drones:= must list at least one drone name')

    return [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                FindPackageShare('svg_ground_control'),
                '/launch/drone_interface.launch.xml',
            ]),
            launch_arguments={'drone_name': name}.items(),
        )
        for name in names
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'drones', default_value='drone_1,drone_2,drone_3',
            description='Comma-separated drone names'),
        OpaqueFunction(function=launch_setup),
    ])
