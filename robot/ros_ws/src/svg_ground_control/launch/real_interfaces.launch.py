"""Launch the px4_interface stack for EVERY drone from one command (hardware).

    ros2 launch svg_ground_control real_interfaces.launch.py \
        drones:=drone_2,drone_3 target_systems:=2,1

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

    target_systems_arg = LaunchConfiguration('target_systems').perform(context)
    if target_systems_arg.strip():
        try:
            target_systems = [
                int(value.strip())
                for value in target_systems_arg.split(',')
                if value.strip()
            ]
        except ValueError as exc:
            raise ValueError(
                'target_systems:= must be a comma-separated list of integers'
            ) from exc
        if len(target_systems) != len(names):
            raise ValueError(
                'target_systems:= must contain one ID for each drone'
            )
        if any(system_id < 1 or system_id > 255
               for system_id in target_systems):
            raise ValueError('target system IDs must be in the range [1, 255]')
    else:
        target_systems = [1] * len(names)

    return [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                FindPackageShare('svg_ground_control'),
                '/launch/drone_interface.launch.xml',
            ]),
            launch_arguments={
                'drone_name': name,
                'target_system': str(target_system),
            }.items(),
        )
        for name, target_system in zip(names, target_systems)
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'drones', default_value='drone_1,drone_2,drone_3',
            description='Comma-separated drone names'),
        DeclareLaunchArgument(
            'target_systems', default_value='',
            description=(
                'Comma-separated MAVLink system IDs matching drones; '
                'defaults to 1 for each drone'
            )),
        OpaqueFunction(function=launch_setup),
    ])
