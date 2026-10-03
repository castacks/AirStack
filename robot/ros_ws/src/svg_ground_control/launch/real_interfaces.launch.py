"""Launch the px4_interface stack for EVERY drone from one command (hardware).

    ros2 launch svg_ground_control real_interfaces.launch.py \
        drones:=drone_1,drone_2,drone_3 [target_systems:=1,2,3]

Includes drone_interface.launch.xml once per name — no more one terminal per
drone. Requires each drone's uXRCE-DDS client to be namespaced to match
(``uxrce_dds_client start -n drone_i`` on the VOXL).

``target_systems`` is each drone's PX4 ``MAV_SYS_ID`` (same order as
``drones``). PX4 silently drops VehicleCommands addressed to another system id,
so a fleet with distinct MAV_SYS_IDs (needed for QGC to show all drones) must
pass them here. Default: the trailing number of each name (drone_2 -> 2; a
name without a number -> 1).
"""

import re

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

    sysids_arg = LaunchConfiguration('target_systems').perform(context).strip()
    if sysids_arg:
        sysids = [x.strip() for x in sysids_arg.split(',')]
        if len(sysids) != len(names):
            raise ValueError(
                f'target_systems:= has {len(sysids)} entries for {len(names)} drones')
    else:
        sysids = [_default_target_system(n) for n in names]

    return [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                FindPackageShare('svg_ground_control'),
                '/launch/drone_interface.launch.xml',
            ]),
            launch_arguments={'drone_name': name,
                              'target_system': sysid}.items(),
        )
        for name, sysid in zip(names, sysids)
    ]


def _default_target_system(name):
    """drone_2 -> '2'; anything without a trailing number -> '1'."""
    m = re.search(r'(\d+)$', name)
    return m.group(1) if m else '1'


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'drones', default_value='drone_1,drone_2,drone_3',
            description='Comma-separated drone names'),
        DeclareLaunchArgument(
            'target_systems', default_value='',
            description='Comma-separated PX4 MAV_SYS_ID per drone (same order '
                        'as drones). Empty = trailing number of each name.'),
        OpaqueFunction(function=launch_setup),
    ])
