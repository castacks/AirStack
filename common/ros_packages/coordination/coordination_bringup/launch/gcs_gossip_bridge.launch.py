"""Launches the GCS-side DDS Router bridging /gossip/peers between GCS domain (0) and gossip domain (99)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('coordination_bringup'),
        'config', 'gcs_gossip_dds_router.yaml',
    )
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ddsrouter', '-c', config],
            env={
                **os.environ,
                # ddsrouter runtime libs are installed under /usr/local/lib.
                # Scope this path to ddsrouter to avoid changing ROS 2 RMW resolution.
                'LD_LIBRARY_PATH': '/usr/local/lib:' + os.environ.get('LD_LIBRARY_PATH', ''),
            },
            output='screen',
            name='gcs_gossip_dds_router',
        ),
    ])
