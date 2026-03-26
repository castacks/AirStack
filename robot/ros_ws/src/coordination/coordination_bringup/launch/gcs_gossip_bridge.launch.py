"""
gcs_gossip_bridge.launch.py
============================
Launches the GCS-side DDS Router that bridges /gossip/peers between the GCS
domain (0) and the shared gossip bus domain (99).

Include this from gcs.launch.xml so the GCS can receive PeerProfile messages
from all robots without going through each robot's onboard DDS router.
"""

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
            output='screen',
            name='gcs_gossip_dds_router',
        ),
    ])
