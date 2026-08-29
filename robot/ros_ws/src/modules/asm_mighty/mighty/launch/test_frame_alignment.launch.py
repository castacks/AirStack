# /* ----------------------------------------------------------------------------
#  * Copyright 2025, Kota Kondo, Aerospace Controls Laboratory
#  * Massachusetts Institute of Technology
#  * All Rights Reserved
#  * Authors: Kota Kondo, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    mighty_dir = get_package_share_directory('mighty')

    # ── 1. Simulator (map generator + RViz) ─────────────────────────────
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mighty_dir, 'launch', 'simulator.launch.py')
        ),
    )

    # ── 2. Agent RR01 (frame rotated +45° from world) ────────────────────
    #  sim_frame_offset = world→RR01/map = +45° about Z
    #  sin(22.5°) = 0.3826834, cos(22.5°) = 0.9238795
    rr01 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mighty_dir, 'launch', 'onboard_mighty.launch.py')
        ),
        launch_arguments={
            'namespace': 'RR01',
            'x': '10.0',
            'y': '0.0',
            'z': '0.5',
            'yaw': '0',
            'sim_env': 'fake_sim',
            'map_frame_id': 'RR01/map',
            'num_agents': '2',
            'use_frame_alignment': 'true',
            'sim_frame_offset_qz': '0.3826834',
            'sim_frame_offset_qw': '0.9238795',
        }.items(),
    )

    # ── 3. Agent RR02 (frame rotated -45° from world) ─────────────────────
    #  sim_frame_offset = world→RR02/map = -45° about Z
    #  Total offset between RR01 and RR02: 90°
    rr02 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mighty_dir, 'launch', 'onboard_mighty.launch.py')
        ),
        launch_arguments={
            'namespace': 'RR02',
            'x': '-10.0',
            'y': '0.0',
            'z': '0.5',
            'yaw': '0',
            'sim_env': 'fake_sim',
            'map_frame_id': 'RR02/map',
            'num_agents': '2',
            'use_frame_alignment': 'true',
            'sim_frame_offset_qz': '-0.3826834',
            'sim_frame_offset_qw': '0.9238795',
        }.items(),
    )

    # ── 4. Frame-alignment publishers (90° yaw offset) ───────────────────
    #  T^{RR01/map}_{RR02/map}: RR02/map → RR01/map = 90° CCW rotation about Z
    fa_rr01_rr02 = Node(
        package='mighty',
        executable='frame_align_publisher.py',
        name='fa_rr01_rr02',
        output='screen',
        parameters=[{
            'ego_name': 'RR01',
            'other_name': 'RR02',
            'tx': 0.0,
            'ty': 0.0,
            'tz': 0.0,
            'qx': 0.0,
            'qy': 0.0,
            'qz': 0.7071068,
            'qw': 0.7071068,
        }],
    )

    #  T^{RR02/map}_{RR01/map}: RR01/map → RR02/map = -90° rotation about Z (inverse)
    fa_rr02_rr01 = Node(
        package='mighty',
        executable='frame_align_publisher.py',
        name='fa_rr02_rr01',
        output='screen',
        parameters=[{
            'ego_name': 'RR02',
            'other_name': 'RR01',
            'tx': 0.0,
            'ty': 0.0,
            'tz': 0.0,
            'qx': 0.0,
            'qy': 0.0,
            'qz': -0.7071068,
            'qw': 0.7071068,
        }],
    )

    # ── Static TFs so RViz can use "map" as fixed frame ────────────────
    #  map → RR01/map  (identity)
    static_tf_map_rr01 = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_map_rr01', output='screen',
        arguments=['0','0','0','0','0','0','1', 'map', 'RR01/map'],
    )
    #  map → RR02/map  (identity — fake_sim data is already in world frame)
    static_tf_map_rr02 = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_map_rr02', output='screen',
        arguments=['0','0','0','0','0','0','1', 'map', 'RR02/map'],
    )

    # ── 5. Goal monitors (use_hardware=false: identity → map = {agent}/map)
    goal_monitor_rr01 = Node(
        package='mighty',
        executable='goal_monitor_node.py',
        namespace='RR01',
        name='goal_monitor_node',
        output='screen',
        parameters=[{
            'use_hardware': False,
            'num_agents': 2,
            'radius': 10.0,
        }],
    )

    goal_monitor_rr02 = Node(
        package='mighty',
        executable='goal_monitor_node.py',
        namespace='RR02',
        name='goal_monitor_node',
        output='screen',
        parameters=[{
            'use_hardware': False,
            'num_agents': 2,
            'radius': 10.0,
        }],
    )

    return LaunchDescription([
        # Simulator + static TFs first (so RViz has "map" frame immediately)
        simulator,
        static_tf_map_rr01,
        static_tf_map_rr02,
        # RR01 after 3 s
        TimerAction(period=3.0, actions=[rr01]),
        # RR02 after 8 s
        TimerAction(period=8.0, actions=[rr02]),
        # Frame-alignment publishers after 10 s
        TimerAction(period=10.0, actions=[fa_rr01_rr02, fa_rr02_rr01]),
        # Goal monitors after 15 s
        TimerAction(period=15.0, actions=[goal_monitor_rr01, goal_monitor_rr02]),
    ])
