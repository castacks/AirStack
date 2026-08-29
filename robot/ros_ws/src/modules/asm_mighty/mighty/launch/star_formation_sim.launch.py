# /* ----------------------------------------------------------------------------
#  * Copyright 2025, Kota Kondo, Aerospace Controls Laboratory
#  * Massachusetts Institute of Technology
#  * All Rights Reserved
#  * Authors: Kota Kondo, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */
#
# Star-formation fake-sim demo (5 drones, no Gazebo).
#
# Spawns 5 agents at the vertices of a regular pentagon centered at the origin,
# configures each with the formation cost so they try to hold the pentagon
# shape throughout their flight, then publishes a shared /swarm_goal after a
# startup delay so the whole swarm starts flying through the random forest
# simulator environment.
#
# Usage:
#   ros2 launch mighty star_formation_sim.launch.py
#
# Optional overrides:
#   ros2 launch mighty star_formation_sim.launch.py radius:=2.5 formation_weight:=50.0 goal_x:=10 goal_y:=0

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


NUM_AGENTS = 5
AGENT_PREFIX = 'NX'


def _pentagon_vertices(radius):
    """Return [(x, y), ...] for 5 regular-pentagon vertices centered at origin.

    Agent 0 (NX01) sits straight ahead on the +x axis; subsequent agents are
    placed counter-clockwise every 2*pi/5 radians.
    """
    vertices = []
    for i in range(NUM_AGENTS):
        angle = 2.0 * math.pi * i / NUM_AGENTS
        vertices.append((radius * math.cos(angle), radius * math.sin(angle)))
    return vertices


def _fmt_vec(vec):
    return ','.join(f'{v:.4f}' for v in vec)


def launch_setup(context, *args, **kwargs):
    radius = float(LaunchConfiguration('radius').perform(context))
    z0 = float(LaunchConfiguration('spawn_z').perform(context))
    formation_weight = float(LaunchConfiguration('formation_weight').perform(context))
    start_x = float(LaunchConfiguration('start_x').perform(context))
    start_y = float(LaunchConfiguration('start_y').perform(context))
    goal_x = float(LaunchConfiguration('goal_x').perform(context))
    goal_y = float(LaunchConfiguration('goal_y').perform(context))
    goal_delay = float(LaunchConfiguration('goal_delay').perform(context))
    spawn_delay = float(LaunchConfiguration('spawn_delay').perform(context))
    map_size_x = float(LaunchConfiguration('map_size_x').perform(context))
    map_size_y = float(LaunchConfiguration('map_size_y').perform(context))
    num_obstacles = int(LaunchConfiguration('num_obstacles').perform(context))

    vertices = _pentagon_vertices(radius)

    actions = []

    # 1) Simulator (random forest + RViz). Launched immediately.
    #    random_forest_sensing.cpp centers the obstacle field at the origin
    #    (see `_x_l = -_x_size/2`), so we can't shift it asymmetrically. We
    #    pick a symmetric extent big enough to cover the desired corridor and
    #    let the agents fly straight through it.
    simulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('mighty'), 'launch', 'simulator.launch.py'])
        ),
        launch_arguments={
            'map_size_x_': f'{map_size_x:.2f}',
            'map_size_y_': f'{map_size_y:.2f}',
            'p_num': str(num_obstacles),
        }.items(),
    )
    actions.append(simulator_launch)

    # 2) Five per-agent onboard_mighty launches, included with a short delay so
    #    the simulator has time to bring up the random-forest map and RViz.
    #
    #    The pentagon is centered at (start_x, start_y, z0). Each agent's
    #    formation_self_offset matches the pentagon offset from the SWARM
    #    center (NOT from the spawn center), so when `/swarm_goal` is published
    #    at (goal_x, goal_y, 1.0) each agent's terminal pin lands at
    #    (goal_x + vx, goal_y + vy, 1.0) — i.e. the pentagon is translated
    #    rigidly from start to goal.
    for i, (vx, vy) in enumerate(vertices):
        agent_id = i + 1
        namespace = f'{AGENT_PREFIX}{agent_id:02d}'

        spawn_x = start_x + vx
        spawn_y = start_y + vy

        # Face the goal heading (all agents face +x-ish, toward the goal).
        yaw_deg = math.degrees(math.atan2(goal_y - spawn_y, goal_x - spawn_x))

        # Self offset δ_i = this agent's position relative to the swarm goal.
        self_offset = [vx, vy, 0.0]

        # Neighbors = all other 4 agents; pairwise offset δ_ij = self - neighbor.
        neighbor_ids = []
        flat_offsets = []
        for j, (ux, uy) in enumerate(vertices):
            if j == i:
                continue
            neighbor_ids.append(j + 1)
            flat_offsets.extend([vx - ux, vy - uy, 0.0])

        agent_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('mighty'), 'launch', 'onboard_mighty.launch.py'
                ])
            ),
            launch_arguments={
                'namespace': namespace,
                'x': f'{spawn_x:.4f}',
                'y': f'{spawn_y:.4f}',
                'z': f'{z0:.4f}',
                'yaw': f'{yaw_deg:.4f}',
                'sim_env': 'fake_sim',
                'num_agents': str(NUM_AGENTS),
                'use_formation': 'true',
                'formation_weight': f'{formation_weight:.4f}',
                'formation_self_offset': _fmt_vec(self_offset),
                'formation_neighbor_ids': ','.join(str(n) for n in neighbor_ids),
                'formation_neighbor_offsets': _fmt_vec(flat_offsets),
            }.items(),
        )

        actions.append(TimerAction(period=spawn_delay, actions=[agent_launch]))

    # 2.5) Formation visualization node (LINE_LIST between every pair of agents).
    formation_viz_node = Node(
        package='mighty',
        executable='formation_viz_node.py',
        name='formation_viz_node',
        output='screen',
        parameters=[{
            'num_agents': NUM_AGENTS,
            'agent_prefix': AGENT_PREFIX,
            'map_frame_id': 'map',
            'line_width': 0.03,  # 3 cm, "very thin"
            'rate_hz': 30.0,     # smooth lines without stressing rclpy executor
        }],
    )
    actions.append(TimerAction(period=spawn_delay, actions=[formation_viz_node]))

    # 3) Auto-publish a shared /swarm_goal after goal_delay seconds so the user
    #    sees the swarm actually move without having to click anything.
    #
    #    The /swarm_goal subscription QoS in mighty_node is RELIABLE+VOLATILE,
    #    and `ros2 topic pub --once` is fire-and-forget: any subscriber that
    #    isn't matched by the time the publisher publishes just misses the
    #    message. With 5 agents coming up at slightly different times, this
    #    caused 3/5 agents to never receive the goal. Belt-and-suspenders fix:
    #      - --wait-matching-subscriptions NUM_AGENTS: block until all 5 subs
    #        have matched before publishing at all
    #      - --times 5 --rate 1: publish 5 times over 5 seconds anyway, in case
    #        any subscription dropped mid-publish
    goal_cmd = [
        'ros2', 'topic', 'pub',
        '--wait-matching-subscriptions', str(NUM_AGENTS),
        '--times', '5', '--rate', '1',
        '/swarm_goal',
        'geometry_msgs/msg/PoseStamped',
        (
            '{header: {frame_id: "map"}, '
            f'pose: {{position: {{x: {goal_x}, y: {goal_y}, z: 1.0}}, '
            'orientation: {w: 1.0}}}'
        ),
    ]
    actions.append(TimerAction(period=goal_delay, actions=[ExecuteProcess(cmd=goal_cmd, output='screen')]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('radius', default_value='2.5',
                              description='Pentagon circumradius [m] (controls formation size)'),
        DeclareLaunchArgument('spawn_z', default_value='1.0',
                              description='Spawn altitude [m] for all agents'),
        DeclareLaunchArgument('formation_weight', default_value='50.0',
                              description='L-BFGS weight on the J_form cost'),
        DeclareLaunchArgument('start_x', default_value='-35.0',
                              description='Pentagon center x coordinate at spawn [m] (just outside the obstacle field at x=-30)'),
        DeclareLaunchArgument('start_y', default_value='0.0',
                              description='Pentagon center y coordinate at spawn [m]'),
        DeclareLaunchArgument('goal_x', default_value='35.0',
                              description='Shared /swarm_goal x coordinate [m]'),
        DeclareLaunchArgument('goal_y', default_value='0.0',
                              description='Shared /swarm_goal y coordinate [m]'),
        # Random-forest extent. random_forest_sensing.cpp centers obstacles at
        # the origin (`_x_l = -_x_size/2`), so the field spans
        # x ∈ [-map_size_x/2, +map_size_x/2]. Both spawn (x=-35) and goal
        # (x=+35) MUST sit inside that range — the planner blows up its cost
        # to ~10^7 for agents starting outside the obstacle field (HGP global
        # path can't anchor, fopt overflows par_.fopt_threshold, no plan ever
        # publishes, the agent never moves). 90 m of extent leaves a 5 m
        # buffer past spawn/goal in x.
        DeclareLaunchArgument('map_size_x', default_value='90.0',
                              description='Random-forest extent in x [m], centered at origin'),
        DeclareLaunchArgument('map_size_y', default_value='20.0',
                              description='Random-forest extent in y [m], centered at origin'),
        # 80 obstacles in a 90×20 field is sparse enough that none of the
        # five pentagon spawn vertices around (-35, 0) ± 2.5 m land too close
        # to a tree. Bumping to 100+ would push one of NX02..NX05's spawn
        # into the inflated bbox of an obstacle and the planner can't find
        # a feasible local plan from a constrained start (fopt blows past
        # par_.fopt_threshold for that one agent). Keep at 80 unless you
        # also widen the spawn clearance.
        DeclareLaunchArgument('num_obstacles', default_value='80',
                              description='Number of random-forest obstacles'),
        DeclareLaunchArgument('spawn_delay', default_value='8.0',
                              description='Seconds to wait before bringing up the agents (gives the simulator time to start)'),
        DeclareLaunchArgument('goal_delay', default_value='20.0',
                              description='Seconds to wait before publishing the shared /swarm_goal'),
        OpaqueFunction(function=launch_setup),
    ])
