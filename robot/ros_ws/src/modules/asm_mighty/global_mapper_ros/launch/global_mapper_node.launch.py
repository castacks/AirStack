#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    #––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
    # launch‐time arguments
    hardware               = LaunchConfiguration('hardware')
    ground_robot           = LaunchConfiguration('ground_robot')
    quad                   = LaunchConfiguration('quad')
    param_file             = LaunchConfiguration('param_file')
    use_obstacle_tracker   = LaunchConfiguration('use_obstacle_tracker')
    depth_pointcloud_topic = LaunchConfiguration('depth_pointcloud_topic')
    pose_type              = LaunchConfiguration('pose_type')
    global_frame           = LaunchConfiguration('global_frame')
    drone_frame            = LaunchConfiguration('drone_frame')
    pose_topic             = LaunchConfiguration('pose_topic')
    goal_topic             = LaunchConfiguration('goal_topic')
    odom_topic             = LaunchConfiguration('odom_topic')
    occupancy_grid_topic   = LaunchConfiguration('occupancy_grid_topic')
    unknown_grid_topic     = LaunchConfiguration('unknown_grid_topic')
    frontier_grid_topic    = LaunchConfiguration('frontier_grid_topic')
    distance_grid_topic    = LaunchConfiguration('distance_grid_topic')
    cost_grid_topic        = LaunchConfiguration('cost_grid_topic')
    path_topic             = LaunchConfiguration('path_topic')
    dynamic_grid_topic     = LaunchConfiguration('dynamic_grid_topic')
    sparse_path_topic      = LaunchConfiguration('sparse_path_topic')

    # Pick param file from (hardware, ground_robot) — one of four variants:
    #   sim_uav.yaml, sim_ground_robot.yaml, hw_uav.yaml, hw_ground_robot.yaml
    # Override explicitly via `param_file:=...` if you need something else.
    default_param_file = PythonExpression([
        "('hw_' if '", hardware, "' == 'true' else 'sim_') + ",
        "('ground_robot' if '", ground_robot, "' == 'true' else 'uav') + '.yaml'"
    ])
    default_pose_type = PythonExpression([
        "'pose_stamped' if '", hardware, "' == 'true' else 'state'"
    ])
    default_global_frame = PythonExpression([
        "'", quad, "/map' if '", hardware, "' == 'true' else 'map'"
    ])
    default_drone_frame = PythonExpression([
        "'", quad, "/base_link' if '", hardware, "' == 'true' else ''"
    ])

    param_file_launch_arg = DeclareLaunchArgument('param_file', default_value=default_param_file, description='name of your params file (in cfg/)')

    # where to find your .yaml
    param_file_path = PathJoinSubstitution([
        FindPackageShare('global_mapper_ros'),
        'cfg',
        param_file
    ])

    return LaunchDescription([
        #––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
        DeclareLaunchArgument('hardware',             default_value='false',                         description='set true for hardware experiments'),
        DeclareLaunchArgument('ground_robot',         default_value='false',                         description='set true for ground-robot vehicle type (picks ground_robot/hw_ground_robot yaml)'),
        DeclareLaunchArgument('use_obstacle_tracker', default_value='true',                          description='enable temporal grid, dynamic grid publishing, and obstacle tracker'),
        DeclareLaunchArgument('quad',                 default_value='NX01',                          description='name of the quad / ros namespace'),
        DeclareLaunchArgument('load_params',          default_value='true',                          description='whether to load the YAML params'),
        DeclareLaunchArgument('depth_pointcloud_topic', default_value='mid360_PointCloud2',          description='input pointcloud topic'),
        DeclareLaunchArgument('pose_type',            default_value=default_pose_type,               description='pose message type: "state" or "pose_stamped"'),
        DeclareLaunchArgument('global_frame',         default_value=default_global_frame,            description='global TF frame'),
        DeclareLaunchArgument('drone_frame',          default_value=default_drone_frame,              description='drone TF frame (empty = auto from namespace)'),
        DeclareLaunchArgument('pose_topic',           default_value='state',                         description='input pose topic'),
        DeclareLaunchArgument('goal_topic',           default_value='/move_base_simple/goal',        description='input goal topic'),
        DeclareLaunchArgument('odom_topic',           default_value='odometry/filtered_no',          description='input odometry topic'),
        DeclareLaunchArgument('occupancy_grid_topic', default_value='occupancy_grid',                description='output occupancy grid topic'),
        DeclareLaunchArgument('unknown_grid_topic',   default_value='unknown_grid',                  description='output unknown grid topic'),
        DeclareLaunchArgument('frontier_grid_topic',  default_value='frontier_grid',                 description='output frontier grid topic'),
        DeclareLaunchArgument('distance_grid_topic',  default_value='distance_grid',                 description='output distance grid topic'),
        DeclareLaunchArgument('cost_grid_topic',      default_value='cost_grid',                     description='output cost grid topic'),
        DeclareLaunchArgument('path_topic',           default_value='path',                          description='output path topic'),
        DeclareLaunchArgument('dynamic_grid_topic',   default_value='dynamic_grid',                  description='output dynamic grid topic'),
        DeclareLaunchArgument('sparse_path_topic',    default_value='sparse_path',                   description='output sparse path topic'),
        param_file_launch_arg,

        #––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
        Node(
            package='global_mapper_ros',
            executable='global_mapper_node',
            name='global_mapper_ros',
            namespace=quad,
            output='screen',
            parameters=[param_file_path, {
                'pose_type': pose_type,
                'global_frame': global_frame,
                'drone_frame': drone_frame,
                'temporal_grid.use_temporal_grid': use_obstacle_tracker,
                'temporal_grid.publish_dynamic_grid': use_obstacle_tracker,
                'obstacle_tracker.enabled': use_obstacle_tracker,
            }],
            remappings=[
                ('depth_pointcloud_topic',    depth_pointcloud_topic),
                ('pose_topic',           pose_topic),
                ('goal_topic',           goal_topic),
                ('odom_topic',           odom_topic),
                ('occupancy_grid_topic', occupancy_grid_topic),
                ('unknown_grid_topic',   unknown_grid_topic),
                ('frontier_grid_topic',  frontier_grid_topic),
                ('distance_grid_topic',  distance_grid_topic),
                ('cost_grid_topic',      cost_grid_topic),
                ('path_topic',           path_topic),
                ('dynamic_grid_topic',   dynamic_grid_topic),
                ('sparse_path_topic',    sparse_path_topic),
            ],
        ),
    ])
