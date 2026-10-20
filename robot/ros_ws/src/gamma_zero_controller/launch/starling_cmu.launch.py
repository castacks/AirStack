import json
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_nodes(context):
    simulation = LaunchConfiguration('simulation').perform(context).lower() == 'true'
    config_path = LaunchConfiguration('config').perform(context)
    config = json.loads(Path(config_path).read_text())
    if not simulation and not all(config.get(k, False) for k in ['transforms_verified', 'thrust_verified', 'firmware_verified']):
        raise ValueError('Live interface launch requires completed hardware verification')
    interface = '/starling_cmu_test/interface' if simulation else '/drone_3/fmu'
    return [
        Node(package='robot_interface', executable='robot_interface_node', namespace=interface,
             parameters=[{'interface': 'px4_interface::PX4Interface', 'gamma_managed': True, 'offboard_command_timeout_s': .1}],
             remappings=[('selected_command', '/gamma/selected_command'), ('selected_command_status', '/gamma/interface_status')], output='screen'),
        Node(package='gamma_zero_controller', executable='controller', name='gamma_controller',
             parameters=[{'simulation': simulation, 'config': config_path, 'robot_name': 'drone_3',
                          'assets': LaunchConfiguration('assets'), 'policy_socket': LaunchConfiguration('policy_socket'),
                          'log': LaunchConfiguration('log')}],
             remappings=[('selected_command', '/gamma/selected_command'), ('selected_command_status', '/gamma/interface_status'),
                         ('vehicle_odometry', interface+'/out/vehicle_odometry'), ('svg_reference', '/gamma/svg_reference'),
                         ('svg_status', '/svg/commander_status'), ('status', '/gamma/status'), ('trace', '/gamma/trace')], output='screen'),
        Node(package='svg_ground_control', executable='swarm_commander', name='swarm_commander', parameters=[{
            'gamma_enabled': True, 'gamma_reference_mode': LaunchConfiguration('reference_mode'),
            'teleop_drones': LaunchConfiguration('teleop_drones'), 'drone_names': ['drone_3'], 'drone_modes': 'real', 'scenario': 'goal',
            'real_command_mode': 'trajectory', 'real_trajectory_command_topic_template': '/gamma/svg_reference',
            'real_robot_command_service_template': interface+'/robot_command',
            'state_topic_template': interface+'/odometry', 'hover_positions': [0.,0.,1.3],
            'drone_position_offsets': [0.,0.,0.], 'scenario_speed_mps': .5, 'goal_accel_mps2': .5,
            'goal_lead_m': .2, 'hold_lead_m': .2, 'teleop_max_speed_mps': .5, 'teleop_accel_mps2': .5,
            'cbf_max_speed_mps': .8, 'takeoff_speed_mps': .3, 'land_speed_mps': .2,
            'fence_enabled': True, 'fence_behavior': 'keep_in', 'fence_brake_accel_mps2': .5,
            'fence_min': config['position_min_enu'], 'fence_max': config['position_max_enu'],
            'takeover_twins': False}], output='screen'),
        Node(package='svg_ground_control', executable='bag_recorder', name='bag_recorder',
             parameters=[{'bag_dir': LaunchConfiguration('bag_dir'), 'record_on_start': False}], output='screen'),
    ]


def generate_launch_description():
    share = get_package_share_directory('gamma_zero_controller')
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='true'),
        DeclareLaunchArgument('reference_mode', default_value='tested_trajectory'),
        DeclareLaunchArgument('teleop_drones', default_value=''),
        DeclareLaunchArgument('config', default_value=share+'/config/simulation.json'),
        DeclareLaunchArgument('assets'), DeclareLaunchArgument('policy_socket'), DeclareLaunchArgument('log'),
        DeclareLaunchArgument('bag_dir', default_value='/runtime/bags'), OpaqueFunction(function=launch_nodes)])
