from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
# (Yunwoo)
def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')
    num_robot = LaunchConfiguration('num_robot')
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='robot_0', description='ROBOT ID'),
        DeclareLaunchArgument('num_robot', default_value='1', description='THE NUMBER OF AGENTS'),
        Node(
            package = "multi_agent_bridge",
            executable = 'multi_agent_bridge',
            name='multi_agent_bridge',
            output="screen",
            parameters=[{'robot_name': robot_name,
                         'num_robot': num_robot,
                         'local_map_frame_id': 'map',
                         'local_mavros_frame_id': 'mavros_enu',
                         'global_map_frame_id': 'world',
                         'x_size': 0.5,'y_size': 0.5,'z_size': 1.0,
                         'neighbor_mask': True}],
            condition=IfCondition(PythonExpression([num_robot, ' > 1']))
            ),
    ])
