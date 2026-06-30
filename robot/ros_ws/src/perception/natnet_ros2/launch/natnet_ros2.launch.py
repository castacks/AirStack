
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import SetParametersFromFile
from launch_ros.actions import LifecycleNode
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
import lifecycle_msgs.msg
from launch.actions import EmitEvent
from launch_ros.events.lifecycle import ChangeState
from launch.events.matchers import matches_action

def node_fn(context,*args, **kwargs):
    serverIP = LaunchConfiguration('serverIP')
    clientIP = LaunchConfiguration('clientIP')
    serverType = LaunchConfiguration('serverType')
    multicastAddress = LaunchConfiguration('multicastAddress')
    serverCommandPort = LaunchConfiguration('serverCommandPort')
    serverDataPort = LaunchConfiguration('serverDataPort')
    global_frame = LaunchConfiguration('global_frame')
    remove_latency = LaunchConfiguration('remove_latency')
    pub_rigid_body = LaunchConfiguration('pub_rigid_body')
    pub_rigid_body_marker = LaunchConfiguration('pub_rigid_body_marker')
    pub_individual_marker = LaunchConfiguration('pub_individual_marker')
    pub_pointcloud = LaunchConfiguration('pub_pointcloud')
    log_internals = LaunchConfiguration('log_internals')
    log_frames = LaunchConfiguration('log_frames')
    log_latencies = LaunchConfiguration('log_latencies')
    conf_file = LaunchConfiguration('conf_file')
    node_name = LaunchConfiguration('node_name')
    activate = LaunchConfiguration('activate')
    immt = LaunchConfiguration('immt')

    params = [
            {"serverIP": serverIP}, 
            {"clientIP": clientIP}, 
            {"serverType": serverType}, 
            {"multicastAddress": multicastAddress},
            {"serverCommandPort": serverCommandPort}, 
            {"serverDataPort": serverDataPort}, 
            {"global_frame": global_frame}, 
            {"remove_latency": remove_latency},
            {"pub_rigid_body": pub_rigid_body},
            {"pub_rigid_body_marker": pub_rigid_body_marker}, 
            {"pub_individual_marker": pub_individual_marker}, 
            {"pub_pointcloud": pub_pointcloud}, 
            {"log_internals": log_internals},
            {"log_frames": log_frames},
            {"log_latencies": log_latencies},
            {"individual_marker_msg_type" : immt},
        ]
    conf_file_path = None
    if pub_individual_marker.perform(context):
        if len(conf_file.perform(context))==0 or conf_file.perform(context).split('.')[-1]!="yaml":
            raise RuntimeError("Provide yaml file for initial configuration")
        conf_file_path = os.path.join(get_package_share_directory(
        'natnet_ros2'), 'config', conf_file.perform(context))
        params.append(conf_file_path)

    ld=[]
    node = LifecycleNode(
        package="natnet_ros2",
        executable="natnet_ros2_node",
        output="screen",
        name=node_name.perform(context),
        namespace='',
        parameters=params,
        #arguments=[
        #        "--ros-args",
        #        "--disable-external-lib-logs"]
    )
    ld.append(node)
    
    driver_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
           transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    ld.append(driver_configure)
    if activate.perform(context):
        driver_activate = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
            )
        )
        ld.append(driver_activate)
    

    return ld


def generate_launch_description():

    return  LaunchDescription([
        DeclareLaunchArgument('serverIP', default_value="192.168.50.5"),
        DeclareLaunchArgument('clientIP', default_value="192.168.50.2"),
        DeclareLaunchArgument('serverType', default_value="multicast"), # multicast/unicast
        DeclareLaunchArgument('multicastAddress', default_value="239.255.42.99"),
        DeclareLaunchArgument('serverCommandPort', default_value="1510"),
        DeclareLaunchArgument('serverDataPort', default_value="1511"),
        DeclareLaunchArgument('global_frame', default_value="world"),
        DeclareLaunchArgument('remove_latency',default_value="false"),
        DeclareLaunchArgument('pub_rigid_body', default_value="true"),
        DeclareLaunchArgument('pub_rigid_body_marker', default_value="False"),
        DeclareLaunchArgument('pub_individual_marker', default_value="False"),
        DeclareLaunchArgument('pub_pointcloud', default_value="False"),
        DeclareLaunchArgument('log_internals', default_value="False"),
        DeclareLaunchArgument('log_frames', default_value="False"),
        DeclareLaunchArgument('log_latencies', default_value="False"),
        DeclareLaunchArgument('conf_file', default_value="initiate.yaml"),
        DeclareLaunchArgument('node_name', default_value="natnet_ros"),
        DeclareLaunchArgument('activate', default_value="false"),
        DeclareLaunchArgument('immt', default_value="PoseStamped",description="publish type of individual markers PoseStampted or PointStamped"),
        OpaqueFunction(function=node_fn)  
        ])
