"""Explicitly requested simulation rehearsal: reset PID, take off, hold, land."""
import argparse,json,time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from airstack_msgs.msg import TrajectoryXYZVYaw,WaypointXYZVYaw
from airstack_msgs.srv import TrajectoryMode
from task_msgs.action import TakeoffTask,LandTask

p=argparse.ArgumentParser();p.add_argument('--height',type=float,default=.6);p.add_argument('--hold',type=float,default=10)
p.add_argument('--wait-for-land',action='store_true')
a=p.parse_args();root=Path('/root/AirStack/robot/ros_ws/ws2_runtime')
rclpy.init();node=rclpy.create_node('ws2_flight_check',parameter_overrides=[Parameter('use_sim_time',value=True)])
latest=None
trace=[]
def observe(msg):
    global latest
    latest=msg
sub=node.create_subscription(Odometry,'/robot_1/odometry_conversion/odometry',observe,qos_profile_sensor_data)
reset=node.create_publisher(Empty,'/robot_1/control/reset_integrators',1)
pub=node.create_publisher(TrajectoryXYZVYaw,'/robot_1/trajectory_controller/trajectory_override',1)
mode=node.create_client(TrajectoryMode,'/robot_1/trajectory_controller/set_trajectory_mode')
takeoff=ActionClient(node,TakeoffTask,'/robot_1/tasks/takeoff');land=ActionClient(node,LandTask,'/robot_1/tasks/land')
def spin():
    if (root/'flight_guard.json').exists():raise RuntimeError('physics guard: '+(root/'flight_guard.json').read_text())
    rclpy.spin_once(node,timeout_sec=.03)
    try:
        state=json.loads((root/'scene_status.json').read_text())
        if not trace or trace[-1]['sim_time']!=state['sim_time']:trace.append(state)
    except (OSError,ValueError):pass
def future(f,t=15):
    deadline=time.monotonic()+t
    while not f.done() and time.monotonic()<deadline:spin()
    if not f.done():raise TimeoutError('ROS action/service timeout')
    return f.result()
def set_mode(value):
    if not mode.wait_for_service(timeout_sec=10):raise RuntimeError('trajectory mode unavailable')
    req=TrajectoryMode.Request();req.mode=value
    if not future(mode.call_async(req)).success:raise RuntimeError('trajectory mode rejected')
def action(client,goal):
    if not client.wait_for_server(timeout_sec=10):raise RuntimeError('action unavailable')
    handle=future(client.send_goal_async(goal))
    if not handle.accepted:raise RuntimeError('action rejected')
    try:result=future(handle.get_result_async(),120)
    except Exception:
        handle.cancel_goal_async();raise
    if result.status!=GoalStatus.STATUS_SUCCEEDED or not result.result.success:raise RuntimeError(str(result.result))
deadline=time.monotonic()+15
while latest is None and time.monotonic()<deadline:spin()
if latest is None:raise RuntimeError('no odometry')
try:
    set_mode(TrajectoryMode.Request.ROBOT_POSE)
    for _ in range(30):reset.publish(Empty());spin()
    goal=TakeoffTask.Goal();goal.target_altitude_m=a.height;goal.velocity_m_s=.15
    print('PID integrators reset; takeoff',flush=True);action(takeoff,goal)
    set_mode(TrajectoryMode.Request.ADD_SEGMENT)
    target=latest.pose.pose.position
    start=node.get_clock().now().nanoseconds/1e9
    (root/'hover_ready.json').write_text(json.dumps({'ready':True,'sim_time':start,'requested_height':a.height}))
    while node.get_clock().now().nanoseconds/1e9-start<a.hold:
        if a.wait_for_land and (root/'land_request').exists():break
        traj=TrajectoryXYZVYaw();traj.header=latest.header
        for _ in range(2):
            wp=WaypointXYZVYaw();wp.position=target;wp.velocity=.15;wp.yaw=0.;traj.waypoints.append(wp)
        pub.publish(traj);spin()
    print('Hover complete; landing',flush=True)
    goal=LandTask.Goal();goal.velocity_m_s=.15;action(land,goal)
    (root/'flight_check_result.json').write_text(json.dumps({'success':True,'height':a.height,'hold_sim_s':a.hold}))
    print('Landed',flush=True)
except Exception as exc:
    (root/'flight_check_result.json').write_text(json.dumps({'success':False,'error':str(exc)}))
    raise
finally:
    (root/'flight_trace.json').write_text(json.dumps(trace))
    node.destroy_node();rclpy.shutdown()
