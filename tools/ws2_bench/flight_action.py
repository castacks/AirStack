"""Noninteractive existing AirStack actions; does not substitute planner/control."""
import argparse,json,time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from task_msgs.action import TakeoffTask,LandTask
from airstack_msgs.srv import TrajectoryMode

p=argparse.ArgumentParser();p.add_argument('action',choices=['takeoff','land','pause'])
p.add_argument('--height',type=float,default=1.2);a=p.parse_args()
rclpy.init();n=rclpy.create_node('ws2_episode_action',parameter_overrides=[Parameter('use_sim_time',value=True)])
latest_odom=None;connected=False
def odom(m):
    global latest_odom
    latest_odom=m
def state(m):
    global connected
    connected=m.connected
odom_sub=n.create_subscription(Odometry,'/robot_1/odometry_conversion/odometry',odom,qos_profile_sensor_data)
state_sub=n.create_subscription(State,'/robot_1/interface/mavros/state',state,qos_profile_sensor_data)
def await_future(f,timeout):
    end=time.monotonic()+timeout
    while not f.done() and time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.05)
        if Path('/root/AirStack/robot/ros_ws/ws2_runtime/flight_guard.json').exists():raise RuntimeError('simulator flight guard stopped physics')
    if not f.done():raise TimeoutError('action/service wall watchdog')
    return f.result()
def mode(value):
    c=n.create_client(TrajectoryMode,'/robot_1/trajectory_controller/set_trajectory_mode')
    if not c.wait_for_service(timeout_sec=15):raise RuntimeError('trajectory service unavailable')
    r=TrajectoryMode.Request();r.mode=value
    for attempt in range(3):
        f=c.call_async(r)
        try:
            if not await_future(f,5).success:raise RuntimeError('mode rejected')
            return
        except TimeoutError:
            f.cancel()
            if attempt==2:raise
try:
    if a.action=='pause':mode(TrajectoryMode.Request.PAUSE)
    else:
        kind=TakeoffTask if a.action=='takeoff' else LandTask
        c=ActionClient(n,kind,'/robot_1/tasks/'+a.action)
        if not c.wait_for_server(timeout_sec=15):raise RuntimeError('flight action missing')
        if a.action=='takeoff':
            end=time.monotonic()+30
            fresh_since=None
            while True:
                rclpy.spin_once(n,timeout_sec=.05)
                now=n.get_clock().now().nanoseconds/1e9
                fresh=latest_odom is not None and connected and -.05<=now-(latest_odom.header.stamp.sec+latest_odom.header.stamp.nanosec*1e-9)<.5
                if fresh:
                    if fresh_since is None:fresh_since=now
                    if now-fresh_since>=2:break
                else:fresh_since=None
                if time.monotonic()>end:raise TimeoutError('fresh FCU/odometry readiness')
            mode(TrajectoryMode.Request.ROBOT_POSE)
            reset=n.create_publisher(Empty,'/robot_1/control/reset_integrators',1)
            for _ in range(30):reset.publish(Empty());rclpy.spin_once(n,timeout_sec=.03)
        g=kind.Goal();g.velocity_m_s=.15
        if a.action=='takeoff':g.target_altitude_m=a.height
        h=await_future(c.send_goal_async(g),15)
        if not h.accepted:raise RuntimeError('action rejected')
        r=await_future(h.get_result_async(),120)
        if r.status!=GoalStatus.STATUS_SUCCEEDED or not r.result.success:raise RuntimeError(str(r.result))
        if a.action=='takeoff':mode(TrajectoryMode.Request.ADD_SEGMENT)
    print(json.dumps({'success':True,'action':a.action}),flush=True)
finally:n.destroy_node();rclpy.shutdown()
