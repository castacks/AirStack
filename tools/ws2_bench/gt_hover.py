"""Controlled demonstration hover using simulator truth and AirStack PID.

This is a sensor/attack-interface demonstration, not learned-planner flight.
"""
import argparse,json,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import PoseStamped,TransformStamped
from nav_msgs.msg import Odometry
from airstack_msgs.msg import Odometry as Reference
from airstack_msgs.srv import RobotCommand
from std_msgs.msg import Empty
from mavros_msgs.srv import ParamSetV2
from mav_msgs.msg import RollPitchYawrateThrust
from tf2_ros import TransformBroadcaster

p=argparse.ArgumentParser();p.add_argument('--height',type=float,default=1.2);p.add_argument('--hold',type=float,default=180);p.add_argument('--wait-for-land',action='store_true')
a=p.parse_args();root=Path('/root/AirStack/robot/ros_ws/ws2_runtime')
rclpy.init();node=rclpy.create_node('ws2_hover_reference',parameter_overrides=[Parameter('use_sim_time',value=True)])
odom_pub=node.create_publisher(Odometry,'/ws2/control/odometry',1)
ref_pub=node.create_publisher(Reference,'/ws2/control/reference',1)
reset=node.create_publisher(Empty,'/ws2/control/reset_integrators',1)
tf=TransformBroadcaster(node);command=node.create_client(RobotCommand,'/robot_1/interface/robot_command')
actual=None;reference=None;velocity=np.zeros(3);stamp=0.;last_publish=-1.;last_wall=0.;px4_yaw=None;measured_yaw=0.;trace=[]
last_command_wall=0.
def command_ready(msg):
    global last_command_wall
    last_command_wall=time.monotonic()
command_sub=node.create_subscription(RollPitchYawrateThrust,'/robot_1/interface/cmd_roll_pitch_yawrate_thrust',command_ready,1)
def px4(msg):
    global px4_yaw
    q=msg.pose.orientation;px4_yaw=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz')[2]
px4_sub=node.create_subscription(PoseStamped,'/robot_1/interface/mavros/local_position/pose',px4,qos_profile_sensor_data)
def observe(msg):
    global actual,reference,velocity,stamp,last_publish,last_wall,measured_yaw
    t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9;p=msg.pose.position;point=np.array([p.x,p.y,p.z])
    if actual is not None and t>stamp:velocity=.8*velocity+.2*(point-actual)/(t-stamp)
    actual=point;stamp=t;last_wall=time.monotonic()
    if reference is None:reference=point.copy()
    if t-last_publish<.025:return
    last_publish=t;q=msg.pose.orientation;rot=Rotation.from_quat([q.x,q.y,q.z,q.w]);measured_yaw=rot.as_euler('xyz')[2]
    tr=TransformStamped();tr.header=msg.header;tr.child_frame_id='ws2_body_stabilized'
    tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z=map(float,point)
    tr.transform.rotation.x,tr.transform.rotation.y,tr.transform.rotation.z,tr.transform.rotation.w=map(float,Rotation.from_euler('z',measured_yaw).as_quat())
    body=TransformStamped();body.header=msg.header;body.child_frame_id='ws2_body'
    body.transform.translation.x,body.transform.translation.y,body.transform.translation.z=map(float,point)
    body.transform.rotation=msg.pose.orientation
    tf.sendTransform([tr,body])
    od=Odometry();od.header=msg.header;od.child_frame_id='ws2_body';od.pose.pose=msg.pose
    od.twist.twist.linear.x,od.twist.twist.linear.y,od.twist.twist.linear.z=map(float,rot.inv().apply(velocity));odom_pub.publish(od)
    ref=Reference();ref.header=msg.header;ref.child_frame_id='ws2_world'
    ref.pose.position.x,ref.pose.position.y,ref.pose.position.z=map(float,reference)
    offset=0. if px4_yaw is None else np.arctan2(np.sin(px4_yaw-measured_yaw),np.cos(px4_yaw-measured_yaw))
    ref.pose.orientation.x,ref.pose.orientation.y,ref.pose.orientation.z,ref.pose.orientation.w=map(float,Rotation.from_euler('z',offset).as_quat());ref_pub.publish(ref)
    trace.append({'sim_time':t,'position':point.tolist(),'velocity':velocity.tolist(),'reference':reference.tolist()})
sub=node.create_subscription(PoseStamped,'/ws2/ground_truth/pose',observe,qos_profile_sensor_data)
def spin():
    rclpy.spin_once(node,timeout_sec=.02)
    if (root/'flight_guard.json').exists():raise RuntimeError('physics flight guard tripped')
    if actual is not None and time.monotonic()-last_wall>2:raise RuntimeError('GT feedback stale')
def future(f,timeout=15):
    end=time.monotonic()+timeout
    while not f.done() and time.monotonic()<end:spin()
    if not f.done():raise TimeoutError('ROS service timed out')
    return f.result()
def available(client):
    end=time.monotonic()+10
    while not client.service_is_ready():
        spin()
        if time.monotonic()>end:raise RuntimeError('ROS service unavailable')
def hold(seconds):
    begin=stamp;end=time.monotonic()+seconds*10+30
    while stamp-begin<seconds:
        spin()
        if time.monotonic()>end:raise TimeoutError('simulation clock stalled')
def call(code):
    available(command)
    r=RobotCommand.Request();r.command=code
    if not future(command.call_async(r)).success:raise RuntimeError('robot command rejected')
def thrust(value):
    client=node.create_client(SetParameters,'/ws2/control/pid_controller/set_parameters')
    available(client)
    r=SetParameters.Request();r.parameters=[Parameter('vz_constant',value=value).to_parameter_msg()]
    if not all(x.successful for x in future(client.call_async(r)).results):raise RuntimeError('PID parameter rejected')
def goto(target,speed=.15):
    global reference
    previous=stamp;begin=stamp
    while np.linalg.norm(actual-target)>.07 or np.linalg.norm(velocity)>.1:
        spin();dt=max(0.,stamp-previous);previous=stamp
        delta=target-reference;d=np.linalg.norm(delta)
        candidate=reference+delta/max(d,1e-9)*min(d,speed*dt)
        if np.linalg.norm(candidate-actual)<.3:reference=candidate
        if stamp-begin>60:raise TimeoutError('hover height did not converge')
    reference=target.copy()
deadline=time.monotonic()+15
while actual is None and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
if actual is None:raise RuntimeError('no ground truth')
start=actual.copy()
try:
    thrust(.5);hold(2)
    if px4_yaw is None:raise RuntimeError('PX4 heading unavailable')
    end=time.monotonic()+10
    while time.monotonic()-last_command_wall>.5:
        spin()
        if time.monotonic()>end:raise RuntimeError('PID output missing; refusing to arm')
    for _ in range(20):reset.publish(Empty());spin()
    client=node.create_client(ParamSetV2,'/robot_1/interface/mavros/param/set')
    available(client)
    req=ParamSetV2.Request();req.param_id='SIM_BAT_MIN_PCT';req.force_set=True;req.value=Parameter('SIM_BAT_MIN_PCT',value=100.).to_parameter_msg().value
    if not future(client.call_async(req)).success:raise RuntimeError('simulation battery configuration failed')
    call(RobotCommand.Request.ARM);call(RobotCommand.Request.REQUEST_CONTROL)
    goto(start+np.array([0,0,.6]));hold(3)
    goto(start+np.array([0,0,a.height]));hold(3)
    (root/'hover_ready.json').write_text(json.dumps({'ready':True,'sim_time':stamp,'position':actual.tolist(),'feedback':'simulator GT / AirStack PID'}))
    print('Stable GT-controlled demonstration hover',flush=True)
    begin=stamp
    while stamp-begin<a.hold:
        spin()
        if a.wait_for_land and (root/'land_request').exists():break
    reference=actual.copy();begin=stamp;previous=stamp
    while actual[2]>start[2]+.007 or np.linalg.norm(velocity)>.04:
        spin();dt=max(0.,stamp-previous);previous=stamp;reference[2]=max(start[2]-.1,reference[2]-.12*dt)
        if stamp-begin>45:raise TimeoutError('landing did not settle')
    thrust(0.)
    for _ in range(10):reset.publish(Empty());spin()
    hold(3);call(RobotCommand.Request.DISARM)
    result={'success':True,'feedback':'simulator GT / AirStack PID','start':start.tolist(),'end':actual.tolist(),'max_z':max(x['position'][2] for x in trace)}
    print('Landed and disarmed',flush=True)
except Exception as exc:
    result={'success':False,'error':str(exc)}
    raise
finally:
    (root/'flight_check_result.json').write_text(json.dumps(result))
    (root/'flight_trace.json').write_text(json.dumps(trace))
    node.destroy_node();rclpy.shutdown()
