"""Deterministic ROS clock ordering on empty domain197, no vehicle interfaces."""
import json
import os
import subprocess
import sys
import time

if os.environ.get('ROS_DOMAIN_ID') != '197':
    raise SystemExit('Required empty isolated domain197')
import rclpy
from airstack_msgs.msg import Odometry as Tracking
from nav_msgs.msg import Odometry
from pid_controller_msgs.msg import PIDInfo
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, String
from builtin_interfaces.msg import Time
from mav_msgs.msg import RollPitchYawrateThrust
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
node=rclpy.create_node('clock_order_probe',namespace='/rrm_clock_test')
for _ in range(10): rclpy.spin_once(node,timeout_sec=.05)
if any(name!='clock_order_probe' for name,_ in node.get_node_names_and_namespaces()):
    raise SystemExit('Domain not empty; refusing synthetic publications')
clock=node.create_publisher(Clock,'/clock',10)
armed=node.create_publisher(Bool,'is_armed',1)
control=node.create_publisher(Bool,'has_control',1)
odom=node.create_publisher(Odometry,'odometry',1)
tracking=node.create_publisher(Tracking,'tracking_point',1)
diagnostics=[]; commands=[]; infos=[]
subscriptions=[node.create_subscription(String,'admission_diagnostic',lambda m:diagnostics.append(json.loads(m.data)),qos_profile_sensor_data),
    node.create_subscription(RollPitchYawrateThrust,'command',commands.append,10),
    node.create_subscription(PIDInfo,'vz_pid_info',infos.append,10)]
proc=subprocess.Popen([sys.argv[1],'--ros-args','--params-file',sys.argv[2],
    '-r','__ns:=/rrm_clock_test','-p','target_frame:=map','-p','use_sim_time:=true'])
def stamp(ns):
    sec,nsec=divmod(ns,10**9); return Time(sec=sec,nanosec=nsec)
def spin(duration):
    until=time.monotonic()+duration
    while time.monotonic()<until: rclpy.spin_once(node,timeout_sec=.005)
def phase(name,now_ns,tp_ns,odom_ns,mask):
    clock.publish(Clock(clock=stamp(now_ns)))
    spin(.15) # Establish clock before intentionally reordered data.
    first=len(diagnostics); first_info=len(infos)
    for _ in range(8):
        clock.publish(Clock(clock=stamp(now_ns)))
        armed.publish(Bool(data=True)); control.publish(Bool(data=True))
        o=Odometry(); o.header.stamp=stamp(odom_ns); o.header.frame_id=o.child_frame_id='map'; o.pose.pose.orientation.w=1.
        odom.publish(o)
        t=Tracking(); t.header.stamp=stamp(tp_ns); t.header.frame_id=t.child_frame_id='map'; t.pose.orientation.w=1.; t.pose.position.z=.2
        tracking.publish(t); spin(.04)
    assert proc.poll() is None and len(diagnostics)>first and commands
    d=diagnostics[-1]
    assert d['ros_now_ns']==now_ns and d['tracking_stamp_ns']==tp_ns and d['odom_stamp_ns']==odom_ns,d
    assert d['reason_mask']==mask,d
    assert d['tf_failure'] is None, 'old TF failure leaked into later callback'
    if mask:
        assert commands[-1].thrust.z==.71 and infos[-1].integral==0.,d
    else:
        valid=[i for i in infos[first_info:] if i.target>0]
        assert valid and valid[0].dt==0. and valid[0].i_component==0.,name
        assert commands[-1].thrust.z>.71,name
    return {'phase':name,'diagnostic':d,'thrust':commands[-1].thrust.z}

def delivery_catchup(base):
    def publish(now_ns, tracking_ns):
        clock.publish(Clock(clock=stamp(now_ns))); spin(.015)
        armed.publish(Bool(data=True)); control.publish(Bool(data=True))
        o=Odometry(); o.header.stamp=stamp(now_ns)
        o.header.frame_id=o.child_frame_id='map'; o.pose.pose.orientation.w=1.
        odom.publish(o); spin(.01)
        t=Tracking(); t.header.stamp=stamp(tracking_ns)
        t.header.frame_id=t.child_frame_id='map'; t.pose.orientation.w=1.; t.pose.position.z=.2
        tracking.publish(t)
    publish(base,base); spin(.04)
    publish(base+30_000_000,base+30_000_000); spin(.04)
    integral_before=infos[-1].integral
    assert integral_before>0., infos[-1]
    publish(base+30_000_000,base+30_000_000); spin(.04)
    assert infos[-1].dt==0. and infos[-1].integral==integral_before
    count=len(commands)
    publish(base+30_000_000,base+60_000_000); spin(.02)
    assert len(commands)==count, 'future target emitted before clock catchup'
    clock.publish(Clock(clock=stamp(base+60_000_000))); spin(.04)
    assert len(commands)==count+1, 'pending target must process exactly once'
    assert diagnostics[-1]['reason_mask']==0 and infos[-1].integral>integral_before
    assert diagnostics[-1]['tracking_receipt_age_s']>0., 'original receipt must survive buffering'
    publish(base+60_000_000,base+90_000_000); spin(.15)
    assert diagnostics[-1]['reason_mask']&256 and infos[-1].integral==0.
    first=len(diagnostics)
    started=time.monotonic()
    for _ in range(4):
        publish(base+60_000_000,base+90_000_000); spin(.02)
    assert len(diagnostics)>first and diagnostics[-1]['reason_mask']&256
    assert time.monotonic()-started<.3, 'replacement must not extend the wait window'
    publish(base+60_000_000,base+90_000_000); spin(.01)
    armed.publish(Bool(data=False)); control.publish(Bool(data=False)); spin(.04)
    assert diagnostics[-1]['reason_mask']&3 and infos[-1].integral==0.
    return {'phase':'bounded-delivery-catchup-expiry-authority-loss','status':'PASS'}
try:
    deadline=time.monotonic()+5.
    while clock.get_subscription_count()<1 or tracking.get_subscription_count()<1:
        assert time.monotonic()<deadline, 'PID subscriptions did not discover'
        spin(.05)
    spin(.2)
    base=1700*10**9
    results=[phase('future-tracking30ms',base,base+30_000_000,base,256),
        phase('clock-catches-tracking',base+30_000_000,base+30_000_000,base+30_000_000,0),
        phase('future-odom30ms',base+60_000_000,base+60_000_000,base+90_000_000,1024),
        phase('clock-catches-odom',base+90_000_000,base+90_000_000,base+90_000_000,0),
        phase('backward-clock-both-future',base,base+90_000_000,base+90_000_000,256|1024),
        phase('clock-restored',base+90_000_000,base+90_000_000,base+90_000_000,0),
        delivery_catchup(base+120_000_000)]
    print(json.dumps({'status':'PASS','domain':197,'phases':results},indent=2))
finally:
    proc.terminate(); proc.wait(timeout=5)
    node.destroy_node(); rclpy.shutdown()
