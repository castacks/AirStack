import json
import sys
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleOdometry, VehicleStatus
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool

rclpy.init();n=Node('svg_lifecycle_fixture')
odpub=n.create_publisher(VehicleOdometry,'/starling_cmu_test/interface/out/vehicle_odometry',qos_profile_sensor_data)
statuspub=n.create_publisher(VehicleStatus,'/starling_cmu_test/interface/out/vehicle_status',qos_profile_sensor_data)
svg=[];gamma=[];bag=[]
n.create_subscription(String,'/svg/commander_status',lambda m:svg.append(json.loads(m.data)),10)
n.create_subscription(String,'/gamma/status',lambda m:gamma.append(json.loads(m.data)),10)
n.create_subscription(String,'/svg/bag_recorder/status',lambda m:bag.append(json.loads(m.data)),10)
Path('/runtime/organization-validation/svg-fixture.jsonl').unlink(missing_ok=True)
p=subprocess.Popen(['ros2','launch','gamma_zero_controller','starling_cmu.launch.py','assets:=/runtime/organization-validation/starling_cmu_inputs.json','policy_socket:=/runtime/organization-validation/policy.sock','log:=/runtime/organization-validation/svg-fixture.jsonl','reference_mode:='+('svg' if len(sys.argv)>1 else 'tested_trajectory')],stdout=open('/runtime/organization-validation/svg-fixture.log','w'),stderr=subprocess.STDOUT,start_new_session=True)
altitude=0.


def spin(duration):
    end=time.monotonic()+duration
    while time.monotonic()<end:
        stamp=n.get_clock().now().nanoseconds//1000
        od=VehicleOdometry();od.timestamp=stamp;od.timestamp_sample=stamp;od.pose_frame=1;od.velocity_frame=1
        od.q=[float(np.sqrt(.5)),0.,0.,float(np.sqrt(.5))];od.position=[0.,0.,-altitude];od.velocity=[0.,0.,0.];od.angular_velocity=[0.,0.,0.];odpub.publish(od)
        status=VehicleStatus();status.timestamp=stamp;status.arming_state=VehicleStatus.ARMING_STATE_ARMED;status.nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD;statuspub.publish(status)
        rclpy.spin_once(n,timeout_sec=.002);time.sleep(.01)


def call(name,record=None):
    kind=Trigger if record is None else SetBool
    cl=n.create_client(kind,name);assert cl.wait_for_service(timeout_sec=5),name
    request=kind.Request()
    if record is not None:request.data=record
    future=cl.call_async(request);end=time.monotonic()+5
    while not future.done() and time.monotonic()<end:spin(.02)
    assert future.done(),name
    response=future.result();assert response.success,(name,response.message)
    return response


try:
    spin(5.)
    assert p.poll() is None
    assert svg and gamma,(svg[-1:],gamma[-1:])
    call('/bag_recorder/record',True)
    call('/swarm_commander/takeoff');spin(6.)
    altitude=1.3;spin(3.)
    assert svg[-1]['drones'][0]['state']=='ACTIVE',svg[-1]
    call('/gamma_controller/engage');spin(.5)
    call('/swarm_commander/start');spin(.5)
    assert gamma[-1]['source']==('svg' if len(sys.argv)>1 else 'tested_trajectory'),gamma[-1]
    call('/swarm_commander/hold');spin(.5)
    assert gamma[-1]['mode']=='ctbr' and gamma[-1]['source']=='hold'
    call('/swarm_commander/land');spin(1.)
    assert svg[-1]['drones'][0]['state']=='LANDING',svg[-1]
    assert gamma[-1]['mode']=='conventional' and gamma[-1]['ack']['mode']==0
    altitude=.05;spin(1.)
    assert svg[-1]['drones'][0]['state']=='IDLE',svg[-1]
    call('/bag_recorder/record',False);spin(.5)
    print(json.dumps({'synthetic_fcu_lifecycle':'takeoff-engage-start-hold-handback-land-idle','reference_mode':('svg' if len(sys.argv)>1 else 'tested_trajectory'),'bag':bag[-1] if bag else None}))
finally:
    p.send_signal(signal.SIGINT)
    try:p.wait(timeout=12)
    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
    n.destroy_node();rclpy.shutdown()

text=Path('/runtime/organization-validation/svg-fixture.log').read_text()
assert 'Fatal glibc error' not in text and 'SEVERE WARNING' not in text,text[-3000:]
