import json
import os
import signal
import sys
from pathlib import Path
import subprocess
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Transform, Twist
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from px4_msgs.msg import VehicleOdometry, VehicleRatesSetpoint
from std_msgs.msg import String
from std_srvs.srv import Trigger

# Override for isolated test environments; defaults to this checkout's local assets.
VALIDATION = Path(os.environ.get('STARLING_VALIDATION_DIR',
    str(Path(__file__).resolve().parents[3] / 'robot/ros_ws/.starling/validation')))
VALIDATION.mkdir(parents=True, exist_ok=True)

rclpy.init(); n=Node('backend_fixture')
statepub=n.create_publisher(VehicleOdometry,'/starling_cmu_test/interface/out/vehicle_odometry',qos_profile_sensor_data)
refpub=n.create_publisher(MultiDOFJointTrajectory,'/gamma/svg_reference',1)
svgpub=n.create_publisher(String,'/svg/commander_status',1)
statuses=[];rates=[]; traces=[]
n.create_subscription(String,'/gamma/status',lambda m: statuses.append(json.loads(m.data)),10)
n.create_subscription(String,'/gamma/trace',lambda m: traces.append(json.loads(m.data)),qos_profile_sensor_data)
n.create_subscription(VehicleRatesSetpoint,'/starling_cmu_test/interface/in/vehicle_rates_setpoint',lambda m: rates.append(m),qos_profile_sensor_data)
args=['--ros-args','-p','config:=/root/AirStack/robot/ros_ws/src/gamma_zero_controller/config/simulation.json','-p',f'assets:={VALIDATION}/starling_cmu_inputs.json','-p',f'policy_socket:={VALIDATION}/policy.sock','-p',f'log:={VALIDATION}/backend-fixture.jsonl',
      '-r','selected_command:=/gamma/selected_command','-r','selected_command_status:=/gamma/interface_status','-r','vehicle_odometry:=/starling_cmu_test/interface/out/vehicle_odometry','-r','svg_reference:=/gamma/svg_reference','-r','svg_status:=/svg/commander_status','-r','status:=/gamma/status','-r','trace:=/gamma/trace']
processes=[]
reset=0
case=sys.argv[1] if len(sys.argv)>1 else "stale"


def spin(duration,state=True,reference=True):
    end=time.monotonic()+duration
    while time.monotonic()<end:
        stamp=n.get_clock().now()
        if state:
            od=VehicleOdometry();od.timestamp=stamp.nanoseconds//1000;od.timestamp_sample=od.timestamp
            od.pose_frame=1;od.velocity_frame=1;od.q=[1.,0.,0.,0.];od.position=[0.,0.,-1.3];od.velocity=[0.,0.,0.];od.angular_velocity=[0.,0.,0.];od.reset_counter=reset
            statepub.publish(od)
        ref=MultiDOFJointTrajectory();ref.header.stamp=stamp.to_msg();ref.header.frame_id='map'
        pt=MultiDOFJointTrajectoryPoint();t=Transform();t.translation.z=1.3;t.rotation.z=np.sqrt(.5);t.rotation.w=np.sqrt(.5)
        pt.transforms=[t];pt.velocities=[Twist()];ref.points=[pt]
        if reference: refpub.publish(ref)
        svgpub.publish(String(data=json.dumps({'mission_active':False,'drones':[{'name':'drone_3','state':'ACTIVE'}]})))
        rclpy.spin_once(n,timeout_sec=.002);time.sleep(.01)


def call(name):
    client=n.create_client(Trigger,'/gamma_controller/'+name)
    assert client.wait_for_service(timeout_sec=3)
    future=client.call_async(Trigger.Request())
    end=time.monotonic()+3
    while not future.done() and time.monotonic()<end:spin(.02)
    assert future.done()
    response=future.result();assert response.success,response.message
    return response


try:
    (VALIDATION / 'backend-fixture.jsonl').unlink(missing_ok=True)
    processes.append(subprocess.Popen(['ros2','run','robot_interface','robot_interface_node','--ros-args','-r','__ns:=/starling_cmu_test/interface','-p','interface:=px4_interface::PX4Interface','-p','gamma_managed:=true','-r','selected_command:=/gamma/selected_command','-r','selected_command_status:=/gamma/interface_status'],stdout=open(f'{VALIDATION}/backend-interface.log','w'),stderr=subprocess.STDOUT,start_new_session=True))
    processes.append(subprocess.Popen(['ros2','run','gamma_zero_controller','controller']+args,stdout=open(f'{VALIDATION}/backend-node.log','w'),stderr=subprocess.STDOUT,start_new_session=True))
    spin(5.)
    assert all(p.poll() is None for p in processes)
    assert statuses and statuses[-1]['mode']=='conventional',statuses[-1:]
    call('engage');spin(.5)
    assert statuses[-1]['mode']=='ctbr' and statuses[-1]['ack']['mode']==1,statuses[-1]
    assert rates
    call('start_trajectory');spin(.3)
    assert statuses[-1]['source']=='tested_trajectory'
    call('hold');spin(.3);assert statuses[-1]['mode']=='ctbr' and statuses[-1]['source']=='hold'
    call('handback');spin(.3);assert statuses[-1]['mode']=='conventional' and statuses[-1]['ack']['mode']==0
    call('use_svg');spin(1.2);call('engage');spin(.3)
    if case=='reference':
        spin(.35,reference=False)
    elif case=='ack':
        os.killpg(processes[0].pid,signal.SIGTERM);processes[0].wait(timeout=5);spin(.4)
    elif case=='worker':
        (VALIDATION / 'stop-worker').touch();spin(.4)
        assert statuses[-1]['mode']=='conventional' and statuses[-1]['fault'],statuses[-1]
    elif case=='reset':
        reset=1;spin(.35)
    else:
        spin(.35,state=False)
    assert statuses[-1]['mode'] in ('fault','conventional') and statuses[-1]['fault'],statuses[-1]
    expected={'stale':'State stale','reset':'Estimator reset','worker':'timed out','reference':'SVG reference','ack':'acknowledgement'}
    assert expected[case] in statuses[-1]['fault'],statuses[-1]
    before=len(rates);spin(.4);assert len(rates)==before
    assert traces, 'Diagnostic subscription must receive trace records'
    print(json.dumps({'local_policy_ctbr_messages':len(rates),'trajectory_start':True,'hold_on_policy':True,'acknowledged_handback':True,'fault_latches':statuses[-1]['fault'],'trace_records':len(traces)}))
finally:
    for p in reversed(processes):
        if p.poll() is None: os.killpg(p.pid,signal.SIGTERM)
        try:p.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
    n.destroy_node();rclpy.shutdown()
