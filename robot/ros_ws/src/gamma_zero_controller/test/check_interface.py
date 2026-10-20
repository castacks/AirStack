import json
import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from gamma_zero_msgs.msg import SelectedCommand
from geometry_msgs.msg import Transform, Twist
from mav_msgs.msg import RateThrust
from px4_msgs.msg import VehicleRatesSetpoint, TrajectorySetpoint, OffboardControlMode
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint

rclpy.init()
n=Node('interface_fixture')
rates=[]; trajectories=[]; heartbeats=[]
n.create_subscription(VehicleRatesSetpoint,'/bv_test/interface/in/vehicle_rates_setpoint',lambda m: rates.append(m),qos_profile_sensor_data)
n.create_subscription(TrajectorySetpoint,'/bv_test/interface/in/trajectory_setpoint',lambda m: trajectories.append(m),qos_profile_sensor_data)
n.create_subscription(OffboardControlMode,'/bv_test/interface/in/offboard_control_mode',lambda m: heartbeats.append(m),qos_profile_sensor_data)
pub=n.create_publisher(SelectedCommand,'/bv_test/interface/selected_command',1)
legacy=n.create_publisher(RateThrust,'/bv_test/interface/rate_thrust_command',1)
process=subprocess.Popen(['ros2','run','robot_interface','robot_interface_node','--ros-args','-r','__ns:=/bv_test/interface','-p','interface:=px4_interface::PX4Interface','-p','gamma_managed:=true'],stdout=open('/runtime/interface-fixture.log','w'),stderr=subprocess.STDOUT,start_new_session=True)


def spin(duration,publish=None):
    end=time.monotonic()+duration
    while time.monotonic()<end:
        if publish: publish()
        rclpy.spin_once(n,timeout_sec=.02)
        time.sleep(.01)


try:
    spin(3.)
    assert process.poll() is None
    spin(.3,lambda: legacy.publish(RateThrust()))
    assert not rates and not heartbeats
    msg=SelectedCommand(); msg.mode=0; msg.header.frame_id='map'
    point=MultiDOFJointTrajectoryPoint(); t=Transform();t.translation.z=1.3;t.rotation.w=1.
    point.transforms=[t]; point.velocities=[Twist()]; msg.trajectory.points=[point]
    def send():
        msg.header.stamp=n.get_clock().now().to_msg();pub.publish(msg)
    spin(.3,send)
    assert trajectories and heartbeats[-1].position and not heartbeats[-1].body_rate
    msg.mode=1; msg.generation=1;msg.rates.thrust.z=.15
    msg.rates.angular_rates.x=.1;msg.rates.angular_rates.y=.2;msg.rates.angular_rates.z=.3
    spin(.3,send)
    assert rates and abs(rates[-1].pitch+.2)<1e-6 and abs(rates[-1].yaw+.3)<1e-6
    assert abs(rates[-1].thrust_body[2]+.15)<1e-6 and heartbeats[-1].body_rate and not heartbeats[-1].position
    spin(.3)
    count=len(heartbeats);spin(.3);assert len(heartbeats)==count
    before=len(rates)
    spin(.2,lambda: pub.publish(msg))
    assert len(rates)==before
    other=Node('conflicting_owner'); otherpub=other.create_publisher(SelectedCommand,'/bv_test/interface/selected_command',1)
    spin(1.)
    before=len(rates);spin(.2,send);assert len(rates)==before
    other.destroy_node()
    print(json.dumps({'legacy_rejected':True,'trajectory_forwarded':True,'ctbr_axes_verified':True,'heartbeat_timeout':True,'replayed_stamp_rejected':True,'conflicting_owner_rejected':True}))
finally:
    os.killpg(process.pid,signal.SIGTERM)
    try:process.wait(timeout=5)
    except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
    n.destroy_node();rclpy.shutdown()
