"""Synthetic ROS smoke test; never launch on a vehicle's ROS domain.

Run with built executable and config paths as arguments. This script publishes
only to /rrm_pid_test on an explicitly isolated, otherwise empty domain197.
No robot interface, arming service, or MAVROS is launched.
"""
import json
import os
import subprocess
import sys
import time
import copy
import math

if os.environ.get('ROS_DOMAIN_ID') != '197':
    raise SystemExit('Required isolated ROS_DOMAIN_ID=197')

import rclpy
from airstack_msgs.msg import Odometry as TrackingPoint
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from rclpy.qos import qos_profile_sensor_data
from mav_msgs.msg import RollPitchYawrateThrust
from pid_controller_msgs.msg import PIDInfo

rclpy.init()
node = rclpy.create_node('lifecycle_probe', namespace='/rrm_pid_test')
for _ in range(10):
    rclpy.spin_once(node, timeout_sec=.05)
if any(name != 'lifecycle_probe' for name, _ in node.get_node_names_and_namespaces()):
    raise SystemExit('Domain is not empty: refusing to publish')

armed_pub = node.create_publisher(Bool, 'is_armed', 1)
control_pub = node.create_publisher(Bool, 'has_control', 1)
odom_pub = node.create_publisher(Odometry, 'odometry', 1)
tp_pub = node.create_publisher(TrackingPoint, 'tracking_point', 1)
infos, commands = {}, []
vz_history = []
diagnostics = []
ARMED_RECEIPT_INVALID=1<<2
TRACKING_FUTURE=1<<8
ODOM_FUTURE=1<<10
ODOM_STALE=1<<11
TRACKING_TF_FAILED=1<<12
ODOM_TF_FAILED=1<<13
def receive_diagnostic(msg):
    d=json.loads(msg.data, parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
    assert d['schema']=='pid-admission/v1'
    assert d['active'] == (d['reason_mask']==0)
    assert not diagnostics or d['sequence']>diagnostics[-1]['sequence'], d
    for key in ('armed_receipt_age_s','control_receipt_age_s','odom_receipt_age_s',
                'tracking_receipt_age_s','tracking_gap_s'):
        assert d[key] is None or math.isfinite(d[key]),d
    assert all(key in d for key in ('ros_now_ns','tracking_stamp_ns','odom_stamp_ns',
                                   'steady_now_s','history_reset_mask','phase')),d
    diagnostics.append(d)
def receive_info(axis, msg):
    infos[axis] = msg
    if axis == 'vz':
        vz_history.append(msg)
subs = [node.create_subscription(PIDInfo, f'{axis}_pid_info',
    lambda msg, axis=axis: receive_info(axis, msg), 10)
    for axis in ('x', 'y', 'z', 'vx', 'vy', 'vz')]
subs.append(node.create_subscription(RollPitchYawrateThrust, 'command', commands.append, 10))
subs.append(node.create_subscription(String, 'admission_diagnostic', receive_diagnostic, qos_profile_sensor_data))

process = subprocess.Popen([sys.argv[1], '--ros-args', '--params-file', sys.argv[2],
    '-r', '__ns:=/rrm_pid_test', '-p', 'target_frame:=map'])
results = []

def phase(name, duration, armed=None, control=None, odom=True, stale_tp=False, bad_tf=False,
          future_tp=False, future_odom=False, stale_odom=False, tp_offset_ns=0,
          bad_odom_tf=False):
    start = time.monotonic()
    first_index = len(vz_history)
    diagnostic_index = len(diagnostics)
    while time.monotonic() - start < duration:
        stamp = node.get_clock().now().to_msg()
        if armed is not None:
            armed_pub.publish(Bool(data=armed))
        if control is not None:
            control_pub.publish(Bool(data=control))
        if odom:
            msg = Odometry()
            msg.header.frame_id = msg.child_frame_id = 'map'
            if bad_odom_tf: msg.header.frame_id='missing_odom_transform_frame'
            msg.header.stamp = copy.deepcopy(stamp)
            if future_odom: msg.header.stamp.sec += 1
            if stale_odom: msg.header.stamp.sec -= 2
            msg.pose.pose.orientation.w = 1.
            odom_pub.publish(msg)
        tp = TrackingPoint()
        tp.header.frame_id = tp.child_frame_id = 'map'
        if bad_tf:
            tp.header.frame_id = 'missing_transform_frame'
        tp.header.stamp = copy.deepcopy(stamp)
        if stale_tp:
            tp.header.stamp.sec -= 2
        if future_tp:
            tp.header.stamp.sec += 1
        if tp_offset_ns:
            total=tp.header.stamp.sec*10**9+tp.header.stamp.nanosec+tp_offset_ns
            tp.header.stamp.sec,tp.header.stamp.nanosec=divmod(total,10**9)
        tp.pose.orientation.w = 1.
        tp.pose.position.x = tp.pose.position.z = .2
        tp_pub.publish(tp)
        until = time.monotonic() + .05
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.005)
    assert process.poll() is None, 'Controller exited'
    assert len(infos) == 6 and commands, 'Missing controller output'
    result = dict(phase=name, integrals={a:i.integral for a,i in infos.items()},
                  thrust=commands[-1].thrust.z,
                  roll=commands[-1].roll, pitch=commands[-1].pitch,
                  yaw_rate=commands[-1].yaw_rate)
    recent=diagnostics[diagnostic_index:]
    assert recent, ('missing callback diagnostics', name)
    result['reason_masks'] = sorted({d['reason_mask'] for d in recent})
    result['reason_examples'] = list({d['reason_mask']:d for d in recent}.values())
    result['last_diagnostic'] = recent[-1]
    results.append(result)
    active_samples = [msg for msg in vz_history[first_index:] if msg.target > 0.]
    if name in ('active', 'fresh-reactivation', 'reactivate', 'reactivate-again',
                'active-before-odom-expiry', 'active-before-transform-failure',
                'reactivation-after-transform-failure', 'reactivation-after-future-stamp',
                'reactivation-after-clock-cases','reactivation-after-odom-transform'):
        assert active_samples, result
        first = active_samples[0]
        assert first.dt == 0. and first.i_component == 0. and first.d_component == 0., name
        result['first_active_vz_dt'] = first.dt
        result['first_active_vz_i'] = first.i_component
    return result

def reason(result, mask):
    matches=[d for d in result['reason_examples'] if d['reason_mask'] & mask == mask]
    assert matches,result
    result['matching_diagnostic']=matches[-1]
    return result

def idle(result):
    assert all(abs(v) < 1e-12 for v in result['integrals'].values()), result
    assert abs(result['thrust'] - .71) < 1e-9, result
    assert all(abs(result[k]) < 1e-12 for k in ('roll','pitch','yaw_rate')), result

try:
    idle(phase('missing-state', 1.5))
    idle(phase('armed-only', .7, armed=True))
    result = phase('active', .8, armed=True, control=True)
    assert result['integrals']['vz'] > 0., result
    idle(phase('disarm', .3, armed=False, control=True))
    idle(phase('rearm-without-new-odometry', .7, armed=True, control=True, odom=False))
    assert phase('fresh-reactivation', .6, armed=True, control=True)['integrals']['vz'] > 0.
    idle(phase('control-false', .3, armed=True, control=False))
    assert phase('reactivate', .6, armed=True, control=True)['integrals']['vz'] > 0.
    idle(phase('control-state-expired', .8, armed=True))
    assert phase('reactivate-again', .6, armed=True, control=True)['integrals']['vz'] > 0.
    idle(phase('armed-state-expired', .8, control=True))
    idle(phase('stale-tracking-stamp', .7, armed=True, control=True, stale_tp=True))
    assert phase('active-before-odom-expiry', .6, armed=True, control=True)['integrals']['vz'] > 0.
    idle(phase('odometry-expired-while-active', .8, armed=True, control=True, odom=False))
    assert phase('active-before-transform-failure', .6, armed=True, control=True)['integrals']['vz'] > 0.
    failed=reason(phase('transform-failure', .8, armed=True, control=True, bad_tf=True),TRACKING_TF_FAILED)
    assert failed['matching_diagnostic']['phase']=='tf'
    idle(failed)
    assert phase('reactivation-after-transform-failure', .6, armed=True, control=True)['integrals']['vz'] > 0.
    idle(reason(phase('future-tracking-stamp', .7, armed=True, control=True, future_tp=True),TRACKING_FUTURE))
    assert phase('reactivation-after-future-stamp', .6, armed=True, control=True)['integrals']['vz'] > 0.
    idle(reason(phase('future-odometry-stamp', .5, armed=True, control=True, future_odom=True),ODOM_FUTURE))
    idle(reason(phase('stale-odometry-stamp', .5, armed=True, control=True, stale_odom=True),ODOM_STALE))
    idle(reason(phase('future-tracking-stale-odom', .5, armed=True, control=True,
                      future_tp=True, stale_odom=True),TRACKING_FUTURE|ODOM_STALE))
    idle(reason(phase('future-tracking-expired-authority', .8, armed=None, control=True,
                      future_tp=True),TRACKING_FUTURE|ARMED_RECEIPT_INVALID))
    assert phase('reactivation-after-clock-cases', .6, armed=True, control=True)['integrals']['vz'] > 0.
    failed=reason(phase('odom-transform-failure', .6, armed=True, control=True,bad_odom_tf=True),ODOM_TF_FAILED)
    assert failed['matching_diagnostic']['phase']=='tf'
    idle(failed)
    assert phase('reactivation-after-odom-transform', .6, armed=True, control=True)['integrals']['vz'] > 0.
    print(json.dumps(dict(status='PASS', domain=197, phases=results), indent=2))
finally:
    process.terminate()
    process.wait(timeout=5)
    node.destroy_node()
    rclpy.shutdown()
