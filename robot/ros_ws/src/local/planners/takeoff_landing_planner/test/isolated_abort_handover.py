"""Actual action-node regression with synthetic telemetry/services, domain198 only."""
import json
import os
import subprocess
import sys
import time

if os.environ.get('ROS_DOMAIN_ID') != '198':
    raise SystemExit('Required empty isolated ROS_DOMAIN_ID=198')
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from mavros_msgs.msg import ExtendedState
from airstack_msgs.msg import Odometry as Tracking, TrajectoryXYZVYaw
from airstack_msgs.srv import TrajectoryMode, RobotCommand
from task_msgs.action import TakeoffTask, LandTask

rclpy.init()
node = rclpy.create_node('abort_probe', namespace='/rrm_abort_test')
for _ in range(10):
    rclpy.spin_once(node, timeout_sec=.05)
if any(name != 'abort_probe' for name,_ in node.get_node_names_and_namespaces()):
    raise SystemExit('Domain not empty; refusing synthetic publications')
odom = node.create_publisher(Odometry, 'odometry', qos_profile_sensor_data)
tracking = node.create_publisher(Tracking, 'tracking_point', 10)
armed = node.create_publisher(Bool, 'is_armed', 10)
control = node.create_publisher(Bool, 'has_control', 10)
extended = node.create_publisher(ExtendedState, 'extended_state', 10)
events = []
scenario = {}
def mode_callback(request, response):
    events.append(('mode', request.mode))
    if request.mode == TrajectoryMode.Request.TRACK and scenario.get('delay_track'):
        time.sleep(2.5)
    if scenario.get('delay_hold') and scenario.get('ascent') and request.mode == 1:
        time.sleep(2.5)
    response.success = not (scenario.get('bad_hold') and scenario.get('ascent') and request.mode == 1)
    if request.mode == TrajectoryMode.Request.TRACK and scenario.get('reject_track'):
        response.success = False
    return response
def command_callback(request, response):
    events.append(('command', request.command))
    if request.command == RobotCommand.Request.ARM and scenario.get('delay_arm'):
        time.sleep(2.5)
    if request.command == RobotCommand.Request.REQUEST_CONTROL:
        scenario['requested'] = True
        if scenario.get('delay_request'):
            time.sleep(2.5)
    if request.command == 4 and scenario.get('delay_land'):
        time.sleep(2.5)
    response.success = not (request.command == 4 and scenario.get('reject_land'))
    if request.command == RobotCommand.Request.REQUEST_CONTROL and scenario.get('reject_request'):
        response.success = False
    if request.command == RobotCommand.Request.ARM and scenario.get('reject_arm'):
        response.success = False
    return response
mode_service = node.create_service(TrajectoryMode, 'set_trajectory_mode', mode_callback)
command_service = node.create_service(RobotCommand, 'robot_command', command_callback)
def trajectory(msg):
    events.append(('trajectory', len(msg.waypoints)))
    scenario['ascent'] = True
    if scenario.get('not_sent') and not scenario.get('ordinary'):
        node.destroy_service(command_service)
trajectory_sub = node.create_subscription(TrajectoryXYZVYaw, 'trajectory_override', trajectory, 10)
takeoff = ActionClient(node, TakeoffTask, '/rrm_abort_test/takeoff_landing_task/takeoff_task')
land = ActionClient(node, LandTask, '/rrm_abort_test/takeoff_landing_task/land_task')

def pump(duration):
    until = time.monotonic() + duration
    while time.monotonic() < until:
        stamp = node.get_clock().now().to_msg()
        msg = Odometry(); msg.header.frame_id='map'; msg.child_frame_id='base_link'
        msg.header.stamp=stamp; msg.pose.pose.orientation.w=1.
        if scenario.get('ascent'):
            msg.pose.pose.position.z = 1.5 if scenario.get('bound')=='altitude' else .5
            if scenario.get('success'):
                msg.pose.pose.position.z = 1.
            msg.pose.pose.position.x = .5 if scenario.get('bound')=='horizontal' else 0.
            msg.twist.twist.linear.z = 2. if scenario.get('bound')=='speed' else 0.
        odom.publish(msg)
        tp=Tracking(); tp.header=msg.header; tp.pose=msg.pose.pose
        tracking.publish(tp)
        if not scenario.get('missing_armed'):
            armed.publish(Bool(data=not (scenario.get('reject_arm') or scenario.get('delay_arm'))))
        if not scenario.get('missing_control') and not (
            scenario.get('stale_pre_request') and scenario.get('requested')) and not (
            scenario.get('stale_ascent') and scenario.get('ascent')):
            control.publish(Bool(data=not scenario.get('false_control') and not (
                scenario.get('false_ascent') and scenario.get('ascent'))))
        extended.publish(ExtendedState(landed_state=1 if scenario.get('ground') else 2))
        end=time.monotonic()+.03
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.005)

def await_future(future,timeout=9.):
    deadline=time.monotonic()+timeout
    while not future.done() and time.monotonic()<deadline:
        pump(.05)
    assert future.done(), 'Bounded result deadline exceeded'
    return future.result()

results=[]
scenario['success'] = True
proc=subprocess.Popen([sys.argv[1], '--ros-args', '-r', '__ns:=/rrm_abort_test',
    '-p', 'takeoff_acceptance_distance:=0.15', '-p', 'takeoff_acceptance_time:=0.1'])
try:
    assert takeoff.wait_for_server(timeout_sec=5.)
    pump(.5)
    handle=await_future(takeoff.send_goal_async(
        TakeoffTask.Goal(target_altitude_m=1., velocity_m_s=.5)))
    assert handle.accepted
    terminal=await_future(handle.get_result_async())
    assert terminal.result.success, terminal.result
    trajectory_at=next(i for i,e in enumerate(events) if e[0]=='trajectory')
    assert not any(e==('mode',1) for e in events[trajectory_at+1:]), events
    pump(.3)
    assert events[-1][0]=='trajectory', 'success must retain the endpoint trajectory'
    results.append({'success_endpoint_retained':True,'events':list(events)})
finally:
    proc.terminate(); proc.wait(timeout=5)
    pump(.2)
# Accepted services must not substitute for observed, fresh flight authority.
for case in ['false_control', 'stale_pre_request', 'missing_control', 'missing_armed',
             'cancel_acquisition', 'false_ascent', 'stale_ascent',
             'reject_request', 'delay_request', 'reject_track', 'delay_track',
             'reject_arm', 'delay_arm']:
    scenario.clear(); scenario[case] = True
    if case == 'cancel_acquisition':
        scenario['false_control'] = True
    events.clear()
    proc=subprocess.Popen([sys.argv[1], '--ros-args', '-r', '__ns:=/rrm_abort_test',
        '-p', 'control_acquisition_timeout_s:=0.7', '-p', 'control_state_max_age_s:=0.2'])
    try:
        assert takeoff.wait_for_server(timeout_sec=5.)
        pump(.4)
        handle=await_future(takeoff.send_goal_async(TakeoffTask.Goal(target_altitude_m=1.,velocity_m_s=.5)))
        assert handle.accepted
        future=handle.get_result_async()
        started=time.monotonic()
        if case == 'cancel_acquisition':
            deadline=time.monotonic()+3.
            while not scenario.get('requested') and time.monotonic()<deadline:
                pump(.03)
            assert scenario.get('requested'), events
            assert await_future(handle.cancel_goal_async()).goals_canceling
        terminal=await_future(future, timeout=6.)
        elapsed=time.monotonic()-started
        assert not terminal.result.success and elapsed < 6., (case,terminal,events)
        assert ('command',4) in events, events
        after_ascent=case in ('false_ascent','stale_ascent')
        trajectories=[e for e in events if e[0]=='trajectory']
        assert len(trajectories) == (1 if after_ascent else 0), (case,events)
        track_requested=after_ascent or case in ('reject_track','delay_track')
        assert (('mode',2) in events) == track_requested, (case,events)
        expected=('arm request' if case in ('reject_arm','delay_arm') else
                  'offboard control request' if case in ('reject_request','delay_request') else
                  'TRACK transition' if case in ('reject_track','delay_track') else
                  'lost during ascent' if after_ascent else 'not observed before ascent')
        assert expected in terminal.result.message, terminal.result.message
        if case == 'cancel_acquisition':
            assert terminal.status == 5, terminal.status  # STATUS_CANCELED
        results.append(dict(authority_case=case,elapsed_s=elapsed,
                            message=terminal.result.message,events=list(events)))
    finally:
        proc.terminate(); proc.wait(timeout=5)
        pump(.2)

for bound,bad_hold,reject_land,delay_land,not_sent,delay_hold in [
    ('horizontal',False,False,False,False,False),('altitude',False,False,False,False,False),
    ('speed',False,False,False,False,False),('speed',True,False,False,False,False),
    ('speed',False,True,False,False,False),('speed',False,False,True,False,False),
    ('speed',False,False,False,False,True),('speed',False,False,False,True,False)]:
    scenario.clear(); scenario.update(bound=bound,bad_hold=bad_hold,reject_land=reject_land,
        delay_land=delay_land,not_sent=not_sent,delay_hold=delay_hold,ordinary=True)
    events.clear()
    proc=subprocess.Popen([sys.argv[1],'--ros-args','-r','__ns:=/rrm_abort_test',
        '-p','takeoff_max_horizontal_displacement:=0.3','-p','takeoff_max_altitude_overshoot:=0.3',
        '-p','takeoff_max_vertical_speed:=1.5','-p','landing_max_duration_s:=3.0'])
    try:
        assert takeoff.wait_for_server(timeout_sec=5.)
        pump(.4)
        # A fresh node's ordinary LAND still generates its bounded trajectory.
        ordinary=await_future(land.send_goal_async(LandTask.Goal(velocity_m_s=.5)))
        assert ordinary.accepted
        ordinary_result=ordinary.get_result_async()
        pump(.2)
        assert ('mode',2) in events and any(e[0]=='trajectory' for e in events),events
        scenario['ground']=True
        assert await_future(ordinary_result).result.success
        scenario['ground']=False; scenario['ascent']=False; scenario['ordinary']=False
        events.clear(); pump(.2)
        handle=await_future(takeoff.send_goal_async(TakeoffTask.Goal(target_altitude_m=1.,velocity_m_s=.5)))
        assert handle.accepted
        started=time.monotonic()
        result=await_future(handle.get_result_async()).result
        abort_elapsed=time.monotonic()-started
        assert abort_elapsed < 5., (abort_elapsed,events)
        assert not result.success and 'limit exceeded' in result.message,result
        if not not_sent:
            assert ('command',4) in events,events
            assert next(i for i,e in enumerate(events) if e[0]=='trajectory') < events.index(('command',4))
            assert ('mode',1) == events[events.index(('command',4))-1],events
        expected='NOT_SENT' if not_sent else 'FAILED_OR_UNCONFIRMED' if reject_land else 'UNCONFIRMED' if delay_land else 'ACCEPTED'
        assert 'abort_land='+expected in result.message,result.message
        assert 'grounding=UNVERIFIED' in result.message
        if bad_hold or delay_hold:
            assert 'abort_hold=UNCONFIRMED' in result.message,result.message
        if not not_sent:  # A false bool response may hide an inner MAVROS timeout.
            mark=len(events)
            recovery=await_future(land.send_goal_async(LandTask.Goal(velocity_m_s=.5)))
            assert recovery.accepted
            recovery_result=recovery.get_result_async()
            pump(.3)
            assert not recovery_result.done(), 'LAND acceptance mistaken for grounding'
            assert not events[mark:],events
            if bound=='horizontal':
                terminal=await_future(recovery_result,timeout=5.).result
                assert not terminal.success and 'timed out' in terminal.message
                assert not events[mark:],events
            elif bound=='altitude':
                assert await_future(recovery.cancel_goal_async()).goals_canceling
                assert not await_future(recovery_result).result.success
                assert not events[mark:],events
            else:
                scenario['ground']=True
                assert await_future(recovery_result).result.success
        results.append(dict(bound=bound,bad_hold=bad_hold,reject_land=reject_land,
                            delay_land=delay_land,not_sent=not_sent,delay_hold=delay_hold,
                            message=result.message,abort_elapsed_s=abort_elapsed,
                            elapsed_s=time.monotonic()-started,events=list(events)))
    finally:
        proc.terminate(); proc.wait(timeout=5)
        pump(.2)
print(json.dumps(dict(status='PASS',domain=198,scenarios=results),indent=2))
node.destroy_node(); rclpy.shutdown()
