"""Integration checks against the built C++ action server, with fake FCU/services.

Run after sourcing the build with TAKEOFF_TASK_EXECUTABLE pointing at the binary.
No simulator or flight hardware is used; altitude is explicitly supplied by the test.
"""
import os
import subprocess
import threading
import time
import uuid
from contextlib import contextmanager

import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from airstack_msgs.msg import TrajectoryXYZVYaw
from airstack_msgs.srv import RobotCommand, TrajectoryMode
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from task_msgs.action import TakeoffTask


def wait_for(predicate, timeout=8):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        value = predicate()
        if value:
            return value
        time.sleep(0.02)
    raise AssertionError('Timed out waiting for ROS event')


def future_result(future):
    wait_for(future.done)
    return future.result()


@contextmanager
def server(confirmation=1.0):
    rclpy.init()
    ns = '/confirmation_' + uuid.uuid4().hex[:8]
    node = rclpy.create_node('fake_fcu', namespace=ns)
    state = {'confirmed': False, 'safety_timeout': False, 'z': 0.0,
             'overrides': 0, 'commands': [], 'modes': [], 'publish_state': True}
    pubs = {name: node.create_publisher(cls, name, 10) for name, cls in (
        ('odometry', Odometry), ('is_armed', Bool), ('has_control', Bool),
        ('state_estimate_timed_out', Bool), ('fcu_state', State))}

    def publish():
        odom = Odometry()
        odom.header.stamp = node.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.pose.pose.orientation.w = 1.0
        odom.pose.pose.position.z = state['z']
        pubs['odometry'].publish(odom)
        for name in ('is_armed', 'has_control', 'state_estimate_timed_out'):
            msg = Bool()
            msg.data = state['safety_timeout'] if name == 'state_estimate_timed_out' else state['confirmed']
            pubs[name].publish(msg)
        if state['publish_state']:
            msg = State()
            msg.connected = True
            msg.armed = state['confirmed']
            msg.mode = 'OFFBOARD' if state['confirmed'] else 'MANUAL'
            pubs['fcu_state'].publish(msg)

    def command(req, res):
        state['commands'].append(req.command)
        res.success = True  # Acknowledges sending only; does not change FCU state.
        return res

    def mode(req, res):
        state['modes'].append(req.mode)
        res.success = True
        return res

    def trajectory(msg):
        state['overrides'] += 1

    node.create_service(RobotCommand, 'robot_command', command)
    node.create_service(TrajectoryMode, 'set_trajectory_mode', mode)
    node.create_subscription(TrajectoryXYZVYaw, 'trajectory_override', trajectory, 10)
    node.create_timer(0.02, publish)
    client = ActionClient(node, TakeoffTask, ns + '/takeoff_landing_task/takeoff_task')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    binary = os.environ['TAKEOFF_TASK_EXECUTABLE']
    args = [binary, '--ros-args', '-r', '__ns:=' + ns,
            '-p', 'takeoff_acceptance_time:=0.1']
    if confirmation is not None:
        args += ['-p', f'takeoff_control_confirmation_s:={confirmation}']
    proc = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        assert client.wait_for_server(timeout_sec=10)
        time.sleep(0.3)

        def goal():
            msg = TakeoffTask.Goal()
            msg.target_altitude_m = 12.0
            msg.velocity_m_s = 1.0
            handle = future_result(client.send_goal_async(msg))
            assert handle.accepted
            return handle

        yield state, goal
    finally:
        proc.terminate()
        try:
            output = proc.communicate(timeout=4)[0]
        except subprocess.TimeoutExpired:
            proc.kill()
            output = proc.communicate()[0]
        print(output)
        executor.shutdown(timeout_sec=2)
        thread.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()


def test_acknowledgement_does_not_start_ascent():
    with server(2.0) as (state, goal):
        handle = goal()
        wait_for(lambda: len(state['commands']) == 2)
        time.sleep(0.25)
        assert state['overrides'] == 0
        assert TrajectoryMode.Request.TRACK not in state['modes']
        state['confirmed'] = True
        wait_for(lambda: state['overrides'] == 1)
        state['z'] = 12.0
        result = future_result(handle.get_result_async())
        assert result.status == 4 and result.result.success
        assert state['commands'] == [RobotCommand.Request.ARM, RobotCommand.Request.REQUEST_CONTROL]


def test_unconfirmed_control_aborts_without_trajectory():
    with server() as (state, goal):
        result = future_result(goal().get_result_async())
        assert result.status == 6 and not result.result.success
        assert 'not confirmed' in result.result.message
        assert state['overrides'] == 0
        assert state['modes'] == [TrajectoryMode.Request.ROBOT_POSE]


@pytest.mark.parametrize('failure', ['control', 'safety', 'stale'])
def test_loss_during_ascent_aborts(failure):
    with server() as (state, goal):
        handle = goal()
        wait_for(lambda: len(state['commands']) == 2)
        state['confirmed'] = True
        wait_for(lambda: state['overrides'] == 1)
        if failure == 'control':
            state['confirmed'] = False
        elif failure == 'safety':
            state['safety_timeout'] = True
        else:
            state['publish_state'] = False
        result = future_result(handle.get_result_async())
        assert result.status == 6 and not result.result.success
        expected = 'state estimate timed out' if failure == 'safety' else 'control lost'
        assert expected in result.result.message
        assert state['modes'][-1] == TrajectoryMode.Request.ROBOT_POSE


@pytest.mark.parametrize('failure', ['safety', 'cancel'])
def test_wait_can_be_interrupted(failure):
    with server(2.0) as (state, goal):
        handle = goal()
        wait_for(lambda: len(state['commands']) == 2)
        if failure == 'safety':
            state['safety_timeout'] = True
        else:
            assert future_result(handle.cancel_goal_async()).goals_canceling
        result = future_result(handle.get_result_async())
        assert result.status == (6 if failure == 'safety' else 5)
        assert not result.result.success
        assert state['overrides'] == 0
        assert state['modes'] == [TrajectoryMode.Request.ROBOT_POSE]


def test_default_preserves_acknowledgement_only_behavior():
    with server(None) as (state, goal):
        handle = goal()
        wait_for(lambda: state['overrides'] == 1)
        assert not state['confirmed']
        state['z'] = 12.0
        result = future_result(handle.get_result_async())
        assert result.status == 4 and result.result.success
