"""Closed-loop squeeze test with PX4-like velocity-tracking lag.

Same harness as functional_squeeze_test.py, but the fake drones track the
commanded velocity through a first-order lag (tau ~ PX4 velocity loop)
instead of integrating it perfectly. This reproduces the real-sim failure
mode where ideal-integrator tests pass: with lag, the conflict cluster
tightens beyond what the commander commanded, and the demo only works if
the intruder is genuinely CBF-exempt.

Run against a live commander with the squeeze profile (see experiment.md).

Pass criteria:
  - the intruder fully crosses the gap (no turn-back before crossing),
  - the holders yield and the holder-holder barrier is respected,
  - everything lands cleanly.
"""

import os
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from airstack_msgs.srv import RobotCommand

NAMES = ['drone_1', 'drone_2', 'drone_3']
# World spawn points = Isaac layout; odometry published in per-drone local
# frames (world - offset) to emulate PX4 SITL origins, exercising the
# commander's drone_position_offsets correction.
GROUND = {'drone_1': [-2.0, 0.0, 0.05],
          'drone_2': [0.0, 0.0, 0.05],
          'drone_3': [2.0, 0.0, 0.05]}
OFFSETS = {'drone_1': np.array([-2.0, 0.0, 0.0]),
           'drone_2': np.array([0.0, 0.0, 0.0]),
           'drone_3': np.array([2.0, 0.0, 0.0])}
SAFETY_RADIUS = 0.55
# Must match squeeze_3drone.yaml (world frame; intruder starts on +x side)
HOLDER_POSTS = np.array([[0.0, -0.69, 1.2], [0.0, 0.69, 1.2]])
INTRUDER_START = np.array([1.5, 0.0, 1.2])
DT = 0.02
# First-order lag of the velocity response; override with VELOCITY_TAU_S env.
VELOCITY_TAU_S = float(os.environ.get('VELOCITY_TAU_S', '0.8'))


class FakeDrone:
    def __init__(self, node, name):
        self.name = name
        self.position = np.array(GROUND[name])
        self.cmd = np.zeros(3)        # commanded velocity
        self.vel = np.zeros(3)        # achieved velocity (lags the command)
        self.commands_received = []
        self.odom_pub = node.create_publisher(
            Odometry, f'/{name}/odometry_conversion/odometry', 10)
        node.create_subscription(
            TwistStamped, f'/{name}/interface/velocity_command', self.cmd_cb, 10)
        node.create_service(
            RobotCommand, f'/{name}/interface/robot_command', self.srv_cb)

    def cmd_cb(self, msg):
        self.cmd = np.array([msg.twist.linear.x, msg.twist.linear.y,
                             msg.twist.linear.z])

    def srv_cb(self, req, res):
        self.commands_received.append(req.command)
        res.success = True
        return res

    def step(self, node):
        self.vel += (self.cmd - self.vel) * (DT / VELOCITY_TAU_S)
        self.position = self.position + self.vel * DT
        self.position[2] = max(self.position[2], 0.0)
        local = self.position - OFFSETS[self.name]   # publish in local frame
        msg = Odometry()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        (msg.pose.pose.position.x, msg.pose.pose.position.y,
         msg.pose.pose.position.z) = local
        msg.pose.pose.orientation.w = 1.0
        (msg.twist.twist.linear.x, msg.twist.twist.linear.y,
         msg.twist.twist.linear.z) = self.vel
        self.odom_pub.publish(msg)


def main():
    rclpy.init()
    node = Node('fake_drones_lag')
    drones = {n: FakeDrone(node, n) for n in NAMES}
    node.create_timer(DT, lambda: [d.step(node) for d in drones.values()])

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    def call_trigger(srv_name):
        client = node.create_client(Trigger, srv_name)
        assert client.wait_for_service(timeout_sec=5.0), f'{srv_name} unavailable'
        fut = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 5.0
        while not fut.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        assert fut.done(), f'{srv_name} call timed out'
        return fut.result()

    def wait_until(pred, timeout, what):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if pred():
                return
            time.sleep(0.1)
        raise AssertionError(f'TIMEOUT waiting for: {what}')

    time.sleep(3.0)  # discovery

    print(f'LAG TEST (velocity tau = {VELOCITY_TAU_S}s)')
    print('TEST 1: takeoff -> squeeze layout')
    res = call_trigger('/swarm_commander/takeoff')
    assert res.success, res.message
    wait_until(
        lambda: all(np.linalg.norm(drones[n].position - t) < 0.25
                    for n, t in zip(NAMES, [*HOLDER_POSTS, INTRUDER_START])),
        30.0, 'drones at squeeze initial positions')
    print('  -> holders at posts, intruder staged')

    print('TEST 2: start -> intruder must cross WITHOUT turning back')
    deadline = time.monotonic() + 15.0
    while True:
        res = call_trigger('/swarm_commander/start')
        if res.success:
            break
        assert time.monotonic() < deadline, f'start never accepted: {res.message}'
        time.sleep(1.0)

    min_holder_pair = np.inf
    max_holder_disp = 0.0
    peak_s = -np.inf   # intruder flies +x -> -x: progress s = -x
    crossed = False
    bounce = 0.0   # how far the intruder retreated from its peak pre-crossing
    t_end = time.monotonic() + 30.0
    while time.monotonic() < t_end:
        p = np.stack([drones[n].position for n in NAMES])
        min_holder_pair = min(min_holder_pair,
                              float(np.linalg.norm(p[0] - p[1])))
        max_holder_disp = max(
            max_holder_disp,
            float(np.linalg.norm(p[:2] - HOLDER_POSTS, axis=-1).max()))
        s = float(-p[2, 0])
        peak_s = max(peak_s, s)
        if s > 1.0:
            crossed = True
            break
        bounce = max(bounce, peak_s - s)
        time.sleep(0.05)

    print(f'  -> crossed={crossed}, reached x={-peak_s:.2f}, '
          f'pre-cross retreat={bounce:.2f} m, holders yielded '
          f'{max_holder_disp:.2f} m, holder pair min {min_holder_pair:.2f} m')
    assert bounce < 0.3, \
        f'INTRUDER TURNED BACK before crossing (retreated {bounce:.2f} m)'
    assert crossed, f'intruder never crossed (reached x = {-peak_s:.2f})'
    assert max_holder_disp > 0.2, 'holders never yielded'
    assert min_holder_pair >= 2.0 * SAFETY_RADIUS - 0.1, \
        f'holder pair breached barrier: {min_holder_pair:.2f} m'

    print('TEST 3: land')
    res = call_trigger('/swarm_commander/land')
    assert res.success, res.message
    wait_until(lambda: all(RobotCommand.Request.DISARM in d.commands_received
                           for d in drones.values()), 25.0, 'all disarmed')
    print('ALL LAG TESTS PASSED')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
