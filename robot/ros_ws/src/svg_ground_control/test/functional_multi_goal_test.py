"""Closed-loop test: multi-drone goal tracking, live per-drone goals + speed.

Run against a live commander launched with goal_tracking.yaml:

    # terminal A:
    ros2 launch svg_ground_control ground_control.launch.py \
        config:=<share>/config/goal_tracking.yaml
    # terminal B:
    python3 test/functional_multi_goal_test.py

Three fake drones publishing odometry in their PX4-SITL local frames (world
- spawn offset), so this also exercises drone_position_offsets. After
takeoff + start it assigns each drone a goal that forces them to cross,
verifies all three arrive, and that the CBF kept them apart throughout.
"""

import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from airstack_msgs.srv import RobotCommand

NAMES = ['drone_1', 'drone_2', 'drone_3']
OFFSETS = {'drone_1': np.array([-2.0, 0.0, 0.0]),
           'drone_2': np.array([0.0, 0.0, 0.0]),
           'drone_3': np.array([2.0, 0.0, 0.0])}
GROUND = {'drone_1': np.array([-2.0, 0.0, 0.05]),
          'drone_2': np.array([0.0, 0.0, 0.05]),
          'drone_3': np.array([2.0, 0.0, 0.05])}
TAKEOFF = {'drone_1': np.array([-1.5, 0.0, 1.2]),   # = hover_positions
           'drone_2': np.array([1.5, 0.0, 1.2]),
           'drone_3': np.array([0.0, 1.5, 1.2])}
SAFETY_RADIUS = 0.55
DT = 0.02


class FakeDrone:
    def __init__(self, node, name):
        self.name = name
        self.position = GROUND[name].copy()
        self.cmd = np.zeros(3)
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
        self.position = self.position + self.cmd * DT
        self.position[2] = max(self.position[2], 0.0)
        local = self.position - OFFSETS[self.name]
        msg = Odometry()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        (msg.pose.pose.position.x, msg.pose.pose.position.y,
         msg.pose.pose.position.z) = local
        msg.pose.pose.orientation.w = 1.0
        (msg.twist.twist.linear.x, msg.twist.twist.linear.y,
         msg.twist.twist.linear.z) = self.cmd
        self.odom_pub.publish(msg)


def main():
    rclpy.init()
    node = Node('fake_multi_drone')
    drones = {n: FakeDrone(node, n) for n in NAMES}
    goal_pubs = {n: node.create_publisher(
        PoseStamped, f'/svg/{n}/goal_command', 10) for n in NAMES}
    speed_pubs = {n: node.create_publisher(
        Float32, f'/svg/{n}/speed_command', 10) for n in NAMES}
    node.create_timer(DT, lambda: [d.step(node) for d in drones.values()])

    ex = rclpy.executors.MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    def call(srv):
        c = node.create_client(Trigger, srv)
        assert c.wait_for_service(timeout_sec=5.0), f'{srv} unavailable'
        fut = c.call_async(Trigger.Request())
        end = time.monotonic() + 5.0
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.05)
        assert fut.done(), f'{srv} timed out'
        return fut.result()

    def send_goal(name, xyz):
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
        goal_pubs[name].publish(m)

    time.sleep(3.0)

    print('TEST 1: takeoff -> all reach their spread-out hover points')
    assert call('/swarm_commander/takeoff').success
    end = time.monotonic() + 25.0
    while not all(np.linalg.norm(drones[n].position - TAKEOFF[n]) < 0.25
                  for n in NAMES):
        assert time.monotonic() < end, 'not all reached takeoff points'
        time.sleep(0.1)
    print('  -> all at hover points')

    print('TEST 2: start + crossing goals, different speeds, CBF keeps apart')
    deadline = time.monotonic() + 15.0
    while not call('/swarm_commander/start').success:
        assert time.monotonic() < deadline, 'start never accepted'
        time.sleep(1.0)
    speed_pubs['drone_1'].publish(Float32(data=0.8))
    speed_pubs['drone_2'].publish(Float32(data=0.8))
    speed_pubs['drone_3'].publish(Float32(data=0.6))
    # swap drone_1 <-> drone_2 (forces a centre crossing); drone_3 to centre-ish
    goals = {'drone_1': [1.5, 0.0, 1.2],
             'drone_2': [-1.5, 0.0, 1.2],
             'drone_3': [0.0, -1.2, 1.4]}
    for n, g in goals.items():
        send_goal(n, g)

    min_pair = np.inf
    end = time.monotonic() + 30.0
    reached = lambda: all(  # noqa: E731
        np.linalg.norm(drones[n].position - np.array(goals[n])) < 0.25
        for n in NAMES)
    while time.monotonic() < end:
        p = np.stack([drones[n].position for n in NAMES])
        dd = np.linalg.norm(p[:, None] - p[None, :], axis=-1)
        np.fill_diagonal(dd, np.inf)
        min_pair = min(min_pair, float(dd.min()))
        if reached():
            break
        time.sleep(0.05)
    assert reached(), 'not all drones reached their swapped goals: ' + \
        ', '.join(f'{n}@{np.round(drones[n].position,2)}' for n in NAMES)
    print(f'  -> all reached crossed goals; min pair distance {min_pair:.2f} m '
          f'(2r = {2*SAFETY_RADIUS:.2f})')
    assert min_pair >= 2 * SAFETY_RADIUS - 0.15, \
        f'CBF let drones get too close: {min_pair:.2f} m'

    print('TEST 3: land')
    assert call('/swarm_commander/land').success
    end = time.monotonic() + 20.0
    while not all(RobotCommand.Request.DISARM in drones[n].commands_received
                  for n in NAMES):
        assert time.monotonic() < end, 'not all disarmed'
        time.sleep(0.1)
    print('  -> all landed + disarmed')
    print('ALL MULTI-GOAL TESTS PASSED')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
