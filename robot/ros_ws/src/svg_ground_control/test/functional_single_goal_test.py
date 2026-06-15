"""Closed-loop test: single-drone goal tracking with configurable speed.

Run against a live commander launched with goal_single.yaml:

    # terminal A:
    ros2 launch svg_ground_control ground_control.launch.py \
        config:=<share>/config/goal_single.yaml
    # terminal B:
    python3 test/functional_single_goal_test.py

Fakes one drone (odometry publisher integrating the commander's velocity
commands + a robot_command service), then: takeoff -> start -> send a goal
-> verify arrival -> raise the speed, send a farther goal -> verify it
arrives faster -> land.
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

NAME = 'drone_1'
GROUND = np.array([0.0, 0.0, 0.05])    # offset 0 in goal_single.yaml
TAKEOFF = np.array([0.0, 0.0, 1.2])    # hover_positions
DT = 0.02


class FakeDrone:
    def __init__(self, node):
        self.position = GROUND.copy()
        self.cmd = np.zeros(3)
        self.commands_received = []
        self.odom_pub = node.create_publisher(
            Odometry, f'/{NAME}/odometry_conversion/odometry', 10)
        node.create_subscription(
            TwistStamped, f'/{NAME}/interface/velocity_command', self.cmd_cb, 10)
        node.create_service(
            RobotCommand, f'/{NAME}/interface/robot_command', self.srv_cb)

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
        msg = Odometry()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        (msg.pose.pose.position.x, msg.pose.pose.position.y,
         msg.pose.pose.position.z) = self.position
        msg.pose.pose.orientation.w = 1.0
        (msg.twist.twist.linear.x, msg.twist.twist.linear.y,
         msg.twist.twist.linear.z) = self.cmd
        self.odom_pub.publish(msg)


def main():
    rclpy.init()
    node = Node('fake_single_drone')
    drone = FakeDrone(node)
    goal_pub = node.create_publisher(
        PoseStamped, f'/svg/{NAME}/goal_command', 10)
    speed_pub = node.create_publisher(
        Float32, f'/svg/{NAME}/speed_command', 10)
    node.create_timer(DT, lambda: drone.step(node))

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

    def send_goal(xyz):
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
        goal_pub.publish(m)

    def wait_reach(xyz, tol, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if np.linalg.norm(drone.position - np.array(xyz)) < tol:
                return time.monotonic()
            time.sleep(0.05)
        raise AssertionError(
            f'did not reach {xyz} (at {np.round(drone.position,2)})')

    time.sleep(3.0)

    print('TEST 1: takeoff -> ascend to hover point')
    assert call('/swarm_commander/takeoff').success
    wait_reach(TAKEOFF, 0.25, 20.0)
    print(f'  -> at takeoff point {np.round(drone.position,2)}')

    print('TEST 2: start + goal -> drone reaches assigned goal')
    deadline = time.monotonic() + 15.0
    while not call('/swarm_commander/start').success:
        assert time.monotonic() < deadline, 'start never accepted'
        time.sleep(1.0)
    goal_a = [1.0, 0.5, 1.4]
    send_goal(goal_a)
    wait_reach(goal_a, 0.2, 20.0)
    print(f'  -> reached goal A {goal_a}')

    print('TEST 3: higher speed -> farther goal reached faster')
    speed_pub.publish(Float32(data=1.0))
    time.sleep(0.5)
    goal_b = [-1.5, -1.0, 1.0]
    dist = np.linalg.norm(np.array(goal_b) - drone.position)
    t_send = time.monotonic()
    send_goal(goal_b)
    t_reach = wait_reach(goal_b, 0.2, 20.0)
    elapsed = t_reach - t_send
    # at ~1.0 m/s, expect roughly dist seconds (+ approach easing); generous bound
    print(f'  -> reached goal B in {elapsed:.1f}s for {dist:.1f}m '
          f'(~{dist/max(elapsed,1e-3):.2f} m/s effective)')
    assert elapsed < dist / 0.4, 'drone slower than the configured speed implies'

    print('TEST 4: land')
    assert call('/swarm_commander/land').success
    end = time.monotonic() + 20.0
    while RobotCommand.Request.DISARM not in drone.commands_received:
        assert time.monotonic() < end, 'never disarmed'
        time.sleep(0.1)
    print('  -> landed + disarmed')
    print('ALL SINGLE-GOAL TESTS PASSED')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
