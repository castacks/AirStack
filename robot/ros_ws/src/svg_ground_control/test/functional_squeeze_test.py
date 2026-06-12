"""Closed-loop ROS functional test: squeeze scenario end to end.

Not run by colcon — run manually against a live swarm_commander:

    # terminal A (any ROS 2 env with the workspace + airstack_msgs built):
    ros2 launch svg_ground_control ground_control.launch.py \
        config:=<share>/config/squeeze_3drone.yaml

    # terminal B:
    python3 functional_squeeze_test.py

Fakes 3 drones (odometry publishers that integrate the commander's velocity
commands + robot_command service servers) and drives the full lifecycle:
takeoff -> ascend to squeeze posts -> start -> intruder crosses the gap
(holders must yield and return, no barrier breach) -> land -> disarm.
"""

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
# World spawn points = the Isaac script's layout (x = 2*(i-1) - 2). Each
# fake drone publishes odometry in its OWN local frame (world - offset),
# emulating PX4 SITL whose EKF origin is the spawn point — so this test
# also validates the commander's drone_position_offsets correction.
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


class FakeDrone:
    def __init__(self, node, name):
        self.name = name
        self.position = np.array(GROUND[name])
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
        local = self.position - OFFSETS[self.name]   # publish in local frame
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
    node = Node('fake_drones')
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

    posts = HOLDER_POSTS
    intruder_start = INTRUDER_START

    print('TEST 1: takeoff -> all drones reach the squeeze layout')
    res = call_trigger('/swarm_commander/takeoff')
    assert res.success, res.message
    wait_until(
        lambda: (np.linalg.norm(drones['drone_1'].position - posts[0]) < 0.25
                 and np.linalg.norm(drones['drone_2'].position - posts[1]) < 0.25
                 and np.linalg.norm(drones['drone_3'].position - intruder_start) < 0.25),
        25.0, 'drones at squeeze initial positions')
    gap = float(np.linalg.norm(posts[0] - posts[1]))
    print(f'  -> holders at posts (gap = {gap:.2f} m), intruder staged')

    print('TEST 2: start -> intruder crosses, holders yield, barrier holds')
    # The commander flips ASCEND->ACTIVE at a tighter threshold than the
    # position check above, so retry start until everyone is holding.
    deadline = time.monotonic() + 15.0
    while True:
        res = call_trigger('/swarm_commander/start')
        if res.success:
            break
        assert time.monotonic() < deadline, f'start never accepted: {res.message}'
        time.sleep(1.0)

    min_pair = np.inf
    max_holder_disp = 0.0
    max_progress = -np.inf   # intruder flies +x -> -x, so progress = -x
    t_end = time.monotonic() + 25.0
    while time.monotonic() < t_end:
        p = np.stack([drones[n].position for n in NAMES])
        d = np.linalg.norm(p[:, None] - p[None, :], axis=-1)
        np.fill_diagonal(d, np.inf)
        min_pair = min(min_pair, float(d.min()))
        max_holder_disp = max(
            max_holder_disp,
            float(np.linalg.norm(p[:2] - posts, axis=-1).max()))
        max_progress = max(max_progress, float(-p[2, 0]))
        time.sleep(0.05)

    assert max_progress > 1.0, \
        f'intruder never crossed (reached x = {-max_progress:.2f})'
    assert max_holder_disp > 0.2, \
        f'holders never yielded (max displacement = {max_holder_disp:.2f} m)'
    assert min_pair >= 1.5 * SAFETY_RADIUS, \
        f'pair separation dropped to {min_pair:.2f} m'
    print(f'  -> intruder crossed to x={-max_progress:.2f}, holders yielded '
          f'{max_holder_disp:.2f} m, min pair distance {min_pair:.2f} m')

    print('TEST 3: hold freezes everyone')
    res = call_trigger('/swarm_commander/hold')
    assert res.success, res.message
    time.sleep(1.0)
    frozen = np.stack([drones[n].position for n in NAMES])
    time.sleep(2.0)
    drift = np.linalg.norm(
        np.stack([drones[n].position for n in NAMES]) - frozen, axis=-1).max()
    assert drift < 0.15, f'drones drifted {drift:.2f} m while holding'
    print(f'  -> max drift while holding: {drift:.3f} m')

    print('TEST 4: land -> descend -> disarm')
    res = call_trigger('/swarm_commander/land')
    assert res.success, res.message
    wait_until(lambda: all(RobotCommand.Request.DISARM in d.commands_received
                           for d in drones.values()), 20.0, 'all disarmed')
    print('  -> all drones landed and disarmed')

    print('ALL TESTS PASSED')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
