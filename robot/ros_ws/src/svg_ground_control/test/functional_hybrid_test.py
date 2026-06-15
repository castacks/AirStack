"""Closed-loop test: hybrid real/sim routing in the squeeze scenario.

Run against a live commander launched with hybrid_squeeze.yaml
(drone_modes: "real,real,sim"):

    # terminal A:
    ros2 launch svg_ground_control ground_control.launch.py \
        config:=<share>/config/hybrid_squeeze.yaml
    # terminal B:
    python3 test/functional_hybrid_test.py

Verifies the per-drone command ROUTING: the two holders (mode 'real') must
be commanded on /{name}/fmu/velocity_command and serviced on
/{name}/fmu/robot_command, while the intruder (mode 'sim') is commanded on
/{name}/interface/velocity_command. Each fake drone ONLY subscribes to its
expected topic — if the commander routed to the wrong one, the drone gets no
command, never moves, and the test fails. Also checks the squeeze still
works (intruder crosses, holders yield, barrier respected).
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

# name -> (mode, command-topic-namespace). Holders real, intruder sim.
MODES = {'drone_1': 'fmu', 'drone_2': 'fmu', 'drone_3': 'interface'}
OFFSETS = {'drone_1': np.array([0.0, 0.0, 0.0]),     # real holders, mocap-anchored
           'drone_2': np.array([0.0, 0.0, 0.0]),
           'drone_3': np.array([2.0, 0.0, 0.0])}     # sim intruder, SITL spawn x=+2
GROUND = {'drone_1': np.array([0.0, -0.7, 0.05]),
          'drone_2': np.array([0.0, 0.7, 0.05]),
          'drone_3': np.array([2.0, 0.0, 0.05])}
HOLDER_POSTS = np.array([[0.0, -0.69, 1.2], [0.0, 0.69, 1.2]])
INTRUDER_START = np.array([1.5, 0.0, 1.2])
SAFETY_RADIUS = 0.55
DT = 0.02


class FakeDrone:
    def __init__(self, node, name):
        self.name = name
        self.ns = MODES[name]
        self.position = GROUND[name].copy()
        self.cmd = np.zeros(3)
        self.cmd_count = 0
        self.commands_received = []
        self.odom_pub = node.create_publisher(
            Odometry, f'/{name}/odometry_conversion/odometry', 10)
        # Subscribe ONLY to the topic the drone's mode should produce.
        node.create_subscription(
            TwistStamped, f'/{name}/{self.ns}/velocity_command', self.cmd_cb, 10)
        node.create_service(
            RobotCommand, f'/{name}/{self.ns}/robot_command', self.srv_cb)

    def cmd_cb(self, msg):
        self.cmd = np.array([msg.twist.linear.x, msg.twist.linear.y,
                             msg.twist.linear.z])
        self.cmd_count += 1

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
    node = Node('fake_hybrid_drones')
    drones = {n: FakeDrone(node, n) for n in MODES}
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

    time.sleep(3.0)

    print('HYBRID ROUTING TEST (drone_1/2=real/fmu, drone_3=sim/interface)')
    print('TEST 1: takeoff -> reach squeeze layout via correctly-routed cmds')
    assert call('/swarm_commander/takeoff').success
    end = time.monotonic() + 30.0
    targets = {'drone_1': HOLDER_POSTS[0], 'drone_2': HOLDER_POSTS[1],
               'drone_3': INTRUDER_START}
    while not all(np.linalg.norm(drones[n].position - targets[n]) < 0.3
                  for n in MODES):
        assert time.monotonic() < end, 'not all reached layout: ' + ', '.join(
            f'{n}@{np.round(drones[n].position,2)}' for n in MODES)
        time.sleep(0.1)
    # Routing proof: every drone moved, so each received commands on its
    # mode-specific topic (it subscribes to no other).
    for n in MODES:
        assert drones[n].cmd_count > 0, f'{n} got no commands on /{n}/{MODES[n]}/'
        assert drones[n].commands_received, f'{n} got no robot_command on /{n}/{MODES[n]}/'
    print('  -> holders commanded on /fmu/, intruder on /interface/ (all moved)')

    print('TEST 2: start -> intruder crosses, holders yield, barrier holds')
    deadline = time.monotonic() + 15.0
    while not call('/swarm_commander/start').success:
        assert time.monotonic() < deadline, 'start never accepted'
        time.sleep(1.0)
    min_holder_pair = np.inf
    max_disp = 0.0
    peak_progress = -np.inf      # intruder flies +x -> -x; progress = -x
    end = time.monotonic() + 30.0
    while time.monotonic() < end:
        p = np.stack([drones[n].position for n in MODES])
        min_holder_pair = min(min_holder_pair, float(np.linalg.norm(p[0] - p[1])))
        max_disp = max(max_disp,
                       float(np.linalg.norm(p[:2] - HOLDER_POSTS, axis=-1).max()))
        peak_progress = max(peak_progress, float(-p[2, 0]))
        if peak_progress > 1.0:
            break
        time.sleep(0.05)
    print(f'  -> intruder reached x={-peak_progress:.2f}, holders yielded '
          f'{max_disp:.2f} m, holder pair min {min_holder_pair:.2f} m')
    assert peak_progress > 1.0, 'intruder never crossed'
    assert max_disp > 0.2, 'holders never yielded'
    assert min_holder_pair >= 2 * SAFETY_RADIUS - 0.1, 'holder barrier breached'

    print('TEST 3: land (disarms via correctly-routed services)')
    assert call('/swarm_commander/land').success
    end = time.monotonic() + 20.0
    while not all(RobotCommand.Request.DISARM in drones[n].commands_received
                  for n in MODES):
        assert time.monotonic() < end, 'not all disarmed'
        time.sleep(0.1)
    print('  -> all disarmed on their mode-specific services')
    print('ALL HYBRID TESTS PASSED')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
