"""Closed-loop test: single-drone goal tracking with live speed changes.

Run against a live commander launched with goal_single.yaml:

    # terminal A:
    ros2 launch svg_ground_control ground_control.launch.py \
        config:=<share>/config/goal_single.yaml
    # terminal B (optional arg: the same config file, default ../config/goal_single.yaml):
    python3 test/functional_single_goal_test.py [<config.yaml>]

Fakes one drone (odometry + a robot_command service) on the topics its
`drone_modes` entry routes to: "real" -> the commander's trajectory output on
/{name}/fmu/trajectory_command (reference point + velocity, tracked like PX4
offboard) and /{name}/fmu/robot_command; "sim" -> /{name}/interface/velocity_command
and /{name}/interface/robot_command. Takeoff point and offset come from the
config. Then: takeoff -> start -> send a goal -> verify arrival -> slow down
via speed_command and verify the speed cap -> speed up again and verify it
gets faster -> land.
"""

import os
import sys
import threading
import time

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from trajectory_msgs.msg import MultiDOFJointTrajectory
from airstack_msgs.srv import RobotCommand

CONFIG = (sys.argv[1] if len(sys.argv) > 1 else
          os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       '..', 'config', 'goal_single.yaml'))
with open(CONFIG) as _f:
    P = yaml.safe_load(_f)['swarm_commander']['ros__parameters']

NAME = P['drone_names'][0]
MODE = str(P.get('drone_modes', 'sim')).split(',')[0].strip() or 'sim'
OFFSET = np.array(P['drone_position_offsets'][:3], dtype=float)
TAKEOFF = np.array(P['hover_positions'][:3], dtype=float)   # world frame
GROUND = np.array([OFFSET[0], OFFSET[1], 0.05])
DT = 0.02
POS_GAIN = 0.95     # PX4-like position correction on the trajectory reference


class FakeDrone:
    """Point-mass drone on the topics the commander uses for MODE."""

    def __init__(self, node):
        self.position = GROUND.copy()          # world frame
        self.cmd = np.zeros(3)
        self.commands_received = []
        self.odom_pub = node.create_publisher(
            Odometry, P['state_topic_template'].format(name=NAME), 10)
        if MODE == 'real':
            self.ref = None
            self.vel_ff = np.zeros(3)
            node.create_subscription(
                MultiDOFJointTrajectory, f'/{NAME}/fmu/trajectory_command',
                self.traj_cb, 10)
            srv = f'/{NAME}/fmu/robot_command'
        else:
            node.create_subscription(
                TwistStamped, f'/{NAME}/interface/velocity_command', self.cmd_cb, 10)
            srv = f'/{NAME}/interface/robot_command'
        node.create_service(RobotCommand, srv, self.srv_cb)

    def cmd_cb(self, msg):
        self.cmd = np.array([msg.twist.linear.x, msg.twist.linear.y,
                             msg.twist.linear.z])

    def traj_cb(self, msg):
        pt = msg.points[0]
        t = pt.transforms[0].translation
        v = pt.velocities[0].linear
        self.ref = np.array([t.x, t.y, t.z]) + OFFSET   # local -> world
        self.vel_ff = np.array([v.x, v.y, v.z])

    def srv_cb(self, req, res):
        self.commands_received.append(req.command)
        res.success = True
        return res

    def step(self, node):
        if MODE == 'real' and self.ref is not None:
            self.cmd = self.vel_ff + POS_GAIN * (self.ref - self.position)
        self.position = self.position + self.cmd * DT
        self.position[2] = max(self.position[2], 0.0)
        local = self.position - OFFSET
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

    def fly_timed(goal, speed):
        speed_pub.publish(Float32(data=speed))
        time.sleep(0.5)
        dist = np.linalg.norm(np.array(goal) - drone.position)
        t_send = time.monotonic()
        send_goal(goal)
        elapsed = wait_reach(goal, 0.2, 30.0) - t_send
        print(f'  -> {dist:.1f} m at speed_command {speed} m/s in {elapsed:.1f}s '
              f'(~{dist/max(elapsed,1e-3):.2f} m/s effective)')
        return dist, elapsed

    print('TEST 3: speed_command caps the cruise speed, and raising it speeds up')
    slow = 0.5
    dist, elapsed = fly_timed([-1.5, -1.0, 1.0], slow)
    assert dist / elapsed < 1.2 * slow, 'faster than the speed_command allows'
    dist_f, elapsed_f = fly_timed([1.0, 0.5, 1.4], 2.0)
    assert dist_f / elapsed_f > 1.5 * dist / elapsed, 'raising the speed did not speed it up'

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
