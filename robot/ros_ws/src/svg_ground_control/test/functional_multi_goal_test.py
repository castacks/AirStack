"""Closed-loop test: multi-drone goal tracking, live per-drone goals + speed.

Run against a live commander launched with goal_tracking.yaml:

    # terminal A:
    ros2 launch svg_ground_control ground_control.launch.py \
        config:=<share>/config/goal_tracking.yaml
    # terminal B:
    python3 test/functional_multi_goal_test.py

The drone names, modes, position offsets, takeoff points and CBF radius are
read from the RUNNING commander's parameters. Each fake drone starts on the
ground under its takeoff point, publishes odometry in its local frame
(world - drone_position_offsets) and listens on the topics its drone_modes
entry routes to ("real" -> /fmu/trajectory_command + /fmu/robot_command,
tracked like PX4 offboard; "sim" -> /interface/velocity_command +
/interface/robot_command). After takeoff + start, the first and last drones
swap takeoff points (a head-on crossing) at different speeds; the test
verifies all arrive and that the CBF kept every pair apart throughout.
"""

import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from trajectory_msgs.msg import MultiDOFJointTrajectory
from airstack_msgs.srv import RobotCommand

COMMANDER = '/swarm_commander'
DT = 0.02
POS_GAIN = 0.95     # PX4-like position correction on the trajectory reference
PARAMS = ['drone_names', 'drone_modes', 'drone_position_offsets', 'hover_positions',
          'state_topic_template', 'cbf_safety_radius_m']


def wait_future(fut, timeout, what):
    end = time.monotonic() + timeout
    while not fut.done() and time.monotonic() < end:
        time.sleep(0.05)
    assert fut.done(), f'{what} timed out'
    return fut.result()


def commander_params(node):
    c = node.create_client(GetParameters, f'{COMMANDER}/get_parameters')
    assert c.wait_for_service(timeout_sec=10.0), f'{COMMANDER} is not running'
    res = wait_future(c.call_async(GetParameters.Request(names=PARAMS)), 5.0,
                      'get_parameters')
    return {n: parameter_value_to_python(v) for n, v in zip(PARAMS, res.values)}


class FakeDrone:
    """Point-mass drone on the topics the commander uses for its mode."""

    def __init__(self, node, name, mode, offset, ground, state_topic):
        self.name, self.mode, self.offset = name, mode, offset
        self.position = ground.copy()          # world frame
        self.cmd = np.zeros(3)
        self.ref = None
        self.vel_ff = np.zeros(3)
        self.commands_received = []
        self.odom_pub = node.create_publisher(Odometry, state_topic, 10)
        if mode == 'real':
            node.create_subscription(MultiDOFJointTrajectory,
                                     f'/{name}/fmu/trajectory_command', self.traj_cb, 10)
            srv = f'/{name}/fmu/robot_command'
        else:
            node.create_subscription(TwistStamped,
                                     f'/{name}/interface/velocity_command', self.cmd_cb, 10)
            srv = f'/{name}/interface/robot_command'
        node.create_service(RobotCommand, srv, self.srv_cb)

    def cmd_cb(self, msg):
        self.cmd = np.array([msg.twist.linear.x, msg.twist.linear.y,
                             msg.twist.linear.z])

    def traj_cb(self, msg):
        pt = msg.points[0]
        t, v = pt.transforms[0].translation, pt.velocities[0].linear
        self.ref = np.array([t.x, t.y, t.z]) + self.offset   # local -> world
        self.vel_ff = np.array([v.x, v.y, v.z])

    def srv_cb(self, req, res):
        self.commands_received.append(req.command)
        res.success = True
        return res

    def step(self, node):
        if self.mode == 'real' and self.ref is not None:
            self.cmd = self.vel_ff + POS_GAIN * (self.ref - self.position)
        self.position = self.position + self.cmd * DT
        self.position[2] = max(self.position[2], 0.0)
        local = self.position - self.offset
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
    ex = rclpy.executors.MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    p = commander_params(node)
    names = list(p['drone_names'])
    assert len(names) >= 2, 'needs at least two drones'
    modes = [m.strip() for m in str(p['drone_modes']).split(',')]
    modes = [modes[i] if i < len(modes) and modes[i] else 'sim' for i in range(len(names))]
    offsets = np.array(p['drone_position_offsets'], dtype=float).reshape(-1, 3)
    takeoff = {n: np.array(p['hover_positions'][3 * i:3 * i + 3], dtype=float)
               for i, n in enumerate(names)}
    radius = float(p['cbf_safety_radius_m'])
    drones = {n: FakeDrone(node, n, modes[i], offsets[i],
                           np.array([takeoff[n][0], takeoff[n][1], 0.05]),
                           p['state_topic_template'].format(name=n))
              for i, n in enumerate(names)}
    goal_pubs = {n: node.create_publisher(
        PoseStamped, f'/svg/{n}/goal_command', 10) for n in names}
    speed_pubs = {n: node.create_publisher(
        Float32, f'/svg/{n}/speed_command', 10) for n in names}
    node.create_timer(DT, lambda: [d.step(node) for d in drones.values()])

    def call(srv):
        c = node.create_client(Trigger, srv)
        assert c.wait_for_service(timeout_sec=5.0), f'{srv} unavailable'
        return wait_future(c.call_async(Trigger.Request()), 5.0, srv)

    def send_goal(name, xyz):
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
        goal_pubs[name].publish(m)

    time.sleep(3.0)

    print('TEST 1: takeoff -> all reach their spread-out hover points')
    assert call('/swarm_commander/takeoff').success
    end = time.monotonic() + 25.0
    while not all(np.linalg.norm(drones[n].position - takeoff[n]) < 0.25
                  for n in names):
        assert time.monotonic() < end, 'not all reached takeoff points'
        time.sleep(0.1)
    print('  -> all at hover points')

    print(f'TEST 2: start + {names[0]} <-> {names[-1]} swap, different speeds, CBF keeps apart')
    deadline = time.monotonic() + 15.0
    while not call('/swarm_commander/start').success:
        assert time.monotonic() < deadline, 'start never accepted'
        time.sleep(1.0)
    # first <-> last swap takeoff points (head-on crossing) at different
    # speeds; any drones in between hold their own point.
    goals = {n: takeoff[n] for n in names}
    goals[names[0]], goals[names[-1]] = takeoff[names[-1]], takeoff[names[0]]
    for i, n in enumerate(names):
        speed_pubs[n].publish(Float32(data=0.8 if i % 2 == 0 else 0.6))
    time.sleep(0.3)
    for n, g in goals.items():
        send_goal(n, list(g))

    min_pair = np.inf
    end = time.monotonic() + 30.0
    reached = lambda: all(  # noqa: E731
        np.linalg.norm(drones[n].position - np.array(goals[n])) < 0.25
        for n in names)
    while time.monotonic() < end:
        p = np.stack([drones[n].position for n in names])
        dd = np.linalg.norm(p[:, None] - p[None, :], axis=-1)
        np.fill_diagonal(dd, np.inf)
        min_pair = min(min_pair, float(dd.min()))
        if reached():
            break
        time.sleep(0.05)
    assert reached(), 'not all drones reached their swapped goals: ' + \
        ', '.join(f'{n}@{np.round(drones[n].position,2)}' for n in names)
    print(f'  -> all reached crossed goals; min pair distance {min_pair:.2f} m '
          f'(2r = {2*radius:.2f})')
    assert min_pair >= 2 * radius - 0.15, \
        f'CBF let drones get too close: {min_pair:.2f} m'

    print('TEST 3: land')
    assert call('/swarm_commander/land').success
    end = time.monotonic() + 20.0
    while not all(RobotCommand.Request.DISARM in drones[n].commands_received
                  for n in names):
        assert time.monotonic() < end, 'not all disarmed'
        time.sleep(0.1)
    print('  -> all landed + disarmed')
    print('ALL MULTI-GOAL TESTS PASSED')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
