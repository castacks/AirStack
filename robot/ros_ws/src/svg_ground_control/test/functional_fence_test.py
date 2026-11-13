"""Closed-loop test: the geofence stops a drone sent outside it.

Run against a live commander with the fence enabled (goal_single.yaml):

    # terminal A:
    ros2 launch svg_ground_control ground_control.launch.py \
        config:=<share>/config/goal_single.yaml
    # terminal B:
    python3 test/functional_fence_test.py

The test reads the fence and the drone setup from the RUNNING commander's
parameters (so launch overrides such as fence_behavior:=hold_all are
honoured) and fakes the first drone on the topics its drone_modes entry
routes to ("real" -> /fmu/trajectory_command + /fmu/robot_command, tracked
like PX4 offboard; "sim" -> /interface/velocity_command + /interface/robot_command).

Takeoff -> start -> command a goal 5 m past fence_max x, then per
fence_behavior:

* hold_all: the breach latches, the drone freezes near the wall, start is
  refused while latched, and ~/reset_fence clears it.
* keep_in:  the drone is clipped at the wall (never past it), nothing
  latches (start is still accepted), and a goal back inside is reached.
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
from std_srvs.srv import Trigger
from trajectory_msgs.msg import MultiDOFJointTrajectory
from airstack_msgs.srv import RobotCommand

COMMANDER = '/swarm_commander'
DT = 0.02
POS_GAIN = 0.95     # PX4-like position correction on the trajectory reference
PARAMS = ['drone_names', 'drone_modes', 'drone_position_offsets', 'hover_positions',
          'state_topic_template', 'fence_enabled', 'fence_behavior', 'fence_max']


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

    def __init__(self, node, name, mode, offset, state_topic):
        self.name, self.mode, self.offset = name, mode, offset
        self.position = np.array([offset[0], offset[1], 0.05])   # world frame
        self.cmd = np.zeros(3)
        self.ref = None
        self.vel_ff = np.zeros(3)
        self.odom_pub = node.create_publisher(Odometry, state_topic, 10)
        if mode == 'real':
            node.create_subscription(MultiDOFJointTrajectory,
                                     f'/{name}/fmu/trajectory_command', self.traj_cb, 10)
            srv = f'/{name}/fmu/robot_command'
        else:
            node.create_subscription(TwistStamped,
                                     f'/{name}/interface/velocity_command', self.cmd_cb, 10)
            srv = f'/{name}/interface/robot_command'
        node.create_service(RobotCommand, srv,
                            lambda req, res: (setattr(res, 'success', True) or res))

    def cmd_cb(self, msg):
        self.cmd = np.array([msg.twist.linear.x, msg.twist.linear.y,
                             msg.twist.linear.z])

    def traj_cb(self, msg):
        pt = msg.points[0]
        t, v = pt.transforms[0].translation, pt.velocities[0].linear
        self.ref = np.array([t.x, t.y, t.z]) + self.offset   # local -> world
        self.vel_ff = np.array([v.x, v.y, v.z])

    def step(self, node):
        if self.mode == 'real' and self.ref is not None:
            self.cmd = self.vel_ff + POS_GAIN * (self.ref - self.position)
        self.position = self.position + self.cmd * DT
        self.position[2] = max(self.position[2], 0.0)
        msg = Odometry()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        (msg.pose.pose.position.x, msg.pose.pose.position.y,
         msg.pose.pose.position.z) = self.position - self.offset
        msg.pose.pose.orientation.w = 1.0
        (msg.twist.twist.linear.x, msg.twist.twist.linear.y,
         msg.twist.twist.linear.z) = self.cmd
        self.odom_pub.publish(msg)


def main():
    rclpy.init()
    node = Node('fake_fence_drone')
    ex = rclpy.executors.MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    p = commander_params(node)
    assert p['fence_enabled'], 'fence_enabled is false in the running commander'
    name = p['drone_names'][0]
    mode = (str(p['drone_modes']).split(',')[0].strip() or 'sim')
    offset = np.array(p['drone_position_offsets'][:3], dtype=float)
    takeoff = np.array(p['hover_positions'][:3], dtype=float)
    fence_x = float(p['fence_max'][0])
    behavior = str(p['fence_behavior'])
    drone = FakeDrone(node, name, mode, offset, p['state_topic_template'].format(name=name))
    goal_pub = node.create_publisher(PoseStamped, f'/svg/{name}/goal_command', 10)
    node.create_timer(DT, lambda: drone.step(node))

    def call(srv):
        c = node.create_client(Trigger, srv)
        assert c.wait_for_service(timeout_sec=5.0), f'{srv} unavailable'
        return wait_future(c.call_async(Trigger.Request()), 5.0, srv)

    def send_goal(xyz):
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
        goal_pub.publish(m)

    def wait_still(timeout, speed=0.02):
        """Wait until the drone has stopped; return the max x seen meanwhile."""
        end = time.monotonic() + timeout
        max_x, last = -np.inf, drone.position.copy()
        while time.monotonic() < end:
            time.sleep(0.5)
            max_x = max(max_x, drone.position[0])
            if np.linalg.norm(drone.position - last) / 0.5 < speed:
                return max_x
            last = drone.position.copy()
        raise AssertionError(f'drone never came to rest (at {np.round(drone.position, 2)})')

    time.sleep(2.0)

    print(f'GEOFENCE TEST ({behavior}, fence_max x = {fence_x} m, {name} {mode})')
    print('TEST 1: takeoff + start')
    assert call(f'{COMMANDER}/takeoff').success
    end = time.monotonic() + 20.0
    while np.linalg.norm(drone.position - takeoff) > 0.25:
        assert time.monotonic() < end, f'never reached takeoff point {takeoff}'
        time.sleep(0.1)
    deadline = time.monotonic() + 15.0
    while not call(f'{COMMANDER}/start').success:
        assert time.monotonic() < deadline, 'start never accepted'
        time.sleep(1.0)
    print('  -> airborne and mission started')

    outside = [fence_x + 5.0, takeoff[1], takeoff[2]]
    if behavior == 'hold_all':
        print('TEST 2: goal OUTSIDE fence -> breach latches, drone freezes')
        send_goal(outside)
        # it freezes a few cm past the boundary, so test for >= the limit
        end = time.monotonic() + 30.0
        while drone.position[0] < fence_x:
            assert time.monotonic() < end, \
                f'drone never reached the fence (x={drone.position[0]:.2f})'
            time.sleep(0.05)
        time.sleep(2.0)  # let the latch + hold settle
        frozen = drone.position.copy()
        time.sleep(2.0)
        drift = np.linalg.norm(drone.position - frozen)
        print(f'  -> latched near x={frozen[0]:.2f}; drift after freeze {drift:.3f} m')
        assert drift < 0.2, f'drone still moving after breach ({drift:.2f} m)'
        assert frozen[0] < fence_x + 1.0, \
            f'drone ran well past the fence to x={frozen[0]:.2f}'

        print('TEST 3: start refused while latched')
        res = call(f'{COMMANDER}/start')
        assert not res.success and 'fence' in res.message.lower(), \
            f'start should be blocked: {res.message}'
        print(f'  -> start refused: "{res.message}"')

        print('TEST 4: reset_fence clears the latch')
        res = call(f'{COMMANDER}/reset_fence')
        assert res.success
        print(f'  -> {res.message}')
    elif behavior == 'keep_in':
        print('TEST 2: goal OUTSIDE fence -> clipped at the wall, never past it')
        send_goal(outside)
        max_x = wait_still(30.0)
        at = drone.position.copy()
        print(f'  -> stopped at x={at[0]:.2f} (max seen {max_x:.2f}, wall {fence_x})')
        assert max_x <= fence_x + 0.05, f'went past the wall to x={max_x:.2f}'
        assert at[0] > fence_x - 1.0, f'stopped far short of the wall (x={at[0]:.2f})'

        print('TEST 3: nothing latched -> start still accepted')
        res = call(f'{COMMANDER}/start')
        assert res.success, f'start refused although keep_in never latches: {res.message}'
        print(f'  -> start accepted: "{res.message}"')

        print('TEST 4: a goal back inside is reached')
        send_goal(takeoff)
        end = time.monotonic() + 30.0
        while np.linalg.norm(drone.position - takeoff) > 0.2:
            assert time.monotonic() < end, \
                f'never came back inside (at {np.round(drone.position, 2)})'
            time.sleep(0.1)
        print(f'  -> back at {np.round(drone.position, 2)}')
    else:
        raise AssertionError(f'unknown fence_behavior {behavior!r}')

    call(f'{COMMANDER}/land')
    print('ALL GEOFENCE TESTS PASSED')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
