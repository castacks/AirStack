"""Closed-loop test: geofence breach latches a swarm-wide hold.

Run against a live commander launched with goal_single.yaml (fence enabled,
fence_max x = 3.5):

    # terminal A:
    ros2 launch svg_ground_control ground_control.launch.py \
        config:=<share>/config/goal_single.yaml
    # terminal B:
    python3 test/functional_fence_test.py

Takeoff -> start -> command a goal OUTSIDE the fence -> verify the drone is
frozen (held) near the fence boundary, that start is refused while latched,
and that ~/reset_fence clears it.
"""

import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from airstack_msgs.srv import RobotCommand

NAME = 'drone_1'
GROUND = np.array([0.0, 0.0, 0.05])
TAKEOFF = np.array([0.0, 0.0, 1.2])
FENCE_MAX_X = 3.5      # from goal_single.yaml
DT = 0.02


class FakeDrone:
    def __init__(self, node):
        self.position = GROUND.copy()
        self.cmd = np.zeros(3)
        self.odom_pub = node.create_publisher(
            Odometry, f'/{NAME}/odometry_conversion/odometry', 10)
        node.create_subscription(
            TwistStamped, f'/{NAME}/interface/velocity_command', self.cmd_cb, 10)
        node.create_service(
            RobotCommand, f'/{NAME}/interface/robot_command',
            lambda req, res: (setattr(res, 'success', True) or res))

    def cmd_cb(self, msg):
        self.cmd = np.array([msg.twist.linear.x, msg.twist.linear.y,
                             msg.twist.linear.z])

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
    node = Node('fake_fence_drone')
    drone = FakeDrone(node)
    goal_pub = node.create_publisher(PoseStamped, f'/svg/{NAME}/goal_command', 10)
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

    time.sleep(3.0)

    print('GEOFENCE TEST (fence_max x = 3.5 m)')
    print('TEST 1: takeoff + start')
    assert call('/swarm_commander/takeoff').success
    end = time.monotonic() + 20.0
    while np.linalg.norm(drone.position - TAKEOFF) > 0.25:
        assert time.monotonic() < end
        time.sleep(0.1)
    deadline = time.monotonic() + 15.0
    while not call('/swarm_commander/start').success:
        assert time.monotonic() < deadline, 'start never accepted'
        time.sleep(1.0)
    print('  -> airborne and mission started')

    print('TEST 2: goal OUTSIDE fence -> breach latches, drone freezes')
    m = PoseStamped()
    m.header.frame_id = 'map'
    m.pose.position.x, m.pose.position.y, m.pose.position.z = 10.0, 0.0, 1.2
    goal_pub.publish(m)
    # wait until it crosses the fence and the breach latches it (it freezes
    # a few cm past the boundary, so test for >= the limit, not well past it)
    end = time.monotonic() + 30.0
    while drone.position[0] < FENCE_MAX_X:
        assert time.monotonic() < end, \
            f'drone never reached the fence (x={drone.position[0]:.2f})'
        time.sleep(0.05)
    time.sleep(2.0)  # let the latch + hold settle
    frozen = drone.position.copy()
    time.sleep(2.0)
    drift = np.linalg.norm(drone.position - frozen)
    print(f'  -> latched near x={frozen[0]:.2f}; drift after freeze {drift:.3f} m')
    assert drift < 0.2, f'drone still moving after breach ({drift:.2f} m)'
    # not far past the fence (it stopped, did not run to x=10)
    assert frozen[0] < FENCE_MAX_X + 1.0, \
        f'drone ran well past the fence to x={frozen[0]:.2f}'

    print('TEST 3: start refused while latched')
    res = call('/swarm_commander/start')
    assert not res.success and 'fence' in res.message.lower(), \
        f'start should be blocked: {res.message}'
    print(f'  -> start refused: "{res.message}"')

    print('TEST 4: reset_fence clears the latch')
    res = call('/swarm_commander/reset_fence')
    assert res.success
    print(f'  -> {res.message}')

    call('/swarm_commander/land')
    print('ALL GEOFENCE TESTS PASSED')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
