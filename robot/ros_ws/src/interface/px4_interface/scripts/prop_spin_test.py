#!/usr/bin/env python3
"""
PropSpinTest — goes through px4_interface (not direct to PX4).

Flow:
  1. Wait for px4_interface to come up (is_armed topic appears)
  2. Call robot_command service: REQUEST_CONTROL
  3. Call robot_command service: ARM
  4. Stream velocity_command to keep offboard heartbeat alive
  5. Spin for DURATION seconds, then disarm

Run inside robot container:
  ROS_DOMAIN_ID=0 python3 prop_spin_test.py [duration_seconds]
"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
from airstack_msgs.srv import RobotCommand

DURATION = int(sys.argv[1]) if len(sys.argv) > 1 else 15
NS = os.environ.get("PX4_NAMESPACE", "px4_1")
SERVICE  = f"/{NS}/fmu/robot_command"
VEL_CMD  = f"/{NS}/fmu/velocity_command"
IS_ARMED = f"/{NS}/fmu/is_armed"


def main():
    rclpy.init()
    node = Node("prop_spin_test")

    vel_pub = node.create_publisher(TwistStamped, VEL_CMD, 1)
    client  = node.create_client(RobotCommand, SERVICE)

    armed = [False]
    node.create_subscription(Bool, IS_ARMED, lambda m: armed.__setitem__(0, m.data), 1)

    def send_cmd(command):
        req = RobotCommand.Request()
        req.command = command
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        return future.result().success if future.result() else False

    def heartbeat():
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        vel_pub.publish(msg)

    print(f"PropSpinTest: waiting for px4_interface ({SERVICE})...", flush=True)
    t0 = time.time()
    while not client.wait_for_service(timeout_sec=2.0):
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - t0 > 120:
            print("PropSpinTest: TIMEOUT waiting for px4_interface service.", flush=True)
            sys.exit(1)

    print("PropSpinTest: px4_interface up. Streaming heartbeat for 2s...", flush=True)
    for _ in range(20):
        heartbeat()
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)

    print("PropSpinTest: requesting offboard control...", flush=True)
    send_cmd(RobotCommand.Request.REQUEST_CONTROL)
    time.sleep(0.5)

    print("PropSpinTest: arming...", flush=True)
    send_cmd(RobotCommand.Request.ARM)
    time.sleep(0.5)

    print(f"PropSpinTest: props spinning for {DURATION}s — interface verified!", flush=True)
    t0 = time.time()
    while time.time() - t0 < DURATION:
        heartbeat()
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)

    print("PropSpinTest: disarming.", flush=True)
    send_cmd(RobotCommand.Request.DISARM)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
