#!/usr/bin/env python3
"""
PropSpinTest — goes through modalai_interface (not direct to PX4).

Flow:
  1. Wait for modalai_interface to come up (robot_command service appears)
  2. Call robot_command service: REQUEST_CONTROL
  3. Call robot_command service: ARM
  4. Stream cmd_velocity to keep offboard heartbeat alive
  5. Spin for DURATION seconds, then disarm

Run inside robot container:
  ROS_DOMAIN_ID=0 PX4_NAMESPACE=px4_1 python3 prop_spin_test.py [duration_seconds]
"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleControlMode
from airstack_msgs.srv import RobotCommand

DURATION    = int(sys.argv[1]) if len(sys.argv) > 1 else 15
NS          = os.environ.get("PX4_NAMESPACE", "px4_1")
ROBOT_NAME  = os.environ.get("ROBOT_NAME", "robot_1")
SERVICE     = f"/{NS}/fmu/robot_command"
VEL_CMD     = f"/{NS}/fmu/cmd_velocity"
IFACE_ODOM  = f"/{ROBOT_NAME}/interface/odometry"


def main():
    rclpy.init()
    node = Node("prop_spin_test")

    vel_pub = node.create_publisher(TwistStamped, VEL_CMD, 1)
    client  = node.create_client(RobotCommand, SERVICE)

    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
    px4_armed = [False]
    node.create_subscription(VehicleControlMode, f"/{NS}/fmu/out/vehicle_control_mode",
                              lambda m: px4_armed.__setitem__(0, m.flag_armed), qos)

    odom_received = [False]
    node.create_subscription(Odometry, IFACE_ODOM,
                              lambda m: odom_received.__setitem__(0, True), 10)

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

    print(f"PropSpinTest: waiting for modalai_interface ({SERVICE})...", flush=True)
    t0 = time.time()
    while not client.wait_for_service(timeout_sec=2.0):
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - t0 > 120:
            print("PropSpinTest: TIMEOUT waiting for modalai_interface service.", flush=True)
            sys.exit(1)

    print("PropSpinTest: modalai_interface up. Streaming heartbeat for 2s...", flush=True)
    for _ in range(20):
        heartbeat()
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)

    odom_status = "FLOWING" if odom_received[0] else "NOT RECEIVED"
    print(f"PropSpinTest: interface odometry ({IFACE_ODOM}): {odom_status}", flush=True)

    print("PropSpinTest: requesting offboard control...", flush=True)
    send_cmd(RobotCommand.Request.REQUEST_CONTROL)
    time.sleep(0.5)

    print("PropSpinTest: arm command sent — waiting for PX4 confirmation...", flush=True)
    send_cmd(RobotCommand.Request.ARM)

    t0 = time.time()
    while not px4_armed[0] and time.time() - t0 < 20:
        heartbeat()
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)

    if not px4_armed[0]:
        print("PropSpinTest: FAIL — PX4 did not arm within 20s.", flush=True)
        sys.exit(1)

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
