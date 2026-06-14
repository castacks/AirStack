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
  ROS_DOMAIN_ID=0 python3 prop_spin_test.py [duration_seconds] --force
      --force: bypass PX4 preflight checks (use when EKF2 yaw not initialized, e.g. bench with no visual features)
"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleControlMode, VehicleStatus, VehicleCommand
from airstack_msgs.srv import RobotCommand

FORCE_ARM   = "--force" in sys.argv
DURATION    = int(next((a for a in sys.argv[1:] if not a.startswith("-")), "15"))
NS          = os.environ.get("PX4_NAMESPACE", "")
ROBOT_NAME  = os.environ.get("ROBOT_NAME", "robot_1")
FMU_PREFIX  = f"/{NS}/fmu" if NS else "/fmu"
SERVICE     = f"{FMU_PREFIX}/robot_command"
VEL_CMD     = f"{FMU_PREFIX}/cmd_velocity"
IFACE_ODOM  = f"/{ROBOT_NAME}/interface/odometry"


def main():
    rclpy.init()
    node = Node("prop_spin_test")

    vel_pub = node.create_publisher(TwistStamped, VEL_CMD, 1)
    client  = node.create_client(RobotCommand, SERVICE)

    # Publisher for direct PX4 vehicle commands (used for force arm)
    cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
    vehicle_cmd_pub = node.create_publisher(VehicleCommand, f"{FMU_PREFIX}/in/vehicle_command", cmd_qos)

    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
    px4_armed = [False]
    node.create_subscription(VehicleControlMode, f"{FMU_PREFIX}/out/vehicle_control_mode",
                              lambda m: px4_armed.__setitem__(0, m.flag_armed), qos)
    # vehicle_status_v1 is the hardware topic name (PX4 v1.15+); arming_state==2 means armed
    node.create_subscription(VehicleStatus, f"{FMU_PREFIX}/out/vehicle_status_v1",
                              lambda m: px4_armed.__setitem__(0, m.arming_state == VehicleStatus.ARMING_STATE_ARMED), qos)

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

    if FORCE_ARM:
        # Bypass preflight checks (use when EKF2 yaw not initialized, e.g. bench with no visual features).
        # param2=21196.0 is PX4's MAVLink force-arm override key.
        time.sleep(1.0)
        print("PropSpinTest: --force: sending direct force-arm override to PX4...", flush=True)
        cmd = VehicleCommand()
        cmd.timestamp = node.get_clock().now().nanoseconds // 1000
        cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 1.0   # ARM
        cmd.param2 = 21196.0  # force override (bypasses preflight)
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        for _ in range(5):
            cmd.timestamp = node.get_clock().now().nanoseconds // 1000
            vehicle_cmd_pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)

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
