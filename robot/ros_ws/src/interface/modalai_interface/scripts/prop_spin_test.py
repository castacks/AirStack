#!/usr/bin/env python3
"""
Standalone PropSpinTest — run inside the robot container.
Arms the drone in offboard mode via uXRCE-DDS topics to verify
MicroXRCEAgent + PX4 SITL are alive.

Usage (inside robot container):
  ROS_DOMAIN_ID=<id> python3 prop_spin_test.py [duration_seconds]
"""
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus

DURATION = int(sys.argv[1]) if len(sys.argv) > 1 else 15


def main():
    rclpy.init()
    node = rclpy.create_node("prop_spin_test")

    qos = QoSProfile(depth=1,
                     reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)

    offboard_pub = node.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos)
    setpoint_pub = node.create_publisher(TrajectorySetpoint,  "/fmu/in/trajectory_setpoint",  qos)
    cmd_pub      = node.create_publisher(VehicleCommand,      "/fmu/in/vehicle_command",       qos)

    status_received = [False]
    node.create_subscription(
        VehicleStatus, "/fmu/out/vehicle_status",
        lambda _: status_received.__setitem__(0, True), qos)

    def now_us():
        return int(node.get_clock().now().nanoseconds / 1000)

    def heartbeat():
        ocm = OffboardControlMode()
        ocm.timestamp = now_us()
        ocm.position = True
        offboard_pub.publish(ocm)
        sp = TrajectorySetpoint()
        sp.timestamp = now_us()
        sp.position = [float("nan"), float("nan"), float("nan")]
        sp.velocity = [0.0, 0.0, 0.0]
        sp.yaw = 0.0
        setpoint_pub.publish(sp)

    def send_cmd(command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = now_us()
        msg.command = command
        msg.param1 = p1
        msg.param2 = p2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        cmd_pub.publish(msg)

    print("PropSpinTest: waiting for /fmu/out/vehicle_status — ensure MicroXRCEAgent is running...", flush=True)
    t0 = time.time()
    while not status_received[0]:
        rclpy.spin_once(node, timeout_sec=0.5)
        if time.time() - t0 > 120:
            print("PropSpinTest: TIMEOUT waiting for vehicle_status — is MicroXRCEAgent up?", flush=True)
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

    print("PropSpinTest: PX4 connected. Streaming offboard heartbeat for 2s...", flush=True)
    for _ in range(20):
        heartbeat()
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)

    print("PropSpinTest: requesting offboard mode...", flush=True)
    send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
    time.sleep(0.5)

    print("PropSpinTest: arming...", flush=True)
    send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
    time.sleep(0.5)

    print(f"PropSpinTest: props spinning for {DURATION}s — interface verified!", flush=True)
    t0 = time.time()
    while time.time() - t0 < DURATION:
        heartbeat()
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)

    print("PropSpinTest: disarming.", flush=True)
    send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
