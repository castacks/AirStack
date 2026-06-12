#!/usr/bin/env python3
"""Lightweight launcher node that starts/stops BPMP tracker nodes on demand.

Subscribes to target_tracking_enable (std_msgs/Bool). On True, launches
bpmp_tracker.launch.xml as a subprocess; on False, terminates it.
"""
import os
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class BPMPLauncherNode(Node):
    def __init__(self):
        super().__init__('bpmp_launcher')
        self._proc = None

        self.declare_parameter('odometry_topic', '')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        self.create_subscription(Bool, 'target_tracking_enable', self._on_enable, qos)
        self._enable_pub = self.create_publisher(Bool, 'target_tracking_enable', qos)
        self.get_logger().info('BPMP Launcher ready — waiting for target_tracking_enable')

    def _on_enable(self, msg: Bool):
        if msg.data:
            self._start()
        else:
            self._stop()

    def _start(self):
        if self._proc is not None and self._proc.poll() is None:
            self.get_logger().info('BPMP nodes already running')
            return

        cmd = ['ros2', 'launch', 'bpmp_tracker', 'bpmp_tracker.launch.xml']
        odometry_topic = self.get_parameter('odometry_topic').get_parameter_value().string_value
        if odometry_topic:
            cmd.append(f'odometry_topic:={odometry_topic}')

        self.get_logger().info(f'Starting BPMP tracker nodes: {" ".join(cmd)}')
        self._proc = subprocess.Popen(
            cmd, env=os.environ.copy(),
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
        )
        threading.Thread(target=self._log_proc_output, daemon=True).start()
        threading.Thread(target=self._confirm_enable_after_start, daemon=True).start()

    def _confirm_enable_after_start(self):
        time.sleep(3.0)
        msg = Bool()
        msg.data = True
        self._enable_pub.publish(msg)
        self.get_logger().info('Re-published tracking enable to newly started tracker node')

    def _log_proc_output(self):
        for line in self._proc.stdout:
            self.get_logger().info(f'[bpmp_launch] {line.rstrip()}')
        rc = self._proc.wait()
        self.get_logger().warn(f'BPMP launch process exited with code {rc}')

    def _stop(self):
        if self._proc is None or self._proc.poll() is not None:
            self.get_logger().info('BPMP nodes are not running')
            self._proc = None
            return

        self.get_logger().info('Stopping BPMP tracker nodes...')
        self._proc.send_signal(signal.SIGINT)
        try:
            self._proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self._proc.kill()
        self._proc = None

    def destroy_node(self):
        self._stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = BPMPLauncherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
