#!/usr/bin/env python3
"""Lightweight launcher node that starts/stops BPMP tracker nodes on demand.

Subscribes to target_tracking_enable (std_msgs/Bool). On True, launches
bpmp_tracker.launch.xml as a subprocess; on False, terminates it.
"""
import os
import signal
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class BPMPLauncherNode(Node):
    def __init__(self):
        super().__init__('bpmp_launcher')
        self._proc = None

        self.declare_parameter('odometry_topic', '')

        self.create_subscription(Bool, 'target_tracking_enable', self._on_enable, 10)
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
        self._proc = subprocess.Popen(cmd, env=os.environ.copy())

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
