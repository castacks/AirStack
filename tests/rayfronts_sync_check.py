#!/usr/bin/env python3
"""Probe rayfronts' three synced input topics: per-topic rate + stamp offsets.

Run inside a robot container (ROS_DOMAIN_ID already set per robot):
    python3 rayfronts_sync_check.py

Interpreting the output:
  - a topic stuck at 0.0Hz            -> delivery/QoS problem on that topic
  - |rgb-depth| or |rgb-pose| >> slop -> stamp/clock mismatch (sim vs wall
    time, or restamping); the synchronizer can only pair within its slop
  - rates healthy and offsets small   -> sync is fine; slow mapping is the
    encoder/GPU or frame_skip, not message matching
"""
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

ROBOT = os.getenv('ROBOT_NAME', f"robot_{os.getenv('ROS_DOMAIN_ID', '1')}")
TOPICS = {
    'rgb':   (Image,       f'/{ROBOT}/sensors/front_stereo/left/image_rect'),
    'depth': (Image,       f'/{ROBOT}/sensors/front_stereo/left/depth_ground_truth'),
    'pose':  (PoseStamped, f'/{ROBOT}/odometry_conversion/pose_stamped'),
}


class SyncCheck(Node):
    def __init__(self):
        super().__init__('rayfronts_sync_check')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.last_stamp = {}
        self.count = {k: 0 for k in TOPICS}
        for key, (typ, topic) in TOPICS.items():
            self.create_subscription(
                typ, topic, lambda m, k=key: self._cb(k, m), qos)
            self.get_logger().info(f'{key}: {topic}')
        self.t0 = time.time()
        self.create_timer(2.0, self._report)

    def _cb(self, key, msg):
        self.last_stamp[key] = (msg.header.stamp.sec
                                + msg.header.stamp.nanosec * 1e-9)
        self.count[key] += 1

    def _report(self):
        elapsed = time.time() - self.t0
        line = ' | '.join(f'{k}: {self.count[k] / elapsed:5.1f}Hz'
                          for k in TOPICS)
        missing = [k for k in TOPICS if k not in self.last_stamp]
        if missing:
            line += f' | NO MESSAGES yet from: {missing}'
        else:
            s = self.last_stamp
            line += (f" | offsets: rgb-depth={s['rgb'] - s['depth']:+8.3f}s"
                     f"  rgb-pose={s['rgb'] - s['pose']:+8.3f}s"
                     f"  (wall-rgb={time.time() - s['rgb']:+8.3f}s)")
        print(line, flush=True)


def main():
    rclpy.init()
    try:
        rclpy.spin(SyncCheck())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
