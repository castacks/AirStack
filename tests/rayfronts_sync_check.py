#!/usr/bin/env python3
"""Probe rayfronts' three synced input topics: per-topic rate + stamp offsets.

Subscriptions adopt each discovered publisher's QoS (like `ros2 topic hz`),
and the publisher's type + QoS are printed, so a QoS/type mismatch shows up
explicitly instead of as silent 0.0Hz.

Run inside a robot container (ROS_DOMAIN_ID already set per robot):
    python3 rayfronts_sync_check.py

Interpreting the output:
  - "no publisher discovered"          -> nothing is publishing that topic
  - a topic stuck at 0.0Hz with a
    publisher present                  -> delivery problem (QoS shown above)
  - |rgb-depth| or |rgb-pose| >> slop  -> stamp/clock mismatch; the
    synchronizer can only pair within its slop
  - rates healthy and offsets small    -> sync is fine; slow mapping is the
    encoder/GPU or frame_skip, not message matching
"""
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
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
        self.last_stamp = {}
        self.count = {k: 0 for k in TOPICS}
        self.subs = {}
        self.t0 = time.time()
        self.create_timer(2.0, self._tick)

    def _try_subscribe(self, key):
        typ, topic = TOPICS[key]
        infos = self.get_publishers_info_by_topic(topic)
        if not infos:
            return False
        info = infos[0]
        qos = QoSProfile(
            depth=10,
            reliability=info.qos_profile.reliability,
            durability=info.qos_profile.durability,
        )
        self.get_logger().info(
            f'{key}: {topic}\n'
            f'    publisher type={info.topic_type} node={info.node_name} '
            f'reliability={info.qos_profile.reliability.name} '
            f'durability={info.qos_profile.durability.name} '
            f'(adopting this QoS)')
        self.subs[key] = self.create_subscription(
            typ, topic, lambda m, k=key: self._cb(k, m), qos)
        return True

    def _cb(self, key, msg):
        self.last_stamp[key] = (msg.header.stamp.sec
                                + msg.header.stamp.nanosec * 1e-9)
        self.count[key] += 1

    def _tick(self):
        for key in TOPICS:
            if key not in self.subs and not self._try_subscribe(key):
                self.get_logger().warn(
                    f'{key}: no publisher discovered on {TOPICS[key][1]}')
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
