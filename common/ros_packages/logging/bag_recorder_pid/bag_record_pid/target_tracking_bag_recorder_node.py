#!/usr/bin/env python3
"""Rosbag recorder that auto-starts when target tracking is enabled.

Subscribes to bpmp/target_tracking_enable (std_msgs/Bool).
- True  → starts MCAP recording
- False → stops recording
- Node shutdown (container stop) → stops recording
"""
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from bag_record_pid.bag_record_node import BagRecorderNode


class TargetTrackingBagRecorderNode(BagRecorderNode):
    def __init__(self):
        super().__init__()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.create_subscription(Bool, 'bpmp/target_tracking_enable', self._on_enable, qos)
        self.get_logger().info('Target tracking bag recorder ready — waiting for target_tracking_enable')

    def _on_enable(self, msg: Bool):
        if msg.data:
            self.run()
        else:
            self.interrupt()


def main(args=None):
    rclpy.init(args=args)
    node = TargetTrackingBagRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.interrupt()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
