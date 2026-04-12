#!/usr/bin/python3
import os

import rclpy
from rclpy.node import Node
from airstack_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

ROBOT_NAME = os.getenv('ROBOT_NAME', '')


class PoseHoldBridge(Node):
    def __init__(self):
        super().__init__('pose_hold_bridge')

        self.latest_tracking_point: PoseStamped | None = None

        self.declare_parameter('publish_rate', 1.0)
        rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.tracking_point_sub = self.create_subscription(
            Odometry,
            f'/{ROBOT_NAME}/trajectory_controller/tracking_point',
            self.tracking_point_callback,
            1,
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            f'/{ROBOT_NAME}/interface/pose_command',
            1,
        )

        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

    def tracking_point_callback(self, msg: Odometry):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose
        self.latest_tracking_point = pose

    def timer_callback(self):
        if self.latest_tracking_point is None:
            return
        self.latest_tracking_point.header.stamp = self.get_clock().now().to_msg()
        self.pose_pub.publish(self.latest_tracking_point)


def main():
    rclpy.init()
    node = PoseHoldBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
