#!/usr/bin/env python3
"""Bridge nav_msgs/Odometry -> geometry_msgs/PoseStamped.

rayfronts' Ros2Subscriber only knows how to subscribe to PoseStamped on its
pose_topic. The airstack pipeline publishes Odometry, so we run this bridge
alongside rayfronts to expose a PoseStamped view of the same pose.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class OdomToPoseStamped(Node):
    def __init__(self):
        super().__init__('odom_to_pose_stamped')
        self.declare_parameter('input_topic',
                               '/robot_1/odometry_conversion/odometry')
        self.declare_parameter('output_topic',
                               '/robot_1/odometry_conversion/pose_stamped')
        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self._pub = self.create_publisher(PoseStamped, out_topic, 10)
        self.create_subscription(Odometry, in_topic, self._cb, 10)
        self.get_logger().info(f'bridging {in_topic} -> {out_topic}')

    def _cb(self, msg: Odometry):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self._pub.publish(ps)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPoseStamped()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
