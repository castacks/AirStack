#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomBridgeNode(Node):
    def __init__(self):
        super().__init__('robot_1_odom_bridge')
        self.pub = self.create_publisher(Odometry, '/robot_1/odom_bridge', 10)
        self.sub = self.create_subscription(
            Odometry,
            '/robot_1/odometry_conversion/odometry',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
