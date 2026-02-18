#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class RelativePositionNode(Node):
    def __init__(self, robot_1_init):
        super().__init__('robot_2_relative_position')
        self.robot_1_init = robot_1_init  # (x, y, z)
        self.robot_1_pos = None
        self.robot_2_pos = None
        self.rel_pub = self.create_publisher(Odometry, '/robot_2/relative_position', 10)
        self.create_subscription(
            Odometry,
            '/robot_1/odometry_conversion/odometry',
            self.robot_1_callback,
            10
        )
        self.create_subscription(
            Odometry,
            '/robot_2/odometry_conversion/odometry',
            self.robot_2_callback,
            10
        )

    def robot_1_callback(self, msg):
        self.robot_1_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )

    def robot_2_callback(self, msg):
        self.robot_2_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        self.publish_relative(msg)

    def publish_relative(self, msg):
        if self.robot_1_pos is None or self.robot_2_pos is None:
            return
        rel = [self.robot_2_pos[i] - self.robot_1_pos[i] for i in range(3)]
        rel_msg = Odometry()
        rel_msg.header = msg.header
        rel_msg.pose.pose.position.x = rel[0]
        rel_msg.pose.pose.position.y = rel[1]
        rel_msg.pose.pose.position.z = rel[2]
        self.rel_pub.publish(rel_msg)

def main(args=None):
    rclpy.init(args=args)
    robot_1_init = [20.0, -7, 0.0015]
    node = RelativePositionNode(robot_1_init)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
