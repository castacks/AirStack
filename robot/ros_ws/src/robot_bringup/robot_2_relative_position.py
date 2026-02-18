#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class RelativePositionNode(Node):
    def __init__(self, robot_1_init, robot_2_init):
        super().__init__('robot_2_relative_position')
        self.robot_1_init = tuple(robot_1_init)
        self.robot_2_init = tuple(robot_2_init)
        self.offset = (
            self.robot_2_init[0] - self.robot_1_init[0],
            self.robot_2_init[1] - self.robot_1_init[1],
            self.robot_2_init[2] - self.robot_1_init[2],
        )

        self.rel_pub = self.create_publisher(Odometry, '/robot_2/relative_position', 10)
        self.create_subscription(
            Odometry,
            '/robot_2/odometry_conversion/odometry',
            self.robot_2_callback,
            10
        )

    def robot_2_callback(self, msg):
        rel_msg = Odometry()
        rel_msg.header = msg.header
        rel_msg.header.frame_id = 'robot_1_start'
        rel_msg.child_frame_id = 'robot_2_relative'

        rel_msg.pose.pose.position.x = self.offset[0] + msg.pose.pose.position.x
        rel_msg.pose.pose.position.y = self.offset[1] + msg.pose.pose.position.y
        rel_msg.pose.pose.position.z = self.offset[2] + msg.pose.pose.position.z
        rel_msg.pose.pose.orientation.w = 1.0
        self.rel_pub.publish(rel_msg)


def main(args=None):
    rclpy.init(args=args)
    robot_1_init = (20.0, -7.0, 0.15)
    robot_2_init = (17.0, 1.5, 0.15)
    node = RelativePositionNode(robot_1_init, robot_2_init)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
