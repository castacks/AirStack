#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class Robot2RelToRobot1Start(Node):
    """
    Publishes robot_2 position expressed in a frame whose origin is robot_1's
    INITIAL world position.

    Inputs:
      /robot_2/odometry_conversion/odometry  (robot2 LOCAL odom; starts near 0,0,0)

    Known constants (world):
      robot_1_init, robot_2_init

    Output:
      /robot_2/relative_position (Odometry) where:
        p_rel = (robot_2_init - robot_1_init) + p2_local
    """

    def __init__(self, robot_1_init, robot_2_init):
        super().__init__('robot_2_relative_to_robot_1_start')

        self.robot_1_init = tuple(robot_1_init)  # (x,y,z) in world
        self.robot_2_init = tuple(robot_2_init)  # (x,y,z) in world

        # Constant offset from robot1 start to robot2 start in world
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

    def robot_2_callback(self, msg: Odometry):
        p2 = msg.pose.pose.position  # robot2 LOCAL (0,0,0 at its start)

        rel_msg = Odometry()
        rel_msg.header = msg.header

        # Make the frame semantics explicit (optional but helpful)
        rel_msg.header.frame_id = 'robot_1_start'
        rel_msg.child_frame_id = 'robot_2_relative'

        rel_msg.pose.pose.position.x = self.offset[0] + p2.x
        rel_msg.pose.pose.position.y = self.offset[1] + p2.y
        rel_msg.pose.pose.position.z = self.offset[2] + p2.z

        # If you only care about position, set identity orientation
        rel_msg.pose.pose.orientation.w = 1.0

        self.rel_pub.publish(rel_msg)


def main(args=None):
    rclpy.init(args=args)

    robot_1_init = (-3.0, 3.5, 0.15)
    robot_2_init = ( 3.0, 3.0, 0.15)

    node = Robot2RelToRobot1Start(robot_1_init, robot_2_init)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
