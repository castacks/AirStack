#!/usr/bin/env python3
"""Republish airstack_msgs/Odometry tracking_point as PoseStamped so RViz can show it.

Usage:
  python3 tracking_point_viz.py
Then in RViz add a Pose display on /robot_1/trajectory_controller/tracking_point_pose
(and optionally an Odometry display on .../tracking_point_odom to see a trail).
"""
import rclpy
from rclpy.node import Node
from airstack_msgs.msg import Odometry as AirstackOdometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry as NavOdometry

TOPIC_IN = '/robot_1/trajectory_controller/tracking_point'


class TrackingPointViz(Node):
    def __init__(self):
        super().__init__('tracking_point_viz')
        self.pose_pub = self.create_publisher(PoseStamped, TOPIC_IN + '_pose', 10)
        self.odom_pub = self.create_publisher(NavOdometry, TOPIC_IN + '_odom', 10)
        self.sub = self.create_subscription(AirstackOdometry, TOPIC_IN, self.callback, 10)
        self.get_logger().info(f'Relaying {TOPIC_IN} -> _pose (PoseStamped) and _odom (nav_msgs/Odometry)')

    def callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose
        self.pose_pub.publish(pose)

        odom = NavOdometry()
        odom.header = msg.header
        odom.child_frame_id = msg.child_frame_id
        odom.pose.pose = msg.pose
        odom.twist.twist = msg.twist
        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = TrackingPointViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
