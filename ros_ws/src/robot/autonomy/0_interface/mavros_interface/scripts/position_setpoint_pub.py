#!/usr/bin/python3
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from airstack_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class OdomModifier(Node):
    def __init__(self):
        super().__init__('odom_modifier')
        '''
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1)
        '''
        self.odom_subscriber = self.create_subscription(Odometry, "/" + os.getenv('ROBOT_NAME', "") + '/trajectory_controller/tracking_point', self.odom_callback, 1)
        self.odom_publisher = self.create_publisher(PoseStamped, 'cmd_pose', 1)

    def odom_callback(self, msg):
        out = PoseStamped()
        out.header = msg.header
        out.header.frame_id = 'base_link'
        out.pose = msg.pose
        self.odom_publisher.publish(out)

if __name__ == '__main__':
    rclpy.init(args=None)
    odom_modifier_node = OdomModifier()
    rclpy.spin(odom_modifier_node)
    odom_modifier_node.destroy_node()
    rclpy.shutdown()
