#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

class OdomModifier(Node):
    def __init__(self):
        super().__init__('odom_modifier')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1)
        self.odom_subscriber = self.create_subscription(Odometry, 'odom_in', self.odom_callback, qos)
        self.odom_publisher = self.create_publisher(Odometry, 'odom_out', 1)

    def odom_callback(self, msg):
        modified_odom = Odometry()
        modified_odom.header = msg.header
        modified_odom.child_frame_id = msg.child_frame_id
        modified_odom.twist = msg.twist
        modified_odom.pose = msg.pose
        
        self.odom_publisher.publish(modified_odom)

if __name__ == '__main__':
    rclpy.init(args=None)
    odom_modifier_node = OdomModifier()
    rclpy.spin(odom_modifier_node)
    odom_modifier_node.destroy_node()
    rclpy.shutdown()
