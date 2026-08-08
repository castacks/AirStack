#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist

class DummyOdometryPublisher(Node):
    def __init__(self):
        super().__init__('dummy_odometry_publisher')
        self.publisher_ = self.create_publisher(Odometry, '/robot_1/interface/mavros/local_position/odom', 10)
        timer_period = 1./20.
        self.timer = self.create_timer(timer_period, self.publish_dummy_odom)

    def publish_dummy_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'

        # Position (0, 0, 0)
        msg.pose.pose.position = Point(x=0.0, y=0.0, z=0.0)
        msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # No velocity
        msg.twist.twist = Twist()

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DummyOdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

