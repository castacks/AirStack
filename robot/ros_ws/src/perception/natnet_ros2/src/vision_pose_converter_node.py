#!/usr/bin/env python3

"""
Vision Pose Converter Node

Optional converter that bridges NatNet pose data to MAVROS vision_pose format
for PX4 external pose estimation and state fusion.

Converts from NatNet coordinate frame to a frame suitable for MAVROS.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class VisionPoseConverterNode(Node):
    """
    Converts NatNet pose to MAVROS vision_pose format.
    
    Listens to NatNet pose data and publishes to MAVROS for external
    pose feedback to PX4 autopilot.
    """

    def __init__(self):
        super().__init__('vision_pose_converter')

        # Declare parameters
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('child_frame_id', 'base_link')

        # Get parameters
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'input_pose',
            self._on_pose,
            10
        )

        # Publishers
        self.pose_pub = self.create_publisher(
            PoseStamped,
            'output_pose',
            10
        )
        self.pose_cov_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'output_pose_cov',
            10
        )

        self.get_logger().info("Vision pose converter node started")

    def _on_pose(self, msg: PoseWithCovarianceStamped):
        """
        Callback for incoming NatNet pose.
        
        Converts and republishes for MAVROS consumption.
        """
        try:
            # Stamp the message with the configured reference frame ID
            msg.header.frame_id = self.frame_id

            # Republish with covariance
            self.pose_cov_pub.publish(msg)

            # Also publish PoseStamped version
            pose_msg = PoseStamped()
            pose_msg.header = msg.header
            pose_msg.pose = msg.pose.pose
            self.pose_pub.publish(pose_msg)

        except Exception as e:
            self.get_logger().error(f"Error converting pose: {e}")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    try:
        node = VisionPoseConverterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
