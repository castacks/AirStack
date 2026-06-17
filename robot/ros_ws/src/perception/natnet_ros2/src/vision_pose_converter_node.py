#!/usr/bin/env python3

"""
Vision Pose Converter Node

Optional converter that bridges NatNet pose data to MAVROS vision_pose format
for PX4 external pose estimation and state fusion.

Converts from NatNet coordinate frame to a frame suitable for MAVROS.

Topics are configurable so the bridge can be retargeted to other middleware.
``input_topic`` / ``output_pose_topic`` / ``output_pose_cov_topic`` default to the
relative names ``input_pose`` / ``output_pose`` / ``output_pose_cov`` (remappable),
but natnet_ros2.launch.py overrides them with the absolute, ROBOT_NAME-namespaced
topics from the robot's ``vision_pose`` block in natnet_config.yaml.
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

        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('canonical_quaternion', True)
        # Topic names — overridden by the launch file from the per-robot
        # vision_pose block; defaults are the historical remappable relative names.
        self.declare_parameter('input_topic', 'input_pose')
        self.declare_parameter('output_pose_topic', 'output_pose')
        self.declare_parameter('output_pose_cov_topic', 'output_pose_cov')

        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.canonical_quaternion = self.get_parameter('canonical_quaternion').value
        input_topic = self.get_parameter('input_topic').value
        output_pose_topic = self.get_parameter('output_pose_topic').value
        output_pose_cov_topic = self.get_parameter('output_pose_cov_topic').value

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            input_topic,
            self._on_pose,
            10
        )

        # Publishers
        self.pose_pub = self.create_publisher(
            PoseStamped,
            output_pose_topic,
            10
        )
        self.pose_cov_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            output_pose_cov_topic,
            10
        )

        self.get_logger().info(
            f'Vision pose converter started '
            f'(frame_id={self.frame_id!r}, child_frame_id={self.child_frame_id!r}, '
            f'canonical_quaternion={self.canonical_quaternion}, '
            f'input_topic={input_topic!r}, output_pose_topic={output_pose_topic!r}, '
            f'output_pose_cov_topic={output_pose_cov_topic!r})'
        )

    @staticmethod
    def _canonical_quaternion(o):
        """
        Return the quaternion in canonical form (qw >= 0) by negating all
        four components when qw < 0.

        q and -q represent the same 3-D rotation, but some EKF implementations
        (ArduPilot EKF3 in particular) are sensitive to sign flips between
        consecutive frames.  Keeping qw >= 0 guarantees a consistent
        representation across the full orientation space.
        """
        if o.w < 0.0:
            o.x, o.y, o.z, o.w = -o.x, -o.y, -o.z, -o.w
        return o

    def _on_pose(self, msg: PoseWithCovarianceStamped):
        """
        Callback for incoming NatNet pose.

        Converts and republishes for MAVROS consumption.
        Normalises the quaternion to canonical form (qw >= 0) before publishing
        so that EKF consumers never see a sign-flip discontinuity.
        """
        try:
            msg.header.frame_id = self.frame_id
            if self.canonical_quaternion:
                msg.pose.pose.orientation = self._canonical_quaternion(
                    msg.pose.pose.orientation
                )

            self.pose_cov_pub.publish(msg)

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
