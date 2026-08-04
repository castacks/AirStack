#!/usr/bin/env python3

"""
PX4 Vision Bridge Node

Feeds motion-capture pose to PX4 EKF2 as external vision over uXRCE-DDS, so a
drone can hold position indoors with no GPS. This is the px4_interface analogue
of vision_pose_converter_node.py, which serves the same purpose over MAVROS.

    natnet_ros2 ──PoseStamped──▶ [this node] ──VehicleOdometry──▶ PX4 EKF2
                  input_pose                    output_visual_odometry
                                                (/{robot}/fmu/in/vehicle_visual_odometry)

The conversion mirrors the flight-proven path from svg_ground_control's
mocap_bridge: timestamp is left at 0 so the uXRCE-DDS client restamps with PX4's
own high-resolution clock (a ground-PC timestamp gets rejected by EKF2), and the
velocity fields are NaN so EKF2 fuses pose only rather than a noisy
finite-difference.

The vehicle also needs EKF2_EV_CTRL enabled, EKF2_HGT_REF=Vision and
EKF2_GPS_CTRL=0 before it will fuse this and allow arming.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry

NAN = float('nan')
SQRT2_INV = 0.70710678118654752


def qmul(a, b):
    """Hamilton product of two [w, x, y, z] quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw]


def enu_flu_to_ned_frd(qw, qx, qy, qz):
    """ROS (FLU body, ENU world) quaternion -> PX4 (FRD body, NED world)."""
    tmp = qmul([0.0, SQRT2_INV, SQRT2_INV, 0.0], [qw, qx, qy, qz])
    return qmul(tmp, [0.0, 1.0, 0.0, 0.0])


class PX4VisionBridgeNode(Node):
    """Republishes mocap PoseStamped as px4_msgs/VehicleOdometry for EKF2."""

    def __init__(self):
        super().__init__('px4_vision_bridge')

        # 'enu_to_ned'   — input is ROS-standard ENU world / FLU body.
        # 'modalai_flip' — 180 deg flip about X on both world and body, matching
        #                  the ModalAI reference bridge. Use when the mocap
        #                  driver republishes Motive's raw Y-up frame instead.
        self.declare_parameter('frame_convention', 'enu_to_ned')
        self.declare_parameter('quality', 100)
        self.declare_parameter('position_variance', 1e-4)
        self.declare_parameter('orientation_variance', 1e-4)
        self.declare_parameter('input_qos_best_effort', False)

        self.frame_convention = str(self.get_parameter('frame_convention').value)
        if self.frame_convention not in ('enu_to_ned', 'modalai_flip'):
            raise ValueError(
                "frame_convention must be 'enu_to_ned' or 'modalai_flip', "
                f'got {self.frame_convention!r}'
            )
        self.quality = int(self.get_parameter('quality').value)
        self.position_variance = float(self.get_parameter('position_variance').value)
        self.orientation_variance = float(
            self.get_parameter('orientation_variance').value)

        sub_qos = QoSProfile(depth=10)
        if bool(self.get_parameter('input_qos_best_effort').value):
            sub_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        # PX4 /fmu/in/* topics expect BEST_EFFORT + VOLATILE to match the client.
        px4_qos = QoSProfile(depth=10,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST)

        self.pose_sub = self.create_subscription(
            PoseStamped, 'input_pose', self._on_pose, sub_qos)
        self.odom_pub = self.create_publisher(
            VehicleOdometry, 'output_visual_odometry', px4_qos)

        self._logged_first = False
        self.get_logger().info(
            f'PX4 vision bridge started (frame_convention={self.frame_convention!r}, '
            f'quality={self.quality})'
        )

    def _convert(self, position, q_wxyz):
        """Return (px4_position, px4_quaternion, pose_frame) for the chosen convention."""
        x, y, z = position
        if self.frame_convention == 'modalai_flip':
            flip = [0.0, 1.0, 0.0, 0.0]   # 180 deg about X, [w, x, y, z]
            return ([x, -y, -z],
                    qmul(qmul(flip, list(q_wxyz)), flip),
                    VehicleOdometry.POSE_FRAME_FRD)
        return ([y, x, -z],
                enu_flu_to_ned_frd(*q_wxyz),
                VehicleOdometry.POSE_FRAME_NED)

    def _on_pose(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation

        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self.get_logger().warn('Dropping mocap pose with non-finite position',
                                   throttle_duration_sec=5.0)
            return

        px4_p, px4_q, pose_frame = self._convert((p.x, p.y, p.z),
                                                 (o.w, o.x, o.y, o.z))

        odom = VehicleOdometry()
        # Left at 0 so the uXRCE-DDS client restamps with PX4's internal clock.
        odom.timestamp = 0
        odom.timestamp_sample = 0
        odom.pose_frame = pose_frame
        odom.position = [float(v) for v in px4_p]
        odom.q = [float(v) for v in px4_q]
        odom.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        odom.velocity = [NAN, NAN, NAN]
        odom.angular_velocity = [NAN, NAN, NAN]
        odom.position_variance = [self.position_variance] * 3
        odom.orientation_variance = [self.orientation_variance] * 3
        odom.velocity_variance = [NAN, NAN, NAN]
        odom.reset_counter = 0
        odom.quality = self.quality
        self.odom_pub.publish(odom)

        if not self._logged_first:
            self._logged_first = True
            self.get_logger().info(
                'First mocap pose forwarded to PX4: '
                f'ENU=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) -> '
                f'PX4=({px4_p[0]:.3f}, {px4_p[1]:.3f}, {px4_p[2]:.3f}). '
                'Verify this matches the drone\'s real position before arming.'
            )


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    try:
        node = PX4VisionBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
