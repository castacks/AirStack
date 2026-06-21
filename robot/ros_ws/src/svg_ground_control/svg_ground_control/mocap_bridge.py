"""Multi-drone mocap-to-PX4 external-vision bridge.

Subscribes to one mocap pose topic per drone (default ``/{name}/pose``,
geometry_msgs/PoseStamped) and feeds it to that drone's PX4 EKF2 as external
vision, so PX4 can hold position indoors with no GPS/VIO. Two output modes:

  px4_vio_mode = 'direct'  (default, mirrors the proven model_ai_tfpub.cpp):
      publish px4_msgs/VehicleOdometry straight to
      /{name}/fmu/in/vehicle_visual_odometry. Sets timestamp=0 (the uXRCE-DDS
      client restamps with PX4's HRT on receive — a ground-clock timestamp
      gets rejected by EKF2), quality=100, velocity=NaN (fuse POSE ONLY).

  px4_vio_mode = 'via_interface':
      publish nav_msgs/Odometry to /{name}/fmu/visual_odometry_in and let
      px4_interface do the ENU->NED conversion + republish. Use only if you
      are NOT running the direct path.

Frame conversion (px4_vio_frame):
  'enu_to_ned'   — input is ROS-standard ENU world / FLU body (the principled
                   default). N=ENU_y, E=ENU_x, D=-ENU_z; quaternion via the
                   standard FLU/ENU -> FRD/NED rotation.
  'modalai_flip' — the reference's transform: 180deg flip about X on both the
                   world and the body (diag(1,-1,-1)), POSE_FRAME_FRD. Use
                   this if your mocap is NOT ENU (e.g. natnet republishes
                   Motive's raw Y-up frame). VERIFY with the hand-check in
                   experiment.md before flying — a wrong frame flies the drone
                   into a wall.

Hardware only: in simulation PX4 SITL estimates its own state.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry

MOCAP_VARIANCE = 1e-4
SQRT2_INV = 0.70710678118654752
NAN = float('nan')


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


class DroneStream:
    def __init__(self):
        self.publisher = None
        self.prev_position = None
        self.prev_stamp_s = None
        self.velocity = np.zeros(3)


class MocapBridge(Node):

    def __init__(self):
        super().__init__('mocap_bridge')

        self.declare_parameter('drone_names', ['drone_1', 'drone_2', 'drone_3'])
        self.declare_parameter('mocap_topic_template', '/{name}/pose')
        # 'direct' (px4_msgs/VehicleOdometry -> /fmu/in/vehicle_visual_odometry)
        # or 'via_interface' (nav_msgs/Odometry -> px4_interface).
        self.declare_parameter('px4_vio_mode', 'direct')
        # 'enu_to_ned' (standard) or 'modalai_flip' (the reference transform).
        self.declare_parameter('px4_vio_frame', 'enu_to_ned')
        self.declare_parameter('px4_vio_topic_template',
                               '/{name}/fmu/in/vehicle_visual_odometry')
        self.declare_parameter('visual_odometry_topic_template',
                               '/{name}/fmu/visual_odometry_in')
        self.declare_parameter('mocap_frame', 'map')
        self.declare_parameter('vio_quality', 100)
        # EMA weight for the finite-difference velocity (via_interface only).
        self.declare_parameter('velocity_filter_alpha', 0.4)
        self.declare_parameter('mocap_qos_best_effort', False)

        names = list(self.get_parameter('drone_names').value)
        mocap_tmpl = str(self.get_parameter('mocap_topic_template').value)
        self.mode = str(self.get_parameter('px4_vio_mode').value)
        self.vio_frame = str(self.get_parameter('px4_vio_frame').value)
        self.frame = str(self.get_parameter('mocap_frame').value)
        self.quality = int(self.get_parameter('vio_quality').value)
        self.velocity_alpha = float(self.get_parameter('velocity_filter_alpha').value)
        if self.mode not in ('direct', 'via_interface'):
            raise ValueError(f'px4_vio_mode must be direct|via_interface, got {self.mode}')
        if self.vio_frame not in ('enu_to_ned', 'modalai_flip'):
            raise ValueError(
                f'px4_vio_frame must be enu_to_ned|modalai_flip, got {self.vio_frame}')

        # Subscriber QoS for the mocap input.
        sub_qos = QoSProfile(depth=10)
        if bool(self.get_parameter('mocap_qos_best_effort').value):
            sub_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        # PX4 /fmu/in/* expect BEST_EFFORT + VOLATILE (match the client).
        px4_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST)

        direct_tmpl = str(self.get_parameter('px4_vio_topic_template').value)
        iface_tmpl = str(self.get_parameter('visual_odometry_topic_template').value)

        out_tmpl = direct_tmpl if self.mode == 'direct' else iface_tmpl
        self.streams = {}
        for name in names:
            stream = DroneStream()
            if self.mode == 'direct':
                stream.publisher = self.create_publisher(
                    VehicleOdometry, out_tmpl.format(name=name), px4_qos)
            else:
                stream.publisher = self.create_publisher(
                    Odometry, out_tmpl.format(name=name), 10)
            self.create_subscription(
                PoseStamped, mocap_tmpl.format(name=name),
                lambda msg, s=stream, n=name: self.pose_callback(s, n, msg), sub_qos)
            self.streams[name] = stream

        self.get_logger().info(
            f'MocapBridge [{self.mode}/{self.vio_frame}] forwarding {len(names)} '
            f'drone(s): ' + ', '.join(
                f'{mocap_tmpl.format(name=n)} -> {out_tmpl.format(name=n)}'
                for n in names))

    def _convert(self, position, q_wxyz):
        """Return (px4_position[3], px4_q[4], pose_frame) for the chosen frame."""
        if self.vio_frame == 'modalai_flip':
            # 180deg flip about X on world and body (reference model_ai_tfpub):
            # p_frd = diag(1,-1,-1) * p ; q likewise pre/post multiplied.
            p = [position[0], -position[1], -position[2]]
            flip = [0.0, 1.0, 0.0, 0.0]   # 180deg about X, [w,x,y,z]
            q = qmul(qmul(flip, list(q_wxyz)), flip)
            return p, q, VehicleOdometry.POSE_FRAME_FRD
        # standard ENU/FLU -> NED/FRD
        p = [position[1], position[0], -position[2]]
        q = enu_flu_to_ned_frd(*q_wxyz)
        return p, q, VehicleOdometry.POSE_FRAME_NED

    def pose_callback(self, stream: DroneStream, name: str, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        position = np.array([p.x, p.y, p.z])
        q_wxyz = (o.w, o.x, o.y, o.z)

        if self.mode == 'direct':
            px4_p, px4_q, pose_frame = self._convert(position, q_wxyz)
            vio = VehicleOdometry()
            vio.timestamp = 0           # uXRCE client restamps with PX4 HRT
            vio.timestamp_sample = 0
            vio.pose_frame = pose_frame
            vio.position = [float(px4_p[0]), float(px4_p[1]), float(px4_p[2])]
            vio.q = [float(px4_q[0]), float(px4_q[1]), float(px4_q[2]), float(px4_q[3])]
            vio.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
            vio.velocity = [NAN, NAN, NAN]          # fuse POSE only
            vio.angular_velocity = [NAN, NAN, NAN]
            vio.position_variance = [MOCAP_VARIANCE] * 3
            vio.orientation_variance = [MOCAP_VARIANCE] * 3
            vio.velocity_variance = [NAN, NAN, NAN]
            vio.reset_counter = 0
            vio.quality = self.quality
            stream.publisher.publish(vio)
            return

        # via_interface: nav_msgs/Odometry (ENU) -> px4_interface converts.
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stream.prev_position is not None:
            dt = stamp_s - stream.prev_stamp_s
            if 1e-6 < dt < 0.5:
                raw = (position - stream.prev_position) / dt
                a = self.velocity_alpha
                stream.velocity = a * raw + (1.0 - a) * stream.velocity
        stream.prev_position = position
        stream.prev_stamp_s = stamp_s

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.frame
        odom.child_frame_id = f'{name}/base_link'
        odom.pose.pose = msg.pose
        for axis in range(6):
            odom.pose.covariance[7 * axis] = MOCAP_VARIANCE
        odom.twist.twist.linear.x = float(stream.velocity[0])
        odom.twist.twist.linear.y = float(stream.velocity[1])
        odom.twist.twist.linear.z = float(stream.velocity[2])
        odom.twist.covariance[0] = -1.0
        stream.publisher.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = MocapBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
