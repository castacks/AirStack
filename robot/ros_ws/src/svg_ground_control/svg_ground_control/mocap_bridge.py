"""Multi-drone mocap-to-PX4 bridge.

Subscribes to one mocap pose topic per drone (default ``/{name}/pose``,
geometry_msgs/PoseStamped, world-frame ENU/FLU) and republishes each as
nav_msgs/Odometry into that drone's px4_interface external-vision input
(default ``/{name}/fmu/visual_odometry_in``). px4_interface converts
ENU -> NED/FRD and forwards to /fmu/in/vehicle_visual_odometry, so this node
performs NO frame gymnastics — it only restamps, attaches a velocity estimate
(filtered finite difference, useful for analysis/visualization), and fans out
over N drones.

Hardware only: in simulation PX4 SITL estimates its own state and this node
is not launched.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

MOCAP_POSITION_VARIANCE = 1e-4
MOCAP_ORIENTATION_VARIANCE = 1e-4


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
        self.declare_parameter('visual_odometry_topic_template',
                               '/{name}/fmu/visual_odometry_in')
        self.declare_parameter('mocap_frame', 'map')
        # EMA weight for the finite-difference velocity estimate (1.0 = raw).
        self.declare_parameter('velocity_filter_alpha', 0.4)
        self.declare_parameter('mocap_qos_best_effort', False)

        names = list(self.get_parameter('drone_names').value)
        mocap_tmpl = str(self.get_parameter('mocap_topic_template').value)
        out_tmpl = str(self.get_parameter('visual_odometry_topic_template').value)
        self.frame = str(self.get_parameter('mocap_frame').value)
        self.velocity_alpha = float(self.get_parameter('velocity_filter_alpha').value)

        qos = QoSProfile(depth=10)
        if bool(self.get_parameter('mocap_qos_best_effort').value):
            qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.streams = {}
        for name in names:
            stream = DroneStream()
            stream.publisher = self.create_publisher(
                Odometry, out_tmpl.format(name=name), 10)
            self.create_subscription(
                PoseStamped, mocap_tmpl.format(name=name),
                lambda msg, s=stream, n=name: self.pose_callback(s, n, msg), qos)
            self.streams[name] = stream

        self.get_logger().info(
            f'MocapBridge forwarding {len(names)} drone(s): '
            + ', '.join(f'{mocap_tmpl.format(name=n)} -> {out_tmpl.format(name=n)}'
                        for n in names))

    def pose_callback(self, stream: DroneStream, name: str, msg: PoseStamped):
        p = msg.pose.position
        position = np.array([p.x, p.y, p.z])
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
            odom.pose.covariance[7 * axis] = (
                MOCAP_POSITION_VARIANCE if axis < 3 else MOCAP_ORIENTATION_VARIANCE)
        odom.twist.twist.linear.x = float(stream.velocity[0])
        odom.twist.twist.linear.y = float(stream.velocity[1])
        odom.twist.twist.linear.z = float(stream.velocity[2])
        # Mark twist covariance unknown: PX4 should fuse pose only.
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
