#!/usr/bin/env python3
# Simulation-only bridge: republishes PX4's VehicleOdometry (NED/FRD) as a
# geometry_msgs/PoseStamped on the "qvio" topic so that modalai_interface can
# consume it identically to how the real VOXL2 publishes qvio.
#
# On real hardware this node is NOT needed — voxl-mpa-to-ros2 publishes /qvio
# directly.  Only run this in simulation.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import PoseStamped


class SimQvioBridge(Node):
    def __init__(self):
        super().__init__('sim_qvio_bridge')

        px4_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._sub = self.create_subscription(
            VehicleOdometry,
            'out/vehicle_odometry',
            self._on_odom,
            px4_qos,
        )

        self._pub = self.create_publisher(PoseStamped, 'qvio', 10)
        self.get_logger().info('sim_qvio_bridge started')

    def _on_odom(self, msg: VehicleOdometry):
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'map'

        # Forward NED position and FRD quaternion as-is — modalai_interface
        # expects qvio in NED/FRD (same as real VOXL2) and does the conversion.
        ps.pose.position.x = float(msg.position[0])
        ps.pose.position.y = float(msg.position[1])
        ps.pose.position.z = float(msg.position[2])

        ps.pose.orientation.w = float(msg.q[0])
        ps.pose.orientation.x = float(msg.q[1])
        ps.pose.orientation.y = float(msg.q[2])
        ps.pose.orientation.z = float(msg.q[3])

        self._pub.publish(ps)


def main():
    rclpy.init()
    node = SimQvioBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
