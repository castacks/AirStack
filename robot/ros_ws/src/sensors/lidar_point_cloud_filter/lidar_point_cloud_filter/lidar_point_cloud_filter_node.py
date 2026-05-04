# Copyright 2026 AirLab CMU
# SPDX-License-Identifier: Apache-2.0

"""ROS 2 node: near-range sphere filter for sensor_msgs/PointCloud2."""

import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


def _point_cloud_qos(depth: int, reliable: bool) -> QoSProfile:
    """QoS aligned with common bridges (e.g. Isaac Replicator: RELIABLE) and RViz."""
    return QoSProfile(
        depth=max(1, depth),
        reliability=(
            QoSReliabilityPolicy.RELIABLE
            if reliable
            else QoSReliabilityPolicy.BEST_EFFORT
        ),
        history=QoSHistoryPolicy.KEEP_LAST,
    )


class LidarPointCloudFilterNode(Node):
    """Drop near-range points (sensor-frame radius) and republish as xyz float32."""

    def __init__(self) -> None:
        super().__init__('lidar_point_cloud_filter')

        _robot = os.environ.get('ROBOT_NAME', 'robot')
        self.declare_parameter('near_range_m', 0.75)
        self.declare_parameter(
            'input_topic',
            f'/{_robot}/sensors/ouster/point_cloud_raw',
        )
        self.declare_parameter(
            'output_topic',
            f'/{_robot}/sensors/ouster/point_cloud',
        )
        self.declare_parameter('qos_depth', 10)
        # Match RELIABLE publishers (e.g. Isaac Sim ROS2 bridge); set false for best-effort lidar.
        self.declare_parameter('qos_reliable', True)

        self._near_range_m = float(self.get_parameter('near_range_m').value)
        self._input_topic = str(self.get_parameter('input_topic').value)
        self._output_topic = str(self.get_parameter('output_topic').value)
        qos_depth = int(self.get_parameter('qos_depth').value)
        qos_reliable = bool(self.get_parameter('qos_reliable').value)
        self._qos = _point_cloud_qos(qos_depth, qos_reliable)

        self._fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        self._pub = self.create_publisher(
            PointCloud2, self._output_topic, self._qos
        )
        self._sub = self.create_subscription(
            PointCloud2, self._input_topic, self._cloud_callback, self._qos
        )

        self._warned_missing_xyz = False

        self.get_logger().info(
            f'Filtering lidar: in={self._input_topic} out={self._output_topic} '
            f'near_range_m={self._near_range_m} qos_reliable={qos_reliable}'
        )

    def _cloud_callback(self, msg: PointCloud2) -> None:
        if not self._has_xyz_fields(msg):
            if not self._warned_missing_xyz:
                self.get_logger().error(
                    'PointCloud2 missing x, y, or z fields; not publishing.'
                )
                self._warned_missing_xyz = True
            return

        pts = list(
            point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True
            )
        )
        if not pts:
            self._pub.publish(point_cloud2.create_cloud(msg.header, self._fields, []))
            return

        mat = np.array(
            [(float(p[0]), float(p[1]), float(p[2])) for p in pts],
            dtype=np.float64,
        )
        r2 = mat[:, 0] ** 2 + mat[:, 1] ** 2 + mat[:, 2] ** 2
        if self._near_range_m <= 0.0:
            mask = np.ones(mat.shape[0], dtype=bool)
        else:
            mask = r2 >= (self._near_range_m ** 2)

        if not np.any(mask):
            self._pub.publish(point_cloud2.create_cloud(msg.header, self._fields, []))
            return

        sel = mat[mask]
        out_rows = [(float(r[0]), float(r[1]), float(r[2])) for r in sel]
        self._pub.publish(
            point_cloud2.create_cloud(msg.header, self._fields, out_rows)
        )

    @staticmethod
    def _has_xyz_fields(msg: PointCloud2) -> bool:
        names = {f.name for f in msg.fields}
        return {'x', 'y', 'z'}.issubset(names)


def main(args=None) -> None:
    """Run the lidar point cloud filter node."""
    rclpy.init(args=args)
    node = LidarPointCloudFilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
