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

try:
    from numpy.lib.recfunctions import structured_to_unstructured
except ImportError:  # pragma: no cover
    from sensor_msgs_py.numpy_compat import structured_to_unstructured


def _read_pointcloud_xyz_nx3(msg: PointCloud2) -> np.ndarray:
    """Return ``(N, 3)`` float32 xyz rows; ``N == 0`` if empty.

    ``read_points`` may return a structured ``ndarray`` (buffer-backed) or a
    Python iterator depending on ``sensor_msgs_py`` / distro. Prefer
    ``read_points_numpy`` when present and compatible; otherwise normalize to a
    dense ``(N, 3)`` array before filtering.
    """
    read_numpy = getattr(point_cloud2, 'read_points_numpy', None)
    if read_numpy is not None:
        try:
            arr = read_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        except (AssertionError, TypeError, ValueError):
            arr = None
        if arr is not None:
            arr = np.asarray(arr, dtype=np.float32, order='C')
            if arr.size == 0:
                return np.zeros((0, 3), dtype=np.float32)
            if arr.ndim != 2 or arr.shape[1] != 3:
                arr = arr.reshape(-1, 3)
            return arr

    pts = point_cloud2.read_points(
        msg, field_names=('x', 'y', 'z'), skip_nans=True
    )
    if isinstance(pts, np.ndarray):
        if pts.size == 0:
            return np.zeros((0, 3), dtype=np.float32)
        return structured_to_unstructured(pts).astype(np.float32, copy=False)

    rows = list(pts)
    if not rows:
        return np.zeros((0, 3), dtype=np.float32)
    return np.asarray(rows, dtype=np.float32)


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

        arr = _read_pointcloud_xyz_nx3(msg)
        if arr.shape[0] == 0:
            self._pub.publish(point_cloud2.create_cloud(msg.header, self._fields, []))
            return

        finite = np.isfinite(arr).all(axis=1)
        r2 = np.sum(arr.astype(np.float64, copy=False) ** 2, axis=1)
        if self._near_range_m <= 0.0:
            near_ok = np.ones(arr.shape[0], dtype=bool)
        else:
            near_ok = r2 >= (self._near_range_m ** 2)
        valid = finite & near_ok

        if not np.any(valid):
            self._pub.publish(point_cloud2.create_cloud(msg.header, self._fields, []))
            return

        out = arr[valid]
        if out.dtype != np.float32:
            out = out.astype(np.float32, copy=False)
        self._pub.publish(point_cloud2.create_cloud(msg.header, self._fields, out))

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
