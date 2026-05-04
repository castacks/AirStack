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


def _lidar_qos(depth: int) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
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
            f'/{_robot}/sensors/lidar/point_cloud_raw',
        )
        self.declare_parameter(
            'output_topic',
            f'/{_robot}/sensors/lidar/point_cloud',
        )
        self.declare_parameter('qos_depth', 10)

        self._near_range_m = float(self.get_parameter('near_range_m').value)
        self._input_topic = str(self.get_parameter('input_topic').value)
        self._output_topic = str(self.get_parameter('output_topic').value)
        qos_depth = int(self.get_parameter('qos_depth').value)
        self._qos = _lidar_qos(max(1, qos_depth))

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
            f'near_range_m={self._near_range_m}'
        )

    def _cloud_callback(self, msg: PointCloud2) -> None:
        if not self._has_xyz_fields(msg):
            if not self._warned_missing_xyz:
                self.get_logger().error(
                    'PointCloud2 missing x, y, or z fields; not publishing.'
                )
                self._warned_missing_xyz = True
            return

        names = [f.name for f in msg.fields]

        dtype_list = [('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32)]
        data = np.frombuffer(msg.data, dtype=np.dtype(dtype_list))

        mask = np.isfinite(data['x']) & np.isfinite(data['y']) & np.isfinite(data['z'])

        clean_data = data[mask]

        x = clean_data['x']
        y = clean_data['y']
        z = clean_data['z']
        r2 = x * x + y * y + z * z
        near2 = self._near_range_m * self._near_range_m
        mask = r2 >= near2

        sel = clean_data[mask]
        out = point_cloud2.create_cloud(msg.header, self._fields, sel)
        self._pub.publish(out)

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

