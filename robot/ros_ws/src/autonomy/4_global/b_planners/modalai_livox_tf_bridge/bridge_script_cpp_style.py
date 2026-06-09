#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def transform_from_rt(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def quaternion_to_rotation_matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ])


def rotation_matrix_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    trace = np.trace(rotation)

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / s
        x = 0.25 * s
        y = (rotation[0, 1] + rotation[1, 0]) / s
        z = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / s
        x = (rotation[0, 1] + rotation[1, 0]) / s
        y = 0.25 * s
        z = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / s
        x = (rotation[0, 2] + rotation[2, 0]) / s
        y = (rotation[1, 2] + rotation[2, 1]) / s
        z = 0.25 * s

    quat = np.array([w, x, y, z])
    quat /= np.linalg.norm(quat)
    return quat


class CppStyleOdometryBridge(Node):
    def __init__(self):
        super().__init__('odometry_bridge_cpp_style')

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(
            Odometry,
            '/laser_odometry',
            self.odom_callback,
            qos,
        )

        self.pub_visual = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            qos,
        )

        # Same transforms as modalai_tfpub_lidar.cpp
        rot_1 = np.array([
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ])
        rot_2 = np.array([
            [0.8703557, 0.0, -0.4924236],
            [0.0, 1.0, 0.0],
            [0.4924236, 0.0, 0.8703557],
        ])
        trans_3 = np.array([-0.01315, 0.01100, -0.01047])
        self.t_superodom_lidarodom_to_imu_frd = transform_from_rt(
            rot_1 @ rot_2,
            rot_1 @ rot_2 @ trans_3,
        )

        rot_4 = np.array([
            [0.8703557, 0.0, 0.4924236],
            [0.0, 1.0, 0.0],
            [-0.4924236, 0.0, 0.8703557],
        ])
        rot_5 = np.array([
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ])
        self.t_px4_local_to_superodom_lidar_mapinit = transform_from_rt(rot_4 @ rot_5, np.zeros(3))

    def odom_callback(self, msg: Odometry):
        msg_rotation = quaternion_to_rotation_matrix(
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        )
        msg_translation = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])
        t_superodom_lidar_mapinit_to_superodom_lidarodom = transform_from_rt(
            msg_rotation,
            msg_translation,
        )

        t_px4_local_to_imu_frd = (
            self.t_px4_local_to_superodom_lidar_mapinit
            @ t_superodom_lidar_mapinit_to_superodom_lidarodom
            @ self.t_superodom_lidarodom_to_imu_frd
        )

        vel_lidar_body = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ])
        angular_vel_lidar = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ])

        rot_imu_from_lidar = self.t_superodom_lidarodom_to_imu_frd[:3, :3].T
        p_imu_in_lidar = self.t_superodom_lidarodom_to_imu_frd[:3, 3]
        vel_imu_origin_in_lidar = vel_lidar_body + np.cross(angular_vel_lidar, p_imu_in_lidar)
        vel_px4_frd = rot_imu_from_lidar @ vel_imu_origin_in_lidar
        angular_vel_frd = rot_imu_from_lidar @ angular_vel_lidar

        pose_quat = rotation_matrix_to_quaternion(t_px4_local_to_imu_frd[:3, :3])

        vo = VehicleOdometry()
        now_us = int(self.get_clock().now().nanoseconds / 1000)
        vo.timestamp = now_us
        vo.timestamp_sample = now_us

        vo.pose_frame = VehicleOdometry.POSE_FRAME_FRD
        vo.position[0] = float(t_px4_local_to_imu_frd[0, 3])
        vo.position[1] = float(t_px4_local_to_imu_frd[1, 3])
        vo.position[2] = float(t_px4_local_to_imu_frd[2, 3])
        vo.q[0] = float(pose_quat[0])
        vo.q[1] = float(pose_quat[1])
        vo.q[2] = float(pose_quat[2])
        vo.q[3] = float(pose_quat[3])

        vo.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        vo.velocity[0] = math.nan
        vo.velocity[1] = math.nan
        vo.velocity[2] = math.nan
        vo.angular_velocity[0] = math.nan
        vo.angular_velocity[1] = math.nan
        vo.angular_velocity[2] = math.nan

        vo.position_variance = [0.01, 0.01, 0.01]
        vo.orientation_variance = [0.0001, 0.0001, 0.0001]
        vo.velocity_variance = [math.nan, math.nan, math.nan]
        vo.reset_counter = 0
        vo.quality = 100

        self.pub_visual.publish(vo)


def main(args=None):
    rclpy.init(args=args)
    node = CppStyleOdometryBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
