#!/usr/bin/env python3

"""
NatNet ROS 2 Node

Receives motion capture data from OptiTrack Motive via NatNet UDP protocol
and publishes poses to the AirStack perception layer.

Uses pure Python parsing (no SDK required).

Data flow:
1. Listen to NatNet UDP packets from external Motive PC
2. Decode rigid body poses and positions via NatNet protocol
3. Publish PoseStamped to /{robot_name}/perception/optitrack/{body_name}
4. Publish PoseWithCovarianceStamped to /{robot_name}/perception/optitrack/pose_cov
5. Optionally bridge to MAVROS for PX4 fusion (via vision_pose_converter_node)
"""

import os
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

# natnet_client.py lives in the same installed directory
sys.path.insert(0, os.path.dirname(__file__))
from natnet_client import NatNetClient, FrameData  # noqa: E402


class NatNetROS2Node(Node):
    """ROS 2 node for NatNet motion capture integration."""

    def __init__(self):
        super().__init__('natnet_ros2_node')

        self.get_logger().info("########################################################")
        self.get_logger().info("Began NatNet ROS 2 Node")
        self.get_logger().info("########################################################")
        
        # Declare parameters
        self.declare_parameter('server_ip', '192.168.1.1')
        self.declare_parameter('command_port', 1510)
        self.declare_parameter('data_port', 1511)
        # Local NIC for UDP bind (NatNet SDK localAddress). 0.0.0.0 = OS chooses.
        self.declare_parameter('client_ip', '0.0.0.0')
        self.declare_parameter('body_name', 'robot_1')
        # body_id: rigid body ID from Motive to track; -1 = track all bodies
        self.declare_parameter('body_id', -1)
        self.declare_parameter('publish_direct_optitrack', True)
        self.declare_parameter('publish_to_mavros', False)
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('debug', False)
        self.declare_parameter('negotiation_enabled', True)
        # Covariance: 3x3 flattened (9 values each)
        self.declare_parameter(
            'position_covariance',
            [0.1, 0.0, 0.0,
             0.0, 0.1, 0.0,
             0.0, 0.0, 0.1]
        )
        self.declare_parameter(
            'orientation_covariance',
            [0.01, 0.0, 0.0,
             0.0, 0.01, 0.0,
             0.0, 0.0, 0.01]
        )

        # Get parameters
        server_ip = self.get_parameter('server_ip').value
        command_port = self.get_parameter('command_port').value
        data_port = self.get_parameter('data_port').value
        client_ip = self.get_parameter('client_ip').value
        self.body_name = self.get_parameter('body_name').value
        self.body_id = self.get_parameter('body_id').value
        self.publish_direct = self.get_parameter('publish_direct_optitrack').value
        self.publish_mavros = self.get_parameter('publish_to_mavros').value
        self.frame_id = self.get_parameter('frame_id').value
        self.debug = self.get_parameter('debug').value
        negotiation_enabled = self.get_parameter('negotiation_enabled').value

        # Build 6x6 covariance matrix from 3x3 position and orientation blocks
        pos_cov = self.get_parameter('position_covariance').value
        ori_cov = self.get_parameter('orientation_covariance').value
        self.covariance_6x6 = self._build_6x6_covariance(pos_cov, ori_cov)

        # Get robot name from environment
        self.robot_name = os.environ.get('ROBOT_NAME', 'robot_1')

        # Publisher dictionaries (created on first publish per topic)
        self.pose_publishers: dict = {}
        self.pose_cov_publishers: dict = {}

        self.get_logger().info("########################################################")
        self.get_logger().info("NatNet ROS 2 Node Parameters:")
        self.get_logger().info("########################################################")
        self.get_logger().info(f'Server IP: {server_ip}')
        self.get_logger().info(f'Client IP: {client_ip}')
        self.get_logger().info(f'Command port: {command_port}')
        self.get_logger().info(f'Data port: {data_port}')
        self.get_logger().info(f'Body name: {self.body_name}')
        self.get_logger().info(f'Body id: {self.body_id}')
        self.get_logger().info(f'Publish direct: {self.publish_direct}')
        self.get_logger().info(f'Publish to mavros: {self.publish_mavros}')
        self.get_logger().info(f'Frame id: {self.frame_id}')
        self.get_logger().info(f'Debug: {self.debug}')
        self.get_logger().info(f'Negotiation enabled: {negotiation_enabled}')

        # Initialize NatNet client
        self.natnet_client = NatNetClient(
            server_ip,
            data_port,
            command_port=command_port,
            local_ip=client_ip,
            negotiation_enabled=negotiation_enabled,
        )
        self.natnet_client.set_frame_callback(self._on_frame_received)

        if not self.natnet_client.start():
            self.get_logger().error(
                f'Failed to bind UDP socket on port {data_port}'
            )
            raise RuntimeError('Failed to initialize NatNet UDP listener')

        self.get_logger().info(
            f'NatNet node started (listening on UDP port {data_port})'
        )
        self.get_logger().info(f'Robot name: {self.robot_name}')
        if self.body_id >= 0:
            self.get_logger().info(
                f'Tracking body ID {self.body_id} as "{self.body_name}"'
            )
        else:
            self.get_logger().info('Tracking all bodies (body_id = -1)')
            self.get_logger().info("########################################################")

    def _build_6x6_covariance(
        self, pos_cov_3x3: list, ori_cov_3x3: list
    ) -> list:
        """Build a 6x6 covariance matrix (row-major) from two 3x3 blocks."""
        c = [0.0] * 36
        # Position block (rows 0-2, cols 0-2)
        for r in range(3):
            for col in range(3):
                c[r * 6 + col] = float(pos_cov_3x3[r * 3 + col])
        # Orientation block (rows 3-5, cols 3-5)
        for r in range(3):
            for col in range(3):
                c[(r + 3) * 6 + (col + 3)] = float(ori_cov_3x3[r * 3 + col])
        return c

    def destroy_node(self):
        """Clean up resources on shutdown."""
        if self.natnet_client:
            self.natnet_client.stop()
        super().destroy_node()

    def _on_frame_received(self, frame: FrameData):
        """Callback invoked when a complete NatNet frame is received."""
        try:
            if self.debug:
                self.get_logger().info(
                    f'Frame {frame.frame_number}: '
                    f'{len(frame.rigid_bodies)} rigid bodies'
                )

            for rb_id, rb_data in frame.rigid_bodies.items():
                if not rb_data.tracking_valid:
                    continue

                # Filter by configured body_id when set
                if self.body_id >= 0 and rb_id != self.body_id:
                    continue

                body_name = self._get_body_name(rb_id)

                if self.publish_direct:
                    self._publish_pose(body_name, rb_data)

                # Always publish pose_cov as the primary pose output
                self._publish_pose_cov(body_name, rb_data)

        except Exception as e:
            self.get_logger().error(f'Error processing frame: {e}')

    def _publish_pose(self, body_name: str, rb_data) -> None:
        """Publish direct optitrack pose as PoseStamped."""
        topic = f'/{self.robot_name}/perception/optitrack/{body_name}'
        if topic not in self.pose_publishers:
            self.pose_publishers[topic] = self.create_publisher(
                PoseStamped, topic, 10
            )

        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(rb_data.position[0])
        msg.pose.position.y = float(rb_data.position[1])
        msg.pose.position.z = float(rb_data.position[2])
        msg.pose.orientation.x = float(rb_data.rotation[0])
        msg.pose.orientation.y = float(rb_data.rotation[1])
        msg.pose.orientation.z = float(rb_data.rotation[2])
        msg.pose.orientation.w = float(rb_data.rotation[3])

        self.pose_publishers[topic].publish(msg)

    def _publish_pose_cov(self, body_name: str, rb_data) -> None:
        """Publish PoseWithCovarianceStamped for this body."""
        topic = f'/{self.robot_name}/perception/optitrack/{body_name}/pose_cov'
        if topic not in self.pose_cov_publishers:
            self.pose_cov_publishers[topic] = self.create_publisher(
                PoseWithCovarianceStamped, topic, 10
            )

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(rb_data.position[0])
        msg.pose.pose.position.y = float(rb_data.position[1])
        msg.pose.pose.position.z = float(rb_data.position[2])
        msg.pose.pose.orientation.x = float(rb_data.rotation[0])
        msg.pose.pose.orientation.y = float(rb_data.rotation[1])
        msg.pose.pose.orientation.z = float(rb_data.rotation[2])
        msg.pose.pose.orientation.w = float(rb_data.rotation[3])
        msg.pose.covariance = self.covariance_6x6

        self.pose_cov_publishers[topic].publish(msg)

    def _get_body_name(self, body_id: int) -> str:
        """
        Map a rigid body ID to a topic-friendly name.

        When tracking a specific body (body_id param >= 0), the configured
        body_name is used.  For all-body mode, falls back to body_{id}.
        """
        if self.body_id >= 0 and body_id == self.body_id:
            return self.body_name
        return f'body_{body_id}'


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = NatNetROS2Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
