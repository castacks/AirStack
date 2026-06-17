#!/usr/bin/env python3

"""
MAVROS GPS Origin Node

Publishes a synthetic GPS origin to MAVROS once at startup for mocap / no-GNSS
flight. With GNSS disabled, PX4 fuses vision into a valid local position but
has no global position, so modes that require one (e.g. AUTO.LOITER) refuse to
arm. Setting an origin lets PX4 derive global position from the fused estimate.

The publish is guarded: it waits for MAVROS to connect, watches for an existing
origin, and only publishes if none is present — GNSS-equipped vehicles are left
untouched.
"""

import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import State


class MavrosGpOriginNode(Node):
    """One-shot synthetic GPS origin publisher for MAVROS / PX4."""

    def __init__(self):
        super().__init__('mavros_gp_origin')

        self.declare_parameter('enabled', True)
        # Defaults match the AirStack shared world datum (Lisbon) used by the GCS
        # (gcs_utils.py) and sim (gps_utils.py). Normally overridden by
        # config/mavros_gp_origin.yaml; kept in sync to avoid a stale fallback.
        self.declare_parameter('latitude', 38.736832)
        self.declare_parameter('longitude', -9.137977)
        self.declare_parameter('altitude', 90.0)
        # Seconds to wait after MAVROS connects (listening for an existing
        # origin) before publishing our synthetic one.
        self.declare_parameter('settle_sec', 5.0)

        self._enabled = self.get_parameter('enabled').value
        if not self._enabled:
            self.get_logger().info('Synthetic GPS origin disabled (enabled=false).')
            return

        self._lat = self.get_parameter('latitude').value
        self._lon = self.get_parameter('longitude').value
        self._alt = self.get_parameter('altitude').value
        self._settle_sec = self.get_parameter('settle_sec').value

        self._done = False
        self._origin_exists = False
        self._connected_since = None
        self._publish_count = 0

        self._set_origin_pub = self.create_publisher(
            GeoPointStamped, 'set_gps_origin', 10
        )
        self._origin_sub = self.create_subscription(
            GeoPointStamped, 'current_gps_origin', self._on_existing_origin, 10
        )
        self._state_sub = self.create_subscription(
            State, 'mavros_state', self._on_mavros_state, 10
        )
        self._timer = self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f'MAVROS GPS origin node started '
            f'(lat={self._lat}, lon={self._lon}, alt={self._alt}, '
            f'settle_sec={self._settle_sec})'
        )

    def _on_existing_origin(self, _msg: GeoPointStamped):
        """An origin already exists (e.g. from GNSS) — never override it."""
        if not self._origin_exists and not self._done:
            self.get_logger().info(
                'Existing GPS origin detected; skipping synthetic origin.'
            )
        self._origin_exists = True

    def _on_mavros_state(self, msg: State):
        if msg.connected and self._connected_since is None:
            self._connected_since = self.get_clock().now()

    def _tick(self):
        if self._done:
            return
        if self._origin_exists:
            self._done = True
            self._timer.cancel()
            return
        if self._connected_since is None:
            return
        elapsed = (self.get_clock().now() - self._connected_since).nanoseconds * 1e-9
        if elapsed < self._settle_sec:
            return

        msg = GeoPointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position.latitude = self._lat
        msg.position.longitude = self._lon
        msg.position.altitude = self._alt
        self._set_origin_pub.publish(msg)
        self._publish_count += 1
        self.get_logger().info(
            f'Published synthetic GPS origin '
            f'(lat={self._lat}, lon={self._lon}, alt={self._alt}) '
            f'[{self._publish_count}/3]'
        )
        # Publish a few times in case MAVROS subscribed late, then stop.
        if self._publish_count >= 3:
            self._done = True
            self._timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MavrosGpOriginNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
