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

import shutil
import subprocess

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
        # Real-hardware geoid handling. mavros/PX4 treat the origin altitude as a
        # WGS-84 ELLIPSOIDAL height and internally apply the egm96-5 geoid model
        # (mavros_uas::egm96_5) to convert to/from AMSL. With no GNSS/baro the
        # vehicle height comes purely from vision (mocap floor ~ 0 AMSL), so to
        # make local_position z equal the OptiTrack height the origin's ellipsoidal
        # altitude must be:  N(lat,lon) + desired_floor_amsl,  where N is the geoid
        # undulation. Because the SAME egm96-5 model computes N here and inside
        # mavros, the undulation cancels EXACTLY (accuracy is independent of the
        # model's absolute error). Skipped when use_sim_time=true: sim's synthetic
        # GPS carries no geoid separation and uses the literal altitude.
        self.declare_parameter('use_geoid_altitude', False)
        # AMSL (m) assigned to the mocap floor / vision z = 0. Local z equals OptiTrack z
        # for any value; this only sets what global altitude the floor reports.
        self.declare_parameter('desired_floor_amsl', 36.0)
        # Geoid model — MUST match mavros (egm96-5) for exact cancellation.
        self.declare_parameter('geoid_model', 'egm96-5')
        # Seconds to wait after MAVROS connects (listening for an existing
        # origin) before publishing our synthetic one.
        self.declare_parameter('settle_sec', 5.0)

        self._enabled = self.get_parameter('enabled').value
        if not self._enabled:
            self.get_logger().info('Synthetic GPS origin disabled (enabled=false).')
            return

        self._lat = self.get_parameter('latitude').value
        self._lon = self.get_parameter('longitude').value
        self._settle_sec = self.get_parameter('settle_sec').value
        self._alt = self._resolve_altitude()

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

    def _geoid_undulation(self, lat, lon, model):
        """
        Geoid undulation N (metres, height of the geoid above the WGS-84
        ellipsoid) at (lat, lon) via GeographicLib's GeoidEval — the same
        egm96-5 dataset mavros loads (mavros_uas::egm96_5), so N cancels exactly
        against mavros' internal ellipsoid<->AMSL conversion. Raises on failure.
        """
        exe = shutil.which('GeoidEval')
        if exe is None:
            raise RuntimeError('GeoidEval not found on PATH (install GeographicLib tools)')
        proc = subprocess.run(
            [exe, '-n', model],
            input=f'{lat:.9f} {lon:.9f}\n',
            capture_output=True, text=True, timeout=10.0,
        )
        if proc.returncode != 0:
            raise RuntimeError(
                f'GeoidEval rc={proc.returncode}: {proc.stderr.strip() or proc.stdout.strip()}'
            )
        return float(proc.stdout.strip().split()[0])

    def _resolve_altitude(self):
        """
        Origin altitude to publish: the literal `altitude` param, unless
        use_geoid_altitude is set on real hardware, in which case it is the
        egm96-5 geoid undulation at (lat, lon) plus desired_floor_amsl.
        """
        if not self.get_parameter('use_geoid_altitude').value:
            return self.get_parameter('altitude').value
        if self.get_parameter('use_sim_time').value:
            self.get_logger().info(
                'use_sim_time=true: using literal altitude (sim datum), not geoid.'
            )
            return self.get_parameter('altitude').value
        floor = self.get_parameter('desired_floor_amsl').value
        model = self.get_parameter('geoid_model').value
        try:
            n = self._geoid_undulation(self._lat, self._lon, model)
        except Exception as e:
            literal = self.get_parameter('altitude').value
            self.get_logger().error(
                f'use_geoid_altitude=true but geoid lookup failed ({e}); falling '
                f'back to literal altitude {literal} m. LOCAL Z WILL BE OFFSET BY '
                f'THE GEOID (tens of m) — fix GeographicLib/GeoidEval before flight.'
            )
            return literal
        alt = n + floor
        self.get_logger().info(
            f'Geoid origin altitude: N({model})={n:.4f} + floor_amsl={floor:.4f} '
            f'=> {alt:.4f} m ellipsoidal (local z will equal OptiTrack z).'
        )
        return alt

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
