"""GPS degradation ROS 2 node — wraps GpsDegradationModel for Isaac Sim integration.

Usage in Isaac Sim main loop:
    while simulation_app.is_running():
        world.step(render=False)
        gps_node.set_world_pos(x, y, z)
        gps_node.physics_step(world.current_time)
        rclpy.spin_once(gps_node, timeout_sec=0.0)
"""
from __future__ import annotations
import math
import os
import sys

_LAUNCH_DIR = os.path.dirname(os.path.realpath(__file__))
_SIM_DIR    = os.path.normpath(os.path.join(_LAUNCH_DIR, ".."))
if _SIM_DIR not in sys.path:
    sys.path.insert(0, _SIM_DIR)

import builtin_interfaces.msg
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped
from visualization_msgs.msg import Marker

from gps_degradation_staging import GpsDegradationModel, GpsDegradationConfig, GpsState
from gps_degradation_staging.runtime_output import set_latest_output


class GPSDegradationNode(Node):
    def __init__(
        self,
        vehicle_id: int = 1,
        scenario: str = "urban",
        cfg: GpsDegradationConfig = None,
        fallback_origin_lat: float | None = None,
        fallback_origin_lon: float | None = None,
        fallback_origin_alt: float = 0.0,
    ):
        super().__init__(f"gps_degradation_{vehicle_id}")
        self._vehicle_id = vehicle_id

        if cfg is None:
            cfg = GpsDegradationConfig(scenario=scenario)

        # Pipeline — passes self as ros2_node so state publisher auto-creates
        self._model = GpsDegradationModel(cfg=cfg, ros2_node=self, robot_id=vehicle_id)

        self._pub_gps = self.create_publisher(
            NavSatFix,
            f"/robot_{vehicle_id}/sensors/gps_degraded",
            10,
        )
        self._pub_twist = self.create_publisher(
            TwistStamped,
            f"/robot_{vehicle_id}/sensors/gps_twist_degraded",
            10,
        )
        self._pub_marker = self.create_publisher(
            Marker,
            f"/robot_{vehicle_id}/gps/degradation_marker",
            10,
        )

        self._latest_gps: NavSatFix | None = None
        self._latest_twist: TwistStamped | None = None
        self._world_pos = (0.0, 0.0, 0.0)
        self._jamming = False
        self._step_count = 0
        self._fallback_origin = (
            fallback_origin_lat,
            fallback_origin_lon,
            float(fallback_origin_alt),
        )
        self._warned_no_raw_gps = False

        self.create_subscription(
            NavSatFix,
            f"/robot_{vehicle_id}/sensors/gps",
            self._gps_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TwistStamped,
            f"/robot_{vehicle_id}/sensors/gps_twist",
            self._twist_cb,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"GPS degradation node: vehicle={vehicle_id}  scenario={scenario}  "
            f"constellation={'TLE' if self._model._constellation.using_tle else 'unavailable'}"
        )

    # ------------------------------------------------------------------
    # Callbacks — just cache latest messages
    # ------------------------------------------------------------------

    def _gps_cb(self, msg: NavSatFix):
        self._latest_gps = msg

    def _twist_cb(self, msg: TwistStamped):
        self._latest_twist = msg

    # ------------------------------------------------------------------
    # Called from Isaac Sim main loop
    # ------------------------------------------------------------------

    def set_world_pos(self, x: float, y: float, z: float):
        """Update drone world position (meters, Isaac Sim Z-up coords)."""
        self._world_pos = (x, y, z)

    def set_jamming(self, active: bool):
        self._jamming = active

    def physics_step(self, sim_time_s: float):
        """Drive pipeline. MUST be called from Isaac Sim loop after world.step()."""
        self._step_count += 1
        self._log_step_count = getattr(self, "_log_step_count", 0) + 1
        if self._latest_gps is None:
            origin_lat, origin_lon, origin_alt = self._fallback_origin
            if origin_lat is None or origin_lon is None:
                return
            if not self._warned_no_raw_gps:
                self.get_logger().warn(
                    "No raw ROS GPS publisher detected; using Pegasus world "
                    "origin + Isaac world position for degradation-state monitoring."
                )
                self._warned_no_raw_gps = True
            lat_deg, lon_deg, alt_m = _world_to_geodetic(
                self._world_pos,
                float(origin_lat),
                float(origin_lon),
                float(origin_alt),
            )
        else:
            gps = self._latest_gps
            lat_deg = gps.latitude
            lon_deg = gps.longitude
            alt_m = gps.altitude

        output = self._model.step(
            sim_time_s=sim_time_s,
            step_count=self._step_count,
            lat_deg=lat_deg,
            lon_deg=lon_deg,
            alt_m=alt_m,
            world_pos=self._world_pos,
            jamming=self._jamming,
        )
        set_latest_output(output)

        if self._log_step_count % 100 == 0:
            self.get_logger().info(
                f"[GPS t={sim_time_s:.1f}s] "
                f"state={output.state.name}  LOS={output.n_los}  "
                f"HDOP={output.hdop:.2f}  VDOP={output.vdop:.2f}  "
                f"EPH={output.eph_m:.2f}m  EPV={output.epv_m:.2f}m  "
                f"dAlt={output.delta_alt:+.2f}m  "
                f"pos=({self._world_pos[0]:.1f},{self._world_pos[1]:.1f},{self._world_pos[2]:.1f})"
            )

        self._publish_gps(output, sim_time_s, raw_alt_m=alt_m)
        self._publish_twist(output, sim_time_s)
        self._publish_marker(output, sim_time_s)

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def _publish_gps(self, output, sim_time_s: float, raw_alt_m: float = 0.0):
        msg = NavSatFix()
        msg.header.stamp = _ros_stamp(sim_time_s)
        msg.header.frame_id = "gps"
        msg.status.service = NavSatStatus.SERVICE_GPS

        if output.state == GpsState.DENIED:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
            msg.latitude = msg.longitude = msg.altitude = 0.0
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        else:
            msg.status.status = NavSatStatus.STATUS_FIX
            msg.latitude  = output.lat_deg
            msg.longitude = output.lon_deg
            msg.altitude = raw_alt_m + output.delta_alt
            eph2 = output.eph_m ** 2
            epv2 = output.epv_m ** 2
            msg.position_covariance = [eph2, 0.0, 0.0, 0.0, eph2, 0.0, 0.0, 0.0, epv2]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self._pub_gps.publish(msg)

    def _publish_twist(self, output, sim_time_s: float):
        if self._latest_twist is None:
            return
        msg = TwistStamped()
        msg.header.stamp = _ros_stamp(sim_time_s)
        msg.header.frame_id = "gps"
        msg.twist = self._latest_twist.twist  # velocity passthrough
        self._pub_twist.publish(msg)

    def _publish_marker(self, output, sim_time_s: float):
        marker = Marker()
        marker.header.stamp = _ros_stamp(sim_time_s)
        marker.header.frame_id = "map"
        marker.ns = "gps_degradation"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(self._world_pos[0])
        marker.pose.position.y = float(self._world_pos[1])
        marker.pose.position.z = float(self._world_pos[2]) + 0.8
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.16
        marker.color.a = 1.0
        marker.color.r, marker.color.g, marker.color.b = {
            GpsState.OPEN_SKY: (0.2, 1.0, 0.2),
            GpsState.DEGRADED: (1.0, 0.8, 0.1),
            GpsState.MARGINAL: (1.0, 0.4, 0.0),
            GpsState.DENIED: (1.0, 0.1, 0.1),
            GpsState.RECOVERY: (0.2, 0.8, 1.0),
        }[output.state]
        marker.text = (
            f"GPS {output.state.name} | LOS {output.n_los} | "
            f"HDOP {output.hdop:.2f} VDOP {output.vdop:.2f}\n"
            f"EPH {output.eph_m:.2f}m EPV {output.epv_m:.2f}m | "
            f"OU dAlt {output.delta_alt:+.2f}m"
        )
        self._pub_marker.publish(marker)


def _ros_stamp(sim_time_s: float) -> builtin_interfaces.msg.Time:
    t = builtin_interfaces.msg.Time()
    t.sec = int(sim_time_s)
    t.nanosec = int((sim_time_s - int(sim_time_s)) * 1e9)
    return t


def _world_to_geodetic(world_pos, origin_lat_deg: float, origin_lon_deg: float, origin_alt_m: float):
    """Approximate Pegasus ENU world coordinates as geodetic coordinates."""
    east_m, north_m, up_m = world_pos
    earth_radius_m = 6_371_000.0
    lat_rad = math.radians(origin_lat_deg)
    dlat = math.degrees(float(north_m) / earth_radius_m)
    dlon = math.degrees(float(east_m) / (earth_radius_m * math.cos(lat_rad) + 1e-9))
    return origin_lat_deg + dlat, origin_lon_deg + dlon, origin_alt_m + float(up_m)
