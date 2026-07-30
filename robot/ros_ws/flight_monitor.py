#!/usr/bin/env python3
"""
Flight monitor — logs drone position, GPS degradation state, arm state, and
navigation phase to CSV and prints a live formatted console table.

Subscribes (domain 0, GCS bridge):
  /robot_1/gps/degradation_state   Float32MultiArray
  /robot_1/sensors/gps             NavSatFix  (raw Pegasus GPS → PX4)
  /robot_1/sensors/gps_degraded    NavSatFix  (OU-corrupted GPS → PX4)

Subscribes (domain 1, robot):
  /robot_1/interface/mavros/state            State
  /robot_1/interface/mavros/local_position/odom  Odometry
  /robot_1/interface/mavros/global_position/global NavSatFix

Run inside robot container:
    docker exec -it airstack-robot-desktop-1 bash -lc \\
        'ROS_DOMAIN_ID=0 python3 /root/AirStack/robot/ros_ws/flight_monitor.py'

CSV is written to /tmp/flight_<timestamp>.csv
"""
from __future__ import annotations
import csv, math, os, threading, time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State

# ── QoS profiles ─────────────────────────────────────────────────────────────
SENSOR_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)
RELIABLE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

# ── State name maps ───────────────────────────────────────────────────────────
GPS_STATE   = {0:"OPEN_SKY", 1:"DEGRADED", 2:"MARGINAL", 3:"DENIED", 4:"RECOVERY"}
SYS_STATUS  = {0:"UNINIT", 1:"BOOT", 2:"CALIBRATING", 3:"STANDBY",
               4:"ACTIVE", 5:"CRITICAL", 6:"EMERGENCY", 7:"POWEROFF"}
EARTH_R     = 6_371_000.0

def haversine_m(lat1, lon1, lat2, lon2):
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    mlat = math.radians((lat1 + lat2) / 2)
    return math.sqrt((EARTH_R * dlat)**2 + (EARTH_R * math.cos(mlat) * dlon)**2)


class FlightMonitor(Node):
    def __init__(self, csv_writer):
        super().__init__("flight_monitor")
        self._w  = csv_writer
        self._t0 = time.monotonic()
        self._lock = threading.Lock()

        # ── live data stores ──────────────────────────────────────────────────
        self._state: State | None          = None
        self._odom:  Odometry | None       = None
        self._gps_raw:  NavSatFix | None   = None
        self._gps_deg:  NavSatFix | None   = None
        self._gps_mav:  NavSatFix | None   = None
        self._deg_arr:  list | None        = None   # Float32MultiArray.data

        # All on domain 0 (GCS bridge passes everything through)
        self.create_subscription(Float32MultiArray,
            "/robot_1/gps/degradation_state", self._cb_deg_state, RELIABLE_QOS)
        self.create_subscription(NavSatFix,
            "/robot_1/sensors/gps",         self._cb_raw,     SENSOR_QOS)
        self.create_subscription(NavSatFix,
            "/robot_1/sensors/gps_degraded", self._cb_deg,    SENSOR_QOS)
        self.create_subscription(NavSatFix,
            "/robot_1/interface/mavros/global_position/global",
            self._cb_mav_gps, SENSOR_QOS)
        self.create_subscription(State,
            "/robot_1/interface/mavros/state", self._cb_state, RELIABLE_QOS)
        self.create_subscription(Odometry,
            "/robot_1/interface/mavros/local_position/odom",
            self._cb_odom, SENSOR_QOS)

        self.create_timer(1.0, self._print_row)
        self.get_logger().info("Flight monitor started. Writing to CSV…")

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _cb_deg_state(self, msg: Float32MultiArray):
        with self._lock:
            self._deg_arr = list(msg.data) if len(msg.data) >= 9 else None

    def _cb_raw(self, msg: NavSatFix):
        with self._lock: self._gps_raw = msg

    def _cb_deg(self, msg: NavSatFix):
        with self._lock: self._gps_deg = msg

    def _cb_mav_gps(self, msg: NavSatFix):
        with self._lock: self._gps_mav = msg

    def _cb_state(self, msg: State):
        with self._lock: self._state = msg

    def _cb_odom(self, msg: Odometry):
        with self._lock: self._odom = msg

    # ── periodic log ──────────────────────────────────────────────────────────

    def _print_row(self):
        with self._lock:
            t   = time.monotonic() - self._t0
            st  = self._state
            od  = self._odom
            raw = self._gps_raw
            deg = self._gps_deg
            mav = self._gps_mav
            arr = self._deg_arr

        # ── extract values with safe defaults ─────────────────────────────────
        armed     = st.armed          if st else False
        mode      = st.mode           if st else "?"
        sys_s     = st.system_status  if st else 0
        sys_name  = SYS_STATUS.get(sys_s, str(sys_s))

        # Local ENU position (map frame, relative to takeoff)
        x = od.pose.pose.position.x if od else 0.0
        y = od.pose.pose.position.y if od else 0.0
        z = od.pose.pose.position.z if od else 0.0

        # Velocity
        vx = od.twist.twist.linear.x if od else 0.0
        vy = od.twist.twist.linear.y if od else 0.0
        vz = od.twist.twist.linear.z if od else 0.0
        speed = math.sqrt(vx**2 + vy**2 + vz**2)

        # GPS degradation
        gps_state_id   = int(arr[0])   if arr else -1
        gps_state_name = GPS_STATE.get(gps_state_id, "?") if arr else "NO_DATA"
        n_los   = int(arr[1])          if arr else 0
        hdop    = float(arr[2])        if arr else 0.0
        vdop    = float(arr[3])        if arr else 0.0
        eph_m   = float(arr[4])        if arr else 0.0
        epv_m   = float(arr[5])        if arr else 0.0
        dlat    = float(arr[6])        if arr else 0.0
        dlon    = float(arr[7])        if arr else 0.0
        dalt    = float(arr[8])        if arr else 0.0

        # Horizontal position error injected (raw vs degraded GPS in metres)
        horiz_err = 0.0
        if raw and deg and raw.latitude != 0:
            horiz_err = haversine_m(raw.latitude, raw.longitude,
                                     deg.latitude, deg.longitude)
        vert_err = abs(deg.altitude - raw.altitude) if (deg and raw) else 0.0

        # PX4 EKF position (MAVROS global → lat/lon)
        px4_lat = mav.latitude  if mav else 0.0
        px4_lon = mav.longitude if mav else 0.0
        px4_alt = mav.altitude  if mav else 0.0

        # ── console output ────────────────────────────────────────────────────
        arm_str = "ARMED  " if armed else "disarmd"
        color = {
            "OPEN_SKY": "\033[92m", "DEGRADED": "\033[93m",
            "MARGINAL": "\033[33m", "DENIED":   "\033[91m",
            "RECOVERY": "\033[94m", "NO_DATA":  "\033[90m",
        }.get(gps_state_name, "\033[0m")
        reset = "\033[0m"

        print(
            f"[{t:7.1f}s] {arm_str} | {mode:<12} | sys={sys_name:<10} | "
            f"ENU=({x:6.1f},{y:6.1f},{z:5.1f})m spd={speed:.1f}m/s | "
            f"{color}GPS:{gps_state_name:<9}{reset} LOS={n_los:2d} "
            f"HDOP={hdop:.2f} EPH={eph_m:.1f}m | "
            f"Δhoriz={horiz_err:.2f}m Δalt={dalt:+.2f}m"
        )

        # ── CSV row ───────────────────────────────────────────────────────────
        self._w.writerow([
            f"{t:.2f}", int(armed), mode, sys_s, sys_name,
            f"{x:.3f}", f"{y:.3f}", f"{z:.3f}", f"{speed:.3f}",
            gps_state_id, gps_state_name,
            n_los, f"{hdop:.3f}", f"{vdop:.3f}",
            f"{eph_m:.3f}", f"{epv_m:.3f}",
            f"{dlat:.8f}", f"{dlon:.8f}", f"{dalt:.4f}",
            f"{horiz_err:.4f}", f"{vert_err:.4f}",
            f"{px4_lat:.8f}", f"{px4_lon:.8f}", f"{px4_alt:.3f}",
        ])


def main():
    rclpy.init()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = f"/tmp/flight_{stamp}.csv"
    fh = open(csv_path, "w", newline="")
    writer = csv.writer(fh)
    writer.writerow([
        "t_s", "armed", "mode", "sys_status_id", "sys_status",
        "x_m", "y_m", "z_m", "speed_ms",
        "gps_state_id", "gps_state",
        "n_los", "hdop", "vdop", "eph_m", "epv_m",
        "delta_lat", "delta_lon", "delta_alt_m",
        "actual_horiz_err_m", "actual_vert_err_m",
        "px4_lat", "px4_lon", "px4_alt_m",
    ])

    print(f"\n[flight_monitor] Logging to {csv_path}")
    print(f"{'Time':>8} | {'Arm':7} | {'Mode':12} | {'System':10} | {'ENU Position':30} | {'GPS State':35} | {'Error':25}")
    print("-" * 130)

    node = FlightMonitor(writer)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        fh.close()
        print(f"\n[flight_monitor] CSV saved → {csv_path}")
        rclpy.shutdown()


if __name__ == "__main__":
    main()
