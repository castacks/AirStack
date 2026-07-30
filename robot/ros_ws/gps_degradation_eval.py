#!/usr/bin/env python3
"""
GPS Degradation Evaluation — live comparison of raw vs degraded GPS with
expected vs actual error metrics.

Subscribes to:
  /robot_1/sensors/gps            — raw NavSatFix (Pegasus ground-truth position)
  /robot_1/sensors/gps_degraded   — NavSatFix after OU noise injection
  /robot_1/gps/degradation_state  — Float32MultiArray [state,n_los,hdop,vdop,eph,epv,dlat,dlon,dalt]

Metrics computed:
  actual_horiz_error  — horizontal distance between raw and degraded GPS (metres)
  actual_vert_error   — |alt_degraded - alt_raw| (metres)
  expected_horiz_error— EPH reported by the degradation model (= UERE × HDOP)
  error_ratio         — actual / expected  (1.0 = perfectly calibrated)

Also saves a CSV log to /tmp/gps_eval_<timestamp>.csv for offline analysis.

Run inside the robot container:
    docker exec -it airstack-robot-desktop-1 bash -lc \\
        'ROS_DOMAIN_ID=0 python3 /root/AirStack/simulation/isaac-sim/gps_degradation_eval.py'
"""
import collections
import csv
import math
import os
import threading
import time
from datetime import datetime

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.gridspec as gridspec
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix

# ── Constants ────────────────────────────────────────────────────────────────
WINDOW_S = 120
BUF      = 10_000

STATE_NAMES  = {0: "OPEN_SKY", 1: "DEGRADED", 2: "MARGINAL", 3: "DENIED", 4: "RECOVERY"}
STATE_COLORS = {0: "#2ecc71", 1: "#f1c40f", 2: "#e67e22", 3: "#e74c3c", 4: "#3498db"}

# Float32MultiArray field indices
I_STATE, I_NLOS, I_HDOP, I_VDOP = 0, 1, 2, 3
I_EPH,   I_EPV,  I_DLAT, I_DLON, I_DALT = 4, 5, 6, 7, 8

EARTH_R = 6_371_000.0   # metres


def haversine_m(lat1, lon1, lat2, lon2):
    """Horizontal distance in metres between two geodetic points."""
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    mlat = math.radians((lat1 + lat2) / 2.0)
    north = EARTH_R * dlat
    east  = EARTH_R * math.cos(mlat) * dlon
    return math.sqrt(north * north + east * east)


# ── ROS 2 subscriber node ────────────────────────────────────────────────────

class EvalNode(Node):
    def __init__(self, buf: dict):
        super().__init__("gps_degradation_eval_node")
        self._buf  = buf
        self._t0   = time.monotonic()

        # Latest raw / degraded GPS (held until both arrive for the same epoch)
        self._raw  = None
        self._deg  = None
        self._lock = threading.Lock()

        self.create_subscription(
            NavSatFix, "/robot_1/sensors/gps",
            self._raw_cb, qos_profile_sensor_data)
        self.create_subscription(
            NavSatFix, "/robot_1/sensors/gps_degraded",
            self._deg_cb, qos_profile_sensor_data)
        self.create_subscription(
            Float32MultiArray, "/robot_1/gps/degradation_state",
            self._state_cb, 10)

        self.get_logger().info(
            "GPS eval node listening on /robot_1/sensors/gps, "
            "/robot_1/sensors/gps_degraded, /robot_1/gps/degradation_state")

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _raw_cb(self, msg: NavSatFix):
        with self._lock:
            self._raw = msg

    def _deg_cb(self, msg: NavSatFix):
        with self._lock:
            self._deg = msg
        self._try_fuse()

    def _state_cb(self, msg: Float32MultiArray):
        d = msg.data
        if len(d) < 9:
            return
        with self._lock:
            self._buf["state_latest"] = d

    def _try_fuse(self):
        """Called each time a degraded GPS arrives — fuse with latest raw."""
        with self._lock:
            raw = self._raw
            deg = self._deg
            state = self._buf.get("state_latest")

        if raw is None or deg is None or state is None:
            return

        # Skip if raw has no fix
        if raw.status.status < 0:
            return

        t = time.monotonic() - self._t0

        # ── Actual errors ────────────────────────────────────────────────────
        horiz_err = haversine_m(raw.latitude, raw.longitude,
                                deg.latitude, deg.longitude)
        vert_err  = abs(deg.altitude - raw.altitude)

        # ── Expected errors from model ────────────────────────────────────────
        eph = float(state[I_EPH])
        epv = float(state[I_EPV])

        # Ratio: how close is actual to what the model predicted
        # Clamp denominator to avoid /0 at startup
        ratio_h = horiz_err / max(eph, 0.01)
        ratio_v = vert_err  / max(epv, 0.01)

        gps_state = int(state[I_STATE])
        n_los     = float(state[I_NLOS])
        hdop      = float(state[I_HDOP])
        vdop      = float(state[I_VDOP])
        dlat      = float(state[I_DLAT])
        dlon      = float(state[I_DLON])
        dalt      = float(state[I_DALT])

        # ── Append to ring buffers ────────────────────────────────────────────
        b = self._buf
        b["t"].append(t)
        b["state"].append(gps_state)
        b["n_los"].append(n_los)
        b["hdop"].append(hdop)
        b["vdop"].append(vdop)
        b["eph"].append(eph)
        b["epv"].append(epv)
        b["horiz_err"].append(horiz_err)
        b["vert_err"].append(vert_err)
        b["ratio_h"].append(ratio_h)
        b["ratio_v"].append(ratio_v)
        b["raw_lat"].append(raw.latitude)
        b["raw_lon"].append(raw.longitude)
        b["deg_lat"].append(deg.latitude)
        b["deg_lon"].append(deg.longitude)
        b["dalt"].append(dalt)

        # ── CSV log ───────────────────────────────────────────────────────────
        writer = self._buf.get("csv_writer")
        if writer:
            writer.writerow([
                f"{t:.3f}", gps_state, STATE_NAMES.get(gps_state, "?"),
                n_los, f"{hdop:.3f}", f"{vdop:.3f}",
                f"{horiz_err:.4f}", f"{vert_err:.4f}",
                f"{eph:.4f}", f"{epv:.4f}",
                f"{ratio_h:.4f}", f"{ratio_v:.4f}",
                f"{raw.latitude:.8f}", f"{raw.longitude:.8f}", f"{raw.altitude:.3f}",
                f"{deg.latitude:.8f}", f"{deg.longitude:.8f}", f"{deg.altitude:.3f}",
                f"{dlat:.8f}", f"{dlon:.8f}", f"{dalt:.4f}",
            ])


# ── Plot ─────────────────────────────────────────────────────────────────────

def build_figure():
    fig = plt.figure(figsize=(16, 12))
    fig.patch.set_facecolor("#1a1a2e")
    gs = gridspec.GridSpec(
        3, 2,
        figure=fig,
        hspace=0.42, wspace=0.32,
        left=0.07, right=0.97, top=0.91, bottom=0.06,
    )

    axes = {
        "state":    fig.add_subplot(gs[0, :]),   # full-width top
        "err":      fig.add_subplot(gs[1, 0]),
        "ratio":    fig.add_subplot(gs[1, 1]),
        "dop":      fig.add_subplot(gs[2, 0]),
        "track":    fig.add_subplot(gs[2, 1]),
    }

    style = dict(facecolor="#16213e", framealpha=0.9)
    for ax in axes.values():
        ax.set_facecolor("#16213e")
        ax.tick_params(colors="#cccccc", labelsize=8)
        ax.xaxis.label.set_color("#aaaaaa")
        ax.yaxis.label.set_color("#aaaaaa")
        for spine in ax.spines.values():
            spine.set_edgecolor("#444466")

    return fig, axes


def make_lines(axes):
    kw = dict(lw=1.5)
    lines = {}

    # ── State panel ──────────────────────────────────────────────────────────
    ax = axes["state"]
    ax.set_ylabel("GPS State", color="#aaaaaa", fontsize=9)
    ax.set_yticks([0, 1, 2, 3, 4])
    ax.set_yticklabels(
        ["OPEN_SKY", "DEGRADED", "MARGINAL", "DENIED", "RECOVERY"],
        fontsize=8, color="#cccccc")
    ax.set_ylim(-0.5, 4.5)
    lines["state"], = ax.plot([], [], "o-", ms=2, color="#a29bfe", **kw)

    # ── Error panel ───────────────────────────────────────────────────────────
    ax = axes["err"]
    ax.set_title("Actual vs Expected Error", color="#eeeeee", fontsize=9)
    ax.set_ylabel("Error (m)", color="#aaaaaa", fontsize=9)
    ax.set_ylim(0, 15)
    lines["horiz_err"], = ax.plot([], [], color="#e74c3c", label="Actual horiz", **kw)
    lines["vert_err"],  = ax.plot([], [], color="#e74c3c", ls=":", lw=1.1, label="Actual vert")
    lines["eph"],       = ax.plot([], [], color="#f39c12", ls="--", label="Expected (EPH)")
    lines["epv"],       = ax.plot([], [], color="#f39c12", ls=":", lw=1.0, label="Expected (EPV)")
    ax.legend(fontsize=7, loc="upper left",
              facecolor="#1a1a2e", edgecolor="#444466", labelcolor="#cccccc")

    # ── Ratio panel ───────────────────────────────────────────────────────────
    ax = axes["ratio"]
    ax.set_title("Calibration: Actual / Expected", color="#eeeeee", fontsize=9)
    ax.set_ylabel("Ratio (1.0 = perfect)", color="#aaaaaa", fontsize=9)
    ax.set_ylim(0, 4)
    ax.axhline(1.0, color="#2ecc71", ls="--", lw=1.0, alpha=0.8, label="Perfect calibration")
    ax.axhspan(0.5, 1.5, color="#2ecc71", alpha=0.07)   # ±50% band
    lines["ratio_h"], = ax.plot([], [], color="#00cec9", label="Horiz ratio", **kw)
    lines["ratio_v"], = ax.plot([], [], color="#00cec9", ls=":", lw=1.1, label="Vert ratio")
    ax.legend(fontsize=7, loc="upper left",
              facecolor="#1a1a2e", edgecolor="#444466", labelcolor="#cccccc")

    # ── DOP panel ─────────────────────────────────────────────────────────────
    ax = axes["dop"]
    ax.set_title("HDOP / n_LOS", color="#eeeeee", fontsize=9)
    ax.set_ylabel("HDOP", color="#aaaaaa", fontsize=9)
    ax2 = ax.twinx()
    ax2.set_ylabel("n_LOS sats", color="#74b9ff", fontsize=8)
    ax2.tick_params(colors="#74b9ff", labelsize=8)
    ax2.set_facecolor("#16213e")
    lines["hdop"],  = ax.plot( [], [], color="#fdcb6e", label="HDOP",  **kw)
    lines["n_los"], = ax2.plot([], [], color="#74b9ff", label="n_LOS", lw=1.2)
    ax.set_ylim(0, 10)
    ax2.set_ylim(0, 55)
    ax.axhline(5.0, color="#e74c3c", ls="--", lw=0.7, alpha=0.6)
    ax.axhline(3.0, color="#e67e22", ls="--", lw=0.7, alpha=0.6)
    ax.axhline(1.5, color="#f1c40f", ls="--", lw=0.7, alpha=0.6)
    lines["_ax2"] = ax2   # store twin for xlim sync

    # ── Ground-track panel ────────────────────────────────────────────────────
    ax = axes["track"]
    ax.set_title("Ground Track (ENU offset from first fix)", color="#eeeeee", fontsize=9)
    ax.set_xlabel("East (m)", color="#aaaaaa", fontsize=9)
    ax.set_ylabel("North (m)", color="#aaaaaa", fontsize=9)
    ax.set_aspect("equal", "datalim")
    lines["raw_track"],  = ax.plot([], [], color="#2ecc71", lw=1.2, label="Raw GPS")
    lines["deg_track"],  = ax.plot([], [], color="#e74c3c", lw=1.0, ls="--", label="Degraded GPS")
    ax.legend(fontsize=7, loc="upper left",
              facecolor="#1a1a2e", edgecolor="#444466", labelcolor="#cccccc")

    return lines


def latlon_to_enu(lats, lons, lat0, lon0):
    """Convert arrays of lat/lon to local ENU (metres) relative to origin."""
    dlat = np.radians(np.array(lats) - lat0)
    dlon = np.radians(np.array(lons) - lon0)
    mlat = math.radians(lat0)
    north = EARTH_R * dlat
    east  = EARTH_R * math.cos(mlat) * dlon
    return east, north


def update_plot(buf, fig, axes, lines, window_s):
    if len(buf["t"]) < 2:
        return

    t_arr = np.array(buf["t"])
    t_min = max(0.0, t_arr[-1] - window_s)
    mask  = t_arr >= t_min
    t_w   = t_arr[mask]

    def w(k):
        return np.array(buf[k])[mask]

    states   = w("state")
    n_los    = w("n_los")
    hdop     = w("hdop")
    eph      = w("eph")
    epv      = w("epv")
    horiz    = w("horiz_err")
    vert     = w("vert_err")
    ratio_h  = w("ratio_h")
    ratio_v  = w("ratio_v")
    raw_lats = w("raw_lat")
    raw_lons = w("raw_lon")
    deg_lats = w("deg_lat")
    deg_lons = w("deg_lon")

    # ── State background colouring ────────────────────────────────────────────
    ax_s = axes["state"]
    ax_s.collections.clear()
    if len(t_w) > 1:
        for i in range(len(t_w) - 1):
            ax_s.axvspan(t_w[i], t_w[i + 1],
                         color=STATE_COLORS.get(int(states[i]), "#555555"),
                         alpha=0.28, lw=0)
    lines["state"].set_data(t_w, states)
    ax_s.set_xlim(t_w[0], max(t_w[-1], t_w[0] + 10))

    # ── Error panel ───────────────────────────────────────────────────────────
    lines["horiz_err"].set_data(t_w, horiz)
    lines["vert_err"].set_data(t_w, vert)
    lines["eph"].set_data(t_w, eph)
    lines["epv"].set_data(t_w, epv)
    ax = axes["err"]
    ax.set_xlim(t_w[0], t_w[-1] + 1)
    ax.set_ylim(0, max(max(horiz.max(), eph.max()) * 1.3, 2.0))

    # ── Ratio panel ───────────────────────────────────────────────────────────
    # Clip extreme spikes for display
    rh_disp = np.clip(ratio_h, 0, 6)
    rv_disp = np.clip(ratio_v, 0, 6)
    lines["ratio_h"].set_data(t_w, rh_disp)
    lines["ratio_v"].set_data(t_w, rv_disp)
    ax = axes["ratio"]
    ax.set_xlim(t_w[0], t_w[-1] + 1)
    ax.set_ylim(0, max(rh_disp.max() * 1.2, 2.5))

    # ── DOP panel ─────────────────────────────────────────────────────────────
    lines["hdop"].set_data(t_w, hdop)
    lines["n_los"].set_data(t_w, n_los)
    ax = axes["dop"]
    ax.set_xlim(t_w[0], t_w[-1] + 1)
    ax.set_ylim(0, max(hdop.max() * 1.3, 3.0))
    lines["_ax2"].set_xlim(t_w[0], t_w[-1] + 1)

    # ── Ground track ──────────────────────────────────────────────────────────
    if len(raw_lats) > 1 and raw_lats[0] != 0:
        lat0, lon0 = raw_lats[0], raw_lons[0]
        re, rn = latlon_to_enu(raw_lats, raw_lons, lat0, lon0)
        de, dn = latlon_to_enu(deg_lats, deg_lons, lat0, lon0)
        lines["raw_track"].set_data(re, rn)
        lines["deg_track"].set_data(de, dn)
        axes["track"].relim()
        axes["track"].autoscale_view()

    # ── Super title with live numbers ─────────────────────────────────────────
    cur_state = STATE_NAMES.get(int(states[-1]), "?")
    col       = STATE_COLORS.get(int(states[-1]), "#ffffff")
    fig.suptitle(
        f"GPS Degradation Evaluation  ·  "
        f"State: {cur_state}  ·  "
        f"n_LOS: {int(n_los[-1])}  ·  HDOP: {hdop[-1]:.2f}  ·  "
        f"Actual Δhoriz: {horiz[-1]:.2f}m  ·  Expected EPH: {eph[-1]:.2f}m  ·  "
        f"Ratio: {ratio_h[-1]:.2f}",
        fontsize=10, fontweight="bold", color=col,
        y=0.97,
    )

    fig.canvas.draw_idle()


def main():
    rclpy.init()

    buf = {k: collections.deque(maxlen=BUF) for k in (
        "t", "state", "n_los", "hdop", "vdop",
        "eph", "epv", "horiz_err", "vert_err",
        "ratio_h", "ratio_v",
        "raw_lat", "raw_lon", "deg_lat", "deg_lon", "dalt",
    )}
    buf["state_latest"] = None

    # ── CSV log ───────────────────────────────────────────────────────────────
    csv_path = f"/tmp/gps_eval_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    csv_fh   = open(csv_path, "w", newline="")
    writer   = csv.writer(csv_fh)
    writer.writerow([
        "t_s", "state_id", "state_name",
        "n_los", "hdop", "vdop",
        "actual_horiz_m", "actual_vert_m",
        "expected_eph_m", "expected_epv_m",
        "ratio_horiz", "ratio_vert",
        "raw_lat", "raw_lon", "raw_alt",
        "deg_lat", "deg_lon", "deg_alt",
        "delta_lat", "delta_lon", "delta_alt_m",
    ])
    buf["csv_writer"] = writer
    print(f"[eval] Logging to {csv_path}")

    node = EvalNode(buf)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # ── Matplotlib ────────────────────────────────────────────────────────────
    fig, axes = build_figure()
    lines     = make_lines(axes)

    # State legend
    patches = [mpatches.Patch(color=c, label=n) for n, c in
               [("OPEN_SKY","#2ecc71"),("DEGRADED","#f1c40f"),
                ("MARGINAL","#e67e22"),("DENIED","#e74c3c"),("RECOVERY","#3498db")]]
    fig.legend(handles=patches, ncol=5, loc="lower center",
               fontsize=8, bbox_to_anchor=(0.5, 0.0),
               facecolor="#1a1a2e", edgecolor="#444466", labelcolor="#eeeeee")

    plt.ion()
    plt.show()

    try:
        while plt.get_fignums():
            update_plot(buf, fig, axes, lines, WINDOW_S)
            plt.pause(0.5)
    finally:
        csv_fh.close()
        print(f"[eval] CSV saved → {csv_path}")
        rclpy.shutdown()


if __name__ == "__main__":
    main()
