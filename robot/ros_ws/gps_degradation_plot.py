#!/usr/bin/env python3
"""
Live GPS degradation plot — subscribes to /robot_1/gps/degradation_state
and produces a rolling time-series graph.

Run inside the robot container:
    docker exec -it airstack-robot-desktop-1 bash -lc \
        'ROS_DOMAIN_ID=0 python3 /root/AirStack/simulation/isaac-sim/gps_degradation_plot.py'

Or on the host (if ROS 2 is sourced):
    python3 gps_degradation_plot.py
"""
import sys
import collections
import threading
import time

import matplotlib
matplotlib.use("TkAgg")   # change to "Qt5Agg" if TkAgg not available
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# ── Field indices in Float32MultiArray.data ─────────────────────────────────
# [state, n_los, hdop, vdop, eph_m, epv_m, delta_lat, delta_lon, delta_alt]
I_STATE   = 0
I_NLOS    = 1
I_HDOP    = 2
I_VDOP    = 3
I_EPH     = 4
I_EPV     = 5
I_DLAT    = 6
I_DLON    = 7
I_DALT    = 8

STATE_NAMES  = {0: "OPEN_SKY", 1: "DEGRADED", 2: "MARGINAL", 3: "DENIED", 4: "RECOVERY"}
STATE_COLORS = {0: "#2ecc71", 1: "#f1c40f", 2: "#e67e22", 3: "#e74c3c", 4: "#3498db"}

WINDOW_S = 120       # how many seconds of history to show
TOPIC    = "/robot_1/gps/degradation_state"
BUF      = 10_000    # max data points


class GPSDegradationSubscriber(Node):
    def __init__(self, buf: dict):
        super().__init__("gps_degradation_plot_node")
        self._buf = buf
        self._t0 = time.monotonic()
        self.create_subscription(Float32MultiArray, TOPIC, self._cb, 10)
        self.get_logger().info(f"Listening on {TOPIC}")

    def _cb(self, msg: Float32MultiArray):
        d = msg.data
        if len(d) < 9:
            return
        t = time.monotonic() - self._t0
        self._buf["t"].append(t)
        self._buf["state"].append(int(d[I_STATE]))
        self._buf["n_los"].append(float(d[I_NLOS]))
        self._buf["hdop"].append(float(d[I_HDOP]))
        self._buf["vdop"].append(float(d[I_VDOP]))
        self._buf["eph"].append(float(d[I_EPH]))
        self._buf["epv"].append(float(d[I_EPV]))
        self._buf["dalt"].append(float(d[I_DALT]))


def _spin(node):
    rclpy.spin(node)


def main():
    rclpy.init()

    buf = {k: collections.deque(maxlen=BUF)
           for k in ("t", "state", "n_los", "hdop", "vdop", "eph", "epv", "dalt")}

    node = GPSDegradationSubscriber(buf)
    t = threading.Thread(target=_spin, args=(node,), daemon=True)
    t.start()

    # ── Figure layout ────────────────────────────────────────────────────────
    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
    fig.suptitle("GPS Degradation — Live Monitor", fontsize=13, fontweight="bold")
    ax_state, ax_los, ax_dop, ax_err = axes

    # State panel (categorical background colouring)
    ax_state.set_ylabel("GPS State", fontsize=9)
    ax_state.set_yticks([0, 1, 2, 3, 4])
    ax_state.set_yticklabels(["OPEN_SKY", "DEGRADED", "MARGINAL", "DENIED", "RECOVERY"], fontsize=8)
    ax_state.set_ylim(-0.5, 4.5)
    (line_state,) = ax_state.plot([], [], "o-", ms=3, lw=1.5, color="#8e44ad")

    # n_LOS panel
    ax_los.set_ylabel("Satellites (n_LOS)", fontsize=9)
    ax_los.set_ylim(0, 60)
    ax_los.axhline(y=4,  color="#e74c3c", ls="--", lw=0.8, label="DENIED  (≤4)")
    ax_los.axhline(y=6,  color="#e67e22", ls="--", lw=0.8, label="MARGINAL (≤6)")
    ax_los.axhline(y=8,  color="#f1c40f", ls="--", lw=0.8, label="DEGRADED (≤8 or HDOP≥1.5)")
    ax_los.legend(fontsize=7, loc="upper right")
    (line_los,) = ax_los.plot([], [], lw=1.5, color="#2980b9")

    # DOP panel
    ax_dop.set_ylabel("DOP", fontsize=9)
    ax_dop.set_ylim(0, 10)
    ax_dop.axhline(y=5.0, color="#e74c3c", ls="--", lw=0.8, label="DENIED  HDOP≥5")
    ax_dop.axhline(y=3.0, color="#e67e22", ls="--", lw=0.8, label="MARGINAL HDOP≥3")
    ax_dop.axhline(y=1.5, color="#f1c40f", ls="--", lw=0.8, label="DEGRADED HDOP≥1.5")
    ax_dop.legend(fontsize=7, loc="upper right")
    (line_hdop,) = ax_dop.plot([], [], lw=1.5, color="#e67e22", label="HDOP")
    (line_vdop,) = ax_dop.plot([], [], lw=1.0, color="#95a5a6", ls=":", label="VDOP")
    ax_dop.legend(fontsize=7, loc="upper right")

    # EPH / position error panel
    ax_err.set_ylabel("Error (m)", fontsize=9)
    ax_err.set_xlabel("Time (s)", fontsize=9)
    ax_err.set_ylim(0, 30)
    (line_eph,) = ax_err.plot([], [], lw=1.5, color="#e74c3c", label="EPH (horiz)")
    (line_epv,) = ax_err.plot([], [], lw=1.0, color="#c0392b", ls=":", label="EPV (vert)")
    (line_dalt,) = ax_err.plot([], [], lw=1.0, color="#8e44ad", ls="--", label="Δalt OU drift")
    ax_err.legend(fontsize=7, loc="upper right")

    # State colour legend at bottom
    patches = [mpatches.Patch(color=c, label=n) for n, c in
               [("OPEN_SKY", "#2ecc71"), ("DEGRADED", "#f1c40f"),
                ("MARGINAL", "#e67e22"), ("DENIED", "#e74c3c"), ("RECOVERY", "#3498db")]]
    fig.legend(handles=patches, ncol=5, loc="lower center", fontsize=8,
               bbox_to_anchor=(0.5, 0.0))

    plt.tight_layout(rect=[0, 0.04, 1, 1])
    plt.ion()
    plt.show()

    # ── Update loop ──────────────────────────────────────────────────────────
    while plt.get_fignums():
        if len(buf["t"]) < 2:
            plt.pause(0.5)
            continue

        t_arr   = np.array(buf["t"])
        t_min   = max(0.0, t_arr[-1] - WINDOW_S)
        mask    = t_arr >= t_min
        t_w     = t_arr[mask]

        def _w(key):
            a = np.array(buf[key])
            return a[mask]

        states = _w("state")
        n_los  = _w("n_los")
        hdop   = _w("hdop")
        vdop   = _w("vdop")
        eph    = _w("eph")
        epv    = _w("epv")
        dalt   = _w("dalt")

        # Colour background of state panel by GPS state
        ax_state.collections.clear()
        if len(t_w) > 1:
            for i in range(len(t_w) - 1):
                ax_state.axvspan(t_w[i], t_w[i + 1],
                                 color=STATE_COLORS.get(int(states[i]), "#bdc3c7"),
                                 alpha=0.25, lw=0)

        line_state.set_data(t_w, states)
        line_los.set_data(t_w, n_los)
        line_hdop.set_data(t_w, hdop)
        line_vdop.set_data(t_w, vdop)
        line_eph.set_data(t_w, eph)
        line_epv.set_data(t_w, epv)
        line_dalt.set_data(t_w, np.abs(dalt))

        x_min, x_max = t_w[0], max(t_w[-1], t_w[0] + 10)
        for ax in axes:
            ax.set_xlim(x_min, x_max)
        ax_dop.set_ylim(0, min(max(hdop.max() * 1.3, 6.0), 20.0))
        ax_err.set_ylim(0, min(max(eph.max() * 1.3, 5.0), 60.0))

        # Live title with current values
        cur_state = STATE_NAMES.get(int(states[-1]), "?")
        fig.suptitle(
            f"GPS Degradation Monitor   |   "
            f"State: {cur_state}   n_LOS: {int(n_los[-1])}   "
            f"HDOP: {hdop[-1]:.2f}   EPH: {eph[-1]:.1f}m   Δalt: {dalt[-1]:+.2f}m",
            fontsize=11, fontweight="bold",
            color=STATE_COLORS.get(int(states[-1]), "#2c3e50"),
        )

        fig.canvas.draw_idle()
        plt.pause(0.5)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
