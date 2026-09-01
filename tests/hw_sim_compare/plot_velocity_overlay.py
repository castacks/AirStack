#!/usr/bin/env python3
"""Overlay hardware /drone_1/pose vs sim pose vs log time; RMSE + FOPDT step fit."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def interp_xyz(t_src, xyz, t_dst):
    if len(t_src) < 2 or len(t_dst) == 0:
        return np.full((len(t_dst), 3), np.nan)
    out = np.column_stack([np.interp(t_dst, t_src, xyz[:, i]) for i in range(3)])
    return out


def rmse(a, b):
    d = a - b
    return {
        "xyz": float(np.sqrt(np.mean(np.sum(d * d, axis=1)))),
        "xy": float(np.sqrt(np.mean(np.sum(d[:, :2] * d[:, :2], axis=1)))),
        "z": float(np.sqrt(np.mean(d[:, 2] * d[:, 2]))),
    }


def fit_fopdt(t, y, y_ss, y0=0.0):
    """First-order-plus-dead-time on a rising/falling step. Returns (dead_s, tau_s)."""
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    amp = y_ss - y0
    if abs(amp) < 1e-6 or len(t) < 5:
        return float("nan"), float("nan")
    frac = (y - y0) / amp
    # dead time: first leave 5% of step
    idx_d = np.argmax(frac >= 0.05)
    if frac[idx_d] < 0.05:
        return float("nan"), float("nan")
    dead = float(t[idx_d] - t[0])
    # tau: time from dead to 63.2%
    idx_t = np.argmax(frac >= 0.632)
    if frac[idx_t] < 0.632:
        # least-squares first-order after dead
        mask = t >= t[idx_d]
        tt = t[mask] - t[idx_d]
        yy = np.clip(frac[mask], 1e-6, 0.999)
        # y = 1 - exp(-t/tau) => tau = -t / log(1-y)
        taus = -tt[yy < 0.95] / np.log(1.0 - yy[yy < 0.95])
        tau = float(np.median(taus)) if len(taus) else float("nan")
        return dead, tau
    tau = float(t[idx_t] - t[idx_d])
    return dead, tau


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--sim", type=Path, required=True, help="sim result npz from replay_velocity_zoh")
    p.add_argument("--hw", type=Path, default=Path(__file__).with_name("velocity_replay.npz"))
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--step-t0", type=float, default=None)
    args = p.parse_args()

    sim = np.load(args.sim)
    hw = np.load(args.hw)
    sim_t = np.asarray(sim["sim_pose_t"], dtype=float)
    sim_xyz = np.asarray(sim["sim_pose"], dtype=float)
    hw_t = np.asarray(hw["pose_times"], dtype=float)
    cmd_t0 = float(np.asarray(hw["command_times"])[0]) if "command_times" in hw.files else 0.0
    # replay t=0 is the first velocity sample, not bag start
    if len(sim_t) and sim_t[0] < 1.0 and cmd_t0 > 1.0:
        sim_t = sim_t + cmd_t0
    hw_xyz = np.asarray(hw["pose_position"], dtype=float)
    cmd_t = np.asarray(hw["command_times"], dtype=float)
    cmd_v = np.asarray(hw["velocity_command"], dtype=float)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    report = {"sim_file": str(args.sim), "n_sim_pose": int(len(sim_t))}

    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=False)

    if len(sim_t) and len(hw_t):
        t_lo = max(float(sim_t[0]), float(hw_t[0]))
        t_hi = min(float(sim_t[-1]), float(hw_t[-1]))
        t_grid = np.arange(t_lo, t_hi, 0.02) if t_hi > t_lo else sim_t
        hw_i = interp_xyz(hw_t, hw_xyz, t_grid)
        sim_i = interp_xyz(sim_t, sim_xyz, t_grid)
        err = rmse(sim_i, hw_i)
        report["rmse"] = err
        report["overlap_s"] = float(t_hi - t_lo)
    else:
        err = None
        t_grid = hw_t

    ax = axes[0]
    ax.plot(hw_t, hw_xyz[:, 0], label="hw x", color="C0")
    ax.plot(hw_t, hw_xyz[:, 1], label="hw y", color="C1")
    ax.plot(hw_t, hw_xyz[:, 2], label="hw z", color="C2")
    if len(sim_t):
        ax.plot(sim_t, sim_xyz[:, 0], "--", label="sim x", color="C0")
        ax.plot(sim_t, sim_xyz[:, 1], "--", label="sim y", color="C1")
        ax.plot(sim_t, sim_xyz[:, 2], "--", label="sim z", color="C2")
    ax.set_ylabel("position (m)")
    ax.set_xlabel("log time (s)")
    title = "hardware /drone_1/pose vs Isaac pose (velocity ZOH)"
    if err:
        title += f"  XY RMSE={err['xy']:.3f} m  Z RMSE={err['z']:.3f} m"
    ax.set_title(title)
    ax.legend(ncol=3, fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.plot(hw_xyz[:, 0], hw_xyz[:, 1], label="hw XY")
    if len(sim_t):
        ax.plot(sim_xyz[:, 0], sim_xyz[:, 1], "--", label="sim XY")
    ax.scatter([0, 0, -1, -2], [-2, -3, -3, -3], c="k", marker="x", label="waypoints (doc only)")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[2]
    ax.plot(cmd_t, cmd_v[:, 0], label="vx cmd")
    ax.plot(cmd_t, cmd_v[:, 1], label="vy cmd")
    ax.plot(cmd_t, cmd_v[:, 2], label="vz cmd")
    ax.set_ylabel("v_des (m/s)")
    ax.set_xlabel("log time (s)")
    ax.legend(ncol=3, fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[3]
    sim_vel_t = np.asarray(sim["sim_vel_t"], dtype=float) if "sim_vel_t" in sim.files else np.array([])
    sim_vel = np.asarray(sim["sim_vel"], dtype=float) if "sim_vel" in sim.files else np.zeros((0, 3))
    if len(sim_vel_t):
        ax.plot(sim_vel_t, sim_vel[:, 0], label="sim vx")
        ax.plot(sim_vel_t, sim_vel[:, 1], label="sim vy")
        ax.plot(sim_vel_t, sim_vel[:, 2], label="sim vz")
    ax.set_ylabel("sim velocity (m/s)")
    ax.set_xlabel("replay time (s)")
    ax.legend(ncol=3, fontsize=8)
    ax.grid(True, alpha=0.3)

    step_t0 = args.step_t0
    if step_t0 is None and args.sim.with_suffix(".json").is_file():
        meta = json.loads(args.sim.with_suffix(".json").read_text())
        step_t0 = meta.get("step_t0_s")
    if step_t0 is not None and len(sim_vel_t):
        mask = (sim_vel_t >= step_t0) & (sim_vel_t <= step_t0 + 3.0)
        if np.count_nonzero(mask) > 5:
            dead, tau = fit_fopdt(sim_vel_t[mask], sim_vel[mask, 1], y_ss=-0.6, y0=0.0)
            report["step"] = {
                "t0_s": float(step_t0),
                "dead_time_s": dead,
                "tau_s": tau,
                "hardware_target_dead_s": 0.12,
                "hardware_target_tau_s": 0.555,
            }
            ax.axvline(step_t0, color="k", ls=":", label="step")
            ax.set_title(f"step FOPDT dead={dead*1000:.0f} ms  τ={tau*1000:.0f} ms  (hw ~120 / 555)")

    fig.tight_layout()
    fig.savefig(args.out, dpi=140)
    report_path = args.out.with_suffix(".json")
    report_path.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
