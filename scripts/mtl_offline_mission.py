#!/usr/bin/env python3
"""Offline closed-loop MTL mission: planner -> follower -> vehicle/gimbal model -> scoring.

    python3 scripts/mtl_offline_mission.py                       # default scenario, all agents
    python3 scripts/mtl_offline_mission.py --run-id offline-test --planner-bin PATH/mtl_search_plan

A no-Isaac rehearsal of exactly the code the robots run:

  * the team is planned by the ``mtl_search_plan`` CLI (same adapter + vendored
    mtl::planner as ``mtl_search_planner_node``),
  * each agent's track is flown by ``mtl_trajectory_follower.follower_core``
    (the node's own logic) at 20 Hz,
  * against a point-mass stand-in for PX4 + the AirStack PID cascade
    (position P-loop -> saturated velocity -> acceleration-limited response,
    rate-limited yaw) and a slew-limited gimbal whose MEASURED angle lags the
    command, like the Isaac mount,
  * and scored by ``mtl_metrics_logger`` (per robot, then the team fusion of
    ``scripts/analyze_mtl_run.py``).

The robots start at their homes already at mission altitude (i.e. just after
TakeoffTask). Outputs land in ``runs/<run_id>/`` like a real run. This is a
kinematic rehearsal, not a flight-dynamics validation: its job is to catch
frame, indexing and scheduling bugs before an Isaac session.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
for sub in ("robot/ros_ws/src/local/controls/mtl_trajectory_follower",
            "robot/ros_ws/src/behavior/mtl_metrics_logger"):
    sys.path.insert(0, str(REPO / sub))

from mtl_metrics_logger.analysis import write_run_outputs  # noqa: E402
from mtl_metrics_logger.detection import boresight_ground_point, footprint_radius  # noqa: E402
from mtl_trajectory_follower import follower_core as fc  # noqa: E402
from mtl_trajectory_follower.gimbal_math import slew_limit, wrap_pi  # noqa: E402

CONFIG = REPO / "stacks/mtl_search/config"


def find_planner(explicit: str | None) -> str:
    cands = [explicit, os.environ.get("MTL_SEARCH_PLAN_BIN"), shutil.which("mtl_search_plan"),
             str(REPO / "robot/ros_ws/install/mtl_search_planner/lib/mtl_search_planner/mtl_search_plan"),
             str(REPO / "robot/ros_ws/build/mtl_search_planner/mtl_search_plan")]
    for c in cands:
        if c and Path(c).is_file() and os.access(c, os.X_OK):
            return c
    raise SystemExit("mtl_search_plan not found: build mtl_search_planner (bws --packages-select "
                     "mtl_search_planner) or pass --planner-bin")


class Vehicle:
    """Point-mass stand-in for PX4 + pid_controller (x/y P gain 1 -> |v| <= v_max)."""

    def __init__(self, pos, yaw, v_max=7.0, vz_max=1.5, a_max=4.0, tau=0.6, yaw_rate=math.radians(90)):
        self.p = list(pos)
        self.v = [0.0, 0.0, 0.0]
        self.yaw = yaw
        self.v_max, self.vz_max, self.a_max, self.tau, self.yaw_rate = v_max, vz_max, a_max, tau, yaw_rate

    def step(self, carrot, carrot_yaw, dt):
        vc = [carrot[0] - self.p[0], carrot[1] - self.p[1], carrot[2] - self.p[2]]
        h = math.hypot(vc[0], vc[1])
        if h > self.v_max:
            vc[0], vc[1] = vc[0] * self.v_max / h, vc[1] * self.v_max / h
        vc[2] = max(-self.vz_max, min(self.vz_max, vc[2]))
        for i in range(3):
            a = max(-self.a_max, min(self.a_max, (vc[i] - self.v[i]) / self.tau))
            self.v[i] += a * dt
            self.p[i] += self.v[i] * dt
        self.yaw = wrap_pi(self.yaw + max(-self.yaw_rate * dt, min(self.yaw_rate * dt, wrap_pi(carrot_yaw - self.yaw))))


def fly_agent(name, track_json, scenario, rate_hz=20.0, max_s=900.0):
    tr = json.loads(Path(track_json).read_text())
    s = tr["samples"]
    n = len(s["t"])
    track = fc.Track(x=s["x_map"], y=s["y_map"], z=s["z_map"], yaw=s["yaw_enu"],
                     speed=[tr["speed_mps"]] * n, bx=s["bx_map"], by=s["by_map"], bz=s["bz_map"],
                     arc=s["arc"], t=s["t"], phi=s["gimbal_phi"])
    cfg = fc.FollowerConfig(min_turn_radius_m=tr["min_turn_radius_m"], single_axis=tr["single_axis"],
                            tilt_rad=tr["tilt_rad"], speed_mps=tr["speed_mps"])
    fol = fc.TrackFollower(track, cfg)
    hx, hy, hz = tr["home_enu"]
    veh = Vehicle((0.0, 0.0, track.z[0]), track.yaw[0])
    gim_meas = None
    slew = math.radians(120.0)
    dt = 1.0 / rate_hz
    fov = tr["fov_rad"]
    rows = []
    t = 0.0
    fol.start(veh.p)
    while t < max_s:
        out = fol.step(veh.p, veh.yaw, dt)
        cmd = out.gimbal
        gim_meas = cmd if gim_meas is None else slew_limit(gim_meas, cmd, slew * dt)
        xw, yw, zw = veh.p[0] + hx, veh.p[1] + hy, veh.p[2] + hz
        bore = boresight_ground_point((xw, yw, zw), gim_meas[1], gim_meas[2], 0.0)
        aim = (out.aim[0] + hx, out.aim[1] + hy)
        rows.append({
            "t": t, "agent": name, "state": out.state_name,
            "x_map": veh.p[0], "y_map": veh.p[1], "z_map": veh.p[2], "x_world": xw, "y_world": yw, "z_world": zw,
            "n": yw, "e": xw, "d": -zw, "yaw": veh.yaw, "vx": veh.v[0], "vy": veh.v[1], "vz": veh.v[2],
            "speed": math.hypot(veh.v[0], veh.v[1]),
            "cmd_roll": cmd[0], "cmd_pitch": cmd[1], "cmd_yaw": cmd[2],
            "meas_roll": gim_meas[0], "meas_pitch": gim_meas[1], "meas_yaw": gim_meas[2], "gimbal_measured": 1,
            "bore_x_world": bore[0] if bore else None, "bore_y_world": bore[1] if bore else None,
            "slant_m": bore[2] if bore else None, "footprint_r_m": footprint_radius(bore[2], fov) if bore else None,
            "progress_m": out.progress_m, "remaining_m": out.remaining_m, "xte_m": out.cross_track_error_m,
            "carrot_x_map": out.carrot[0], "carrot_y_map": out.carrot[1], "carrot_z_map": out.carrot[2],
            "aim_x_world": aim[0], "aim_y_world": aim[1],
            "pointing_error_m": math.hypot(bore[0] - aim[0], bore[1] - aim[1]) if bore else None,
        })
        if out.state == fc.COMPLETE:
            break
        veh.step(out.carrot, out.carrot_yaw, dt)
        t += dt
    planned = {"planned": [[x + hx, y + hy] for x, y in zip(s["x_map"], s["y_map"])], "home": [hx, hy],
               "serviced_cells": tr["serviced_cells"], "planned_length_m": track.total}
    return rows, planned, track.total


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", type=Path, default=CONFIG / "scenario.json")
    ap.add_argument("--ground-truth", type=Path, default=None)
    ap.add_argument("--run-id", default="offline")
    ap.add_argument("--runs-root", type=Path, default=REPO / "runs")
    ap.add_argument("--planner-bin", default=None)
    ap.add_argument("--rate-hz", type=float, default=20.0)
    args = ap.parse_args(argv)

    gt_path = args.ground_truth or args.scenario.with_name("ground_truth.json")
    scenario = json.loads(args.scenario.read_text())
    gt = json.loads(gt_path.read_text())
    png_path = args.scenario.with_name("belief.png")
    png = png_path.read_bytes() if png_path.is_file() else None
    run_dir = args.runs_root / args.run_id
    run_dir.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory() as tmp:
        exe = find_planner(args.planner_bin)
        res = subprocess.run([exe, "--scenario", str(args.scenario), "--out-dir", tmp],
                             capture_output=True, text=True)
        print(res.stdout.strip())
        if res.returncode != 0:
            print(res.stderr, file=sys.stderr)
            return 1
        rows_all, planned_all = {}, {}
        for a in scenario["team"]["agents"]:
            name = a["name"]
            track = Path(tmp) / f"{name}_track.json"
            rows, planned, total = fly_agent(name, track, scenario, rate_hz=args.rate_hz)
            rows_all[name], planned_all[name] = rows, planned
            out = run_dir / name
            out.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(track, out / "track.json")
            shutil.copyfile(Path(tmp) / "plan.json", out / "plan.json")
            shutil.copyfile(args.scenario, out / "scenario.json")
            r = write_run_outputs(out, scenario=scenario, ground_truth=gt, rows_by_agent={name: rows},
                                  planned_by_agent={name: planned}, title=f"MTL sortie — {name} (offline)",
                                  subtitle=f"run {args.run_id} · kinematic rehearsal · {len(rows)} samples",
                                  belief_png=png)
            xte = [r_["xte_m"] for r_ in rows if r_["state"] == "SEARCH"]
            rms = math.sqrt(sum(v * v for v in xte) / len(xte)) if xte else float("nan")
            s = r["summary"]
            print(f"  {name}: track {total:.0f} m flown in {rows[-1]['t']:.0f} s, cross-track RMS {rms:.2f} m, "
                  f"{s['targets_detected']} targets, covered {s['belief_mass_covered']:.0f} "
                  f"(planned {s['planned_belief_mass']:.0f})")
        shutil.copyfile(gt_path, run_dir / "ground_truth.json")
        if png:
            (run_dir / "belief.png").write_bytes(png)
    team = write_run_outputs(run_dir, scenario=scenario, ground_truth=gt, rows_by_agent=rows_all,
                             planned_by_agent=planned_all, title=f"MTL team search — {args.run_id} (offline)",
                             subtitle="kinematic rehearsal: mtl_search_plan -> follower_core -> point-mass "
                                      "vehicle + slew-limited gimbal -> Moon et al. scoring",
                             belief_png=png)
    s = team["summary"]
    print(f"TEAM: {s['targets_detected']}/{s['targets_total']} targets, mean time to discovery "
          f"{s['mean_time_to_discovery_s'] or float('nan'):.1f} s, covered {s['belief_mass_covered']:.0f} of "
          f"planned {s['planned_belief_mass']:.0f} ({100 * s.get('realized_over_planned_mass', 0):.1f} %), "
          f"{s['belief_mass_per_km']:.0f} mass/km -> {run_dir / 'report.html'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
