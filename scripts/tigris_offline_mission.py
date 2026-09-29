#!/usr/bin/env python3
"""Offline TIGRIS mission: planner CLI -> follower -> vehicle/gimbal model -> scoring.

    python3 scripts/tigris_offline_mission.py                          # stack scenario + stack TIGRIS params
    python3 scripts/tigris_offline_mission.py --run-id tigris-offline --set reward_mode=matched

A no-Isaac rehearsal of the TIGRIS baseline, the counterpart of scripts/mtl_offline_mission.py:

  * the whole receding-horizon sortie is planned by the ``tigris_search_plan`` CLI (the same
    core as ``tigris_search_planner_node``) with the parameters of
    stacks/tigris_search/config/tigris_search_planner.yaml (``--set`` overrides them). The CLI
    assumes the committed track is flown perfectly when it folds the "flown" looks in;
  * the final track is flown by ``mtl_trajectory_follower.follower_core`` at 20 Hz with the
    gimbal limits the planner publishes (LOCKED = body-fixed camera by default; the
    cross-track travel opens when mission.yaml ``gimbal_actuation.enabled`` makes it sweep),
    against the same point-mass PX4 stand-in and slew-limited gimbal as the MTL rehearsal;
  * it is scored by ``mtl_metrics_logger`` per robot, then ``scripts/analyze_tigris_run.py``
    writes the team outputs and both TIGRIS reward curves.

Outputs land in ``runs/<run_id>/`` like a real run. A kinematic rehearsal, not a flight test.
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

import yaml

REPO = Path(__file__).resolve().parents[1]
for sub in ("robot/ros_ws/src/local/controls/mtl_trajectory_follower",
            "robot/ros_ws/src/behavior/mtl_metrics_logger", "scripts"):
    sys.path.insert(0, str(REPO / sub))

from mtl_metrics_logger.analysis import planned_looks_from_track, write_run_outputs  # noqa: E402
from mtl_metrics_logger.detection import boresight_ground_point, footprint_radius  # noqa: E402
from mtl_trajectory_follower import follower_core as fc  # noqa: E402
from mtl_trajectory_follower.gimbal_math import slew_limit  # noqa: E402

import analyze_tigris_run  # noqa: E402
from mtl_offline_mission import Vehicle  # noqa: E402  (the same point-mass PX4 stand-in)

CONFIG = REPO / "stacks/tigris_search/config"
CLI_KEYS = ("reward_mode", "sampler", "planning_time_s", "initial_planning_time_s", "replan_period_s",
            "commit_margin_s", "extend_dist_m", "extend_radius_m", "prune_radius_m", "reward_step_m", "grid_res_m",
            "view_point_goal", "bounds_margin_m", "use_entropy", "rs", "rf", "initial_confidence", "budget_m",
            "camera_fov_deg", "camera_tilt_deg", "max_iterations", "seed")
LOCKED = 1e-6


def find_planner(explicit: str | None) -> str:
    cands = [explicit, os.environ.get("TIGRIS_SEARCH_PLAN_BIN"), shutil.which("tigris_search_plan"),
             str(REPO / "robot/ros_ws/install/tigris_search_planner/lib/tigris_search_planner/tigris_search_plan"),
             str(REPO / "robot/ros_ws/build/tigris_search_planner/tigris_search_plan")]
    for c in cands:
        if c and Path(c).is_file() and os.access(c, os.X_OK):
            return c
    raise SystemExit("tigris_search_plan not found: build tigris_search_planner (bws --packages-select "
                     "tigris_search_planner) or pass --planner-bin")


def params_from_yaml(path: Path) -> list[str]:
    if not path.is_file():
        return []
    raw = yaml.safe_load(path.read_text()) or {}
    prm = (raw.get("/**") or {}).get("ros__parameters") or {}
    out = []
    for k in CLI_KEYS:
        if k not in prm:
            continue
        v = prm[k]
        if k == "seed" and int(v) < 0:
            continue
        if k in ("budget_m", "camera_fov_deg") and float(v) <= 0:
            continue
        if k == "camera_tilt_deg" and float(v) < 0:
            continue
        out.append(f"{k}={str(v).lower() if isinstance(v, bool) else v}")
    if prm.get("receding") is False:
        out.append("__one_shot__")
    return out


def fly(name: str, tr: dict, rate_hz: float = 20.0, max_s: float = 900.0):
    s = tr["samples"]
    n = len(s["t"])
    track = fc.Track(x=s["x_map"], y=s["y_map"], z=s["z_map"], yaw=s["yaw_enu"], speed=[tr["speed_mps"]] * n,
                     bx=s["bx_map"], by=s["by_map"], bz=s["bz_map"], arc=s["arc"], t=s["t"], phi=s["gimbal_phi"])
    tg = tr.get("tigris") or {}
    locked = bool(tg.get("gimbal_locked", True))
    # the SearchPlan gimbal limits the node would publish (older track.json: from gimbal_locked)
    gmax = float(tg.get("gimbal_max_rad", LOCKED if locked else math.radians(80.0)))
    nudge = float(tg.get("pitch_nudge_max_rad", LOCKED if locked else math.radians(5.0)))
    cfg = fc.FollowerConfig(min_turn_radius_m=tr["min_turn_radius_m"], single_axis=True, tilt_rad=tr["tilt_rad"],
                            speed_mps=tr["speed_mps"], gimbal_max_rad=gmax, pitch_nudge_max_rad=nudge)
    fol = fc.TrackFollower(track, cfg)
    hx, hy, hz = tr["home_enu"]
    veh = Vehicle((0.0, 0.0, track.z[0]), track.yaw[0])
    gim = None
    slew = math.radians(120.0)
    dt = 1.0 / rate_hz
    fov = tr["fov_rad"]
    rows, t = [], 0.0
    fol.start(veh.p)
    while t < max_s:
        out = fol.step(veh.p, veh.yaw, dt)
        cmd = out.gimbal
        gim = cmd if gim is None else slew_limit(gim, cmd, slew * dt)
        xw, yw, zw = veh.p[0] + hx, veh.p[1] + hy, veh.p[2] + hz
        bore = boresight_ground_point((xw, yw, zw), gim[1], gim[2], 0.0)
        aim = (out.aim[0] + hx, out.aim[1] + hy)
        rows.append({
            "t": t, "agent": name, "state": out.state_name,
            "x_map": veh.p[0], "y_map": veh.p[1], "z_map": veh.p[2], "x_world": xw, "y_world": yw, "z_world": zw,
            "n": yw, "e": xw, "d": -zw, "yaw": veh.yaw, "vx": veh.v[0], "vy": veh.v[1], "vz": veh.v[2],
            "speed": math.hypot(veh.v[0], veh.v[1]),
            "cmd_roll": cmd[0], "cmd_pitch": cmd[1], "cmd_yaw": cmd[2],
            "meas_roll": gim[0], "meas_pitch": gim[1], "meas_yaw": gim[2], "gimbal_measured": 1,
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
               "serviced_cells": tr["serviced_cells"], "planned_length_m": track.total,
               "looks": planned_looks_from_track(tr)}
    return rows, planned, track.total


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", type=Path, default=CONFIG / "scenario.json")
    ap.add_argument("--params", type=Path, default=CONFIG / "tigris_search_planner.yaml")
    ap.add_argument("--ground-truth", type=Path, default=None)
    ap.add_argument("--agent", default="robot_1")
    ap.add_argument("--run-id", default="tigris-offline")
    ap.add_argument("--runs-root", type=Path, default=REPO / "runs")
    ap.add_argument("--planner-bin", default=None)
    ap.add_argument("--set", action="append", default=[], help="override a TIGRIS parameter, key=value")
    ap.add_argument("--one-shot", action="store_true")
    args = ap.parse_args(argv)

    gt_path = args.ground_truth or args.scenario.with_name("ground_truth.json")
    scenario = json.loads(args.scenario.read_text())
    gt = json.loads(gt_path.read_text())
    png_path = args.scenario.with_name("belief.png")
    png = png_path.read_bytes() if png_path.is_file() else None
    run_dir = args.runs_root / args.run_id
    out = run_dir / args.agent
    out.mkdir(parents=True, exist_ok=True)

    sets = params_from_yaml(args.params)
    one_shot = args.one_shot or "__one_shot__" in sets
    sets = [s for s in sets if s != "__one_shot__"] + list(args.set)
    exe = find_planner(args.planner_bin)
    with tempfile.TemporaryDirectory() as tmp:
        cmd = [exe, "--scenario", str(args.scenario), "--out-dir", tmp, "--agent", args.agent]
        for s in sets:
            cmd += ["--set", s]
        if one_shot:
            cmd.append("--one-shot")
        res = subprocess.run(cmd, capture_output=True, text=True)
        print(res.stdout.strip())
        if res.returncode != 0:
            print(res.stderr, file=sys.stderr)
            return 1
        for f in ("track.json", "plan.json", "tigris_replans.json"):
            shutil.copyfile(Path(tmp) / f, out / f)
    shutil.copyfile(args.scenario, out / "scenario.json")
    tr = json.loads((out / "track.json").read_text())
    rows, planned, total = fly(args.agent, tr)
    r = write_run_outputs(out, scenario=scenario, ground_truth=gt, rows_by_agent={args.agent: rows},
                          planned_by_agent={args.agent: planned}, title=f"TIGRIS sortie — {args.agent} (offline)",
                          subtitle=f"run {args.run_id} · kinematic rehearsal · {len(rows)} samples",
                          belief_png=png)
    shutil.copyfile(gt_path, run_dir / "ground_truth.json")
    if png:
        (run_dir / "belief.png").write_bytes(png)
    xte = [x["xte_m"] for x in rows if x["state"] == "SEARCH"]
    rms = math.sqrt(sum(v * v for v in xte) / len(xte)) if xte else float("nan")
    s = r["summary"]
    print(f"  {args.agent}: track {total:.0f} m flown in {rows[-1]['t']:.0f} s, cross-track RMS {rms:.2f} m, "
          f"{s['targets_detected']} targets, residual belief {s['residual_belief_mass']:.4f} "
          f"(planned {s['planned_residual_belief_mass']:.4f})")
    return analyze_tigris_run.main(["--run-dir", str(run_dir)])


if __name__ == "__main__":
    sys.exit(main())
