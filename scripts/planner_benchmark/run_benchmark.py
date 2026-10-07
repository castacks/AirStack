#!/usr/bin/env python3
"""Run planner "arms" over a generated scenario set and score every plan the same way.

    python3 scripts/planner_benchmark/run_benchmark.py --bench bench/wide \\
        --arms scripts/planner_benchmark/arms/default.json --workers 8

For every (scenario, arm) it writes a per-arm scenario variant (the arm's overrides applied),
runs the arm's planner CLI (``mtl_search_plan`` or ``tigris_search_plan``, both ROS-free),
scores the resulting track with ``score.py`` (residual belief, the anytime curve) and appends
one JSON line to ``<bench>/results.jsonl``. Resumable: pairs already in the file are skipped,
so you can stop it and start it again, or add arms later.

Planner binaries are found in the ROS workspace (install/ or build/), on PATH, or via
--mtl-bin / --tigris-bin, or build them host-side with build_offline_tools.sh.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))
import score  # noqa: E402

TIGRIS_YAML = REPO / "stacks/tigris_search/config/tigris_search_planner.yaml"
TIGRIS_CLI_KEYS = ["reward_mode", "sampler", "planning_time_s", "initial_planning_time_s", "replan_period_s",
                   "commit_margin_s", "extend_dist_m", "extend_radius_m", "prune_radius_m", "reward_step_m",
                   "grid_res_m", "view_point_goal", "bounds_margin_m", "use_entropy", "rs", "rf",
                   "initial_confidence", "budget_m", "camera_fov_deg", "camera_tilt_deg", "max_iterations", "seed"]


def find_bin(explicit, name, pkg):
    cands = [explicit, shutil.which(name), str(HERE / "bin" / name),
             str(REPO / f"robot/ros_ws/install/{pkg}/lib/{pkg}/{name}"),
             str(REPO / f"robot/ros_ws/build/{pkg}/{name}")]
    for c in cands:
        if c and Path(c).is_file() and os.access(c, os.X_OK):
            return c
    return None


def tigris_stack_sets(path: Path) -> list[str]:
    """The TIGRIS tuning the stack flies (stacks/tigris_search/config/tigris_search_planner.yaml)."""
    try:
        import yaml
        raw = yaml.safe_load(path.read_text()) or {}
    except Exception:
        return []
    prm = (raw.get("/**") or {}).get("ros__parameters") or {}
    out = {}
    for k in TIGRIS_CLI_KEYS:
        if k not in prm:
            continue
        v = prm[k]
        if k == "seed" and int(v) < 0:
            continue
        if k in ("budget_m", "camera_fov_deg") and float(v) <= 0:
            continue
        if k == "camera_tilt_deg" and float(v) < 0:
            continue
        out[k] = str(v).lower() if isinstance(v, bool) else str(v)
    return out


def set_path(d: dict, dotted: str, v):
    ks = dotted.split(".")
    for k in ks[:-1]:
        d = d.setdefault(k, {})
    if v is None:
        d.pop(ks[-1], None)
    else:
        d[ks[-1]] = v


def decimate(track: dict, step_m: float = 25.0):
    """A light copy of the track (aircraft + boresight, NED) for the report's maps."""
    s = track["samples"]
    n, e, sn, se = s["n"], s["e"], s["sensor_n"], s["sensor_e"]
    out, last = [], None
    for k in range(len(n)):
        if last is None or abs(n[k] - last[0]) + abs(e[k] - last[1]) >= step_m or k == len(n) - 1:
            out.append([round(n[k], 1), round(e[k], 1), round(sn[k], 1), round(se[k], 1)])
            last = (n[k], e[k])
    return out


def sweep_check(tracks: list[dict]) -> dict:
    """The planned 1-DOF sweep against the common gimbal (80 deg travel, 120 deg/s): max |phi| off
    the mount axis, max |roll + phi| (level frame) and the max planned slew rate, over all agents."""
    out = {"phi_max_deg": 0.0, "cross_max_deg": 0.0, "rate_max_deg_s": 0.0}
    for t in tracks:
        s = t["samples"]
        phi = [float(v) for v in s.get("gimbal_phi") or []]
        roll = [float(v) for v in s.get("roll") or [0.0] * len(phi)]
        dt = float(t.get("dt_s", 0.1))
        if not phi:
            continue
        out["phi_max_deg"] = max(out["phi_max_deg"], math.degrees(max(abs(v) for v in phi)))
        out["cross_max_deg"] = max(out["cross_max_deg"], math.degrees(max(abs(a + b) for a, b in zip(phi, roll))))
        if len(phi) > 1:
            out["rate_max_deg_s"] = max(out["rate_max_deg_s"], math.degrees(
                max(abs(phi[k + 1] - phi[k]) for k in range(len(phi) - 1)) / dt))
    out = {k: round(v, 3) for k, v in out.items()}
    out["within_limits"] = out["phi_max_deg"] <= score.HW["gimbal_max_deg"] + 1e-6 and \
        out["rate_max_deg_s"] <= score.HW["slew_rate_deg_s"] + 1e-6
    return out


def curve_meta(plan: dict) -> dict:
    """The curve planner's own diagnostics (plan.json), per agent: sweep, optimiser, fast residual."""
    keys = ["sweep_amplitude_deg", "sweep_freq_hz", "sweep_peak_rate_deg_s", "gimbal_max_deg", "roll_max_deg",
            "slant_range_max_m", "swath_half_width_m", "max_curvature", "optimizer_exit", "optimizer_iters",
            "init_strategy", "fast_objective", "flown_length_m", "budget_used_frac", "info_fraction", "altitude_m"]
    mc = (plan.get("meta") or {}).get("curve") or {}
    return {"fast_residual": mc.get("fast_residual"), "fast_grid_step_m": mc.get("fast_grid_step_m"),
            "realloc_accepted": mc.get("realloc_accepted"),
            "agents": [{k: (a.get("diagnostics") or {}).get(k) for k in keys} for a in plan.get("agents", [])]}


def apply_arm(sc: dict, arm: dict) -> dict:
    """The scenario an arm plans: its ``scenario_overrides`` applied (dotted keys), and a TIGRIS
    sweep amplitude of "detection" resolved to degrees. Shared by run_one and export_to_sim.py,
    so a scenario flown in Isaac is exactly the one the benchmark planned."""
    for k, v in (arm.get("scenario_overrides") or {}).items():
        set_path(sc, k, copy.deepcopy(v))
    ga = (sc.get("airstack") or {}).get("gimbal_actuation") or {}
    if ga.get("sweep_amplitude_deg") == "detection":
        # sweep out to the cross-track angle where the boresight slant reaches 0.97 beta at this
        # altitude (what MTL's scheduler may aim at), capped below the gimbal roll limit
        h = float(sc["aircraft"]["altitude_m"])
        tilt = math.radians(float(sc["sensor"]["tilt_deg"]))
        S = 0.97 * float(sc["sensor"]["detection"]["beta"])
        c = h / (S * math.cos(tilt))
        amp = math.degrees(math.acos(min(1.0, c))) if c < 1 else 5.0
        ga["sweep_amplitude_deg"] = round(max(5.0, min(float(ga.get("max_amplitude_deg", 70.0)), amp)), 2)
        ga.pop("max_amplitude_deg", None)
    return sc


def run_one(job):
    sid, arm, sc_path, bins, res, keep_dir, timeout = job
    sc = apply_arm(json.loads(Path(sc_path).read_text()), arm)
    rec = {"id": sid, "arm": arm["name"], "planner": arm["planner"]}
    with tempfile.TemporaryDirectory() as tmp:
        scf = Path(tmp) / "scenario.json"
        scf.write_text(json.dumps(sc))
        out = Path(tmp) / "out"
        out.mkdir()
        agents = [a["name"] for a in sc["team"]["agents"]]
        cmds = []  # (command, track file) per planner call
        if arm["planner"] == "mtl":
            # MTL plans the whole team at once (its task allocation is part of the method)
            cmds.append(([bins["mtl"], "--scenario", str(scf), "--out-dir", str(out), "--no-alt"],
                         [out / f"{n}_track.json" for n in agents]))
        else:
            # TIGRIS is single-agent: with a team every agent runs its own TIGRIS on the full
            # prior, uncoordinated (the natural multi-agent baseline for a single-agent planner)
            sets = dict(bins.get("tigris_stack") or {}) if arm.get("use_stack_params", True) else {}
            sets.update({k: str(v).lower() if isinstance(v, bool) else str(v)
                         for k, v in (arm.get("tigris_set") or {}).items()})
            for n in agents:
                od = out / n
                od.mkdir()
                cmd = [bins["tigris"], "--scenario", str(scf), "--out-dir", str(od), "--agent", n]
                for k, v in sets.items():
                    cmd += ["--set", f"{k}={v}"]
                if arm.get("one_shot"):
                    cmd.append("--one-shot")
                cmds.append((cmd, [od / "track.json"]))
        t0 = time.time()
        track_files = []
        for cmd, tfs in cmds:
            try:
                p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
            except subprocess.TimeoutExpired:
                rec.update(error=f"timeout after {timeout} s")
                return rec
            if p.returncode != 0 or not all(t.is_file() for t in tfs):
                rec.update(error=(p.stderr or p.stdout)[-800:])
                return rec
            track_files += tfs
        rec["plan_s"] = round(time.time() - t0, 2)
        tracks = [json.loads(t.read_text()) for t in track_files]
        sco = score.residual(sc, tracks, res)
        # the same plan flown through the same slew-limited gimbal for every planner
        hw = score.residual(sc, tracks, res, hardware={})
        rec.update(residual_hw=hw["residual"], curve_hw=hw["curve"])
        if arm["planner"] == "mtl":  # MTL's schedule without the +-5 deg airframe pitch nudge
            rec["residual_hw_nonudge"] = score.residual(sc, tracks, res, hardware={"pitch_nudge_max_deg": 0.0})["residual"]
        curve_arm = arm["planner"] == "mtl" and (sc.get("planner") or {}).get("type") == "curve"
        if curve_arm:
            # secondary score: the follower's open_loop law (mission.yaml follower.gimbal_law), which
            # replays the planned cross-track angle instead of aiming; residual_hw stays the headline
            ol = score.residual(sc, tracks, res, hardware={"gimbal_law": "open_loop"})
            rec.update(residual_hw_openloop=ol["residual"], curve_hw_openloop=ol["curve"])
            rec["sweep_check"] = sweep_check(tracks)
        rec.update(residual=sco["residual"], flown_m=round(sco["flown_m"], 1), curve=sco["curve"],
                   budget_m=len(agents) * float(sc["team"]["max_flight_time_s"]) * float(sc["aircraft"]["speed_mps"]),
                   n_agents=len(agents), track=decimate(tracks[0]),
                   tracks=[decimate(t) for t in tracks] if len(tracks) > 1 else None)
        if curve_arm:
            try:
                plan = json.loads((out / "plan.json").read_text())
                rec["curve_meta"] = curve_meta(plan)
            except Exception as ex:  # metadata only; the scores above do not depend on it
                rec["curve_meta"] = {"error": str(ex)[:200]}
        if arm["planner"] == "mtl":
            try:
                meta = json.loads((out / "plan.json").read_text()).get("meta", {})
                ia = meta.get("info_aware") or {}
                if ia:
                    rec["info_aware"] = {"chosen": ia.get("chosen"), "detected": ia.get("chosen_detected"),
                                         "baseline_detected": ia.get("baseline_detected"),
                                         "candidates": len(ia.get("candidates", []))}
            except Exception:
                pass
        if keep_dir:
            dst = Path(keep_dir) / sid / arm["name"]
            dst.mkdir(parents=True, exist_ok=True)
            for n, t in zip(agents, track_files):
                shutil.copyfile(t, dst / f"{n}_track.json")
    return rec


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bench", type=Path, required=True, help="output dir of gen_scenarios.py")
    ap.add_argument("--arms", type=Path, default=HERE / "arms/default.json")
    ap.add_argument("--only-arms", default=None, help="comma-separated subset of arm names")
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--res", type=float, default=10.0, help="scoring raster [m] (default 10)")
    ap.add_argument("--timeout", type=float, default=3600.0, help="per-plan timeout [s]")
    ap.add_argument("--limit", type=int, default=None, help="first N scenarios only")
    ap.add_argument("--ids", default=None, help="comma-separated scenario ids only (default: all)")
    ap.add_argument("--keep-tracks", action="store_true", help="also keep full track.json files")
    ap.add_argument("--mtl-bin", default=None)
    ap.add_argument("--tigris-bin", default=None)
    ap.add_argument("--tigris-params", type=Path, default=TIGRIS_YAML)
    a = ap.parse_args(argv)

    arms = json.loads(a.arms.read_text())["arms"]
    if a.only_arms:
        keep = set(a.only_arms.split(","))
        arms = [x for x in arms if x["name"] in keep]
    idx = json.loads((a.bench / "index.json").read_text())["scenarios"]
    if a.limit:
        idx = idx[:a.limit]
    if a.ids:
        want = set(a.ids.split(","))
        idx = [r for r in idx if r["id"] in want]
    bins = {"mtl": find_bin(a.mtl_bin, "mtl_search_plan", "mtl_search_planner"),
            "tigris": find_bin(a.tigris_bin, "tigris_search_plan", "tigris_search_planner"),
            "tigris_stack": tigris_stack_sets(a.tigris_params)}
    need = {x["planner"] for x in arms}
    for p in need:
        if not bins[p]:
            raise SystemExit(f"no {p} planner binary: pass --{p}-bin, build the ROS package, or run "
                             "scripts/planner_benchmark/build_offline_tools.sh")
    res_path = a.bench / "results.jsonl"
    done = set()
    if res_path.is_file():
        for line in res_path.read_text().splitlines():
            try:
                r = json.loads(line)
                if "error" not in r:
                    done.add((r["id"], r["arm"]))
            except Exception:
                pass
    # arms.json records every arm ever run into this bench: arms from earlier runs are kept (an arm
    # of the same name is replaced by this run's definition)
    prev = []
    try:
        prev = json.loads((a.bench / "arms.json").read_text()).get("arms", [])
    except Exception:
        pass
    names = {x["name"] for x in arms}
    (a.bench / "arms.json").write_text(json.dumps(
        {"arms": [x for x in prev if x["name"] not in names] + arms, "binaries": bins}, indent=1))
    jobs = [(r["id"], arm, str(a.bench / "scenarios" / f"{r['id']}.json"), bins, a.res,
             str(a.bench / "tracks") if a.keep_tracks else None, a.timeout)
            for r in idx for arm in arms if (r["id"], arm["name"]) not in done]
    # interleave arms so partial results are balanced
    jobs.sort(key=lambda j: (idx.index(next(x for x in idx if x["id"] == j[0])), j[1]["name"]))
    print(f"[run_benchmark] {len(jobs)} plans to run ({len(done)} already done), {a.workers} workers; "
          f"mtl={bins['mtl']} tigris={bins['tigris']}")
    t0, n = time.time(), 0
    with ProcessPoolExecutor(max_workers=a.workers) as ex, res_path.open("a") as f:
        futs = [ex.submit(run_one, j) for j in jobs]
        for fu in as_completed(futs):
            r = fu.result()
            f.write(json.dumps(r) + "\n")
            f.flush()
            n += 1
            el = time.time() - t0
            msg = f"{r['id']:>28} {r['arm']:<18} " + (f"residual {r['residual']:.4f} in {r['plan_s']:.0f} s"
                                                     if "residual" in r else f"ERROR {r['error'][:80]}")
            print(f"[{n}/{len(jobs)} {el / 60:.1f} min, eta {el / n * (len(jobs) - n) / 60:.0f} min] {msg}",
                  flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
