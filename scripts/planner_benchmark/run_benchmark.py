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


def run_one(job):
    sid, arm, sc_path, bins, res, keep_dir, timeout = job
    sc = json.loads(Path(sc_path).read_text())
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
        rec.update(residual=sco["residual"], flown_m=round(sco["flown_m"], 1), curve=sco["curve"],
                   budget_m=len(agents) * float(sc["team"]["max_flight_time_s"]) * float(sc["aircraft"]["speed_mps"]),
                   n_agents=len(agents), track=decimate(tracks[0]),
                   tracks=[decimate(t) for t in tracks] if len(tracks) > 1 else None)
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
    (a.bench / "arms.json").write_text(json.dumps({"arms": arms, "binaries": bins}, indent=1))
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
