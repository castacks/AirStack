#!/usr/bin/env python3
"""Fly benchmark scenarios (or new ones) in Isaac Sim with the curve planner or TIGRIS.

    # 1. pick a scenario: one of the 160 benchmark scenarios ...
    python3 scripts/planner_benchmark/sim_scenario.py list --budget-s 700 --home near_center
    # ... or a new one from family / budget / altitude / home / family parameters
    python3 scripts/planner_benchmark/sim_scenario.py new --family gaussian_blobs --budget-s 700 \\
        --altitude-m 300 --home near_center --param n_peaks=8 --param 'sigma=[150,300]' --seed 7 --offline
    # 2. fly it (exports the bundle, airstack up, sortie + rosbag + report, airstack down)
    python3 scripts/planner_benchmark/sim_scenario.py fly --id s0136_gaussian_blobs --planner curve
    python3 scripts/planner_benchmark/sim_scenario.py fly --id s0136_gaussian_blobs --planner tigris
    # 3. compare the sim runs with each other and with the offline benchmark
    python3 scripts/planner_benchmark/sim_scenario.py compare --id s0136_gaussian_blobs
    # 4. put the stacks back as they were (before committing)
    python3 scripts/planner_benchmark/sim_scenario.py restore

Subcommands
-----------
list     Benchmark (and custom) scenarios with their factors and offline scores, filterable.
new      Generate a scenario from a prior family with your parameters, in the benchmark's world
         (copied from a benchmark scenario: 5 km area, 900 m detection, sensor, mapping). Written
         to <bench>/custom/ (a bench of its own: scenarios/, index.json); --offline also plans and
         scores it with both arms offline (run_benchmark.py), for the sim-vs-offline comparison.
export   Write the scenario into the Isaac stack(s) with the arm's overrides applied by the SAME
         code the benchmark used (run_benchmark.apply_arm): stacks/{mtl,tigris}_search/config/
         {scenario.json, ground_truth.json, belief.png} and robot_1's spawn in config/fleets/
         {mtl,tigris}_search_fleet.yaml (= the scenario's home). The first export backs the
         originals up to <bench>/sim/_stack_backup/.
fly      export, then `airstack down` (any previous sim), `airstack up --sim isaac --fleet F
         --stack S --play --wait`, the stack's start script (takeoff to the scenario altitude,
         search_mission, MCAP rosbag, team report, Foxglove file: everything under
         runs/<run_id>/ exactly as before), then `airstack down` (--keep-up to leave it running).
         The run is recorded in <bench>/sim/<id>/runs.json. --print-only shows the commands.
compare  Every recorded sim run of a scenario: flown residual (runs/<run_id>/detection.json, the
         logger's score of the real flight), planned residual, and the offline benchmark's numbers.
restore  Copy the original stack bundles and fleet files back.

Planners: --planner curve (arm mtl_curve, stack mtl_search) or tigris (arm tigris_det_60dps,
stack tigris_search); other arms with --mtl-arm / --tigris-arm. Both get the same prior, cells,
budget, altitude, home, sensor model and ground-truth targets (sampled from the prior with the
`targets:` block of stacks/mtl_search/config/mission.yaml, seed = scenario seed + 1).

Live flight differs from the offline benchmark by design:
* TIGRIS replans every 5 s with a 5 s wall-time solve (stacks/tigris_search/config/
  tigris_search_planner.yaml), not the benchmark's 150 iterations per replan (a 5 s solve ran
  ~500-700 iterations on the robot; the score was insensitive to the count).
* Gimbal law: TIGRIS plans carry none, so the follower aims (aim_point). The curve gets the same
  law by default (--gimbal-law aim_point = the benchmark's headline scoring); mission.yaml's own
  setting for the curve is open_loop.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO / "robot/ros_ws/src/global/planners/mtl_search_planner"))
import run_benchmark  # noqa: E402
from gen_scenarios import _py, descriptors, extract_cells  # noqa: E402
from priors import FAMILIES, make_prior, rasterize, sample_family_params  # noqa: E402

PLANNERS = {
    "curve": {"key": "mtl", "arm": "mtl_curve", "stack": "mtl_search", "fleet": "mtl_search_fleet",
              "start": "scripts/mtl_start_mission.sh"},
    "tigris": {"key": "tigris", "arm": "tigris_det_60dps", "stack": "tigris_search", "fleet": "tigris_search_fleet",
               "start": "scripts/tigris_start_mission.sh"},
}
STACK_DIR = {"mtl": REPO / "stacks/mtl_search/config", "tigris": REPO / "stacks/tigris_search/config"}
FLEET_FILE = {"mtl": REPO / "config/fleets/mtl_search_fleet.yaml", "tigris": REPO / "config/fleets/tigris_search_fleet.yaml"}
BUNDLE = ("scenario.json", "ground_truth.json", "belief.png")
MISSION_YAML = REPO / "stacks/mtl_search/config/mission.yaml"
HOMES = {"near_center": [-170.0, -170.0], "edge": [-2300.0, 0.0], "corner": [-2300.0, -2300.0]}  # specs/wide900.json


# --------------------------------------------------------------------------- scenarios
def custom_dir(bench: Path) -> Path:
    return bench / "custom"


def find_scenario(bench: Path, sid: str) -> tuple[Path, Path]:
    """(scenario file, the bench it belongs to: <bench> or <bench>/custom)."""
    for b in (bench, custom_dir(bench)):
        p = b / "scenarios" / f"{sid}.json"
        if p.is_file():
            return p, b
    if not (bench / "scenarios").is_dir() and (bench / "scenarios.zip").is_file():
        raise SystemExit(f"unzip {bench / 'scenarios.zip'} into {bench / 'scenarios'} first "
                         f"(cd {bench} && mkdir -p scenarios && cd scenarios && unzip ../scenarios.zip)")
    raise SystemExit(f"scenario {sid} not found in {bench}/scenarios or {custom_dir(bench)}/scenarios")


def load_index(b: Path) -> list[dict]:
    p = b / "index.json"
    return json.loads(p.read_text())["scenarios"] if p.is_file() else []


def load_results(b: Path) -> dict:
    out: dict = {}
    p = b / "results.jsonl"
    if p.is_file():
        for line in p.read_text().splitlines():
            if line.strip():
                r = json.loads(line)
                if "error" not in r:
                    out.setdefault(r["id"], {})[r["arm"]] = r
    return out


def load_arms(path: Path) -> dict:
    return {x["name"]: x for x in json.loads(path.read_text())["arms"]}


def cmd_list(a) -> int:
    rows = []
    for b in (a.bench, custom_dir(a.bench)):
        res = load_results(b)
        for r in load_index(b):
            if a.family and r["family"] != a.family:
                continue
            if a.budget_s and float(r["budget_s"]) != a.budget_s:
                continue
            if a.home and r["home"] != a.home:
                continue
            if a.altitude_m and float(r["altitude_m"]) != a.altitude_m:
                continue
            c = res.get(r["id"], {}).get(a.mtl_arm, {}).get("residual_hw")
            t = res.get(r["id"], {}).get(a.tigris_arm, {}).get("residual_hw")
            rows.append((r, c, t, b is not a.bench))
    if a.sort == "adv":
        rows.sort(key=lambda x: -((x[2] or 0) - (x[1] or 0)))
    f = lambda v: "   -  " if v is None else f"{v:.3f}"  # noqa: E731
    print(f"{'id':28s} {'family':18s} {'budget':>6s} {'alt':>4s} {'home':11s}  curve  tigris   adv")
    for r, c, t, custom in rows[:a.limit]:
        adv = "" if c is None or t is None else f"{t - c:+.3f}"
        print(f"{r['id']:28s} {r['family']:18s} {r['budget_s']:6.0f} {r['altitude_m']:4.0f} {str(r['home']):11s} "
              f"{f(c)} {f(t)} {adv:>6s}{'  (custom)' if custom else ''}")
    print(f"[{len(rows)} scenarios; residual_hw offline, lower is better; adv = tigris - curve]")
    return 0


def cmd_new(a) -> int:
    if a.family not in FAMILIES:
        raise SystemExit(f"--family must be one of {', '.join(FAMILIES)}")
    idx = load_index(a.bench)
    if not idx:
        raise SystemExit(f"{a.bench}/index.json not found")
    template = json.loads(find_scenario(a.bench, idx[0]["id"])[0].read_text())
    res = float(json.loads((a.bench / "index.json").read_text()).get("res_m", 5.0))
    size = float(template["mission"]["area"]["size_m"])
    cn0, ce0 = template["mission"]["area"].get("center_ned", [0.0, 0.0])
    area = (cn0 - size / 2, cn0 + size / 2, ce0 - size / 2, ce0 + size / 2)
    if a.home in HOMES:
        home_name, home = a.home, HOMES[a.home]
    else:
        try:
            home = [float(v) for v in a.home.split(",")]
            assert len(home) == 2
        except Exception:
            raise SystemExit("--home is near_center | edge | corner | N,E (mission NED metres)")
        if not (area[0] <= home[0] <= area[1] and area[2] <= home[1] <= area[3]):
            raise SystemExit(f"--home {home} is outside the {size:.0f} m area")
        home_name = f"{home[0]:.0f},{home[1]:.0f}"
    seed = a.seed if a.seed is not None else int(np.random.default_rng().integers(1 << 31))
    prng = np.random.default_rng(seed)
    params = dict(sample_family_params(a.family, prng))  # unspecified parameters: sampled from the seed
    for kv in a.param or []:
        k, _, v = kv.partition("=")
        if not _:
            raise SystemExit(f"--param {kv!r}: use key=value (value as JSON, e.g. sigma=[150,300])")
        try:
            params[k] = json.loads(v)
        except json.JSONDecodeError:
            params[k] = v
    try:
        bumps, floor, used = make_prior(a.family, prng, area, home=home, **params)
    except TypeError as ex:
        fn = FAMILIES[a.family][0]
        raise SystemExit(f"{a.family}: {ex}\n  parameters of {fn.__name__}: "
                         f"{', '.join(fn.__code__.co_varnames[2:fn.__code__.co_argcount])}")
    cap = float(template["airstack"]["belief"].get("belief_cap", 0.85))
    na, ea, V = rasterize(bumps, floor, area, res=res, cap=cap)
    cell_m = float(template["mapping"]["target_cell_size_m"])
    centers, masses, blk = extract_cells(na, ea, V, cell_m, float(template["mapping"]["minimum_belief_mass"]))
    if not centers:
        raise SystemExit("no cell holds enough belief mass; widen the prior")
    cd = custom_dir(a.bench)
    (cd / "scenarios").mkdir(parents=True, exist_ok=True)
    cidx = load_index(cd)
    sid = a.name or f"c{len(cidx):04d}_{a.family}"
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", sid):
        raise SystemExit("--name: letters, digits, _ . - only")
    sc = json.loads(json.dumps(template))
    sc["mission"]["name"], sc["mission"]["seed"] = sid, seed
    sc["aircraft"]["altitude_m"] = float(a.altitude_m)
    sc["team"]["max_flight_time_s"], sc["team"]["max_flight_distance_m"] = float(a.budget_s), None
    sc["team"]["agents"] = [{"name": "robot_1", "start_ned": home, "home_ned": home, "altitude_offset_m": 0.0}]
    sc["cells"] = {"centers": centers, "mass": masses, "total_map_mass": 1.0}
    sc["airstack"]["belief"] = {"bumps": [{k: round(v, 6) for k, v in b.items()} for b in bumps], "belief_cap": cap,
                                "base_uncertainty": round(floor, 9), "normalised": True, "peak": float(V.max()),
                                "texture": "belief.png"}
    (cd / "scenarios" / f"{sid}.json").write_text(json.dumps(sc, default=_py))
    budget_m = float(a.budget_s) * float(sc["aircraft"]["speed_mps"])
    rec = {"id": sid, "family": a.family, "params": {k: (list(v) if isinstance(v, tuple) else v)
                                                    for k, v in used.items() if k != "home"},
           "home": home_name, "home_ned": home, "budget_s": float(a.budget_s), "budget_m": budget_m,
           "altitude_m": float(a.altitude_m), "team_size": 1, "seed": seed, "n_bumps": len(bumps), "floor": floor,
           "n_cells": len(centers), "cells_mass": round(sum(masses), 4),
           **descriptors(na, ea, V, blk, home, budget_m, cell_m)}
    cidx = [r for r in cidx if r["id"] != sid] + [rec]
    (cd / "index.json").write_text(json.dumps({"spec": {"custom": True, "template": idx[0]["id"]}, "res_m": res,
                                              "scenarios": cidx}, indent=1, default=_py))
    print(f"[sim_scenario] new scenario {sid} -> {cd / 'scenarios' / (sid + '.json')}")
    print(f"  family {a.family}, seed {seed}, params {json.dumps(rec['params'], default=_py)}")
    print(f"  budget {a.budget_s:.0f} s ({budget_m:.0f} m), altitude {a.altitude_m:.0f} m, home {home_name} {home}, "
          f"{len(centers)} cells, {len(bumps)} bumps")
    if a.offline:
        names = ",".join([a.mtl_arm, a.tigris_arm])
        print(f"[sim_scenario] planning it offline with {names} (the benchmark's runner and scorer) ...")
        run_benchmark.main(["--bench", str(cd), "--arms", str(a.arms), "--only-arms", names, "--workers", "2",
                            "--timeout", "3600"])
        r = load_results(cd).get(sid, {})
        for arm in (a.mtl_arm, a.tigris_arm):
            if arm in r:
                print(f"  offline {arm:18s} residual_hw {r[arm]['residual_hw']:.4f} (planned {r[arm]['residual']:.4f}, "
                      f"{r[arm]['plan_s']:.0f} s to plan)")
    return 0


# --------------------------------------------------------------------------- export
def backup_dir(bench: Path) -> Path:
    return bench / "sim" / "_stack_backup"


def backup(bench: Path) -> None:
    bk = backup_dir(bench)
    if (bk / "DONE").is_file():
        return  # keep the ORIGINAL files of the first export, never a previous export
    for key, d in STACK_DIR.items():
        (bk / key).mkdir(parents=True, exist_ok=True)
        for f in BUNDLE:
            if (d / f).is_file():
                shutil.copy2(d / f, bk / key / f)
        shutil.copy2(FLEET_FILE[key], bk / key / FLEET_FILE[key].name)
    (bk / "DONE").write_text("original stack bundles + fleet files, before sim_scenario.py export\n")
    print(f"[sim_scenario] backed up the stack bundles and fleet files to {bk}")


def cmd_restore(a) -> int:
    bk = backup_dir(a.bench)
    if not (bk / "DONE").is_file():
        print(f"[sim_scenario] nothing to restore ({bk} holds no backup)")
        return 1
    for key, d in STACK_DIR.items():
        for f in BUNDLE:
            if (bk / key / f).is_file():
                shutil.copy2(bk / key / f, d / f)
        shutil.copy2(bk / key / FLEET_FILE[key].name, FLEET_FILE[key])
    shutil.rmtree(bk)
    print("[sim_scenario] restored the stacks/{mtl,tigris}_search/config bundles and both fleet files")
    return 0


def set_spawn(fleet: Path, n: float, e: float) -> None:
    """robot_1's spawn = world ENU [x = e, y = n, z]; the fleet must hold exactly one robot."""
    txt = fleet.read_text()
    active = re.findall(r"^  (robot_\d+):\s*$", txt, flags=re.M)
    if active != ["robot_1"]:
        raise SystemExit(f"{fleet}: expected exactly one active robot (robot_1), found {active}; "
                         "comment the others out - the benchmark is single-agent")
    pat = re.compile(r"(^  robot_1:\s*\n\s+spawn:\s*)\[[^\]]*\](.*)$", flags=re.M)
    if not pat.search(txt):
        raise SystemExit(f"{fleet}: could not find robot_1's spawn line")
    fleet.write_text(pat.sub(lambda m: f"{m.group(1)}[{e:.1f}, {n:.1f}, 0.07]   # mission NED home "
                                       f"[{n:.1f}, {e:.1f}] (sim_scenario.py)", txt, count=1))


def ground_truth(sc: dict, res: float = 5.0):
    """Targets sampled from the scenario's prior (mtl_generate_scenario's sampler and seed rule) and
    the grid the ground texture is painted from."""
    import yaml
    from mtl_search_planner.scenario import GROUND_TRUTH_SCHEMA, BeliefGrid, sample_targets
    mission = (yaml.safe_load(MISSION_YAML.read_text()) or {}).get("mission", {})
    ar = sc["mission"]["area"]
    size = float(ar["size_m"])
    cn, ce = ar.get("center_ned", [0.0, 0.0])
    area = (cn - size / 2, cn + size / 2, ce - size / 2, ce + size / 2)
    bel = sc["airstack"]["belief"]
    na, ea, V = rasterize(bel["bumps"], float(bel.get("base_uncertainty", 0.0)), area, res=res,
                          cap=float(bel.get("belief_cap", 0.85)))
    grid = BeliefGrid(V.tolist(), [float(x) for x in na], [float(x) for x in ea], res)
    spec = dict(mission.get("targets") or {})
    targets = sample_targets(grid, spec, int(sc["mission"]["seed"]) + 1)
    gt = {"schema": GROUND_TRUTH_SCHEMA, "mission": sc["mission"]["name"], "seed": int(sc["mission"]["seed"]),
          "frame": "mission NED; z = 0 on the ground plane",
          "note": "GROUND TRUTH ONLY - never handed to a planner. Sampled by sim_scenario.py.",
          "render": dict(spec.get("render") or {}), "targets": targets}
    return gt, grid


def export(a, planners: list[str]) -> dict:
    from mtl_search_planner.scenario import write_scenario_bundle
    sc_path, sbench = find_scenario(a.bench, a.id)
    arms = load_arms(a.arms)
    arm_of = {"curve": a.mtl_arm, "tigris": a.tigris_arm}
    for p in planners:
        if arm_of[p] not in arms:
            raise SystemExit(f"arm {arm_of[p]} not in {a.arms}")
    base = json.loads(sc_path.read_text())
    if len(base["team"]["agents"]) != 1:
        raise SystemExit(f"{a.id} has {len(base['team']['agents'])} agents; only single-agent scenarios are supported")
    backup(a.bench)
    gt, grid = ground_truth(base)
    home_n, home_e = (float(v) for v in base["team"]["agents"][0]["home_ned"])
    alt = float(base["aircraft"]["altitude_m"])
    md = a.bench / "sim" / a.id
    md.mkdir(parents=True, exist_ok=True)
    man_p = md / "export.json"
    man = json.loads(man_p.read_text()) if man_p.is_file() else {}
    man.update({"id": a.id, "scenario": str(sc_path.relative_to(REPO)) if sc_path.is_relative_to(REPO) else str(sc_path),
                "home_ned": [home_n, home_e], "spawn_enu": [home_e, home_n, 0.07], "altitude_m": alt,
                "budget_s": float(base["team"]["max_flight_time_s"]), "speed_mps": float(base["aircraft"]["speed_mps"]),
                "targets": len(gt["targets"])})
    man.setdefault("planners", {})
    offline = load_results(sbench).get(a.id, {})
    for p in planners:
        cfg = PLANNERS[p]
        arm = arm_of[p]
        sc = run_benchmark.apply_arm(json.loads(sc_path.read_text()), arms[arm])
        sc["mission"]["name"] = f"{a.id}__{arm}"
        sc["airstack"].setdefault("flight", {})["takeoff_altitude_m"] = alt
        sc["airstack"]["provenance"] = {"generator": "scripts/planner_benchmark/sim_scenario.py",
                                        "scenario": man["scenario"], "arm": arm, "arms_file": a.arms.name}
        if cfg["key"] == "mtl":
            sc["airstack"].pop("gimbal_actuation", None)
            sc["airstack"]["follower"] = {"gimbal_law": a.gimbal_law}
            if (sc.get("planner") or {}).get("type") == "curve":
                sc["planner"]["compare_orienteering"] = bool(a.compare_orienteering)
        write_scenario_bundle(STACK_DIR[cfg["key"]], sc, dict(gt, mission=sc["mission"]["name"]), grid)
        set_spawn(FLEET_FILE[cfg["key"]], home_n, home_e)
        off = offline.get(arm) or {}
        man["planners"][p] = {"arm": arm, "stack": cfg["stack"], "fleet": cfg["fleet"],
                              "gimbal_law": a.gimbal_law if cfg["key"] == "mtl" else "aim_point (TIGRIS plans carry none)",
                              "sweep": sc["airstack"].get("gimbal_actuation") if cfg["key"] == "tigris" else
                              {"sweep_freq_hz": (sc.get("curve") or {}).get("sweep_freq_hz")},
                              "offline": {k: off.get(k) for k in ("residual", "residual_hw", "residual_hw_openloop",
                                                                  "flown_m", "budget_m", "plan_s")} if off else None}
        print(f"[sim_scenario] {a.id} -> {STACK_DIR[cfg['key']].relative_to(REPO)} ({arm}); robot_1 spawn ENU "
              f"[{home_e:.0f}, {home_n:.0f}] in {FLEET_FILE[cfg['key']].relative_to(REPO)}")
        if off:
            print(f"  offline benchmark: residual_hw {off['residual_hw']:.4f} (planned {off['residual']:.4f})")
    man_p.write_text(json.dumps(man, indent=1))
    print(f"  altitude {alt:.0f} m, budget {man['budget_s']:.0f} s: each sortie is ~{alt / 2 / 60:.1f} min of climb "
          f"(2 m/s) + {man['budget_s'] / 60:.1f} min of search in real time")
    return man


def cmd_export(a) -> int:
    export(a, ["curve", "tigris"] if a.planner == "both" else [a.planner])
    return 0


# --------------------------------------------------------------------------- fly / compare
def sh(cmd: list[str], print_only: bool, check: bool = True) -> int:
    print("$ " + " ".join(cmd), flush=True)
    if print_only:
        return 0
    rc = subprocess.call(cmd, cwd=REPO)
    if check and rc != 0:
        raise SystemExit(f"command failed ({rc}): {' '.join(cmd)}")
    return rc


def run_summary(run_id: str) -> dict:
    p = REPO / "runs" / run_id / "detection.json"
    if not p.is_file():
        return {}
    s = json.loads(p.read_text()).get("summary") or {}
    # every scalar of the logger's team summary (residual_belief_mass, planned_residual_belief_mass,
    # targets detected, ...); lists and nested blocks stay in detection.json
    return {k: v for k, v in s.items() if isinstance(v, (int, float, str, bool)) or v is None}


def cmd_fly(a) -> int:
    cfg = PLANNERS[a.planner]
    man = export(a, [a.planner])
    run_id = a.run_id or f"{a.id}__{a.planner}__{dt.datetime.utcnow().strftime('%Y%m%d-%H%M%S')}"
    airstack = a.airstack
    if not a.no_up:
        sh([airstack, "down"], a.print_only, check=False)   # a previous sim (other stack) must not linger
        sh([airstack, "up", "--sim", "isaac", "--fleet", cfg["fleet"], "--stack", cfg["stack"], "--play", "--wait"],
           a.print_only)
    rc = sh(["bash", cfg["start"], "-n", "1", "-a", f"{man['altitude_m']:g}", "-r", run_id] + (a.start_args or []),
            a.print_only, check=False)
    if not a.no_up and not a.keep_up:
        sh([airstack, "down"], a.print_only, check=False)
    if a.print_only:
        return 0
    rec = {"run_id": run_id, "planner": a.planner, "arm": man["planners"][a.planner]["arm"],
           "utc": dt.datetime.utcnow().isoformat(timespec="seconds") + "Z", "exit_code": rc,
           "run_dir": f"runs/{run_id}", **run_summary(run_id)}
    rp = a.bench / "sim" / a.id / "runs.json"
    runs = json.loads(rp.read_text()) if rp.is_file() else []
    runs.append(rec)
    rp.write_text(json.dumps(runs, indent=1))
    print(f"[sim_scenario] run {run_id}: {json.dumps(rec)}")
    print(f"  bag, report.html, detection.json, telemetry, Foxglove: runs/{run_id}/   (recorded in {rp})")
    return 0 if rc == 0 else rc


def cmd_compare(a) -> int:
    md = a.bench / "sim" / a.id
    man = json.loads((md / "export.json").read_text()) if (md / "export.json").is_file() else {}
    runs = json.loads((md / "runs.json").read_text()) if (md / "runs.json").is_file() else []
    if not runs:
        print(f"[sim_scenario] no sim runs recorded for {a.id} ({md / 'runs.json'})")
    f = lambda v: "   -  " if v is None else f"{v:.4f}"  # noqa: E731
    print(f"{a.id}: budget {man.get('budget_s', 0):.0f} s, altitude {man.get('altitude_m', 0):.0f} m, home {man.get('home_ned')}")
    print(f"{'run':52s} {'planner':7s} {'flown':>7s} {'planned':>7s} {'offline':>7s}  targets")
    for r in runs:
        s = run_summary(r["run_id"]) or r
        off = ((man.get("planners") or {}).get(r["planner"]) or {}).get("offline") or {}
        found = next((s[k] for k in ("targets_detected", "detected", "n_detected", "targets_found") if k in s), "-")
        total = next((s[k] for k in ("targets_total", "n_targets", "targets") if k in s and not isinstance(s[k], list)), "-")
        tg = f"{found}/{total}"
        print(f"{r['run_id']:52s} {r['planner']:7s} {f(s.get('residual_belief_mass'))} "
              f"{f(s.get('planned_residual_belief_mass'))} {f(off.get('residual_hw'))}  {tg}")
    print("flown = the logger's residual belief of the real flight (runs/<run>/detection.json); planned = the same "
          "scorer on the plan; offline = the benchmark's residual_hw. Lower is better.")
    return 0


# --------------------------------------------------------------------------- CLI
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--bench", type=Path, default=REPO / "bench/wide900")
    common.add_argument("--arms", type=Path, default=HERE / "arms/fair900_curve.json")
    common.add_argument("--mtl-arm", default=PLANNERS["curve"]["arm"])
    common.add_argument("--tigris-arm", default=PLANNERS["tigris"]["arm"])
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("list", parents=[common], help="list scenarios with offline scores")
    p.add_argument("--family", choices=list(FAMILIES))
    p.add_argument("--budget-s", type=float)
    p.add_argument("--altitude-m", type=float)
    p.add_argument("--home", choices=list(HOMES))
    p.add_argument("--sort", choices=["id", "adv"], default="id")
    p.add_argument("--limit", type=int, default=400)
    p.set_defaults(fn=cmd_list)

    p = sub.add_parser("new", parents=[common], help="generate a scenario from family + parameters")
    p.add_argument("--family", required=True, choices=list(FAMILIES))
    p.add_argument("--budget-s", type=float, default=1000.0)
    p.add_argument("--altitude-m", type=float, default=300.0)
    p.add_argument("--home", default="near_center", help="near_center | edge | corner | N,E (mission NED m)")
    p.add_argument("--seed", type=int, default=None, help="prior seed (default: random, printed)")
    p.add_argument("--param", action="append", help="family parameter key=value (JSON value), repeatable")
    p.add_argument("--name", default=None, help="scenario id (default cNNNN_<family>)")
    p.add_argument("--offline", action="store_true", help="also plan + score it offline with both arms")
    p.set_defaults(fn=cmd_new)

    for name, fn, hlp in (("export", cmd_export, "write the scenario into the Isaac stack(s)"),
                          ("fly", cmd_fly, "export + airstack up + sortie (bag, report) + airstack down")):
        p = sub.add_parser(name, parents=[common], help=hlp)
        p.add_argument("--id", required=True)
        p.add_argument("--planner", choices=(["curve", "tigris", "both"] if name == "export" else ["curve", "tigris"]),
                       default="both" if name == "export" else None, required=(name == "fly"))
        p.add_argument("--gimbal-law", choices=["aim_point", "open_loop"], default="aim_point",
                       help="follower law for the curve plan (TIGRIS always flies aim_point)")
        p.add_argument("--compare-orienteering", action="store_true",
                       help="curve: also plan both orienteering modes for the run report (slower boot)")
        if name == "fly":
            p.add_argument("--run-id", default=None, help="default <id>__<planner>__<UTC time>")
            p.add_argument("--no-up", action="store_true", help="the sim is already up with the right stack")
            p.add_argument("--keep-up", action="store_true", help="leave the sim running afterwards")
            p.add_argument("--print-only", action="store_true", help="export, then only print the commands")
            p.add_argument("--airstack", default="airstack", help="the airstack CLI (default: airstack on PATH)")
            p.add_argument("--start-args", nargs=argparse.REMAINDER,
                           help="extra arguments for the start script, e.g. --start-args --no-images")
        p.set_defaults(fn=fn)

    p = sub.add_parser("compare", parents=[common], help="sim runs vs offline scores for a scenario")
    p.add_argument("--id", required=True)
    p.set_defaults(fn=cmd_compare)

    p = sub.add_parser("restore", parents=[common], help="put the stack bundles and fleet files back")
    p.set_defaults(fn=cmd_restore)

    a = ap.parse_args(argv)
    if not a.bench.is_absolute():
        a.bench = Path.cwd() / a.bench
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
