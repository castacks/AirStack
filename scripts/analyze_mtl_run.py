#!/usr/bin/env python3
"""Post-flight team scoring for an MTL search run.

    python3 scripts/analyze_mtl_run.py --run-dir runs/latest
    python3 scripts/analyze_mtl_run.py --run-dir runs/20260923-141500 --out-dir /tmp/report

Reads every ``<run-dir>/<robot>/telemetry.csv`` written by mtl_metrics_logger,
fuses the agents on one timeline (joint miss product per step, so two robots
looking at a target at once are credited together and the responsible agent is
the one whose look crossed the threshold), and writes the TEAM outputs next to
the per-robot folders:

    <run-dir>/telemetry.csv     all agents, one table (agent column)
    <run-dir>/detection.json    per-target discovery time, responsible agent,
                                residual-belief and covered-mass curves, summary
    <run-dir>/residual_belief.csv   prior vs residual belief per 10 m block
    <run-dir>/report.html       self-contained interactive report

The headline number is the RESIDUAL BELIEF MASS = P(target missed by the
search): every pixel of the prior (normalised to sum to 1) gets the same miss
update as a target standing there. Lower is better; it is the number to compare
planners (and plan vs flight: the planned value scores each robot's track.json).

Inputs are resolved inside the run dir first (``<robot>/scenario.json`` and
``<robot>/track.json`` from mtl_search_planner, ``ground_truth.json`` and
``belief.png`` copied by the logger), then fall back to
``stacks/mtl_search/config/``. Stdlib only (plus the repo's own modules).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "robot/ros_ws/src/behavior/mtl_metrics_logger"))

from mtl_metrics_logger.analysis import planned_looks_from_track, write_run_outputs  # noqa: E402
from mtl_metrics_logger.report import read_telemetry_csv  # noqa: E402

DEFAULT_CONFIG = REPO / "stacks/mtl_search/config"


def _first(*paths: Path) -> Path | None:
    return next((p for p in paths if p is not None and p.is_file()), None)


def load_run(run_dir: Path, scenario: Path | None, ground_truth: Path | None):
    agents = sorted(d for d in run_dir.iterdir() if d.is_dir() and (d / "telemetry.csv").is_file())
    if not agents:
        raise SystemExit(f"{run_dir}: no <robot>/telemetry.csv found - did the sortie finish "
                         "(mtl_metrics_logger writes on COMPLETE/ABORTED)?")
    sc_path = _first(scenario, *(a / "scenario.json" for a in agents), DEFAULT_CONFIG / "scenario.json")
    gt_path = _first(ground_truth, run_dir / "ground_truth.json", DEFAULT_CONFIG / "ground_truth.json")
    if sc_path is None or gt_path is None:
        raise SystemExit("cannot find scenario.json / ground_truth.json (pass --scenario / --ground-truth)")
    sc = json.loads(sc_path.read_text(encoding="utf-8"))
    gt = json.loads(gt_path.read_text(encoding="utf-8"))

    rows, planned = {}, {}
    for a in agents:
        name = a.name
        rows[name] = [r for r in read_telemetry_csv(a / "telemetry.csv") if r.get("agent") in (None, "", name)]
        track = a / "track.json"
        if track.is_file():
            tr = json.loads(track.read_text(encoding="utf-8"))
            hx, hy, _ = tr.get("home_enu", [0.0, 0.0, 0.0])
            s = tr["samples"]
            planned[name] = {"planned": [[x + hx, y + hy] for x, y in zip(s["x_map"], s["y_map"])],
                             "home": [hx, hy], "serviced_cells": tr.get("serviced_cells", []),
                             "planned_length_m": tr.get("flown_length_m"),
                             "looks": planned_looks_from_track(tr)}
    png = _first(run_dir / "belief.png", sc_path.with_name("belief.png"), DEFAULT_CONFIG / "belief.png")
    return sc, gt, rows, planned, (png.read_bytes() if png else None), sc_path, gt_path


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run-dir", type=Path, default=REPO / "runs" / "latest")
    ap.add_argument("--out-dir", type=Path, default=None, help="default: the run dir itself")
    ap.add_argument("--scenario", type=Path, default=None)
    ap.add_argument("--ground-truth", type=Path, default=None)
    args = ap.parse_args(argv)

    run_dir = args.run_dir.resolve()
    if not run_dir.is_dir():
        raise SystemExit(f"run dir not found: {args.run_dir}")
    sc, gt, rows, planned, png, sc_path, gt_path = load_run(run_dir, args.scenario, args.ground_truth)
    out = (args.out_dir or run_dir).resolve()
    res = write_run_outputs(
        out, scenario=sc, ground_truth=gt, rows_by_agent=rows, planned_by_agent=planned,
        title=f"MTL team search — {run_dir.name}",
        subtitle=f"scenario {sc['mission']['name']} · {len(rows)} agent(s): {', '.join(sorted(rows))} · "
                 f"fused on one timeline",
        belief_png=png,
        extra={"run_id": run_dir.name, "inputs": {"scenario": str(sc_path), "ground_truth": str(gt_path)}})
    s = res["summary"]
    mttd = s["mean_time_to_discovery_s"]
    resid, planned_resid = s.get("residual_belief_mass"), s.get("planned_residual_belief_mass")
    if resid is None:
        print(f"[analyze_mtl_run] {run_dir.name}: residual belief n/a (the scenario carries no prior bumps)")
    else:
        print(f"[analyze_mtl_run] {run_dir.name}: residual belief {resid:.4f} = P(target missed), lower is better"
              f"{'' if planned_resid is None else f' (planned {planned_resid:.4f})'}; "
              f"{100 * s['searched_belief_fraction']:.1f} % of the prior searched")
    print(f"  {s['targets_detected']}/{s['targets_total']} targets found"
          f"{'' if mttd is None else f', mean time to discovery {mttd:.1f} s'}; "
          f"valid cells reached {s['cells_covered']}/{s['cells_total']} "
          f"({100 * s['belief_mass_fraction']:.1f} % of their mass) over {s['total_path_length_m'] / 1000:.2f} km")
    if s.get("realized_over_planned_mass") is not None:
        print(f"  realized / planned coverage: {100 * s['realized_over_planned_mass']:.1f} %")
    for name in ("telemetry.csv", "detection.json", "residual_belief.csv", "report.html"):
        if (out / name).is_file():
            print(f"  wrote {out / name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
