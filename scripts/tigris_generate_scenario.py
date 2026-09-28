#!/usr/bin/env python3
"""Generate the TIGRIS search scenario bundle (stacks/tigris_search/config).

    python3 scripts/tigris_generate_scenario.py                 # mission.yaml + tigris_search_fleet -> bundle
    python3 scripts/tigris_generate_scenario.py --check         # fail if the committed bundle is stale
    python3 scripts/tigris_generate_scenario.py --compare-mtl   # prove it is the SAME problem as mtl_search

The bundle format is the MTL one (``mtl.scenario/1`` + ground_truth.json + belief.png), so
the same Isaac scene and mtl_metrics_logger score both planners. It is written by the
same generator (scripts/mtl_generate_scenario.py) with TIGRIS's own mission file and its
single-robot fleet; nothing of the MTL *planner* is involved.

``--compare-mtl`` checks that the prior bumps, valid cells, ground-truth targets, sensor
and detection model, aircraft and per-agent budget match stacks/mtl_search/config, and that
robot_1 has the same home. Run it after editing either mission.yaml: a TIGRIS-vs-MTL
comparison is only meaningful on an identical problem.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
GEN = REPO / "scripts/mtl_generate_scenario.py"
DEFAULT_MISSION = REPO / "stacks/tigris_search/config/mission.yaml"
DEFAULT_FLEET = REPO / "config/fleets/tigris_search_fleet.yaml"
DEFAULT_OUT = REPO / "stacks/tigris_search/config"
MTL_CONFIG = REPO / "stacks/mtl_search/config"


def compare_with_mtl(tigris_dir: Path, mtl_dir: Path) -> int:
    a = json.loads((mtl_dir / "scenario.json").read_text())
    b = json.loads((tigris_dir / "scenario.json").read_text())
    ga = json.loads((mtl_dir / "ground_truth.json").read_text())
    gb = json.loads((tigris_dir / "ground_truth.json").read_text())
    checks = {
        "prior bumps": a["airstack"]["belief"]["bumps"] == b["airstack"]["belief"]["bumps"],
        "prior cap/floor": (a["airstack"]["belief"].get("belief_cap"), a["airstack"]["belief"].get("base_uncertainty"))
        == (b["airstack"]["belief"].get("belief_cap"), b["airstack"]["belief"].get("base_uncertainty")),
        "area": a["mission"]["area"] == b["mission"]["area"],
        "valid cells": a["cells"] == b["cells"],
        "targets": ga.get("targets") == gb.get("targets"),
        "sensor + detection": a["sensor"] == b["sensor"],
        "aircraft": a["aircraft"] == b["aircraft"],
        "budget (per agent)": (a["team"].get("max_flight_time_s"), a["team"].get("max_flight_distance_m"))
        == (b["team"].get("max_flight_time_s"), b["team"].get("max_flight_distance_m")),
    }
    ha = next((x["home_ned"] for x in a["team"]["agents"] if x["name"] == "robot_1"), None)
    hb = next((x["home_ned"] for x in b["team"]["agents"] if x["name"] == "robot_1"), None)
    checks["robot_1 home"] = ha == hb
    bad = [k for k, ok in checks.items() if not ok]
    for k, ok in checks.items():
        print(f"  {'ok  ' if ok else 'DIFF'} {k}")
    if bad:
        print(f"[tigris_generate_scenario] the TIGRIS and MTL problems differ in: {', '.join(bad)}")
        return 1
    print("[tigris_generate_scenario] identical problem to mtl_search (MTL agents: "
          f"{', '.join(x['name'] for x in a['team']['agents'])}; TIGRIS: "
          f"{', '.join(x['name'] for x in b['team']['agents'])})")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mission", type=Path, default=DEFAULT_MISSION)
    ap.add_argument("--fleet", type=Path, default=DEFAULT_FLEET)
    ap.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--check", action="store_true", help="fail if the committed bundle is stale")
    ap.add_argument("--compare-mtl", action="store_true", help="compare the bundle with stacks/mtl_search/config")
    ap.add_argument("--mtl-dir", type=Path, default=MTL_CONFIG)
    args = ap.parse_args(argv)

    if not args.compare_mtl or args.check:
        cmd = [sys.executable, str(GEN), "--mission", str(args.mission), "--fleet", str(args.fleet),
               "--out-dir", str(args.out_dir)] + (["--check"] if args.check else [])
        rc = subprocess.call(cmd)
        if rc != 0:
            return rc
    if args.compare_mtl:
        return compare_with_mtl(args.out_dir, args.mtl_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
