#!/usr/bin/env python3
"""Generate the MTL search scenario bundle from mission.yaml + a fleet file.

    python3 scripts/mtl_generate_scenario.py                    # defaults below
    python3 scripts/mtl_generate_scenario.py --check            # fail if the committed bundle is stale
    python3 scripts/mtl_generate_scenario.py --mission M.yaml --fleet F.yaml --out-dir DIR

Writes ``scenario.json`` (planner-facing, a superset of ``mtl.scenario/1``),
``ground_truth.json`` (targets; never read by a planner) and ``belief.png``
(the ground texture) into ``--out-dir``. Agent homes come from the fleet's
``spawn:`` entries (world ENU -> mission NED), in fleet order, so robot N is
agent N and domain N.

Deterministic: same inputs, byte-identical outputs (stdlib ``random`` seeded
from ``mission.seed``). Needs PyYAML (present in every AirStack container and
most hosts) and nothing else.
"""

from __future__ import annotations

import argparse
import filecmp
import sys
import tempfile
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "robot/ros_ws/src/global/planners/mtl_search_planner"))

from mtl_search_planner.scenario import (  # noqa: E402
    build_scenario, enu_to_ned, write_scenario_bundle)

DEFAULT_MISSION = REPO / "stacks/mtl_search/config/mission.yaml"
DEFAULT_FLEET = REPO / "config/fleets/mtl_search_fleet.yaml"
DEFAULT_OUT = REPO / "stacks/mtl_search/config"


def agents_from_fleet(fleet_path: Path) -> list[dict]:
    fleet = yaml.safe_load(fleet_path.read_text(encoding="utf-8")) or {}
    robots = fleet.get("robots") or {}
    if not robots:
        raise SystemExit(f"{fleet_path}: fleet has no robots")
    agents = []
    for name, entry in robots.items():
        x, y, _z = (entry or {}).get("spawn", [0.0, 0.0, 0.07])
        n, e, _ = enu_to_ned(float(x), float(y), 0.0)
        agents.append({"name": name, "home_ned": [n, e]})
    return agents


def generate(mission_path: Path, fleet_path: Path, out_dir: Path) -> dict:
    raw = yaml.safe_load(mission_path.read_text(encoding="utf-8")) or {}
    if "mission" not in raw:
        raise SystemExit(f"{mission_path}: no top-level 'mission:' block")
    agents = agents_from_fleet(fleet_path)
    rel = lambda p: str(p.resolve().relative_to(REPO)) if p.resolve().is_relative_to(REPO) else str(p)  # noqa: E731
    scenario, gt, grid = build_scenario(
        raw["mission"], agents,
        provenance={"generator": "scripts/mtl_generate_scenario.py",
                    "mission": rel(mission_path), "fleet": rel(fleet_path)})
    paths = write_scenario_bundle(out_dir, scenario, gt, grid)
    return {"scenario": scenario, "ground_truth": gt, "paths": paths}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mission", type=Path, default=DEFAULT_MISSION)
    ap.add_argument("--fleet", type=Path, default=DEFAULT_FLEET)
    ap.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--check", action="store_true",
                    help="regenerate into a temp dir and fail if --out-dir differs")
    args = ap.parse_args(argv)

    if args.check:
        with tempfile.TemporaryDirectory() as tmp:
            generate(args.mission, args.fleet, Path(tmp))
            stale = [n for n in ("scenario.json", "ground_truth.json", "belief.png")
                     if not (args.out_dir / n).is_file()
                     or not filecmp.cmp(Path(tmp) / n, args.out_dir / n, shallow=False)]
        if stale:
            print(f"STALE: {', '.join(stale)} in {args.out_dir} - rerun "
                  "scripts/mtl_generate_scenario.py", file=sys.stderr)
            return 1
        print(f"OK: {args.out_dir} matches {args.mission.name} + {args.fleet.name}")
        return 0

    out = generate(args.mission, args.fleet, args.out_dir)
    sc, gt = out["scenario"], out["ground_truth"]
    mass = sum(sc["cells"]["mass"])  # P(target in a valid cell); the prior sums to 1
    print(f"[mtl_generate_scenario] {sc['mission']['name']}: "
          f"{len(sc['cells']['centers'])} valid cells (belief mass > "
          f"{sc['mapping']['minimum_belief_mass']:g} each; they hold {100 * mass:.1f} % of the "
          f"prior, total {sc['cells']['total_map_mass']:.4f}), "
          f"{len(gt['targets'])} targets, {len(sc['team']['agents'])} agents "
          f"({', '.join(a['name'] for a in sc['team']['agents'])})")
    for key, path in out["paths"].items():
        print(f"  wrote {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
