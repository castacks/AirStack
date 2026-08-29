#!/usr/bin/env python3
"""Resolve an `airstack up --scene <shortname>` against simulation/scenes.yaml.

Called host-side by airstack.sh. Prints KEY=VALUE lines for the selected
simulator on stdout; on an unknown or unavailable scene it prints the
availability table on stderr and exits 1.

Usage:
  resolve_scene.py --sim isaac|msairsim --scene NAME   # emit export lines
  resolve_scene.py --table                             # print the table
"""

import argparse
import os
import sys

import yaml

CATALOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scenes.yaml")

SIM_ALIASES = {
    "isaac": "isaac", "isaacsim": "isaac", "isaac-sim": "isaac",
    "airsim": "msairsim", "msairsim": "msairsim", "ms-airsim": "msairsim",
}
SIM_LABELS = {"isaac": "Isaac", "msairsim": "MS AirSim"}


def load_catalog(path=CATALOG):
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    scenes = data.get("scenes") or {}
    if not isinstance(scenes, dict):
        raise ValueError(f"{path}: top-level 'scenes' must be a mapping")
    return scenes


def isaac_entry(spec):
    """Normalize an isaac scene spec to (ref, stage_scale)."""
    if isinstance(spec, str):
        return spec, 1.0
    if isinstance(spec, dict) and "ref" in spec:
        return str(spec["ref"]), float(spec.get("stage_scale", 1.0))
    raise ValueError(f"invalid isaac scene spec: {spec!r}")


def short_ref(sim, spec):
    """Human-readable cell for the availability table."""
    if spec is None:
        return "-"
    if sim == "isaac":
        ref, _ = isaac_entry(spec)
        if "://" in ref or ref.endswith((".usd", ".usda", ".usdc", ".usdz")):
            return f"nucleus:{os.path.basename(ref)}" if "airlab-nucleus" in ref \
                else os.path.basename(ref)
        return f"pegasus:{ref}"
    return str(spec)


def print_table(scenes, out=sys.stderr):
    rows = [("SCENE", SIM_LABELS["isaac"].upper(), SIM_LABELS["msairsim"].upper())]
    for name in sorted(scenes):
        entry = scenes[name] or {}
        rows.append((name,
                     short_ref("isaac", entry.get("isaac")),
                     short_ref("msairsim", entry.get("msairsim"))))
    widths = [max(len(r[i]) for r in rows) for i in range(3)]
    for i, r in enumerate(rows):
        print("  ".join(c.ljust(w) for c, w in zip(r, widths)).rstrip(), file=out)
        if i == 0:
            print("  ".join("-" * w for w in widths), file=out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", help="target simulator (isaac|airsim aliases ok)")
    ap.add_argument("--scene", help="scene shortname from scenes.yaml")
    ap.add_argument("--table", action="store_true",
                    help="print the availability table to stdout and exit")
    ap.add_argument("--catalog", default=CATALOG)
    args = ap.parse_args()

    scenes = load_catalog(args.catalog)

    if args.table:
        print_table(scenes, out=sys.stdout)
        return 0

    if not args.sim or not args.scene:
        ap.error("--sim and --scene are required (or use --table)")
    sim = SIM_ALIASES.get(args.sim)
    if sim is None:
        print(f"ERROR: unknown simulator '{args.sim}' "
              f"(expected one of: {', '.join(sorted(set(SIM_ALIASES)))})",
              file=sys.stderr)
        return 1

    entry = scenes.get(args.scene)
    spec = (entry or {}).get(sim)
    if spec is None:
        if entry is None:
            print(f"ERROR: unknown scene '{args.scene}'. Available scenes:",
                  file=sys.stderr)
        else:
            others = [SIM_LABELS[s] for s in ("isaac", "msairsim")
                      if s != sim and entry.get(s) is not None]
            print(f"ERROR: scene '{args.scene}' is not available for "
                  f"{SIM_LABELS[sim]} (only: {', '.join(others) or 'none'}). "
                  f"Available scenes:", file=sys.stderr)
        print_table(scenes)
        return 1

    if sim == "isaac":
        ref, scale = isaac_entry(spec)
        print(f"ISAAC_SIM_SCENE={ref}")
        print(f"ISAAC_SIM_STAGE_SCALE={scale}")
    else:
        print(f"MS_AIRSIM_SCENE={spec}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
