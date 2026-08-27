#!/usr/bin/env python3
"""quake_timings.py — the archetype bake's cost, per style and per grade.

    python3 scene_gen/tools/quake_timings.py [archetypes.json] [--md]

Reads the manifest `bake_quake_archetypes_launch_script.py` writes (which
carries `fracture_s` per archetype and `settle_s` / `settle_bodies` /
`still_moving` per style row) and prints a table — the reference for "how
long does live damage take" against the assembled city's load time, which
`downtown_quake_launch_script.py` prints in its banner. Host-side, no USD.
"""

import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT = os.path.join(_HERE, "..", "assets", "archetypes_quake", "archetypes.json")


_REPO = os.path.normpath(os.path.join(_HERE, "..", ".."))


def _local(path):
    return path.replace("/isaac-sim/AirStack", _REPO) if path.startswith("/isaac-sim/AirStack") else path


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    md = "--md" in sys.argv
    path = args[0] if args else DEFAULT
    recs = json.load(open(path))
    styles = []
    for r in recs:
        if r["style"] not in styles:
            styles.append(r["style"])
    grades = sorted({r["level"] for r in recs})
    rows = []
    tot_frac = tot_settle = 0.0
    for st in styles:
        mine = {r["level"]: r for r in recs if r["style"] == st}
        frac = sum(float(r.get("fracture_s") or 0.0) for r in mine.values())
        any_r = next(iter(mine.values()))
        settle = float(any_r.get("settle_s") or 0.0)
        bodies = int(any_r.get("settle_bodies") or 0)
        moving = int(any_r.get("still_moving") or 0)
        meshes = sum(int(r.get("meshes") or 0) for r in mine.values())
        # the manifest carries the bake machine's (container) paths
        size = sum(os.path.getsize(_local(r["usd"])) for r in mine.values()
                   if os.path.exists(_local(r["usd"]))) / 1e6
        tot_frac += frac
        tot_settle += settle
        per = "  ".join("{0}:{1:.0f}s".format(g[2:], float(mine[g].get("fracture_s") or 0.0))
                        for g in grades if g in mine and g != "DG0")
        rows.append((st, any_r.get("type", "?"), "{0:.0f}x{1:.0f}x{2:.0f}".format(
            float(any_r.get("W", 0)), float(any_r.get("D", 0)), float(any_r.get("H", 0))),
            frac, per, bodies, settle, moving, meshes, size))
    if md:
        print("| style | type | W x D x H | fracture s (all grades) | per grade | settle bodies | settle s | still moving | meshes | MB |")
        print("|---|---|---|---|---|---|---|---|---|---|")
        for r in rows:
            print("| {0} | {1} | {2} | {3:.0f} | {4} | {5} | {6:.0f} | {7} | {8} | {9:.1f} |".format(*r))
        print("| **total** | | | **{0:.0f}** | | | **{1:.0f}** | | | |".format(tot_frac, tot_settle))
    else:
        for r in rows:
            print("{0:<18} {1:<9} {2:<10} frac {3:5.0f} s  [{4}]  settle {5:5d} bodies {6:6.0f} s  moving {7:4d}  meshes {8:6d}  {9:5.1f} MB".format(*r))
        print("TOTAL fracture {0:.0f} s, settle {1:.0f} s over {2} styles ({3} archetypes)".format(
            tot_frac, tot_settle, len(styles), len(recs)))


if __name__ == "__main__":
    main()
