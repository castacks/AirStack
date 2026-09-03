#!/usr/bin/env python3
"""verify_dump_matches_kit.py — prove the OFFLINE dump is the same city Kit
builds.

WHY THIS EXISTS. `plan_to_fc_dump.py` reimplements, on CPU, the placement dump
that `urban_fire_city_launch_script.dump_city_placements` writes inside Kit.
That is what lets the whole manifest chain run on a laptop — and it is also a
new failure mode the old pipeline could not have: if the two disagree, the
manifest names buildings the assembled city does not have in those positions,
and the bakes compose onto the wrong cells. The symptom appears three stages
later as `[fc] manifest/city match: 4/34`, long after the cheap place to catch
it.

Two Stage-1 runs of the IDENTICAL preset and seed have produced 16,066 vs
15,997 prims before now — a genuinely different city — which is why this is a
diff and not an assumption.

HOW TO GET THE KIT DUMP (once per level, on the pod):

    docker exec -e PYTHONHASHSEED=0 -e SG_INSTANCE_PLACEMENTS=1 \\
        -e PYTHONUNBUFFERED=1 isaac-sim bash -lc '
      cd /isaac-sim/AirStack &&
      SCENE_CONFIG=downtown_urban_fire_1000_l1 \\
      FC_INTACT_ONLY=1 \\
      FC_DUMP=/isaac-sim/AirStack/scene_gen/_plans/kit_dump_l1.json \\
      ./python.sh simulation/isaac-sim/launch_scripts/urban_fire_city_launch_script.py --no-window'

`FC_INTACT_ONLY=1` builds the city and dumps placements WITHOUT composing any
bake, so it is cheap and needs no bakes to exist yet. The two env vars are the
determinism pair — omit either and you are diffing two different cities and
learning nothing.

THEN:

    python3 scene_gen/tools/verify_dump_matches_kit.py \\
        --offline scene_gen/_plans/fc_dump_1km_l1.json \\
        --kit     scene_gen/_plans/kit_dump_l1.json

Exits non-zero on any disagreement that matters. Run it once per level; if it
passes, the offline dump is trustworthy for that (preset, seed) and the pod
half can be driven from plans made entirely on the host.

WHAT COUNTS AS A DISAGREEMENT. Position and model are load-bearing — a bake is
composed onto a cell matched by index, verified by usd and distance, so those
three must agree. `cell` NAMES are explicitly NOT compared: `plan_to_fc_dump`'s
own docstring says the synthesised name is not guaranteed to equal Kit's, and
nothing in the pipeline matches on it alone.
"""
import argparse
import json
import math
import os
import sys

#: A building further than this from its counterpart is a different placement,
#: not float noise. The assembly's own frame guard uses 0.5 m
#: (`CELL_MATCH_TOL_M`), so anything under that would pass there anyway.
POS_TOL_M = 0.5


def _houses(doc):
    return {int(p["i"]): p for p in (doc.get("placements") or [])
            if p.get("category") == "house"}


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--offline", required=True,
                    help="plan_to_fc_dump.py output")
    ap.add_argument("--kit", required=True,
                    help="FC_DUMP output from a real Kit build")
    ap.add_argument("--tol-m", type=float, default=POS_TOL_M)
    ap.add_argument("--max-report", type=int, default=15)
    a = ap.parse_args()

    off = json.load(open(a.offline))
    kit = json.load(open(a.kit))
    oh, kh = _houses(off), _houses(kit)

    print("offline : %-42s %5d houses, %6d placements, preset=%s seed=%s"
          % (os.path.basename(a.offline), len(oh),
             off.get("n_placements_total", 0), off.get("preset"),
             off.get("seed")))
    print("kit     : %-42s %5d houses, %6d placements, preset=%s seed=%s"
          % (os.path.basename(a.kit), len(kh),
             kit.get("n_placements_total", 0), kit.get("preset"),
             kit.get("seed")))

    bad = []
    if str(off.get("preset")) != str(kit.get("preset")):
        bad.append("preset differs: %r vs %r"
                   % (off.get("preset"), kit.get("preset")))
    if int(off.get("seed", -1)) != int(kit.get("seed", -2)):
        bad.append("seed differs: %s vs %s" % (off.get("seed"), kit.get("seed")))
    # The TOTAL placement count is what `i` indexes into, so a difference here
    # means every index is suspect even if the houses happen to line up.
    if off.get("n_placements_total") != kit.get("n_placements_total"):
        bad.append("n_placements_total differs: %s vs %s — `i` indexes the "
                   "FULL list, so every manifest index is suspect"
                   % (off.get("n_placements_total"),
                      kit.get("n_placements_total")))

    only_off = sorted(set(oh) - set(kh))
    only_kit = sorted(set(kh) - set(oh))
    if only_off:
        bad.append("%d house index(es) only in the offline dump (e.g. %s)"
                   % (len(only_off), only_off[:8]))
    if only_kit:
        bad.append("%d house index(es) only in the Kit dump (e.g. %s)"
                   % (len(only_kit), only_kit[:8]))

    moved, swapped, resized = [], [], []
    for i in sorted(set(oh) & set(kh)):
        o, k = oh[i], kh[i]
        d = math.hypot(float(o["x_m"]) - float(k["x_m"]),
                       float(o["y_m"]) - float(k["y_m"]))
        if d > a.tol_m:
            moved.append((d, i, o, k))
        if os.path.basename(str(o.get("usd"))) != os.path.basename(str(k.get("usd"))):
            swapped.append((i, o, k))
        for f in ("W", "D", "H"):
            if o.get(f) is None or k.get(f) is None:
                continue
            if abs(float(o[f]) - float(k[f])) > max(0.25, 0.02 * float(k[f])):
                resized.append((i, f, o[f], k[f]))
                break

    moved.sort(reverse=True)
    print("\nmatched indices : %d" % len(set(oh) & set(kh)))
    print("moved > %.2f m   : %d" % (a.tol_m, len(moved)))
    print("different model : %d" % len(swapped))
    print("different W/D/H : %d   (offline sizes come from the asset-set "
          "scrape, not from opening the USD)" % len(resized))

    for d, i, o, k in moved[:a.max_report]:
        print("   i=%-6d %7.2f m   offline(%9.2f,%9.2f)  kit(%9.2f,%9.2f)  %s"
              % (i, d, o["x_m"], o["y_m"], k["x_m"], k["y_m"],
                 os.path.basename(str(k.get("usd")))[:34]))
    for i, o, k in swapped[:a.max_report]:
        print("   i=%-6d MODEL  offline=%-32s kit=%s"
              % (i, os.path.basename(str(o.get("usd")))[:32],
                 os.path.basename(str(k.get("usd")))[:32]))

    if moved:
        bad.append("%d house(s) moved more than %.2f m" % (len(moved), a.tol_m))
    if swapped:
        bad.append("%d house(s) carry a different model at the same index"
                   % len(swapped))
    # A W/D/H difference is NOT fatal on its own: the offline dump measures
    # from the checked-in asset-set scrape while Kit opens the USD. It only
    # matters if it crosses a routing threshold (the fire height cap, the
    # kit_substitute ratios), so it is reported loudly and not failed on.
    if resized:
        print("\n   NOTE: %d size difference(s) — offline sizes come from the "
              "asset-set scrape. Not failed on, but check none crosses the "
              "232 m fire height cap or a kit_substitute ratio." % len(resized))
        for i, f, ov, kv in resized[:5]:
            print("     i=%-6d %s offline=%.2f kit=%.2f" % (i, f, ov, kv))

    print()
    if bad:
        print("\033[31mMISMATCH\033[0m — the offline dump is NOT the city Kit "
              "builds for this (preset, seed):")
        for b in bad:
            print("  * %s" % b)
        print("\nDo not drive a pod run from this plan. Check that both sides "
              "ran with PYTHONHASHSEED=0 and SG_INSTANCE_PLACEMENTS=1 first — "
              "that is the usual cause.")
        return 1
    print("\033[32mMATCH\033[0m — offline dump and Kit agree on every house "
          "index, position and model. The offline plan is trustworthy for "
          "this (preset, seed).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
