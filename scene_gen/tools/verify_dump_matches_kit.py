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

WHAT COUNTS AS A DISAGREEMENT. Only building model and world position are
load-bearing.  Houses are paired geometrically (same model within tolerance),
not by their index in the full placement stream: Kit appends city details and
building props that the lightweight offline planner intentionally omits.
Details, total placement count, list order, and `cell` names are therefore not
part of this gate.  Shared-index diagnostics are printed only to expose stream
ordering changes; they do not make the verification fail.
"""
import argparse
import collections
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


def _geometry_match(left, right, tol_m):
    """Greedy nearest same-model matching, independent of full-list index."""
    by_model = collections.defaultdict(list)
    for p in right.values():
        by_model[os.path.basename(str(p.get("usd") or ""))].append(p)
    used = set()
    matched = []
    missing = []
    for p in left.values():
        name = os.path.basename(str(p.get("usd") or ""))
        candidates = []
        for q in by_model.get(name, ()):
            marker = id(q)
            if marker in used:
                continue
            d = math.hypot(float(p["x_m"]) - float(q["x_m"]),
                           float(p["y_m"]) - float(q["y_m"]))
            candidates.append((d, marker, q))
        if not candidates:
            missing.append(p)
            continue
        d, marker, q = min(candidates, key=lambda x: x[0])
        if d <= tol_m:
            used.add(marker)
            matched.append((d, p, q))
        else:
            missing.append(p)
    extra = [q for qs in by_model.values() for q in qs if id(q) not in used]
    return matched, missing, extra


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

    # Once packing diverges, W/D/H at a shared list index usually compares
    # two unrelated assets. Also compare dimensions by model so a stale
    # offline footprint can identify itself instead of being hidden by the
    # downstream index avalanche it caused.
    def _model_sizes(doc):
        vals = collections.defaultdict(list)
        world_xy = (doc.get("dimensions_space") == "world_xy" or
                    str(doc.get("_source", "")).startswith("plan_to_fc_dump"))
        for p in doc.get("placements") or ():
            name = os.path.basename(str(p.get("usd") or ""))
            if name and all(p.get(k) is not None for k in ("W", "D", "H")):
                w, d, h = (float(p[k]) for k in ("W", "D", "H"))
                if world_xy and 45.0 <= \
                        (float(p.get("yaw_deg", 0.0)) % 180.0) < 135.0:
                    w, d = d, w
                vals[name].append((w, d, h))
        return {name: tuple(sum(v[j] for v in samples) / len(samples)
                            for j in range(3))
                for name, samples in vals.items()}

    oms = _model_sizes(off)
    kms = _model_sizes(kit)
    model_resized = []
    for name in sorted(set(oms) & set(kms)):
        ov, kv = oms[name], kms[name]
        if any(abs(ov[j] - kv[j]) > max(0.05, 0.005 * abs(kv[j]))
               for j in range(3)):
            model_resized.append((name, ov, kv))

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
    # Total placements deliberately differ: the lightweight offline planner
    # stops before Kit's city-detail and building-prop passes. `i` is therefore
    # diagnostic only; the load-bearing comparison below is house geometry.

    only_off = sorted(set(oh) - set(kh))
    only_kit = sorted(set(kh) - set(oh))

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
    geo_matched, geo_off, geo_kit = _geometry_match(oh, kh, a.tol_m)
    print("\nmatched indices : %d" % len(set(oh) & set(kh)))
    print("moved > %.2f m   : %d" % (a.tol_m, len(moved)))
    print("different model : %d" % len(swapped))
    print("different W/D/H : %d   (offline sizes come from the asset-set "
          "scrape, not from opening the USD)" % len(resized))
    print("model size conflicts: %d   (same model, independent of index)"
          % len(model_resized))
    print("geometry matched: %d/%d offline houses; %d Kit-only house(s) "
          "(same model within %.2f m, index ignored)"
          % (len(geo_matched), len(oh), len(geo_kit), a.tol_m))

    for d, i, o, k in moved[:a.max_report]:
        print("   i=%-6d %7.2f m   offline(%9.2f,%9.2f)  kit(%9.2f,%9.2f)  %s"
              % (i, d, o["x_m"], o["y_m"], k["x_m"], k["y_m"],
                 os.path.basename(str(k.get("usd")))[:34]))
    for i, o, k in swapped[:a.max_report]:
        print("   i=%-6d MODEL  offline=%-32s kit=%s"
              % (i, os.path.basename(str(o.get("usd")))[:32],
                 os.path.basename(str(k.get("usd")))[:32]))
    for name, ov, kv in model_resized[:a.max_report]:
        print("   %-34s offline=%7.2f x %7.2f x %7.2f  "
              "kit=%7.2f x %7.2f x %7.2f"
              % ((name[:34],) + ov + kv))
    for p in geo_off[:a.max_report]:
        print("   GEOMETRY OFFLINE-ONLY  (%9.2f,%9.2f) %s"
              % (p["x_m"], p["y_m"], os.path.basename(str(p.get("usd")))))
    for p in geo_kit[:a.max_report]:
        print("   GEOMETRY KIT-ONLY      (%9.2f,%9.2f) %s"
              % (p["x_m"], p["y_m"], os.path.basename(str(p.get("usd")))))

    if geo_off or geo_kit:
        bad.append("building geometry differs: %d offline-only and %d "
                   "Kit-only house placement(s)" % (len(geo_off), len(geo_kit)))
    if model_resized:
        bad.append("%d model dimension conflict(s) remain" % len(model_resized))
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
    print("\033[32mMATCH\033[0m — offline dump and Kit agree on every building "
          "model and world position (detail streams ignored). The offline plan is trustworthy for "
          "this (preset, seed).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
