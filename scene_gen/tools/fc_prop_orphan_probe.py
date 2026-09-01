#!/usr/bin/env python3
"""fc_prop_orphan_probe — WHICH ROOF/WALL PROPS ARE LEFT FLOATING when the
fire city swaps a damaged building for its bake, and which ones the
launcher's companion-hide rule would take with it.

    python3 scene_gen/tools/fc_prop_orphan_probe.py \
        [--dump   scene_gen/_plans/fc_dump_500.json] \
        [--manifest scene_gen/_plans/fire_city_500m.json] \
        [--preset downtown_fire_500] [--radius 0.0]

THE BUG THIS EXISTS FOR (user, 2026-08-31, on the live 500 m city):
"/World/stage/generated/roof_house_94_1354 this roof house is floating with
no building near it" + "Lots of floating debris and roof props".

`detail/gac_props.dress()` authors the rooftop kit (`roof_house` stair/lift
bulkheads, `roof_tank`, `roof_mast`, `roof_plant`, `wall_run`) as SEPARATE
placements appended to the city's placement list — NOT as children of the
building they stand on. `urban_fire_city_launch_script.compose_bakes` hides
the damaged building's intact prim (`FC_HIDE`, `MakeInvisible` by default)
and composes the bake beside it. The bake carries its OWN settled roof plant
(`fire_bake` bakes `dress_roof_urban`'s plant into the export — see the F5c
sidecar's "roof plant: bulkhead on W, 1 row(s) of condensers on a pad" note),
so the CITY's props for that building are now standing, unsupported, at the
elevation of a roof that is no longer drawn.

THE ASSOCIATION IS EXPLICIT — no proximity heuristic is needed.
`gac_props._place()` writes `"of": "{basename}@{x:.1f},{y:.1f}"` onto every
prop placement, built from the BUILDING PLACEMENT's own `usd` basename and
`x_m`/`y_m`. `gac_props.roof_plant_of()` already matches on exactly that tag
and its docstring says why: "MATCHED BY IDENTITY, NEVER PROXIMITY: ...
Matching by nearest-position instead fails on a real street, where the
neighbour is closer than the far side of the same building."

So the launcher's rule is: **hide every placement whose `of` tag equals the
tag of the building placement it just hid.** `--radius R` additionally
reports (never hides) props within R m of a hidden building's footprint that
carry a DIFFERENT `of`, as a cross-check that the tag rule is not missing a
population.

WHY THIS RUNS OFFLINE. `dress()` is deterministic and its RNG
(`random.Random(config["seed"] + 6301)`, `generate_scene.py`) is drawn ONLY
inside `roof_props`/`wall_props`, i.e. only for `category == "house"`
placements — every other placement is `continue`d before any draw. So
replaying `dress()` over just the 75 house placements of the FC dump, in
their recorded index order, reproduces the exact prop list Kit built, without
Isaac Sim and without touching the live session.

THE ONE APPROXIMATION: a non-GAC building named in
`building_props.flat_roof` takes its W/D/H/z0 from the live `SizeResolver`.
The dump carries the measured W/D/H (same resolver, same process that built
the city) but not `base`, so `z0` is stubbed at 0 for those. That moves only
such a building's `roof_z` — never a prop's `of` tag, its XY, or which
building it belongs to, which is all this probe concludes from.
"""

import argparse
import json
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SG = os.path.normpath(os.path.join(_HERE, ".."))
sys.path.insert(0, _SG)

from detail import gac_props                                    # noqa: E402


class _DumpResolver:
    """The `SizeResolver` face `gac_props.dress` uses, answered from the FC
    dump's MEASURED W/D/H. Only `sx`/`sy`/`sz`/`base` are read there."""

    def __init__(self, by_usd):
        self._by = by_usd
        self.misses = []

    def get(self, usd, category, scale=1.0, axis_up="Z"):
        rec = self._by.get(usd)
        if rec is None:
            self.misses.append(usd)
            return {"sx": 0.0, "sy": 0.0, "sz": 0.0, "base": 0.0}
        return {"sx": rec["W"], "sy": rec["D"], "sz": rec["H"], "base": 0.0}


def tag_of(usd, x_m, y_m):
    """`gac_props._place`'s own `of` tag, verbatim."""
    return "%s@%.1f,%.1f" % (gac_props._name_of(str(usd)), float(x_m),
                             float(y_m))


def rect_dist(px, py, cx, cy, W, D, yaw_deg):
    a = math.radians(float(yaw_deg))
    ca, sa = math.cos(a), math.sin(a)
    dx, dy = px - cx, py - cy
    u = ca * dx + sa * dy
    v = -sa * dx + ca * dy
    return math.hypot(max(0.0, abs(u) - W / 2.0), max(0.0, abs(v) - D / 2.0))


def rebuild_props(dump, config):
    """Replay `gac_props.dress` over the dump's house placements."""
    houses = sorted(dump["placements"], key=lambda p: p["i"])
    by_usd = {p["usd"]: p for p in houses}
    resolver = _DumpResolver(by_usd)
    rng = random.Random(int(config.get("seed", 0)) + 6301)
    props = gac_props.dress(config, houses, rng, resolver=resolver)
    return houses, props, resolver


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--dump", default=os.path.join(_SG, "_plans",
                                                   "fc_dump_500.json"))
    ap.add_argument("--manifest", default=os.path.join(_SG, "_plans",
                                                       "fire_city_500m.json"))
    ap.add_argument("--preset", default="")
    ap.add_argument("--radius", type=float, default=0.0,
                    help="also REPORT (never hide) foreign-tag props within "
                         "R m of a hidden building's footprint")
    ap.add_argument("--index", type=int, default=None,
                    help="name the building a given full-list placement "
                         "index belongs to (e.g. 1354 for roof_house_94_1354)")
    a = ap.parse_args(argv)

    dump = json.load(open(a.dump))
    man = json.load(open(a.manifest))
    preset = a.preset or dump.get("preset") or man.get("preset")

    from compile_disaster import load_scene_config
    config = load_scene_config(preset, spec_overrides={"disaster-type": "none"})

    houses, props, resolver = rebuild_props(dump, config)
    n_total = int(dump["n_placements_total"])
    n_props = len(props)
    first = n_total - n_props          # props are the TAIL of the list:
    #   generate_scene.py: `placements = placements + gac_props.dress(...)`
    #   and nothing is appended after it before `return placements`.
    print("\n[probe] dump {0}: {1} house placement(s) of {2} total"
          .format(os.path.basename(a.dump), len(houses), n_total))
    print("[probe] replayed gac_props.dress -> {0} prop placement(s); they "
          "occupy full-list indices {1}..{2}".format(n_props, first,
                                                     n_total - 1))
    if resolver.misses:
        print("[probe] NOTE: resolver had no dump entry for {0} usd(s)"
              .format(len(set(resolver.misses))))
    by_cat = {}
    for p in props:
        by_cat[p["category"]] = by_cat.get(p["category"], 0) + 1
    print("[probe] props by category: {0}".format(
        ", ".join("{0}={1}".format(k, v) for k, v in sorted(by_cat.items()))))

    house_by_i = {p["i"]: p for p in houses}
    house_by_cell = {p["cell"]: p for p in houses}

    if a.index is not None:
        k = a.index - first
        if 0 <= k < n_props:
            p = props[k]
            print("\n[probe] full-list index {0} = prop #{1}: category {2!r} "
                  "at ({3:+.1f}, {4:+.1f}, {5:.1f}) belonging to {6!r}"
                  .format(a.index, k, p["category"], p["x_m"], p["y_m"],
                          p["z_m"], p["of"]))
        else:
            print("\n[probe] full-list index {0} is NOT in the prop tail "
                  "({1}..{2})".format(a.index, first, n_total - 1))

    # --- the rule: identity on the `of` tag ------------------------------
    hidden_tags = {}
    for rec in man["records"]:
        i = rec.get("i")
        p = house_by_i.get(i)
        if p is None or p["usd"] != rec.get("usd"):
            p = house_by_cell.get(rec.get("cell"))
        if p is None:
            print("[probe] *** record i={0} has no dump placement".format(i))
            continue
        hidden_tags[tag_of(p["usd"], p["x_m"], p["y_m"])] = (rec, p)

    print("\n[probe] {0} damaged building(s) -> {1} distinct `of` tag(s)"
          .format(len(man["records"]), len(hidden_tags)))
    hit = {}
    for k, p in enumerate(props):
        t = p.get("of")
        if t in hidden_tags:
            hit.setdefault(t, []).append((first + k, p))
    n_hidden_props = sum(len(v) for v in hit.values())
    print("[probe] COMPANION-HIDE RULE would hide {0} prop placement(s) "
          "across {1} of the {2} damaged building(s)\n"
          .format(n_hidden_props, len(hit), len(hidden_tags)))
    print("  {0:<4} {1:<28} {2:<5} {3:>9}  {4}".format(
        "d#", "building", "lvl", "props", "prop indices (category)"))
    for t, (rec, p) in sorted(hidden_tags.items(),
                              key=lambda kv: kv[1][0].get("i", 0)):
        rows = hit.get(t, [])
        cats = ", ".join("{0} {1}".format(i, q["category"]) for i, q in rows)
        print("  {0:<4} {1:<28} {2:<5} {3:>9}  {4}".format(
            rec.get("i"), gac_props._name_of(p["usd"]), rec.get("level"),
            len(rows), cats or "-"))

    # --- cross-check: is the tag rule missing a population? --------------
    if a.radius > 0.0:
        print("\n[probe] cross-check: FOREIGN-tag props within {0:.0f} m of a "
              "hidden building's footprint (reported, NEVER hidden)"
              .format(a.radius))
        n = 0
        for k, p in enumerate(props):
            if p.get("of") in hidden_tags:
                continue
            for t, (rec, h) in hidden_tags.items():
                d = rect_dist(p["x_m"], p["y_m"], h["x_m"], h["y_m"],
                              h["W"], h["D"], h["yaw_deg"])
                if d <= a.radius:
                    n += 1
                    print("  {0:<5} {1:<12} at ({2:+7.1f},{3:+7.1f}) is "
                          "{4:5.1f} m from d{5} — belongs to {6!r}".format(
                              first + k, p["category"], p["x_m"], p["y_m"], d,
                              rec.get("i"), p.get("of")))
                    break
        print("  {0} foreign-tag prop(s) inside {1:.0f} m".format(n, a.radius))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
