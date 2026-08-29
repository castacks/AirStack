#!/usr/bin/env python3
"""urban_fire_dryrun.py — solve an urban fire over a REAL generated layout,
on the host, without Isaac Sim.

    python3 scene_gen/tools/urban_fire_dryrun.py --config downtown_gac \
        --region 500 --elapsed 40,70,100,140,190

WHY THIS EXISTS
---------------
"Moderate" is not a number anyone can pick from a desk. The fire's reach is
set by the GAP DISTRIBUTION of the layout it runs on — `urban_fire_spread`
crosses <1.2 m by conduction in minutes, 1-13 m by radiation in tens of
minutes, and 8-55 m only by a downwind brand — so the same elapsed time reads
as a single burning building on one plat and half the downtown on another.
The only way to set the clock is to solve it against the layout that will
actually be built.

`tools/plan_png.py` already proved `build_city` runs host-side with `pxr`
stubbed and the footprints answered from the measured comments and
`_plans/*.json`; this reuses that machinery (`plan_png.build`) and adds the
spread solve on top. One second, no container, no GPU.

WHAT IT REPORTS
---------------
  * the layout: blocks, buildings, and the GAP HISTOGRAM broken down by which
    spread mechanism each pair falls into — this is the graph, and it is the
    thing that decides everything else;
  * per elapsed time, the F0..F5 tally and how many buildings were reached.

Run it BEFORE a launch, and again after any layout change: a retune of
`block_short_m` or `building_gap_m` moves the fire as surely as moving the
clock does.
"""

import argparse
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
sys.path.insert(0, _SCENE_GEN)
sys.path.insert(0, _HERE)


def _load(config, region, seed):
    """`plan_png.build`, with a region / seed override applied to the spec."""
    import types
    import yaml
    import plan_png                      # stubs pxr at import; see its header
    from compile_disaster import resolve_config_path, compile_spec, DEFAULT_BASE
    import scene_generator as sg
    from layout import city_layout
    from detail import districts

    path = resolve_config_path(config)
    spec = yaml.safe_load(open(path))
    if region:
        spec["region_m"] = [float(region), float(region)]
    if seed is not None:
        spec["seed"] = int(seed)
    cfg = compile_spec(spec, yaml.safe_load(open(DEFAULT_BASE)))
    cfg = sg.resolve_asset_set(cfg, path)
    cfg["measure_usds"] = False

    sets = os.path.join(_SCENE_GEN, "config", "asset_sets")
    sizes = plan_png.measured_sizes([os.path.join(sets, f)
                                     for f in os.listdir(sets)
                                     if f.endswith(".yaml")])
    plans = os.path.join(_SCENE_GEN, "_plans")
    js = {}
    js.update(plan_png.measured_json(
        [os.path.join(plans, "gac_buildings.json")], default_ext=".usd"))
    js.update(plan_png.measured_json([os.path.join(plans, "gac_faces.json")]))
    js.update(plan_png.measured_json([os.path.join(plans, "dtc_faces.json")]))
    res = plan_png.StubResolver(sizes, cfg.get("fallback_sizes"), js)
    rng = random.Random(int(cfg.get("seed", 0)) + 7717)
    with city_layout.patched(cfg):
        placements, layout = sg.build_city(cfg, res)
    da, rings = districts.assign(cfg, layout)
    if rings:
        districts.remap_buildings(cfg, layout, placements, res, rng, da)
    return cfg, layout, placements, res


def buildings_of(placements, res, category="house"):
    """The spread model's building records, in placement order.

    W/D are the asset's own extents at its placement scale, NOT the world
    AABB: `urban_fire_spread._corners` rotates them by `yaw` itself, so
    handing it a world box would rotate an already-rotated footprint.
    """
    out = []
    for i, p in enumerate(placements):
        if p.get("category") != category:
            continue
        fp = res.get(p["usd"], "house", scale=float(p.get("scale", 1.0)),
                     axis_up=p.get("axis_up", "Z"))
        out.append({"idx": i, "x": float(p["x_m"]), "y": float(p["y_m"]),
                    "W": float(fp["sx"]), "D": float(fp["sy"]),
                    "H": float(fp["sz"]), "yaw": float(p.get("yaw_deg", 0.0)),
                    "style": os.path.basename(str(p["usd"])).rsplit(".", 1)[0],
                    "usd": p["usd"]})
    return out


def gap_report(bl, ufs, wind_dir, wind_mps):
    """How many ordered pairs each mechanism can carry. THE GRAPH IS THE SCENE."""
    n = len(bl)
    att = rad = spot = 0
    nearest = []
    for i in range(n):
        best = 1e9
        for j in range(n):
            if i == j:
                continue
            g = ufs.gap_m(bl[i], bl[j])
            best = min(best, g)
            if g <= ufs.ATTACHED_GAP_M:
                att += 1
            elif g <= ufs.RAD_REACH_M:
                rad += 1
            elif ufs.SPOT_MIN_M <= g <= ufs.SPOT_REACH_M:
                if math.cos(ufs.bearing(bl[i], bl[j]) - wind_dir) >= 0.35 \
                        and wind_mps >= 1.0:
                    spot += 1
        nearest.append(best)
    nearest.sort()
    q = lambda f: nearest[min(len(nearest) - 1, int(f * len(nearest)))]
    isolated = sum(1 for g in nearest if g > ufs.SPOT_REACH_M)
    print("[dryrun] spread graph: {0} attached, {1} radiation, {2} downwind-spot "
          "ordered pair(s)".format(att, rad, spot))
    print("[dryrun] nearest-neighbour gap  p10 {0:.1f}  median {1:.1f}  "
          "p90 {2:.1f} m;  {3} building(s) beyond every mechanism's reach"
          .format(q(0.10), q(0.50), q(0.90), isolated))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="downtown_gac")
    ap.add_argument("--region", type=float, default=500.0)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--elapsed", default="",
                    help="minutes since ignition, comma separated. Default: "
                         "the times solved for each LADDER rung, which is "
                         "what the launcher will actually build")
    # DEFAULTS THAT MATCH THE LAUNCHER, or this predicts a scene nobody will
    # build. The wind is the LADDER's; the seed is `downtown_fire_launch_
    # script.py`'s `UF_SEED`, and `assemble` offsets it by 991 internally, so
    # the same offset is applied here rather than to the flag's default.
    ap.add_argument("--wind", default="20,8",
                    help="'<deg>,<m/s>' — the direction it blows TOWARD. "
                         "Saturates at 8 m/s (urban_fire_spread._wind_factor)")
    ap.add_argument("--fire-seed", type=int, default=21,
                    help="the launcher's UF_SEED")
    ap.add_argument("--origin", default="",
                    help="'x,y' metres — the point the ignition search is "
                         "centred on (default: the upwind quarter-point)")
    a = ap.parse_args()

    cfg, layout, placements, res = _load(a.config, a.region, a.seed)
    from disaster import urban_fire_city as ufc
    from disaster import urban_fire_spread as ufs

    bl = buildings_of(placements, res)
    blocks = len(layout.get("blocks") or [])
    print("[dryrun] {0} at {1:.0f} m: {2} block(s), {3} building(s), "
          "{4} distinct model(s)".format(a.config, a.region, blocks, len(bl),
                                         len({b['style'] for b in bl})))
    if not bl:
        return 1
    wd, wm = [float(v) for v in a.wind.split(",")]
    wind_dir = math.radians(wd)
    gap_report(bl, ufs, wind_dir, wm)

    # THE SAME IGNITION RULE THE BUILD USES, imported rather than
    # reimplemented — a dry run that picks a different ignition point predicts
    # a different scene, which is worse than no dry run at all. It puts the
    # search upwind of centre (on the edge half the brands blow off the plate;
    # dead centre the fire runs out of downwind city) and then takes the
    # best-connected candidate inside it.
    org = None
    if a.origin:
        org = tuple(float(v) for v in a.origin.split(","))
    ign = ufc.ignition_index(bl, wind_dir, a.region, org)
    deg = sum(1 for j in range(len(bl))
              if j != ign and ufs.gap_m(bl[ign], bl[j]) <= ufs.RAD_REACH_M)
    print("[dryrun] ignition: {0} at ({1:.0f}, {2:.0f}), H {3:.0f} m, "
          "{4} neighbour(s) in radiation reach; wind {5:.0f} deg @ {6:.0f} m/s"
          .format(bl[ign]["style"], bl[ign]["x"], bl[ign]["y"], bl[ign]["H"],
                  deg, wd, wm))

    if a.elapsed.strip():
        times = [(None, float(v)) for v in a.elapsed.split(",") if v.strip()]
    else:
        times = []
        for nm in ("light", "moderate", "severe"):
            mins, got, reach = ufc._time_for_fraction(
                bl, ign, ufc.LADDER[nm]["involved_frac"], wind_dir, wm,
                a.fire_seed)
            times.append((nm, mins))
        print("[dryrun] reachable from this ignition at all: {0} of {1} "
              "({2:.0%})".format(reach, len(bl), reach / float(len(bl))))

    print("\n  {0:<9} {1:>7}  {2:>7}  {3}".format(
        "rung", "T+min", "reached", "F0..F5"))
    for nm, mins in times:
        plan = ufs.solve(bl, ign, mins * 60.0, wind_dir=wind_dir, wind_mps=wm,
                         rng=random.Random(a.fire_seed + 991),
                         btype_of=ufc._btype)
        tally = {}
        for p in plan:
            tally[p["level"]] = tally.get(p["level"], 0) + 1
        reached = sum(1 for p in plan if p["t_ignite"] is not None)
        involved = len(bl) - tally.get("F0", 0)
        print("  {0:<9} {1:>7.0f}  {2:>3d}/{3:<3d}  {4}   involved {5} "
              "({6:.0%})".format(nm or "-", mins, reached, len(bl),
                      "  ".join("{0}={1}".format(k, tally[k])
                                for k in ufs.LEVELS if k in tally),
                      involved, involved / float(len(bl))))
    return 0


if __name__ == "__main__":
    sys.exit(main())
