#!/usr/bin/env python3
"""hurricane_debris_plot.py — the DEBRIS stream's offline gate, no Isaac Sim.

    python3 tools/hurricane_debris_plot.py --level l3 \\
        --gt ~/hurricane_previews/V2_L3/GT_hurricane.json \\
        --out ~/hurricane_previews/offline/debris/debris_L3.png

WHY THIS EXISTS
---------------
The same trade `tornado_png.py`/`fire_png.py` make: the questions that
decide whether the 2026-08-31 debris fixes (draft/attitude, wet darkening,
the coarse hurricane stock mix, the waterline strand line, obstruction
clustering, and the previously-dead land debris pass) are working are all
pure geometry and arithmetic, answerable in well under a second, and wrong
in a way no unit test alone makes vivid.

It runs the REAL code path -- `disaster.washaway.raft_specs` and
`disaster.washaway.land_debris_specs`, unmodified, fed the SAME
`scfg`/`hcfg` construction `suburb_hurricane_launch_script.py` uses
(`compile_disaster.load_scene_config`, `hurricane.resolve_cfg`,
`surge.resolve_cfg` layered the same way) and the SAME seed (11) -- so a
number that changes here is a number that will change in the next Isaac
render, not a divergent reimplementation.

HOUSE/TREE POSITIONS COME FROM A PAST RUN'S GROUND TRUTH
---------------------------------------------------------
`suburb_scene.generate_suburb_on_stage` needs a live USD stage; reproducing
its house/tree layout offline is out of scope for a debris review. Instead
this tool reads `~/hurricane_previews/V2_L{2,3}/GT_hurricane.json`, which
already recorded every house's and tree's `(x, y, level, intensity, ...)`
from an actual run at `SEED=11` against these same presets. Per the task
brief: "another agent is re-cutting the house ladder -- treat the levels as
illustrative." Positions and the wind/surge field itself are still exact;
only the DAMAGE LEVEL distribution may drift from whatever is on disk today.

CARS ARE NOT INCLUDED. The launcher finds cars by walking the assembled
STAGE (its own comment: "`info_out` publishes only cars/clusters/... and
`cars` among them is a comment rather than an assignment"), so there is no
offline position list for them; `raft_specs(obstacles=...)` here carries
TREES only. Noted as a gap in the final report, not silently dropped.

WHAT IT IS NOT
---------------
A render. There is no water shading realism here beyond a flat depth-based
tint; it exists to verify PLACEMENT (draft invariants, strand-line density,
obstruction exclusion, land-debris gating/reach) and PIECE COUNTS, not
colour.
"""

import argparse
import collections
import json
import math
import os
import sys
import types

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
sys.path.insert(0, _HERE)
sys.path.insert(0, _SCENE_GEN)

# `scene_generator`/`suburb_scene`/`detail.modular_house` import `pxr` at
# module scope (needed here only for `suburb_scene.modular_catalogue`'s
# per-style footprint table); `disaster.hurricane`/`disaster.surge`/
# `disaster.washaway`/`disaster.planks` do not. Stub every submodule any of
# them touch, the same idiom `tornado_png.py:40-56` uses, widened to cover
# `UsdSkel`/`Vt` which `scene_generator` also imports at module scope.
for _m in ("pxr", "pxr.Gf", "pxr.Sdf", "pxr.Usd", "pxr.UsdGeom",
          "pxr.UsdShade", "pxr.UsdSkel", "pxr.Vt", "pxr.Ar"):
    sys.modules.setdefault(_m, types.ModuleType(_m))
for _n in ("Gf", "Sdf", "Usd", "UsdGeom", "UsdShade", "UsdSkel", "Vt", "Ar"):
    setattr(sys.modules["pxr"], _n, types.SimpleNamespace())

import numpy as np                                              # noqa: E402
import matplotlib                                                # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                  # noqa: E402
from matplotlib.patches import Circle                             # noqa: E402

import suburb_scene as ss                                        # noqa: E402
from compile_disaster import load_scene_config                    # noqa: E402
from disaster import hurricane as hu                              # noqa: E402
from disaster import surge as sgw                                 # noqa: E402
from disaster import washaway as wash                             # noqa: E402
from disaster import planks                                        # noqa: E402

SEED = 11
TREE_FOOTPRINT_M = 3.0     # canopy diameter fallback -- GT has no per-tree
                            # size, matching `washaway._footprint_list`'s
                            # own generic obstacle default order of magnitude

_KIND_COLOUR = {
    "vegetation": "#2f6b1f", "log": "#5a3b1e",
    "timber": "#d8c9a0", "sheet": "#b0a58a", "wall": "#c7c2b5",
    "panel": "#cbb98d", "fence": "#8f8770", "siding_strip": "#dcd6c8",
    "roof_top": "#3a332c", "roof_under": "#c2ac82",
}
_LEVEL_COLOUR = {"pristine": "#2f7d32", "shingles_lost": "#8bbf4a",
                 "cover_lost": "#c9d14a", "deck_panels_lost": "#e0c23d",
                 "roof_stripped": "#e8a33d", "roof_collapsed": "#d4622c",
                 "partial_collapse": "#b8402a", "leveled": "#96261f",
                 "swept": "#3d0f0c"}


def _resolve_scfg(config, region):
    """EXACT copy of `suburb_hurricane_launch_script.py`'s `scfg`
    construction (lines ~528-534) -- preset sub-block layered over
    `surge.DEFAULTS`, env overrides in between, preset winning last."""
    hsub = ((config.get("disaster") or {}).get("hurricane") or {})
    scfg = sgw.resolve_cfg({k: v for k, v in hsub.items()
                            if k in sgw.DEFAULTS})
    span = max(region[2] - region[0], region[3] - region[1])
    scfg.update(sgw.knobs_from_env(span))
    for k, v in hsub.items():
        if k in sgw.DEFAULTS:
            scfg[k] = v
    return scfg


def _fp_by_style(config):
    return {e["style"]: max(e["w"], e["d"])
           for e in ss.modular_catalogue(config)}


def load_ground_truth(path):
    with open(os.path.expanduser(path)) as f:
        return json.load(f)


def build_field(preset_name, gt_path):
    """Everything the debris passes need, reproduced exactly as the
    launcher builds it, house/tree POSITIONS from `gt_path`. Returns a dict
    with `region`, `hcfg`, `scfg`, `houses`, `trees`, `wrecks`, `depth_fn`,
    `wind_fn`, `fp_by_style`."""
    config = load_scene_config(preset_name)
    gt = load_ground_truth(gt_path)
    region = tuple(gt["region"])
    hcfg = hu.resolve_cfg(config)
    scfg = _resolve_scfg(config, region)
    fp_by_style = _fp_by_style(config)

    houses = gt["houses"]
    trees = gt["trees"]
    depth_fn = sgw.depth_at(scfg, region, np.random.default_rng(SEED + 41))
    wind_fn = lambda x, y: hu.wind_bearing_at(hcfg, x, y)          # noqa: E731

    wrecks = [(h["x"], h["y"], fp_by_style.get(h["style"], 12.0),
              h["intensity"], h["level"], h.get("palette"))
             for h in houses if h["level"] != "pristine"]

    return dict(config=config, region=region, hcfg=hcfg, scfg=scfg,
               fp_by_style=fp_by_style, houses=houses, trees=trees,
               wrecks=wrecks, depth_fn=depth_fn, wind_fn=wind_fn)


def build_debris(field, seed=SEED):
    """Runs the actual `washaway` code path -- `raft_specs` (water) and
    `land_debris_specs` (land) -- with the SAME `debris_mix`-derived kind
    weights the launcher's raft call already threads through, and returns
    `(rafts, land)` spec lists plus the rng streams' seeds used, printed so
    a rerun with the same `--seed` is byte-identical.

    `rafts` INCLUDES `land_debris_specs`'s own SUBMERGED conversions (D2
    review, "LAND DEBRIS ON SUBMERGED GROUND"): a land-debris piece whose
    landing point is under real water is pulled into the raft population
    rather than sitting invisibly on the flooded ground, and merged in here
    so every downstream consumer (`report_counts`, `_classify_zone`,
    `plot_level`) treats it exactly like any other raft -- it IS one."""
    import random as _random

    scfg, region = field["scfg"], field["region"]
    wrng = _random.Random(seed + 61)

    dm = hu.debris_mix(0.70, wrng)
    veg = sum(v for k, v in dm.items()
             if any(w in k for w in ("leaf", "limb", "frond", "veg")))
    kw = wash.raft_kind_weights(veg)

    wcfg = wash.resolve_cfg(field["config"])
    wcfg["water_level_m"] = sgw.water_level(scfg)

    # ALL houses (pristine included), matching the launcher's own
    # `_h_recs` list (every successfully-referenced house, not only the
    # damaged ones) -- WITH a real per-style footprint, which is this
    # stream's fix to the launcher's own call (it previously passed bare
    # `(x, y)` and every house fell back to `raft_specs`'s generic 10 m
    # default regardless of style).
    houses_xy = [(h["x"], h["y"], field["fp_by_style"].get(h["style"], 12.0))
                for h in field["houses"]]
    obstacles = [(t["x"], t["y"], TREE_FOOTPRINT_M) for t in field["trees"]]

    # `wind_bearing_fn`, ADDED for the DENSITY pass (2026-08-31): threads the
    # same `hu.wind_bearing_at` closure `field["wind_fn"]` already carries
    # into `raft_specs`'s new mid-water drift-line pass, matching the
    # launcher's own "# DEBRIS RAFTS" block so this tool's counts are the
    # numbers that render, not a divergent reimplementation missing the new
    # structure entirely.
    rafts = wash.raft_specs(wcfg, region, wrng, houses_xy, field["depth_fn"],
                            kind_weights=kw, obstacles=obstacles,
                            wind_bearing_fn=field["wind_fn"])

    # LAND DEBRIS -- `HUR_DEBRIS`'s dead knob, brought alive here for the
    # first time. `SEED + 97` matches the launcher-edit convention this
    # stream proposes (see `suburb_hurricane_launch_script.py`'s
    # "# DEBRIS RAFTS" block). `depth_fn`/`water_level_m` wired through so a
    # comet that lands on submerged ground converts to a raft instead of
    # sitting on the sea floor under the flood.
    lrng = _random.Random(seed + 97)
    land, land_rafts = wash.land_debris_specs(
        field["wrecks"], field["wind_fn"], lrng, depth_fn=field["depth_fn"],
        water_level_m=wcfg["water_level_m"])
    rafts = rafts + land_rafts

    return rafts, land, kw


def _classify_zone(spec, scfg, region, depth_fn, houses, trees):
    """Post-hoc zone label for reporting: `raft_specs`/`land_debris_specs`
    do not tag provenance on the spec itself, so a raft's zone is inferred
    from where it ended up, the same measurement a reviewer would make by
    eye. `land_debris_specs` output is trivially `"land"`.

    `"strand line"`, ADDED for the D2 review: depth below `raft_min_depth_m`
    is the TRUE waterline `_strand_line_specs` marches (previously a dead
    band with nothing in it at all -- see that function's own docstring),
    distinct from `"waterline"`, the pre-existing boosted BAND a little
    further out (`[raft_min_depth_m, raft_min_depth_m + raft_waterline_
    band_m]`)."""
    x, y = spec["x"], spec["y"]
    d = depth_fn(x, y)
    kn = wash.resolve_cfg({})
    band = kn["raft_waterline_band_m"]
    min_depth = kn["raft_min_depth_m"]
    for hx, hy, hfp in _all_footprints(houses, trees):
        if math.hypot(x - hx, y - hy) <= 0.5 * hfp + 2.5:
            return "against obstruction"
    if d < min_depth:
        return "strand line"
    if min_depth <= d <= min_depth + band:
        return "waterline"
    return "open water"


def _all_footprints(houses, trees):
    for h in houses:
        yield h["x"], h["y"], 12.0
    for t in trees:
        yield t["x"], t["y"], TREE_FOOTPRINT_M


def report_counts(level_name, rafts, land):
    by_kind = collections.Counter(s["kind"] for s in rafts)
    print("[hurricane_debris_plot] {0} rafts: {1} total, by kind: {2}"
         .format(level_name, len(rafts), dict(sorted(by_kind.items()))))
    by_kind_land = collections.Counter(s["class"] for s in land)
    print("[hurricane_debris_plot] {0} land debris: {1} total, by class: "
         "{2}".format(level_name, len(land), dict(sorted(by_kind_land.items()))))


def shoreline_length_m(field, grid_n=400, trigger_m=0.0):
    """Shoreline contour length via grid boundary-crossing count x cell
    size -- the "count boundary cells x cell size" measurement the D2
    review names, used to turn the strand-line piece count into a linear
    density (pieces per metre of shoreline) for `plot_level`'s title."""
    region = field["region"]
    depth_fn = field["depth_fn"]
    x0, y0, x1, y1 = region
    xs = [x0 + (x1 - x0) * i / grid_n for i in range(grid_n + 1)]
    ys = [y0 + (y1 - y0) * j / grid_n for j in range(grid_n + 1)]
    dx = (x1 - x0) / grid_n
    dy = (y1 - y0) / grid_n
    wet = [[depth_fn(x, y) > trigger_m for x in xs] for y in ys]
    length = 0.0
    for j in range(grid_n + 1):
        for i in range(grid_n):
            if wet[j][i] != wet[j][i + 1]:
                length += dy
    for i in range(grid_n + 1):
        for j in range(grid_n):
            if wet[j][i] != wet[j + 1][i]:
                length += dx
    return length


def land_comet_stats(field, seed=SEED, cone_deg=35.0):
    """Per-house mean downwind displacement and +-`cone_deg` cone fraction,
    plate-wide MEDIANS -- the D2 review's "ring, not a comet" measurement.

    `land_debris_specs` does not tag its output by source house, and a
    NEAREST-WRECK re-grouping of the plain `land` list is unsound now that
    the tail reaches tens of metres: a piece thrown toward open yard from
    one house can land closer to an unrelated NEIGHBOUR than to its own
    source, which corrupts exactly the per-house direction statistic this
    function exists to report. So this reproduces `land_debris_specs`'s own
    per-house loop directly (same gate, same `_reach_cap`, same rng
    consumption when called with the same `seed` the real pass used) and
    tags every piece with the house that threw it -- ground truth, not an
    inference. Computed over the FULL originally-scattered comet (before
    any submerged-ground conversion, correction #3): the "ring vs comet"
    shape is a placement-algorithm property independent of how many pieces
    later turn out to be over water.
    """
    import random as _random

    wrecks = field["wrecks"]
    if not wrecks:
        return dict(n_houses=0, n_pieces=0, median_downwind_m=float("nan"),
                   median_cone_frac=float("nan"))
    wind_fn = field["wind_fn"]
    rng = _random.Random(seed + 97)
    order = list(wash._HOUSE_LEVEL_ORDER)
    gate = order.index("cover_lost")
    footprints = [(float(w[0]), float(w[1]),
                  float(w[2]) if len(w) > 2 else 12.0) for w in wrecks]

    downwinds, cone_fracs = [], []
    n_pieces_total = 0
    for i, w in enumerate(wrecks):
        level = w[4] if len(w) > 4 else "cover_lost"
        if level not in order or order.index(level) < gate:
            continue
        hx, hy = float(w[0]), float(w[1])
        fp = float(w[2]) if len(w) > 2 else 12.0
        it = max(0.0, min(1.0, float(w[3]) if len(w) > 3 else 0.5))
        classes = wash.LAND_DEBRIS_CLASSES.get(level, tuple(planks.STOCK))
        heading = float(wind_fn(hx, hy))

        others = [f for j, f in enumerate(footprints) if j != i]
        cap = wash._reach_cap(hx, hy, heading, others)
        reach = (min(wash.LAND_DEBRIS_REACH_M, cap) if cap is not None
                 else wash.LAND_DEBRIS_REACH_M)
        reach = max(wash.LAND_DEBRIS_MIN_REACH_M, reach)
        n_target = wash.LAND_DEBRIS_N_LO + (
            wash.LAND_DEBRIS_N_HI - wash.LAND_DEBRIS_N_LO) * it
        denom = max(1e-6, 0.25 + 0.75 * it)
        n_pieces = max(1, int(round(n_target / denom)))

        pieces = planks.scatter_from_wreck(
            hx, hy, fp, it, heading, reach, rng, n_pieces=n_pieces,
            ground_z=float(wash.LAND_DEBRIS_GROUND_Z_M), classes=classes,
            tail_pow=wash.LAND_DEBRIS_TAIL_POW,
            tail_lateral_base=wash.LAND_DEBRIS_TAIL_LATERAL_BASE,
            tail_lateral_growth=wash.LAND_DEBRIS_TAIL_LATERAL_GROWTH)
        n_pieces_total += len(pieces)
        if not pieces:
            continue
        th = math.radians(heading)
        ux, uy = math.cos(th), math.sin(th)
        s_list, in_cone = [], 0
        for p in pieces:
            dx, dy = p["x"] - hx, p["y"] - hy
            s = dx * ux + dy * uy
            t = -dx * uy + dy * ux
            ang = math.degrees(abs(math.atan2(t, s)))
            s_list.append(s)
            if ang <= cone_deg:
                in_cone += 1
        downwinds.append(sum(s_list) / len(s_list))
        cone_fracs.append(in_cone / len(s_list))

    def median(xs):
        xs = sorted(xs)
        n = len(xs)
        if n == 0:
            return float("nan")
        return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])

    return dict(n_houses=len(downwinds), n_pieces=n_pieces_total,
               median_downwind_m=median(downwinds),
               median_cone_frac=median(cone_fracs))


def plot_level(level_name, field, rafts, land, out_path, grid_n=180,
              seed=SEED):
    region = field["region"]
    x0, y0, x1, y1 = region
    depth_fn = field["depth_fn"]

    xs = np.linspace(x0, x1, grid_n)
    ys = np.linspace(y0, y1, grid_n)
    depth_grid = np.zeros((grid_n, grid_n), dtype=np.float32)
    for j, yy in enumerate(ys):
        for i, xx in enumerate(xs):
            depth_grid[j, i] = depth_fn(xx, yy)
    wet = depth_grid > 1e-6

    fig, ax = plt.subplots(figsize=(11, 11), dpi=140)
    ax.set_facecolor("#e8e4d8")
    water_rgba = np.zeros((grid_n, grid_n, 4), dtype=np.float32)
    water_rgba[..., 0] = 0.42
    water_rgba[..., 1] = 0.34
    water_rgba[..., 2] = 0.22
    water_rgba[..., 3] = np.clip(depth_grid / max(1e-6, depth_grid.max()),
                                 0.15, 0.65) * wet
    ax.imshow(water_rgba, origin="lower", extent=(x0, x1, y0, y1),
             interpolation="nearest", zorder=1)

    for h in field["houses"]:
        colour = _LEVEL_COLOUR.get(h["level"], "#777777")
        ax.add_patch(Circle((h["x"], h["y"]), 5.0, facecolor=colour,
                            edgecolor="black", linewidth=0.2, zorder=3))
    for t in field["trees"]:
        ax.plot(t["x"], t["y"], marker=".", color="#1f5c17",
               markersize=1.5, zorder=2)

    zone_counts = collections.Counter()
    for s in rafts:
        zone = _classify_zone(s, field["scfg"], region, depth_fn,
                              field["houses"], field["trees"])
        zone_counts[(s["kind"], zone)] += 1
        ax.plot(s["x"], s["y"], marker="s",
               color=_KIND_COLOUR.get(s["kind"], "#ff00ff"),
               markersize=3.6 if zone == "strand line" else
               (2.2 if zone != "waterline" else 3.2),
               markeredgewidth=0, zorder=4,
               alpha=0.9 if zone == "against obstruction" else 0.75)

    for s in land:
        zone_counts[(s["class"], "land")] += 1
        ax.plot(s["x"], s["y"], marker="D", color="#7a5230",
               markersize=1.8, markeredgewidth=0, zorder=5, alpha=0.85)

    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")

    # D2 REVIEW MEASUREMENTS -- shoreline linear density (correction #2) and
    # the land-debris comet's cone fraction (correction #1), both in the
    # title per the review's own instruction ("report ... in the plot
    # title"). `shore_len`/`strand_n` feed the density; `land_comet_stats`
    # groups `land` by nearest wreck since `land_debris_specs` itself does
    # not tag provenance on its output.
    shore_len = shoreline_length_m(field)
    # PER-ZONE COUNTS, ADDED for the DENSITY pass (2026-08-31, user: "the
    # debris in the flooded area needs to increase a lot in number") --
    # collapsed from the (kind, zone) `zone_counts` ALREADY built by the
    # scatter loop above, rather than a second `_classify_zone` pass over
    # every raft (that call walks every house/obstacle footprint per piece,
    # real cost at the new 10-15k-piece counts). `strand_n` below is now
    # read from the same collapse instead of its own separate full pass.
    per_zone = collections.Counter()
    for (_kind, zone), n in zone_counts.items():
        per_zone[zone] += n
    strand_n = per_zone.get("strand line", 0)
    comet = land_comet_stats(field, seed=seed)
    zone_str = ", ".join("{0} {1}".format(z, per_zone.get(z, 0))
                         for z in ("open water", "waterline", "strand line",
                                   "against obstruction"))
    ax.set_title(
        "{0} -- rafts {1} ({9}), land debris {2}, water {3:.1%}\n"
        "shoreline {4:.0f} m, strand-line {5} px (1 per {6:.2f} m) -- "
        "land comet: median downwind {7:.1f} m, cone(+-35 deg) frac {8:.2f}"
        .format(level_name, len(rafts), len(land), float(np.mean(wet)),
               shore_len, strand_n, shore_len / max(1, strand_n),
               comet["median_downwind_m"], comet["median_cone_frac"],
               zone_str),
        fontsize=9)
    handles = [plt.Line2D([0], [0], marker="s", linestyle="",
                          color=_KIND_COLOUR[k], label=k)
              for k in sorted(_KIND_COLOUR)]
    handles.append(plt.Line2D([0], [0], marker="D", linestyle="",
                              color="#7a5230", label="land debris"))
    ax.legend(handles=handles, loc="upper right", fontsize=6, ncol=2,
             framealpha=0.85)

    out_path = os.path.expanduser(out_path)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)
    print("[hurricane_debris_plot] wrote {0}".format(out_path))

    print("[hurricane_debris_plot] {0} zone breakdown:".format(level_name))
    for (kind, zone), n in sorted(zone_counts.items()):
        print("    {0:<14s} {1:<20s} {2}".format(kind, zone, n))
    return zone_counts


_PRESET_BY_LEVEL = {"l2": "suburb_hurricane_500_l2",
                    "l3": "suburb_hurricane_500_l3"}
_GT_BY_LEVEL = {"l2": "~/hurricane_previews/V2_L2/GT_hurricane.json",
               "l3": "~/hurricane_previews/V2_L3/GT_hurricane.json"}
_OUT_BY_LEVEL = {"l2": "~/hurricane_previews/offline/debris/debris_L2.png",
                 "l3": "~/hurricane_previews/offline/debris/debris_L3.png"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--level", choices=("l2", "l3", "both"), default="both")
    ap.add_argument("--gt", default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--seed", type=int, default=SEED)
    args = ap.parse_args()

    levels = ("l2", "l3") if args.level == "both" else (args.level,)
    for lvl in levels:
        preset = _PRESET_BY_LEVEL[lvl]
        gt_path = args.gt or _GT_BY_LEVEL[lvl]
        out_path = args.out or _OUT_BY_LEVEL[lvl]
        print("[hurricane_debris_plot] === {0} ({1}) ===".format(lvl, preset))
        field = build_field(preset, gt_path)
        rafts, land, kw = build_debris(field, seed=args.seed)
        report_counts(lvl, rafts, land)
        plot_level(lvl.upper(), field, rafts, land, out_path, seed=args.seed)


if __name__ == "__main__":
    main()
