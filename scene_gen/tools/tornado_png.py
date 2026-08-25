"""tornado_png.py — the tornado plan, without Isaac Sim.

    python3 tools/tornado_png.py --config suburb_tornado --out _plans/tornado.png

WHY THIS EXISTS
---------------
The same trade `fire_png.py` makes, for the same reason. The questions that
decide whether a tornado scene is worth a twenty-minute container launch are
all pure geometry:

    Does the track actually cross the fabric, or does it clip a corner?
    How many houses are in it, and are they spread across the damage ladder
      or all in one class?
    Is there intact suburb left on BOTH sides of the corridor?
    How many trees come down, and how many boards is that?

Every one of those is answerable in about a second, and getting any of them
wrong is invisible until the scene is built. `tornado.jpeg` in the repo root
is what the answer is supposed to look like: a swathe with green either side.

It runs the REAL code path — `fence_png.build` for the placements and
`disaster.tornado`'s own field and ladders — so the tallies here are the ones
the launchers will produce, not a reimplementation that can drift.

WHAT IT IS NOT
--------------
A render. There is no wreckage in it; it shows WHERE the damage classes fall.
Fuel and tree counts run low against the real scene for the same reason
`fire_png` records: `fence_png.build` stops before the on-stage yard planting,
so yard trees are missing here and present in Isaac Sim. Treat the tree counts
as a floor and the house counts as exact.
"""

import argparse
import collections
import math
import os
import random
import sys
import types

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
sys.path.insert(0, _HERE)
sys.path.insert(0, _SCENE_GEN)

# `disaster.tornado` imports numpy at module scope and pxr nowhere; `planks`
# and `ground` do reach for pxr, but only inside the authoring functions this
# tool never calls. Stub it the way `fire_png` does so the import cannot fail
# on a host with no USD.
for _m in ("pxr", "pxr.Gf", "pxr.Sdf", "pxr.Usd", "pxr.UsdGeom"):
    sys.modules.setdefault(_m, types.ModuleType(_m))
for _n in ("Gf", "Sdf", "Usd", "UsdGeom"):
    setattr(sys.modules["pxr"], _n, types.SimpleNamespace())

import numpy as np                                             # noqa: E402
import matplotlib                                              # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                # noqa: E402

import fence_png                                               # noqa: E402
from disaster import planks, tornado as tn                     # noqa: E402

# Pale for light damage through to dark for a swept slab, so the plot reads
# the way the ladder does rather than by hue lookup.
_HCOLOUR = {"pristine": "#2f7d32", "roof_stripped": "#c9d14a",
            "roof_collapsed": "#e8a33d", "partial_collapse": "#d4622c",
            "leveled": "#96261f", "swept": "#3d0f0c"}
_TCOLOUR = {"pristine": "#2f7d32", "limbed": "#8bbf4a", "leaning": "#c9b23d",
            "fallen": "#a8632a", "snapped": "#5a2f17"}


def _house_xy(placements):
    """Every house placement as `(x, y, footprint_m)`.

    Footprint is the longer plan dimension where the placement records one and
    a 12 m nominal where it does not — it is only used to size the debris mat,
    and being a metre out on that changes nothing anyone can see.
    """
    out = []
    for q in placements:
        cat = str(q.get("category", ""))
        if "house" not in cat or "floor" in cat or "wall" in cat:
            continue
        fp = max(float(q.get("w_m", 0.0) or 0.0),
                 float(q.get("d_m", 0.0) or 0.0)) or 12.0
        out.append((float(q.get("x_m", 0.0)), float(q.get("y_m", 0.0)), fp))
    return out


def _tree_xy(placements):
    return [(float(q.get("x_m", 0.0)), float(q.get("y_m", 0.0)))
            for q in placements
            if "tree" in str(q.get("category", ""))]


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="suburb_tornado",
                    help="preset name or path (default: suburb_tornado)")
    ap.add_argument("--seed", type=int, default=None,
                    help="override the layout seed")
    ap.add_argument("--out", default="_plans/tornado.png")
    args = ap.parse_args()

    scene = fence_png.build(seed=args.seed, config_name=args.config)
    cfg_all = scene["cfg"]
    tcfg = tn.resolve_cfg(cfg_all)

    reg = (cfg_all.get("layout") or {}).get("region_m") or [500.0, 500.0]
    rw, rh = float(reg[0]), float(reg[1])
    region = (-rw / 2.0, -rh / 2.0, rw / 2.0, rh / 2.0)
    seed = int(scene["seed"])

    inten = tn.intensity_field(tcfg, region, np.random.default_rng(seed + 23))
    rng = random.Random(seed + 5)

    houses = _house_xy(scene["placements"])
    trees = _tree_xy(scene["placements"])
    h_lv, t_lv = [], []
    for (x, y, _fp) in houses:
        h_lv.append(tn.house_level_for_intensity(inten(x, y), rng))
    for (x, y) in trees:
        t_lv.append(tn.tree_level_for_intensity(inten(x, y), rng))
    ht, tt = collections.Counter(h_lv), collections.Counter(t_lv)

    summ = tn.summarise(tcfg, region, np.random.default_rng(seed + 23))

    # THE PLANK BUDGET, because it is the number that decides whether the
    # assembly is seconds or minutes and it is otherwise only discoverable by
    # running it. Counted rather than estimated: the same functions the
    # launcher calls.
    dbg = random.Random(seed + 77)
    n_plank = 0
    for (x, y, fp), lv in zip(houses, h_lv):
        if lv == "pristine":
            continue
        n_plank += len(planks.scatter_from_wreck(
            x, y, fp, inten(x, y), tcfg["heading_deg"], tcfg["throw_m"], dbg,
            n_pieces=140))
    n_track = len(planks.scatter_over_region(
        region, inten, tcfg["heading_deg"], dbg, per_100m2=1.6))

    print("\n[tornado_png] {0}".format(args.config))
    print("  region        {0:.0f} x {1:.0f} m, seed {2}".format(rw, rh, seed))
    print("  track         {0:.0f} m wide toward {1:.0f} deg through "
          "({2:.0f}, {3:.0f})".format(tcfg["width_m"], tcfg["heading_deg"],
                                      tcfg["origin_m"][0],
                                      tcfg["origin_m"][1]))
    print("  in path       {0:.1%} of the plate, mean intensity {1:.2f}, "
          "peak {2:.2f}".format(summ["in_path_frac"], summ["mean_intensity"],
                                summ["max_intensity"]))
    print("  houses        {0} total, {1} damaged".format(
        len(houses), sum(v for k, v in ht.items() if k != "pristine")))
    for lv in tn.HOUSE_LEVELS:
        print("    {0:<18} {1:>4}".format(lv, ht.get(lv, 0)))
    print("  trees         {0} host-side (yard planting not included), "
          "{1} damaged".format(
              len(trees), sum(v for k, v in tt.items() if k != "pristine")))
    for lv in tn.TREE_LEVELS:
        print("    {0:<18} {1:>4}".format(lv, tt.get(lv, 0)))
    print("  plank debris  {0} off the wrecks + {1} across the corridor "
          "= {2} board(s)".format(n_plank, n_track, n_plank + n_track))

    # WHAT TO LOOK FOR IN THESE NUMBERS, printed rather than left implicit,
    # because they are the three ways this scene fails and all three are
    # cheap to fix here and expensive to fix after a build.
    warn = []
    if summ["in_path_frac"] > 0.55:
        warn.append("the track covers most of the plate — there is no intact "
                    "fabric left to contrast against; narrow `width_m`")
    if summ["in_path_frac"] < 0.08:
        warn.append("the track barely touches the plate — move `epicenter` "
                    "or change `heading_deg`")
    dmg = sum(v for k, v in ht.items() if k != "pristine")
    if dmg and max(v for k, v in ht.items() if k != "pristine") > 0.7 * dmg:
        warn.append("one damage class is >70% of the damaged houses — the "
                    "gradient has collapsed; lower `core_frac` or `peak`")
    if dmg < 8:
        warn.append("fewer than 8 damaged houses — the corridor is missing "
                    "the fabric; check `epicenter` against the plan below")
    for w in warn:
        print("  !! {0}".format(w))
    if not warn:
        print("  OK            gradient and coverage both in band")

    # ---- the plot --------------------------------------------------------
    fig, ax = plt.subplots(figsize=(11, 11 * rh / rw))

    # The intensity field as a background, sampled coarsely — this is the
    # thing being judged and everything else is annotation on it.
    n = 220
    xs = np.linspace(region[0], region[2], n)
    ys = np.linspace(region[1], region[3], n)
    grid = np.array([[inten(x, y) for x in xs] for y in ys])
    # `extent` IS (left, right, bottom, top) AND `region` IS (x0, y0, x1, y1).
    # Passing the region straight in gives matplotlib left=x0, right=y0, and it
    # warns about a singular transform and draws NOTHING — which looks like an
    # intensity field that is zero everywhere rather than like an argument
    # order bug, and is why the first plan came out with a white background.
    im = ax.imshow(grid, origin="lower",
                   extent=(region[0], region[2], region[1], region[3]),
                   cmap="pink_r", vmin=0.0, vmax=1.0, zorder=0,
                   interpolation="bilinear")
    cb = fig.colorbar(im, ax=ax, shrink=0.7, pad=0.02)
    cb.set_label("intensity (EF proxy)")

    if trees:
        for lv in tn.TREE_LEVELS:
            pts = [p for p, l in zip(trees, t_lv) if l == lv]
            if not pts:
                continue
            ax.scatter([p[0] for p in pts], [p[1] for p in pts], s=7,
                       c=_TCOLOUR[lv], marker="^", linewidths=0, zorder=2,
                       label="tree {0} ({1})".format(lv, len(pts)))
    for lv in tn.HOUSE_LEVELS:
        pts = [p for p, l in zip(houses, h_lv) if l == lv]
        if not pts:
            continue
        ax.scatter([p[0] for p in pts], [p[1] for p in pts], s=44,
                   c=_HCOLOUR[lv], marker="s", edgecolors="k", linewidths=0.3,
                   zorder=3, label="house {0} ({1})".format(lv, len(pts)))

    # The centreline, meander and all, plus the nominal path edges. Sampled
    # from `frame()` rather than drawn as a straight line, so what is plotted
    # is the track the field actually uses.
    to_track, (ux, uy), (vx, vy) = tn.frame(tcfg)
    ox, oy = tcfg["origin_m"]
    half = 0.5 * float(tcfg["width_m"])
    reach = math.hypot(rw, rh)
    cl, ed_l, ed_r = [], [], []
    for k in range(-120, 121):
        a = reach * k / 120.0
        # Invert the meander: `frame` subtracts it from the cross coordinate,
        # so the centreline at `a` sits at +wobble(a) in the straight frame.
        probe = (ox + ux * a, oy + uy * a)
        _a2, c2 = to_track(*probe)
        off = -c2                       # how far the meander moved it
        cl.append((probe[0] + vx * off, probe[1] + vy * off))
        ed_l.append((probe[0] + vx * (off + half), probe[1] + vy * (off + half)))
        ed_r.append((probe[0] + vx * (off - half), probe[1] + vy * (off - half)))
    for pts, style in ((cl, dict(color="crimson", lw=1.8, ls="-")),
                       (ed_l, dict(color="crimson", lw=1.0, ls="--")),
                       (ed_r, dict(color="crimson", lw=1.0, ls="--"))):
        ax.plot([p[0] for p in pts], [p[1] for p in pts], zorder=4, **style)

    # Which way the debris went, which is a different arrow from the track.
    th = math.radians(float(tcfg["heading_deg"]) + float(tcfg["curl_deg"]))
    arrow = min(rw, rh) * 0.16
    ax.arrow(ox, oy, arrow * math.cos(th), arrow * math.sin(th),
             width=arrow * 0.03, color="#1f4fd8", zorder=6,
             length_includes_head=True)
    ax.annotate("debris {0:.0f} deg".format(
        float(tcfg["heading_deg"]) + float(tcfg["curl_deg"])),
        (ox + arrow * math.cos(th), oy + arrow * math.sin(th)),
        fontsize=8, color="#1f4fd8", zorder=6)

    ax.add_patch(plt.Rectangle((region[0], region[1]), rw, rh, fill=False,
                               ec="0.35", lw=1.4, zorder=5))
    ax.set_xlim(region[0] - rw * 0.05, region[2] + rw * 0.05)
    ax.set_ylim(region[1] - rh * 0.05, region[3] + rh * 0.05)
    ax.set_aspect("equal")
    ax.set_title("{0} — {1:.0f} m track toward {2:.0f} deg, seed {3}\n"
                 "{4} of {5} houses damaged, {6} board(s) of debris".format(
                     args.config, tcfg["width_m"], tcfg["heading_deg"], seed,
                     dmg, len(houses), n_plank + n_track))
    ax.legend(loc="upper left", fontsize=7, framealpha=0.9, ncol=2)

    out = args.out
    if not os.path.isabs(out):
        out = os.path.join(_SCENE_GEN, out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, dpi=130, bbox_inches="tight")
    print("  -> {0}\n".format(out))


if __name__ == "__main__":
    main()
