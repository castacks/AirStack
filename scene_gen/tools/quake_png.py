#!/usr/bin/env python3
"""quake_png.py — the earthquake plan: the epicentre and the rungs.

    python3 tools/quake_png.py --config earthquake --out _plans/quake.png

WHY THIS EXISTS
---------------
The wildfire has `tools/fire_png.py`: where the front gets to, and how many
emitters that is, answered in a second instead of by launching Isaac Sim. This
is the same trade for the earthquake, and the question is the same shape —
"how far out does `pancaked` reach, and how many buildings is that?"

An earthquake's field is radial, so its ladder is a set of RINGS around the
epicentre. Each ring is where one rung takes over, and the rung is what Stage A
baked and Stage B references — so these radii are the boundaries between the
archetypes the scene is actually assembled from.

It runs the REAL code path: `plan_png.build` compiles and builds the scene the
way `scene_launch_script.py` does, and the rungs come from `disaster.levels`
(`ladder`, `local_damage`, `level_for`) rather than a reimplementation of the
thresholds. The ring radii are found by bisecting the field object itself, so
this tool cannot disagree with `disaster/field.py` about where the falloff is.

WHAT IT IS NOT
--------------
A render. Nothing here says what a `pancaked` tower looks like — that is
`tools/quake_preview.py` (one building, every rung) and
`tools/damage_gallery.py`. This says WHERE each rung falls and on how many
buildings.

Building counts inherit `plan_png`'s footprints, which are scraped from the
asset-pack comments rather than measured, so the count is the plan's count and
not the sim's. See `plan_png.StubResolver` for the size of that gap.

Radial fields only — the epicentre is a radial idea. A track (tornado) and a
front (fire) have their rungs drawn by `plan_png --config <track preset>` and
`fire_png.py` respectively.
"""

import argparse
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
sys.path.insert(0, _HERE)
sys.path.insert(0, _SCENE_GEN)

import matplotlib                                              # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                # noqa: E402

import plan_png                                                # noqa: E402
from disaster import levels                                    # noqa: E402


#: Where a rung's ring lies, when it has no ring. Distinguished because they
#: mean opposite things: `never` is a rung this severity cannot reach anywhere,
#: `everywhere` is one the field clears even at the far corners.
NEVER, EVERYWHERE = "never", "everywhere"


def ring_radius(field, sev, at, reach):
    """Metres from the epicentre at which local damage falls below *at*.

    Bisected on the field object rather than solved against the falloff curve:
    the curve is `field.py`'s to change (it is a smoothstep today), and a
    preview that hard-codes the inverse of it is a preview that silently goes
    wrong the day it stops being one. Twenty halvings of a few hundred metres
    is sub-metre, which is far finer than a building.

    Returns metres, or `NEVER` / `EVERYWHERE`.
    """
    cx, cy = field.center

    def damage_at(d):
        return levels.local_damage(field(cx + d, cy), sev)

    if damage_at(0.0) < at:
        return NEVER
    if damage_at(reach) >= at:
        return EVERYWHERE
    lo, hi = 0.0, reach
    for _ in range(20):
        mid = 0.5 * (lo + hi)
        if damage_at(mid) >= at:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def rings(field, dtype, sev, reach):
    """`[(rung, radius-or-sentinel)]` for every rung above `pristine`.

    Outward order — the tightest ring first — because that is the order they
    are read in: the worst damage is at the epicentre and each ring out is one
    rung gentler.
    """
    out = []
    for r in reversed(levels.ladder(dtype)):
        if r.at > 0.0:
            out.append((r, ring_radius(field, sev, r.at, reach)))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--config", default="earthquake",
                    help="preset name or path (default: earthquake)")
    ap.add_argument("--out", default="_plans/quake.png")
    a = ap.parse_args()

    # A preset can be named or pathed; the title wants the name either way.
    name = os.path.splitext(os.path.basename(str(a.config)))[0]

    cfg, layout, placements, _res = plan_png.build(a.config)
    field = plan_png.field_of(cfg, layout)
    import scene_generator as sg
    dis = sg._stage(cfg, "disaster") or {}
    dtype = dis.get("type")
    sev = float(dis.get("severity", 1.0) or 0.0)

    if field is None or not hasattr(field, "center"):
        sys.exit("[quake_png] {0} has no radial field ({1}) — there is no "
                 "epicentre to draw. Use tools/plan_png.py.".format(
                     name, getattr(field, "kind", dis.get("type", "none"))))

    x0, y0, x1, y1 = layout["region"]
    cx, cy = field.center
    # The reach the bisection brackets: past the end of the ease there is
    # nothing left to find, but a ring can also sit outside the region and
    # still be worth reporting — it says the whole map is inside that rung.
    reach = max(field.full + field.falloff,
                math.hypot(x1 - x0, y1 - y0))

    ladder = list(levels.ladder(dtype))
    n_rung, budget = {}, 0
    pts = {r.name: ([], []) for r in ladder}
    for p in placements:
        if p.get("category") not in ("house", "building"):
            continue
        rung = levels.level_for(dtype, field, float(p["x_m"]),
                                float(p["y_m"]), sev).name
        n_rung[rung] = n_rung.get(rung, 0) + 1
        pts[rung][0].append(p["x_m"])
        pts[rung][1].append(p["y_m"])
        if float(p.get("_mesh_damage") or 0.0) > 0.0:
            budget += 1

    print("\n[quake_png] {0}".format(name))
    print("  region        {0:.0f} x {1:.0f} m, seed {2}".format(
        x1 - x0, y1 - y0, cfg.get("seed")))
    print("  epicentre     ({0:.0f}, {1:.0f}), severity {2:g}".format(
        cx, cy, sev))
    print("  field         {0}: {1:.2f} out to {2:.0f} m, easing to {3:.2f} "
          "over {4:.0f} m".format(field.kind, field.inside, field.full,
                                  field.outside, field.falloff))
    band = rings(field, dtype, sev, reach)
    for r, rad in band:
        where = (rad if isinstance(rad, str)
                 else "r < {0:.0f} m".format(rad))
        print("  {0:<18}{1:<18}(local damage >= {2:.2f})".format(
            r.name, where, r.at))
    print("  buildings     {0}: {1}".format(
        sum(n_rung.values()),
        "  ".join("{0} {1}".format(r.name, n_rung[r.name])
                 for r in ladder if r.name in n_rung)))
    # The rung is what a building would BE; the budget is how many the mesh
    # pipeline could afford to actually break (`mesh_damage.fracture.
    # max_buildings`, spent on the worst-hit). A rung count far above this is
    # not a contradiction — the rest keep the disaster stage's tilt and sink.
    print("  mesh damaged  {0} carry `_mesh_damage`".format(budget))

    rw, rh = x1 - x0, y1 - y0
    fig, ax = plt.subplots(figsize=(11, 11 * rh / rw))
    ax.add_patch(plt.Rectangle((x0, y0), rw, rh, fill=False, ec="0.55",
                               lw=1.2, zorder=0))

    # One colour per rung, dark at the epicentre end of the ladder. Ordered,
    # not categorical: the ladder is ordered, and a categorical palette would
    # let `cracked` read as more severe than `pancaked`.
    cmap = matplotlib.colormaps["inferno_r"]
    for i, r in enumerate(ladder):
        xs, ys = pts[r.name]
        if not xs:
            continue
        shade = cmap(0.15 + 0.75 * (i / max(len(ladder) - 1, 1)))
        ax.scatter(xs, ys, s=26, color=shade, edgecolors="k", linewidths=0.25,
                   zorder=4, label="{0} ({1})".format(r.name, len(xs)))

    # The bands, widest first so each rung overpaints the one outside it. The
    # rings alone are not enough on a severity-1.0 scene: there the tightest
    # ring can sit beyond the region and the map goes back to looking like an
    # undifferentiated field, when what it is saying is "all of this is on the
    # top rung".
    for r, rad in sorted(
            ((r, rad) for r, rad in band if not isinstance(rad, str)),
            key=lambda t: -t[1]):
        i = [x.name for x in ladder].index(r.name)
        ax.add_patch(plt.Circle(
            (cx, cy), rad, facecolor=cmap(0.15 + 0.75 * (i / max(len(ladder) - 1, 1))),
            alpha=0.10, edgecolor="none", zorder=1))

    for r, rad in band:
        if isinstance(rad, str):
            continue
        ax.add_patch(plt.Circle((cx, cy), rad, fill=False, ec="0.30", lw=0.9,
                                ls="--", zorder=5))
        # Labelled on the ring itself rather than in the legend, and slid down
        # the arc when the ring's north pole is off the map — a ring wider
        # than the region is the interesting case (every building is inside
        # that rung), so that is exactly when its label must not vanish.
        ly = min(cy + rad, y1 - 0.04 * rh)
        lx = cx + math.sqrt(max(rad * rad - (ly - cy) ** 2, 0.0))
        ax.annotate("{0}  {1:.0f} m".format(r.name, rad), (lx, ly),
                    fontsize=7, color="0.30", ha="right", va="bottom",
                    zorder=6)

    ax.plot([cx], [cy], marker="*", ms=18, color="crimson", mec="k", mew=0.6,
            zorder=7, label="epicentre")

    ax.set_aspect("equal")
    ax.set_xlim(x0 - 40, x1 + 40)
    ax.set_ylim(y0 - 40, y1 + 40)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("{0} — earthquake plan\nseverity {1:g}, epicentre "
                 "({2:.0f}, {3:.0f}), {4} buildings".format(
                     name, sev, cx, cy, sum(n_rung.values())))
    ax.legend(loc="upper left", fontsize=8, framealpha=0.9)
    ax.grid(alpha=0.15)

    out = a.out
    if not os.path.isabs(out):
        out = os.path.join(_SCENE_GEN, out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, dpi=130, bbox_inches="tight")
    print("  wrote         {0}\n".format(out))


if __name__ == "__main__":
    main()
