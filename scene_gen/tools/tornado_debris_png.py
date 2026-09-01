#!/usr/bin/env python3
"""tornado_debris_png.py -- the offline 2D verification gate for ROUND 3's
debris CONCENTRATION fix (`_plans/urban_tornado_plan.md` §8, R8; stream DB).

WHY THIS EXISTS
----------------
The user's verdict on the round-2 bench: "it looks like wooden planks
scattered everywhere rather than actual street props." Round 2 already fixed
the MATERIAL (`tornado_urban._kind_of` retired `deck`/wood); this round fixes
the SPATIAL pattern -- a majority of every removed façade piece's fragments
now heap in a rubble BERM against the piece's own wall base
(`tornado_urban._deposit_berm`) instead of joining a uniform downwind fan,
and the corridor's own ambient field (`tornado_urban_ground.scatter_corridor`)
now biases toward building-footprint edges and street corners instead of a
flat lattice. Per this round's own rule ("the lead reviews the PATTERN
offline before any launch"), this script is PURE matplotlib over the SAME
pure-Python planner/scatter functions those two modules already expose --
no `pxr`, no Isaac Sim, runs on the host in well under a second.

WHAT IT DRAWS (one figure, three panels)
------------------------------------------
1. ONE synthetic building's own plan (`tornado_urban.plan_damage`, level T4,
   a fixed wind bearing): footprint outline, every removed piece's berm
   fragments (colour = `z_lift`, the height-profile stacking a future
   builder pass will honour -- see `_deposit_berm`'s own docstring for why
   it is authored but not yet consumed) versus the remaining ballistic/
   downwind fragments (small grey dots, the old round-1/2 fan) and glass
   shards (near-façade, a third colour). This is the "berm at the foot of
   every façade that failed" panel.
2. THE SAME building's debris, but as a HEATMAP/hexbin density -- the
   quickest way to see "does this read as a heap, or as confetti" at a
   glance, which is exactly the user's own complaint.
3. A CORRIDOR PATCH: `tornado_urban_ground.scatter_corridor` over a small
   grid of building footprints, points coloured by `near_edge` (the round-3
   acceptance-reweighting flag) -- drifts hugging building feet and corners
   versus the sparse uniform floor on open ground.

Usage:
    python3 scene_gen/tools/tornado_debris_png.py
        [--out _plans/tornado_debris_preview.png] [--seed 13]
"""
import argparse
import math
import os
import random
import sys

os.environ.setdefault("MPLBACKEND", "Agg")

_HERE = os.path.dirname(os.path.abspath(__file__))
_SG = os.path.normpath(os.path.join(_HERE, ".."))
if _SG not in sys.path:
    sys.path.insert(0, _SG)
_TESTS = os.path.join(_SG, "tests")
if _TESTS not in sys.path:
    sys.path.insert(0, _TESTS)

import matplotlib                                              # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                 # noqa: E402
from matplotlib.patches import Rectangle                        # noqa: E402

from disaster import quake_flow as qf                            # noqa: E402
from disaster import tornado as trn                                # noqa: E402
from disaster import tornado_urban as tu                            # noqa: E402
from disaster import tornado_urban_ground as tug                     # noqa: E402
from test_quake_sliced import fake_sliced_building                    # noqa: E402


def _fake_wind(bearing_deg, speed_frac=0.9, cross_frac=-0.35, over=False):
    return {"bearing_deg": float(bearing_deg), "speed_frac": float(speed_frac),
            "cross_frac": float(cross_frac), "over": bool(over)}


def _building_plan(seed, bearing_deg=140.0):
    pls, style, grid = fake_sliced_building(
        W=30.0, D=24.0, H=40.0, storeys=10, seed=seed)
    info = qf.describe(style, pls, 0.0, 0.0, 0.0)
    info["type"] = "rc"
    rng = random.Random(seed)
    plan = tu.plan_damage(info, info["elements"], "T4", "rc", rng,
                          _fake_wind(bearing_deg), intensity=0.9)
    return info, plan


def _draw_building_panel(ax, info, plan):
    m = info["masses"]["main"]
    w2, d2 = m["W"] / 2.0, m["D"] / 2.0
    corners_local = [(-w2, -d2), (w2, -d2), (w2, d2), (-w2, d2)]
    corners_world = [qf._to_world(m, lx, ly) for lx, ly in corners_local]
    xs = [c[0] for c in corners_world] + [corners_world[0][0]]
    ys = [c[1] for c in corners_world] + [corners_world[0][1]]
    ax.plot(xs, ys, color="black", linewidth=1.5, zorder=5)
    ax.fill(xs[:-1], ys[:-1], color="0.85", zorder=1)

    berm = [f for f in plan["debris"] if f.get("stacked") and f["kind"] != "glass"]
    ballistic = [f for f in plan["debris"]
                if not f.get("stacked") and f["kind"] != "glass"]
    glass = [f for f in plan["debris"] if f["kind"] == "glass"]

    if ballistic:
        ax.scatter([f["x"] for f in ballistic], [f["y"] for f in ballistic],
                  s=6, c="0.55", alpha=0.5, zorder=2,
                  label="ballistic/downwind (n={0})".format(len(ballistic)))
    if glass:
        ax.scatter([f["x"] for f in glass], [f["y"] for f in glass],
                  s=4, c="#2a6fdb", alpha=0.45, zorder=2,
                  label="glass, near-façade (n={0})".format(len(glass)))
    if berm:
        zs = [f["z_lift"] for f in berm]
        sca = ax.scatter([f["x"] for f in berm], [f["y"] for f in berm],
                        s=14, c=zs, cmap="inferno", vmin=0.0,
                        vmax=max(tu._BERM_H_MAX_M, max(zs)), zorder=4,
                        edgecolors="none",
                        label="berm, wall-base heap (n={0})".format(len(berm)))
        cb = plt.colorbar(sca, ax=ax, fraction=0.046, pad=0.04)
        cb.set_label("berm z_lift (m)")

    bearing = plan["wind"]["bearing_deg"]
    brad = math.radians(bearing)
    ax.annotate("", xy=(w2 * 1.6 * math.cos(brad), w2 * 1.6 * math.sin(brad)),
               xytext=(0, 0),
               arrowprops=dict(arrowstyle="->", color="crimson", lw=2))
    ax.text(w2 * 1.7 * math.cos(brad), w2 * 1.7 * math.sin(brad),
           "wind {0:.0f}deg".format(bearing), color="crimson", fontsize=8)

    st = plan["stats"]
    # `n_berm_kept` (post per-building thinning) is the count that shares a
    # scale with `n_struct_debris` (also post-thin) -- `n_berm` alone is
    # the pre-thin AUTHORED count and can exceed `n_struct_debris` once
    # `DEBRIS_MAX_PER_BUILDING` truncates the pool, so it is shown
    # separately rather than divided into the wrong denominator.
    ax.set_title(
        "1. One building, T4 -- berm {0}/{1} struct frags kept ({2:.0%}), "
        "{3} authored\nlevel={4}  share_table={5:.2f}".format(
            st["n_berm_kept"], st["n_struct_debris"],
            (st["n_berm_kept"] / st["n_struct_debris"]) if st["n_struct_debris"] else 0.0,
            st["n_berm"], plan["level"], st["berm_share_level"]),
        fontsize=9)
    ax.set_aspect("equal")
    ax.legend(fontsize=6, loc="upper left")
    pad = max(w2, d2) * 2.0
    ax.set_xlim(-pad, pad)
    ax.set_ylim(-pad, pad)


def _draw_density_panel(ax, info, plan):
    m = info["masses"]["main"]
    w2, d2 = m["W"] / 2.0, m["D"] / 2.0
    xs = [f["x"] for f in plan["debris"]]
    ys = [f["y"] for f in plan["debris"]]
    pad = max(w2, d2) * 2.0
    hb = ax.hexbin(xs, ys, gridsize=40, cmap="magma",
                   extent=(-pad, pad, -pad, pad), mincnt=1)
    plt.colorbar(hb, ax=ax, fraction=0.046, pad=0.04, label="fragment count")
    corners_local = [(-w2, -d2), (w2, -d2), (w2, d2), (-w2, d2)]
    corners_world = [qf._to_world(m, lx, ly) for lx, ly in corners_local]
    xs2 = [c[0] for c in corners_world] + [corners_world[0][0]]
    ys2 = [c[1] for c in corners_world] + [corners_world[0][1]]
    ax.plot(xs2, ys2, color="cyan", linewidth=1.2, zorder=5)
    ax.set_title("2. Same building -- density read\n"
                "(heaps at wall feet, not a flat scatter)", fontsize=9)
    ax.set_aspect("equal")
    ax.set_xlim(-pad, pad)
    ax.set_ylim(-pad, pad)


def _corridor_patch(seed):
    region = (-120.0, -70.0, 120.0, 70.0)

    def intensity(x, y, half=70.0, peak=0.85):
        return max(0.0, peak * (1.0 - abs(y) / half))

    placements = []
    for x in range(-100, 101, 40):
        for y in (-45.0, 45.0):
            placements.append({"x": float(x), "y": y, "W": 16.0, "D": 12.0,
                              "yaw": 0.0})
    corners = [(x + 20.0, 0.0) for x in range(-100, 101, 40)]
    cfg = dict(trn.DEFAULTS)
    cfg["heading_deg"] = 60.0
    cfg["width_m"] = 140.0
    cfg["peak"] = 0.9
    frags = tug.scatter_corridor(region, intensity, cfg, random.Random(seed),
                                 placements=placements, corners=corners)
    return region, placements, frags


def _draw_corridor_panel(ax, seed):
    region, placements, frags = _corridor_patch(seed)
    for p in placements:
        w2, d2 = p["W"] / 2.0, p["D"] / 2.0
        ax.add_patch(Rectangle((p["x"] - w2, p["y"] - d2), p["W"], p["D"],
                              facecolor="0.8", edgecolor="black",
                              linewidth=0.8, zorder=1))
    edge = [f for f in frags if f.get("near_edge")]
    floor = [f for f in frags if not f.get("near_edge")]
    if floor:
        ax.scatter([f["x"] for f in floor], [f["y"] for f in floor],
                  s=5, c="0.6", alpha=0.5, zorder=2,
                  label="uniform floor (n={0})".format(len(floor)))
    if edge:
        ax.scatter([f["x"] for f in edge], [f["y"] for f in edge],
                  s=8, c="#c0392b", alpha=0.75, zorder=3,
                  label="edge/corner drift (n={0})".format(len(edge)))
    frac = (len(edge) / float(len(frags))) if frags else 0.0
    ax.set_title(
        "3. Corridor patch -- {0} fragments, {1:.0%} edge/corner-biased\n"
        "(drifts against building feet + corners, sparse floor between)"
        .format(len(frags), frac), fontsize=9)
    ax.set_aspect("equal")
    ax.legend(fontsize=6, loc="upper right")
    ax.set_xlim(region[0], region[2])
    ax.set_ylim(region[1], region[3])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(
        _SG, "_plans", "tornado_debris_preview.png"))
    ap.add_argument("--seed", type=int, default=13)
    ap.add_argument("--bearing", type=float, default=140.0)
    args = ap.parse_args()

    info, plan = _building_plan(args.seed, args.bearing)

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    _draw_building_panel(axes[0], info, plan)
    _draw_density_panel(axes[1], info, plan)
    _draw_corridor_panel(axes[2], args.seed)
    fig.suptitle(
        "Urban tornado ROUND 3 -- debris CONCENTRATION "
        "(_plans/urban_tornado_plan.md §8, R8)  seed={0}".format(args.seed),
        fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.94))

    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    fig.savefig(args.out, dpi=140)
    print("[tornado_debris_png] wrote {0}".format(args.out))


if __name__ == "__main__":
    main()
