"""fire_png.py — the wildfire plan, without Isaac Sim.

    python3 tools/fire_png.py --config suburb_wildfire --out _plans/fire.png

WHY THIS EXISTS
---------------
Tuning a fire by launching Isaac Sim is a minutes-long loop for a question —
"where does the front get to, and how many emitters is that?" — that is pure
geometry and answerable in a second. Same trade `plan_png.py` and
`suburb_net_png.py` make for the layout.

It runs the REAL code path: `fence_png.build` for the suburb's placements, then
`disaster.fire`'s own `select_fuels`, `thin_by_spacing` and `plan_ignition`. So
the emitter count and arrival times printed here are the ones Isaac Sim will
use, not a reimplementation that can drift.

WHAT IT IS NOT
--------------
A render — there are no flames in it. It shows WHEN each fuel ignites and where
the front is at a few instants. What the fire LOOKS like is a Flow question
(colormap, density, plume) that only the renderer can answer.

Fuel counts run low against the real scene: `fence_png.build` stops before the
on-stage yard planting, so the yard trees are missing here and present in Isaac
Sim. Treat the emitter count as a floor.
"""

import argparse
import math
import os
import sys
import types

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
sys.path.insert(0, _HERE)
sys.path.insert(0, _SCENE_GEN)

# `disaster.fire` imports pxr at module scope for the emitter authoring half.
# Nothing this tool calls touches it — the spread model is pure arithmetic.
for _m in ("pxr", "pxr.Gf", "pxr.Sdf", "pxr.Usd", "pxr.UsdGeom"):
    sys.modules.setdefault(_m, types.ModuleType(_m))
for _n in ("Gf", "Sdf", "Usd", "UsdGeom"):
    setattr(sys.modules["pxr"], _n, types.SimpleNamespace())

import matplotlib                                              # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                # noqa: E402

import fence_png                                               # noqa: E402
from disaster import fire                                      # noqa: E402


def _front_polygon(t, cfg, n=180):
    """The burnt ellipse at time *t*, in metres, as (xs, ys).

    Mirrors the geometry `_ignition_time` inverts: semi-axis `A*t` along the
    wind, `B*t` across, centre `C*t` downwind of the ignition point.
    """
    head = float(cfg["head_mps"])
    flank = float(cfg["flank_mps"])
    back = float(cfg["back_mps"])
    A = 0.5 * (head + back) * t
    C = 0.5 * (head - back) * t
    B = flank * t

    th = math.radians(float(cfg["heading_deg"]))
    cos_t, sin_t = math.cos(th), math.sin(th)
    ox, oy = cfg["origin_m"]

    xs, ys = [], []
    for i in range(n + 1):
        a = 2.0 * math.pi * i / n
        u = C + A * math.cos(a)
        v = B * math.sin(a)
        xs.append(ox + u * cos_t - v * sin_t)
        ys.append(oy + u * sin_t + v * cos_t)
    return xs, ys


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="suburb_wildfire",
                    help="preset name or path (default: suburb_wildfire)")
    ap.add_argument("--seed", type=int, default=None,
                    help="override the layout seed")
    ap.add_argument("--out", default="_plans/fire.png")
    args = ap.parse_args()

    scene = fence_png.build(seed=args.seed, config_name=args.config)
    cfg_all = scene["cfg"]
    fcfg = dict(fire.DEFAULTS)
    fcfg.update((cfg_all.get("disaster") or {}).get("fire") or {})

    region = (cfg_all.get("layout") or {}).get("region_m") or [1600.0, 1200.0]
    rw, rh = float(region[0]), float(region[1])

    fuels = fire.select_fuels(scene["placements"], fcfg["fuel_categories"],
                              require_prim_path=False)
    kept = fire.thin_by_spacing(fuels, float(fcfg["emitter_spacing_m"]),
                                int(fcfg["max_emitters"]))
    import random
    planned = fire.plan_ignition(kept, fcfg, random.Random(fcfg["seed"]))

    print("\n[fire_png] {0}".format(args.config))
    print("  region        {0:.0f} x {1:.0f} m, seed {2}".format(
        rw, rh, scene["seed"]))
    print("  fuels         {0} (host-side; yard planting not included)".format(
        len(fuels)))
    print("  after thin    {0} at {1} m spacing, cap {2}".format(
        len(kept), fcfg["emitter_spacing_m"], fcfg["max_emitters"]))
    print("  emitters      {0} ignite within {1:.0f}s".format(
        len(planned), fcfg["duration_s"]))
    if planned:
        print("  arrival       {0:.0f}s .. {1:.0f}s ({2} alight at t=0)".format(
            planned[0][3], planned[-1][3],
            sum(1 for p in planned if p[3] <= 0.0)))
        # THE frame-rate number. An emitter only allocates Flow blocks while it
        # is inside its burn window, so what costs is how many overlap, not how
        # many exist.
        burn = (float(fcfg["ignition_s"]) + float(fcfg["flame_s"])
                + float(fcfg["smoulder_s"]))
        peak, peak_t = 0, 0.0
        for i in range(0, int(planned[-1][3] + burn) + 1, 5):
            n = sum(1 for p in planned if p[3] <= i < p[3] + burn)
            if n > peak:
                peak, peak_t = n, float(i)
        print("  peak active   {0} emitters at t+{1:.0f}s "
              "(burn window {2:.0f}s)".format(peak, peak_t, burn))

        # THE POST-DISASTER CHECK: what the scene looks like on frame one.
        # Most of it should be smouldering or residual, with a minority alight.
        ign_s = float(fcfg["ignition_s"])
        fl_s = float(fcfg["flame_s"])
        sm_s = float(fcfg["smoulder_s"])
        tally = {"unburnt": 0, "igniting": 0, "flaming": 0,
                 "smouldering": 0, "residual": 0}
        for _x, _y, _p, t, _inten, flames in planned:
            d = -t
            f = fl_s if flames else 0.0
            if d < 0:
                tally["unburnt"] += 1
            elif d < ign_s:
                tally["igniting"] += 1
            elif d < ign_s + f:
                tally["flaming"] += 1
            elif d < ign_s + f + sm_s:
                tally["smouldering"] += 1
            else:
                tally["residual"] += 1
        print("  at t=0        " + "  ".join(
            "{0}={1}".format(k, v) for k, v in tally.items()))
        print("  ever flames   {0} of {1}".format(
            sum(1 for e in planned if e[5]), len(planned)))
    print("  spread        head {0} / flank {1} / back {2} m/s toward "
          "{3:.0f} deg".format(fcfg["head_mps"], fcfg["flank_mps"],
                               fcfg["back_mps"], fcfg["heading_deg"]))

    fig, ax = plt.subplots(figsize=(11, 11 * rh / rw))
    ax.add_patch(plt.Rectangle((-rw / 2, -rh / 2), rw, rh, fill=False,
                               ec="0.55", lw=1.2, zorder=0))

    if fuels:
        ax.scatter([f[0] for f in fuels], [f[1] for f in fuels], s=2.0,
                   c="0.80", marker=".", linewidths=0, zorder=1,
                   label="fuel ({0})".format(len(fuels)))

    # The front at a few instants, so the shape of the run is readable at a
    # glance rather than only through the colour ramp.
    duration = float(fcfg["duration_s"])
    offset = fire.resolve_start_offset(fcfg)
    for frac in (0.25, 0.5, 0.75, 1.0):
        t = offset + duration * frac
        xs, ys = _front_polygon(t, fcfg)
        ax.plot(xs, ys, color="0.35", lw=0.9, ls="--", zorder=2)
        ax.annotate("t+{0:.0f}s".format(duration * frac), (xs[0], ys[0]),
                    fontsize=7, color="0.35", zorder=5)
    # Where the front already is when the scene opens.
    if offset > 0.0:
        xs, ys = _front_polygon(offset, fcfg)
        ax.plot(xs, ys, color="crimson", lw=1.6, zorder=3,
                label="alight at t=0")

    if planned:
        sc = ax.scatter([p[0] for p in planned], [p[1] for p in planned],
                        c=[p[3] for p in planned], s=42, cmap="inferno_r",
                        edgecolors="k", linewidths=0.3, zorder=4,
                        label="emitter ({0})".format(len(planned)))
        cb = fig.colorbar(sc, ax=ax, shrink=0.7, pad=0.02)
        cb.set_label("ignition time (s after scene start)")

    ox, oy = fcfg["origin_m"]
    ax.plot([ox], [oy], marker="*", ms=18, color="crimson", mec="k", mew=0.6,
            zorder=6, label="ignition point")
    th = math.radians(float(fcfg["heading_deg"]))
    arrow = min(rw, rh) * 0.14
    ax.arrow(ox, oy, arrow * math.cos(th), arrow * math.sin(th),
             width=arrow * 0.03, color="crimson", zorder=6, length_includes_head=True)

    ax.set_aspect("equal")
    ax.set_xlim(-rw / 2 - 40, rw / 2 + 40)
    ax.set_ylim(-rh / 2 - 40, rh / 2 + 40)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("{0} — wildfire plan\nhead {1} m/s, wind {2:.0f} deg, "
                 "{3} emitters".format(args.config, fcfg["head_mps"],
                                       fcfg["heading_deg"], len(planned)))
    ax.legend(loc="upper left", fontsize=8, framealpha=0.9)
    ax.grid(alpha=0.15)

    out = args.out
    if not os.path.isabs(out):
        out = os.path.join(_SCENE_GEN, out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, dpi=130, bbox_inches="tight")
    print("  wrote         {0}\n".format(out))


if __name__ == "__main__":
    main()
