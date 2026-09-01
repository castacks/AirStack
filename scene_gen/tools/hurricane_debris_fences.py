#!/usr/bin/env python3
"""hurricane_debris_fences.py — the offline gate for the FENCES pass
(DEBRIS D3 review, 2026-08-31): "fences stand intact in 2 m of surge."

    python3 tools/hurricane_debris_fences.py --level l3 \\
        --out ~/hurricane_previews/offline/debris/fences_L3.png

WHY THIS IS A SEPARATE TOOL FROM `hurricane_debris_plot.py`
-------------------------------------------------------------
That tool deliberately avoids a live USD stage (it stubs `pxr` in
`sys.modules` and reads house/tree positions from a past run's
`GT_hurricane.json`) because `raft_specs`/`land_debris_specs` only need
plain `(x, y, level, ...)` tuples. Fence geometry has no such shortcut:
picket/rail runs come out of `suburb_scene`'s lot-boundary fitting
(`_lay_fence_run`, `_FenceGrid`, the enclosure sweep), which is reachable
only by actually building the suburb layout — there is no lightweight
"just the fences" entry point, and no ground-truth file records them.
So this tool runs the REAL generator (`suburb_scene.generate_suburb_on_stage`
against a real, un-stubbed `pxr`, an in-memory stage, ~15-20 s a preset) and
then the REAL `disaster.washaway.measure_fence` / `fence_specs` /
`apply_fence_pose` — unmodified, same code path the launcher's new
`# FENCES` block runs — so a number that changes here is a number that
will change in the next Isaac render.

NUCLEUS ASSETS DO NOT RESOLVE HERE, AND THAT IS FINE FOR THIS PURPOSE. Two
of the three fence-pool assets are local (`scene_gen/assets/objaverse/...`)
and measure correctly; the ornamental-iron-rail asset is Nucleus-hosted
(`omniverse://.../Dmytro/.../prop_park_railing/...`) and fails to open
offline exactly the way `suburb_scene`'s OWN placement-time measurement
already logs as `[scene_gen] fallback fence: ... -> 4.00 x 4.00 m` — this
is not a defect introduced by this tool or by `measure_fence`, it is the
same "no Nucleus offline" gap every other Nucleus-hosted asset in this
scene already has (cars, street furniture, the ornamental rail). Affected
fences measure `length_m == 0.0` here and are reported as such; NONE of
`fence_specs`'s gone/flat/stands decision reads `length` at all, so the
tally itself is exact regardless.

WHAT IT PLOTS
--------------
The flooded region (from `surge.depth_at`, the same depth field
`fence_specs`'s water gate reads), houses coloured by wind level, and every
fence coloured by its `fence_specs` outcome: green = stands, orange X =
flat (with a short tick showing the fall azimuth), grey ring = gone (the
panel itself, deactivated) with small blue squares for the `"fence"`-kind
rafts `fence_specs` built to replace it — literally "add fence rafts to
the plot".
"""

import argparse
import collections
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
sys.path.insert(0, _HERE)
sys.path.insert(0, _SCENE_GEN)

import numpy as np                                              # noqa: E402
import matplotlib                                                # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                  # noqa: E402
from matplotlib.patches import Circle                             # noqa: E402

from pxr import Sdf, Usd                                          # noqa: E402

import compile_disaster as cd                                     # noqa: E402
import suburb_scene as ss                                          # noqa: E402
from disaster import hurricane as hu                               # noqa: E402
from disaster import surge as sgw                                  # noqa: E402
from disaster import washaway as wash                               # noqa: E402

SEED = 11

_LEVEL_COLOUR = {"pristine": "#2f7d32", "shingles_lost": "#8bbf4a",
                 "cover_lost": "#c9d14a", "deck_panels_lost": "#e0c23d",
                 "roof_stripped": "#e8a33d", "roof_collapsed": "#d4622c",
                 "partial_collapse": "#b8402a", "leveled": "#96261f",
                 "swept": "#3d0f0c"}
_ACTION_COLOUR = {"stands": "#2f7d32", "flat": "#e0862a", "gone": "#888888"}


def _resolve_scfg(config, region):
    """EXACT copy of the launcher's `scfg` construction (also duplicated in
    `hurricane_debris_plot.py`'s `_resolve_scfg` — kept in sync by being
    this short and this literal a copy, not by importing across the two
    stub-vs-real-`pxr` tools)."""
    hsub = ((config.get("disaster") or {}).get("hurricane") or {})
    scfg = sgw.resolve_cfg({k: v for k, v in hsub.items()
                            if k in sgw.DEFAULTS})
    span = max(region[2] - region[0], region[3] - region[1])
    scfg.update(sgw.knobs_from_env(span))
    for k, v in hsub.items():
        if k in sgw.DEFAULTS:
            scfg[k] = v
    return scfg


def build_field(preset_name, seed=SEED):
    """Runs the REAL suburb generator on a real in-memory stage. Returns a
    dict: `region`, `hcfg`, `scfg`, `houses` (with an approximate wind
    `level`, `hu.house_level_for_intensity` + `hu.draw_vulnerability`, the
    SAME two calls the launcher's wind pass makes — approximate only in
    skipping that pass's `swept`/surge-state override, a minority path this
    diagnostic does not need), `depth_fn`, `fence_paths`, `fence_geo`
    (`measure_fence` output), `stage`.
    """
    config = cd.load_scene_config(preset_name)
    stage = Usd.Stage.CreateInMemory()
    binfo = {}
    ss.generate_suburb_on_stage(
        stage, config, parent_path="/World/stage/generated",
        scene_scale_factor=1.0, info_out=binfo, assembly=True)

    region = tuple(binfo.get("region") or (-250.0, -250.0, 250.0, 250.0))
    hcfg = hu.resolve_cfg(config)
    scfg = _resolve_scfg(config, region)

    wrng = random.Random(seed + 61)
    drng = random.Random(seed + 5)
    inten_fn = hu.intensity_field(hcfg, region, wrng)

    houses = []
    wrecks = []
    for h in binfo.get("house_instances", []):
        i = float(inten_fn(h["x"], h["y"]))
        _era, vuln = hu.draw_vulnerability(drng)
        level = hu.house_level_for_intensity(i, drng, vuln=vuln)
        houses.append({"x": h["x"], "y": h["y"], "level": level,
                       "intensity": i})
        wrecks.append((h["x"], h["y"], 12.0, i, level, None))

    depth_fn = sgw.depth_at(scfg, region,
                            np.random.default_rng(seed + 41))

    root = stage.GetPrimAtPath(Sdf.Path("/World/stage/generated"))
    fence_paths = [str(p.GetPath()) for p in Usd.PrimRange(root)
                  if p.GetName().startswith("fence_")]
    fence_geo = [wash.measure_fence(stage, stage.GetPrimAtPath(p), ssf=1.0)
                for p in fence_paths]

    return {"region": region, "hcfg": hcfg, "scfg": scfg, "houses": houses,
           "wrecks": wrecks, "depth_fn": depth_fn, "inten_fn": inten_fn,
           "fence_paths": fence_paths, "fence_geo": fence_geo,
           "stage": stage}


def decide_and_apply(field, seed=SEED):
    """Runs the REAL `wash.fence_specs`/`wash.apply_fence_pose` — the exact
    two calls the launcher's `# FENCES` block makes. Returns
    `(decisions, fence_rafts)`.

    REUSES `field["inten_fn"]` -- the SAME closure that decided every
    house's own wind-damage level in `build_field`, not a freshly built
    `hu.intensity_field(...)` -- see the launcher's own `# FENCES` comment
    ("`inten` -- NOT a freshly-built...") for why a second realisation of
    the field's small noise term would disagree with the houses.
    """
    frng = random.Random(seed + 131)
    decisions = wash.fence_specs(
        field["fence_geo"], field["depth_fn"],
        lambda x, y: hu.wind_bearing_at(field["hcfg"], x, y),
        field["inten_fn"],
        field["wrecks"], sgw.water_level(field["scfg"]), frng)
    fence_rafts = []
    for path, dec in zip(field["fence_paths"], decisions):
        wash.apply_fence_pose(field["stage"], path, dec, ssf=1.0)
        fence_rafts.extend(dec.get("rafts") or ())
    return decisions, fence_rafts


def plot_level(level_name, field, decisions, fence_rafts, out_path):
    region = field["region"]
    x0, y0, x1, y1 = region
    grid_n = 240
    xs = np.linspace(x0, x1, grid_n)
    ys = np.linspace(y0, y1, grid_n)
    depth_grid = np.array([[field["depth_fn"](x, y) for x in xs]
                           for y in ys])
    wet = (depth_grid > 0.0).astype(np.float32)

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

    tally = collections.Counter()
    for d in decisions:
        tally[d["action"]] += 1
        colour = _ACTION_COLOUR[d["action"]]
        if d["action"] == "stands":
            ax.plot(d["x"], d["y"], marker=".", color=colour,
                   markersize=3.0, markeredgewidth=0, zorder=4)
        elif d["action"] == "flat":
            ax.plot(d["x"], d["y"], marker="x", color=colour,
                   markersize=5.0, markeredgewidth=1.2, zorder=5)
            az = math.radians(d["azimuth_deg"])
            ax.plot([d["x"], d["x"] + 4.0 * math.cos(az)],
                   [d["y"], d["y"] + 4.0 * math.sin(az)],
                   color=colour, linewidth=0.8, zorder=5)
        else:   # gone
            ax.add_patch(Circle((d["x"], d["y"]), 1.2, facecolor="none",
                                edgecolor=colour, linewidth=0.8, zorder=4))

    for r in fence_rafts:
        ax.plot(r["x"], r["y"], marker="s", color="#2a5fe0",
               markersize=2.6, markeredgewidth=0, zorder=6)

    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")

    n_total = sum(tally.values())
    n_unresolved = sum(1 for g in field["fence_geo"] if g[3] <= 1e-6)
    ax.set_title(
        "{0} -- fences {1}: {2} stands, {3} flat, {4} gone "
        "({5} fence-raft(s)); {6} unresolved offline (Nucleus asset, "
        "harmless -- see module docstring)"
        .format(level_name, n_total, tally.get("stands", 0),
               tally.get("flat", 0), tally.get("gone", 0),
               len(fence_rafts), n_unresolved),
        fontsize=10)
    handles = [
        plt.Line2D([0], [0], marker=".", linestyle="",
                  color=_ACTION_COLOUR["stands"], label="fence: stands"),
        plt.Line2D([0], [0], marker="x", linestyle="",
                  color=_ACTION_COLOUR["flat"], label="fence: flat"),
        plt.Line2D([0], [0], marker="o", linestyle="", markerfacecolor="none",
                  markeredgecolor=_ACTION_COLOUR["gone"], label="fence: gone"),
        plt.Line2D([0], [0], marker="s", linestyle="", color="#2a5fe0",
                  label="fence raft"),
    ]
    ax.legend(handles=handles, loc="upper right", fontsize=8,
             framealpha=0.85)

    out_path = os.path.expanduser(out_path)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)
    print("[hurricane_debris_fences] wrote {0}".format(out_path))
    return tally


_PRESET_BY_LEVEL = {"l2": "suburb_hurricane_500_l2",
                    "l3": "suburb_hurricane_500_l3"}
_OUT_BY_LEVEL = {"l2": "~/hurricane_previews/offline/debris/fences_L2.png",
                 "l3": "~/hurricane_previews/offline/debris/fences_L3.png"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--level", choices=("l2", "l3", "both"), default="both")
    ap.add_argument("--out", default=None)
    ap.add_argument("--seed", type=int, default=SEED)
    args = ap.parse_args()

    levels = ("l2", "l3") if args.level == "both" else (args.level,)
    for lvl in levels:
        preset = _PRESET_BY_LEVEL[lvl]
        out_path = args.out or _OUT_BY_LEVEL[lvl]
        print("[hurricane_debris_fences] === {0} ({1}) ===".format(lvl, preset))
        field = build_field(preset, seed=args.seed)
        decisions, fence_rafts = decide_and_apply(field, seed=args.seed)
        lengths = [g[3] for g in field["fence_geo"]]
        n_zero = sum(1 for l in lengths if l <= 1e-6)
        print("[hurricane_debris_fences] {0} fence(s), length "
             "min={1:.2f} median={2:.2f} max={3:.2f} m ({4} unresolved "
             "offline)".format(
                 len(lengths), min(lengths) if lengths else 0.0,
                 sorted(lengths)[len(lengths) // 2] if lengths else 0.0,
                 max(lengths) if lengths else 0.0, n_zero))
        tally = plot_level(lvl.upper(), field, decisions, fence_rafts,
                           out_path)
        print("[hurricane_debris_fences] {0} tally: {1}"
             .format(lvl, dict(tally)))


if __name__ == "__main__":
    main()
