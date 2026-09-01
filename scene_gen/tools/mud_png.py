#!/usr/bin/env python3
"""mud_png.py — the surge deposit/pond layout, without Isaac Sim.

    python3 tools/mud_png.py --preset suburb_hurricane_500_l3 --seed 11 \\
        --out ~/hurricane_previews/offline/mounds/deposits_layout_l3.png

WHY THIS EXISTS
---------------
The STREAM-W wet-mounds fix (`.agents/skills/build-hurricane-scenes/
SESSION_2026-08-31.md` §4 item 3 and the user's "improve the textures being
used for the wet dirt mounds") touches `disaster.surge`'s deposit/pond
placement AND material selection: `build_ponding`'s rim now feathers by a
constant METRES offset instead of a radius ratio, and `build_deposits` now
splits wrack/washover specs into a WET band (near the current waterline,
`_DEPOSIT_WET_BAND_M`) versus the spatial-noise dry/ageing tonal ladder
further out. Both are geometric/placement facts this tool can check WITHOUT
Isaac Sim or a real USD stage — `pond_specs`, `wrack_specs`,
`_washover_specs` and `signed_depth_at` are all pure Python (the module
docstring's own split, "a pure-Python field/scatter half a test can pin with
no pxr on the path").

WHAT IT DRAWS
-------------
Over the plate's own extent:
  * the shoreline (`depth_at == 0`) and the `_DEPOSIT_WET_BAND_M` band around
    it, as a shaded strip — this is the boundary the wet/dry material split
    (both here and in the launcher's silt-overlay call) actually keys off;
  * every pond as two circles: the CORE radius (`r_m`) and the RIM radius
    (`r_m + pond_rim_feather_m`) — the additive feather this round replaced
    the old `_POND_RIM_GROW` ratio with;
  * every wrack windrow as its own ridge polyline, coloured wet vs dry by
    the SAME `_DEPOSIT_WET_BAND_M` test `build_deposits` uses;
  * every washover fan/mound as an ellipse at its own `(rx, ry, yaw)`, same
    wet/dry colouring.

Nothing here authors a single USD prim. `disaster.surge`'s stage-touching
entry points (`water_materials`, `build_inundation`, `build_ponding`,
`build_deposits`) are never called — only their pure-Python spec/field
halves.
"""

import argparse
import os
import sys
import types

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
sys.path.insert(0, _HERE)
sys.path.insert(0, _SCENE_GEN)

# Same stub idiom `tornado_png.py:40-56` uses, and the one this preset's own
# yaml (`suburb_hurricane_500_l3.yaml`) recommends for loading a compiled
# scene config with no `pxr` on a bare dev host. `setdefault` is a no-op if a
# real `pxr` is already importable (e.g. `usd-core` pip-installed), so this
# is safe either way.
for _m in ("pxr", "pxr.Gf", "pxr.Sdf", "pxr.Usd", "pxr.UsdGeom", "pxr.UsdShade",
          "pxr.UsdSkel", "pxr.Vt"):
    sys.modules.setdefault(_m, types.ModuleType(_m))
for _n in ("Gf", "Sdf", "Usd", "UsdGeom", "UsdShade", "UsdSkel", "Vt"):
    setattr(sys.modules["pxr"], _n, types.SimpleNamespace())

import random                                                   # noqa: E402
import numpy as np                                              # noqa: E402
import matplotlib                                                # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                 # noqa: E402
from matplotlib.patches import Circle, Ellipse                  # noqa: E402

from disaster import surge                                      # noqa: E402


def _load_region_and_surge_cfg(preset, seed_override=None):
    """`(region, scfg, span, seed)` for *preset*, the SAME merge recipe
    `suburb_hurricane_launch_script.py`'s "3) THE WATER" section uses:
    `surge.resolve_cfg` layered with the preset's OWN `disaster.hurricane`
    sub-block winning over `surge.knobs_from_env`'s environment defaults.
    """
    from compile_disaster import load_scene_config

    config = load_scene_config(preset)
    reg = (config.get("layout") or {}).get("region_m") or [500.0, 500.0]
    rw, rh = float(reg[0]), float(reg[1])
    region = (-rw / 2.0, -rh / 2.0, rw / 2.0, rh / 2.0)
    span = max(rw, rh)

    hsub = ((config.get("disaster") or {}).get("hurricane") or {})
    scfg = surge.resolve_cfg({k: v for k, v in hsub.items()
                              if k in surge.DEFAULTS})
    scfg.update(surge.knobs_from_env(span))
    for k, v in hsub.items():
        if k in surge.DEFAULTS:
            scfg[k] = v
    seed = int(seed_override if seed_override is not None
              else config.get("seed", 11))
    return region, scfg, span, seed


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--preset", default="suburb_hurricane_500_l3",
                    help="preset name (default: suburb_hurricane_500_l3)")
    ap.add_argument("--seed", type=int, default=11,
                    help="rng seed for pond/wrack/washover scatter -- "
                        "11 matches the launcher's HUR_SEED default")
    ap.add_argument("--out", default=os.path.expanduser(
        "~/hurricane_previews/offline/mounds/deposits_layout_l3.png"))
    args = ap.parse_args()

    region, scfg, span, _cfg_seed = _load_region_and_surge_cfg(
        args.preset, seed_override=None)
    # THE LAUNCHER'S water rng is `random.Random(HUR_SEED + 61)`
    # (`suburb_hurricane_launch_script.py`'s "5) THE WATER" section), shared
    # across `build_inundation`/`build_ponding`/`build_deposits` in that
    # order -- matched here so a seed of 11 draws the SAME pond/wrack/
    # washover population the launcher would place, not just a same-seeded
    # but differently-ordered one.
    wrng = random.Random(args.seed + 61)

    ponds = surge.pond_specs(scfg, region, wrng)
    wrack = surge.wrack_specs(scfg, region, wrng)
    washover = surge._washover_specs(scfg, region, wrng)
    feather_m = float(scfg["pond_rim_feather_m"])
    wet_band_m = float(surge._DEPOSIT_WET_BAND_M)

    sd_fn = surge.signed_depth_at(scfg, region, None)
    cov = surge.coverage(scfg, region, None, n=96)

    x0, y0, x1, y1 = region
    n_grid = 220
    xs = np.linspace(x0, x1, n_grid)
    ys = np.linspace(y0, y1, n_grid)
    xg, yg = np.meshgrid(xs, ys)
    sd_grid = np.vectorize(sd_fn)(xg, yg)

    fig, ax = plt.subplots(figsize=(9, 9), dpi=140)
    ax.set_facecolor("#dcdccf")     # dry ground

    # WET vs DRY as a filled contour of the SAME signed-depth field every
    # material decision in this round is keyed off -- wet water body
    # (sd > 0), the wet silt band (|sd| <= wet_band_m), dry silt beyond it.
    levels = [sd_grid.min() - 1.0, -wet_band_m, 0.0, wet_band_m,
             sd_grid.max() + 1.0]
    colours = ["#c9b892", "#8a6d4a", "#2c4a63", "#3d6f93"]
    ax.contourf(xg, yg, sd_grid, levels=levels, colors=colours)
    ax.contour(xg, yg, sd_grid, levels=[0.0], colors="black", linewidths=1.2)

    def _in_wet_band(x, y):
        return abs(sd_fn(x, y)) <= wet_band_m

    n_pond_wet = 0
    for p in ponds:
        x, y, r = float(p["x"]), float(p["y"]), float(p["r_m"])
        wet = _in_wet_band(x, y)
        n_pond_wet += int(wet)
        colour = "#5fb0e0" if not p.get("paved") else "#3d7ea6"
        ax.add_patch(Circle((x, y), r, facecolor=colour, edgecolor="none",
                            alpha=0.85, zorder=3))
        ax.add_patch(Circle((x, y), r + feather_m, facecolor="none",
                            edgecolor=colour, linewidth=0.8, linestyle="--",
                            alpha=0.6, zorder=3))

    n_wrack_wet = 0
    for w in wrack:
        wet = _in_wet_band(float(w["x"]), float(w["y"]))
        n_wrack_wet += int(wet)
        colour = "#0c0c0a" if wet else "#4a3a28"
        st = w["stations"]
        xs_r = [s[0] for s in st]
        ys_r = [s[1] for s in st]
        lw = max(1.0, float(w.get("width_m", 0.5)) * 3.0)
        ax.plot(xs_r, ys_r, color=colour, linewidth=lw, alpha=0.85,
               solid_capstyle="round", zorder=4)

    n_sand_wet = 0
    for s in washover:
        wet = _in_wet_band(float(s["x"]), float(s["y"]))
        n_sand_wet += int(wet)
        colour = "#c9a24a" if wet else "#e2cf9a"
        e = Ellipse((float(s["x"]), float(s["y"])),
                   width=2.0 * float(s["rx"]), height=2.0 * float(s["ry"]),
                   angle=float(s["yaw"]), facecolor=colour, edgecolor="#333",
                   linewidth=0.5, alpha=0.75, zorder=2)
        ax.add_patch(e)

    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.set_title(
        "{0} seed={1}  ponds {2} ({3} wet)  wrack {4} ({5} wet)  "
        "washover {6} ({7} wet)\ncoverage={8:.1%}  wet_band={9:.1f} m  "
        "pond_rim_feather={10:.2f} m".format(
            args.preset, args.seed, len(ponds), n_pond_wet, len(wrack),
            n_wrack_wet, len(washover), n_sand_wet, cov, wet_band_m,
            feather_m),
        fontsize=9)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")

    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    fig.tight_layout()
    fig.savefig(args.out)
    print("[mud_png] {0}: {1} pond(s) ({2} wet), {3} wrack ({4} wet), "
         "{5} washover ({6} wet), coverage={7:.4f} -> {8}".format(
             args.preset, len(ponds), n_pond_wet, len(wrack), n_wrack_wet,
             len(washover), n_sand_wet, cov, args.out))


if __name__ == "__main__":
    main()
