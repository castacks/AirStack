#!/usr/bin/env python3
"""damage_spread.py — every KIND of damage the pipeline can make, priced.

Builds a grid: one ROW per urban building asset, one COLUMN per (disaster,
level) rung of `disaster/levels.LADDERS`, plus a `pristine` reference column.
Each cell is a settled, self-contained USD, and each cell is TIMED.

    cd AirStack
    uv run --env-file .env.host python scene_gen/tools/damage_spread.py
    ... --assets bg_c,bg_e --disasters earthquake,fire      # a slice
    ... --list                                              # what it would do

Then render the sheet (separate 3.13 + `bpy` env):

    cd scene_gen && uv run --script tools/render_damage_gallery.py \
        _damage_lab/spread/manifest.json --res 480

WHY A RUNG GRID AND NOT A SEVERITY SWEEP
----------------------------------------
`levels.py` draws the line this tool is built on: severity is scene-wide and
continuous; a LEVEL is which rung of a disaster's ladder one asset lands on — a
KIND of damage, not a magnitude. Stage A bakes one archetype per (type, level),
so the rungs are the unit of art that ever ships. A severity sweep previews
something the pipeline never builds.

WHAT EACH CELL ACTUALLY RUNS
----------------------------
The same two stacks the archetype baker dispatches to, unchanged:

    earthquake   `disaster.quake.at_level` — the merged pipeline: subdivide,
                 field, solidify, interior slabs/columns, fracture, consume,
                 blast, settle. Its recipe per rung is `quake.BREAK_PLAN`.
    everything   `disaster.mesh_damage.damage_building` at the MIDPOINT of the
    else         rung's band (a rung's `at` is where it begins, so damaging at
                 `at` renders every archetype at the gentlest damage its name
                 allows), then the same settle.

So the timings below are the real cost of Stage A per archetype, per asset
class — which is the number that decides whether a pack is a twenty-minute
bake or an overnight one.

RUNS ON THE HOST. `AirStack/.venv` carries Isaac Sim 5.1; `.env.host` supplies
OMNI_KIT_ACCEPT_EULA. Assets must be LOCAL (the mirror under
`assets/nucleus/`), because Blender cannot resolve `omniverse://` either.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

_TOOLS = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_TOOLS)
for _p in (_SCENE_GEN, _TOOLS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

_MIRROR = os.path.join(_SCENE_GEN, "assets", "nucleus", "Muyang")
OUT_DIR = os.path.join(_SCENE_GEN, "_damage_lab", "spread")

#: The row set: a spread across BOTH axes that cost scales on — polygon count
#: and physical size. `mesh_damage` subdivides to a 4 m edge floor and cuts a
#: cell count off the plan radius, so a 1,460-face midrise and a 287,430-face
#: tower are two different pipelines' worth of work on the same code path.
#:
#: (key, label, path under assets/nucleus/Muyang, asset-pack scale)
ASSETS = [
    ("bg_c", "BG_Building_C  (1.5k faces, 50x40x66 m)",
     "DownTown/Assets/BG_Building_C.usd", 0.01),
    ("bg_e", "BG_Building_E  (4.2k faces, 60x36x131 m)",
     "DownTown/Assets/BG_Building_E.usd", 0.01),
    ("bg_f", "BG_Building_F  (7.3k faces, 81x50x72 m)",
     "DownTown/Assets/BG_Building_F.usd", 0.01),
    ("mb01", "MBuilding01  (103k faces, 29x19x29 m)",
     "ModernCityEnvironment/Collected_Building01/"
     "SM_MERGED_BP_MBuilding01.usd", 1.0),
    ("mb02", "MBuilding02  (287k faces, 91x96x69 m)",
     "ModernCityEnvironment/Collected_Building02/"
     "SM_MERGED_BP_MBuilding02.usd", 0.01),
]

#: Ladder order, so the sheet reads left to right as the ladders do.
DISASTERS = ("earthquake", "fire", "tornado", "hurricane", "flood")


def _midpoint(rungs, level: float) -> float:
    """The intensity a rung NAME means: the middle of its band, not its floor."""
    names = [r.name for r in rungs]
    i = names.index(level)
    lo = rungs[i].at
    hi = rungs[i + 1].at if i + 1 < len(rungs) else 1.0
    return (lo + hi) / 2.0


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--assets", default="",
                   help="comma-separated keys; default all "
                        f"({','.join(k for k, _, _, _ in ASSETS)})")
    p.add_argument("--disasters", default="",
                   help=f"comma-separated; default {','.join(DISASTERS)}")
    p.add_argument("--levels", default="",
                   help="comma-separated rung names to keep")
    p.add_argument("--steps", type=int, default=1500, help="settle ceiling")
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--out", default=OUT_DIR)
    p.add_argument("--list", action="store_true",
                   help="print the grid and exit; starts no Isaac Sim")
    return p.parse_args(argv)


def build_grid(args):
    """``(assets, columns)`` — the rows and the (disaster, level) columns."""
    from disaster import levels as L

    keys = [k.strip() for k in args.assets.split(",") if k.strip()]
    rows = [a for a in ASSETS if not keys or a[0] in keys]
    want_d = [d.strip() for d in args.disasters.split(",") if d.strip()] \
        or list(DISASTERS)
    want_l = {s.strip() for s in args.levels.split(",") if s.strip()}

    cols = []
    for d in want_d:
        rungs = L.ladder(d, L.STRUCTURE)
        for r in rungs:
            if r.name == "pristine":
                continue          # one shared reference column, not five
            if want_l and r.name not in want_l:
                continue
            cols.append((f"{d}_{r.name}", d, r.name, _midpoint(rungs, r.name)))
    return rows, cols


def main(argv=None):
    args = parse_args(argv)
    rows, cols = build_grid(args)

    if args.list:
        print(f"{len(rows)} asset(s) x {1 + len(cols)} column(s) = "
              f"{len(rows) * (1 + len(cols))} cells")
        for key, label, path, scale in rows:
            here = os.path.join(_MIRROR, path)
            print(f"  {key:5s} {'ok ' if os.path.exists(here) else 'MISSING'} "
                  f"{label}")
        for name, d, lvl, inten in cols:
            print(f"  {name:28s} intensity {inten:.2f}")
        return 0

    missing = [k for k, _l, p, _s in rows
               if not os.path.exists(os.path.join(_MIRROR, p))]
    if missing:
        print(f"[spread] not in the local mirror: {missing}\n"
              f"  run tools/localize_nucleus_assets.py first", file=sys.stderr)
        return 2

    from isaacsim import SimulationApp
    app = SimulationApp(launch_config={"headless": True})

    import omni.kit.app                                         # noqa: E402
    import omni.usd                                             # noqa: E402
    from pxr import Gf, Sdf, UsdGeom                            # noqa: E402

    from disaster import mesh_damage as md                      # noqa: E402
    from disaster import quake                                  # noqa: E402

    os.makedirs(args.out, exist_ok=True)
    usd_dir = os.path.join(args.out, "usd")
    os.makedirs(usd_dir, exist_ok=True)

    manifest = {
        "title": "Damage kinds x urban assets",
        "subtitle": ("one column per ladder rung; earthquake via quake.py, "
                     "the rest via mesh_damage; time is wall-clock per cell"),
        "columns": ["pristine"] + [c[0] for c in cols],
        "rows": [],
    }

    def fresh_stage(path, scale):
        """A new CONTEXT stage with the asset referenced under /World/Building.

        The context stage, not `Usd.Stage.CreateNew`: `settle` drives
        `SimulationContext`, which attaches PhysX to whatever stage the USD
        context holds, so a standalone stage settles an empty scene in silence.
        """
        ctx = omni.usd.get_context()
        ctx.new_stage()
        stage = ctx.get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
        # TYPELESS, not `Xform.Define`. Some of these assets are a single
        # Mesh at their default prim; referencing one onto a prim already
        # declared an Xform makes the local type win, so the geometry composes
        # as an Xform with GeomSubset children and `mesh_prims` finds nothing
        # — every cell then reports 0 cells in 0.0 s and exports the pristine
        # asset. A typeless prim lets the reference supply the type.
        b = stage.DefinePrim(Sdf.Path("/World/Building"))
        b.GetReferences().AddReference(path)
        UsdGeom.Xformable(b).AddScaleOp().Set(Gf.Vec3f(*([float(scale)] * 3)))
        stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
        # PUMP THE APP. A reference is composed by Kit's own update loop, not
        # by `AddReference`; without this the prim has no meshes yet, so
        # `mesh_prims` comes back empty and every cell reports 0 cells / 0.0 s
        # and silently exports the pristine asset.
        for _ in range(4):
            omni.kit.app.get_app().update()
        return stage

    def export(stage, name):
        out = os.path.join(usd_dir, name + ".usd")
        # ROOT LAYER, not `stage.Export`: exporting flattens, which composes
        # the referenced building and dies on attributes Kit's crate reader
        # cannot repack ("unpack unsupported type enum value 0").
        stage.GetRootLayer().Export(out)
        return out

    t_all = time.time()
    for key, label, rel, scale in rows:
        src = os.path.join(_MIRROR, rel)
        row = {"name": label, "cells": {}, "stats": {}}

        stage = fresh_stage(src, scale)
        t = time.time()
        out = export(stage, f"{key}_pristine")
        prim = stage.GetPrimAtPath("/World/Building")
        bnd = md.bounds_of(md.mesh_prims(prim))
        row["cells"]["pristine"] = os.path.relpath(out, args.out)
        row["stats"]["pristine"] = {
            "seconds": round(time.time() - t, 1),
            "footprint_m": [round(float(bnd.dims[0])), round(float(bnd.dims[1]))]
            if bnd else ["?", "?"],
            "height_m": round(float(bnd.dims[2])) if bnd else "?",
            "note": "reference",
        }
        print(f"[spread] {key} pristine  {row['stats']['pristine']['seconds']}s",
              flush=True)

        for name, dtype, level, inten in cols:
            stage = fresh_stage(src, scale)
            prim = stage.GetPrimAtPath("/World/Building")
            seed = args.seed + abs(hash((key, name))) % 9973
            t = time.time()
            try:
                if dtype == "earthquake":
                    rep = quake.at_level(stage, prim, level, seed=seed,
                                         steps=args.steps)
                else:
                    # The archetype baker's path for every non-quake type,
                    # plus the settle it does once over the whole grid.
                    rep = md.damage_building(stage, prim, dtype, inten,
                                             seed=seed, wall_m=quake.WALL_M)
                    rep["settle"] = quake.collapse(stage, rep, seed=seed,
                                                   blast=0.0,
                                                   steps=args.steps)
            except Exception as exc:                            # noqa: BLE001
                # One bad (asset, rung) must not cost the rest of the grid.
                print(f"[spread] SKIP {key} {name}: "
                      f"{type(exc).__name__}: {exc}", flush=True)
                continue
            took = time.time() - t
            st = rep.get("settle") or {}
            out = export(stage, f"{key}_{name}")
            row["cells"][name] = os.path.relpath(out, args.out)
            row["stats"][name] = {
                "seconds": round(took, 1),
                "solve_s": round(float(st.get("solve_s", 0.0)), 1),
                "intensity": round(inten, 3),
                "field": rep.get("field"),
                "dismissed": bool(rep.get("dismissed")),
                "thickened": rep.get("thickened", 0),
                "cells": rep.get("cells", 0),
                "fragments": len(rep.get("loose") or ()),
                "consumed": rep.get("consumed", 0),
                "drop_m": round(float(st.get("drop_mean", 0.0)), 2),
                "spread_m": round(float(st.get("spread_mean", 0.0)), 2),
                "still_moving": st.get("still_moving", 0),
                "note": ("dismissed" if rep.get("dismissed") else
                         f"{rep.get('cells', 0)} cells · "
                         f"{len(rep.get('loose') or ())} loose · "
                         f"{took:.0f}s"),
            }
            print("[spread] {0:5s} {1:26s} {2:6.1f}s  (settle {3:5.1f}s)  "
                  "cells={4:<4d} loose={5:<4d} drop={6:+.2f} m".format(
                      key, name, took, float(st.get("solve_s", 0.0)),
                      rep.get("cells", 0), len(rep.get("loose") or ()),
                      float(st.get("drop_mean", 0.0))), flush=True)

        manifest["rows"].append(row)
        # Written after every ROW, not at the end: a grid this size runs for
        # hours and a partial sheet is still worth rendering.
        with open(os.path.join(args.out, "manifest.json"), "w") as fh:
            json.dump(manifest, fh, indent=1)

    total = time.time() - t_all
    print(f"[spread] {sum(len(r['cells']) for r in manifest['rows'])} cells "
          f"in {total / 60.0:.1f} min -> {args.out}")
    app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
