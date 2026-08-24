#!/usr/bin/env python3
"""quake_preview.py — see what `disaster/quake.py` does, one rung at a time.

Walks the earthquake LADDER (`disaster/levels.py`) for one building and writes
a settled USD per rung, so the whole progression — `cracked`, `soft_storey`,
`partial_collapse`, `pancaked` — can be looked at side by side. That is the
question a damage pipeline actually has to answer: not "does severity 0.9 look
bad" but "do the rungs read as different KINDS of damage, in order".

    cd AirStack
    uv run --env-file .env.host python scene_gen/tools/quake_preview.py
    uv run --env-file .env.host python scene_gen/tools/quake_preview.py \
        --asset airstack://scene_gen/assets/... --levels cracked pancaked

Then render the sheet (separate 3.13 + `bpy` env, see `render_usd.py`):

    cd scene_gen && uv run --script render_usd.py _damage_lab/quake -o /tmp/q

WHY THE LADDER AND NOT A SEVERITY SWEEP
---------------------------------------
`levels.py` draws the distinction this tool is built on: severity is scene-wide
and continuous, the FIELD is a spatial shape, and a LEVEL is which rung of this
disaster's ladder one asset lands on — a kind of damage, not a magnitude.
Stage A bakes one archetype per (type, level) and Stage B picks a rung by
evaluating the field, so the rungs are the unit of art that ever gets shipped.
Previewing anything else previews something the pipeline does not build.

Each rung is a SET of mechanisms over regions of the plan (`quake.RUNG_PLAN`),
not one mode: `partial_collapse` shears one side off, drops the ground floor
under another, and cracks the rest. Intensity, release, consume and blast are
properties of those mechanisms, not of any severity. Severity never enters this
tool: its only job in the real pipeline is choosing which rung a building lands
on (`quake.damage_at`).

RUNS ON THE HOST. `AirStack/.venv` carries Isaac Sim 5.1, so `omni.physx` and
`SimulationContext` are here; `.env.host` supplies OMNI_KIT_ACCEPT_EULA.
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

#: The urban tower every other pipeline failed on — 91 x 96 x 69 m, 287,430
#: faces in ONE open-shell mesh. Mirrored locally by
#: `tools/localize_nucleus_assets.py`; `omniverse://` works too, inside Kit.
DEFAULT_ASSET = os.path.join(
    _SCENE_GEN, "assets", "nucleus", "Muyang", "ModernCityEnvironment",
    "Collected_Building02", "SM_MERGED_BP_MBuilding02.usd")
DEFAULT_SCALE = 0.01                     # as `urban_nucleus.yaml` declares it
OUT_DIR = os.path.join(_SCENE_GEN, "_damage_lab", "quake")


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--asset", default=DEFAULT_ASSET)
    p.add_argument("--scale", type=float, default=DEFAULT_SCALE,
                   help="asset-pack scale; 0.01 for centimetre-authored USDs")
    p.add_argument("--levels", nargs="*", default=None,
                   help="rung names; default is the whole earthquake ladder")
    p.add_argument("--steps", type=int, default=1500, help="settle ceiling")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--out", default=OUT_DIR)
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    from isaacsim import SimulationApp
    app = SimulationApp(launch_config={"headless": True})

    import omni.usd                                             # noqa: E402
    from pxr import Gf, Sdf, UsdGeom                            # noqa: E402

    from disaster import levels as L                            # noqa: E402
    from disaster import quake                                  # noqa: E402

    rungs = L.ladder("earthquake", L.STRUCTURE)
    names = [r.name for r in rungs]
    want = args.levels or names
    os.makedirs(args.out, exist_ok=True)

    report = []
    for level in want:
        if level not in names:
            print(f"[quake_preview] no such rung: {level} (have {names})")
            continue
        # A NEW CONTEXT STAGE PER RUNG. `settle` drives `SimulationContext`,
        # which attaches PhysX to whatever stage the USD context holds — a
        # standalone `Usd.Stage.CreateNew` is not that stage and the solver
        # silently runs on an empty scene.
        ctx = omni.usd.get_context()
        ctx.new_stage()
        stage = ctx.get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
        b = UsdGeom.Xform.Define(stage, Sdf.Path("/World/Building"))
        b.GetPrim().GetReferences().AddReference(args.asset)
        b.AddScaleOp().Set(Gf.Vec3f(*([float(args.scale)] * 3)))
        stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

        # THE RUNG DRIVES IT, NOT AN INTENSITY. `at_level` reads
        # `quake.RUNG_PLAN[level]` — the mechanisms in play and how much of
        # the plan each takes. Severity is nowhere in this call, because the
        # rung is a KIND of damage and severity's only job was choosing it.
        plan = quake.plan_for(level)
        inten = plan.intensity if plan else 0.0
        t = time.time()
        rep = quake.at_level(stage, stage.GetPrimAtPath("/World/Building"),
                             level, seed=args.seed, steps=args.steps)
        took = time.time() - t
        st = rep.get("settle") or {}
        print("[quake_preview] {0:<17} intensity={1:.2f}  {2:.0f}s  "
              "thickened={3} cells={4} loose={5} consumed={6}  "
              "drop={7:+.2f} m".format(
                  level, inten, took, rep.get("thickened"), rep.get("cells"),
                  len(rep.get("loose") or ()), rep.get("consumed", 0),
                  st.get("drop_mean", 0.0)), flush=True)

        out = os.path.join(args.out, f"earthquake_{level}.usd")
        # ROOT LAYER, not `stage.Export`: exporting flattens, which composes
        # the referenced building and dies on attributes Kit's crate reader
        # cannot repack ("unpack unsupported type enum value 0").
        stage.GetRootLayer().Export(out)
        report.append({"level": level, "intensity": round(inten, 3),
                       "seconds": round(took, 1),
                       "thickened": rep.get("thickened"),
                       "cells": rep.get("cells"),
                       "loose": len(rep.get("loose") or ()),
                       "consumed": rep.get("consumed", 0),
                       "drop_mean": round(st.get("drop_mean", 0.0), 2),
                       "spread_mean": round(st.get("spread_mean", 0.0), 2),
                       "still_moving": st.get("still_moving"),
                       "usd": out})

    with open(os.path.join(args.out, "manifest.json"), "w") as fh:
        json.dump(report, fh, indent=2)
    print(f"[quake_preview] wrote {len(report)} USD(s) to {args.out}")
    app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
