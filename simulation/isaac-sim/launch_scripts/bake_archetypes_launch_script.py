#!/usr/bin/env python
"""
Bake one USD per house TYPE and tree TYPE — the archetype library that lets the
1600 x 1200 plat be ASSEMBLED by reference instead of fractured/settled live.

    ARCH_DIR=/isaac-sim/AirStack/scene_gen/assets/archetypes \
    ISAAC_SIM_SCRIPT_NAME=bake_archetypes_launch_script.py airstack up isaac-sim

Builds every (house style x structural level) and (tree species x burn level)
ONCE, on a spread-out grid so nothing collides, fractures/burns each with the
SAME code the live scene uses (`disaster.damage_flow`, `disaster.vegetation`),
settles the whole grid in one pass, and exports each object re-centred to the
origin via `disaster.bake`. The 1600 launcher then references
`house_<style>_<level>.usd` / `tree_<species>_<level>.usd` at each placement.
"""

import math
import os
import random
import sys
import time

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("omni.kit.window.script_editor")

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

import numpy as np                                              # noqa: E402
import scene_generator as sg                                   # noqa: E402
from scene_prep import get_stage_meters_per_unit               # noqa: E402
from detail import modular_house as mh                         # noqa: E402
from disaster import bake, damage, damage_flow, fracture, settle  # noqa: E402
from disaster import vegetation as veg                         # noqa: E402

PARENT = "/World/stage/generated"
SEED = int(os.environ.get("ARCH_SEED", "7"))
OUT_DIR = os.environ.get(
    "ARCH_DIR", os.path.join(_SCENE_GEN_DIR, "assets", "archetypes"))
GRID = 40.0

HOUSE_LEVELS = ("pristine", "scorched", "roof_collapsed",
                "partial_collapse", "burned_out", "rubble")
_FINISH = {"roof_collapsed": "char", "partial_collapse": "char",
           "burned_out": "ash", "rubble": "ash"}

TREE_SPECIES = {
    "Black_Oak": "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/Black_Oak/Black_Oak.usd",
    "Shumard_Oak": "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/Shumard_Oak/Shumard_Oak.usd",
    "Douglas_Fir": "airstack://scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/Douglas_Fir.usd",
    "Largetooth_Aspen": "airstack://scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/Largetooth_Aspen.usd",
    "Common_Apple": "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/Common_Apple/Common_Apple.usd",
    "American_Beech": "airstack://scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/American_Beech.usd",
}
TREE_LEVELS = ("scorched", "torched", "snag", "fallen", "stump")


def build_ground_and_light(stage):
    plane = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/arch_ground"))
    e = 800.0
    plane.CreatePointsAttr([Gf.Vec3f(-e, -e, 0), Gf.Vec3f(e, -e, 0),
                            Gf.Vec3f(e, e, 0), Gf.Vec3f(-e, e, 0)])
    plane.CreateFaceVertexCountsAttr([4])
    plane.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    plane.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    plane.CreateDisplayColorAttr([Gf.Vec3f(0.2, 0.25, 0.15)])
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/dome"))
    dome.CreateIntensityAttr(900.0)
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/key"))
    key.CreateIntensityAttr(2200.0)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 30.0))


def main():
    omni.timeline.get_timeline_interface().stop()
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/stage"))
    _, ssf = get_stage_meters_per_unit(stage)
    build_ground_and_light(stage)
    fracture.ensure_deps()
    os.makedirs(OUT_DIR, exist_ok=True)
    t_build0 = time.time()

    # ---- HOUSES: build every (style, level) on a grid ----------------------
    styles = list(mh.STYLES.keys())
    hcombos = [(st, lv) for st in styles for lv in HOUSE_LEVELS]
    ncol = int(math.ceil(math.sqrt(len(hcombos))))
    house_specs = []          # [style, level, X, Y, parent, placements, frags]
    for idx, (st, lv) in enumerate(hcombos):
        X, Y = (idx % ncol) * GRID, (idx // ncol) * GRID
        parent = "{0}/h_{1}_{2}".format(PARENT, st, lv)
        UsdGeom.Scope.Define(stage, Sdf.Path(parent))
        pls = mh.build_building(st, X, Y, 0.0, random.Random(SEED),
                                category="house")
        pal = mh.STYLES[st].get("palette")
        if pal:
            for q in pls:
                q["palette"] = pal
        sg.apply_placements(stage, pls, parent, ssf)
        mh.apply_palette(stage, pls, parent)
        house_specs.append([st, lv, X, Y, parent, pls, []])
    print("[arch] built {0} house grid cells".format(len(house_specs)))

    all_loose, all_static = [], []
    for spec in house_specs:
        st, lv, X, Y, parent, pls, _f = spec
        if lv == "pristine":
            continue
        if lv == "scorched":
            damage.soot_materials(stage, pls, parent, random.Random(SEED),
                                  strength=(3, 5))
            continue
        mats = damage.char_materials(stage, parent)
        seed = SEED + (abs(hash((st, lv))) % 100000)
        frags = damage_flow.damage_building(
            stage, parent, pls, "{0}_{1}".format(st, lv), lv, _FINISH[lv],
            mats, random.Random(seed), np.random.default_rng(seed))
        spec[6] = frags
        all_loose.extend(frags)
        damage.soot_materials(stage, pls, parent, random.Random(SEED),
                              strength=(4, 6))
    print("[arch] houses damaged: {0} loose fragment(s)".format(len(all_loose)))

    # ---- TREES: reference each species, burn to each level -----------------
    tree_specs = []           # [species, level, X, Y, tree_path, extra_paths]
    tcombos = [(sp, lv) for sp in TREE_SPECIES for lv in TREE_LEVELS]
    tcol = int(math.ceil(math.sqrt(len(tcombos))))
    y0 = (len(hcombos) // ncol + 2) * GRID    # trees below the house grid
    for idx, (sp, lv) in enumerate(tcombos):
        X = (idx % tcol) * GRID
        Y = y0 + (idx // tcol) * GRID
        tp = "{0}/t_{1}_{2}".format(PARENT, sp, lv)
        prim = stage.DefinePrim(Sdf.Path(tp))
        prim.GetReferences().AddReference(sg._join_asset_root(TREE_SPECIES[sp], ""))
        xf = UsdGeom.Xformable(prim)
        xf.AddTranslateOp().Set(Gf.Vec3d(X, Y, 0.0))
        xf.AddScaleOp().Set(Gf.Vec3f(0.01, 0.01, 0.01))
        for _ in range(2):
            omni.kit.app.get_app().update()
        rng = random.Random(SEED + (abs(hash((sp, lv))) % 100000))
        res = veg.burn_tree(stage, tp, lv, PARENT, PARENT + "/tdeb_%s_%s" % (sp, lv),
                            rng, debris_scale=0.5, ground_z=0.0, verbose=False)
        extra = list(res.get("statics", [])) + list(res.get("loose", [])) \
            + list((res.get("info") or {}).get("made", []))
        all_loose.extend(res.get("loose", []))
        all_static.extend(res.get("statics", []))
        tree_specs.append([sp, lv, X, Y, tp, extra])
    print("[arch] built {0} tree grid cells".format(len(tree_specs)))

    # ---- settle the whole grid once ---------------------------------------
    for spec in house_specs:
        for q in spec[5]:
            pr = stage.GetPrimAtPath(q["prim_path"])
            if pr and pr.IsValid() and pr.IsActive():
                all_static.append(q["prim_path"])
    all_static.append("/World/arch_ground")
    for _ in range(20):
        omni.kit.app.get_app().update()
    if all_loose:
        settle.run(stage, all_loose, all_static, steps=420, kick=0.15,
                   rng=random.Random(SEED), bake_result=True)
    for _ in range(10):
        omni.kit.app.get_app().update()

    # ---- export every archetype, re-centred to the origin ------------------
    records, miss = [], 0

    def _safe_export(paths, out, rec, meta):
        try:
            if bake.export_object(stage, None, paths, out, recenter=rec):
                m, ok, ms = bake.validate(out)
                records.append(dict(usd=os.path.abspath(out), meshes=m,
                                    bound_missing=ms, **meta))
                return ms
        except Exception as exc:
            print("[arch] export FAILED for {0}: {1}".format(
                os.path.basename(out), exc))
        return 0

    for st, lv, X, Y, parent, pls, frags in house_specs:
        miss += _safe_export(
            [q["prim_path"] for q in pls] + frags,
            os.path.join(OUT_DIR, "house_{0}_{1}.usd".format(st, lv)),
            (X, Y, 0.0), dict(kind="house", style=st, level=lv))
    for sp, lv, X, Y, tp, extra in tree_specs:
        miss += _safe_export(
            [tp] + extra,
            os.path.join(OUT_DIR, "tree_{0}_{1}.usd".format(sp, lv)),
            (X, Y, 0.0), dict(kind="tree", species=sp, level=lv))
    bake.write_manifest(os.path.join(OUT_DIR, "archetypes.json"), records)
    dt = time.time() - t_build0
    nh = sum(1 for r in records if r["kind"] == "house")
    nt = sum(1 for r in records if r["kind"] == "tree")
    sz = sum(os.path.getsize(r["usd"]) for r in records
             if os.path.exists(r["usd"])) / 1e6

    print("\n" + "=" * 72)
    print("ARCHETYPE BAKE")
    print("  {0} house + {1} tree archetypes -> {2}".format(nh, nt, OUT_DIR))
    print("  {0:.0f} MB, built+baked in {1:.0f} s, {2} unresolved material(s)"
          .format(sz, dt, miss))
    print("=" * 72 + "\n")

    app = omni.kit.app.get_app()
    while simulation_app.is_running():
        app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
