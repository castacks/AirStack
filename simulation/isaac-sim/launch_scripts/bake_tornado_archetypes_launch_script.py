#!/usr/bin/env python
"""
Bake one USD per house TYPE and tree TYPE for a TORNADO — the archetype library
`suburb_tornado_launch_script.py` assembles the track from.

    ARCH_DIR=/isaac-sim/AirStack/scene_gen/assets/archetypes_tornado \
    ISAAC_SIM_SCRIPT_NAME=bake_tornado_archetypes_launch_script.py \
    airstack up isaac-sim

The wind counterpart of `bake_archetypes_launch_script.py`, and structurally
the same script: build every (house style x damage level) and (tree species x
wind level) ONCE on a spread-out grid, break each with the SAME code the live
scene would use, settle the whole grid in one pass, and export each object
re-centred to the origin via `disaster.bake`. What differs is every table and
three of the four fracture arguments — see `disaster/wind_flow.py`.

WHY THIS IS A SEPARATE DIRECTORY FROM THE FIRE BAKE
---------------------------------------------------
`ARCH_DIR` must NOT be the wildfire library. Both write
`house_<style>_<level>.usd`, and while the level names differ today
(`roof_collapsed` and `partial_collapse` are in BOTH ladders), a shared
directory means a tornado `roof_collapsed` silently overwrites the burnt one
and the wildfire plat comes up with unburnt wreckage in it. Default is
`assets/archetypes_tornado`.

THE THROW DIRECTION IS LOCAL +X, ALWAYS
---------------------------------------
`settle` runs with a `bias` along +X, so every archetype's debris leans and
travels the same way in its own frame. The assembly then chooses the yaw:

  * a house that is still recognisable as a house (`roof_stripped` up to
    `partial_collapse`) is yawed to the STREET, because a house facing the
    wrong way is the most obvious defect a suburb can have, and its debris
    field is small enough that its direction is not the read;
  * a house that is a pile (`leveled`, `swept`) is yawed to the TRACK,
    because nobody can tell which way a pile was facing and the direction its
    material went is the whole point.

Same for trees: a fallen archetype is baked lying toward +X and yawed into the
track at assembly. That is legal — the skill's "do not roll or pitch a baked
archetype" warning is about X and Y, and a yaw about Z carries the debris bed
round with the object exactly as intended.

Env knobs:

    ARCH_DIR    output directory (default `scene_gen/assets/archetypes_tornado`)
    ARCH_SEED   rng seed (default 7)
    ARCH_KINDS  which halves to rebuild — `house,tree` (default), `tree`,
                `house`. The manifest is MERGED on a restricted run.
    THROW_MPS   settle bias magnitude, m/s (default 9.0). This is the ONE knob
                that decides whether wreckage leans downwind or piles in place.
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
from disaster import bake, fracture, planks, settle, wind_flow  # noqa: E402
from disaster import tornado as tn                             # noqa: E402
from disaster import vegetation as veg                         # noqa: E402

PARENT = "/World/stage/generated"
SEED = int(os.environ.get("ARCH_SEED", "7"))
OUT_DIR = os.environ.get(
    "ARCH_DIR", os.path.join(_SCENE_GEN_DIR, "assets", "archetypes_tornado"))
# 50 rather than the fire bake's 40. Nothing here is consumed, so a wrecked
# house keeps more material than a burnt one does, and the settle THROWS it —
# at 9 m/s a fragment can carry the better part of ten metres before it beds
# in. A piece that lands inside the neighbouring cell is not an export bug
# (the exporter is given explicit prim paths) but it does settle against the
# wrong geometry, which is a physically wrong resting pose.
GRID = 50.0
KINDS = set(k.strip().lower()
            for k in os.environ.get("ARCH_KINDS", "house,tree").split(",")
            if k.strip())
THROW_MPS = float(os.environ.get("THROW_MPS", "9.0"))

HOUSE_LEVELS = tn.HOUSE_LEVELS            # pristine .. swept
# `pristine` trees are referenced from the green species USD by the assembly
# and never baked — same as the wildfire path.
TREE_LEVELS = ("limbed", "leaning", "fallen", "snapped")

TREE_SPECIES = {
    "Black_Oak": "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/Black_Oak/Black_Oak.usd",
    "Shumard_Oak": "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/Shumard_Oak/Shumard_Oak.usd",
    "Douglas_Fir": "airstack://scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/Douglas_Fir.usd",
    "Largetooth_Aspen": "airstack://scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/Largetooth_Aspen.usd",
    "Common_Apple": "airstack://scene_gen/assets/aec/tower/Assets/Vegetation/Common_Apple/Common_Apple.usd",
    "American_Beech": "airstack://scene_gen/assets/aec/brownstone/Assets/Vegetation/Trees/American_Beech.usd",
}

# LEVELS WHOSE BASE IS DELIBERATELY OFF THE GROUND. `bake.export_object`'s
# `drop_to_ground` seats an object by its FIRST root's world min-z, and for a
# tipped tree that minimum is a crown branch below grade — so dropping it
# would lift the whole tree until the lowest twig touched z=0 and leave the
# trunk floating several metres up. `tip_tree` already seats the base by
# construction (it pivots about the local origin) and lifts it by the root
# plate's own radius on purpose.
_NO_DROP = ("leaning", "fallen")


def build_ground_and_light(stage):
    plane = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/arch_ground"))
    e = 900.0
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


def merge_manifest(path, records, kinds):
    """The new records, plus whatever the manifest had for OTHER kinds.

    Same bookkeeping the fire bake documents: a restricted run
    (`ARCH_KINDS=tree`) must not delete the manifest entries for the half it
    did not build, because those USDs are still on disk and the assembly
    drives entirely off the manifest.
    """
    old = []
    if os.path.exists(path):
        try:
            old = bake.read_manifest(path)
        except Exception as exc:
            print("[tarch] existing manifest unreadable, replacing it: "
                  "{0}".format(exc))
            old = []

    def key(r):
        return (r.get("kind"), r.get("style") or r.get("species"),
                r.get("level"))

    fresh = set(key(r) for r in records)
    kept = [r for r in old
            if r.get("kind") not in kinds and key(r) not in fresh]
    if kept:
        print("[tarch] manifest: keeping {0} record(s) for kind(s) not built "
              "this run".format(len(kept)))
    return kept + records


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
    # A FRESH CONTAINER PAYS A RUINED RUN, NOT A PIP INSTALL. trimesh caches
    # which engines are available at import time, so a package installed after
    # `import trimesh` is invisible for the life of the process: every capped
    # slice silently returns empty and every archetype bakes with no debris,
    # with a complete-looking banner. See the wildfire skill's Environment
    # section.
    fracture.ensure_deps()
    os.makedirs(OUT_DIR, exist_ok=True)
    t_build0 = time.time()

    # One shared plank/lumber material set for the whole grid — `wreck_building`
    # binds bare-timber fragments out of it, so a broken stud in an archetype
    # and a loose board the assembly scatters beside it agree.
    pmats = planks.materials(stage, PARENT)
    mat_cache = {}

    # SOIL FOR THE ROOT PLATES. A windthrown tree levers a wheel of earth out
    # of the ground and that plate is the feature that tells a fallen tree
    # from a felled one at any distance — so it has to look like earth rather
    # than like whatever `displayColor` the renderer falls back to. Referenced
    # once here and handed to every `wind_tree` call; `bake.export_object`
    # rebuilds it by value into each archetype that uses it.
    SOIL_PATH = PARENT + "/WindLooks/soil"
    try:
        _sp = stage.DefinePrim(Sdf.Path(SOIL_PATH))
        _sp.GetReferences().AddReference(sg._join_asset_root(
            "airstack://scene_gen/assets/materials/Dirt.usda", ""))
        _sp.Load()
        if not UsdShade.Material.Get(stage, SOIL_PATH):
            SOIL_PATH = ""
    except Exception as _exc:
        print("[tarch] soil material unavailable ({0}); root plates will "
              "render untextured".format(_exc))
        SOIL_PATH = ""

    # ---- HOUSES ----------------------------------------------------------
    # The grid geometry is computed whether or not houses are built this run,
    # so the TREE rows do not move between a full bake and `ARCH_KINDS=tree`.
    styles = list(mh.STYLES.keys())
    hcombos = [(st, lv) for st in styles for lv in HOUSE_LEVELS]
    ncol = max(1, int(math.ceil(math.sqrt(len(hcombos)))))
    house_specs = []          # [style, level, X, Y, parent, placements, frags]
    for idx, (st, lv) in enumerate(hcombos if "house" in KINDS else []):
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
    print("[tarch] built {0} house grid cells".format(len(house_specs)))

    all_loose, all_static = [], []
    for spec in house_specs:
        st, lv, X, Y, parent, pls, _f = spec
        if lv == "pristine":
            continue
        seed = SEED + (abs(hash((st, lv))) % 100000)
        frags = wind_flow.wreck_building(
            stage, parent, pls, "{0}_{1}".format(st, lv), lv,
            random.Random(seed), np.random.default_rng(seed), pmats,
            mat_cache=mat_cache)
        spec[6] = frags
        all_loose.extend(frags)
        # NO `soot_materials` CALL, and its absence is the point. Every
        # material pass in `disaster.damage` composites a soot wash; there is
        # no parameterisation that turns that off, and its lowest coverage
        # bucket is 0.52. A tornado scene calls none of them.
    print("[tarch] houses wrecked: {0} loose fragment(s)".format(
        len(all_loose)))

    # ---- TREES -----------------------------------------------------------
    tree_specs = []           # [species, level, X, Y, tree_path, extra_paths]
    tree_loose = []           # -> convexDecomposition (a limb is not convex)
    # SKIP WHAT THE ASSEMBLY WILL NEVER ASK FOR. `tornado.NO_UPROOT` promotes
    # `fallen` to `snapped` for the wide-crowned species, so baking their
    # fallen archetype is pure cost — and a stale one on disk is worse than
    # absent, because it would be referenced by any future caller that forgot
    # the promotion.
    tcombos = [(sp, lv) for sp in TREE_SPECIES for lv in TREE_LEVELS
               if not (lv == "fallen" and sp in tn.NO_UPROOT)]
    tcol = max(1, int(math.ceil(math.sqrt(len(tcombos)))))
    y0 = (len(hcombos) // ncol + 2) * GRID    # trees below the house grid
    for idx, (sp, lv) in enumerate(tcombos if "tree" in KINDS else []):
        X = (idx % tcol) * GRID
        Y = y0 + (idx // tcol) * GRID
        tp = "{0}/t_{1}_{2}".format(PARENT, sp, lv)
        prim = stage.DefinePrim(Sdf.Path(tp))
        prim.GetReferences().AddReference(
            sg._join_asset_root(TREE_SPECIES[sp], ""))
        xf = UsdGeom.Xformable(prim)
        xf.AddTranslateOp().Set(Gf.Vec3d(X, Y, 0.0))
        xf.AddScaleOp().Set(Gf.Vec3f(0.01, 0.01, 0.01))
        for _ in range(2):
            omni.kit.app.get_app().update()
        rng = random.Random(SEED + (abs(hash((sp, lv))) % 100000))
        # `azimuth_deg=0` — everything falls toward +X in the archetype's own
        # frame, and the assembly's yaw aims it. See the header.
        res = veg.wind_tree(stage, tp, lv, PARENT,
                            PARENT + "/tdeb_%s_%s" % (sp, lv), rng,
                            azimuth_deg=0.0, soil_material_path=SOIL_PATH,
                            debris_scale=0.5, ground_z=0.0,
                            seated_collide=False, verbose=False)
        extra = list(res.get("statics", [])) + list(res.get("loose", [])) \
            + list(res.get("seated", [])) \
            + list((res.get("info") or {}).get("made", []))
        all_loose.extend(res.get("loose", []))
        all_static.extend(res.get("statics", []))
        tree_loose.extend(res.get("loose", []))
        tree_specs.append([sp, lv, X, Y, tp, extra])
        # THE SEATED LEAN IS THE DIAGNOSTIC for the whole windthrow path, and
        # it is not recoverable after the export. `tip_tree` bisects it down
        # per species until the crown is just into the turf, so a value stuck
        # at the 46 deg floor means that species' crown is too wide to lie
        # down and it should be in `tornado.NO_UPROOT` — which is exactly how
        # Black_Oak was found. Printed here rather than inferred later.
        if res.get("lean_deg"):
            print("[tarch]   {0:<18} {1:<8} seated at {2:.0f} deg".format(
                sp, lv, res["lean_deg"]))
    print("[tarch] built {0} tree grid cells".format(len(tree_specs)))

    # ---- settle the whole grid once, WITH A WIND ---------------------------
    for spec in house_specs:
        for q in spec[5]:
            pr = stage.GetPrimAtPath(q["prim_path"])
            if pr and pr.IsValid() and pr.IsActive():
                all_static.append(q["prim_path"])
    all_static.append("/World/arch_ground")
    for _ in range(20):
        omni.kit.app.get_app().update()
    if all_loose:
        approx_map = {pth: "convexDecomposition" for pth in tree_loose}
        bias = wind_flow.throw_bias(0.0, 0.0, THROW_MPS)
        print("[tarch] settling {0} bodies with a {1:.1f} m/s bias toward +X"
              .format(len(all_loose), THROW_MPS))
        settle.run(stage, all_loose, all_static,
                   # FAR MORE STEPS THAN THE FIRE BAKE'S 420, and 700 was
                   # still not enough: measured on the first full run, all
                   # 700 were consumed and 182 of 7,219 bodies were STILL
                   # MOVING when it baked — frozen mid-flight, which is the
                   # one failure `bake_result` can produce and the wildfire
                   # skill warns about. A thrown fragment has a trajectory as
                   # well as a fall, and at 9 m/s into 0.16 damping it is
                   # airborne for a long time. `run` treats `steps` as a
                   # CEILING with an early exit once nothing is moving, so a
                   # generous budget costs nothing on the cells that settle
                   # quickly. Watch the "STILL MOVING" line: if it is not
                   # zero, this number is too low.
                   steps=1200, kick=0.15,
                   rng=random.Random(SEED), bake_result=True,
                   approx_map=approx_map,
                   bias=bias,
                   # BOTH CEILINGS MUST MOVE WITH THE BIAS or it is silently
                   # clamped: `maxLinearVelocity` defaults to 4 m/s and
                   # `linearDamping` to 0.55, which between them turn a 9 m/s
                   # throw into a 4 m/s one that has lost half its speed
                   # within a second. See `settle.prepare`.
                   max_speed=max(6.0, THROW_MPS * 2.2), damping=0.16)
    for _ in range(10):
        omni.kit.app.get_app().update()

    # ---- export every archetype, re-centred to the origin ------------------
    records, miss = [], 0

    def _safe_export(paths, out, rec, meta, drop=False):
        try:
            if bake.export_object(stage, None, paths, out, recenter=rec,
                                  drop_to_ground=drop):
                m, ok, ms = bake.validate(out)
                records.append(dict(usd=os.path.abspath(out), meshes=m,
                                    bound_missing=ms, **meta))
                return ms
        except Exception as exc:
            print("[tarch] export FAILED for {0}: {1}".format(
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
            (X, Y, 0.0), dict(kind="tree", species=sp, level=lv),
            drop=(lv not in _NO_DROP))
    man_path = os.path.join(OUT_DIR, "archetypes.json")
    merged = merge_manifest(man_path, records, KINDS)
    bake.write_manifest(man_path, merged)
    dt = time.time() - t_build0
    nh = sum(1 for r in merged if r["kind"] == "house")
    nt = sum(1 for r in merged if r["kind"] == "tree")
    sz = sum(os.path.getsize(r["usd"]) for r in records
             if os.path.exists(r["usd"])) / 1e6
    nm = sum(r["meshes"] for r in records)

    print("\n" + "=" * 72)
    print("TORNADO ARCHETYPE BAKE  (kinds: {0})".format(",".join(sorted(KINDS))))
    print("  manifest: {0} house + {1} tree archetypes -> {2}"
          .format(nh, nt, OUT_DIR))
    print("  this run: {0} archetype(s), {1} mesh(es), {2:.0f} MB, "
          "{3:.0f} s, {4} unresolved material(s)"
          .format(len(records), nm, sz, dt, miss))
    print("  throw     {0:.1f} m/s toward +X (assembly yaws each into the "
          "track)".format(THROW_MPS))
    print("=" * 72 + "\n")

    app = omni.kit.app.get_app()
    while simulation_app.is_running():
        app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
