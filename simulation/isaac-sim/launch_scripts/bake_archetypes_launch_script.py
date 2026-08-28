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

Env knobs:

    ARCH_DIR    output directory (default `scene_gen/assets/archetypes`)
    ARCH_SEED   rng seed (default 7)
    ARCH_KINDS  which halves to rebuild — `house,tree` (default), `tree`,
                `house`. Houses are most of the ~570 s wall time and their
                geometry changes far less often than the vegetation passes do,
                so `ARCH_KINDS=tree` turns the iteration loop from ten minutes
                into two. The manifest is MERGED on a restricted run (see
                `merge_manifest`) so the untouched half survives it.
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


def _env(name, default):
    """`os.environ.get` with EMPTY TREATED AS ABSENT, which it is not.

    THE COMPOSE FILE EXPORTS THESE EMPTY. `docker exec isaac-sim env` shows
    `ARCH_SEED=`, `ARCH_DIR=`, `SETTLE_STEPS=`, `ARCH_STYLES=` — declared,
    unset, and therefore present as `""`, so `os.environ.get(name, default)`
    never reaches its default. `int("")` raises fourteen seconds into a launch
    (reads like a code bug, is an env one), and `ARCH_DIR` does NOT raise: it
    becomes `""`, `os.makedirs("")` is a no-op, and the library is written into
    the container's CWD under a banner that says it worked.
    """
    v = os.environ.get(name)
    return default if v is None or not v.strip() else v.strip()


def _flag(name, default="0"):
    return _env(name, default) not in ("", "0", "false", "False")


SEED = int(_env("ARCH_SEED", "7"))
OUT_DIR = _env(
    "ARCH_DIR", os.path.join(_SCENE_GEN_DIR, "assets", "archetypes"))
GRID = 40.0
KINDS = set(k.strip().lower()
            for k in _env("ARCH_KINDS", "house,tree").split(",")
            if k.strip())

# THE SETTLE KNOBS. This bake ran `steps=420` with no convergence test, no
# quiet phase, no ground plane and no CCD — the same configuration that left
# the TORNADO library with 11-29% of every pile's meshes in mid-air and 21-42%
# of them below the world. Nothing about a fire makes its fragments land any
# better than a wind's; the burnt path simply never measured it.
SETTLE_STEPS = int(_env("SETTLE_STEPS", "1200"))
SETTLE_MAX_STEPS = int(_env("SETTLE_MAX_STEPS", "6000"))
SETTLE_QUIET_STEPS = int(_env("SETTLE_QUIET_STEPS", "900"))
# DECOMPOSE THE BIG PIECES, HULL THE REST. This path used to send EVERY tree
# body to `convexDecomposition`, which is the documented 20-minute settle: a
# debris stick is convex to within its own bark and cooking a decomposition
# for thousands of them dominates start-up. What actually needs it is the
# toppled bole — a branching trunk hulls to a 20 m blob and comes to rest
# balanced on its own limb tips — and a size threshold picks exactly those.
SETTLE_DECOMP_M = float(_env("SETTLE_DECOMP_M", "2.5"))
BAKE_STRICT = _flag("BAKE_STRICT")
ARCH_AUDIT = _flag("ARCH_AUDIT", "1")
# See `.agents/skills/bake-tree-and-debris`. A settle does NOT reproduce, so
# the pile somebody approved is the only copy of it: `KEEP_PHYSICS=1` holds
# the stage with gravity and every rigid body still on, writes nothing, and
# bakes exactly what is on screen when the trigger file appears.
KEEP_PHYSICS = _flag("KEEP_PHYSICS")
SIM_ALL_DEBRIS = _flag("SIM_ALL_DEBRIS")
BAKE_TRIGGER = _env("BAKE_TRIGGER", "/isaac-sim/.bake_now")
_PILE_LEVELS = bake.PILE_LEVELS

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


def merge_manifest(path, records, kinds):
    """The new records, plus whatever the manifest already had for OTHER kinds.

    A RESTRICTED RUN MUST NOT DELETE THE HALF IT DID NOT BUILD. `ARCH_KINDS`
    exists so a vegetation change can be re-baked without paying for 48 houses
    that did not change — but the USD files for those houses stay on disk, so
    a manifest rewritten from this run's records alone would claim they are
    gone. `suburb_reload_launch_script` drives entirely off the manifest and
    would then load a plat with no buildings in it, and the failure would look
    like a bake bug rather than a bookkeeping one.

    Identity is `(kind, style-or-species, level)`, which is exactly what the
    filename encodes, so a record from this run replaces the record for the
    same archetype and nothing else. Kinds we rebuilt are dropped wholesale
    first, so an archetype REMOVED from the tables (a species retired from
    `TREE_SPECIES`, say) does not linger in the manifest for ever.
    """
    old = []
    if os.path.exists(path):
        try:
            old = bake.read_manifest(path)
        except Exception as exc:
            print("[arch] existing manifest unreadable, replacing it: "
                  "{0}".format(exc))
            old = []

    def key(r):
        return (r.get("kind"), r.get("style") or r.get("species"),
                r.get("level"))

    fresh = set(key(r) for r in records)
    kept = [r for r in old
            if r.get("kind") not in kinds and key(r) not in fresh]
    if kept:
        print("[arch] manifest: keeping {0} record(s) for kind(s) not built "
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
    fracture.ensure_deps()
    os.makedirs(OUT_DIR, exist_ok=True)
    t_build0 = time.time()

    # ---- HOUSES: build every (style, level) on a grid ----------------------
    # The grid geometry is computed whether or not houses are built this run,
    # because the TREE rows are placed below the house block and their Y must
    # not move between a full bake and an `ARCH_KINDS=tree` one — otherwise a
    # partial re-bake would sit the trees on top of a house grid that is not
    # there this time but is there in the files already on disk.
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
    tree_loose = []           # tree debris -> convexDecomposition (no float)
    tcombos = [(sp, lv) for sp in TREE_SPECIES for lv in TREE_LEVELS]
    tcol = max(1, int(math.ceil(math.sqrt(len(tcombos)))))
    y0 = (len(hcombos) // ncol + 2) * GRID    # trees below the house grid
    for idx, (sp, lv) in enumerate(tcombos if "tree" in KINDS else []):
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
        # `seated_collide=False`: the small ground sticks are geometry, not
        # collision. Cooking a triangle mesh for each of several hundred per
        # tree so that a couple of loose logs can land on them is most of the
        # static-collider count and buys nothing you can see — see `burn_tree`.
        res = veg.burn_tree(stage, tp, lv, PARENT, PARENT + "/tdeb_%s_%s" % (sp, lv),
                            rng, debris_scale=0.375, ground_z=0.0,
                            simulate_above_m=(0.0 if SIM_ALL_DEBRIS else None),
                            seated_collide=False, verbose=False)
        # `seated` listed explicitly now that it is no longer inside `statics`.
        # `info["made"]` already covers it, but relying on that would make the
        # export depend on a bookkeeping side effect of another module.
        # `bake.export_object` dedupes, so the overlap costs nothing.
        extra = list(res.get("statics", [])) + list(res.get("loose", [])) \
            + list(res.get("seated", [])) \
            + list((res.get("info") or {}).get("made", []))
        all_loose.extend(res.get("loose", []))
        all_static.extend(res.get("statics", []))
        tree_loose.extend(res.get("loose", []))
        tree_specs.append([sp, lv, X, Y, tp, extra,
                           list(res.get("anchored") or [tp])])
        # WHAT ACTUALLY GOT PHYSICS, per cell. `seated` pieces are laid flat
        # by arithmetic and never simulated, so they look settled whatever the
        # solver did; only `loose` is evidence about the settle.
        print("[arch]   {0:<18} {1:<9} bodies {2:>3} loose / {3:>3} seated "
              "(no physics) / {4} static".format(
                  sp, lv, len(res.get("loose") or []),
                  len(res.get("seated") or []),
                  len(res.get("statics") or [])))
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
    sinfo = {}
    if all_loose:
        # NO BLANKET `approx_map`. `SETTLE_DECOMP_M` picks the pieces whose
        # hull is a lie — the toppled boles — by size, and leaves a debris
        # stick as the hull it already is. See the knob's comment above.
        print("[arch] settling {0} bodies".format(len(all_loose)))
        sinfo = settle.run(
            stage, all_loose, all_static, kick=0.15,
            rng=random.Random(SEED), bake_result=not KEEP_PHYSICS,
            # `steps` IS A TARGET, NOT A CEILING. The throw phase runs on to
            # `max_steps` while the count of moving bodies is still falling,
            # and gives up early and loudly when it is not.
            steps=SETTLE_STEPS, converge=True, max_steps=SETTLE_MAX_STEPS,
            # THE QUIET PHASE is what lands whatever is still in the air when
            # the budget expires: damping raised, speed capped, no new
            # impulse, so nothing that has already come to rest is disturbed.
            quiet_steps=SETTLE_QUIET_STEPS,
            # A REAL PHYSX HALF-SPACE plus CCD on every body. The harness
            # ground is a four-vertex quad cooked as a triangle mesh —
            # infinitely thin — and a falling fragment can be wholly through
            # it by the end of a step and never generate a contact. A
            # half-space cannot be passed through: being under it is
            # penetration, not escape.
            ground_plane_z=0.0, ccd=True,
            # And the belt to that brace: anything still under grade is
            # lifted back onto it before the quiet phase and again after.
            floor_z=0.0,
            decompose_larger_than=(SETTLE_DECOMP_M or None))
        if sinfo.get("faults"):
            print("[arch] " + "!" * 68)
            print("[arch] !! THE SETTLE DID NOT FINISH CLEANLY — these "
                  "archetypes will contain")
            print("[arch] !! debris frozen in mid-air and/or below grade:")
            for f in sinfo["faults"]:
                print("[arch] !!   - " + f)
            print("[arch] !! Knobs: SETTLE_STEPS / SETTLE_MAX_STEPS / "
                  "SETTLE_QUIET_STEPS / SETTLE_DECOMP_M")
            print("[arch] " + "!" * 68)
            if BAKE_STRICT:
                raise SystemExit(
                    "[arch] BAKE_STRICT=1 and the settle did not converge; "
                    "refusing to export.")
        else:
            print("[arch] settle AT REST: nothing moving, nothing below "
                  "grade at bake time")
    for _ in range(10):
        omni.kit.app.get_app().update()

    # ---- KEEP_PHYSICS: pause here, then bake IN PLACE when told to --------
    #
    # A SETTLE DOES NOT REPRODUCE. Two back-to-back runs of the tornado bake
    # with the same seed, the same knobs and the same code finished 0 bodies
    # still moving / 0 clamped, and 1 still moving / 595 clamped. So
    # "re-run it with the same seed to export it" throws away the pile
    # somebody approved and ships a different one — measured, once, the
    # expensive way. This mode holds the stage instead: physics on, gravity
    # on, timeline paused, NOTHING written, until a trigger file appears.
    # See `.agents/skills/bake-tree-and-debris`.
    if KEEP_PHYSICS:
        tl = omni.timeline.get_timeline_interface()
        try:
            tl.pause()
        except Exception as exc:
            print("[arch] could not pause the timeline: {0}".format(exc))
        for _ in range(10):
            omni.kit.app.get_app().update()
        print("\n" + "=" * 72)
        print("WILDFIRE ARCHETYPE GRID — PHYSICS LEFT ON, NOTHING WRITTEN")
        print("  {0} rigid bod(ies), {1} static collider(s), gravity on"
              .format(len(all_loose), len(all_static)))
        print("  every debris piece is a rigid body"
              if SIM_ALL_DEBRIS else
              "  only pieces over 0.8 m are rigid bodies "
              "(set SIM_ALL_DEBRIS=1 for all of them)")
        print("  PAUSED. Play settles it further; STOP rewinds every body to "
              "its authored pose")
        print("  (which is the debris back in the air — that is the rewind, "
              "not a regression).")
        print("  trees are on a {0:.0f} m grid from y={1:.0f}.".format(
            GRID, y0))
        print("  BAKE IN PLACE when it looks right:")
        print("    touch {0}".format(BAKE_TRIGGER))
        print("=" * 72 + "\n")
        app = omni.kit.app.get_app()
        while simulation_app.is_running():
            app.update()
            if not os.path.exists(BAKE_TRIGGER):
                continue
            try:
                os.remove(BAKE_TRIGGER)
            except OSError:
                pass
            print("[arch] BAKE IN PLACE requested — freezing {0} body(s) "
                  "where they stand".format(len(all_loose)))
            tl.pause()
            for _ in range(5):
                app.update()
            # FREEZE, DO NOT RE-SIMULATE. `settle.bake` reads each body's
            # current local-to-world and re-authors it; nothing steps between
            # the look and the write.
            try:
                n_frozen = settle.bake(stage, [
                    stage.GetPrimAtPath(q) for q in all_loose
                    if stage.GetPrimAtPath(q)
                    and stage.GetPrimAtPath(q).IsValid()])
                print("[arch] froze {0} body(s)".format(n_frozen))
            except Exception as exc:
                print("[arch] freeze FAILED: {0}".format(exc))
                continue
            for _ in range(10):
                app.update()
            break
        else:
            simulation_app.close()
            return
        if not simulation_app.is_running():
            simulation_app.close()
            return

    # ---- export every archetype, re-centred to the origin ------------------
    records, miss, audits = [], 0, []

    def _safe_export(paths, out, rec, meta, drop=False, reseat=False,
                     reseat_first=True, reseat_freeze=None):
        est = {}
        try:
            if bake.export_object(stage, None, paths, out, recenter=rec,
                                  drop_to_ground=drop, reseat=reseat,
                                  reseat_first=reseat_first,
                                  reseat_freeze=reseat_freeze,
                                  stats_out=est):
                m, ok, ms = bake.validate(out)
                records.append(dict(usd=os.path.abspath(out), meshes=m,
                                    bound_missing=ms, **meta))
                # The export's Z decisions, which are otherwise invisible.
                # `datum` is the drop's reference — the first root's own world
                # min-z — and a large one either way is the number to look at
                # first when debris sits off the ground.
                if abs(est.get("seat_rz", 0.0)) > 0.05 or est.get(
                        "seat_lifted"):
                    print("[arch]   seat  {0:<32} datum {1:+.3f} m, {2} "
                          "root(s) moved, worst {3:.3f} m".format(
                              os.path.basename(out)[:32],
                              est.get("seat_rz", 0.0),
                              est.get("seat_lifted", 0),
                              est.get("seat_lift_max", 0.0)))
                if ARCH_AUDIT:
                    try:
                        au = bake.audit_archetype(out)
                    except Exception as exc:
                        au = None
                        print("[arch] audit failed for {0}: {1}".format(
                            os.path.basename(out), exc))
                    if au:
                        au["name"] = os.path.splitext(
                            os.path.basename(out))[0]
                        au["level"] = meta.get("level")
                        audits.append(au)
                return ms
        except Exception as exc:
            print("[arch] export FAILED for {0}: {1}".format(
                os.path.basename(out), exc))
        return 0

    for st, lv, X, Y, parent, pls, frags in house_specs:
        miss += _safe_export(
            [q["prim_path"] for q in pls] + frags,
            os.path.join(OUT_DIR, "house_{0}_{1}.usd".format(st, lv)),
            (X, Y, 0.0), dict(kind="house", style=st, level=lv), reseat=True)
    for sp, lv, X, Y, tp, extra, anchored in tree_specs:
        # SEAT THE DEBRIS, FREEZE THE TREE. `drop_to_ground` places the tree;
        # `reseat` is the pure-geometry pass that puts a fragment the solver
        # lost through the floor back on the pile and drops one it froze in
        # mid-air onto whatever is under it. `reseat_first=False` keeps it off
        # the tree itself, whose pose is authored rather than settled.
        miss += _safe_export(
            [tp] + extra,
            os.path.join(OUT_DIR, "tree_{0}_{1}.usd".format(sp, lv)),
            (X, Y, 0.0), dict(kind="tree", species=sp, level=lv),
            drop=True, reseat=True, reseat_first=False,
            reseat_freeze=anchored)
    man_path = os.path.join(OUT_DIR, "archetypes.json")
    merged = merge_manifest(man_path, records, KINDS)
    bake.write_manifest(man_path, merged)
    dt = time.time() - t_build0
    nh = sum(1 for r in merged if r["kind"] == "house")
    nt = sum(1 for r in merged if r["kind"] == "tree")
    sz = sum(os.path.getsize(r["usd"]) for r in records
             if os.path.exists(r["usd"])) / 1e6
    nm = sum(r["meshes"] for r in records)

    if audits:
        print("\n[arch] FLOATING-DEBRIS AUDIT of the files just written "
              "(lower bounds — see bake.audit_archetype)")
        print("[arch]   {0:<34}{1:>7}{2:>16}{3:>15}{4:>8}".format(
            "archetype", "meshes", "airborne", "sunk", "z_min"))
        for au in sorted(audits, key=lambda q: -q["airborne"] / max(
                1, q["meshes"])):
            m = max(1, au["meshes"])
            print("[arch]   {0:<34}{1:>7}{2:>9} {3:5.1f}%{4:>8} {5:5.1f}%"
                  "{6:>8.2f}{7}".format(
                      au["name"][:34], au["meshes"], au["airborne"],
                      100.0 * au["airborne"] / m, au["sunk"],
                      100.0 * au["sunk"] / m, au["z_min"],
                      "" if au.get("level") in _PILE_LEVELS
                      else "   (has legitimate air: not a pile)"))
        pile = [au for au in audits if au.get("level") in _PILE_LEVELS]
        pm = sum(au["meshes"] for au in pile)
        if pm:
            pa = sum(au["airborne"] for au in pile)
            ps = sum(au["sunk"] for au in pile)
            print("[arch]   PILE LEVELS ONLY ({0}): {1} of {2} meshes with "
                  "nothing under them ({3:.1f}%), {4} below grade ({5:.1f}%)"
                  .format("/".join(_PILE_LEVELS), pa, pm, 100.0 * pa / pm,
                          ps, 100.0 * ps / pm))

    print("\n" + "=" * 72)
    print("ARCHETYPE BAKE  (kinds: {0})".format(",".join(sorted(KINDS))))
    print("  manifest: {0} house + {1} tree archetypes -> {2}"
          .format(nh, nt, OUT_DIR))
    print("  this run: {0} archetype(s), {1} mesh(es), {2:.0f} MB, "
          "{3:.0f} s, {4} unresolved material(s)"
          .format(len(records), nm, sz, dt, miss))
    print("=" * 72 + "\n")

    app = omni.kit.app.get_app()
    while simulation_app.is_running():
        app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
