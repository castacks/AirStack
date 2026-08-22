#!/usr/bin/env python
"""
Collapsed-house bench — three burnt-out, smoking houses, on empty ground.

    ISAAC_SIM_SCRIPT_NAME=house_damage_test_launch_script.py airstack up isaac-sim

THREE OF THE SAME LEVEL, COLLAPSED DIFFERENTLY. All three are `burned_out`
with an ash finish and residual smoke; what changes is how they came apart:

    A  x=-30   roof falls whole, walls stay put
    B  x=  0   roof breaks into panels
    C  x=+30   roof breaks AND the long walls split

Same house, same style, same palette — so the row shows how much variety the
break-up rules alone produce, which is the thing worth judging before any of
it goes near a street.

HOW A COLLAPSE IS MADE
----------------------
1. `damage.damage_placements` decides what falls
2. `damage.break_up` replaces some felled modules with several smaller
   window-less panels, because an intact roof lying at an angle reads as a
   placement error rather than as a collapse
3. `damage.scatter_debris` throws loose slabs and panels across the lot
4. `disaster.settle` gives every loose piece a rigid body, lets PhysX drop
   them onto the standing walls and the ground, and BAKES the result back to
   static transforms — so the pile is arranged by gravity but costs nothing
   at capture time

No mesh is ever cut. There is no fracture in this build; the pieces are whole
kit modules, which is exactly why this works.
"""

import os
import random
import sys

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("omni.flowusd")
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

import scene_generator as sg                                   # noqa: E402
from scene_prep import get_stage_meters_per_unit               # noqa: E402
from detail import modular_house                               # noqa: E402
from disaster import damage, fire, fracture, settle            # noqa: E402

PARENT = "/World/stage/generated"
STYLE = os.environ.get("HOUSE_STYLE", "cottage")
SEED = int(os.environ.get("HOUSE_SEED", "7"))
SPACING_M = float(os.environ.get("HOUSE_SPACING", "34"))
SETTLE_STEPS = int(os.environ.get("SETTLE_STEPS", "360"))
# KEEP_PHYSICS=1 leaves the rigid bodies live instead of baking them, so
# you can drag a piece into the air in the viewport, press play, and watch
# it fall. Baked is the default because capture wants static geometry.
KEEP_PHYSICS = os.environ.get("KEEP_PHYSICS", "0") not in ("0", "", "false")

# FOUR WAYS OF BREAKING THE SAME HOUSE. Everything else is held constant —
# same style, same seed, same materials — so the row is a controlled
# comparison of how the geometry comes apart and nothing else.
#
# (label, how, voronoi seeds per module, seed mode)
HOUSES = [
    ("2 uniform",     "fracture", 10, "uniform"),
    ("3 char grid",   "fracture", 12, "char"),
    ("4 splinters",   "fracture", 10, "splinter"),
]

# The three texture variants that survived the last bench, on boards behind.
SWATCHES = [
    ("Char_Ref (photo)",  "Burn_Char_Ref.png"),
    ("Ash_Over_Char",     "Burn_Ash_Over_Char.png"),
    ("Scorch",            "Burn_Scorch.png"),
]

LEVEL = "burned_out"
FINISH = "ash"
FIRE_STATE = "smoulder"


def build_ground_and_light(stage):
    """Neutral ground and daylight — the opposite of the fire bench.

    That one is deliberately gloomy because flame is emissive. Here the subject
    is MATERIAL, and char against ash needs even, honest light or the whole
    comparison is worthless.
    """
    ground = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/ground"))
    e = 260.0
    ground.CreatePointsAttr([Gf.Vec3f(-e, -e, 0.0), Gf.Vec3f(e, -e, 0.0),
                             Gf.Vec3f(e, e, 0.0), Gf.Vec3f(-e, e, 0.0)])
    ground.CreateFaceVertexCountsAttr([4])
    ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    ground.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    ground.CreateDisplayColorAttr([Gf.Vec3f(0.28, 0.29, 0.27)])
    ground.CreateExtentAttr([Gf.Vec3f(-e, -e, 0.0), Gf.Vec3f(e, e, 0.0)])

    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/domeLight"))
    dome.CreateIntensityAttr(900.0)
    dome.CreateColorAttr(Gf.Vec3f(0.72, 0.76, 0.85))

    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/keyLight"))
    key.CreateIntensityAttr(2600.0)
    key.CreateAngleAttr(0.8)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-42.0, 0.0, 28.0))


def add_swatch_boards(stage, mats_by_png):
    """A row of upright boards behind the houses, one per texture variant.

    Flat, evenly lit, all the same size — so the only thing separating them is
    the material. Judging a texture off a collapsed pile is hopeless; judging
    it off a board is not.
    """
    from pxr import UsdShade

    y = 26.0
    w, h = 7.0, 5.0
    x0 = -0.5 * (len(SWATCHES) - 1) * (w + 2.0)
    for i, (label, png) in enumerate(SWATCHES):
        x = x0 + i * (w + 2.0)
        path = "/World/swatches/board_{0}".format(i)
        mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
        mesh.CreatePointsAttr([
            Gf.Vec3f(x - w / 2, y, 0.4), Gf.Vec3f(x + w / 2, y, 0.4),
            Gf.Vec3f(x + w / 2, y, 0.4 + h), Gf.Vec3f(x - w / 2, y, 0.4 + h)])
        mesh.CreateFaceVertexCountsAttr([4])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        mesh.CreateNormalsAttr([Gf.Vec3f(0, -1, 0)] * 4)
        # One tile across the board, so what you see is the map itself rather
        # than a tiling of it.
        mesh.CreateExtentAttr([Gf.Vec3f(x - w / 2, y, 0.4),
                               Gf.Vec3f(x + w / 2, y, 0.4 + h)])
        UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray,
            UsdGeom.Tokens.varying).Set(
            [Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
        m = mats_by_png.get(png)
        if m is not None:
            UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(m)
        print("[house_damage]   board {0}: {1}".format(i, label))


def main():
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()

    usd_ctx = omni.usd.get_context()
    usd_ctx.new_stage()
    stage = usd_ctx.get_stage()
    if stage is None:
        raise RuntimeError("Failed to create a new stage")
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/stage"))

    build_ground_and_light(stage)

    pal = modular_house.STYLES[STYLE].get("palette")
    x0 = -0.5 * SPACING_M * (len(HOUSES) - 1)
    all_pl, per_house = [], []
    for i, (label, how, n_seeds, mode) in enumerate(HOUSES):
        x = x0 + i * SPACING_M
        rng = random.Random(SEED + 31 * i)
        pristine = modular_house.build_building(
            STYLE, x, 0.0, 0.0, random.Random(SEED), category="house")
        if pal:
            for q in pristine:
                q["palette"] = pal

        # move_felled False on the fracture houses: the modules stay UPRIGHT
        # and physics collapses the fragments. Felling them first put every
        # piece flat on the ground before the solver ran, so nothing fell and
        # the wall read as intact-but-cut.
        dmg = damage.damage_placements(pristine, LEVEL, rng,
                                       move_felled=(how == "kit"))
        if how == "kit":
            dmg = damage.break_up(dmg, rng, roof_chance=0.65, wall_chance=0.45)
            dmg = dmg + damage.scatter_debris(dmg, rng)
        per_house.append([label, how, n_seeds, mode, x, dmg, []])
        all_pl.extend(dmg)
        print("[house_damage] {0:<18s} {1:<9s} {2:3d} placed pieces".format(
            label, how, len(dmg)))

    _, ssf = get_stage_meters_per_unit(stage)
    sg.apply_placements(stage, all_pl, PARENT, ssf)
    n_pal = modular_house.apply_palette(stage, all_pl, PARENT)
    print("[house_damage] palette: {0} subsets rebound".format(n_pal))

    mats = damage.char_materials(stage, PARENT)
    n_mat = 0
    for label, how, n_seeds, mode, x, dmg, frags in per_house:
        n_mat += damage.char_placements(stage, dmg, LEVEL, mats,
                                        random.Random(SEED), finish=FINISH)
    print("[house_damage] {0} subsets re-materialised".format(n_mat))

    # FRACTURE — cut the felled modules into real Voronoi fragments. This is
    # the actual house geometry being split, so a fragment carries the wall's
    # thickness, its corner, its window reveal. Needs manifold3d + shapely.
    import numpy as np
    fracture.ensure_deps()
    n_frag_total = 0
    for hi, h in enumerate(per_house):
        label, how, n_seeds, mode, x, dmg, frags = h
        if how != "fracture":
            continue
        nrng = np.random.default_rng(SEED + 7 * hi)
        for q in dmg:
            if not q.get("felled") or not q.get("prim_path"):
                continue
            paths = fracture.fracture_prim(
                stage, q["prim_path"],
                "{0}/frag_{1}_{2}".format(PARENT, hi,
                                          q["prim_path"].rsplit("/", 1)[-1]),
                n_pieces=n_seeds, rng=nrng, mode=mode, rough=0.045)
            frags.extend(paths)
        n_frag_total += len(frags)
        print("[house_damage] {0:<18s} -> {1:3d} fragments".format(
            label, len(frags)))
    print("[house_damage] {0} fragments total".format(n_frag_total))

    r = random.Random(SEED)
    for h in per_house:
        for path in h[6]:
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                UsdShade.MaterialBindingAPI(prim).Bind(
                    damage._pick(r, FINISH, mats))

    # Texture variants, on flat boards, one tile each.
    swatch_mats = {}
    for label, png in SWATCHES:
        swatch_mats[png] = damage._pbr(
            stage, "/World/swatches/Looks/{0}".format(png.replace(".png", "")),
            (1.0, 1.0, 1.0), 0.9,
            os.path.join(damage._TEX_DIR, png), scale_uv=(1.0, 1.0))
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/swatches"))
    add_swatch_boards(stage, swatch_mats)

    for _ in range(20):
        omni.kit.app.get_app().update()

    fragged = {q["prim_path"] for h in per_house for q in h[5]
               if q.get("felled") and q.get("prim_path") and h[1] == "fracture"}
    loose = [q["prim_path"] for q in all_pl
             if q.get("felled") and q.get("prim_path")
             and q["prim_path"] not in fragged]
    loose += [p for h in per_house for p in h[6]]
    standing = [q["prim_path"] for q in all_pl
                if not q.get("felled") and q.get("prim_path")]
    settle.run(stage, loose, standing + ["/World/ground"],
               steps=SETTLE_STEPS, kick=0.15, rng=random.Random(SEED),
               bake_result=not KEEP_PHYSICS)
    if KEEP_PHYSICS:
        print("[house_damage] KEEP_PHYSICS=1 — bodies left live; press play")

    fire.setup_flow_stack(stage, density_cell_size_m=0.1, max_blocks=16384,
                          scene_scale_factor=1.0)
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    n_em = 0
    for i, (label, how, n_seeds, mode, x, dmg, frags) in enumerate(per_house):
        settled = []
        for q in dmg:
            prim = stage.GetPrimAtPath(q.get("prim_path") or "")
            if not prim or not prim.IsValid():
                continue
            r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
            if r.IsEmpty():
                continue
            c = r.GetMidpoint()
            settled.append({"x_m": c[0], "y_m": c[1], "z_m": r.GetMin()[2],
                            "category": q.get("category", "house_wall")})
        n_em += fire.add_structure_fire(
            stage, settled, FIRE_STATE, fire.FLOW_ROOT, "h{0}".format(i),
            random.Random(SEED + 100 + i), max_emitters=5)
    print("[house_damage] {0} structure emitters".format(n_em))

    cam = UsdGeom.Camera.Define(stage, Sdf.Path("/World/benchCam"))
    cam.AddTranslateOp().Set(Gf.Vec3d(0.0, -62.0, 21.0))
    cam.AddRotateXYZOp().Set(Gf.Vec3f(74.0, 0.0, 0.0))
    try:
        import omni.kit.viewport.utility as vp
        vp.get_active_viewport().camera_path = "/World/benchCam"
    except Exception as exc:
        carb.log_warn("could not retarget the viewport: {0}".format(exc))

    for _ in range(20):
        omni.kit.app.get_app().update()

    print("\n" + "=" * 72)
    print("DEBRIS + TEXTURE VARIETY BENCH")
    print("  front row  — same material, four ways of making debris:")
    for label, how, n_seeds, mode, x, dmg, frags in per_house:
        print("    x={0:+7.1f}  {1}".format(x, label))
    print("  back row   — same surface, six texture variants (left to right):")
    for label, _png in SWATCHES:
        print("    {0}".format(label))
    print("=" * 72 + "\n")

    timeline.play()
    app = omni.kit.app.get_app()
    while simulation_app.is_running():
        app.update()
    timeline.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
