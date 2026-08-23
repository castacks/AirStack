#!/usr/bin/env python
"""
Partial-damage bench — houses that lost their roof and some walls, not all.

    ISAAC_SIM_SCRIPT_NAME=house_partial_damage_test_launch_script.py \\
    airstack up isaac-sim

The heavy bench (`house_damage_test_launch_script.py`) is the worst case: a
burned-out shell with everything down. Most of a real burn scar is not that.
Fire moves through a street unevenly, so the common house has lost its roof —
which is what catches the embers — and a wall or two, while the rest of the
structure is still standing and recognisable.

FOUR HOUSES, INCREASING BUT RANDOMISED
--------------------------------------
    A  roof only
    B  roof + 1 wall
    C  roof + 2 walls, one of them partial
    D  roof + 3 walls, mostly partial

"Partial" means the wall failed HALFWAY: it is cut at a random height, the
lower part stays standing as a ragged stub, and only what was above comes
down. That silhouette — a standing stub with a broken top edge — is one of the
most recognisable things in a burnt-out building, and fracturing whole walls
loses it completely.

Which walls break is random per house, so two houses with the same count do
not come apart the same way.

Everything still standing is a static collider, so the fragments land ON the
remaining structure rather than through it.
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
from disaster import damage, fire, settle            # noqa: E402
from disaster import vtk_fracture as fracture

PARENT = "/World/stage/generated"
STYLE = os.environ.get("HOUSE_STYLE", "cottage")
SEED = int(os.environ.get("HOUSE_SEED", "7"))
SPACING_M = float(os.environ.get("HOUSE_SPACING", "32"))
SETTLE_STEPS = int(os.environ.get("SETTLE_STEPS", "360"))
KEEP_PHYSICS = os.environ.get("KEEP_PHYSICS", "0") not in ("0", "", "false")

# (label, walls to break, chance each of those is a partial failure)
HOUSES = [
    ("A roof only",        0, 0.0),
    ("B roof + 1 wall",    1, 0.0),
    ("C roof + 2, 1 part", 2, 0.5),
    ("D roof + 3, mostly", 3, 0.8),
]
SEEDS_PER_MODULE = 9
FINISH = "char"          # fresher damage than the burned-out shell
FIRE_STATE = "smoke"


def build_ground_and_light(stage):
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


def main():
    import numpy as np

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
    for i, (label, n_walls, partial_p) in enumerate(HOUSES):
        x = x0 + i * SPACING_M
        # NOTHING IS REMOVED and nothing is pre-felled. The house is built
        # whole and only the chosen modules are broken, so what survives is
        # genuinely the original structure rather than a reduced version.
        pl = modular_house.build_building(
            STYLE, x, 0.0, 0.0, random.Random(SEED), category="house")
        if pal:
            for q in pl:
                q["palette"] = pal
        per_house.append([label, n_walls, partial_p, x, pl, [], []])
        all_pl.extend(pl)

    _, ssf = get_stage_meters_per_unit(stage)
    sg.apply_placements(stage, all_pl, PARENT, ssf)
    print("[partial] palette: {0} subsets rebound".format(
        modular_house.apply_palette(stage, all_pl, PARENT)))

    mats = damage.char_materials(stage, PARENT)
    fracture.ensure_deps()

    keep_material = set()
    for hi, h in enumerate(per_house):
        label, n_walls, partial_p, x, pl, loose, statics = h
        rng = random.Random(SEED + 41 * hi)
        nrng = np.random.default_rng(SEED + 41 * hi)

        roofs = [q for q in pl
                 if damage._sub_of(q.get("category")) in ("roof", "bay_roof")]
        walls = [q for q in pl
                 if damage._sub_of(q.get("category")) == "wall"]
        picked = rng.sample(walls, min(n_walls, len(walls))) if n_walls else []

        n_part = 0
        for q in roofs + picked:
            path = q.get("prim_path")
            if not path:
                continue
            out = "{0}/brk_{1}_{2}".format(PARENT, hi, path.rsplit("/", 1)[-1])
            is_wall = q in picked
            # READ THE MATERIAL FIRST. Fracturing deactivates the source, and
            # a partially collapsed wall's fragments should carry the wall's
            # OWN texture heavily scorched — not a generic char map, which
            # makes them look like they came from a different building.
            src_tex = damage.bound_texture(stage, path)
            if is_wall and rng.random() < partial_p:
                st, lo = fracture.fracture_partial(
                    stage, path, out, n_pieces=SEEDS_PER_MODULE, rng=nrng,
                    cut_frac=rng.uniform(0.35, 0.62), mode="char")
                statics.extend(st)
                loose.extend(lo)
                if src_tex:
                    heavy = damage.scorched_material(
                        stage, PARENT, None, len(damage.SOOT_LEVELS) - 1,
                        texture=src_tex,
                        triplanar=True)
                    for pth in st + lo:
                        pr = stage.GetPrimAtPath(pth)
                        if pr and pr.IsValid():
                            UsdShade.MaterialBindingAPI(pr).Bind(heavy)
                            keep_material.add(pth)
                n_part += 1
            else:
                loose.extend(fracture.fracture_prim(
                    stage, path, out, n_pieces=SEEDS_PER_MODULE, rng=nrng,
                    mode="char", rough=0.045, verbose=False))
        print("[partial] {0:<20s} {1} roof + {2} wall(s), {3} partial "
              "-> {4:3d} fragments".format(label, len(roofs), len(picked),
                                           n_part, len(loose)))

    r = random.Random(SEED)
    for h in per_house:
        for path in h[5] + h[6]:
            if path in keep_material:
                continue        # partial-wall pieces already wear their own
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                UsdShade.MaterialBindingAPI(prim).Bind(
                    damage._pick(r, FINISH, mats))
    # SOOT, NOT CHAR, on everything still standing. `char_placements` swaps in
    # a burn texture, which turns an intact wall into a charcoal one and loses
    # the house's identity. Sooting duplicates whatever material the surface
    # already has and multiplies its albedo down, so cream siding stays cream
    # siding — just smoke-darkened.
    # Scorch tied to how badly each house burned, so the row shades from
    # lightly marked to fouled to the eaves rather than varying at random.
    n_soot = 0
    for hi, h in enumerate(per_house):
        base_cov = 0.70 + 0.10 * hi
        n_soot += damage.soot_materials(
            stage, h[4], PARENT, random.Random(SEED + hi),
            coverage_at=lambda _x, _y, c=base_cov: c)
    print("[partial] {0} surviving subsets sooted".format(n_soot))

    for _ in range(20):
        omni.kit.app.get_app().update()

    broken = {p for h in per_house for p in h[5]}
    loose_all = sorted(broken)
    static_all = [q["prim_path"] for q in all_pl
                  if q.get("prim_path")
                  and stage.GetPrimAtPath(q["prim_path"]).IsActive()]
    static_all += [p for h in per_house for p in h[6]]
    settle.run(stage, loose_all, static_all + ["/World/ground"],
               steps=SETTLE_STEPS, kick=0.15, rng=random.Random(SEED),
               bake_result=not KEEP_PHYSICS)

    fire.setup_flow_stack(stage, density_cell_size_m=0.1, max_blocks=16384,
                          scene_scale_factor=1.0)
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    n_em = 0
    for i, h in enumerate(per_house):
        # h[4] is the HOUSE's placements. Passing h[5] (the loose fragments)
        # measured the debris field instead — a cloud centred wherever rubble
        # landed and far wider than the building, which is why the plumes sat
        # outside the walls no matter how the radius was tightened.
        n_em += fire.add_structure_fire(
            stage, h[4], FIRE_STATE, fire.FLOW_ROOT, "p{0}".format(i),
            random.Random(SEED + 100 + i), max_emitters=4)
    print("[partial] {0} structure emitters".format(n_em))

    cam = UsdGeom.Camera.Define(stage, Sdf.Path("/World/benchCam"))
    cam.AddTranslateOp().Set(Gf.Vec3d(0.0, -58.0, 20.0))
    cam.AddRotateXYZOp().Set(Gf.Vec3f(74.0, 0.0, 0.0))
    try:
        import omni.kit.viewport.utility as vp
        vp.get_active_viewport().camera_path = "/World/benchCam"
    except Exception as exc:
        carb.log_warn("viewport retarget failed: {0}".format(exc))

    for _ in range(20):
        omni.kit.app.get_app().update()

    print("\n" + "=" * 72)
    print("PARTIAL DAMAGE BENCH — roof down, structure mostly standing")
    for label, n_walls, partial_p, x, pl, loose, statics in per_house:
        print("  x={0:+7.1f}  {1}".format(x, label))
    print("=" * 72 + "\n")

    timeline.play()
    app = omni.kit.app.get_app()
    while simulation_app.is_running():
        app.update()
    timeline.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
