#!/usr/bin/env python
"""
The tornado CASUALTY bench — every attitude against every occlusion pattern on
one grid, and one wrecked house with the real planner running over it.

    ARCH_DIR=/isaac-sim/AirStack/scene_gen/assets/archetypes_tornado \
    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/people_bench \
    ISAAC_SIM_SCRIPT_NAME=tornado_people_preview_launch_script.py \
    ./airstack.sh up isaac-sim

WHY A BENCH AND NOT THE FULL SCENE
----------------------------------
Every figure this module places is a claim about what a drone can SEE, and the
whole of `tornado_people` now turns on two axes that only a photograph can
settle:

    ATTITUDE   face-up, face-down, on the left side, on the right side, drawn
               up, pinned sitting. Six silhouettes, five of them added on
               2026-08-27 and NONE of them ever rendered — the lateral poses in
               particular are the first thing in this repo to use `pitch_deg`
               as a spin about the body's own long axis, and if the sign is
               wrong the figure is face-down with a knee through the ground.
    OCCLUSION  thirteen named patterns, each covering a NAMED stretch of the
               body. `legs` has to mean the legs. `head_only` has to leave a
               body with no head and everything else. `all_but_head` has to
               leave a head.

In the assembled 100 m scene any one of those is one figure of twenty,
somewhere in a plank field, at whatever altitude the overview camera happened
to be at. Here each is the subject of its own photograph.

THE TWO UNITS

    GRID    the catalogue. 8 attitudes x 13 occlusion patterns on a 3.4 x 4.6 m
            lattice over flat mud, every body laid along +X so a row reads left
            to right and a column compares like with like. Photographed one row
            at a time in two halves, close enough that a hand is a hand.

    HOUSE   one wrecked house, a real `planks` field scattered off it, and the
            REAL `plan_people` running over that field with `plank_specs` in
            the context — so what is photographed is the code the 100 m
            assembly runs, including `_Deck`'s measured debris surface and the
            tilt refusal. Every casualty it places gets its own close top-down
            and oblique.

    UNITS=GRID or UNITS=HOUSE narrows it.

WHAT TO LOOK AT, in order:

    1. Is a side-lying figure ON ITS SIDE — one shoulder up, the other in the
       debris — rather than face-down with a floating hip? (`_lying_lift`'s
       0.115 H is MODELLED, and it is the number to correct if not.)
    2. Does a covered body still show exactly the parts the record names in
       `visible_parts`?
    3. Is anything effectively invisible? `max_covered_frac` says nothing may
       be, and a body that is has to be caught here.
    4. In HOUSE: does each body lie ON the boards, not through them and not
       hovering over them?

Env knobs:

    ARCH_DIR     archetype library (default `scene_gen/assets/archetypes_tornado`)
    PEOPLE_SEED  rng seed (default 5)
    SNAP_DIR     viewport PNGs; MUST be under the mounted log directory
    UNITS        GRID,HOUSE (default both)
    WRECK_LEVEL  the HOUSE archetype's damage level (default `leveled`)
    WRECK_STYLE  the HOUSE archetype's style (default `ranch`)
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

import scene_generator as sg                                   # noqa: E402
from scene_prep import get_stage_meters_per_unit               # noqa: E402
import suburb_scene as ss                                      # noqa: E402
from suburb_scene import AssetPools, _raw_pool                 # noqa: E402
from compile_disaster import load_scene_config                 # noqa: E402
from disaster import planks                                    # noqa: E402
from disaster import tornado as tn                             # noqa: E402
from disaster import tornado_people as tpp                     # noqa: E402

PARENT = "/World/stage/generated"
SCENE_CONFIG = os.environ.get("SCENE_CONFIG", "suburb_tornado")
SEED = int(os.environ.get("PEOPLE_SEED", "5"))
ARCH_DIR = os.environ.get(
    "ARCH_DIR", os.path.join(_SCENE_GEN_DIR, "assets", "archetypes_tornado"))
SNAP_DIR = os.environ.get("SNAP_DIR", "")
UNITS = set(u.strip().upper() for u in
            os.environ.get("UNITS", "GRID,HOUSE").split(",") if u.strip())
WRECK_LEVEL = os.environ.get("WRECK_LEVEL", "leveled")
WRECK_STYLE = os.environ.get("WRECK_STYLE", "ranch")

# The catalogue lattice. 3.4 m between columns is a body length plus a metre —
# close enough that two cells fit in one frame, far enough that one body's
# propped board never lands in its neighbour's photograph. 4.6 m between rows
# because `lying_prone_reach` is 2.1 m of ground length.
GRID_STEP = (3.4, 4.6)
GRID_ORIGIN = (0.0, 0.0)
# The house unit sits well clear of the catalogue's north-east corner.
HOUSE_XY = (-56.0, 26.0)


def build_ground_and_light(stage, ssf):
    """Open ground, a dome and a key light. Mud, because that is what the
    corridor floor is and a figure's contrast against it is half of what this
    bench exists to judge."""
    plane = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/bench_ground"))
    plane.CreatePointsAttr([Gf.Vec3f(-140, -60, 0), Gf.Vec3f(120, -60, 0),
                            Gf.Vec3f(120, 90, 0), Gf.Vec3f(-140, 90, 0)])
    plane.CreateFaceVertexCountsAttr([4])
    plane.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    plane.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    plane.CreateDisplayColorAttr([Gf.Vec3f(0.30, 0.25, 0.17)])
    try:
        mat = tn_mud_material(stage)
        if mat:
            UsdShade.MaterialBindingAPI.Apply(plane.GetPrim()).Bind(mat)
    except Exception as exc:
        print("[bench] mud material unavailable: {0}".format(exc))
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/dome"))
    dome.CreateIntensityAttr(1000.0)
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/key"))
    key.CreateIntensityAttr(2400.0)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-48.0, 0.0, 35.0))
    del ssf


def tn_mud_material(stage):
    """The scour surface as a plain OPAQUE mud — no cutout, no opacity bands.

    THE FIRST BENCH RUN USED `planks.wood_material` FOR THE GROUND and it made
    every judgement on the sheet worthless: the floor and the debris were then
    the same sawn-timber map at different tile sizes, so a pale figure on a
    pale board on a pale floor had no contrast anywhere and "can you see this
    casualty" could not be answered. Contrast against the corridor floor is
    half of what this bench exists to judge, so the floor has to be the
    corridor floor.

    The assembled scene lays `Soil_Mud` as a TRANSLUCENT overlay in opacity
    bands, which needs the fractional-cutout Kit flag and a coverage field.
    Neither is wanted here: this is just the ground, bound opaque.
    """
    from pxr import Gf as _Gf, Sdf as _Sdf, UsdShade as _UsdShade
    path = PARENT + "/BenchLooks/mud"
    existing = _UsdShade.Material.Get(stage, path)
    if existing:
        return existing
    mat = _UsdShade.Material.Define(stage, _Sdf.Path(path))
    sh = _UsdShade.Shader.Define(stage, _Sdf.Path(path).AppendChild("Shader"))
    sh.CreateIdAttr("OmniPBR")
    sh.SetSourceAsset(_Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("diffuse_texture", _Sdf.ValueTypeNames.Asset).Set(
        _Sdf.AssetPath(sg._join_asset_root(tn.MUD_TEXTURE, "")))
    sh.CreateInput("diffuse_color_constant",
                   _Sdf.ValueTypeNames.Color3f).Set(_Gf.Vec3f(0.62, 0.58, 0.54))
    sh.CreateInput("project_uvw", _Sdf.ValueTypeNames.Bool).Set(True)
    sh.CreateInput("world_or_object", _Sdf.ValueTypeNames.Bool).Set(True)
    k = 1.0 / 2.6
    sh.CreateInput("texture_scale",
                   _Sdf.ValueTypeNames.Float2).Set(_Gf.Vec2f(k, k))
    sh.CreateInput("reflection_roughness_constant",
                   _Sdf.ValueTypeNames.Float).Set(0.95)
    sh.CreateInput("metallic_constant",
                   _Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    mat.CreateDisplacementOutput("mdl").ConnectToSource(sh.ConnectableAPI(),
                                                        "out")
    mat.CreateVolumeOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    return mat


def _ref(stage, dst, usd, x, y, yaw, ssf, scale=1.0):
    prim = stage.DefinePrim(Sdf.Path(dst), "Xform")
    if not prim.GetReferences().AddReference(usd):
        return False
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(x * ssf, y * ssf, 0.0))
    xf.AddRotateZOp().Set(float(yaw))
    if scale != 1.0:
        xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    return True


def _build_planks(stage, specs, root, ssf):
    if not specs:
        return 0
    planks.build(stage, root, specs, planks.materials(stage, PARENT), ssf,
                 verbose=False)
    return len(specs)


def _to_plank_spec(d, rng):
    """One `_cover` output as a `planks.build` spec.

    THE POSE, THE THICKNESS AND THE TILT COME FROM THE PLANNER. `_cover_piece`
    solves each piece against the body it is lying on — a propped one runs from
    the debris surface up over the casualty and its pitch is `-atan(rise/run)`
    — so randomising the tilt here is the difference between a piece resting on
    somebody and one that has fallen off them.
    """
    return {"x": d["x"], "y": d["y"], "z": d["z"],
            "l": d["len"], "w": d["wide"],
            "t": d.get("t", rng.uniform(0.02, 0.05)),
            "yaw": d["yaw"],
            "pitch": d.get("pitch", rng.uniform(-8.0, 8.0)),
            "roll": d.get("roll", rng.uniform(-10.0, 10.0)),
            "class": d.get("class",
                           "sheathing" if d["wide"] > 0.4 else "board")}


def main():
    t0 = time.time()
    omni.timeline.get_timeline_interface().stop()
    ctx_usd = omni.usd.get_context()
    ctx_usd.new_stage()
    stage = ctx_usd.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/stage"))
    UsdGeom.Scope.Define(stage, Sdf.Path(PARENT))
    _, ssf = get_stage_meters_per_unit(stage)
    build_ground_and_light(stage, ssf)

    config = load_scene_config(SCENE_CONFIG)
    resolver = sg._make_resolver(config)
    pools = AssetPools(config)
    raw_h = _raw_pool(config, "humans")
    rigged = pools.load_tagged(raw_h, "rigged")
    if not rigged:
        posed = pools.load_tagged(raw_h, "posed_standing")
        rigged = [u for u in pools.load(raw_h) if u not in posed]
    print("[bench] {0} rigged human asset(s)".format(len(rigged)))
    if not rigged:
        print("[bench] !! NO RIGGED HUMANS. Only rigged characters can take a "
              "pose; a posed_standing static mesh ships in whatever attitude "
              "it was authored in and this bench would photograph nothing.")

    arch = {os.path.splitext(f)[0]: os.path.join(ARCH_DIR, f)
            for f in (os.listdir(ARCH_DIR) if os.path.isdir(ARCH_DIR) else [])
            if f.endswith(".usd")}
    if not arch:
        print("[bench] !! ARCH_DIR {0} has no archetypes — the HOUSE unit will "
              "have no wreck in it and only the people will show"
              .format(ARCH_DIR))

    base_ctx = {"humans": rigged, "resolver": resolver, "asset_pools": pools,
                "throw_deg": 58.0}
    rng = random.Random(SEED)
    cfg = tpp.resolve_cfg({})
    subjects = {}
    # SUBJECTS THAT NEED ALTITUDE. `subjects` is shot at 13 m because its
    # subject is a body; a 52 m house plate at 13 m is a crop of one corner.
    wide = {}
    all_humans, all_cover, all_records = [], [], []

    # ---- GRID: the catalogue ----------------------------------------------
    if "GRID" in UNITS:
        c = dict(base_ctx)
        h, d, r, cells = tpp.plan_catalogue(
            cfg, c, random.Random(SEED + 11),
            origin=GRID_ORIGIN, step=GRID_STEP)
        all_humans += h
        all_cover += d
        all_records += r
        print("[bench] GRID  {0} figure(s), {1} covering piece(s), "
              "{2} cell(s)".format(len(h), len(d), len(cells)))
        # ONE CAMERA PER HALF-ROW. A whole row of thirteen cells is 41 m wide;
        # at the 18 mm lens `snapshots.place_camera` uses that needs 21 m of
        # altitude and puts a 1.8 m body at ~55 px, which is the condition this
        # bench exists to avoid judging from. Two frames per row at 13 m each
        # is ~90 px a body and a hand is a hand.
        n_pat = len(tpp.CATALOGUE_OCCLUSION)
        for j, pose in enumerate(tpp.CATALOGUE_POSES):
            y = GRID_ORIGIN[1] + j * GRID_STEP[1]
            for half in (0, 1):
                lo = half * (n_pat // 2)
                hi = n_pat if half else (n_pat // 2)
                if hi <= lo:
                    continue
                cx = GRID_ORIGIN[0] + (lo + hi - 1) * 0.5 * GRID_STEP[0]
                subjects["row_%d_%s_%d" % (j, pose, half)] = (cx, y + 0.9)

    # ---- HOUSE: the real planner over one wrecked house --------------------
    if "HOUSE" in UNITS:
        hx, hy = HOUSE_XY
        fp = 15.0
        key = "house_{0}_{1}".format(WRECK_STYLE, WRECK_LEVEL)
        usd = arch.get(key)
        if usd:
            _ref(stage, PARENT + "/wreck", usd, hx, hy,
                 rng.uniform(0.0, 360.0), ssf)
        else:
            print("[bench] !! no archetype {0} in {1}".format(key, ARCH_DIR))
        # A REAL PLANK FIELD, scattered by the same call the assembly makes, so
        # `_Deck` has the same kind of surface to measure that it will have in
        # the scene. Without this the bench would test the flat-deck fallback
        # and prove nothing about the thing that was actually wrong.
        prng = random.Random(SEED + 77)
        field = planks.scatter_from_wreck(hx, hy, fp, 0.9, 58.0, 16.0, prng,
                                          n_pieces=620)
        region = (hx - 26.0, hy - 26.0, hx + 26.0, hy + 26.0)
        field, _off = planks.clip_to_region(field, region, verbose=False)
        _build_planks(stage, field, PARENT + "/debris", ssf)
        c = dict(base_ctx)
        c.update({
            "wrecks": [{"x": hx, "y": hy, "fp": fp, "intensity": 0.9,
                        "level": WRECK_LEVEL}],
            "road_pts": [(hx + t, hy - 21.0, 0.0) for t in range(-14, 15, 5)],
            "plank_specs": field,
            "region": region,
        })
        hcfg = dict(cfg)
        # ONE HOUSE HAS TO CARRY THE WHOLE UNIT, so it is asked for more bodies
        # than a house in the corridor would get. Everything else — where they
        # go, how flat the deck has to be, what goes on top — is the assembly's
        # own configuration, untouched.
        hcfg["per_wreck"] = dict(cfg["per_wreck"])
        hcfg["per_wreck"][WRECK_LEVEL] = [8, 10]
        hcfg["max_total"] = 12
        h, d, r = tpp.plan_people(hcfg, c, random.Random(SEED + 23))
        all_humans += h
        all_cover += d
        all_records += r
        print("[bench] HOUSE {0} board(s) in the field, {1} casualt(ies), "
              "{2} covering piece(s)".format(len(field), len(h), len(d)))
        subjects["house_all"] = (hx, hy)
        wide["house_all"] = (hx, hy)
        for i, rec in enumerate(r):
            subjects["house_%d_%s_%s" % (i, rec["attitude"],
                                         rec["occlusion"])] = (
                rec["x"] + math.cos(math.radians(rec["body_axis_deg"]))
                * rec["reach_m"] * 0.5,
                rec["y"] + math.sin(math.radians(rec["body_axis_deg"]))
                * rec["reach_m"] * 0.5)

    # ---- author everything -------------------------------------------------
    if all_humans:
        sg.apply_placements(stage, all_humans, PARENT + "/people", ssf,
                            resolver=resolver, instance_categories=set())
    if all_cover:
        _build_planks(stage, [_to_plank_spec(x, rng) for x in all_cover],
                      PARENT + "/cover", ssf)

    for _ in range(25):
        omni.kit.app.get_app().update()

    summ = tpp.summarise(all_records)
    print("\n" + "=" * 72)
    print("TORNADO CASUALTY BENCH   units: {0}   {1:.0f} s"
          .format(",".join(sorted(UNITS)), time.time() - t0))
    print("  bodies      {0}".format(summ["total"]))
    print("  attitude    {0}".format(", ".join(
        "%s=%d" % kv for kv in sorted(summ["by_attitude"].items()))))
    print("  occlusion   {0}".format(", ".join(
        "%s=%d" % kv for kv in sorted(summ["by_occlusion"].items()))))
    print("  visibility  {0}  (max covered {1:.0%}, mean partial {2:.0%})"
          .format(", ".join("%s=%d" % kv
                            for kv in sorted(summ["by_visibility"].items())),
                  summ["max_covered_frac"], summ["mean_covered_frac"]))
    print("  parts seen  {0}".format(", ".join(
        "%s=%d" % kv for kv in sorted(summ["visible_parts"].items()))))
    print("  pieces      {0} laid on bodies".format(summ["boards"]))
    print("  LOOK FOR:")
    print("    1  is a side-lying figure on its SIDE, both hips level, one")
    print("       shoulder up — not face-down and not floating?")
    print("    2  does each covered body show the parts its record names?")
    print("    3  is anything effectively invisible? nothing may be.")
    print("    4  HOUSE: is every body ON the boards — not through, not over?")
    print("=" * 72 + "\n")

    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            _sp = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "..", "utils", "snapshots.py")
            _spec = _ilu.spec_from_file_location("snapshots",
                                                 os.path.normpath(_sp))
            _snaps = _ilu.module_from_spec(_spec)
            _spec.loader.exec_module(_snaps)
            os.makedirs(SNAP_DIR, exist_ok=True)
            # CLOSE. The subject is a body about 1.8 m long lying flat, not a
            # 100 m plate: `views_around`'s 60 m top-down puts it at a dozen
            # pixels, which is exactly the condition that let a bad pose ship.
            _snaps.views_around(stage, subjects, SNAP_DIR, ssf,
                                top_h=13.0, obl_dist=11.0, obl_h=7.0)
            if wide:
                _snaps.views_around(stage, {k + "_wide": v
                                            for k, v in wide.items()},
                                    SNAP_DIR, ssf, top_h=46.0, obl_dist=40.0,
                                    obl_h=26.0)
            print("[bench] snapshots -> {0} ({1} subject(s))"
                  .format(SNAP_DIR, len(subjects) + len(wide)))
        except Exception as _exc:
            import traceback
            print("[bench] snapshots FAILED: {0}".format(_exc))
            traceback.print_exc()

    app = omni.kit.app.get_app()
    while simulation_app.is_running():
        app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
