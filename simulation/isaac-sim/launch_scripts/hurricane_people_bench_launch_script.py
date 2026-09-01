#!/usr/bin/env python
"""hurricane_people_bench_launch_script.py — the PEOPLE bench.

WHY THIS EXISTS. Roof posture has now failed review twice on the full
500 m plate, where a figure is a few pixels and a rebuild costs ~3 minutes.
The user's instruction was explicit: "Let's not bring up the whole scene.
Just bring up 2 houses and the water. Show me people in the water, people
sitting/standing on the roof and people under debris in the last house."

So: TWO BAKED TORNADO HOUSES (never re-baked here — `archetypes_tornado` is
the approved damage library), one flat water quad, no trees, no debris
field beyond what the trapped figures need, and close cameras. It builds in
well under a minute.

THE ROOF A/B/C ROW is the point of the bench. Three figures sit side by side
on the SAME slope so the reviewer compares postures in one frame:
    A  seated, roll SOLVED by `hurricane_people._solve_sit_slump_roll`
       (what the full scene ships today)
    B  seated, roll ZERO — the control: is the solved roll helping or
       hurting?
    C  standing upright (`idle`), feet on the slope — the user asked to see
       "some ... standing on their [roof]" and it is the cheapest posture to
       get right, so it is the fallback if seated keeps failing.
Each is labelled in the log with its world position so a crop can be tied
back to a variant.

Env:
    BENCH_SNAP_DIR   where the PNGs go (default …/logs/PEOPLE_BENCH)
    BENCH_KEEP_OPEN  1 to hold the GUI open after the captures
    BENCH_HEADLESS   0 for a window (default 1)
"""
import math
import os
import random
import sys
import time

from isaacsim import SimulationApp

_HEADLESS = os.environ.get("BENCH_HEADLESS", "1").strip() not in ("0", "false")
simulation_app = SimulationApp(launch_config={"headless": _HEADLESS})

import omni.kit.app                                              # noqa: E402
import omni.usd                                                  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade          # noqa: E402

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

import scene_generator as sg                                     # noqa: E402
import suburb_scene as ss                                        # noqa: E402
from compile_disaster import load_scene_config                   # noqa: E402
from disaster import hurricane_people as hpp                     # noqa: E402
from disaster import people as pp                                # noqa: E402
from disaster import tornado_people as tpp                     # noqa: E402
from disaster import planks                                      # noqa: E402

PARENT = "/World/bench"
WATER_Z = 1.05          # metres above the ground plane
SHORE_X = 0.0           # water occupies x < SHORE_X
HOUSE_INTACT = ("cottage", "pristine", 8.0, 9.0)      # style, level, x, y
HOUSE_WRECK = ("ranch", "partial_collapse", 8.0, -9.0)
ARCH_DIR = os.environ.get(
    "HOUSE_ARCH_DIR",
    os.path.join(_SCENE_GEN_DIR, "assets", "archetypes_tornado"))
SNAP_DIR = os.environ.get(
    "BENCH_SNAP_DIR", "/isaac-sim/.nvidia-omniverse/logs/PEOPLE_BENCH")


def _quad(stage, path, x0, y0, x1, y1, z, rgb, opacity=1.0):
    m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    m.CreatePointsAttr([(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)])
    m.CreateFaceVertexCountsAttr([4])
    m.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    m.CreateExtentAttr([(x0, y0, z), (x1, y1, z)])
    mat = UsdShade.Material.Define(stage, Sdf.Path(path + "_mat"))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(path + "_mat/Shader"))
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateIdAttr("OmniPBR")
    sh.CreateInput("diffuse_color_constant",
                   Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("reflection_roughness_constant",
                   Sdf.ValueTypeNames.Float).Set(0.35 if opacity < 1 else 0.8)
    if opacity < 1.0:
        sh.CreateInput("enable_opacity", Sdf.ValueTypeNames.Bool).Set(True)
        sh.CreateInput("opacity_constant",
                       Sdf.ValueTypeNames.Float).Set(float(opacity))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
    return m


def _ref_house(stage, dst, style, level, x, y, yaw=0.0):
    usd = os.path.join(ARCH_DIR, "house_{0}_{1}.usd".format(style, level))
    if not os.path.isfile(usd):
        print("[bench] MISSING ARCHETYPE {0}".format(usd))
        return False
    prim = stage.DefinePrim(Sdf.Path(dst), "Xform")
    prim.GetReferences().AddReference(usd)
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
    xf.AddRotateZOp().Set(float(yaw))
    print("[bench] house {0}/{1} at ({2:.0f}, {3:.0f}) -> {4}"
          .format(style, level, x, y, dst))
    return True


def main():
    ctxo = omni.usd.get_context()
    ctxo.new_stage()
    stage = ctxo.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.DefinePrim(Sdf.Path(PARENT), "Xform")

    # ---- ground, water, light -------------------------------------------
    _quad(stage, PARENT + "/ground", -26, -20, 26, 20, 0.0, (0.20, 0.26, 0.12))
    _quad(stage, PARENT + "/water", -26, -20, SHORE_X, 20, WATER_Z,
          (0.16, 0.13, 0.08), opacity=0.72)
    sun = UsdLux.DistantLight.Define(stage, Sdf.Path(PARENT + "/sun"))
    sun.CreateIntensityAttr(2600.0)
    sun.CreateAngleAttr(3.0)
    UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-42, 0, 35))
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path(PARENT + "/dome"))
    dome.CreateIntensityAttr(900.0)
    # A TEXTURED SKY, or every oblique is a uniform frame and `snapshots_rp`
    # correctly calls it blank and gives up (measured 2026-09-01: only the
    # top-downs survived). Same HDR the full launcher swaps in.
    dome.CreateTextureFileAttr(Sdf.AssetPath(os.environ.get(
        "BENCH_SKY_HDRI",
        "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
        "Assets/Isaac/4.5/NVIDIA/Assets/Skies/Storm/"
        "approaching_storm_4k.hdr")))

    # ---- the two houses, straight out of the TORNADO library ------------
    _ref_house(stage, PARENT + "/h_intact", HOUSE_INTACT[0], HOUSE_INTACT[1],
               HOUSE_INTACT[2], HOUSE_INTACT[3])
    _ref_house(stage, PARENT + "/h_wreck", HOUSE_WRECK[0], HOUSE_WRECK[1],
               HOUSE_WRECK[2], HOUSE_WRECK[3])

    # ---- the rigged humans, from the real asset pool --------------------
    config = load_scene_config("suburb_hurricane_500_l3")
    resolver = sg._make_resolver(config)
    pools = ss.AssetPools(config)
    raw = ss._raw_pool(config, "humans")
    rigged = pools.load_tagged(raw, "rigged") or pools.load(raw)
    print("[bench] {0} rigged human asset(s)".format(len(rigged)))
    rng = random.Random(7)
    humans = []

    # ---- A/B/C ROOF ROW --------------------------------------------------
    # The SAME slope, three postures, 1.6 m apart along the ridge direction
    # so one oblique frames all three.
    style = HOUSE_INTACT[0]
    slope = hpp._ROOF_SLOPES_LOCAL[style][0]        # downhill +X facet
    hx, hy = HOUSE_INTACT[2], HOUSE_INTACT[3]
    pitch = float(slope["pitch_deg"])
    ridge_z, eave_z = float(slope["ridge_z"]), float(slope["eave_z"])
    dx, dy = slope["downhill_xy"]
    yaw_world = hpp._facing_yaw_for_dir(dx, dy)
    # half-way down the fall line
    x0, y0, x1, y1 = slope["bbox"]
    run = abs(x1 - x0)
    seat_frac = 0.5
    seat_local_x = x0 + run * seat_frac if dx > 0 else x1 - run * seat_frac
    seat_z = ridge_z - (ridge_z - eave_z) * seat_frac
    variants = []
    # THE SKILL'S OWN FIX (`place-people-in-scenes`, "Posing (UsdSkel)"): the
    # rig origin is at the FEET, posing joints does NOT move it, and the
    # per-pose `_POSE_Z_OFFSET` (sit_ground -0.80, sit_edge -0.85, crouch
    # -0.61, scaled by measured height) plus `_MALE_SEATED_DZ_M = -0.15` is
    # what puts the support back on the surface. `people._human_placement`
    # applies all of it; the hand-rolled z solve this file used before did
    # not, which is why the figures sat 0.9 m INTO the roof. Every variant
    # below therefore goes through that call — the only thing under test now
    # is the ROLL.
    _pctx = {"asset_pools": pools, "resolver": resolver}
    for i, tag in enumerate(("stand_1", "stand_2", "stand_3")):
        usd = rigged[i % len(rigged)]
        ty = (i - 1) * 1.8                     # spread along the ridge
        target = (hx + seat_local_x, hy + ty, seat_z)
        pose = "idle"          # STANDING ONLY -- the approved roof posture
        # SAME JITTER THE MODULE NOW USES — three figures on one slope must
        # not line up like a parade (user, on the previous bench frame).
        _yaw = yaw_world + rng.uniform(-70.0, 70.0)
        pl = pp._human_placement(_pctx, usd, target[0], target[1], target[2],
                                 _yaw, pose)
        # No roll: a standing figure's feet sit on the plane and any tilt is
        # exactly what made the seated variants recline into the sky.
        if pl:
            humans.append(pl)
            variants.append((tag, target))
            print("[bench] roof {0}: pose={1} roll={2:.1f} yaw={3:.1f} "
                  "at ({4:.2f}, {5:.2f}, {6:.2f})"
                  .format(tag, pl.get("pose"), float(pl.get("roll_deg", 0.0)),
                          float(pl.get("yaw_deg", 0.0)), *target))

    # ---- PEOPLE IN THE WATER (the pool posture, chest-deep) --------------
    for i in range(4):
        usd = rigged[(i + 1) % len(rigged)]
        # HEIGHT THE SAME WAY `people._human_placement` gets it: the
        # resolver's own footprint for this asset, never a guessed constant.
        try:
            height = float(resolver.get(usd, "human", scale=1.0,
                                        axis_up="Z").get("sz", 1.8)) or 1.8
        except Exception:
            height = 1.8
        wx = -4.0 - 3.2 * i
        wy = -6.0 + 4.5 * i
        # THROUGH `_human_placement`, NEVER HAND-BUILT. A hand-built dict
        # carries `"scale": 1.0`, but these rigs are authored in CENTIMETRES
        # and `ap.scale_of(usd)` is what supplies their real 0.01 — which is
        # why the hand-built water figures rendered as giants next to the
        # correctly-placed roof ones (user, on the bench: "some of the people
        # are huge"). The chest-deep sink is then just the pool posture's own
        # z_ground, exactly as `people`'s pool scenario does it.
        _wpl = pp._human_placement(
            {"asset_pools": pools, "resolver": resolver}, usd,
            wx, wy, WATER_Z - pp.CHEST_FRAC * height,
            rng.uniform(0.0, 360.0), "idle")
        humans.append(_wpl)
    print("[bench] water figures: 4 at chest depth (water z={0:.2f})"
          .format(WATER_Z))

    # ---- PEOPLE UNDER DEBRIS: TORNADO'S OWN CATALOGUE ---------------------
    # `tornado_people.plan_catalogue` is the function the tornado people
    # PREVIEW bench itself uses: it lays a figure per (pose x occlusion
    # pattern) and returns the covering plank specs that partially bury each
    # one (`_cover_piece`: `flat` boards bridging the body, `propped` boards
    # with one end on the deck rising over it). Hand-scattering boards near a
    # figure -- what this bench did before -- covers nothing, which is why
    # nothing read as trapped.
    trapped_specs = []
    trap_xy = (HOUSE_WRECK[2], HOUSE_WRECK[3])
    trap_span = 18.0
    trap_closeups = []
    try:
        _tctx = {"region": (-26.0, -20.0, 26.0, 20.0), "humans": rigged,
                 "resolver": resolver, "asset_pools": pools,
                 # THE HOUSES, SO A BODY IS NOT LAID INSIDE ONE. `_plan_dry`
                 # derives its `intact` keepout list from `ctx["houses"]`
                 # (pristine ones only) and the full launcher always supplies
                 # it; this bench did not, so `intact` came back EMPTY, the
                 # `house_clear_m` (7 m) keepout had nothing to test against,
                 # and the 2026-09-01 run laid a casualty at (+4.34, +3.57) —
                 # 6.7 m from the cottage centre, i.e. INSIDE the standing
                 # house. Its close-up rendered as a black interior wall,
                 # which is how it was caught.
                 "houses": [
                     {"x": HOUSE_INTACT[2], "y": HOUSE_INTACT[3],
                      "style": HOUSE_INTACT[0], "level": HOUSE_INTACT[1]},
                     {"x": HOUSE_WRECK[2], "y": HOUSE_WRECK[3],
                      "style": HOUSE_WRECK[0], "level": HOUSE_WRECK[1]}],
                 # DICTS, not the launcher's tuples: `tornado_people`'s own
                 # docstring specifies `wrecks [{x, y, fp, intensity, level}]`.
                 "wrecks": [{"x": HOUSE_WRECK[2], "y": HOUSE_WRECK[3],
                             "fp": 12.0, "intensity": 0.8,
                             "level": HOUSE_WRECK[1]}],
                 # A REAL PLANK FIELD around the wreck: `tornado_people`
                 # measures the debris DECK from these to decide how deep a
                 # body sits and where a covering piece rests. With an empty
                 # list it falls back to a flat deck and the figures lie on
                 # bare grass, which is not what a tornado casualty looks
                 # like.
                 "plank_specs": planks.scatter_from_wreck(
                     HOUSE_WRECK[2], HOUSE_WRECK[3], 12.0, 0.85,
                     200.0, 14.0, random.Random(19), n_pieces=140,
                     ground_z=0.0),
                 "deck_points": [],
                 "intensity_at": lambda x, y: 0.8}
        # THE HURRICANE'S OWN DRY CONFIG, NOT THE TORNADO'S RAW ONE (changed
        # 2026-09-01). `hurricane_people.resolve_cfg(config)["dry"]` IS
        # `tpp.resolve_cfg(config)` plus the two hurricane deltas —
        # `min_intensity = 0` (a near-uniform wind field makes the tornado's
        # corridor gate meaningless here) and the reweighted `occlusion` bag
        # ("more under debris, fewer lying in the open": `none` 0.44 -> 0.18).
        # The bench previously ran the TORNADO weights, so it under-showed
        # the covering the full scene actually ships. Nothing else differs.
        _tcfg = dict(hpp.resolve_cfg(config)["dry"])
        # A BENCH BUDGET, not the plate's. `per_wreck` gives a
        # `partial_collapse` house 1-2 casualties and the plate ceiling is 40
        # -- correct for a 500 m corridor, but this bench has ONE wreck and
        # the point is to SHOW the occlusion, so ask that one wreck for a
        # group. Nothing about the covering geometry changes.
        _tcfg["per_wreck"] = dict(_tcfg.get("per_wreck") or {})
        _tcfg["per_wreck"]["partial_collapse"] = [6, 8]
        _tcfg["max_total"] = 10
        # `_plan_dry`, which is the EXACT call `hurricane_people.plan_people`
        # makes for the non-flooded region — not `tpp.plan_people` directly
        # and not `plan_catalogue`. The catalogue is a diagnostic GRID (one
        # figure per pose x occlusion pattern, laid out in the open) and
        # renders as bodies in a lawn nowhere near the wreck; `_plan_dry` is
        # the in-situ path the hurricane scene ships: casualties AT the
        # wreck, with the covering pieces that partially bury them.
        t_humans, t_debris, _t_recs = hpp._plan_dry(
            _tcfg, _tctx, random.Random(31))
        humans.extend(t_humans)
        trapped_specs = list(t_debris or [])
        _occ = {}
        for _r in _t_recs:
            _k = _r.get("occlusion") or "none"
            _occ[_k] = _occ.get(_k, 0) + 1
        print("[bench] trapped: {0} figure(s) from hurricane's dry pass, "
              "{1} covering piece(s)".format(len(t_humans), len(trapped_specs)))
        print("[bench] trapped occlusion: {0}".format(dict(sorted(_occ.items()))))
        for _r in _t_recs:
            print("[bench]   ({0:+.2f}, {1:+.2f}) {2} / {3} / covered "
                  "{4:.2f} / sees {5}".format(
                      _r["x"], _r["y"], _r.get("pose"), _r.get("occlusion"),
                      float(_r.get("covered_frac") or 0.0),
                      ",".join(_r.get("visible_parts") or ["-"])))
        # AIM AT WHERE THE BODIES ACTUALLY ARE. The trapped cameras used to
        # be pointed at `(wx - 7, wy - 2)`, where `wx`/`wy` are the WATER
        # loop's last iteration variables -- (-13.6, 7.5), i.e. out over the
        # water quad, ~24 m from the wreck. Both `trapped_debris` and
        # `trapped_close` have therefore been photographing empty water on
        # every run this bench has ever made, which is why the burial has
        # never been visible in a bench frame. The casualty group is a comet
        # off the wreck, not centred on it, so use the group's own centroid
        # and size the top-down to contain it.
        if _t_recs:
            _cx = sum(_r["x"] for _r in _t_recs) / len(_t_recs)
            _cy = sum(_r["y"] for _r in _t_recs) / len(_t_recs)
            _rad = max(4.0, max(max(abs(_r["x"] - _cx), abs(_r["y"] - _cy))
                                for _r in _t_recs))
            trap_xy = (_cx, _cy)
            # CAPPED. `_trail` throws part of the population 10-40 m off the
            # wreck, so containing every one of them costs a ~40 m top-down
            # in which a body is a smudge and a board over it is invisible.
            # The group frame is context; the per-casualty crops below are
            # the actual evidence.
            trap_span = min(26.0, 2.0 * _rad + 6.0)
            print("[bench] trapped group centred ({0:+.2f}, {1:+.2f}), "
                  "span {2:.1f} m (full spread {3:.1f} m)"
                  .format(_cx, _cy, trap_span, 2.0 * _rad + 6.0))
        # ONE CLOSE FRAME PER CASUALTY, one per DISTINCT occlusion pattern.
        # "Show me people under debris partially visible" is a per-body
        # judgement -- which parts the boards cover and which a camera can
        # still see -- and no single wide frame answers it. Each crop is
        # named for the pattern it is meant to demonstrate so a reviewer can
        # check the picture against the label this scene writes into
        # `PEOPLE_JSON` for that same figure.
        _seen = set()
        for _r in _t_recs:
            _k = _r.get("occlusion") or "none"
            if _k in _seen:
                continue
            _seen.add(_k)
            trap_closeups.append(("trapped_%s" % _k, (_r["x"], _r["y"]),
                                  _r.get("covered_frac"),
                                  _r.get("visible_parts")))
    except Exception as _exc:
        import traceback
        print("[bench] dry/trapped pass FAILED: {0}".format(_exc))
        traceback.print_exc()

    sg.apply_placements(stage, humans, PARENT + "/people", 1.0,
                        resolver=resolver, instance_categories=set())

    # READ BACK WHAT WAS ACTUALLY AUTHORED. The module SOLVES a root position
    # so the pelvis lands on the roof plane; whether `apply_placements`
    # consumes `z_m` as that root is the open question, and a render that
    # shows figures buried to the chest says it does not. Print intent vs
    # reality for the roof row, plus the lowest authored point of each body.
    xcache = UsdGeom.XformCache()
    for tag, target in variants:
        hit = None
        for prim in stage.GetPrimAtPath(Sdf.Path(PARENT + "/people")).GetChildren():
            t = xcache.GetLocalToWorldTransform(prim).ExtractTranslation()
            if abs(t[0] - target[0]) < 2.5 and abs(t[1] - target[1]) < 1.0:
                hit = (prim, t)
                break
        if hit is None:
            print("[bench] {0}: no prim matched the target".format(tag))
            continue
        prim, t = hit
        lo = 1e9
        for d in Usd.PrimRange(prim):
            if not d.IsA(UsdGeom.Mesh):
                continue
            pts = UsdGeom.Mesh(d).GetPointsAttr().Get()
            if not pts:
                continue
            m = xcache.GetLocalToWorldTransform(d)
            for q in pts:
                z = (Gf.Vec4d(q[0], q[1], q[2], 1.0) * m)[2]
                lo = min(lo, z)
        print("[bench] {0}: intended pelvis z={1:.2f} | authored root z={2:.2f} "
              "(dz={3:+.2f}) | lowest body point z={4:.2f} | roof plane z={5:.2f}"
              .format(tag, target[2], t[2], t[2] - target[2], lo, target[2]))

    # `len`/`wide` -> `l`/`w`, THE MAPPING THE PLANNER'S OUTPUT REQUIRES.
    # `tornado_people._cover_piece` emits `{x, y, z, len, wide, t, yaw,
    # pitch, roll, class}`; `planks._box` reads `l`/`w`/`t`. Handing it the
    # raw specs raises `KeyError: 'l'` on the FIRST board, and the bare
    # `except` below then printed one line and carried on — so this bench
    # has authored ZERO covering boards on every run it has ever made, which
    # (with the mis-aimed cameras above) is the whole reason the burial has
    # never appeared in a bench frame. Both assembly launchers do exactly
    # this mapping; the bench simply never did.
    # THE DEBRIS FIELD ITSELF, AUTHORED — not only measured. `_tctx
    # ["plank_specs"]` is what `tornado_people._Deck` reads to decide how deep
    # a body sits and where a covering piece rests, and the bench computed it
    # and never built it: the casualties were therefore seated on the top of
    # an INVISIBLE deck, up to 0.5 m above bare grass, and the frames showed
    # bodies and boards floating over an untouched lawn. The full launcher
    # builds this field (`_ld`, `# LAND DEBRIS`), so a bench without it is
    # not showing the scene. `scatter_from_wreck` already emits l/w/t, so
    # these need no mapping — unlike the covering pieces below.
    try:
        _fmats = planks.materials(stage, PARENT + "/field_looks")
        _fspec, _n_off = planks.clip_to_region(
            list(_tctx.get("plank_specs") or []), (-26.0, -20.0, 26.0, 20.0),
            verbose=False)
        if _fspec:
            planks.build(stage, PARENT + "/field", _fspec, _fmats, 1.0,
                         verbose=False)
        print("[bench] debris field: {0} piece(s) built ({1} off-plate)"
              .format(len(_fspec), _n_off))
    except Exception as exc:
        import traceback
        print("[bench] FIELD BUILD FAILED: {0}: {1}".format(
            type(exc).__name__, exc))
        traceback.print_exc()

    specs = [{"x": d["x"], "y": d["y"], "z": d["z"],
              "l": d["len"], "w": d["wide"], "t": d["t"], "yaw": d["yaw"],
              "pitch": d["pitch"], "roll": d["roll"], "class": d["class"]}
             for d in trapped_specs]
    try:
        _pmats = planks.materials(stage, PARENT + "/debris_looks")
        planks.build(stage, PARENT + "/debris", specs, _pmats, 1.0)
        print("[bench] {0} board(s) over the trapped figures".format(len(specs)))
    except Exception as exc:
        # LOUD. The silent version of this handler is what hid the KeyError
        # above through every previous run of this bench.
        import traceback
        print("[bench] PLANK BUILD FAILED: {0}: {1}".format(
            type(exc).__name__, exc))
        traceback.print_exc()

    for _ in range(60):
        omni.kit.app.get_app().update()

    # ---- CAPTURES --------------------------------------------------------
    os.makedirs(SNAP_DIR, exist_ok=True)
    import importlib.util as ilu
    spec = ilu.spec_from_file_location(
        "snapshots_rp", os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots_rp.py"))
    sn = ilu.module_from_spec(spec)
    spec.loader.exec_module(sn)
    # AIM AT THE FIGURES, CLOSE. A 16 m top-down over a flat water quad with
    # four small figures in it is a near-uniform frame, and `snapshots_rp`
    # calls that blank and gives up -- correctly. Each subject therefore
    # carries its own framing, tight enough that the people fill the frame.
    subjects = {
        "roof_abc": (hx + seat_local_x, hy),
        "water_people": (-7.2, -1.5),
        "trapped_debris": trap_xy,
    }
    cam = {
        "roof_abc": dict(top_h=16.0, obl_dist=12.0, obl_h=7.0, aim_h=1.6),
        "water_people": dict(top_h=7.0, obl_dist=6.0, obl_h=2.8, aim_h=1.0),
        # SIZED TO THE GROUP, not to one body: `_plan_dry` scatters 6-8
        # casualties over a comet several metres long, and the old fixed
        # 6.5 m oblique framed a fraction of it.
        "trapped_debris": dict(top_h=max(8.0, 0.9 * trap_span),
                               obl_dist=max(6.5, 0.75 * trap_span),
                               obl_h=max(3.2, 0.38 * trap_span), aim_h=0.8),
    }
    # ONE `views_around` CALL FOR ALL SUBJECTS. Three separate calls each
    # build their own render product, and measured 2026-09-01 the later ones
    # come back BLANK and burn their three retries ("GAVE UP ... frame stayed
    # blank") while the first subject's frames are fine — the same
    # not-ready-buffer family as the black-frame trap in the hurricane bug
    # catalogue. One call reuses one product, and the app is pumped first.
    # PER-SUBJECT AZIMUTH: `azimuth_deg` is the compass bearing of the CAMERA
    # FROM the subject, so each one is chosen to put that subject's own
    # content (the house, the shoreline) BEHIND it in frame.
    az = {"roof_abc": 0.0,          # camera east of the seats, looking west
          "water_people": 180.0,    # camera west, looking back at the shore
          "trapped_debris": 180.0}  # camera west, looking at the wreck
    for _ in range(40):
        omni.kit.app.get_app().update()
    # ONE SUBJECT PER RUN when `BENCH_SUBJECT` is set. Measured across five
    # bench runs: whichever subject is captured FIRST renders, and every
    # later one comes back uniform and burns its retries -- the render
    # product does not survive being re-aimed in a scene this small. Three
    # 40-second runs are cheaper than one unreliable run.
    _only = os.environ.get("BENCH_SUBJECT", "").strip()
    for name, (sx, sy) in subjects.items():
        if _only and name != _only:
            continue
        # MORE SUBFRAMES AND MORE PUMPING PER SUBJECT. Blank frames here are
        # the buffer-not-ready family from the hurricane bug catalogue, and
        # in this tiny bench the later subjects were the ones that came back
        # uniform; `snapshots_rp` then correctly gave up after its retries.
        for _ in range(25):
            omni.kit.app.get_app().update()
        try:
            sn.views_around(stage, {name: (sx, sy)}, SNAP_DIR, 1.0,
                            azimuth_deg=az.get(name, 225.0), subframes=16,
                            **cam.get(name, cam["roof_abc"]))
        except Exception as exc:
            print("[bench] capture {0} FAILED: {1}".format(name, exc))
    try:
        sn.overview(stage, (0.0, 0.0), 60.0,
                    os.path.join(SNAP_DIR, "bench_overview.png"))
    except Exception as exc:
        print("[bench] overview FAILED: {0}".format(exc))
    # TIGHT TOP-DOWNS VIA `overview`, WHICH RENDERS WHERE `views_around`'s
    # oblique does not. Five runs of evidence: the oblique path returns a
    # uniform frame for the water and trapped subjects at every distance
    # tried, while `overview` has rendered on every single run. Rather than
    # keep bisecting the capture stack, use the path that works -- a close
    # top-down still shows a reviewer whether a figure is in the water up to
    # the chest and whether boards lie over a trapped one.
    # 9 m span for a casualty crop. `snapshots_rp.overview` puts the camera at
    # `0.95 * span` metres up, so the 5.5 m first tried sat at 5.2 m — BELOW
    # this kit's 5.6 m ridge — and any subject near a house rendered as the
    # inside of its roof (measured 2026-09-01: `trapped_torso_head` came back
    # a black interior wall). 9 m clears the ridge with margin and still puts
    # a 1.8 m body across a fifth of the frame, which is enough to read which
    # parts a board is covering.
    _CLOSE_SPAN_M = 9.0
    _closes = [("water_close", (-7.2, -1.5), 16.0),
               ("trapped_close", trap_xy, trap_span),
               ("roof_close", (hx + seat_local_x, hy), 18.0)]
    _closes += [(_n, _xy, _CLOSE_SPAN_M)
                for (_n, _xy, _cf, _vp) in trap_closeups]
    for _cname, _cxy, _cspan in _closes:
        if _only and not _cname.startswith(_only.split("_")[0]):
            continue
        for _ in range(20):
            omni.kit.app.get_app().update()
        try:
            sn.overview(stage, _cxy, _cspan,
                        os.path.join(SNAP_DIR, _cname + ".png"))
        except Exception as exc:
            print("[bench] {0} FAILED: {1}".format(_cname, exc))
    for (_n, _xy, _cf, _vp) in trap_closeups:
        print("[bench] {0}.png -> covered {1:.2f}, visible {2}".format(
            _n, float(_cf or 0.0), ",".join(_vp or ["-"])))
    print("[bench] snapshots -> {0}".format(SNAP_DIR))
    print("[bench] BENCH_DONE")
    if os.environ.get("BENCH_KEEP_OPEN", "").strip() == "1":
        while simulation_app.is_running():
            omni.kit.app.get_app().update()


if __name__ == "__main__":
    # PRINT THE TRACEBACK BEFORE `close()`. `simulation_app.close()` hard-exits
    # the process, so an exception propagating out of `main()` is swallowed
    # and the run looks like a clean early shutdown with no error at all
    # (measured 2026-09-01: the first bench run died silently right after its
    # last print). Catch, print, flush, THEN close.
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
    finally:
        simulation_app.close()
