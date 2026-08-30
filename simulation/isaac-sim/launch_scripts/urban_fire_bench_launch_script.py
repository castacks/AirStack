#!/usr/bin/env python
"""
Urban fire bench — a few urban kit buildings on empty ground, each a
DIFFERENT type and a DIFFERENT fire severity, fractured/settled live and
captured from above, obliquely and from the street.

    UF_SET=default SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/uf_bench \
    ISAAC_SIM_SCRIPT_NAME=urban_fire_bench_launch_script.py airstack up isaac-sim

WHY A MIXED ROW RATHER THAN ONE STYLE PER ROW
----------------------------------------------
`eq_building_bench_launch_script.py` is the opposite design on purpose: it
builds ONE style and varies only the recipe, because an earthquake ladder is
a controlled comparison and any change of style is a confound. A fire is not
being compared here — the question is whether an urban block that has had a
fire in it reads as one — so every column is a different building AND a
different severity, which is what an actual street after a fire looks like:
one gutted shell, its neighbours smoke-stained, a tower with a stripe up it
and something further along untouched.

Set with `UF_SET` (see `SETS` below) or spell it out with
`UF_BUILDINGS=<style>:<level>,<style>:<level>,...`.

Env:
    UF_SET          named set from `SETS` (default `default`)
    UF_BUILDINGS    explicit `style:level` list, overrides UF_SET
    UF_SEED         rng seed (default 7)
    UF_SPACING      metres between columns (default = 2.2 x the widest, min 60)
    UF_FLOW         1 (default) authors the NVIDIA Flow stack and the flames;
                    0 builds the damage only, which is ~40 s faster and is
                    the right setting for judging geometry
    UF_ORIGIN       force every fire to start at this storey (default: drawn)
    UF_SIDES        force the burning elevations for EVERY building, e.g.
                    `S` or `S,E`. Per building, a row in `SETS` can carry its
                    own instead as a fourth field (see `SETS` below) — which
                    is what `mce_collapse` uses to point the one partially
                    collapsed elevation at the cameras.
    SETTLE_STEPS    physics step ceiling (default 1600)
    KEEP_PHYSICS    1 leaves bodies live instead of baking
    SNAP_DIR        viewport captures, MUST be under
                    /isaac-sim/.nvidia-omniverse/logs/
    KEEP_OPEN       1 keeps the app up after the captures (headless default
                    is to exit)
"""

import math
import os
import random
import sys
import time

from isaacsim import SimulationApp


def _env(name, default=""):
    """The container exports every launcher knob as an EMPTY STRING, so
    `os.environ.get(name, default)` never reaches its default — a numeric
    knob raises fourteen seconds into the launch and a path knob silently
    becomes "". Treat empty as absent."""
    v = os.environ.get(name)
    return default if not (v or "").strip() else v.strip()


_HEADLESS = _env("ISAAC_SIM_HEADLESS", "false").lower() in ("1", "true", "yes")
# FRACTIONAL CUTOUT OPACITY — `extra_args`, NOT `carb.settings`.
# `disaster/wall_overlay.py` reveals its soot mask through OmniPBR's
# `enable_opacity_texture` + `opacity_mode=2`, and `urban_fire._glass_pane`
# grades its smoke deposit the same way. Both are FRACTIONAL CUTOUT opacity
# (`OmniPBRBase.mdl`), and RTX Real-Time DISCARDS fractional cutout unless
# this setting is on — so the overlay renders as a binary stamp: fully opaque
# wherever the mask is above zero, gone everywhere else. Reported as "burn
# edges don't fade. Most of the building just looks normal" (user,
# 2026-08-29).
#
# IT HAS TO BE A COMMAND-LINE FLAG. `disaster/ground.py` records this from the
# burnt-forest ground scar, which "drew nothing" until it was understood: "a
# `carb.settings.set_bool` after startup is never copied onto the USD render
# property the renderer reads, and the overlay silently vanishes." Setting it
# from Python after `SimulationApp` starts — which is what the first attempt
# at this fix did — therefore looks exactly like an art bug. `extra_args` is
# the path that works.
#
# Kept as a literal because `scene_gen` is not on `sys.path` until after
# `SimulationApp` is constructed; `disaster.ground.KIT_ARGS` is the source of
# truth and these must stay in step with it.
KIT_ARGS = ["--/rtx/raytracing/fractionalCutoutOpacity=true",
            "--/rtx/pathtracing/fractionalCutoutOpacity=true"]

simulation_app = SimulationApp(launch_config={"headless": _HEADLESS,
                                              "extra_args": KIT_ARGS})

from isaacsim.core.utils.extensions import enable_extension    # noqa: E402

enable_extension("omni.kit.window.script_editor")
enable_extension("omni.flowusd")

import omni.kit.app                                            # noqa: E402
import omni.timeline                                           # noqa: E402
import omni.usd                                                # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade             # noqa: E402

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

import numpy as np                                             # noqa: E402
import scene_generator as sg                                   # noqa: E402
from scene_prep import get_stage_meters_per_unit               # noqa: E402
from detail import urban_building as ub                        # noqa: E402
from disaster import fire as fx                                # noqa: E402
from disaster import fire_collapse as fcol                     # noqa: E402
from disaster import fracture, settle                          # noqa: E402
from disaster import quake_flow as qf                          # noqa: E402
from disaster import urban_fire as uf                          # noqa: E402

PARENT = "/World/stage/generated"

# (style, level) sets. VARIETY IS THE DELIVERABLE, so the default spans the
# three construction types, five of the eight kit families, 17 m to 103 m, and
# every severity from smoke-damaged to burnt collapse.
# An entry is `(style, level)` or `(style, level, origin_storey)`. THE ORIGIN
# IS WORTH PINNING ON A TALL BUILDING: `plan_fire` draws it low-biased, which
# is right (a fire starts in an occupied lower floor) and useless on a
# 33-storey tower, where a fire on storey 2 is 6 % of the elevation and
# invisible from any view that also shows the building.
SETS = {
    "default": [
        # burning NOW — flame out of the windows, clean floors below
        ("brownstone_row", "F2"),     # urm  38x14 x17 m  — terrace, low fire
        ("office_wide",    "F3"),     # rc   40x24 x22 m  — frame, spalling
        ("commercial",     "F4"),     # urm  22x18 x20 m  — burnt-out shell
        ("apartment_tall", "F3", 4),  # urm  27x17 x33 m  — vertical stripe
        ("skyscraper_a",   "F3", 17), # glass 25x25 x103 m — tower stripe
        ("dw_terrace",     "F5"),     # urm  25x15 x17 m  — burnt collapse
        ("department_store", "F1"),   # urm  42x30 x17 m  — smoke only
    ],
    # the four severities on comparable stock, for judging the ladder
    "ladder": [("commercial", "F1"), ("commercial", "F2"),
               ("commercial", "F3"), ("commercial", "F4"),
               ("commercial", "F5")],
    "towers": [("skyscraper_a", "F2", 22), ("skyscraper_b", "F3", 14),
               ("skyscraper_c", "F4", 11), ("highrise_02", "F3", 9),
               ("tower", "F4", 5)],
    "masonry": [("brownstone", "F3"), ("brownstone_row", "F4"),
                ("walkup", "F5"), ("church", "F3"), ("dw_terrace_long", "F2")],
    "concrete": [("office", "F3"), ("office_slab", "F4"),
                 ("office_plain", "F2"), ("civic_offices", "F3"),
                 ("civic_hall", "F4")],
    # THE MODERNCITY STREET, WITH ONE BUILDING PARTIALLY COLLAPSED.
    # Only the five kit styles `kit_substitute.route()` sends a merged
    # ModernCityEnvironment building to — `commercial_mid` and `commercial`
    # (family 04, urm), `apartment` (01, urm), `block_residential` (02, rc)
    # and `highrise_step` (05, rc_glass) — so what this row shows is what an
    # MCE block actually looks like after a fire, not a kit sampler.
    #
    # SIX at F1-F5 and EXACTLY ONE at F5c (user, 2026-08-30: "spawn the
    # modern city env with 1 partial collapsed building"). The F5c building
    # is the SAME STYLE as the F5 one on purpose: side by side they are the
    # whole argument for the new recipe — F5 (`fire_collapse`) drops the top
    # storeys into the shell and still has four walls from the street, F5c
    # (`fire_collapse.r_partial_collapse`) puts one burnt elevation IN the
    # street with the floors behind it sagged and the interior on show.
    #
    # The fourth field pins the burning elevations per building. It matters
    # for exactly one of them: `urban_fire_bench`'s `face` / `obl` / `close`
    # cameras are aimed at `fire["sides"][0]`, and
    # `fire_collapse.plan_partial_collapse` loses that same elevation — so
    # pinning it to S guarantees the collapse is the thing photographed
    # rather than the intact back of the building.
    "mce_collapse": [
        ("apartment",         "F1"),
        ("commercial",        "F2"),
        ("apartment",         "F2"),
        ("highrise_step",     "F3",  6),
        ("block_residential", "F4", 14),
        # the pair. SAME style, SAME origin, SAME venting elevations — the
        # only difference between them is which collapse recipe ran.
        ("commercial_mid",    "F5",   1, "S,E"),
        ("commercial_mid",    "F5c",  1, "S,E"),
    ],
}

SEED = int(_env("UF_SEED", "7"))
SETTLE_STEPS = int(_env("SETTLE_STEPS", "1600"))

# CONVEX DECOMPOSITION THRESHOLD for the settle. A convex hull cannot
# represent a re-entrant profile, so a fractured cornice or parapet rests on
# a hull that is not where its visible surface is and depenetration pushes it
# through the floor — 147 bodies finished BELOW GRADE on this bench and the
# clamp put them back, which hid the bug rather than fixing it.
#
# MEASURED on `SM_build_b_mod_top_trim`, the `dw_terrace` cornice module these
# fragments come from (bare-USD probe, 2026-08-29): convex hull 5.567 m3
# against a true mesh volume of 1.986 m3 — the hull is 2.8x the solid, with a
# genuine air gap through the middle of the section.
#
# 0.8 m, NOT the 2.5 m the wildfire archetypes use. The floating `dw_terrace`
# fragments measure 0.39-1.26 m on their bbox diagonal, clustering 0.75-1.0 m,
# so the house-archetype value would not have caught a single one of them.
# Set to 0 to disable.
SETTLE_DECOMP_M = float(_env("SETTLE_DECOMP_M", "0.8"))
KEEP_PHYSICS = _env("KEEP_PHYSICS", "0") not in ("0", "false")
SNAP_DIR = _env("SNAP_DIR", "")
FLOW = _env("UF_FLOW", "1") not in ("0", "false")
ORIGIN = _env("UF_ORIGIN", "")
SIDES = tuple(q.strip().upper()[:1] for q in _env("UF_SIDES", "").split(",")
              if q.strip()) or None

def _sides(spec):
    """`"S,E"` / `("S", "E")` / None -> a tuple of elevation letters or None."""
    if not spec:
        return None
    if isinstance(spec, str):
        spec = spec.replace("/", ",").split(",")
    out = tuple(q.strip().upper()[:1] for q in spec if str(q).strip())
    return out or None


_spec = _env("UF_BUILDINGS", "")
if _spec:
    # `<style>:<level>[:<origin>[:<sides>]]`, rows separated by `;` so that
    # the sides field can use `,` (`commercial_mid:F5c:1:S/E` works too — the
    # `/` form is there for a shell that would rather not quote).
    BUILDINGS = []
    for item in (_spec.split(";") if ";" in _spec else _spec.split(",")):
        if not item.strip():
            continue
        bits = [q.strip() for q in item.strip().split(":")]
        BUILDINGS.append((bits[0],
                          (bits[1] if len(bits) > 1 and bits[1] else "F3"),
                          int(bits[2]) if len(bits) > 2 and bits[2] else None,
                          bits[3] if len(bits) > 3 and bits[3] else None))
else:
    BUILDINGS = SETS[_env("UF_SET", "default")]
# normalise to (style, level, origin_or_None, sides_or_None). THE LEVEL IS
# NOT `.upper()`d any more: `F5c` is a real level name (`urban_fire.LEVELS`)
# and upper-casing it turns it into an unknown one, which the check below
# then rejects fourteen seconds into the launch.
BUILDINGS = [(b[0], b[1],
              b[2] if len(b) > 2 else None,
              _sides(b[3]) if len(b) > 3 else None) for b in BUILDINGS]


def build_ground_and_light(stage, span):
    """Pavement-grey ground and a low warm key.

    THE LIGHT IS PART OF THE DAMAGE HERE. Char is 0.15 on screen and a spall
    scar is 0.44; under a flat overhead key the whole elevation crushes to
    black and none of the plume structure survives. A low sun (25 deg) rakes
    the façade, so the tongues and the scars separate, and it is also what a
    drone flies in.
    """
    e = max(400.0, span * 1.4)
    ground = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/ground"))
    ground.CreatePointsAttr([Gf.Vec3f(-e, -e, 0.0), Gf.Vec3f(e, -e, 0.0),
                             Gf.Vec3f(e, e, 0.0), Gf.Vec3f(-e, e, 0.0)])
    ground.CreateFaceVertexCountsAttr([4])
    ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    ground.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    ground.CreateDisplayColorAttr([Gf.Vec3f(0.30, 0.30, 0.29)])
    ground.CreateExtentAttr([Gf.Vec3f(-e, -e, 0.0), Gf.Vec3f(e, e, 0.0)])
    try:
        mp = stage.DefinePrim(Sdf.Path("/World/Looks/pavement"))
        mp.GetReferences().AddReference(sg._join_asset_root(
            "airstack://scene_gen/assets/materials/megascans/Road_Asphalt.usda", ""))
        mp.Load()
        m = UsdShade.Material.Get(stage, "/World/Looks/pavement")
        if m:
            UsdShade.MaterialBindingAPI(ground.GetPrim()).Bind(m)
    except Exception as exc:
        print("[uf_bench] ground material unavailable: {0}".format(exc))
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/domeLight"))
    dome.CreateIntensityAttr(700.0)
    dome.CreateColorAttr(Gf.Vec3f(0.72, 0.76, 0.86))
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/keyLight"))
    key.CreateIntensityAttr(3200.0)
    key.CreateAngleAttr(0.9)
    key.CreateColorAttr(Gf.Vec3f(1.0, 0.94, 0.86))
    key.AddRotateXYZOp().Set(Gf.Vec3f(-25.0, 0.0, 28.0))


def main():
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/stage"))
    _, ssf = get_stage_meters_per_unit(stage)
    fracture.ensure_deps()
    fracture.ensure_vtk(verbose=True)
    t0 = time.time()

    problems = ub.check(verbose=False) + qf.check(verbose=False) \
        + uf.check(verbose=False)
    # ...and, only when the row actually asks for one, the PARTIAL-collapse
    # planner over real kit buildings. It is ~2 s host-side and it fails on
    # exactly the things that are invisible in a render until the very end:
    # a failure line under the fire origin, a windrow planned on the wrong
    # side of the wall, an elevation lost that was never alight.
    if any(lv == fcol.FIRE_LEVEL for _st, lv, _o, _sd in BUILDINGS):
        problems += fcol.check(verbose=False)
    if problems:
        raise RuntimeError("; ".join(problems))
    for st, lv, _o, _sd in BUILDINGS:
        if st not in ub.STYLES:
            raise RuntimeError("unknown style {0}".format(st))
        if lv not in uf.LEVELS:
            raise RuntimeError("unknown fire level {0}".format(lv))

    dims = {st: ub.footprint(ub.STYLES[st]) + (ub.height(ub.STYLES[st]),)
            for st, _lv, _o, _sd in BUILDINGS}
    big = max(max(w, d) for w, d, _h in dims.values())
    spacing = float(_env("UF_SPACING", "0") or 0) or max(60.0, 2.2 * big)
    span = spacing * (len(BUILDINGS) - 1) + 2.0 * big
    build_ground_and_light(stage, span)

    flow_root = None
    if FLOW:
        # The pool, not the USD attribute: `flowRender/renderSettings.maxBlocks`
        # does nothing on its own and Flow silently starves every emitter past
        # the first few, which reads as "the fire is only in a couple of
        # places". 0.14 m cells over a 500 m row is coarser than the wildfire
        # bench's 0.1 because the row is longer and the card is the same.
        # 0.09 m CELLS, NOT 0.14. A window flame is about a metre across, so
        # at 0.14 it is seven voxels wide and quantises into a lump — half of
        # why the first bench read as "individual fire balls". NVIDIA's
        # warehouse fire runs 0.05; the wildfire block runs 0.1 over a whole
        # suburb. This row is 550 m long but the fire is only ever on a few
        # façades, so the sparse grid stays affordable at 0.09.
        fx.setup_flow_stack(stage, density_cell_size_m=0.09, max_blocks=24576,
                            scene_scale_factor=ssf)
        flow_root = fx.FLOW_ROOT
        print("[uf_bench] flow stack up at {0}".format(flow_root))

    # 1) place every building
    x0 = -0.5 * spacing * (len(BUILDINGS) - 1)
    cols = []
    for i, (st, lv, org, sds) in enumerate(BUILDINGS):
        x, y = x0 + i * spacing, 0.0
        parent = "{0}/b{1}".format(PARENT, i)
        UsdGeom.Scope.Define(stage, Sdf.Path(parent))
        pls = ub.build_building(st, x, y, 0.0, random.Random(SEED + i))
        sg.apply_placements(stage, pls, parent, ssf)
        W, D, H = dims[st]
        cols.append(dict(i=i, style=st, level=lv, origin=org, sides=sds,
                         x=x, y=y,
                         parent=parent, pls=pls, W=W, D=D, H=H))
    n_glass = ub.apply_glass_tint(stage, [p for c in cols for p in c["pls"]])
    for _ in range(5):
        omni.kit.app.get_app().update()
    print("[uf_bench] {0} building(s), glass tint on {1}, spacing {2:.0f} m"
          .format(len(cols), n_glass, spacing))

    # 2) burn each one
    mats = uf.materials(stage, PARENT)
    cache = {}
    loose, static, vel = [], ["/World/ground"], {}
    for c in cols:
        rng = random.Random(SEED + 101 * c["i"])
        nrng = np.random.default_rng(SEED + 101 * c["i"])
        tb = time.time()
        res = uf.burn_building(
            stage, c["parent"], c["style"], c["pls"], c["x"], c["y"], 0.0,
            c["level"], rng, nrng, mats, "b{0}".format(c["i"]),
            flow_root=flow_root,
            origin=(int(ORIGIN) if ORIGIN else c["origin"]),
            # UF_SIDES is the blunt override for the whole row; a set row's
            # own fourth field is what points ONE building's fire (and, at
            # F5c, its collapsed elevation) at the cameras.
            sides=(SIDES or c["sides"]),
            mat_cache=cache)
        loose += res["loose"]
        static += res["static_extra"]
        vel.update(res["velocity"])
        c["notes"] = res["notes"]
        c["fire"] = res["fire"]
        print("[uf_bench] {0:<17} {1}  {2:4d} loose, {3:4d} static, "
              "{4:5d} authored  ({5:.0f} s)".format(
                  c["style"], c["level"], len(res["loose"]),
                  len(res["static_extra"]), len(res["authored"]),
                  time.time() - tb))
        for n in res["notes"]:
            print("[uf_bench]     " + n)
    for _ in range(10):
        omni.kit.app.get_app().update()

    # 3) settle — gravity only. Masonry/concrete density, not the fire path's
    #    420 kg/m3 timber: almost everything that moves here is a slab, a
    #    roof deck or a wall fragment.
    if loose:
        # `converge` + `quiet_steps`: a fire collapse drops a whole roof deck
        # and its plant from 17 m, which is a longer fall than the step cap
        # was sized for — the run kept baking two bodies mid-flight.
        #
        # `ccd` + `ground_plane_z` + `floor_z`: `/World/ground` (below) is
        # exactly the "four-vertex ground quad" `settle.py`'s own module
        # docstring names as tunnel-prone at speed — a real half-space plus
        # continuous collision detection is the documented fix, and this
        # bench never asked for either (measured: 669 rigid bodies, 1 STILL
        # MOVING at bake time, 454.69 m of horizontal "spread" — a body
        # cannot cover that distance under this settle's own max_speed cap
        # unless it lost contact with the world entirely; uf_bench_ref,
        # 2026-08-29). `floor_z=0.0` is the belt: anything that still ends
        # up under grade gets clamped back onto it before the bake.
        settle.run(stage, loose, static, steps=SETTLE_STEPS, kick=0.10,
                   rng=random.Random(SEED), bake_result=not KEEP_PHYSICS,
                   velocity_map=vel, density=1600.0, max_speed=6.0,
                   converge=True, max_steps=int(SETTLE_STEPS * 2.5),
                   quiet_steps=60, ccd=True, ground_plane_z=0.0,
                   floor_z=0.0,
                   decompose_larger_than=(SETTLE_DECOMP_M or None))
    for _ in range(10):
        omni.kit.app.get_app().update()

    # RE-ASSERT FRACTIONAL CUTOUT OPACITY, NOW THAT THE STAGE IS COMPOSED.
    # This is the SECOND of the two forms, and both are required:
    # `launch-generated-scene-with-drones` records it as "It has to be a
    # `SimulationApp` `extra_args` entry ... *and* re-asserted through
    # `carb.settings.get_settings().set_bool(...)` after the stage is
    # composed. THE STARTUP FORM ALONE DOES NOT SURVIVE COMPOSITION; THE CARB
    # FORM ALONE IS TOO LATE FOR STARTUP. Both launchers do both."
    #
    # Getting this wrong is invisible in the log and looks like an art bug:
    # the soot overlay and the glass deposits are fractional-cutout opacity,
    # so the renderer forces them to 1.0 and they become hard-edged stamps
    # instead of graded staining — "burn edges don't fade. Most of the
    # building just looks normal" (user, 2026-08-29). The same defect hid
    # every occupant inside a car for the people work, and made the
    # burnt-forest ground scar "draw nothing" before that.
    try:
        import carb
        _s = carb.settings.get_settings()
        for _k in ("/rtx/raytracing/fractionalCutoutOpacity",
                   "/rtx/pathtracing/fractionalCutoutOpacity"):
            _s.set_bool(_k, True)
        print("[uf_bench] fractionalCutoutOpacity re-asserted post-composition "
              "(raytracing={0}, pathtracing={1})".format(
                  _s.get("/rtx/raytracing/fractionalCutoutOpacity"),
                  _s.get("/rtx/pathtracing/fractionalCutoutOpacity")))
    except Exception as _exc:
        print("[uf_bench] WARNING: could not re-assert "
              "fractionalCutoutOpacity ({0}); the soot overlay and the glass "
              "deposits will render as hard cutouts".format(_exc))
    for _ in range(4):
        omni.kit.app.get_app().update()

    print("\n" + "=" * 76)
    print("URBAN FIRE BENCH")
    for c in cols:
        f = c["fire"]
        print("  x={0:+7.1f}  {1:<17} {2}  {3:>8}  {4:.0f} m  storeys {5}-{6} "
              "on {7}{8}".format(
                  c["x"], c["style"], c["level"],
                  qf.FAMILY_TYPE.get(ub.STYLES[c["style"]].get("family")),
                  c["H"], f["origin"], f["top"], "/".join(f["sides"]),
                  ", roof through" if f["roof"] else ""))
    print("  {0} loose bodies, {1:.0f} s".format(len(loose), time.time() - t0))
    print("=" * 76 + "\n")

    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            _sp = os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py")
            _spec2 = _ilu.spec_from_file_location("snapshots", _sp)
            _snaps = _ilu.module_from_spec(_spec2)
            _spec2.loader.exec_module(_snaps)
            os.makedirs(SNAP_DIR, exist_ok=True)
            # FLOW NEEDS TIME BEFORE IT IS PHOTOGRAPHED. The emitters inject
            # fuel per step, so a capture at t=0 is of an empty grid — the
            # car-occupants bench pays 240 updates for the same reason.
            if FLOW:
                timeline.play()
                for _ in range(300):
                    omni.kit.app.get_app().update()
            _snaps.overview(stage, (0.0, 0.0), span,
                            os.path.join(SNAP_DIR, "row.png"), ssf)
            tallest = max(c["H"] for c in cols)
            _snaps.place_camera(
                stage, (-0.20 * span, -0.72 * span, 0.34 * span + tallest),
                (0.0, 0.0, tallest * 0.28))
            _snaps.snapshot(os.path.join(SNAP_DIR, "row_obl.png"))
            import omni.kit.viewport.utility as vp
            for c in cols:
                name = "{0}_{1}_{2}".format(c["i"], c["style"], c["level"])
                x, y, W, D, H = c["x"], c["y"], c["W"], c["D"], c["H"]
                d = 1.5 * max(W, D, H * 0.8)
                f = c["fire"]
                # THE BURNING ELEVATION IS THE ONE TO PHOTOGRAPH, and it is
                # drawn per building — a fixed south camera shows a clean wall
                # on the half of the row whose fire vented north. `face` looks
                # square at the first burning side; `close` stands on the
                # pavement in front of it at eye height, which is the only
                # view that can tell a plume from a paint stripe.
                side = f["sides"][0]
                nx, ny = {"S": (0.0, -1.0), "N": (0.0, 1.0),
                          "E": (1.0, 0.0), "W": (-1.0, 0.0)}[side]
                back = (D if side in ("S", "N") else W) / 2.0
                face_w = W if side in ("S", "N") else D
                # STAND OFF BY THE LENS, NOT BY A GUESSED MULTIPLE OF H. At
                # 18 mm on the 20.955 mm aperture and a 1280x720 frame the
                # view covers 1.164 x distance horizontally and 0.655 x
                # vertically, so framing a 22 x 20 m elevation needs ~31 m,
                # not the 19 m that `0.95 * max(H, 24)` gave — the first
                # bench's `face` shots were all crops of the middle of a wall
                # (uf_smoke, 2026-08-28).
                # ...and on a TALL building, frame the FIRE, not the whole
                # tower. A 103 m skyscraper needs a 198 m standoff to fit, at
                # which range a stripe of blown-out bays is a few pixels
                # (uf_bench skyscraper_a, 2026-08-28). The subject is the
                # band: its own height plus a storey of clean wall either
                # side, which is what shows the contrast.
                band_z0 = f["origin"] * 3.2
                band_z1 = (f["top"] + 1) * 3.2
                subj_h = min(H, (band_z1 - band_z0) + 2 * 3.2)
                subj_z = min(H * 0.92, max(subj_h * 0.5,
                                           0.5 * (band_z0 + band_z1)))
                fit = max(face_w / 1.164, subj_h / 0.655) * 1.18 + back
                # centre the face view on the fire band, not on the building
                zb = 0.5 * (f["origin"] + f["top"] + 1) * 3.2
                views = {
                    "top": ((x, y, max(W, D) / 1.164 * 1.45 + H), (x, y, 0.0)),
                    "obl": ((x + nx * fit * 0.80 - ny * fit * 0.62,
                             y + ny * fit * 0.80 + nx * fit * 0.62,
                             0.45 * fit + 0.55 * subj_z),
                            (x, y, subj_z * 0.9)),
                    "face": ((x + nx * fit, y + ny * fit, subj_z),
                             (x, y, subj_z)),
                    # A PERSON ON THE PAVEMENT. Framed on the fire band, not
                    # on the whole building: this is the only view that can
                    # tell a soot tongue from a painted stripe.
                    "close": ((x + nx * (back + 16.0), y + ny * (back + 16.0),
                               2.0),
                              (x + nx * back * 0.2, y + ny * back * 0.2,
                               min(H * 0.75, max(5.0, zb)))),
                    "back": ((x - nx * fit, y - ny * fit, subj_z),
                             (x, y, subj_z)),
                }
                for vname, (eye, tgt) in views.items():
                    cpath = "/World/ReviewCams/b{0}_{1}".format(c["i"], vname)
                    cam = UsdGeom.Camera.Define(stage, Sdf.Path(cpath))
                    cam.GetHorizontalApertureAttr().Set(20.955)
                    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.5, 20000.0))
                    cam.GetFocalLengthAttr().Set(18.0)
                    xf = UsdGeom.Xformable(cam)
                    xf.ClearXformOpOrder()
                    xf.AddTranslateOp().Set(Gf.Vec3d(*eye))
                    xf.AddRotateXYZOp().Set(_snaps._look_at(eye, tgt))
                    vp.get_active_viewport().camera_path = cpath
                    _snaps.snapshot(os.path.join(
                        SNAP_DIR, "{0}_{1}.png".format(name, vname)))
            vp.get_active_viewport().camera_path = _snaps.CAM
            print("[uf_bench] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[uf_bench] snapshots FAILED: {0}".format(exc))

    print("URBAN FIRE BENCH DONE")
    keep = _env("KEEP_OPEN", "") == "1" or not _HEADLESS
    app = omni.kit.app.get_app()
    if keep:
        timeline.play()
        while simulation_app.is_running():
            app.update()
        timeline.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
