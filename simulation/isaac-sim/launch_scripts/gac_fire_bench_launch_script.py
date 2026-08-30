#!/usr/bin/env python
"""
GAC fire bench — the GreatAmericanCity (GAC) counterpart of
`urban_fire_bench_launch_script.py`: SIX merged GAC buildings in a row on
empty ground, each at a DIFFERENT fire level, each sliced into a kit with its
soot baked into its atlases BEFORE the slice, burned by the full ladder,
settled once and captured.

    ISAAC_SIM_HEADLESS=false GF_SEED=7 GF_FLOW=1 SETTLE_STEPS=1600 \
    KEEP_OPEN=1 SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/gac_fire \
    SOOT_SKIN_DIR=/isaac-sim/.nvidia-omniverse/logs/gac_fire/skins \
    PYTHONUNBUFFERED=1 PYTHONPATH="$ISAAC_SIM_PYTHONPATH" \
    /isaac-sim/python.sh \
    /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/gac_fire_bench_launch_script.py \
    --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts

WHY THIS EXISTS, AND WHY THE SOOT IS BAKED BEFORE THE SLICE
-------------------------------------------------------------
`urban_fire_bench_launch_script.py` is the reference bench for the fire
ladder, but every building on it is a KIT: a style assembled from its own
façade modules, each carrying its own small atlas, so `urban_fire`'s
per-piece soot bake (`_bind_soot` / `soot_bake`) is right — one baked map per
sooted module. A GAC building is the opposite shape: ONE merged mesh, ~14
material subsets, a few dozen textures shared by the whole façade, which
`gac_storey_slice` then cuts into hundreds of position-addressed pieces that
all still sample those same few atlases. Baking per PIECE on a GAC building
would write hundreds of copies of the same 2K maps for nothing; baking per
ATLAS, once, through the merged mesh's own UVs before it is cut, means the
hundred-plus pieces of one building share a dozen sooted textures, exactly
the way the untouched asset shares its clean ones.

That whole pipeline — measure the windows and storey grid on the MERGED
asset, plan the fire and its soot events from them, bake the skin into every
material's atlas ONCE, then slice and run the same `urban_fire` ladder the
kit row uses — is `disaster/gac_fire.py:burn_gac`. This bench exists to put
six different GAC buildings through six different severities so the ladder
can be judged on the pack that is NOT a hand-authored kit: the one every
downtown actually mostly draws from.

Env:
    GF_BUILDINGS    comma list of GAC asset names, in COLUMN order (default
                    SM_Building_02,SM_Building_24,SM_Building_01,
                    SM_Building_04,SM_Building_06_Small,SM_Building_09) —
                    sizes come from `scene_gen/_plans/gac_buildings.json`
    GF_LEVELS       fire levels across the columns, cycled if there are fewer
                    levels than buildings (default F1,F2,F3,F4,F5,F6)
    GF_SEED         rng seed (default 7); building i gets
                    `random.Random(SEED + 31*i)` /
                    `np.random.default_rng(SEED + 31*i)`
    GF_SPACING      metres between columns (default = 2.2 x the widest
                    building's plan WIDTH, min 90)
    GF_EXTRA_KIT    extra KIT columns after the GAC row, `style:level[:sides]`
                    entries (e.g. `commercial_mid:F5c:S,apartment:F5c:E`) —
                    ModernCityEnvironment kit buildings through
                    `urban_fire.burn_building`, so a partially collapsed MCE
                    building can stand in the same row as the GAC stock
    GF_FLOW         1 (default) authors the NVIDIA Flow stack and the flames
    GF_ORIGIN       force every fire to start at this storey (default: each
                    building draws its own, low-biased)
    GF_SIDES        force the burning elevations, e.g. `S` or `S,E`
                    (default: drawn per building)
    GF_BAKED_KITS   1 (default) uses a pre-baked kit
                    (`tools/bake_gac_kits.py`) where one exists for the asset,
                    falling back to a live `gac_storey_slice.slice_to_kit`;
                    0 always slices live
    SETTLE_STEPS    physics step ceiling (default 1600)
    SETTLE_DECOMP_M convex-decomposition threshold for the settle, metres
                    (default 0.8 — see `SETTLE_DECOMP_M` in
                    `urban_fire_bench_launch_script.py` for why 0.8 and not
                    the 2.5 the wildfire archetypes use; 0 disables it)
    KEEP_PHYSICS    1 leaves bodies live instead of baking
    SNAP_DIR        viewport captures, MUST be under
                    /isaac-sim/.nvidia-omniverse/logs/
    SOOT_SKIN_DIR   read by `disaster/urban_fire.py` itself (not this
                    launcher) — set to also dump each building's soot skin
                    as a PNG for review
    KEEP_OPEN       1 keeps the app up after the captures (headless default
                    is to exit)

`GSS_*` (the slicer's own knobs, e.g. `GSS_NO_LOCK`) pass straight through:
`gac_storey_slice.py` reads them from `os.environ` directly, so there is
nothing for this launcher to wire up.

THE SLICER IS SEQUENTIAL ON PURPOSE. It holds a machine-wide lock
(`gac_storey_slice`'s own `GSS_LOCK_PATH`) and is memory hungry — the six
buildings are therefore built one at a time in a plain loop, never overlapped,
and each one reports its own wall-clock time so a slow building is visible
rather than hidden inside one big number.
"""

import json
import os
import random
import sys
import time

from isaacsim import SimulationApp


def _env(name, default=""):
    """The container exports every launcher knob as an EMPTY STRING, so
    `os.environ.get(name, default)` never reaches its default — a numeric
    knob raises partway into the launch and a path knob silently becomes "".
    Treat empty as absent."""
    v = os.environ.get(name)
    return default if not (v or "").strip() else v.strip()


_HEADLESS = _env("ISAAC_SIM_HEADLESS", "false").lower() in ("1", "true", "yes")
# FRACTIONAL CUTOUT OPACITY — `extra_args`, NOT `carb.settings`, and BOTH
# forms are required (the startup flag does not survive stage composition;
# the carb form alone is too late for startup). `gac_fire.burn_gac` ends up
# authoring the same soot-mask / smoke-deposit materials
# `urban_fire_bench_launch_script.py` does (`wall_overlay`'s mask,
# `urban_fire._glass_pane`'s deposits, and here also the baked-atlas soot
# itself), all FRACTIONAL CUTOUT opacity, which RTX discards unless this is
# set — the overlay then renders as a hard binary stamp instead of graded
# staining. `disaster/ground.py:KIT_ARGS` is the source of truth; kept as a
# literal here because `scene_gen` is not on `sys.path` until after
# `SimulationApp` is constructed.
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
from detail import gac_slice as gsl                            # noqa: E402
from disaster import fire as fx                                # noqa: E402
from disaster import fracture, settle                          # noqa: E402
from disaster import gac_fire as gcf                           # noqa: E402
from disaster import urban_fire as uf                          # noqa: E402
from detail import urban_building as ub                        # noqa: E402
from disaster import quake_flow as qf                          # noqa: E402

PARENT = "/World/stage/generated"

# The GAC building sizes, measured once and recorded in
# `scene_gen/_plans/gac_buildings.json` (`tools/gac_slice_probe.py`'s own
# output) — read here purely to size the column pitch BEFORE anything is
# placed; `gac_fire.burn_gac` measures the asset itself again once it is on
# the stage and that measurement, not this one, drives everything downstream.
_PLAN_PATH = os.path.join(_SCENE_GEN_DIR, "_plans", "gac_buildings.json")
with open(_PLAN_PATH) as _f:
    GAC_PLAN = {r["name"]: r for r in json.load(_f)}

BUILDINGS = [v.strip() for v in _env(
    "GF_BUILDINGS",
    "SM_Building_02,SM_Building_24,SM_Building_01,SM_Building_04,"
    "SM_Building_06_Small,SM_Building_09").split(",") if v.strip()]
LEVELS = [v.strip().upper() for v in _env(
    "GF_LEVELS", "F1,F2,F3,F4,F5,F6").split(",") if v.strip()]

SEED = int(_env("GF_SEED", "7"))
SETTLE_STEPS = int(_env("SETTLE_STEPS", "1600"))
# See `urban_fire_bench_launch_script.py`'s `SETTLE_DECOMP_M` for the
# measurement behind 0.8 m rather than the 2.5 m the wildfire archetypes use.
SETTLE_DECOMP_M = float(_env("SETTLE_DECOMP_M", "0.8"))
KEEP_PHYSICS = _env("KEEP_PHYSICS", "0") not in ("0", "false")
SNAP_DIR = _env("SNAP_DIR", "")
FLOW = _env("GF_FLOW", "1") not in ("0", "false", "no")
# EXTRA KIT COLUMNS after the GAC row: `style:level[:sides]` entries, e.g.
# `GF_EXTRA_KIT=commercial_mid:F5c:S,apartment:F5c:E` — ModernCityEnvironment
# kit buildings put through `urban_fire.burn_building` exactly as
# `urban_fire_bench` does, so a partially collapsed MCE building can stand in
# the same row as the GAC stock ("to that bench add 2 diff partially
# collapsed MCE building", user 2026-08-30).
EXTRA_KIT = []
for _ent in [v.strip() for v in _env("GF_EXTRA_KIT", "").split(",") if v.strip()]:
    _parts = _ent.split(":")
    EXTRA_KIT.append((_parts[0], _parts[1] if len(_parts) > 1 else "F5",
                      tuple(q.strip().upper()[:1] for q in _parts[2].split("/")
                            if q.strip()) if len(_parts) > 2 else None))
ORIGIN = _env("GF_ORIGIN", "")
SIDES = tuple(q.strip().upper()[:1] for q in _env("GF_SIDES", "").split(",")
              if q.strip()) or None
BAKED_KITS = _env("GF_BAKED_KITS", "1") not in ("0", "false", "no")


def build_ground_and_light(stage, span):
    """Pavement-grey ground and a low warm key — same seat as
    `urban_fire_bench_launch_script.py`'s: a low sun (25 deg) so char and
    scorch separate from the shadowed rest of the elevation instead of
    crushing to black under a flat overhead key."""
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
        print("[gf_bench] ground material unavailable: {0}".format(exc))
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

    problems = uf.check(verbose=False) + gsl.check(verbose=False)
    if problems:
        raise RuntimeError("; ".join(problems))
    for name in BUILDINGS:
        if name not in GAC_PLAN:
            raise RuntimeError(
                "unknown GAC building {0!r} (not in {1})".format(
                    name, _PLAN_PATH))
    for lv in LEVELS:
        if lv not in uf.LEVELS:
            raise RuntimeError("unknown fire level {0}".format(lv))

    # COLUMN PITCH FROM THE PLAN WIDTH ONLY (not width-or-depth): the row
    # faces the same way for every building, so it is the WIDTH that decides
    # how close two columns can stand.
    big = max(GAC_PLAN[nm]["W"] for nm in BUILDINGS)
    for _st, _lv, _sd in EXTRA_KIT:
        if _st not in ub.STYLES:
            raise RuntimeError("unknown kit style {0!r} in GF_EXTRA_KIT".format(_st))
        big = max(big, max(ub.footprint(ub.STYLES[_st])))
    n_cols = len(BUILDINGS) + len(EXTRA_KIT)
    spacing = float(_env("GF_SPACING", "0") or 0) or max(90.0, 2.2 * big)
    span = spacing * (n_cols - 1) + 2.0 * big
    build_ground_and_light(stage, span)

    flow_root = None
    if FLOW:
        fx.setup_flow_stack(stage, density_cell_size_m=0.30, max_blocks=12288,
                            scene_scale_factor=ssf)
        flow_root = fx.FLOW_ROOT
        print("[gf_bench] flow stack up at {0}".format(flow_root))

    mats = uf.materials(stage, PARENT)
    mat_cache = {}
    loose, static, vel = [], ["/World/ground"], {}
    x0 = -0.5 * spacing * (n_cols - 1)
    cols = []
    for i, name in enumerate(BUILDINGS):
        x, y = x0 + i * spacing, 0.0
        level = LEVELS[i % len(LEVELS)]
        cell = "{0}/g{1}".format(PARENT, i)
        cxf = UsdGeom.Xform.Define(stage, Sdf.Path(cell))
        cxf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
        rng = random.Random(SEED + 31 * i)
        nrng = np.random.default_rng(SEED + 31 * i)
        tag = "g{0}".format(i)
        tb = time.time()
        try:
            bctx = gcf.burn_gac(
                stage, cell, name, level, rng, nrng, mats, tag,
                flow_root=flow_root, mat_cache=mat_cache, ssf=ssf,
                origin=(int(ORIGIN) if ORIGIN else None), sides=SIDES,
                use_baked_kit=BAKED_KITS, verbose=True)
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[gf_bench] FAILED {0} ({1}): {2}".format(name, level, exc))
            stage.RemovePrim(Sdf.Path(cell))
            continue
        dt = time.time() - tb
        loose += bctx["loose"]
        static += bctx["static_extra"]
        vel.update(bctx["velocity"])
        gac = bctx["gac"]
        fire_note = next((n for n in bctx["notes"]
                          if n.startswith("fire events:")), "")
        H = float(gac["grid"].get("H")
                  or (gac["mass"]["top"] - gac["mass"]["z0"]))
        cols.append(dict(i=i, name=name, level=level, x=x, y=y,
                         W=float(gac["mass"]["W"]), D=float(gac["mass"]["D"]),
                         H=H, storeys=len(gac["mass"]["levels"]),
                         pieces=gac["n_pieces"], atlases=gac["n_atlases"],
                         fire_note=fire_note))
        print("[gf_bench] {0:<20} {1}  {2:4d} loose, {3:4d} static, "
              "{4:5d} authored  ({5:.0f} s)".format(
                  name, level, len(bctx["loose"]), len(bctx["static_extra"]),
                  len(bctx["authored"]), dt))
        for n in bctx["notes"]:
            print("[gf_bench]     " + n)
        for _ in range(2):
            omni.kit.app.get_app().update()

    # -- the extra KIT columns (MCE styles), `urban_fire_bench`'s own path ----
    for j, (style, level, sides) in enumerate(EXTRA_KIT):
        i = len(BUILDINGS) + j
        x, y = x0 + i * spacing, 0.0
        cell = "{0}/k{1}".format(PARENT, j)
        cxf = UsdGeom.Xform.Define(stage, Sdf.Path(cell))
        cxf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
        rng = random.Random(SEED + 31 * i)
        nrng = np.random.default_rng(SEED + 31 * i)
        tag = "k{0}".format(j)
        tb = time.time()
        try:
            # BUILD INTO A CHILD, NOT THE CELL: `apply_placements` turns its
            # parent into a Scope, which is not Xformable, and the cell's
            # translate is then silently dropped (mce_fire_launch_script's
            # own note)
            pls = ub.build_building(style, 0.0, 0.0, 0.0,
                                    random.Random(SEED + 7 * i))
            sg.apply_placements(stage, pls, cell + "/parts", ssf)
            ub.apply_glass_tint(stage, pls)
            specs = qf._mass_specs(style, 0.0, 0.0, 0.0)
            main_spec = max(specs, key=lambda m: len(m["levels"]))
            n_st = max(1, len(main_spec["levels"]))
            origin = (int(ORIGIN) if ORIGIN
                      else max(0, min(n_st - 1, int(round(0.25 * (n_st - 1))))))
            bctx = uf.burn_building(stage, cell, style, pls, 0.0, 0.0, 0.0,
                                    level, rng, nrng, mats, tag,
                                    flow_root=flow_root, origin=origin,
                                    sides=sides, mat_cache=mat_cache)
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[gf_bench] FAILED kit {0} ({1}): {2}".format(style, level, exc))
            stage.RemovePrim(Sdf.Path(cell))
            continue
        dt = time.time() - tb
        loose += bctx["loose"]
        static += bctx["static_extra"]
        vel.update(bctx["velocity"])
        fire_note = next((n for n in bctx["notes"]
                          if n.startswith("fire events:")), "")
        W_, D_ = ub.footprint(ub.STYLES[style])
        H_ = max(m["top"] for m in specs)
        cols.append(dict(i=i, name="kit:" + style, level=level, x=x, y=y,
                         W=float(W_), D=float(D_), H=float(H_), storeys=n_st,
                         pieces=len(pls), atlases=0, fire_note=fire_note))
        print("[gf_bench] {0:<20} {1}  {2:4d} loose, {3:4d} static, "
              "{4:5d} authored  ({5:.0f} s)".format(
                  "kit:" + style, level, len(bctx["loose"]),
                  len(bctx["static_extra"]), len(bctx["authored"]), dt))
        for n in bctx["notes"]:
            print("[gf_bench]     " + n)
        for _ in range(2):
            omni.kit.app.get_app().update()
    for _ in range(10):
        omni.kit.app.get_app().update()

    # settle — gravity only, masonry/concrete density, exactly the call
    # `urban_fire_bench_launch_script.py` makes (see its own comments for
    # why `ccd` / `ground_plane_z` / `floor_z` / `decompose_larger_than` are
    # all needed rather than a plain `settle.run(stage, loose, static)`).
    if loose:
        settle.run(stage, loose, static, steps=SETTLE_STEPS, kick=0.10,
                   rng=random.Random(SEED), bake_result=not KEEP_PHYSICS,
                   velocity_map=vel, density=1600.0, max_speed=6.0,
                   converge=True, max_steps=int(SETTLE_STEPS * 2.5),
                   quiet_steps=60, ccd=True, ground_plane_z=0.0,
                   floor_z=0.0,
                   decompose_larger_than=(SETTLE_DECOMP_M or None))
    for _ in range(10):
        omni.kit.app.get_app().update()

    # RE-ASSERT FRACTIONAL CUTOUT OPACITY, NOW THAT THE STAGE IS COMPOSED —
    # the second of the two required forms; see `KIT_ARGS` above and
    # `urban_fire_bench_launch_script.py`'s longer note on why both are
    # needed and neither alone is.
    try:
        import carb
        _s = carb.settings.get_settings()
        for _k in ("/rtx/raytracing/fractionalCutoutOpacity",
                   "/rtx/pathtracing/fractionalCutoutOpacity"):
            _s.set_bool(_k, True)
        print("[gf_bench] fractionalCutoutOpacity re-asserted post-composition "
              "(raytracing={0}, pathtracing={1})".format(
                  _s.get("/rtx/raytracing/fractionalCutoutOpacity"),
                  _s.get("/rtx/pathtracing/fractionalCutoutOpacity")))
    except Exception as _exc:
        print("[gf_bench] WARNING: could not re-assert "
              "fractionalCutoutOpacity ({0}); the soot overlay and the glass "
              "deposits will render as hard cutouts".format(_exc))
    for _ in range(4):
        omni.kit.app.get_app().update()

    print("\n" + "=" * 78)
    print("GAC FIRE BENCH")
    for c in cols:
        print("  x={0:+7.1f}  {1:<20} {2}  storeys {3:2d}  pieces {4:4d}  "
              "sooted atlases {5:2d}  {6}".format(
                  c["x"], c["name"], c["level"], c["storeys"], c["pieces"],
                  c["atlases"], c["fire_note"]))
    print("  {0} loose bodies, {1:.0f} s".format(len(loose), time.time() - t0))
    print("=" * 78 + "\n")

    if SNAP_DIR and cols:
        try:
            import importlib.util as _ilu
            _sp = os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py")
            _spec2 = _ilu.spec_from_file_location("snapshots", _sp)
            _snaps = _ilu.module_from_spec(_spec2)
            _spec2.loader.exec_module(_snaps)
            os.makedirs(SNAP_DIR, exist_ok=True)
            # FLOW NEEDS TIME BEFORE IT IS PHOTOGRAPHED — the emitters inject
            # fuel per step, so a capture at t=0 is of an empty grid.
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
            for c in cols:
                name = "{0}_{1}_{2}".format(c["i"], c["name"], c["level"])
                # TOP-VIEW HEIGHT FROM THE BUILDING'S OWN MEASURED SIZE, not
                # a row-wide constant — the same `max(W, D) / 1.164 * 1.45 +
                # H` `urban_fire_bench_launch_script.py` uses for its own
                # per-building "top" camera at an 18 mm lens (0.5 x
                # horizontal FOV = 1.164, so this is the standoff that fits
                # the footprint and clears the roof).
                top_h = max(c["W"], c["D"]) / 1.164 * 1.45 + c["H"]
                obl_dist = max(50.0, 1.3 * max(c["W"], c["D"], c["H"]))
                obl_h = max(18.0, 0.4 * c["H"])
                _snaps.views_around(stage, {name: (c["x"], c["y"])}, SNAP_DIR,
                                    ssf, top_h=top_h, obl_dist=obl_dist,
                                    obl_h=obl_h)
            print("[gf_bench] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[gf_bench] snapshots FAILED: {0}".format(exc))

    print("GAC FIRE BENCH DONE")
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
