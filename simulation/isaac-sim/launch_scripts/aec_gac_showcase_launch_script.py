#!/usr/bin/env python
"""aec_gac_showcase_launch_script.py -- the AEC brownstone rows burnt LIVE
at four levels, side by side with two GAC per-building BAKES, for one review.

    AEC_ROWS=Reference_Brownstone2Row:F2,Reference_Brownstone5Row:F3,\
Reference_Brownstone8Row:F4,Reference_Brownstone10Row:F5 \
    SHOW_GAC=/isaac-sim/.cache/fire_bakes/gac_SM_Building_02_F3_s7.usd,\
/isaac-sim/.cache/fire_bakes/gac_SM_Building_06_Small_F5_s38.usd \
    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/aec_show1 KEEP_OPEN=1 \
    ISAAC_SIM_HEADLESS=false PYTHONPATH="$ISAAC_SIM_PYTHONPATH" \
    /isaac-sim/python.sh .../aec_gac_showcase_launch_script.py \
        --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts \
        --/rtx/raytracing/fractionalCutoutOpacity=true \
        --/rtx/pathtracing/fractionalCutoutOpacity=true

WHY (user, 2026-09-02, after the 5Row F3 render: "yes this is good"): *"Now
spawn all 4 versions of the brownstones at different damage levels. Also
spawn 2 GAC buildings at diff damage levels since from what I could tell the
way that the materials are sourced/fallback changed for that too."*

Two different mechanisms in one scene, on purpose:

* the brownstone rows are referenced RAW and burnt in place by
  `disaster/aec_burn.py` (seconds per row, the MDL brick untouched under a
  conformal soot layer, interiors charred) -- see that module's docstring;
* the GAC buildings are the per-building BAKES `scene_gen/tools/fire_bake.sh`
  writes (slice + burn + settle + `rehome_for_export` + root-layer export),
  referenced as static geometry -- exactly what the city assembly does, so
  what renders here is what the dataset ships. No Flow is re-placed: this
  is a MATERIAL review, not a smoke one.

Columns run along +X at `SHOW_PITCH` m; every brownstone row's street front
faces -X, so `<col>_front.png` is the street side. `AEC_ROWS` entries are
`asset:level[:units[:seed]]` (`units` as `2-3`; empty = the middle two).
"""
import os
import sys

from isaacsim import SimulationApp  # noqa: E402

_HEADLESS = (os.environ.get("ISAAC_SIM_HEADLESS") or "false").strip().lower() in (
    "1", "true", "yes")
KIT_ARGS = ["--/rtx/raytracing/fractionalCutoutOpacity=true",
            "--/rtx/pathtracing/fractionalCutoutOpacity=true"]
simulation_app = SimulationApp(launch_config={"headless": _HEADLESS,
                                              "extra_args": KIT_ARGS})

import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux  # noqa: E402

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")


def _env(name, default=""):
    v = os.environ.get(name)
    return default if not (v or "").strip() else v.strip()


AEC_DIR = ("/isaac-sim/AirStack/scene_gen/assets/aec/brownstone/Assets/"
           "Create_Brownstone02/")
AEC_ROWS = _env("AEC_ROWS", "Reference_Brownstone2Row:F2,Reference_Brownstone5Row:F3,"
                            "Reference_Brownstone8Row:F4,Reference_Brownstone10Row:F5")
SHOW_GAC = [p for p in _env("SHOW_GAC", "").split(",") if p.strip()]
PITCH = float(_env("SHOW_PITCH", "100"))   # a 67 m row's front camera needs the room
AEC_SEED = int(_env("AEC_SEED", "7"))
AEC_BURN_OUT = _env("AEC_BURN_OUT", "/isaac-sim/.cache/aec_burn_tex")
SNAP_DIR = _env("SNAP_DIR", "")
KEEP_OPEN = _env("KEEP_OPEN", "0") == "1"
SHOW_FLOW = _env("SHOW_FLOW", "1") not in ("0", "false", "no")
#: 0.14 m cells / 16384 blocks: at 0.12 / 24576 the Kit log carried one
#: "Out of GPU memory allocating resource 'flow'" on a 16 GB card holding
#: the GUI, six columns and 667 settled bodies (aec_show4) -- the flames
#: still rendered, but that line is the one that ends in a scene with no
#: smoke and every count looking right
FLOW_CELL_M = float(_env("FLOW_CELL_M", "0.14"))
FLOW_MAX_BLOCKS = int(_env("FLOW_MAX_BLOCKS", "16384"))
#: frames of simulated time before the captures, so the smoke has risen
FLOW_WARMUP_FRAMES = int(_env("FLOW_WARMUP_FRAMES", "240"))


def _auto_scale(usd):
    try:
        s = Usd.Stage.Open(usd)
        if s:
            mpu = float(UsdGeom.GetStageMetersPerUnit(s))
            if 1e-6 < mpu <= 1e3:
                return mpu
    except Exception:
        pass
    return 1.0


def _ref(stage, path, usd, x, y, scale=1.0):
    prim = stage.DefinePrim(Sdf.Path(path), "Xform")
    if not prim.GetReferences().AddReference(usd):
        print("[show] *** FAILED to reference {0}".format(usd))
        return None
    prim.Load()
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(float(x), float(y), 0.0))
    if abs(scale - 1.0) > 1e-9:
        xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    return prim


def _parse_row(spec):
    f = spec.split(":")
    name, level = f[0], (f[1] if len(f) > 1 and f[1] else "F3")
    units = None
    if len(f) > 2 and f[2]:
        units = tuple(int(q) for q in f[2].replace("-", ",").split(",") if q)
    seed = int(f[3]) if len(f) > 3 and f[3] else None
    return name, level, units, seed


def main():
    from disaster import aec_burn

    usd_ctx = omni.usd.get_context()
    usd_ctx.new_stage()
    stage = usd_ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.DefinePrim("/World", "Xform")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/domeLight"))
    dome.CreateIntensityAttr(1200.0)
    dome.CreateColorAttr(Gf.Vec3f(0.55, 0.72, 0.93))
    sun = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/keyLight"))
    sun.CreateIntensityAttr(3500.0)
    sun.CreateAngleAttr(0.8)
    UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-55.0, 0.0, 135.0))
    ground = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/ground"))
    s = 600.0
    ground.CreatePointsAttr([(-s, -s, 0), (s, -s, 0), (s, s, 0), (-s, s, 0)])
    ground.CreateFaceVertexCountsAttr([4])
    ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    ground.CreateDisplayColorAttr([(0.32, 0.32, 0.33)])

    # FIRE AND SMOKE (user, 2026-09-02: "also add fire and smoke"): the Flow
    # stack first, the emitters per row from the same events as the soot
    # (`aec_burn.flames_row` -> `urban_fire.r_flames`). 0.12 m cells: four
    # rows of window flames on one 16 GB card that also carries the GUI;
    # the pool is a carb setting, and past it Flow renders nothing while
    # every count looks right (build-urban-fire-scenes skill).
    flow_root = None
    if SHOW_FLOW:
        try:
            from disaster import fire as fx
            fx.setup_flow_stack(stage, density_cell_size_m=FLOW_CELL_M,
                                max_blocks=FLOW_MAX_BLOCKS, scene_scale_factor=1.0)
            flow_root = fx.FLOW_ROOT
            print("[show] flow stack up at {0} ({1} m cells, {2} blocks)".format(
                flow_root, FLOW_CELL_M, FLOW_MAX_BLOCKS))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[show] *** flow stack FAILED: {0}".format(exc))

    cols = []                      # (name, prim path, plan or None)
    col = 0
    loose, static, vel = [], [], {}
    for spec in [q for q in AEC_ROWS.split(",") if q.strip()]:
        name, level, units, seed = _parse_row(spec)
        usd = AEC_DIR + name + ".usd"
        path = "/World/aec{0}_{1}".format(col, level)
        x = col * PITCH
        print("\n[show] column {0} at x={1:.0f}: {2} {3} units={4}".format(
            col, x, name, level, units or "default"))
        prim = _ref(stage, path, usd, x, 0.0, _auto_scale(usd))
        plan = None
        if prim is not None and level.upper() not in ("F0", "NONE"):
            try:
                _m, plan, st_ = aec_burn.burn_row(
                    stage, path, level=level, units=units,
                    seed=AEC_SEED if seed is None else seed, out_dir=AEC_BURN_OUT,
                    flow_root=flow_root)
                loose += list(st_.get("damage_loose") or [])
                static += list(st_.get("damage_static") or [])
                vel.update(st_.get("damage_velocity") or {})
            except Exception as exc:
                import traceback
                traceback.print_exc()
                print("[show] *** burn_row FAILED on {0}: {1}".format(name, exc))
        cols.append((path.rsplit("/", 1)[-1], path, plan))
        col += 1
        for _ in range(5):
            omni.kit.app.get_app().update()

    # THE COLLAPSE IS PHYSICS. Floors, ceilings, roof deck and plant, stairs
    # and (at F5) the lost wall's strips and bricks are rigid-body candidates
    # in each row's `_debris` scope; the shell, the origin floor, the stoop
    # and the rubble mound are the statics they land on. Same settle the
    # urban-fire bench runs (`urban_fire_bench_launch_script`): gravity,
    # a small kick, CCD against a real ground half-space, a quiet phase so
    # nothing is baked mid-flight.
    if loose:
        import random
        from disaster import settle
        # the bake driver's budget, not the bench's: at 700 + 200 quiet the
        # settle stalled with 14 wall strips frozen in mid-air in front of
        # the facades (aec_show4, 2026-09-02)
        SETTLE_STEPS = int(_env("SETTLE_STEPS", "1400"))
        SETTLE_QUIET = int(_env("SETTLE_QUIET", "400"))
        print("\n[show] settle: {0} loose, {1} static, {2} with velocity".format(
            len(loose), len(static), len(vel)))
        try:
            settle.run(stage, loose, static, steps=SETTLE_STEPS, kick=0.10,
                       rng=random.Random(AEC_SEED), bake_result=True,
                       velocity_map=vel, density=900.0, max_speed=6.0,
                       converge=True, max_steps=int(SETTLE_STEPS * 3.0),
                       quiet_steps=SETTLE_QUIET, stall_chunks=6, ccd=True,
                       ground_plane_z=0.0,
                       floor_z=0.0, decompose_larger_than=0.8)
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[show] *** settle FAILED: {0}".format(exc))
        for _ in range(10):
            omni.kit.app.get_app().update()

    for bake in SHOW_GAC:
        if not os.path.isfile(bake):
            print("[show] *** GAC bake missing, skipped: {0}".format(bake))
            continue
        stem = os.path.basename(bake).rsplit(".", 1)[0]
        path = "/World/{0}".format(stem.replace("-", "_"))
        x = col * PITCH
        print("\n[show] column {0} at x={1:.0f}: bake {2}".format(col, x, bake))
        # a bake is metres with /World as defaultPrim, built at the origin
        _ref(stage, path, bake, x, 0.0, 1.0)
        cols.append((stem, path, None))
        col += 1

    # both forms of the cutout flag (build-urban-fire-scenes skill, bug 4)
    try:
        import carb.settings
        _cs = carb.settings.get_settings()
        _cs.set_bool("/rtx/raytracing/fractionalCutoutOpacity", True)
        _cs.set_bool("/rtx/pathtracing/fractionalCutoutOpacity", True)
    except Exception as exc:
        print("[show] fractionalCutoutOpacity re-assert FAILED: {0}".format(exc))

    for _ in range(60):
        omni.kit.app.get_app().update()

    # FLOW NEEDS TIME BEFORE IT IS PHOTOGRAPHED: the emitters inject per
    # step, so a capture at t=0 is of an empty grid (car_occupants bench).
    # The bodies are baked (rigid bodies disabled), so play moves nothing.
    if flow_root is not None:
        try:
            omni.timeline.get_timeline_interface().play()
            for _ in range(FLOW_WARMUP_FRAMES):
                omni.kit.app.get_app().update()
            print("[show] flow warmed up for {0} frame(s)".format(FLOW_WARMUP_FRAMES))
        except Exception as exc:
            print("[show] flow warm-up FAILED: {0}".format(exc))

    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    ext = {}
    for name, path, plan in cols:
        p = stage.GetPrimAtPath(path)
        r = bc.ComputeWorldBound(p).ComputeAlignedRange() if p else None
        if r is None or r.IsEmpty():
            print("[show] {0}: EMPTY (did not compose)".format(name))
            continue
        mn, mx = r.GetMin(), r.GetMax()
        ext[name] = (mn, mx)
        print("[show] {0}: {1:.1f} x {2:.1f} x {3:.1f} m at x {4:.0f}..{5:.0f}".format(
            name, mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2], mn[0], mx[0]))

    if SNAP_DIR:
        try:
            import importlib.util as ilu
            sp = os.path.join("/isaac-sim/AirStack/simulation/isaac-sim",
                              "utils", "snapshots.py")
            spec = ilu.spec_from_file_location("snapshots", sp)
            snaps = ilu.module_from_spec(spec)
            spec.loader.exec_module(snaps)
            os.makedirs(SNAP_DIR, exist_ok=True)
            if ext:
                xs0 = min(v[0][0] for v in ext.values()); xs1 = max(v[1][0] for v in ext.values())
                ys0 = min(v[0][1] for v in ext.values()); ys1 = max(v[1][1] for v in ext.values())
                span = max(xs1 - xs0, ys1 - ys0) + 40.0
                snaps.overview(stage, (0.5 * (xs0 + xs1), 0.5 * (ys0 + ys1)), span,
                               os.path.join(SNAP_DIR, "row.png"), 1.0)
                # the street side of the whole row: a three-quarter view from
                # the front-left corner, close enough that the nearest rows
                # fill the frame and the far ones read behind them
                cx, cy = 0.5 * (xs0 + xs1), 0.5 * (ys0 + ys1)
                snaps.place_camera(stage, (xs0 - 40.0, ys0 - 55.0, 26.0),
                                   (xs0 + 0.35 * (xs1 - xs0), cy, 5.0))
                snaps.snapshot(os.path.join(SNAP_DIR, "row_street.png"))
            for name, path, plan in cols:
                if name not in ext:
                    continue
                mn, mx = ext[name]
                cx, cy = 0.5 * (mn[0] + mx[0]), 0.5 * (mn[1] + mx[1])
                H = mx[2] - mn[2]
                # never further back than the neighbouring column (the
                # camera was standing inside it at a 60 m pitch)
                d = min(0.95 * max(mx[1] - mn[1], mx[0] - mn[0], 1.4 * H), PITCH - 30.0)
                h = max(10.0, 0.45 * H)
                snaps.place_camera(stage, (mn[0] - d, cy, h), (cx, cy, 0.4 * (mx[2] - mn[2])))
                snaps.snapshot(os.path.join(SNAP_DIR, "{0}_front.png".format(name)))
                snaps.place_camera(stage, (mx[0] + d, cy, h), (cx, cy, 0.4 * (mx[2] - mn[2])))
                snaps.snapshot(os.path.join(SNAP_DIR, "{0}_rear.png".format(name)))
                if plan is not None:
                    by = plan["m"]["cy"]
                    snaps.place_camera(stage, (mn[0] - 20.0, by - 9.0, 8.0), (mn[0] + 4.0, by, 5.0))
                    snaps.snapshot(os.path.join(SNAP_DIR, "{0}_front_close.png".format(name)))
                    snaps.place_camera(stage, (mx[0] + 20.0, by + 9.0, 8.0), (mx[0] - 4.0, by, 5.0))
                    snaps.snapshot(os.path.join(SNAP_DIR, "{0}_rear_close.png".format(name)))
                # the plumb camera must clear the ROOF: a 62 m GAC tower under
                # a 55 m top view rendered solid black (aec_show2)
                top_h = max(55.0, 1.0 * max(mx[1] - mn[1], mx[0] - mn[0]), H + 40.0)
                snaps.views_around(stage, {name: (cx, cy)}, SNAP_DIR, 1.0,
                                   top_h=top_h, obl_dist=max(45.0, 0.8 * top_h),
                                   obl_h=max(18.0, 0.5 * H + 10.0),
                                   azimuth_deg=200.0, aim_h=0.4 * H)
            print("[show] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[show] snapshots FAILED: {0}".format(exc))

    print("\n==========================================================")
    print("AEC GAC SHOWCASE DONE: {0} column(s)".format(len(cols)))
    print("==========================================================")
    if KEEP_OPEN:
        while simulation_app.is_running():
            simulation_app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
