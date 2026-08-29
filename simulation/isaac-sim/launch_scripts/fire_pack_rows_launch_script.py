#!/usr/bin/env python
"""
The urban fire ladder on THREE ASSET PACKS, one row each, for comparison.

    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/fire_rows \
    ISAAC_SIM_SCRIPT_NAME=fire_pack_rows_launch_script.py airstack up isaac-sim

Row 0  KIT          `bld_*` styles, i.e. the ModernCityEnvironment01 /
                    Downtown_West / CivilianArea façade kit. The reference:
                    `urban_building.build_building` assembles the style from
                    its pieces and the full ladder runs on the element table.
Row 1  GAC          GreatAmericanCity, a single merged mesh, SLICED into
                    kit-shaped pieces by `detail/gac_slice.py` and then put
                    through the SAME ladder. This is the new thing.
Row 2  BROWNSTONE   AEC brownstones. Already 307 meshes / 491 instance proxies
                    (`tools/pack_structure_probe.py`), so they are a bag of
                    parts rather than a monolith — they need DE-INSTANCING,
                    not slicing, and that is what this row tests.

Every row runs the same severities across its columns, so a column is a
like-for-like comparison of the packs at one point on the ladder.

Muyang DownTown is deliberately absent: it is the one pack with no route to
credible damage (one merged mesh, 756-4,643 points, no glass subset, windows
painted into the texture), and `disaster/kit_substitute.UNBURNABLE` gates it
out of fire everywhere.

Env:
    FR_LEVELS   severities across the columns (default F1,F2,F3,F4,F5)
    FR_KIT      kit styles for row 0 (default: one per column, mixed families)
    FR_GAC      GAC assets for row 1
    FR_AEC      brownstone assets for row 2
    FR_COL_M    column pitch, m (default 95)
    FR_ROW_M    row pitch, m (default 130)
    FR_SEED     (default 7)
    FR_FLOW     1 authors NVIDIA Flow and the flames (default 1)
    SETTLE_STEPS physics ceiling (default 1400)
    SNAP_DIR / KEEP_OPEN
"""

import math
import os
import random
import sys
import time

from isaacsim import SimulationApp


def _env(n, d=""):
    v = os.environ.get(n)
    return d if not (v or "").strip() else v.strip()


_HEADLESS = _env("ISAAC_SIM_HEADLESS", "false").lower() in ("1", "true", "yes")
simulation_app = SimulationApp(launch_config={"headless": _HEADLESS})

from isaacsim.core.utils.extensions import enable_extension    # noqa: E402

enable_extension("omni.kit.window.script_editor")
enable_extension("omni.flowusd")

import omni.kit.app                                            # noqa: E402
import omni.timeline                                           # noqa: E402
import omni.usd                                                # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux                  # noqa: E402

_ISAAC = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".."))
_SG = os.path.normpath(os.path.join(_ISAAC, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC, "utils"))
sys.path.insert(0, _SG)

import numpy as np                                             # noqa: E402
import scene_generator as sg                                   # noqa: E402
from scene_prep import get_stage_meters_per_unit               # noqa: E402
from detail import gac_slice as gsl                            # noqa: E402
from detail import urban_building as ub                        # noqa: E402
from disaster import fracture, settle                          # noqa: E402
from disaster import quake_flow as qf                          # noqa: E402
from disaster import urban_fire as uf                          # noqa: E402

PARENT = "/World/bench"
NUC = "omniverse://airlab-nucleus.andrew.cmu.edu:443/"
GAC_DIR = NUC + "Projects/SEI-COA/GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/"
AEC_DIR = ("airstack://scene_gen/assets/aec/brownstone/Assets/"
           "Create_Brownstone02/")

LEVELS = [v.strip() for v in _env("FR_LEVELS", "F1,F2,F3,F4,F5").split(",") if v.strip()]
KIT = [v.strip() for v in _env(
    "FR_KIT", "brownstone_row,commercial,office_wide,apartment_tall,dw_terrace"
).split(",") if v.strip()]
GAC = [v.strip() for v in _env(
    "FR_GAC", "SM_Building_02,SM_Building_01,SM_Building_04,SM_Building_24,"
              "SM_Building_09").split(",") if v.strip()]
AEC = [v.strip() for v in _env(
    "FR_AEC", "Reference_Brownstone02,Reference_Brownstone2Row,"
              "Reference_Brownstone5Row,Reference_Brownstone6Row,"
              "Reference_Brownstone8Row").split(",") if v.strip()]
COL_M = float(_env("FR_COL_M", "95"))
ROW_M = float(_env("FR_ROW_M", "130"))
SEED = int(_env("FR_SEED", "7"))
FLOW = _env("FR_FLOW", "1") not in ("0", "false", "no")
SETTLE_STEPS = int(_env("SETTLE_STEPS", "1400"))
SNAP_DIR = _env("SNAP_DIR")


def ground_and_light(stage, w, d):
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/dome"))
    dome.CreateIntensityAttr(1000.0)
    dome.CreateColorAttr(Gf.Vec3f(0.78, 0.82, 0.90))
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/key"))
    key.CreateIntensityAttr(2800.0)
    key.CreateAngleAttr(0.8)
    key.CreateColorAttr(Gf.Vec3f(1.0, 0.95, 0.88))
    key.AddRotateXYZOp().Set(Gf.Vec3f(-44.0, 0.0, 30.0))
    g = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/ground"))
    hw, hd = w * 0.62, d * 0.62
    g.CreatePointsAttr([Gf.Vec3f(-hw, -hd, 0), Gf.Vec3f(hw, -hd, 0),
                        Gf.Vec3f(hw, hd, 0), Gf.Vec3f(-hw, hd, 0)])
    g.CreateFaceVertexCountsAttr([4])
    g.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    g.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    g.CreateDisplayColorAttr([Gf.Vec3f(0.29, 0.29, 0.28)])
    g.CreateExtentAttr([Gf.Vec3f(-hw, -hd, 0), Gf.Vec3f(hw, hd, 0)])


def place_asset(stage, cell, usd, scale=1.0):
    """Reference `usd` under `cell/src`, seated with its base at z=0 and its
    plan centred — the frame `gac_slice` and `quake_flow` both assume."""
    holder = cell + "/src"
    UsdGeom.Xform.Define(stage, Sdf.Path(holder))
    kid = stage.DefinePrim(Sdf.Path(holder + "/asset"))
    kid.GetReferences().AddReference(sg._join_asset_root(usd, ""))
    stage.Load(Sdf.Path(holder))
    xf = UsdGeom.Xformable(kid)
    xf.ClearXformOpOrder()
    tr = xf.AddTranslateOp()
    if abs(scale - 1.0) > 1e-9:
        xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    r = cache.ComputeWorldBound(stage.GetPrimAtPath(holder)).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    mn, mx = r.GetMin(), r.GetMax()
    cxf = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(cell))
    ox, oy, oz = cxf.ExtractTranslation()
    tr.Set(Gf.Vec3d(-(0.5 * (mn[0] + mx[0]) - ox),
                    -(0.5 * (mn[1] + mx[1]) - oy), -(mn[2] - oz)))
    return holder


def deinstance(stage, root_path):
    """Turn every instanceable prim under `root_path` into a real one.

    THE BROWNSTONES ARE ALREADY A BAG OF PARTS — 307 meshes behind 491
    INSTANCE PROXIES. That is why `monolith_damage.cut_shell` died on them
    with a bare `Tf.ErrorException`: it traverses instance proxies and then
    writes to the mesh, and USD forbids editing through a proxy. Nothing needs
    slicing here; the pieces exist. They just have to be made editable first.
    """
    n = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
        if prim.IsInstanceable():
            prim.SetInstanceable(False)
            n += 1
    return n


def parts_as_placements(stage, root_path, style, cell):
    """Every mesh under `root_path` as a kit-shaped placement.

    For a pack that is ALREADY assembled (the brownstones), the pieces are the
    meshes; only the (role, side, storey) tags have to be inferred, which is
    the same inference `gac_slice.cell_of` makes — from the piece's own
    position and extent rather than from a face normal.
    """
    xc = UsdGeom.XformCache()
    root = stage.GetPrimAtPath(root_path)
    root_inv = xc.GetLocalToWorldTransform(root).GetInverse()
    recs = []
    lo = hi = None
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        me = UsdGeom.Mesh(prim)
        pts = me.GetPointsAttr().Get()
        if pts is None or not len(pts):
            continue
        M = np.asarray(xc.GetLocalToWorldTransform(prim) * root_inv, dtype=float)
        P = np.asarray(pts, dtype=float)
        P = (np.c_[P, np.ones(len(P))] @ M)[:, :3]
        mn, mx = P.min(axis=0), P.max(axis=0)
        lo = mn if lo is None else np.minimum(lo, mn)
        hi = mx if hi is None else np.maximum(hi, mx)
        recs.append((str(prim.GetPath()), mn, mx))
    if not recs:
        return [], None
    H = float(hi[2] - lo[2])
    per = 3.4
    floors = gsl.lattice(per, float(lo[2]), float(lo[2]), float(hi[2]))
    g = {"storey_h": per, "storeys": floors,
         "bays": {s: {"pitch": 4.0} for s in gsl.SIDES},
         "confidence": 1.0, "bbox": (tuple(map(float, lo)), tuple(map(float, hi))),
         "W": float(hi[0] - lo[0]), "D": float(hi[1] - lo[1]), "H": H,
         "z0": float(lo[2])}
    out = []
    for path, mn, mx in recs:
        cen = 0.5 * (mn + mx)
        st = 0
        for i, z in enumerate(floors):
            if cen[2] >= z - 1e-6:
                st = i
        role = "roof" if mn[2] >= hi[2] - 0.6 * per else "wall"
        # a piece touching two elevations is a corner
        touch = sum(1 for a, b, c in ((0, lo[0], hi[0]), (1, lo[1], hi[1]),
                                      (0, lo[0], hi[0])) [:2]
                    if mn[a] <= b + 0.8 or mx[a] >= c - 0.8)
        if role == "wall" and touch >= 2:
            role = "corner"
        out.append({"category": "bld_{0}_{1}".format(style, gsl._sub_for(role)),
                    "usd": "aecpart://{0}".format(path.rsplit("/", 1)[-1]),
                    "x_m": float(cen[0]), "y_m": float(cen[1]),
                    "z_m": float(mn[2]), "yaw_deg": 0.0, "scale": 1.0,
                    "prim_path": path,
                    "_size": (float(mx[0] - mn[0]), float(mx[1] - mn[1]),
                              float(mx[2] - mn[2])),
                    "_role": role})
    return out, g


def burn(stage, cell, style, placements, level, rng, nrng, mats, tag,
         flow_root, cache):
    n_st = max(1, len(qf._mass_specs(style, 0.0, 0.0, 0.0)[0]["levels"]))
    origin = max(0, min(n_st - 1, int(round(0.25 * (n_st - 1)))))
    sides = ("S",) if level in ("F1", "F2") else ("S", "E")
    return uf.burn_building(stage, cell, style, placements, 0.0, 0.0, 0.0,
                            level, rng, nrng, mats, tag, flow_root=flow_root,
                            origin=origin, sides=sides, mat_cache=cache)


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
    UsdGeom.Scope.Define(stage, Sdf.Path(PARENT))
    _, ssf = get_stage_meters_per_unit(stage)
    t0 = time.time()
    fracture.ensure_deps()
    fracture.ensure_vtk(verbose=False)
    problems = uf.check(verbose=False) + gsl.check(verbose=False)
    if problems:
        raise RuntimeError("; ".join(problems))

    rows = [("kit", KIT), ("gac", GAC), ("brownstone", AEC)]
    ncol = max(len(r[1]) for r in rows)
    ground_and_light(stage, ncol * COL_M + 140.0, len(rows) * ROW_M + 160.0)
    flow_root = None
    if FLOW:
        from disaster import fire as fx
        fx.setup_flow_stack(stage, density_cell_size_m=0.30, max_blocks=12288,
                            scene_scale_factor=ssf)
        flow_root = fx.FLOW_ROOT
    mats = uf.materials(stage, PARENT)
    mat_cache = {}
    loose, static, vel = [], ["/World/ground"], {}
    cells, tally = [], {"kit": 0, "gac": 0, "brownstone": 0}

    for ri, (pack, names) in enumerate(rows):
        y = (ri - (len(rows) - 1) / 2.0) * ROW_M
        print("\n[fire_rows] === row {0}: {1} ===".format(ri, pack))
        for ci, nm in enumerate(names):
            x = (ci - (ncol - 1) / 2.0) * COL_M
            lvl = LEVELS[ci % len(LEVELS)]
            tag = "{0}{1}".format(pack[:3], ci)
            cell = "{0}/{1}_{2}".format(PARENT, pack, ci)
            cxf = UsdGeom.Xform.Define(stage, Sdf.Path(cell))
            cxf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
            rng = random.Random(SEED + 37 * ri + ci)
            nrng = np.random.default_rng(SEED + 37 * ri + ci)
            tb = time.time()
            try:
                if pack == "kit":
                    pls = ub.build_building(nm, 0.0, 0.0, 0.0,
                                            random.Random(SEED + ci))
                    sg.apply_placements(stage, pls, cell, ssf)
                    ub.apply_glass_tint(stage, pls)
                    style, H = nm, 0.0
                elif pack == "gac":
                    src = place_asset(stage, cell, GAC_DIR + nm + ".usd", 0.01)
                    if not src:
                        print("      skip {0}: nothing composed".format(nm))
                        continue
                    wins, bbox = gsl.window_centres(stage, src)
                    g = gsl.measure_grid(wins, bbox, name=nm)
                    if g.get("confidence", 0.0) < gsl.MIN_CONFIDENCE:
                        print("      skip {0}: grid not recovered "
                              "(confidence {1:.2f})".format(
                                  nm, g.get("confidence", 0.0)))
                        continue
                    style = "gac_" + nm
                    pls = gsl.slice_building(stage, src, cell + "/pieces", g,
                                             style)
                    gsl.register_style(g, style, pieces_of=pls)
                    # HIDE THE MERGED ORIGINAL, DO NOT DEACTIVATE IT.
                    # The asset's `Looks` scope lives INSIDE this subtree, and
                    # the sliced pieces bind to those materials — deactivating
                    # the source takes the materials with it and every sliced
                    # building renders WHITE, with its geometry and its damage
                    # both intact and no brick or glazing anywhere on it.
                    # Visibility only affects imageable prims, so a Material is
                    # untouched by it; this hides the original and keeps the
                    # bindings alive.
                    UsdGeom.Imageable(stage.GetPrimAtPath(src)).MakeInvisible()
                elif pack == "brownstone":
                    src = place_asset(stage, cell, AEC_DIR + nm + ".usd", 0.01)
                    if not src:
                        print("      skip {0}: nothing composed".format(nm))
                        continue
                    n_di = deinstance(stage, src)
                    style = "aec_" + nm
                    pls, g = parts_as_placements(stage, src, style, cell)
                    if not pls:
                        print("      skip {0}: no meshes".format(nm))
                        continue
                    gsl.register_style(g, style, pieces_of=pls)
                    print("      {0}: de-instanced {1} prim(s), {2} part(s)"
                          .format(nm, n_di, len(pls)))
                res = burn(stage, cell, style, pls, lvl, rng, nrng, mats, tag,
                           flow_root, mat_cache)
                loose += res["loose"]
                static += res["static_extra"]
                vel.update(res["velocity"])
                tally[pack] += 1
                cells.append(("{0}_{1}_{2}_{3}".format(ri, pack, nm, lvl), x, y))
                print("      {0:<26} {1}  {2:4d} loose {3:5d} authored  ({4:.0f} s)"
                      .format(nm, lvl, len(res["loose"]),
                              len(res.get("authored", [])), time.time() - tb))
            except Exception as exc:
                import traceback
                traceback.print_exc()
                print("      FAILED {0}: {1}".format(nm, exc))
            for _ in range(2):
                omni.kit.app.get_app().update()

    for _ in range(8):
        omni.kit.app.get_app().update()
    if loose:
        settle.run(stage, loose, static, steps=SETTLE_STEPS, kick=0.08,
                   rng=random.Random(SEED), bake_result=True,
                   velocity_map=vel, density=1600.0, max_speed=6.0,
                   converge=True, max_steps=int(SETTLE_STEPS * 2.2),
                   quiet_steps=50)
    for _ in range(8):
        omni.kit.app.get_app().update()

    print("\n" + "=" * 78)
    print("FIRE ACROSS PACKS   kit={0}  gac={1}  brownstone={2}   levels {3}"
          .format(tally["kit"], tally["gac"], tally["brownstone"],
                  ",".join(LEVELS)))
    print("  row 0 kit        assembled from façade pieces (the reference)")
    print("  row 1 gac        merged mesh SLICED on its measured storey/bay grid")
    print("  row 2 brownstone already 307 meshes — de-instanced, parts reused")
    print("  {0} loose bodies, {1:.0f} s".format(len(loose), time.time() - t0))
    print("=" * 78 + "\n")

    app = omni.kit.app.get_app()
    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            sp = os.path.join(_ISAAC, "utils", "snapshots.py")
            spec = _ilu.spec_from_file_location("snapshots", sp)
            sn = _ilu.module_from_spec(spec)
            spec.loader.exec_module(sn)
            os.makedirs(SNAP_DIR, exist_ok=True)
            if FLOW:
                timeline.play()
            for _ in range(300 if FLOW else 90):
                app.update()
            span_x, span_y = ncol * COL_M, len(rows) * ROW_M
            sn.overview(stage, (0.0, 0.0), max(span_x, span_y) * 1.2,
                        os.path.join(SNAP_DIR, "grid_top.png"), ssf)
            # per row, looking steeply down so the rows behind stay out of frame
            for ri, (pack, _n) in enumerate(rows):
                y = (ri - (len(rows) - 1) / 2.0) * ROW_M
                d, h = span_x * 0.30, span_x * 0.40
                sn.place_camera(stage, (0.0, (y - d) * ssf, h * ssf),
                                (0.0, y * ssf, 12.0 * ssf))
                sn.snapshot(os.path.join(SNAP_DIR,
                                         "row{0}_{1}.png".format(ri, pack)))
            sn.views_around(stage, {c[0]: (c[1], c[2]) for c in cells},
                            SNAP_DIR, ssf, top_h=95.0, obl_dist=62.0, obl_h=32.0)
            print("[fire_rows] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[fire_rows] snapshots FAILED: {0}".format(exc))

    if _env("KEEP_OPEN") == "1" or not _HEADLESS:
        while simulation_app.is_running():
            app.update()
    timeline.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
