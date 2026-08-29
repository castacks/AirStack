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
    FR_COL_M    MINIMUM column pitch, m (default 95) -- a FLOOR, not the
                pitch itself; see LAYOUT below
    FR_ROW_M    MINIMUM row pitch, m (default 130) -- same, a floor
    FR_SPACING_MULT  multiplier on the widest MEASURED footprint that sets
                the actual pitch once the floor above is cleared (default
                2.2) -- see LAYOUT
    FR_SEED     (default 7)
    FR_FLOW     1 authors NVIDIA Flow and the flames (default 1)
    SETTLE_STEPS physics ceiling (default 1400)
    SNAP_DIR / KEEP_OPEN

LAYOUT -- why two passes, and why the pitch is measured rather than fixed
--------------------------------------------------------------------------
This file used to place every cell on a FIXED lattice: `COL_M` x `ROW_M`,
the same numbers regardless of what actually got built there. That is wrong
in two ways at once. `SM_Building_24` alone measures 29.0 x 58.0 m and other
GAC/brownstone assets are bigger or smaller still, so a constant pitch is
either wasted empty space or an overlap depending on which asset landed in
that cell -- and an F5 collapse throws debris well outside the footprint on
top of that. Separately, `ncol = max(len(r[1]) for r in rows)` used the
WIDEST row's column count (5, from KIT/GAC) to lay out every row, so the
4-entry brownstone row (`FR_AEC`) was positioned on a 5-wide pitch with an
empty hole instead of being centred on its own 4 buildings.

The fix is a measure-then-place pass, because a GAC/brownstone footprint
is only known AFTER `gac_storey_slice.slice_to_kit` has run -- there is no
style table for a sliced merged mesh the way `urban_building.footprint`
covers the kit styles (that table-lookup shortcut is what
`urban_fire_bench_launch_script.py` uses for its `UF_SPACING`, since every
building on ITS row is a known kit style). So:

  PASS 1  every cell is built at a PROVISIONAL position (`FR_COL_M` /
          `FR_ROW_M` on the old fixed lattice). This is fine BECAUSE `burn()`
          always authors in the cell's own LOCAL 0,0,0 frame (see
          `qf.describe`/`ub.build_building` calls below) and `place_asset`
          centres a referenced asset's world bbox on the CELL'S OWN world
          translate -- so nothing downstream cares what that translate
          value actually is yet.
  PASS 2  each cell's REAL world XY box is measured with `UsdGeom.BBoxCache`
          using BOTH `default_` and `render` purposes -- a default-only
          cache is the exact blind spot that once let airborne debris audit
          as clean (see the `fix-floating-debris` skill). The SAME
          translate op returned by `AddTranslateOp()` in pass 1 is then
          `.Set()` again (never a second `AddTranslateOp` -- that appends a
          second op to `xformOpOrder` and BOTH apply) so that:
            - within a row, columns are spaced
              `max(FR_COL_M, FR_SPACING_MULT * the widest measured column in
              THAT row)` apart, and a row with fewer entries (the
              brownstones) is centred on its OWN entries -- it no longer
              inherits another row's column count;
            - rows are spaced
              `max(FR_ROW_M, FR_SPACING_MULT * the widest measured building
              anywhere)` apart, so one huge sliced building in one row
              cannot bleed into its neighbour row.
This mirrors `urban_fire_bench_launch_script.py`'s blessed `UF_SPACING`
convention (`max(60, 2.2 * widest)`) on purpose -- same idea, generous
rather than tight, just measured post-build instead of looked up, and
applied on both axes because this file has three rows, not one.

An audit at the end of `main()` (always -- not gated on `SNAP_DIR`) then
checks every pair of cells' FINAL (post-settle) world XY boxes for overlap
and prints anything it finds; it is a report, not an exception, because
settle's physics can still carry a piece of debris further than pass 2's
pre-settle measurement expected.
"""

import itertools
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
from detail import gac_storey_slice as gss                     # noqa: E402
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
    "FR_AEC", "Reference_Brownstone2Row,"
              "Reference_Brownstone5Row,Reference_Brownstone6Row,"
              "Reference_Brownstone8Row").split(",") if v.strip()]
COL_M = float(_env("FR_COL_M", "95"))
ROW_M = float(_env("FR_ROW_M", "130"))
SPACING_MULT = float(_env("FR_SPACING_MULT", "2.2"))
SEED = int(_env("FR_SEED", "7"))
FLOW = _env("FR_FLOW", "1") not in ("0", "false", "no")
SETTLE_STEPS = int(_env("SETTLE_STEPS", "1400"))
SNAP_DIR = _env("SNAP_DIR")

# The image an `overview()` capture writes is 16:9. `snapshots.place_camera`
# uses a plumb (straight-down) rotation, and its own docstring is explicit
# that world +X reads right and +Y reads up in that view -- so the VERTICAL
# field of the frame is the one that has to fit world Y, and it is narrower
# than the horizontal field by the aspect ratio. ROWS run along Y here, which
# is exactly why the brownstone row used to fall out of the overview frame:
# the old `max(span_x, span_y) * 1.2` treated both axes as if they got equal
# coverage. See the framing block near the end of `main()`.
FRAME_V_ASPECT = 9.0 / 16.0
FRAME_MARGIN_M = 20.0


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


def measure_xy(stage, path, cache):
    """World-space `((min_x, min_y), (max_x, max_y))` of everything under
    `path`, or `None` if there is nothing there to measure.

    `cache` MUST be a `UsdGeom.BBoxCache` built with BOTH `default_` and
    `render` purposes. A default-purpose-only cache is the known blind spot
    in this repo (see the `fix-floating-debris` skill): geometry that is
    only tagged `render` audits as absent instead of as out of place.
    """
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return None
    r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    mn, mx = r.GetMin(), r.GetMax()
    return (float(mn[0]), float(mn[1])), (float(mx[0]), float(mx[1]))


def xy_overlap_area(box_a, box_b):
    """XY overlap area in m^2 of two `(mn, mx)` boxes from `measure_xy`, or
    0.0 if they do not intersect."""
    (amn0, amn1), (amx0, amx1) = box_a
    (bmn0, bmn1), (bmx0, bmx1) = box_b
    ox = min(amx0, bmx0) - max(amn0, bmn0)
    oy = min(amx1, bmx1) - max(amn1, bmn1)
    if ox <= 0.0 or oy <= 0.0:
        return 0.0
    return ox * oy


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
    # PROVISIONAL grid width ONLY, for pass 1's initial placement. This used
    # to also be the FINAL layout, which is exactly what put the 4-entry
    # brownstone row on a 5-wide lattice with a hole in it — see LAYOUT in
    # the module docstring. Real positions are computed in pass 2, below.
    prov_ncol = max(len(r[1]) for r in rows)
    flow_root = None
    if FLOW:
        from disaster import fire as fx
        fx.setup_flow_stack(stage, density_cell_size_m=0.30, max_blocks=12288,
                            scene_scale_factor=ssf)
        flow_root = fx.FLOW_ROOT
    mats = uf.materials(stage, PARENT)
    mat_cache = {}
    loose, static, vel = [], ["/World/ground"], {}
    cell_info, tally = [], {"kit": 0, "gac": 0, "brownstone": 0}

    # ------------------------------------------------------------------
    # PASS 1: build every cell at a provisional position. Each cell Xform
    # (`{PARENT}/{pack}_{ci}_link`) is the LINK PRIM for its building — the
    # one handle that moves the kit/sliced pieces, the fracture fragments,
    # the settled debris and everything `burn_building` authors, all at
    # once, because every one of those lives underneath it. The `_link`
    # suffix and the escapee check right after `burn()` below both exist
    # to make and keep that true.
    # ------------------------------------------------------------------
    for ri, (pack, names) in enumerate(rows):
        y0 = (ri - (len(rows) - 1) / 2.0) * ROW_M  # provisional only
        print("\n[fire_rows] === row {0}: {1} ===".format(ri, pack))
        for ci, nm in enumerate(names):
            x0 = (ci - (prov_ncol - 1) / 2.0) * COL_M  # provisional only
            lvl = LEVELS[ci % len(LEVELS)]
            tag = "{0}{1}".format(pack[:3], ci)
            cell = "{0}/{1}_{2}_link".format(PARENT, pack, ci)
            cxf = UsdGeom.Xform.Define(stage, Sdf.Path(cell))
            tr = cxf.AddTranslateOp()
            tr.Set(Gf.Vec3d(x0, y0, 0.0))
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
                    style = "sl_" + nm
                    pls, g, meas = gss.slice_to_kit(stage, src, cell, style,
                                                    verbose=False)
                    if not pls:
                        print("      skip {0}: sliced to nothing".format(nm))
                        continue
                    print("      {0}: {1} kit piece(s), {2} grid".format(
                        nm, len(pls), "measured" if meas else "regular"))
                elif pack == "brownstone":
                    src = place_asset(stage, cell, AEC_DIR + nm + ".usd", 0.01)
                    if not src:
                        print("      skip {0}: nothing composed".format(nm))
                        continue
                    style = "sl_" + nm
                    pls, g, meas = gss.slice_to_kit(stage, src, cell, style,
                                                    verbose=False)
                    if not pls:
                        print("      skip {0}: sliced to nothing".format(nm))
                        continue
                    print("      {0}: {1} kit piece(s), {2} grid".format(
                        nm, len(pls), "measured" if meas else "regular"))
                res = burn(stage, cell, style, pls, lvl, rng, nrng, mats, tag,
                           flow_root, mat_cache)
                # EVERY PATH `burn_building` HANDS BACK MUST LIVE UNDER THIS
                # CELL, because the cell is the whole building's link prim —
                # moving it must move everything. Report, do not reparent:
                # reparenting can silently break a reference or a material
                # binding authored with an absolute path, and this is meant
                # to catch a bug in the damage code, not paper over it here.
                escapees = []
                for lname, lst in (("loose", res["loose"]),
                                   ("static_extra", res["static_extra"]),
                                   ("authored", res.get("authored", []))):
                    for p in lst:
                        if p != cell and not p.startswith(cell + "/"):
                            escapees.append((lname, p))
                if escapees:
                    print("      [fire_rows] ESCAPEE(S) from {0}:".format(cell))
                    for lname, p in escapees:
                        print("      [fire_rows]   {0}: {1}".format(lname, p))
                loose += res["loose"]
                static += res["static_extra"]
                vel.update(res["velocity"])
                tally[pack] += 1
                label = "{0}_{1}_{2}_{3}".format(ri, pack, nm, lvl)
                cell_info.append({"path": cell, "row": ri, "col": ci,
                                  "pack": pack, "name": nm, "label": label,
                                  "tr": tr, "x0": x0, "y0": y0})
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

    # ------------------------------------------------------------------
    # PASS 2: measure what actually got built, then set the REAL pitch.
    # See LAYOUT in the module docstring for why this has to be a second
    # pass instead of a computed-up-front constant.
    # ------------------------------------------------------------------
    cells, row_y = [], {}
    if cell_info:
        mcache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        for c in cell_info:
            box = measure_xy(stage, c["path"], mcache)
            if box is None:
                # Nothing measurable under a cell that got far enough to be
                # tallied is unexpected, but not this file's damage bug to
                # diagnose — leave it at its provisional spot rather than
                # dividing by a zero-size footprint below.
                c["full_x"] = c["full_y"] = 0.0
                c["off_x"] = c["off_y"] = 0.0
                continue
            (mn0, mn1), (mx0, mx1) = box
            c["full_x"] = mx0 - mn0
            c["full_y"] = mx1 - mn1
            # How far the MEASURED box's centre sits from the cell's own
            # (provisional) translate — a fixed LOCAL offset. `place_asset`
            # and `apply_placements` both centre a building's core on the
            # cell's own origin, so this is normally ~0, but debris from an
            # F5 collapse is not guaranteed symmetric, and keeping the
            # offset explicit means the reposition below is exactly right
            # either way instead of assuming perfect centring.
            c["off_x"] = 0.5 * (mn0 + mx0) - c["x0"]
            c["off_y"] = 0.5 * (mn1 + mx1) - c["y0"]

        by_row = {}
        for c in cell_info:
            by_row.setdefault(c["row"], []).append(c)
        row_indices = sorted(by_row)
        big_all = max(max(c["full_x"], c["full_y"]) for c in cell_info)
        # Same convention `urban_fire_bench_launch_script.py` uses for
        # UF_SPACING (`max(60, 2.2 * widest)`): generous rather than tight,
        # applied here on the ROW axis using the single widest measured
        # building ANYWHERE, so a huge sliced GAC/brownstone building in one
        # row cannot bleed into the row next to it.
        row_spacing = max(ROW_M, SPACING_MULT * big_all)
        ry0 = -0.5 * row_spacing * (len(row_indices) - 1)
        for rpos, ri in enumerate(row_indices):
            row_cells = sorted(by_row[ri], key=lambda c: c["col"])
            big_row = max(max(c["full_x"], c["full_y"]) for c in row_cells)
            # Same convention again, this time per row and on the COLUMN
            # axis: the pitch is set by THIS row's own widest measured
            # column, so a row with fewer/smaller entries (the brownstones)
            # is centred on exactly what it has — no reference to another
            # row's column count, which is what produced the hole before.
            col_spacing = max(COL_M, SPACING_MULT * big_row)
            cx0 = -0.5 * col_spacing * (len(row_cells) - 1)
            by_ = ry0 + rpos * row_spacing
            row_y[ri] = by_
            for cpos, c in enumerate(row_cells):
                bx = cx0 + cpos * col_spacing
                c["x"] = bx - c["off_x"]
                c["y"] = by_ - c["off_y"]
                # Reuse the SAME translate op from pass 1 — a second
                # `AddTranslateOp()` here would append a second op to
                # `xformOpOrder` and BOTH would apply.
                c["tr"].Set(Gf.Vec3d(c["x"], c["y"], 0.0))
                cells.append((c["label"], c["x"], c["y"]))
            print("[fire_rows] row {0} {1:<12} {2} column(s), pitch {3:.1f} m "
                  "(floor {4:.0f}, {5:.1f} x widest {6:.1f} m)".format(
                      ri, row_cells[0]["pack"], len(row_cells), col_spacing,
                      COL_M, SPACING_MULT, big_row))
        print("[fire_rows] row pitch {0:.1f} m (floor {1:.0f}, {2:.1f} x "
              "widest {3:.1f} m anywhere)".format(row_spacing, ROW_M,
                                                  SPACING_MULT, big_all))
        for _ in range(4):
            omni.kit.app.get_app().update()

    # Ground is sized from what was actually measured (plus debris scatter,
    # since pass 2's boxes already include any fracture/rubble authored by
    # `burn()`) instead of the old `ncol * COL_M` guess, which knew nothing
    # about real asset size either. `ground_and_light` also authors the
    # lights, so it is called exactly once, here, after pass 2 knows the
    # true extent — nothing earlier in this function needs the ground prim
    # to exist (`settle.run` below is the only consumer, via `static`).
    pre_settle_box = None
    if cell_info:
        gcache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        pre_settle_box = measure_xy(stage, PARENT, gcache)
    if pre_settle_box:
        (pmn0, pmn1), (pmx0, pmx1) = pre_settle_box
        ground_and_light(stage, (pmx0 - pmn0) + 140.0, (pmx1 - pmn1) + 160.0)
    else:
        ground_and_light(stage, COL_M + 140.0, ROW_M + 160.0)

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

    # ------------------------------------------------------------------
    # HARD AUDIT — every cell pair, FINAL (post-settle) world XY box.
    # Always runs, never gated on SNAP_DIR: this is a report, not an
    # exception, because settle's physics can still carry debris further
    # than pass 2's pre-settle measurement expected, and the run should
    # finish and snapshot regardless so a human can look at what happened.
    # ------------------------------------------------------------------
    boxes = []
    if cell_info:
        acache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        for c in cell_info:
            b = measure_xy(stage, c["path"], acache)
            if b is not None:
                boxes.append((c["label"], c["path"], b))
    overlaps = []
    for (la, pa, ba), (lb, pb, bb) in itertools.combinations(boxes, 2):
        area = xy_overlap_area(ba, bb)
        if area > 0.0:
            overlaps.append((la, pa, lb, pb, area))
    print("[fire_rows] OVERLAP AUDIT ({0} cell(s) checked, final/post-settle "
          "boxes)".format(len(boxes)))
    if overlaps:
        print("[fire_rows] OVERLAP " + "=" * 60)
        print("[fire_rows] OVERLAP  {0} pair(s) of cells intersect in XY:"
              .format(len(overlaps)))
        for la, pa, lb, pb, area in overlaps:
            print("[fire_rows] OVERLAP    {0}  x  {1}   {2:.1f} m^2"
                  .format(la, lb, area))
            print("[fire_rows] OVERLAP      {0}".format(pa))
            print("[fire_rows] OVERLAP      {0}".format(pb))
        print("[fire_rows] OVERLAP  this is a report, not an exception — "
              "the run continues and still snapshots.")
        print("[fire_rows] OVERLAP " + "=" * 60)
    else:
        print("[fire_rows] OVERLAP  none found.")

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
            # Frame the overview from the AUDITED world bounds of everything
            # under PARENT (the `boxes` collected above) rather than the old
            # `ncol * COL_M` guess, and correct for the 16:9 capture: world Y
            # (the row axis) is the frame's VERTICAL dimension in a plumb
            # top-down shot, and the vertical field only covers
            # FRAME_V_ASPECT (~0.5625x) of what the horizontal field does at
            # the same span — which is why the brownstone row used to be cut
            # out of frame even though it was well within the horizontal
            # span. The span passed to `overview()` has to satisfy the
            # SHORTER (vertical) dimension, so it is sized off `depth /
            # FRAME_V_ASPECT`, not `depth` directly.
            if boxes:
                mn0 = min(b[0][0] for _, _, b in boxes)
                mn1 = min(b[0][1] for _, _, b in boxes)
                mx0 = max(b[1][0] for _, _, b in boxes)
                mx1 = max(b[1][1] for _, _, b in boxes)
            else:
                mn0 = mn1 = -0.5 * COL_M
                mx0 = mx1 = 0.5 * COL_M
            cx, cy = 0.5 * (mn0 + mx0), 0.5 * (mn1 + mx1)
            width = (mx0 - mn0) + 2.0 * FRAME_MARGIN_M
            depth = (mx1 - mn1) + 2.0 * FRAME_MARGIN_M
            frame_span = max(width, depth / FRAME_V_ASPECT)
            sn.overview(stage, (cx, cy), frame_span,
                        os.path.join(SNAP_DIR, "grid_top.png"), ssf)
            # per row, looking steeply down so the rows behind stay out of
            # frame; `row_y[ri]` is the FINAL (pass 2) row centre, not the
            # old fixed `(ri - ...) * ROW_M`.
            for ri in sorted(row_y):
                pack = rows[ri][0]
                y = row_y[ri]
                # `width`, not `frame_span` -- this is a per-row shot, so it
                # wants the column (X) extent only, not the vertical-aspect-
                # corrected span the multi-row overview above needs.
                d, h = width * 0.30, width * 0.40
                sn.place_camera(stage, (cx, (y - d) * ssf, h * ssf),
                                (cx, y * ssf, 12.0 * ssf))
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
