#!/usr/bin/env python
"""
ModernCityEnvironment ONLY, on the fire ladder, with each merged original
standing next to its damaged kit substitute.

    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/mce_fire \
    ISAAC_SIM_SCRIPT_NAME=mce_fire_launch_script.py airstack up isaac-sim

WHY THIS PACK GETS NO SLICER
-----------------------------
`ModernCityEnvironment` ships TWO exports of the SAME source art:
`ModernCityEnvironment01`, the façade KIT that `detail/urban_building.py`
assembles from modules, and `SM_MERGED_BP_MBuilding*` — the name says
MERGED — the same art welded into one mesh.

`disaster/urban_fire.burn_building` damages a building by TAKING ELEMENTS
AWAY: it empties a named window, removes a named façade module and puts joist
stubs where it was. `detail/gac_storey_slice.py` can recover parts from a
merged mesh by cutting it on a measured storey/bay grid, but slicing recovers
separability by POSITION and never by IDENTITY — you can cut out the
rectangle at (storey 3, side S, bay 2), you cannot ask which triangles are the
window reveal. Every recipe that addresses a named part therefore degrades to
"blacken a rectangle", which is the artefact this pack does not need to suffer
because its kit twin exists.

So (user, 2026-08-29: "why are we splitting up the moderncity buildings if we
have versions of them that look good? Just use those") a merged MCE building
is never sliced. `disaster/kit_substitute.route()` matches it to the kit style
nearest its MEASURED size and `build_kit` assembles that style; the merged
original is kept on the stage, intact, in column 0 of its own row so the swap
can be judged rather than taken on trust.

LAYOUT IS COMPUTED BEFORE ANYTHING IS BUILT, NOT MEASURED AFTERWARDS
---------------------------------------------------------------------
The previous bench laid its cells out on a provisional lattice, built into
them, measured, and moved them. That failed twice over: one cell measured
421 m wide (debris and authored art reach far outside a building), which drove
the pitch to 926 m, and cells still ended up on top of each other. There is no
need for any of it here. A kit style's footprint and height are KNOWN before
it is built — `urban_building.footprint` and `quake_flow._mass_specs` — and
the merged original's box can be measured the moment it is referenced, before
a single recipe runs. So every cell's translate is set ONCE, from numbers that
are already in hand, and nothing moves afterwards.

The audit at the end is therefore a check on that arithmetic rather than a
repair pass. It still runs every time.

Env:
    MF_ASSETS   which merged MCE buildings, comma separated
                (default MBuilding01,MBuilding05,MBuilding02)
    MF_LEVELS   severities across the columns (default F1,F2,F3,F4,F5)
    MF_GAP_M    clear gap between neighbouring cell boxes, m (default 30)
    MF_SEED     (default 7)
    MF_FLOW     1 authors NVIDIA Flow and the flames (default 1)
    SETTLE_STEPS physics ceiling (default 1600)
    SNAP_DIR / KEEP_OPEN
"""

import itertools
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

import scene_generator as sg                                   # noqa: E402
from scene_prep import get_stage_meters_per_unit               # noqa: E402
from detail import urban_building as ub                        # noqa: E402
from disaster import fracture, settle                          # noqa: E402
from disaster import kit_substitute as ksub                    # noqa: E402
from disaster import quake_flow as qf                          # noqa: E402
from disaster import urban_fire as uf                          # noqa: E402

PARENT = "/World/bench"

# The merged MCE stock, with the scale each one needs. BOTH COME FROM
# `config/asset_sets/urban.yaml`, which records them next to the entries
# themselves — MBuilding01 is authored in metres and takes scale 1.0 while
# 02 and 05 are in centimetres and take 0.01. Getting this wrong does not
# error, it silently produces a 2.8 km building, so it is stated per asset
# rather than defaulted.
MCE_DIR = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Library/Stages/"
           "Muyang/ModernCityEnvironment/")
ASSETS = {
    "MBuilding01": ("Collected_Building01/SM_MERGED_BP_MBuilding01.usd", 1.0),
    "MBuilding02": ("Collected_Building02/SM_MERGED_BP_MBuilding02.usd", 0.01),
    "MBuilding03": ("Collected_Building03/SM_MERGED_BP_MBuilding03.usd", 0.01),
    "MBuilding05": ("Collected_Building05/SM_MERGED_BP_MBuilding05.usd", 0.01),
}

WANT = [v.strip() for v in _env(
    "MF_ASSETS", "MBuilding01,MBuilding05,MBuilding02").split(",") if v.strip()]
LEVELS = [v.strip() for v in _env(
    "MF_LEVELS", "F1,F2,F3,F4,F5").split(",") if v.strip()]
GAP_M = float(_env("MF_GAP_M", "30"))
SEED = int(_env("MF_SEED", "7"))
FLOW = _env("MF_FLOW", "1") not in ("0", "false", "no")
SETTLE_STEPS = int(_env("SETTLE_STEPS", "1600"))
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
    hw, hd = w * 0.5 + 60.0, d * 0.5 + 60.0
    g.CreatePointsAttr([Gf.Vec3f(-hw, -hd, 0), Gf.Vec3f(hw, -hd, 0),
                        Gf.Vec3f(hw, hd, 0), Gf.Vec3f(-hw, hd, 0)])
    g.CreateFaceVertexCountsAttr([4])
    g.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    g.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    g.CreateDisplayColorAttr([Gf.Vec3f(0.29, 0.29, 0.28)])
    g.CreateExtentAttr([Gf.Vec3f(-hw, -hd, 0), Gf.Vec3f(hw, hd, 0)])


def measure_asset(stage, holder, usd, scale):
    """Reference `usd` under `holder`, seat its base at z=0 and centre its
    plan on the holder, and return `(W, D, H)` in metres.

    The measurement happens HERE, before any cell has been positioned and
    before any recipe has run, which is the whole point: the merged original
    is the only thing in the scene whose size is not already known from the
    style table, and it is knowable the instant it composes.
    """
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
    r = cache.ComputeWorldBound(
        stage.GetPrimAtPath(holder)).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    mn, mx = r.GetMin(), r.GetMax()
    # CENTRE RELATIVE TO THE HOLDER, NOT TO THE WORLD ORIGIN. `mn`/`mx` come
    # back in WORLD space, so centring on them alone sets a translate that
    # cancels whatever transform the holder's ancestors carry and drops the
    # asset back at (0, 0) — measured: the merged original in a cell placed
    # at x = -150 m came back at x = 0 and audited as fully overlapping every
    # other cell. Subtract the holder's own world origin so the op is
    # expressed in the frame it actually lives in.
    hxf = UsdGeom.XformCache().GetLocalToWorldTransform(
        stage.GetPrimAtPath(holder))
    ox, oy, oz = hxf.ExtractTranslation()
    tr.Set(Gf.Vec3d(-(0.5 * (mn[0] + mx[0]) - ox),
                    -(0.5 * (mn[1] + mx[1]) - oy), -(mn[2] - oz)))
    return (float(mx[0] - mn[0]), float(mx[1] - mn[1]), float(mx[2] - mn[2]))


def style_size(style):
    """`(W, D, H)` of a kit style WITHOUT building it.

    `_mass_specs` mirrors `build_building`'s own arithmetic, so its top is the
    real height including wings and a tower — a plain sum over `spec["bands"]`
    misses those and calls the 67 m `block_residential` an 8 m podium.
    """
    W, D = ub.footprint(ub.STYLES[style])
    top = max(m["top"] for m in qf._mass_specs(style, 0.0, 0.0, 0.0))
    return float(W), float(D), float(top)


def measure_xy(stage, path, cache):
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        return None
    r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    mn, mx = r.GetMin(), r.GetMax()
    return (float(mn[0]), float(mn[1])), (float(mx[0]), float(mx[1]))


def overlap_area(a, b):
    (amn0, amn1), (amx0, amx1) = a
    (bmn0, bmn1), (bmx0, bmx1) = b
    ox = min(amx0, bmx0) - max(amn0, bmn0)
    oy = min(amx1, bmx1) - max(amn1, bmn1)
    return 0.0 if (ox <= 0.0 or oy <= 0.0) else ox * oy


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
    problems = uf.check(verbose=False) + ksub.check(verbose=False)
    if problems:
        raise RuntimeError("; ".join(problems))

    # ------------------------------------------------------------------
    # STEP 1  measure every merged original and route it to a kit style.
    # Nothing is positioned yet; this only decides WHAT each row contains.
    # ------------------------------------------------------------------
    print("\n[mce] routing the merged originals")
    rows = []
    for nm in WANT:
        if nm not in ASSETS:
            print("[mce]   {0}: not in ASSETS, skipped".format(nm))
            continue
        rel, scale = ASSETS[nm]
        holder = "{0}/probe_{1}".format(PARENT, nm)
        dims = measure_asset(stage, holder, MCE_DIR + rel, scale)
        if dims is None:
            print("[mce]   {0}: composed nothing, skipped".format(nm))
            continue
        W, D, H = dims
        kind, style = ksub.route(MCE_DIR + rel, W, D, H)
        if kind != "kit" or not style:
            print("[mce]   {0:<12} {1:5.1f} x {2:5.1f} x {3:5.1f} m  ->  {4} "
                  "({5})".format(nm, W, D, H, kind.upper(), style or "-"))
            continue
        kw, kd, kh = style_size(style)
        print("[mce]   {0:<12} {1:5.1f} x {2:5.1f} x {3:5.1f} m  ->  kit "
              "{4:<18} {5:5.1f} x {6:5.1f} x {7:5.1f} m".format(
                  nm, W, D, H, style, kw, kd, kh))
        rows.append({"name": nm, "holder": holder, "style": style,
                     "src": (W, D, H), "kit": (kw, kd, kh)})
    if not rows:
        raise RuntimeError("no MCE asset routed to a kit style")

    # ------------------------------------------------------------------
    # STEP 2  the layout, from the numbers STEP 1 already has.
    # Column 0 is the merged original; columns 1..N are the kit at each
    # severity. Pitch is the widest thing that will stand in that column
    # plus a clear gap, so two cells cannot touch by construction.
    # ------------------------------------------------------------------
    ncol = 1 + len(LEVELS)
    col_w = [0.0] * ncol
    for r in rows:
        col_w[0] = max(col_w[0], r["src"][0])
        for c in range(1, ncol):
            col_w[c] = max(col_w[c], r["kit"][0])
    xs, acc = [], 0.0
    for c in range(ncol):
        acc += (col_w[c] * 0.5 if c == 0
                else (col_w[c - 1] * 0.5 + GAP_M + col_w[c] * 0.5))
        xs.append(acc)
    span_x = acc + col_w[-1] * 0.5
    xs = [x - span_x * 0.5 for x in xs]
    ys, acc = [], 0.0
    for i, r in enumerate(rows):
        dep = max(r["src"][1], r["kit"][1])
        acc += (dep * 0.5 if i == 0
                else (max(rows[i - 1]["src"][1], rows[i - 1]["kit"][1]) * 0.5
                      + GAP_M + dep * 0.5))
        ys.append(acc)
    span_y = acc + max(rows[-1]["src"][1], rows[-1]["kit"][1]) * 0.5
    ys = [y - span_y * 0.5 for y in ys]
    print("\n[mce] layout {0} row(s) x {1} column(s), {2:.0f} x {3:.0f} m, "
          "{4:.0f} m gap".format(len(rows), ncol, span_x, span_y, GAP_M))
    ground_and_light(stage, span_x, span_y)

    flow_root = None
    if FLOW:
        from disaster import fire as fx
        fx.setup_flow_stack(stage, density_cell_size_m=0.30, max_blocks=12288,
                            scene_scale_factor=ssf)
        flow_root = fx.FLOW_ROOT
    mats = uf.materials(stage, PARENT)
    mat_cache = {}
    loose, static, vel = [], ["/World/ground"], {}
    cells, escaped = [], []

    # ------------------------------------------------------------------
    # STEP 3  build. Every cell's translate is set ONCE, here, and nothing
    # moves afterwards. The cell Xform is the LINK PRIM for that building:
    # the kit pieces, the fracture fragments, the rubble and the settled
    # debris all live under it, so dragging it moves the whole thing.
    # ------------------------------------------------------------------
    for ri, r in enumerate(rows):
        print("\n[mce] === {0} -> {1} ===".format(r["name"], r["style"]))
        # column 0: the merged original, moved from its probe spot into place
        cell0 = "{0}/{1}_orig_link".format(PARENT, r["name"])
        cxf = UsdGeom.Xform.Define(stage, Sdf.Path(cell0))
        cxf.AddTranslateOp().Set(Gf.Vec3d(xs[0], ys[ri], 0.0))
        # reparent the probe under its cell by re-referencing rather than
        # moving prims: cheaper, and the probe holder keeps its own centring
        rel, scale = ASSETS[r["name"]]
        if measure_asset(stage, cell0 + "/src", MCE_DIR + rel, scale) is None:
            print("      original composed nothing")
        stage.RemovePrim(Sdf.Path(r["holder"]))
        cells.append(("{0}_orig".format(r["name"]), cell0, xs[0], ys[ri]))

        for ci, lvl in enumerate(LEVELS):
            cell = "{0}/{1}_{2}_link".format(PARENT, r["name"], lvl)
            cxf = UsdGeom.Xform.Define(stage, Sdf.Path(cell))
            cxf.AddTranslateOp().Set(Gf.Vec3d(xs[ci + 1], ys[ri], 0.0))
            rng = random.Random(SEED + 37 * ri + ci)
            import numpy as np
            nrng = np.random.default_rng(SEED + 37 * ri + ci)
            tb = time.time()
            try:
                # BUILD INTO A CHILD, NOT INTO THE LINK PRIM ITSELF.
                # `build_kit` -> `scene_generator.apply_placements` starts
                # with `UsdGeom.Scope.Define(stage, parent_path)`, which
                # CONVERTS whatever is at that path into a Scope. A Scope is
                # not Xformable, so the `xformOp:translate` authored on the
                # cell survives on the prim and is then silently never
                # applied — every building snaps back to (0, 0) with no
                # error and no warning. MEASURED: six cells laid out 60 m
                # apart audited as 15 of 15 pairs overlapping, with each
                # overlap area exactly equal to the smaller building's own
                # footprint, i.e. perfectly concentric.
                #
                # Handing it `<cell>/parts` lets it demote a child it owns
                # while the link prim above stays an Xform and keeps the
                # move. `burn_building` still gets `cell`, so the damage art
                # it authors lands under the same link prim.
                pls = ksub.build_kit(stage, cell + "/parts", r["style"],
                                     seed=SEED + ci, ssf=ssf)
                # THE TAG MUST START WITH A LETTER. It is interpolated
                # straight into prim names (`/World/flow/emitters/<tag>_...`,
                # `roofslab_<tag>_...`), and USD rejects an identifier that
                # begins with a digit — "Ill-formed SdfPath
                # </World/flow/emitters/012_00_0>", then a bare
                # `Tf.ErrorException` out of `DefinePrim`. Building the tag
                # from `"MBuilding01"[-2:]` gave exactly that, and only from
                # F2 up, because F1 authors no flames.
                tag = "m{0}{1}".format(r["name"][-2:], lvl)
                res = burn(stage, cell, r["style"], pls, lvl, rng, nrng,
                           mats, tag, flow_root, mat_cache)
                loose += res["loose"]
                static += res["static_extra"]
                vel.update(res["velocity"])
                for lst in ("loose", "static_extra", "authored"):
                    for p in res.get(lst, []):
                        if p and not str(p).startswith(cell + "/"):
                            escaped.append((cell, lst, p))
                cells.append(("{0}_{1}".format(r["name"], lvl), cell,
                              xs[ci + 1], ys[ri]))
                print("      {0}  {1}  {2:4d} loose {3:5d} authored  ({4:.0f} s)"
                      .format(r["style"], lvl, len(res["loose"]),
                              len(res.get("authored", [])), time.time() - tb))
            except Exception as exc:
                import traceback
                traceback.print_exc()
                print("      FAILED {0} {1}: {2}".format(r["name"], lvl, exc))
            for _ in range(2):
                omni.kit.app.get_app().update()

    for _ in range(8):
        omni.kit.app.get_app().update()
    if loose:
        settle.run(stage, loose, static, steps=SETTLE_STEPS, kick=0.08,
                   rng=random.Random(SEED), bake_result=True,
                   velocity_map=vel, density=1600.0, max_speed=6.0,
                   converge=True, max_steps=int(SETTLE_STEPS * 3.0),
                   quiet_steps=90)
    for _ in range(8):
        omni.kit.app.get_app().update()

    # ------------------------------------------------------------------
    # THE AUDIT. Here it checks STEP 2's arithmetic rather than repairing
    # it — every cell was placed from a known size, so an overlap means the
    # size was wrong, not that the placement drifted.
    # ------------------------------------------------------------------
    acache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    boxes = []
    for label, path, _x, _y in cells:
        b = measure_xy(stage, path, acache)
        if b is not None:
            boxes.append((label, path, b))
    bad = []
    for (la, pa, ba), (lb, pb, bb) in itertools.combinations(boxes, 2):
        a = overlap_area(ba, bb)
        if a > 0.0:
            bad.append((la, lb, a))
    print("\n" + "=" * 78)
    print("MODERNCITY FIRE   {0} row(s), levels {1}".format(
        len(rows), ",".join(LEVELS)))
    for r in rows:
        print("  {0:<12} {1:5.1f} x {2:5.1f} x {3:5.1f} m  ->  kit {4}".format(
            r["name"], r["src"][0], r["src"][1], r["src"][2], r["style"]))
    print("  {0} loose bodies, {1:.0f} s".format(len(loose), time.time() - t0))
    if escaped:
        print("  {0} authored path(s) OUTSIDE their link prim:".format(
            len(escaped)))
        for cell, lst, p in escaped[:8]:
            print("    {0}  [{1}]  {2}".format(cell, lst, p))
    else:
        print("  link prims: every authored path is under its own cell")
    if bad:
        print("  OVERLAP  {0} pair(s) intersect in XY:".format(len(bad)))
        for la, lb, a in bad:
            print("    {0}  x  {1}   {2:.1f} m^2".format(la, lb, a))
    else:
        print("  OVERLAP  none — {0} cell(s) checked".format(len(boxes)))
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
            # Frame from the audited bounds. The capture is 16:9, so the
            # VERTICAL field covers only ~0.5625x what the horizontal does at
            # the same span — the span passed must satisfy the tighter one, or
            # the last row falls out of frame (which is what hid the
            # brownstone row on the previous bench).
            if boxes:
                x0 = min(b[2][0][0] for b in boxes)
                x1 = max(b[2][1][0] for b in boxes)
                y0 = min(b[2][0][1] for b in boxes)
                y1 = max(b[2][1][1] for b in boxes)
                need = max((x1 - x0), (y1 - y0) / 0.5625) * 1.15
                sn.overview(stage, (0.5 * (x0 + x1), 0.5 * (y0 + y1)), need,
                            os.path.join(SNAP_DIR, "grid_top.png"), ssf)
            for ri, r in enumerate(rows):
                d = span_x * 0.32
                sn.place_camera(stage, (0.0, (ys[ri] - d) * ssf,
                                        span_x * 0.38 * ssf),
                                (0.0, ys[ri] * ssf, 12.0 * ssf))
                sn.snapshot(os.path.join(
                    SNAP_DIR, "row{0}_{1}.png".format(ri, r["name"])))
            sn.views_around(stage, {c[0]: (c[2], c[3]) for c in cells},
                            SNAP_DIR, ssf, top_h=95.0, obl_dist=62.0,
                            obl_h=32.0)
            print("[mce] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[mce] snapshots FAILED: {0}".format(exc))

    if _env("KEEP_OPEN") == "1" or not _HEADLESS:
        while simulation_app.is_running():
            app.update()
    timeline.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
