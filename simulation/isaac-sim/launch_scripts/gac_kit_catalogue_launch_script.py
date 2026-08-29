#!/usr/bin/env python
"""
EVERY GAC building and AEC brownstone, each shown assembled with its own
kit-bash pieces laid out in front of it. No damage.

    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/kit_cat \
    ISAAC_SIM_SCRIPT_NAME=gac_kit_catalogue_launch_script.py airstack up isaac-sim

One BAY per asset, in a row along X. Each bay is:

    the assembled building, standing at the back
    its kit pieces, laid out on the ground IN FRONT of it in a grid

so a piece that came out malformed cannot hide inside the silhouette of the
building it was cut from, and the assembled copy beside it is the control.

The slicing itself, the measured storey/bay grid, the cut-between-the-windows
rule and the five bugs that each render as a flat brown box are documented in
`.agents/skills/slice-buildings-into-kits`.

BROWNSTONES HAVE NO WINDOWS TO MEASURE. They carry no glass subset
(`tools/openings_probe.py`), so their grid cannot be derived and they fall back
to `KC_FALLBACK_H`. The banner says which assets were measured and which
assumed — do not read an assumed one as a validated slice.

Env:
    KC_GAC        comma list of GAC assets (default: a spread of eight)
    KC_AEC        comma list of brownstones (default: three)
    KC_BAY_M      spacing between assets, m (default 150)
    KC_FALLBACK_H target storey height for the regular grid (default 3.95,
                  the median of the GAC buildings that ARE measurable)
    KC_PITCH      grid pitch for the laid-out pieces, m (default 3.0)
    KC_MAX_PIECES cap on pieces laid out per asset (default 420) — a 1,100
                  piece building carpets the plate otherwise
    SNAP_DIR / KEEP_OPEN
"""
import math
import os
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

import omni.kit.app                                            # noqa: E402
import omni.timeline                                           # noqa: E402
import omni.usd                                                # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux                  # noqa: E402

_ISAAC = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".."))
_SG = os.path.normpath(os.path.join(_ISAAC, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC, "utils"))
sys.path.insert(0, _SG)

from scene_prep import get_stage_meters_per_unit               # noqa: E402
from detail import gac_slice as gsl                            # noqa: E402
from detail import gac_storey_slice as gss                     # noqa: E402
from disaster import fracture                                  # noqa: E402

NUC = "omniverse://airlab-nucleus.andrew.cmu.edu:443/"
GAC_DIR = NUC + "Projects/SEI-COA/GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/"
AEC_DIR = ("airstack://scene_gen/assets/aec/brownstone/Assets/"
           "Create_Brownstone02/")
# ALL 31 GAC BUILDINGS, not the eight the first cut sampled.
# The Meshes/ folder holds 60 `SM_Building_*` files, but most are not
# buildings: `_Part_01`/`_Part_02` are sub-assemblies, `_19_Lower_Entrance` /
# `_Middle_Part` / `_Upper_Part` are pieces of _19, and Door/Ceiling/Stair/
# Floor/Air are components. A real building is `SM_Building_<n>` with an
# optional `_Small`.
_GAC_ALL = ["SM_Building_{0:02d}".format(k) for k in range(1, 32)]
_GAC_ALL[5] = "SM_Building_06_Small"          # the only one with a suffix
GAC = [v.strip() for v in _env("KC_GAC", ",".join(_GAC_ALL)).split(",")
       if v.strip()]
AEC = [v.strip() for v in _env(
    "KC_AEC", "Reference_Brownstone2Row,"
              "Reference_Brownstone5Row").split(",") if v.strip()]
BAY_M = float(_env("KC_BAY_M", "150"))
FALLBACK_H = float(_env("KC_FALLBACK_H", "3.95"))
PITCH = float(_env("KC_PITCH", "3.0"))
MAX_PIECES = int(_env("KC_MAX_PIECES", "420"))
SNAP_DIR = _env("SNAP_DIR")
PARENT = "/World/cat"


def light_and_ground(stage, w, d):
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/dome"))
    dome.CreateIntensityAttr(1200.0)
    dome.CreateColorAttr(Gf.Vec3f(0.82, 0.85, 0.92))
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/key"))
    key.CreateIntensityAttr(2600.0)
    key.CreateAngleAttr(0.9)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-46.0, 0.0, 34.0))
    g = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/ground"))
    hw, hd = w * 0.6, d * 0.6
    g.CreatePointsAttr([Gf.Vec3f(-hw, -hd, 0), Gf.Vec3f(hw, -hd, 0),
                        Gf.Vec3f(hw, hd, 0), Gf.Vec3f(-hw, hd, 0)])
    g.CreateFaceVertexCountsAttr([4])
    g.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    g.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    g.CreateDisplayColorAttr([Gf.Vec3f(0.33, 0.33, 0.32)])


def place_source(stage, cell, usd, scale):
    holder = cell + "/src"
    UsdGeom.Xform.Define(stage, Sdf.Path(holder))
    kid = stage.DefinePrim(Sdf.Path(holder + "/asset"))
    import scene_generator as sg
    kid.GetReferences().AddReference(sg._join_asset_root(usd, ""))
    stage.Load(Sdf.Path(holder))
    xf = UsdGeom.Xformable(kid)
    xf.ClearXformOpOrder()
    tr = xf.AddTranslateOp()
    if abs(scale - 1.0) > 1e-9:
        xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    r = bc.ComputeWorldBound(stage.GetPrimAtPath(holder)).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    mn, mx = r.GetMin(), r.GetMax()
    c = UsdGeom.XformCache().GetLocalToWorldTransform(
        stage.GetPrimAtPath(cell)).ExtractTranslation()
    tr.Set(Gf.Vec3d(-(0.5 * (mn[0] + mx[0]) - c[0]),
                    -(0.5 * (mn[1] + mx[1]) - c[1]), -(mn[2] - c[2])))
    return holder


def do_asset(stage, cell, name, usd, scale):
    """Slice one asset; assembled copy at the back, pieces laid out in front."""
    src = place_source(stage, cell, usd, scale)
    if not src:
        return None
    wins, bbox = gsl.window_centres(stage, src)
    # Measured window lattice where the asset has one, a regular grid where it
    # does not. Only 10 of 31 GAC buildings have a readable lattice; a glass
    # tower has no window rows to avoid cutting, so a regular grid is the
    # right answer for it rather than a concession.
    g, measured = gss.grid_for(stage, src, bbox, wins, name=name,
                               target=FALLBACK_H, verbose=False)
    m = gss.read_mesh(stage, src, verbose=False)
    if m is None:
        return None
    lines = gss.cut_lines(g, 0.5, verbose=False)
    clear, hit = gss.window_clearance(lines, wins) if wins else (None, 0)
    bands = gss.storeys(m, lines, verbose=False)
    leg = max(1.2, 0.6 * ((g["bays"].get("E") or {}).get("pitch") or 3.5))
    bay = (g["bays"].get("E") or {}).get("pitch") or 3.5
    cells = []
    for i, (lo, hi, band) in enumerate(bands):
        bb = ((bbox[0][0], bbox[0][1], lo), (bbox[1][0], bbox[1][1], hi))
        for role, side, k, piece in gss.ring(band, bb, leg, bay):
            cells.append((i, role, side, k, piece))

    # the assembled copy stands at the BACK of the bay
    back = cell + "/assembled"
    UsdGeom.Scope.Define(stage, Sdf.Path(back))
    for j, (i, role, side, k, piece) in enumerate(cells):
        gss.write_piece(stage, "{0}/p{1:04d}".format(back, j), piece, m["mats"])
    UsdGeom.Xformable(
        UsdGeom.Xform.Define(stage, Sdf.Path(back))).AddTranslateOp().Set(
            Gf.Vec3d(0.0, g["D"] * 1.1 + 14.0, 0.0))
    UsdGeom.Imageable(stage.GetPrimAtPath(src)).MakeInvisible()

    # ...and the pieces are laid out IN FRONT of it, on the ground
    front = cell + "/pieces"
    UsdGeom.Scope.Define(stage, Sdf.Path(front))
    show = cells[:MAX_PIECES]
    ncol = max(1, int(math.ceil(math.sqrt(len(show)))))
    laid = 0
    for j, (i, role, side, k, piece) in enumerate(show):
        path = "{0}/{1}_{2}_{3:04d}".format(front, role, side.replace("-", "x"), j)
        if not gss.write_piece(stage, path, piece, m["mats"]):
            continue
        gx = (j % ncol) * PITCH - 0.5 * ncol * PITCH
        gy = -(j // ncol) * PITCH - 6.0
        import numpy as np
        c = piece["P"].mean(axis=0)
        UsdGeom.Xformable(stage.GetPrimAtPath(path)).AddTranslateOp().Set(
            Gf.Vec3d(gx - float(c[0]), gy - float(c[1]),
                     -float(piece["P"][:, 2].min())))
        laid += 1
    return {"name": name, "g": g, "measured": measured, "bands": len(bands),
            "cells": len(cells), "laid": laid, "clear": clear, "hit": hit,
            "mats": len(m["mats"])}


def main():
    tl = omni.timeline.get_timeline_interface()
    tl.stop()
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    w = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(w.GetPrim())
    UsdGeom.Scope.Define(stage, Sdf.Path(PARENT))
    _, ssf = get_stage_meters_per_unit(stage)
    t0 = time.time()
    fracture.ensure_vtk(verbose=False)

    assets = ([(n, GAC_DIR + n + ".usd", 0.01) for n in GAC]
              + [(n, AEC_DIR + n + ".usd", 0.01) for n in AEC])
    light_and_ground(stage, len(assets) * BAY_M + 200.0, 420.0)
    rows = []
    for ci, (name, usd, sc) in enumerate(assets):
        x = (ci - (len(assets) - 1) / 2.0) * BAY_M
        cell = "{0}/a{1:02d}".format(PARENT, ci)
        UsdGeom.Xform.Define(stage, Sdf.Path(cell)).AddTranslateOp().Set(
            Gf.Vec3d(x, 0.0, 0.0))
        tb = time.time()
        try:
            r = do_asset(stage, cell, name, usd, sc)
            if r:
                rows.append(r)
                print("      {0:<24} {1:>4} piece(s) from {2:>2} band(s), "
                      "storey {3:.2f} m {4}  ({5:.0f} s)".format(
                          name, r["cells"], r["bands"], r["g"]["storey_h"],
                          "measured" if r["measured"] else "regular",
                          time.time() - tb))
            else:
                print("      {0:<24} SKIPPED (nothing composed)".format(name))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("      FAILED {0}: {1}".format(name, exc))
        for _ in range(2):
            omni.kit.app.get_app().update()

    for _ in range(10):
        omni.kit.app.get_app().update()
    print("\n" + "=" * 78)
    print("KIT CATALOGUE   {0} asset(s)".format(len(rows)))
    for r in rows:
        print("  {0:<24} {1:5.1f} x {2:5.1f} x {3:5.1f} m  storey {4:.2f} m "
              "{5:<8} {6:>4} piece(s), {7} laid out, {8} material(s){9}".format(
                  r["name"], r["g"]["W"], r["g"]["D"], r["g"]["H"],
                  r["g"]["storey_h"],
                  "measured" if r["measured"] else "regular",
                  r["cells"], r["laid"], r["mats"],
                  "" if r["clear"] is None else
                  "  cut clearance {0:.2f} m, {1} window(s) crossed".format(
                      r["clear"], r["hit"])))
    print("  {0:.0f} s".format(time.time() - t0))
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
            for _ in range(120):
                app.update()
            span = len(assets) * BAY_M
            sn.overview(stage, (0.0, -20.0), span * 1.05,
                        os.path.join(SNAP_DIR, "catalogue_top.png"), ssf)
            for ci, r in enumerate(rows):
                x = (ci - (len(assets) - 1) / 2.0) * BAY_M
                d = max(r["g"]["W"], r["g"]["D"]) * 2.2 + 40.0
                sn.place_camera(stage, ((x + d * 0.7) * ssf, -d * 1.15 * ssf,
                                        d * 0.55 * ssf),
                                (x * ssf, -10.0 * ssf, r["g"]["H"] * 0.3 * ssf))
                sn.snapshot(os.path.join(SNAP_DIR, "{0}.png".format(r["name"])))
            print("[kit_cat] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[kit_cat] snapshots FAILED: {0}".format(exc))

    if _env("KEEP_OPEN") == "1" or not _HEADLESS:
        while simulation_app.is_running():
            app.update()
    tl.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
