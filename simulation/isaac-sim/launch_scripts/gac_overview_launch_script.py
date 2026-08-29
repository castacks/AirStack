#!/usr/bin/env python
"""
GreatAmericanCity — open the pack's own `Overview_Map.usd` and contact-sheet
its 31 buildings.

    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/gac \
    ISAAC_SIM_SCRIPT_NAME=gac_overview_launch_script.py airstack up isaac-sim

WHAT `Overview_Map.usd` ACTUALLY IS — MEASURED BEFORE OPENING IT
----------------------------------------------------------------
Not a city. It is the vendor's SHOWROOM STRIP: 175 top-level Xforms holding
exactly ONE instance of each of the pack's 164 assets, referenced once apiece,
laid out in a line 130 m wide and 2429 m long (whole bound 200 x 2890 x 312 m).
So the review that matters is not an aerial of a block — it is a contact sheet,
one frame per building, which is what `GAC_SHEET` does.

The stage is opened DIRECTLY off Nucleus rather than referenced into a metres
stage, so its own `metersPerUnit` of 0.01 governs and nothing needs rescaling.
Every distance below is therefore in CENTIMETRES on the stage and metres in
the printout; `S` is the one conversion.

WEIGHT. The 31 buildings alone are ~8.0M triangles (36k-823k each, median
240k) over 1.18 GB of USD, and the map pulls in the props too. First open is
slow and that is the asset pack, not the launcher — `GAC_BUILDINGS_ONLY=1`
loads only the `SM_Building_*` payloads when all that is wanted is the sheet.

Env:
    GAC_SHEET           1 (default) writes one frame per building
    GAC_BUILDINGS_ONLY  1 loads only the building payloads
    SNAP_DIR            captures, under /isaac-sim/.nvidia-omniverse/logs/
    KEEP_OPEN           1 to hold the app open in headless mode
"""

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
import omni.usd                                                # noqa: E402
from pxr import Gf, Usd, UsdGeom                               # noqa: E402

_ISAAC = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(_ISAAC, "utils"))

URL = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
       "GreatAmericanCity/Overview_Map.usd")
SNAP_DIR = _env("SNAP_DIR", "")
SHEET = _env("GAC_SHEET", "1") not in ("0", "false")
ONLY_B = _env("GAC_BUILDINGS_ONLY", "0") not in ("0", "false")


def _pump(n=30):
    app = omni.kit.app.get_app()
    for _ in range(n):
        app.update()


def main():
    t0 = time.time()
    ctx = omni.usd.get_context()
    # OPEN, DO NOT REFERENCE. Referencing a cm asset into a metres stage
    # rescales nothing (the trap that landed a DownTown block 100x oversize);
    # opening it lets its own metersPerUnit govern the whole stage.
    ctx.open_stage(URL)
    stage = ctx.get_stage()
    while stage is None:
        _pump(10)
        stage = ctx.get_stage()
    S = UsdGeom.GetStageMetersPerUnit(stage)          # 0.01 -> stage units cm
    _pump(60)

    root = stage.GetDefaultPrim()
    builds = []
    for p in stage.Traverse():
        n = p.GetName()
        if n.startswith("SM_Building_") and p.GetParent() == root:
            builds.append(p)
    if ONLY_B:
        stage.Unload(root.GetPath())
        for p in builds:
            stage.Load(p.GetPath())
    else:
        stage.Load(root.GetPath())
    _pump(120)

    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    meshes = sum(1 for p in stage.Traverse() if p.IsA(UsdGeom.Mesh))
    r = bc.ComputeWorldBound(stage.GetPseudoRoot()).ComputeAlignedRange()
    mn, mx = r.GetMin(), r.GetMax()
    print("\n" + "=" * 74)
    print("GREAT AMERICAN CITY — Overview_Map.usd")
    print("  metersPerUnit {0}   meshes {1}   SM_Building_* prims {2}"
          .format(S, meshes, len(builds)))
    print("  bound {0:.0f} x {1:.0f} x {2:.0f} m   opened in {3:.0f} s"
          .format((mx[0] - mn[0]) * S, (mx[1] - mn[1]) * S,
                  (mx[2] - mn[2]) * S, time.time() - t0))
    print("=" * 74 + "\n")

    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            sp = os.path.join(_ISAAC, "utils", "snapshots.py")
            spec = _ilu.spec_from_file_location("snapshots", sp)
            sn = _ilu.module_from_spec(spec)
            spec.loader.exec_module(sn)
            os.makedirs(SNAP_DIR, exist_ok=True)
            cx, cy = 0.5 * (mn[0] + mx[0]), 0.5 * (mn[1] + mx[1])
            # THE STRIP IS 20x LONGER THAN IT IS WIDE, so one overview of the
            # whole thing resolves nothing. Four along its length instead.
            for i in range(4):
                y = mn[1] + (i + 0.5) * (mx[1] - mn[1]) / 4.0
                sn.place_camera(stage,
                                (cx - 160.0 / S, y, 120.0 / S),
                                (cx, y, 30.0 / S))
                sn.snapshot(os.path.join(SNAP_DIR, "strip_%d.png" % i))
            if SHEET:
                for p in builds:
                    br = bc.ComputeWorldBound(p).ComputeAlignedRange()
                    if br.IsEmpty():
                        continue
                    a, b = br.GetMin(), br.GetMax()
                    W = (b[0] - a[0]) * S
                    D = (b[1] - a[1]) * S
                    H = (b[2] - a[2]) * S
                    px, py = 0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])
                    d = max(W, D, H) * 1.35 + 25.0
                    sn.place_camera(
                        stage,
                        (px - d * 0.80 / S, py - d * 0.80 / S,
                         (a[2] * S + H * 0.72 + d * 0.30) / S),
                        (px, py, (a[2] * S + H * 0.42) / S))
                    sn.snapshot(os.path.join(
                        SNAP_DIR, "b_" + p.GetName() + ".png"))
                    print("  {0:<30} {1:5.1f} x {2:5.1f} x {3:6.1f} m"
                          .format(p.GetName(), W, D, H), flush=True)
            print("[gac] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[gac] snapshots FAILED: {0}".format(exc))

    print("GAC OVERVIEW DONE")
    keep = _env("KEEP_OPEN", "") == "1" or not _HEADLESS
    app = omni.kit.app.get_app()
    if keep:
        while simulation_app.is_running():
            app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
