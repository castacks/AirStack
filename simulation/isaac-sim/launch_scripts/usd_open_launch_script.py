#!/usr/bin/env python
"""
Open ONE USD stage and look at it. No scene generation, no damage.

    OPEN_URL=omniverse://.../CitySample/All_Buildings_Lineup_Hero.usd \
    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/openusd \
    ISAAC_SIM_SCRIPT_NAME=usd_open_launch_script.py airstack up isaac-sim

OPENED, NOT REFERENCED. A referenced asset keeps ITS OWN `metersPerUnit` and
USD converts nothing, so a centimetre asset arrives 100x oversize in a metres
stage; opening it lets its own units govern and the view is honest about what
the file actually contains.

Reports what composed — prim and mesh counts, the bound, and per top-level
group the size and where it sits — then frames the whole thing and each of
the largest groups. Use it to check a vendor pack before writing anything
against it.

Env:
    OPEN_URL     the stage to open (required)
    OPEN_GROUPS  how many top-level groups to photograph (default 6)
    SNAP_DIR     captures, under /isaac-sim/.nvidia-omniverse/logs/
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
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux                  # noqa: E402

_ISAAC = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(_ISAAC, "utils"))

URL = _env("OPEN_URL", "")
SNAP_DIR = _env("SNAP_DIR", "")
N_GROUPS = int(_env("OPEN_GROUPS", "6"))


def _pump(n=30):
    app = omni.kit.app.get_app()
    for _ in range(n):
        app.update()


def main():
    if not URL:
        print("OPEN_URL is required")
        simulation_app.close()
        return
    t0 = time.time()
    ctx = omni.usd.get_context()
    ctx.open_stage(URL)
    stage = ctx.get_stage()
    while stage is None:
        _pump(10)
        stage = ctx.get_stage()
    S = UsdGeom.GetStageMetersPerUnit(stage)
    root = stage.GetDefaultPrim()
    _pump(40)
    if root:
        stage.Load(root.GetPath())
    else:
        stage.Load()
    _pump(150)

    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    meshes = [p for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)]
    npts = 0
    for m in meshes:
        pts = UsdGeom.Mesh(m).GetPointsAttr().Get()
        npts += len(pts) if pts is not None else 0
    r = bc.ComputeWorldBound(stage.GetPseudoRoot()).ComputeAlignedRange()
    mn, mx = (r.GetMin(), r.GetMax()) if not r.IsEmpty() else (
        Gf.Vec3d(0), Gf.Vec3d(0))
    print("\n" + "=" * 78)
    print("OPENED  {0}".format(URL.rsplit("/", 1)[-1]))
    print("  metersPerUnit {0}   up {1}   default prim {2}"
          .format(S, UsdGeom.GetStageUpAxis(stage),
                  root.GetName() if root else None))
    print("  meshes {0}   points {1:,}   loaded in {2:.0f} s"
          .format(len(meshes), npts, time.time() - t0))
    print("  bound {0:.1f} x {1:.1f} x {2:.1f} m"
          .format((mx[0] - mn[0]) * S, (mx[1] - mn[1]) * S,
                  (mx[2] - mn[2]) * S))

    groups = []
    for p in (root.GetChildren() if root else stage.GetPseudoRoot().GetChildren()):
        gb = bc.ComputeWorldBound(p).ComputeAlignedRange()
        if gb.IsEmpty():
            continue
        a, b = gb.GetMin(), gb.GetMax()
        groups.append((( b[0]-a[0]) * (b[1]-a[1]) * S * S, p,
                       (b[0]-a[0])*S, (b[1]-a[1])*S, (b[2]-a[2])*S,
                       0.5*(a[0]+b[0]), 0.5*(a[1]+b[1]), a[2], b[2]))
    groups.sort(reverse=True, key=lambda q: q[0])
    print("\n  top-level groups with geometry: {0}".format(len(groups)))
    print("  {0:<46} {1:>8} {2:>8} {3:>8}".format("group", "W", "D", "H"))
    for _, p, W, D, H, cx, cy, z0, z1 in groups[:24]:
        print("  {0:<46} {1:8.1f} {2:8.1f} {3:8.1f}   base z {4:.1f}"
              .format(p.GetName()[:46], W, D, H, z0 * S))
    print("=" * 78 + "\n")

    if SNAP_DIR:
        try:
            import importlib.util as _ilu
            sp = os.path.join(_ISAAC, "utils", "snapshots.py")
            spec = _ilu.spec_from_file_location("snapshots", sp)
            sn = _ilu.module_from_spec(spec)
            spec.loader.exec_module(sn)
            os.makedirs(SNAP_DIR, exist_ok=True)
            L = UsdLux.DistantLight.Define(stage, Sdf.Path("/_reviewKey"))
            L.CreateIntensityAttr(3200.0)
            L.CreateAngleAttr(0.8)
            L.AddRotateXYZOp().Set(Gf.Vec3f(-38.0, 0.0, 26.0))
            D_ = UsdLux.DomeLight.Define(stage, Sdf.Path("/_reviewDome"))
            D_.CreateIntensityAttr(1100.0)
            _pump(20)
            cx, cy = 0.5 * (mn[0] + mx[0]) * S, 0.5 * (mn[1] + mx[1]) * S
            W = (mx[0] - mn[0]) * S
            D = (mx[1] - mn[1]) * S
            H = (mx[2] - mn[2]) * S
            span = max(W, D)
            sn.place_camera(stage, (cx / S, cy / S, (H + span / 1.05) / S),
                            (cx / S, cy / S, 0.0))
            sn.snapshot(os.path.join(SNAP_DIR, "all_top.png"))
            sn.place_camera(stage,
                            ((cx - span * 0.55) / S, (cy - span * 0.75) / S,
                             (H * 0.7 + span * 0.30) / S),
                            (cx / S, cy / S, (H * 0.35) / S))
            sn.snapshot(os.path.join(SNAP_DIR, "all_view.png"))
            for i, (_, p, gW, gD, gH, gcx, gcy, gz0, gz1) in \
                    enumerate(groups[:N_GROUPS]):
                gs = max(gW, gD, gH)
                sn.place_camera(
                    stage,
                    (gcx - gs * 0.9 / S * S / 1.0, gcy - gs * 0.9,
                     gz0 * 1.0 + (gH * 0.65 + gs * 0.35) / S),
                    (gcx, gcy, gz0 + (gH * 0.40) / S))
                sn.snapshot(os.path.join(
                    SNAP_DIR, "g{0}_{1}.png".format(i, p.GetName()[:40])))
                print("  shot {0}: {1}  {2:.0f} x {3:.0f} x {4:.0f} m"
                      .format(i, p.GetName()[:40], gW, gD, gH), flush=True)
            print("[open] snapshots -> {0}".format(SNAP_DIR))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[open] snapshots FAILED: {0}".format(exc))

    print("OPEN USD DONE")
    keep = _env("KEEP_OPEN", "") == "1" or not _HEADLESS
    app = omni.kit.app.get_app()
    if keep:
        while simulation_app.is_running():
            app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
