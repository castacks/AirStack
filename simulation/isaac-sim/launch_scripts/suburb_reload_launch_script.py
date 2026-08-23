#!/usr/bin/env python
"""
Reload a scene from PER-OBJECT baked USDs and time it — the other half of the
bake speedup test.

    # 1. build once, writing per-object bakes:
    MINI_BAKE_DIR=/isaac-sim/AirStack/scene_gen/assets/baked/mini \
    SCENE_CONFIG=suburb_mini_wildfire \
    ISAAC_SIM_SCRIPT_NAME=suburb_mini_wildfire_launch_script.py airstack up isaac-sim

    # 2. reload from those bakes (relaunch in the same container via tmux):
    MINI_RELOAD_DIR=/isaac-sim/AirStack/scene_gen/assets/baked/mini \
    ISAAC_SIM_SCRIPT_NAME=suburb_reload_launch_script.py <relaunch>

WHAT IT MEASURES
----------------
The build prints `[bake] full build ... took N s` — generate + damage +
fracture + settle + scorch + ground + fire, ~20 min on the 250 m block. This
script makes a fresh stage, adds a ground plane + sky + lights, and REFERENCES
every baked house/tree from the manifest at its own world transform — no
generation, no trimesh fracture, no PhysX settle. It prints `referenced N
object(s) in M s`. M vs N is the speedup the "bake damage, reference it" path
buys, and it is what generalises to the 1600x1200 plat (there the same
archetype is referenced many times instead of once).
"""

import os
import sys
import time

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={
    "headless": False,
    "extra_args": ["--/rtx/raytracing/fractionalCutoutOpacity=true",
                   "--/rtx/pathtracing/fractionalCutoutOpacity=true"],
})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("omni.kit.window.script_editor")

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

from disaster import bake                                        # noqa: E402

RELOAD_DIR = os.environ.get("MINI_RELOAD_DIR", "")


def main():
    if not RELOAD_DIR:
        raise SystemExit("[reload] set MINI_RELOAD_DIR to a baked dir")
    manifest = os.path.join(RELOAD_DIR, "manifest.json")
    if not os.path.exists(manifest):
        raise SystemExit("[reload] no manifest at {0}".format(manifest))
    records = bake.read_manifest(manifest)

    omni.timeline.get_timeline_interface().stop()
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Scope.Define(stage, Sdf.Path("/World/baked"))
    app = omni.kit.app.get_app()

    # A ground plane + lights so the referenced objects are not floating in
    # the dark. This is scene dressing, not the thing under test.
    plane = UsdGeom.Mesh.Define(stage, "/World/ground")
    e = 200.0
    plane.CreatePointsAttr([Gf.Vec3f(-e, -e, 0), Gf.Vec3f(e, -e, 0),
                            Gf.Vec3f(e, e, 0), Gf.Vec3f(-e, e, 0)])
    plane.CreateFaceVertexCountsAttr([4])
    plane.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    plane.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    plane.CreateDisplayColorAttr([Gf.Vec3f(0.20, 0.30, 0.14)])
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/domeLight"))
    dome.CreateIntensityAttr(900.0)
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/keyLight"))
    key.CreateIntensityAttr(2400.0)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-46.0, 0.0, 32.0))

    # REFERENCE every baked object. Each file's prims carry their own world
    # transform, so an identity parent lands them where they were.
    t0 = time.time()
    n = 0
    for r in records:
        usd = r["usd"]
        if not os.path.exists(usd):
            print("[reload] MISSING {0}".format(usd))
            continue
        dst = "/World/baked/{0}_{1}".format(r["kind"], r["id"])
        prim = stage.DefinePrim(dst, "Xform")
        prim.GetReferences().AddReference(usd)
        n += 1
    # Pump until composition + assets finish loading.
    for _ in range(30):
        app.update()
    for _ in range(6000):
        try:
            st = ctx.get_stage_loading_status()
            if isinstance(st, (tuple, list)) and len(st) >= 3 and int(st[2]) == 0:
                break
        except Exception:
            break
        app.update()
    for _ in range(30):
        app.update()
    dt = time.time() - t0

    for _key in ("/rtx/raytracing/fractionalCutoutOpacity",
                 "/rtx/pathtracing/fractionalCutoutOpacity"):
        carb.settings.get_settings().set_bool(_key, True)
    for _ in range(5):
        app.update()

    miss = sum(int(r.get("bound_missing", 0)) for r in records)
    nh = sum(1 for r in records if r["kind"] == "house")
    nt = sum(1 for r in records if r["kind"] == "tree")
    print("\n" + "=" * 72)
    print("PER-OBJECT BAKED RELOAD")
    print("  dir         {0}".format(RELOAD_DIR))
    print("  referenced  {0} object(s): {1} house + {2} tree, in {3:.1f} s"
          .format(n, nh, nt, dt))
    print("  materials   {0} unresolved mesh binding(s) across all bakes"
          .format(miss))
    print("  compare     to the build's '[bake] full build ... took N s' line")
    print("=" * 72 + "\n")

    while simulation_app.is_running():
        app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
