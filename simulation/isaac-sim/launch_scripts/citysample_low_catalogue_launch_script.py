#!/usr/bin/env python
"""Open CitySampleLow's authored all-buildings lineup for visual review."""

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.kit.window.script_editor")

import omni.kit.app
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux

URL = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
       "CitySampleLow/All_Buildings_Lineup_Reference.usd")


def main():
    context = omni.usd.get_context()
    context.open_stage(URL)
    for _ in range(180):
        omni.kit.app.get_app().update()
    stage = context.get_stage()
    if not stage:
        raise RuntimeError("CitySampleLow lineup did not open: " + URL)
    # The imported Unreal lineup contains a 13-km reversed lighting sphere.
    # Its bound poisons "frame all", and its dark interior is what made the
    # viewport look black. Hide it and author neutral review lighting/camera in
    # the session layer, leaving the Nucleus stage untouched.
    stage.SetEditTarget(stage.GetSessionLayer())
    old_light = stage.GetPrimAtPath("/Root/Light")
    if old_light:
        old_light.SetActive(False)
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/Review/Dome"))
    dome.CreateIntensityAttr(900.0)
    dome.CreateColorAttr(Gf.Vec3f(0.78, 0.82, 0.90))
    sun = UsdLux.DistantLight.Define(stage, Sdf.Path("/Review/Sun"))
    sun.CreateIntensityAttr(3200.0)
    sun.CreateAngleAttr(0.7)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(-38.0, 0.0, 28.0))

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    ranges = []
    root = stage.GetPrimAtPath("/Root")
    for prim in root.GetChildren():
        if not prim.GetName().startswith("BPP_"):
            continue
        bound = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if not bound.IsEmpty():
            ranges.append(bound)
    mn = Gf.Vec3d(*[min(r.GetMin()[i] for r in ranges) for i in range(3)])
    mx = Gf.Vec3d(*[max(r.GetMax()[i] for r in ranges) for i in range(3)])
    centre = 0.5 * (mn + mx)
    span = max(mx[0] - mn[0], mx[1] - mn[1])

    import importlib.util
    camera_api = ("/isaac-sim/AirStack/simulation/isaac-sim/utils/"
                  "snapshots.py")
    spec = importlib.util.spec_from_file_location("snapshots", camera_api)
    snapshots = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(snapshots)
    snapshots.place_camera(
        stage,
        (centre[0] - span * 0.42, centre[1] - span * 0.42,
         max(mx[2] * 1.5, span * 0.52)),
        (centre[0], centre[1], max(0.0, mx[2] * 0.12)),
        focal_mm=18.0)
    review_camera = UsdGeom.Camera(stage.GetPrimAtPath("/World/ReviewCamera"))
    review_camera.CreateClippingRangeAttr(Gf.Vec2f(1.0, span * 5.0))
    snapshots.hide_decorations()
    for _ in range(90):
        omni.kit.app.get_app().update()

    prim_count = sum(1 for _ in stage.Traverse())
    print("\n" + "=" * 78)
    print("CITYSAMPLE LOW BUILDING CATALOGUE READY")
    print("  source: " + URL)
    print("  composed prims: {0}".format(prim_count))
    print("  building bounds: {0} -> {1}".format(tuple(mn), tuple(mx)))
    print("=" * 78 + "\n")
    while simulation_app.is_running():
        omni.kit.app.get_app().update()
    simulation_app.close()


if __name__ == "__main__":
    main()
