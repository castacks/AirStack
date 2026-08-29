#!/usr/bin/env python3
"""Five standalone buildings with generic, disaster-neutral damage recipes."""

import importlib.util
import os
import sys
import time

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": os.environ.get(
    "ISAAC_SIM_HEADLESS", "false").strip().lower() in ("1", "true", "yes")})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.kit.window.script_editor")

import omni.kit.app
import omni.kit.viewport.utility as vp
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdLux

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     "..", "..", ".."))
ISAAC_DIR = os.path.join(ROOT, "simulation", "isaac-sim")
sys.path.insert(0, os.path.join(ROOT, "scene_gen"))
from disaster import monolith_damage as md

SNAP_DIR = os.environ.get("SNAP_DIR", "").strip()
KEEP_OPEN = os.environ.get("KEEP_OPEN", "1").strip() == "1"
SEED = int(os.environ.get("MONO_SEED", "17") or 17)

ASSETS = [
    ("tower_corner", "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/selected_citydemo/tower/tower_01_0006.usd", (36.233, 49.376, 106.256), "corner_loss", "urm"),
    ("tower_roof", "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/selected_citydemo/tower/tower_03_0015.usd", (32.872, 50.414, 83.296), "roof_collapse", "rc"),
    ("midrise_soft", "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/selected_citydemo/midrise/midrise_02_0059a.usd", (19.531, 13.804, 16.318), "soft_storey", "rc"),
    ("midrise_mid", "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/selected_citydemo/midrise/midrise_04_0082.usd", (35.065, 18.076, 15.668), "mid_storey", "rc"),
    ("midrise_partial", "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/selected_citydemo/midrise/midrise_05_0097a.usd", (15.983, 20.501, 20.907), "partial_collapse", "urm"),
]


def ground_light(stage):
    g = UsdGeom.Cube.Define(stage, "/World/ground")
    g.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(g)
    xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.15))
    xf.AddScaleOp().Set(Gf.Vec3f(650, 350, 0.3))
    g.CreateDisplayColorAttr([Gf.Vec3f(0.26, 0.27, 0.28)])
    dome = UsdLux.DomeLight.Define(stage, "/World/domeLight")
    dome.CreateIntensityAttr(900.0)
    dome.CreateColorAttr(Gf.Vec3f(0.78, 0.83, 0.92))
    key = UsdLux.DistantLight.Define(stage, "/World/keyLight")
    key.CreateIntensityAttr(3000.0)
    key.CreateAngleAttr(0.7)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-48, 0, 32))


def main():
    t0 = time.time()
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    ground_light(stage)
    xs = (-120.0, -60.0, 0.0, 55.0, 110.0)
    rows = []
    for i, ((name, url, dims, recipe, construction), x) in enumerate(zip(ASSETS, xs)):
        root = "/World/Buildings/b{}".format(i)
        holder = UsdGeom.Xform.Define(stage, root)
        holder.AddTranslateOp().Set(Gf.Vec3d(x, 0, 0))
        asset = stage.DefinePrim(root + "/source")
        asset.GetReferences().AddReference(url)
        asset.Load()
        d = md.Descriptor(dims[0], dims[1], dims[2], construction=construction)
        result = md.author(stage, root + "/damage", d, recipe, SEED+i,
                           source_root=root + "/source")
        rows.append(dict(i=i, name=name, recipe=recipe, x=x, W=d.width,
                         D=d.depth, H=d.height, rubble=result["rubble_count"]))
        print("[monolith_bench] {} {}: {} rubble pieces; cut {} meshes / {} faces".format(
            name, recipe, result["rubble_count"], result["cut"]["meshes"],
            result["cut"]["faces_removed"]))
    for _ in range(12):
        omni.kit.app.get_app().update()

    if SNAP_DIR:
        sp = os.path.join(ISAAC_DIR, "utils", "snapshots.py")
        spec = importlib.util.spec_from_file_location("snapshots", sp)
        snaps = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(snaps)
        os.makedirs(SNAP_DIR, exist_ok=True)
        snaps.overview(stage, (0, 0), 310, os.path.join(SNAP_DIR, "row_top.png"), 1.0)
        snaps.place_camera(stage, (-175, -245, 150), (0, 0, 35))
        snaps.snapshot(os.path.join(SNAP_DIR, "row_oblique.png"))
        for r in rows:
            d = 1.25*max(r["W"], r["D"], 0.65*r["H"])
            views = {
                "front": ((r["x"], -r["D"]/2-d, 0.42*r["H"]), (r["x"], -r["D"]/2, 0.42*r["H"])),
                "oblique": ((r["x"]-0.65*d, -r["D"]/2-0.85*d, 0.62*r["H"]), (r["x"], 0, 0.38*r["H"])),
                "top": ((r["x"], -0.15*r["D"], r["H"]+1.25*d), (r["x"], 0, 0.35*r["H"])),
                "close": ((r["x"]-0.15*r["W"], -r["D"]/2-9, 2.0), (r["x"]+0.08*r["W"], -r["D"]/2, min(6.0, 0.30*r["H"]))),
            }
            for vname, (eye, target) in views.items():
                cpath = "/World/ReviewCams/b{}_{}".format(r["i"], vname)
                cam = UsdGeom.Camera.Define(stage, Sdf.Path(cpath))
                cam.GetHorizontalApertureAttr().Set(20.955)
                cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.3, 20000))
                cam.GetFocalLengthAttr().Set(22.0 if vname == "close" else 18.0)
                xf = UsdGeom.Xformable(cam)
                xf.AddTranslateOp().Set(Gf.Vec3d(*eye))
                xf.AddRotateXYZOp().Set(snaps._look_at(eye, target))
                vp.get_active_viewport().camera_path = cpath
                snaps.snapshot(os.path.join(SNAP_DIR, "{}_{}.png".format(r["name"], vname)))
        print("[monolith_bench] snapshots -> " + SNAP_DIR)
    print("="*72)
    print("GENERIC MONOLITH DAMAGE BENCH READY in {:.1f} s".format(time.time()-t0))
    for r in rows:
        print("  {name:<18} {recipe:<18} rubble={rubble}".format(**r))
    print("="*72)
    print("MONOLITH BENCH DONE")
    if KEEP_OPEN:
        timeline.play()
        while simulation_app.is_running():
            omni.kit.app.get_app().update()
        timeline.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
