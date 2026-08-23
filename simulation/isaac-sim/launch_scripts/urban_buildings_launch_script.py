#!/usr/bin/env python
"""
Five urban buildings assembled piece by piece from a modular façade kit
(scene_gen/detail/urban_building.py), on an otherwise empty stage.

    ISAAC_SIM_SCRIPT_NAME=urban_buildings_launch_script.py airstack up isaac-sim

One building per kit family — apartment block, office, brownstone, glass
commercial, podium-and-tower — in a row along +X with their fronts facing -Y,
so a camera south of the origin sees all five. No drone, no sensors, no ROS 2
bridge: this boots fast and is only about whether the kit assembles.

THE KIT. The request was `/Projects/SEI-COA/ProceduralBuildingGenerator/`;
that folder holds Unreal Engine 4.22 `.uasset` files, which Isaac Sim cannot
reference, so the buildings come from `/Projects/SEI-COA/ModernCityEnvironment01/`
(the same pack family, already exported to USD). Swapping kits is a table edit
in `urban_building.py` — see its docstring.

Environment:
    URBAN_SEED        RNG seed for façade variety (default 7)
    URBAN_GAP         metres between buildings (default 8)
    URBAN_COLLIDERS   0 to skip PhysX colliders (default on — ~450 gprims)
    URBAN_STRIP       1 to also lay out the kit's ambiguous pieces in a strip
                      behind the buildings, for reading their frames off a render
    URBAN_SNAP_DIR    write viewport captures (front, aerial, per building)
                      to this directory once the scene is up, then keep running

RELOAD WITHOUT RESTARTING
-------------------------
Open the Kit Script Editor (Window -> Script Editor) and run:

    import os, sys, importlib, random
    SCENE_GEN = "/isaac-sim/AirStack/scene_gen"
    ISAAC_DIR = "/isaac-sim/AirStack/simulation/isaac-sim"
    sys.path[:0] = [SCENE_GEN, os.path.join(ISAAC_DIR, "utils")]
    import scene_generator, scene_prep
    from detail import urban_building
    for m in (scene_generator, urban_building):
        importlib.reload(m)
    import omni.usd, omni.timeline
    from pxr import Sdf
    stage = omni.usd.get_context().get_stage()
    omni.timeline.get_timeline_interface().stop()
    stage.RemovePrim(Sdf.Path("/World/stage/generated"))
    _, ssf = scene_prep.get_stage_meters_per_unit(stage)
    pl, where = urban_building.build_street(random.Random(7))
    scene_generator.apply_placements(stage, pl, "/World/stage/generated", ssf)
    omni.timeline.get_timeline_interface().play()
"""

import os
import random
import sys
import time

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("omni.kit.window.script_editor")

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from omni.isaac.core.world import World
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

from scene_prep import scale_stage_prim, add_sky, get_stage_meters_per_unit
import scene_generator as sg
from detail import urban_building

# ----- CONFIGURATION -----
ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]
STAGE_SCALE = 1.00
SEED = int(os.environ.get("URBAN_SEED", "7"))
GAP_M = float(os.environ.get("URBAN_GAP", "8"))
COLLIDERS = os.environ.get("URBAN_COLLIDERS", "1") not in ("0", "false", "no")
STRIP = os.environ.get("URBAN_STRIP", "0") in ("1", "true", "yes")
SNAP_DIR = os.environ.get("URBAN_SNAP_DIR", "").strip()
PARENT = "/World/stage/generated"
CAM = "/World/urbanCam"
# -------------------------


def add_colliders_skip_empty(prim):
    """`scene_prep.add_colliders`, skipping point-less stub meshes."""
    applied = skipped = 0
    for p in Usd.PrimRange(prim):
        if not p.IsA(UsdGeom.Gprim):
            continue
        if p.IsA(UsdGeom.Mesh):
            pts = p.GetAttribute("points")
            if not pts or not pts.HasAuthoredValue() or not (pts.Get() or []):
                skipped += 1
                continue
        if not p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(p)
            applied += 1
    print(f"[urban] colliders: {applied} applied, {skipped} empty skipped")
    return applied, skipped


def report_missing(stage, placements):
    """Name any placement that resolved to nothing (a payload that did not load)."""
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    empty = {}
    for p in placements:
        path = p.get("prim_path")
        if not path:
            continue
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            continue
        if bc.ComputeWorldBound(prim).ComputeAlignedRange().IsEmpty():
            empty[p["usd"]] = empty.get(p["usd"], 0) + 1
    if empty:
        print(f"[urban] WARN {sum(empty.values())} placements drew nothing:")
        for usd, n in sorted(empty.items(), key=lambda kv: -kv[1]):
            print(f"[urban]   {n:4d} x {usd.rsplit('/', 1)[-1]}")
    else:
        print("[urban] every placement drew geometry")
    return empty


def wait_for_stage(stage, timeout_s=10.0):
    for _ in range(int(timeout_s / 0.1)):
        omni.kit.app.get_app().update()
        world_prim = stage.GetPrimAtPath("/World")
        if world_prim.IsValid():
            if [c for c in world_prim.GetChildren() if c.GetName() != "PhysicsScene"]:
                return True
        time.sleep(0.1)
    return False


def _look_at(eye, target):
    """rotateXYZ for a camera at *eye* looking at *target*, Z up."""
    import math
    dx, dy, dz = (target[i] - eye[i] for i in range(3))
    yaw = math.degrees(math.atan2(dy, dx)) - 90.0
    pitch = math.degrees(math.atan2(dz, math.hypot(dx, dy)))
    return Gf.Vec3f(90.0 + pitch, 0.0, yaw)


def aim_camera(stage, eye, target):
    cam = UsdGeom.Camera.Get(stage, Sdf.Path(CAM))
    if not cam:
        cam = UsdGeom.Camera.Define(stage, Sdf.Path(CAM))
        cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.5, 5000.0))
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*eye))
    xf.AddRotateXYZOp().Set(_look_at(eye, target))
    try:
        import omni.kit.viewport.utility as vp
        vp.get_active_viewport().camera_path = CAM
    except Exception as exc:
        carb.log_warn(f"could not retarget the viewport: {exc}")


def snapshot(path, frames=30):
    """Capture the active viewport to *path* after *frames* updates."""
    import omni.kit.viewport.utility as vp
    app = omni.kit.app.get_app()
    for _ in range(frames):
        app.update()
    vp.capture_viewport_to_file(vp.get_active_viewport(), path)
    for _ in range(frames):
        app.update()
    print(f"[urban] snapshot -> {path}")


class UrbanBuildingsApp:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()
        self.timeline.stop()

        pg = PegasusInterface()
        pg._world = World(**pg._world_settings)
        pg.load_environment(ENV_URL)

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage failed to load")
        if not wait_for_stage(stage):
            carb.log_warn("Stage load timed out — continuing anyway.")
        self.stage = stage

        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            scale_stage_prim(stage, "/World/stage", STAGE_SCALE)
            for _ in range(10):
                omni.kit.app.get_app().update()

        problems = urban_building.check()
        if problems:
            raise RuntimeError("urban_building.check() failed: " + "; ".join(problems))

        rng = random.Random(SEED)
        placements, where = urban_building.build_street(rng, gap_m=GAP_M)
        if STRIP:
            placements += urban_building.build_prop_strip(-60.0, 40.0)
        print("[urban] " + urban_building.summarise(placements)
              .replace("\n", "\n[urban] "))
        self.where = where

        _, ssf = get_stage_meters_per_unit(stage)
        sg.apply_placements(stage, placements, PARENT, ssf)
        for _ in range(10):
            omni.kit.app.get_app().update()

        if COLLIDERS:
            gen = stage.GetPrimAtPath(PARENT)
            if gen.IsValid():
                add_colliders_skip_empty(gen)
        for _ in range(10):
            omni.kit.app.get_app().update()

        report_missing(stage, placements)
        add_sky(stage, "")

        self.poses = [("front", (0.0, -95.0, 28.0), (0.0, 0.0, 10.0)),
                      ("aerial", (0.0, -70.0, 90.0), (0.0, 0.0, 0.0)),
                      ("east_end", (95.0, -60.0, 35.0), (40.0, 0.0, 12.0))]
        for s, x, w, d in where:
            self.poses.append((s, (x + w * 0.9, -(d / 2 + max(w, 22.0) * 1.1), 14.0),
                               (x, 0.0, 8.0)))
        aim_camera(stage, *self.poses[0][1:])

        print("\n" + "=" * 70)
        print("URBAN BUILDINGS READY")
        print(f"  seed {SEED}   gap {GAP_M} m   colliders {'on' if COLLIDERS else 'off'}"
              f"   strip {'on' if STRIP else 'off'}")
        for s, x, w, d in where:
            print(f"    {s:<18} x={x:+7.1f}  {w:.0f} x {d:.0f} m   "
                  f"{urban_building.STYLES[s]['note']}")
        print("  reload: see the snippet in this script's docstring")
        print("=" * 70 + "\n")
        self.timeline.play()

    def run(self):
        app = omni.kit.app.get_app()
        if SNAP_DIR:
            os.makedirs(SNAP_DIR, exist_ok=True)
            for _ in range(60):
                app.update()
            for name, eye, target in self.poses:
                aim_camera(self.stage, eye, target)
                snapshot(os.path.join(SNAP_DIR, f"{name}.png"))
            aim_camera(self.stage, *self.poses[0][1:])
            print(f"[urban] {len(self.poses)} snapshots in {SNAP_DIR}")
        while simulation_app.is_running():
            app.update()
        self.timeline.stop()
        simulation_app.close()


def main():
    UrbanBuildingsApp().run()


if __name__ == "__main__":
    main()
