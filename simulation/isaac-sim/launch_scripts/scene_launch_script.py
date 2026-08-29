#!/usr/bin/env python
"""
Preview launcher for the detailed city (scene_gen/generate_scene.py).

Started as a near-copy of the plain scene preview launcher — same base
environment, same stage prep, same lighting — and absorbed it when the v1/v2
split was retired: it was a superset, so there is nothing the other did that
this does not.

    SCENE_CONFIG=downtown \
    ISAAC_SIM_SCRIPT_NAME=scene_launch_script.py \
    airstack up isaac-sim

No drone, no sensors, no ROS 2 bridge, so startup is fast and the scene can be
regenerated in place after editing the YAML — see RELOAD below.

RELOAD WITHOUT RESTARTING
--------------------------
Open the Kit Script Editor (Window -> Script Editor) and run:

    import os, sys, importlib
    SCENE_GEN = "/isaac-sim/AirStack/scene_gen"
    ISAAC_DIR = "/isaac-sim/AirStack/simulation/isaac-sim"
    sys.path[:0] = [SCENE_GEN, os.path.join(ISAAC_DIR, "utils")]
    import scene_generator
    from detail import city_detail, districts, road_markings
    from layout import city_layout
    import generate_scene, scene_prep
    for m in (scene_generator, city_detail, city_layout, districts,
              road_markings, generate_scene):
        importlib.reload(m)          # pick up edits to the new passes too
    import omni.usd, omni.timeline
    stage = omni.usd.get_context().get_stage()
    omni.timeline.get_timeline_interface().stop()
    _, ssf = scene_prep.get_stage_meters_per_unit(stage)
    # add_colliders_skip_empty, not scene_prep.add_colliders — the stock one
    # floods the log with PhysX errors for point-less stub meshes.
    import scene_launch_script as sl
    generate_scene.reload_scene_on_stage(
        stage, os.path.join(SCENE_GEN, "config", "presets", "downtown.yaml"),
        scene_scale_factor=ssf, add_colliders_fn=sl.add_colliders_skip_empty)
    omni.timeline.get_timeline_interface().play()
"""

import os
import sys
import time

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

from isaacsim.core.utils.extensions import enable_extension
# The reload loop above depends on the Script Editor every run.
enable_extension("omni.kit.window.script_editor")

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics
from omni.isaac.core.world import World
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)
from scene_prep import (scale_stage_prim, add_colliders, add_sky,
                        get_stage_meters_per_unit, settle_rigid_props)
from scene_generator import resolve_sky
from generate_scene import generate_scene_on_stage
from compile_disaster import load_scene_config

# ----- CONFIGURATION -----
ENV_URL     = SIMULATION_ENVIRONMENTS["Default Environment"]
STAGE_SCALE = 1.00
SCENE_CONFIG = os.environ.get("SCENE_CONFIG") or os.path.join(
    _SCENE_GEN_DIR, "config", "presets", "downtown.yaml")
# -------------------------


def _spec_overrides():
    """Per-run size override, matching the generated-scene drone launcher."""
    raw = (os.environ.get("REGION_M") or "").strip()
    if not raw:
        return None
    parts = [p.strip() for p in raw.replace("x", ",").replace("X", ",").split(",")
             if p.strip()]
    try:
        vals = [float(p) for p in parts]
    except ValueError:
        raise SystemExit("REGION_M={0!r}: expected N, NxN or N,N".format(raw))
    if not vals:
        raise SystemExit("REGION_M={0!r}: expected N, NxN or N,N".format(raw))
    vals = [vals[0], vals[0]] if len(vals) == 1 else vals[:2]
    if any(v <= 0.0 for v in vals):
        raise SystemExit("REGION_M sides must be positive: {0}".format(vals))
    print("[scene_gen] spec override: region_m={0}".format(vals))
    return {"region_m": vals}


_ENV_CLUTTER = {"GroundPlane", "Environment"}


def _remove_env_clutter(stage):
    """Deactivate the GroundPlane and Environment xforms the base environment
    brings in. The generator lays its own ground, so these cause z-fighting and
    an unwanted visual backdrop — `Environment/Geometry` is the grid mesh that
    reads as a blue square under the spawn point.

    `default_environment.usd` has defaultPrim `/World`, which Pegasus references
    at `/World/stage`, so everything here composes from a referenced layer.
    `RemovePrim` cannot delete across a reference: it returns False rather than
    raising, so a try/except around it never fires and the prim stays visible.
    Deactivation does compose, and is reversible in the viewport.

    `SphereLight` is a sibling of these, not a child, so lighting survives.
    """
    n = 0
    for root_path in ("/", "/World", "/World/stage"):
        root = (stage.GetPseudoRoot() if root_path == "/"
                else stage.GetPrimAtPath(root_path))
        if not root or not root.IsValid():
            continue
        for child in root.GetChildren():
            if child.GetName() not in _ENV_CLUTTER or not child.IsActive():
                continue
            if child.SetActive(False):
                n += 1
                carb.log_info(f"[scene_gen] deactivated {child.GetPath()}")
            else:
                UsdGeom.Imageable(child).MakeInvisible()
                carb.log_info(f"[scene_gen] hid {child.GetPath()}")
    print(f"[scene_gen] env clutter: {n} prim(s) deactivated")


def _disable_sky_sun(stage):
    """Switch off the DistantLight the sky rig hangs under its axis chain.

    `add_sky` builds /World/Environment/sky/AxisNorth/.../DistantLight — a hard
    directional sun on top of the dome. It runs AFTER `_remove_env_clutter`, so
    the clutter pass cannot see it; this has to come after `add_sky` instead.
    The dome itself is left alone, since that is what lights the scene.
    """
    n = 0
    for prim in stage.Traverse():
        if prim.GetTypeName() != "DistantLight":
            continue
        if "/Environment/sky/" not in str(prim.GetPath()):
            continue
        if prim.IsActive() and prim.SetActive(False):
            n += 1
            carb.log_info(f"[scene_gen] disabled sun {prim.GetPath()}")
    print(f"[scene_gen] sky sun: {n} DistantLight(s) disabled")
    return n


def add_colliders_skip_empty(prim):
    """`scene_prep.add_colliders`, but skipping meshes that have no points.

    The stock version applies UsdPhysics.CollisionAPI to every gprim it finds.
    Some referenced props carry empty stub Meshes — `Planter_03`, pulled in by
    the Tower pack's `Bench_01`, is the one that shows up here — and PhysX then
    logs "Provided mesh geom with a PhysicsCollisionAPI does not have points,
    collision will not be created" once per empty mesh **per instance**, which
    at city density is thousands of error lines that bury real problems.

    Skipping them changes nothing physically: a mesh with no points has no
    geometry to collide with either way. Returns (applied, skipped).
    """
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
    print(f"[scene_gen] colliders: {applied} applied, {skipped} empty meshes "
          f"skipped")
    return applied, skipped


def wait_for_stage(stage, timeout_s: float = 10.0):
    for _ in range(int(timeout_s / 0.1)):
        omni.kit.app.get_app().update()
        world_prim = stage.GetPrimAtPath("/World")
        if world_prim.IsValid():
            non_physics = [c for c in world_prim.GetChildren()
                           if c.GetName() != "PhysicsScene"]
            if non_physics:
                return True
        time.sleep(0.1)
    return False


class CityV2PreviewApp:

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

        _remove_env_clutter(stage)

        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            scale_stage_prim(stage, "/World/stage", STAGE_SCALE)
            # colliders off — see the note below
            # add_colliders_skip_empty(stage_prim)
            for _ in range(10):
                omni.kit.app.get_app().update()
        else:
            carb.log_warn("/World/stage not found — skipping scale and collision.")

        config = load_scene_config(SCENE_CONFIG,
                                   spec_overrides=_spec_overrides())

        _, ssf = get_stage_meters_per_unit(stage)
        placements = generate_scene_on_stage(
            stage, config, parent_path="/World/stage/generated",
            scene_scale_factor=ssf)

        # PHYSICS COLLIDERS ARE OFF, deliberately.
        #
        # `add_colliders_skip_empty` applies UsdPhysics.CollisionAPI to EVERY
        # gprim under the generated root — 76,668 of them on a full suburb —
        # and PhysX then cooks collision for all of them inside the FIRST
        # `app.update()`. That one frame ran long enough that the window stopped
        # responding, X invalidated it (`BadWindow`), and Kit tore down a render
        # thread through pybind11 without the GIL, which aborts the process.
        #
        # Nothing here needs physics yet: this is a capture scene, and neither
        # rendering nor LiDAR reads CollisionAPI. Re-enable when something has
        # to fly into the geometry — and filter by category first, because a
        # drone needs to collide with building shells, trees and poles, not with
        # hedges, bins, chairs, paving slabs or window frames.
        #
        # if generated_prim.IsValid():
        #     add_colliders_skip_empty(generated_prim)
        for _ in range(10):
            omni.kit.app.get_app().update()

        settle_rigid_props(
            stage,
            [p["prim_path"] for p in placements
             if p.get("settle") and p.get("prim_path")],
            ground_path="/World/stage/generated/ground",
        )

        add_sky(stage, resolve_sky(config))
        _disable_sky_sun(stage)

        print("\n" + "=" * 70)
        print("CITY V2 PREVIEW READY")
        print(f"  config: {SCENE_CONFIG}")
        print("  reload: see the snippet in this script's docstring")
        print("=" * 70 + "\n")

        self.timeline.play()

    def snapshot_city(self, out_dir):
        """Nadir, four obliques and two street-level views of the region.

        A GENERATED CITY CANNOT BE REVIEWED FROM ITS LOG. The counts say 26
        blocks and 188 buildings whether or not a single one faces the right
        way, and this launcher wrote no captures at all — so every check of a
        `downtown` scene meant a person at the GUI. Framed off the REGION and
        the tallest thing standing, because a nadir framed off the region alone
        turns into a close-up of one roof when a 150 m tower is on the plate.
        """
        import importlib.util as _ilu
        sp = os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py")
        spec = _ilu.spec_from_file_location("snapshots", sp)
        sn = _ilu.module_from_spec(spec)
        spec.loader.exec_module(sn)
        os.makedirs(out_dir, exist_ok=True)
        stage = omni.usd.get_context().get_stage()
        bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                               [UsdGeom.Tokens.default_])
        r = bc.ComputeWorldBound(
            stage.GetPseudoRoot()).ComputeAlignedRange()
        if r.IsEmpty():
            print("[scene] nothing to capture")
            return
        mn, mx = r.GetMin(), r.GetMax()
        cx, cy = 0.5 * (mn[0] + mx[0]), 0.5 * (mn[1] + mx[1])
        span = max(mx[0] - mn[0], mx[1] - mn[1])
        tall = mx[2]
        sn.place_camera(stage, (cx, cy, tall + span / 1.10), (cx, cy, 0.0))
        sn.snapshot(os.path.join(out_dir, "city_top.png"))
        for nm, (ax, ay) in (("sw", (-1, -1)), ("se", (1, -1)),
                             ("ne", (1, 1)), ("nw", (-1, 1))):
            d = span * 0.78
            sn.place_camera(stage, (cx + ax * d, cy + ay * d,
                                    0.42 * d + tall * 0.55),
                            (cx, cy, tall * 0.22))
            sn.snapshot(os.path.join(out_dir, "city_" + nm + ".png"))
        # STREET VIEWS FROM OUTSIDE THE PLATE, NOT FROM ITS CENTRE. Aimed at
        # the middle of a generated city the camera lands inside whatever
        # building happens to be there — the first pair came back solid black
        # with a lit soffit at the top of frame. Backing off past the edge and
        # looking in along the axis always has open ground in front of it.
        for nm, eye, tgt in (
                ("street_ew", (cx - span * 0.62, cy, 6.0),
                 (cx + span * 0.5, cy, 26.0)),
                ("street_ns", (cx, cy - span * 0.62, 6.0),
                 (cx, cy + span * 0.5, 26.0))):
            sn.place_camera(stage, eye, tgt)
            sn.snapshot(os.path.join(out_dir, nm + ".png"))
        print("[scene] snapshots -> {0}".format(out_dir))

    def run(self):
        app = omni.kit.app.get_app()
        snap = (os.environ.get("SNAP_DIR") or "").strip()
        if snap:
            for _ in range(120):        # let the renderer converge first
                app.update()
            try:
                self.snapshot_city(snap)
            except Exception as exc:
                import traceback
                traceback.print_exc()
                print("[scene] snapshots FAILED: {0}".format(exc))
        while simulation_app.is_running():
            app.update()
        self.timeline.stop()
        simulation_app.close()


def main():
    CityV2PreviewApp().run()


if __name__ == "__main__":
    main()
