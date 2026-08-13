#!/usr/bin/env python
"""
Preview launcher for the graph-based suburb (scene_gen/suburb_net.py).

Deliberately a near-copy of city_v2_preview_launch_script.py — same base
environment, same stage prep, same lighting, same clutter removal — so running
the two side by side isolates the thing under test: the suburb layout itself.
The only differences are which generator entry point is called and which config
it defaults to.

    SCENE_CONFIG=suburb_net \
    ISAAC_SIM_SCRIPT_NAME=suburb_preview_launch_script.py \
    airstack up isaac-sim

Compare against the RECT-subdivider suburb, which is a different generator and
stays runnable, with:

    SCENE_CONFIG=suburban_v2 \
    ISAAC_SIM_SCRIPT_NAME=city_v2_preview_launch_script.py \
    airstack up isaac-sim

No drone, no sensors, no ROS 2 bridge, so startup is fast and the scene can be
regenerated in place after editing the YAML — see RELOAD below.

WHAT YOU ARE LOOKING AT
-----------------------
Streets are polyline centrelines swept into ribbon meshes at their own width,
not axis-aligned rects, so they curve; blocks are the polygon faces between
them. Every junction was solved as a boundary-value problem — the side street
leaves its host along the host's normal — so roads meet cleanly rather than
being bridged by a patch of asphalt. Houses take their yaw from the frontage
tangent, so a row of houses turns with its street.

Check the plan on the HOST first if you only want to judge the layout; it needs
no Isaac Sim and takes about a second:

    python3 scene_gen/tools/suburb_net_png.py --seed 3 --out /tmp/suburb.png

RELOAD WITHOUT RESTARTING
--------------------------
Open the Kit Script Editor (Window -> Script Editor) and run:

    import os, sys, importlib
    SCENE_GEN = "/isaac-sim/AirStack/scene_gen"
    ISAAC_DIR = "/isaac-sim/AirStack/simulation/isaac-sim"
    sys.path[:0] = [SCENE_GEN, os.path.join(ISAAC_DIR, "utils")]
    import scene_generator, suburb_net, suburb_parcel, suburb_scene, scene_prep
    for m in (scene_generator, suburb_net, suburb_parcel, suburb_scene):
        importlib.reload(m)
    import omni.usd, omni.timeline
    from pxr import Sdf
    stage = omni.usd.get_context().get_stage()
    omni.timeline.get_timeline_interface().stop()
    stage.RemovePrim(Sdf.Path("/World/stage/generated"))   # clear the old build
    _, ssf = scene_prep.get_stage_meters_per_unit(stage)
    from compile_disaster import load_scene_config
    cfg = load_scene_config(os.path.join(SCENE_GEN, "config", "presets",
                                         "suburb_net.yaml"))
    suburb_scene.generate_suburb_on_stage(stage, cfg, scene_scale_factor=ssf)
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
from scene_prep import (scale_stage_prim, add_sky, get_stage_meters_per_unit)
from scene_generator import resolve_sky
from suburb_scene import generate_suburb_on_stage
from compile_disaster import load_scene_config

# ----- CONFIGURATION -----
ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]
STAGE_SCALE = 1.00
SCENE_CONFIG = os.environ.get("SCENE_CONFIG") or os.path.join(
    _SCENE_GEN_DIR, "config", "presets", "suburb_net.yaml")
# -------------------------


_ENV_CLUTTER = {"GroundPlane", "Environment"}


def _remove_env_clutter(stage):
    """Deactivate the GroundPlane and Environment xforms the base environment
    brings in. The generator lays its own ground, so these cause z-fighting and
    an unwanted visual backdrop.

    `RemovePrim` cannot delete across a reference — it returns False rather than
    raising — so deactivation is used instead, which composes and is reversible
    in the viewport. `SphereLight` is a sibling, not a child, so lighting
    survives. Same reasoning as city_v2_preview_launch_script.
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
            else:
                UsdGeom.Imageable(child).MakeInvisible()
    print(f"[suburb] env clutter: {n} prim(s) deactivated")


def _disable_sky_sun(stage):
    """Switch off the DistantLight the sky rig hangs under its axis chain.

    Runs after `add_sky`, which is what creates it; the dome is left alone
    because that is what lights the scene.
    """
    n = 0
    for prim in stage.Traverse():
        if prim.GetTypeName() != "DistantLight":
            continue
        if "/Environment/sky/" not in str(prim.GetPath()):
            continue
        if prim.IsActive() and prim.SetActive(False):
            n += 1
    print(f"[suburb] sky sun: {n} DistantLight(s) disabled")
    return n


def add_colliders_skip_empty(prim):
    """`scene_prep.add_colliders`, but skipping meshes that have no points.

    Some referenced props carry empty stub Meshes, and PhysX logs an error per
    empty mesh PER INSTANCE, which at suburb density is thousands of lines that
    bury real problems. A mesh with no points has nothing to collide with
    either way, so skipping changes nothing physically.
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
    print(f"[suburb] colliders: {applied} applied, {skipped} empty meshes "
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


class SuburbPreviewApp:

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
            for _ in range(10):
                omni.kit.app.get_app().update()
        else:
            carb.log_warn("/World/stage not found — skipping scale.")

        config = load_scene_config(SCENE_CONFIG)

        _, ssf = get_stage_meters_per_unit(stage)
        generate_suburb_on_stage(
            stage, config, parent_path="/World/stage/generated",
            scene_scale_factor=ssf)

        generated_prim = stage.GetPrimAtPath("/World/stage/generated")
        if generated_prim.IsValid():
            add_colliders_skip_empty(generated_prim)
        for _ in range(10):
            omni.kit.app.get_app().update()

        add_sky(stage, resolve_sky(config))
        _disable_sky_sun(stage)

        print("\n" + "=" * 70)
        print("SUBURB PREVIEW READY")
        print(f"  config: {SCENE_CONFIG}")
        print("  reload: see the snippet in this script's docstring")
        print("=" * 70 + "\n")

        self.timeline.play()

    def run(self):
        app = omni.kit.app.get_app()
        while simulation_app.is_running():
            app.update()
        self.timeline.stop()
        simulation_app.close()


def main():
    SuburbPreviewApp().run()


if __name__ == "__main__":
    main()
