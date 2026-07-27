#!/usr/bin/env python
"""
Scene-generator preview for a LOCAL Isaac Sim installation — no AirStack
containers, no Pegasus extension required.

Creates an empty Z-up stage (no base environment), runs the procedural city
generator on it, then sits in a render loop. Same reload workflow as the
containerized scene_preview_launch_script.py.

USAGE
-----
Run with the python bundled in your Isaac Sim install:

    cd ~/isaacsim            # or wherever Isaac Sim is installed
    ./python.sh /path/to/AirStack/simulation/isaac-sim/launch_scripts/local_scene_preview_launch_script.py

(Windows: python.bat instead of python.sh.)

The asset library lives on Nucleus (see asset_root in the config), so the
machine needs network access to that server; log in via the browser prompt
if one appears on first asset fetch.

RELOAD WITHOUT RESTARTING
-------------------------
Edit the scene config (see SCENE_CONFIG below) or utils/scene_generator.py, then in
the Kit Script Editor (Window -> Script Editor) run the one-liner printed to
the terminal at startup (it exec's reload_scene.py from this repo).
"""

import os
import sys
import time

import carb
from isaacsim import SimulationApp

# Must be created before any omni imports.
simulation_app = SimulationApp(launch_config={"headless": False})

from isaacsim.core.utils.extensions import enable_extension
# The reload loop (see module docstring and reload_scene.py) runs through the
# Script Editor, so make sure it's available every run.
enable_extension("omni.kit.window.script_editor")

import omni.kit.app
import omni.usd
from pxr import Sdf, UsdGeom

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
from scene_prep import (add_colliders, add_sky, get_stage_meters_per_unit,
                        settle_rigid_props)
from scene_generator import generate_scene_on_stage, resolve_sky
# Accepts a config at either level: a high-level disaster spec is
# compiled in memory, a low-level scene config is used as is.
from compile_disaster import load_scene_config

# Let reload_scene.py locate this repo when exec'd from the Script Editor
# (the Script Editor copies scripts to /tmp, so __file__ can't be trusted there).
os.environ["AIRSTACK_ISAAC_SIM_DIR"] = _ISAAC_SIM_DIR

# ----- CONFIGURATION -----
SCENE_CONFIG = os.path.join(_ISAAC_SIM_DIR, "config", "scene_generation",
                            "low_level", "compiled", "earthquake.yaml")
APPLY_COLLIDERS = False   # not needed for visual iteration; True to test physics
# -------------------------


class LocalScenePreviewApp:

    def __init__(self):
        usd_ctx = omni.usd.get_context()
        usd_ctx.new_stage()
        stage = usd_ctx.get_stage()
        if stage is None:
            raise RuntimeError("Failed to create a new stage")

        # Metric Z-up stage; the generator's scene_scale_factor handles any
        # other unit convention, but a fresh stage is ours to define.
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
        stage.SetDefaultPrim(world.GetPrim())
        # Parent matches the containerized scripts so reload_scene.py's default
        # parent_path ("/World/stage/generated") clears/rebuilds the same subtree.
        UsdGeom.Xform.Define(stage, Sdf.Path("/World/stage"))

        config = load_scene_config(SCENE_CONFIG)

        _, ssf = get_stage_meters_per_unit(stage)
        placements = generate_scene_on_stage(stage, config,
                                             parent_path="/World/stage/generated",
                                             scene_scale_factor=ssf)

        if APPLY_COLLIDERS:
            generated_prim = stage.GetPrimAtPath("/World/stage/generated")
            if generated_prim.IsValid():
                add_colliders(generated_prim)

        # Pump the app loop so the renderer picks up the new prims.
        for _ in range(10):
            omni.kit.app.get_app().update()

        # Physics-settle toppled/strewn props (ground colliders are ensured
        # by settle_rigid_props itself, so this works with APPLY_COLLIDERS
        # off too).
        settle_rigid_props(
            stage,
            [p["prim_path"] for p in placements
             if p.get("settle") and p.get("prim_path")],
            ground_path="/World/stage/generated/ground",
        )

        add_sky(stage, resolve_sky(config))

        _reload_script = os.path.join(_ISAAC_SIM_DIR, "reload_scene.py")
        print("\n" + "=" * 70)
        print("LOCAL SCENE PREVIEW READY — to reload, open Window → Script Editor and run:")
        print(f'  exec(open({repr(_reload_script)}).read())')
        print("Edit the scene config or reload_scene.py between runs.")
        print("=" * 70 + "\n")

    def run(self):
        app = omni.kit.app.get_app()
        while simulation_app.is_running():
            app.update()
        simulation_app.close()


def main():
    LocalScenePreviewApp().run()


if __name__ == "__main__":
    main()
