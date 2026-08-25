#!/usr/bin/env python
"""
Entrypoint 2 — open a scene that was already baked, and run Stage C on it.

SPEC.md's second entrypoint is "simply use a link to a stage": the city was
assembled and damaged offline by `scene_gen/bake_scene.py`, so loading it costs
a `Usd.Stage.Open` instead of a full generation pass. What still has to happen
at load time is Stage C:

    1. load the pre-baked scene                     (here)
    2. place the targets, recording where they are  (scene_gen/targets.py)
    3. apply whatever the USD could not carry       (Disaster.attach_runtime)

Step 2 is why this script exists rather than pointing Pegasus at the USD as an
`ENV_URL`: victims are deliberately NOT baked, so one baked city can be
searched again and again with a fresh population. Step 3 is a real gap the
`ENV_URL` route has — a baked fire scene loaded that way renders nothing,
because `rtx/flow/*` is renderer state, not scene description.

USAGE
-----
    SCENE_USD=/path/to/scene.usd \\
    SCENE_CONFIG=urban_quake_showcase \\
    ISAAC_SIM_SCRIPT_NAME=baked_scene_launch_script.py airstack up isaac-sim

`SCENE_CONFIG` is the config the scene was baked from — Stage C needs it for
the damage field, the asset pack's human assets and the `targets:` block. With
`SCENE_USD` unset the scene cache is asked for the USD that config bakes to,
which is the normal path:

    python3 scene_gen/bake_scene.py --config urban_quake_showcase
    SCENE_CONFIG=urban_quake_showcase ISAAC_SIM_SCRIPT_NAME=... airstack up isaac-sim

Re-roll the people on the same city without re-baking anything:

    TARGET_SEED=7 SCENE_CONFIG=urban_quake_showcase ... airstack up isaac-sim
"""

import os
import sys

import carb
from isaacsim import SimulationApp

# Must be created before any omni imports.
simulation_app = SimulationApp(launch_config={"headless": False})

import omni.kit.app
import omni.usd
from pxr import Sdf, UsdGeom

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

from scene_prep import add_colliders, add_sky, get_stage_meters_per_unit
from scene_generator import resolve_sky
from compile_disaster import load_scene_config
from disaster import kinds

import scene_cache
import targets

# ----- CONFIGURATION -----
SCENE_CONFIG = os.environ.get("SCENE_CONFIG") or "urban_quake_showcase"
SCENE_USD = os.environ.get("SCENE_USD", "")
TARGET_SEED = os.environ.get("TARGET_SEED", "")
APPLY_COLLIDERS = os.environ.get("APPLY_COLLIDERS", "true").lower() == "true"
# -------------------------


def _resolve_usd(config: dict) -> str:
    """The baked scene for *config*: an explicit path wins, else the cache."""
    if SCENE_USD:
        return SCENE_USD
    usd = scene_cache.SceneCache().get(config)
    if not usd:
        raise SystemExit(
            f"no baked scene for {SCENE_CONFIG!r} in the scene cache. Bake it "
            f"first:\n    python3 scene_gen/bake_scene.py --config "
            f"{SCENE_CONFIG}\nor point SCENE_USD at a scene USD.")
    return usd


class BakedSceneApp:

    def __init__(self):
        config = load_scene_config(SCENE_CONFIG)
        if TARGET_SEED:
            # Same city, different people. The whole reason Stage C places
            # targets at load time instead of baking them in.
            config.setdefault("targets", {})["seed"] = int(TARGET_SEED)

        usd = _resolve_usd(config)
        print(f"[baked_scene] opening {usd}")
        usd_ctx = omni.usd.get_context()
        usd_ctx.open_stage(usd)
        stage = usd_ctx.get_stage()
        if stage is None:
            raise RuntimeError(f"Failed to open {usd}")
        for _ in range(10):
            omni.kit.app.get_app().update()

        if stage.GetPrimAtPath(Sdf.Path("/World")).IsValid() is False:
            UsdGeom.Xform.Define(stage, Sdf.Path("/World"))

        # Colliders BEFORE Stage C, so the city is solid to the drone and the
        # people are not: a victim is something to see, not something to fly
        # into, and a collider on a skinned mesh is an expensive way to be
        # wrong about that.
        if APPLY_COLLIDERS:
            root = stage.GetPrimAtPath(Sdf.Path("/World"))
            if root.IsValid():
                add_colliders(root)

        # ----- Stage C -----
        # No placement list here — the survey is read back off the stage, from
        # the `assetCategory` customData every generated prim carries.
        _, ssf = get_stage_meters_per_unit(stage)
        disaster = kinds.get(config)
        disaster.place_targets(stage, config, scene_scale_factor=ssf)
        n = disaster.attach_runtime(stage)
        if n:
            print(f"[baked_scene] attach_runtime applied {n} setting(s)")

        try:
            add_sky(stage, resolve_sky(config))
        except Exception as exc:                                  # noqa: BLE001
            carb.log_warn(f"sky not applied: {exc}")

        for _ in range(10):
            omni.kit.app.get_app().update()
        print(f"[baked_scene] ready — targets under "
              f"{targets.settings(config).get('parent_path')}")

    def run(self):
        app = omni.kit.app.get_app()
        while simulation_app.is_running():
            app.update()
        simulation_app.close()


def main():
    BakedSceneApp().run()


if __name__ == "__main__":
    main()
