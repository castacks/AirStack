"""
Reload the procedural city on the live Isaac Sim stage without restarting.

Run from the Kit Script Editor (Window → Script Editor) with a single line:
    exec(open("/isaac-sim/AirStack/scene_gen/reload_scene.py").read())

Edit this file, the scene config, or the generator/compiler freely between
reloads — importlib.reload picks up both YAML and Python changes every time.
Point SCENE_CONFIG below at a high-level disaster spec or a low-level scene
config; high-level specs are recompiled in memory on every reload.
"""

import os, sys

# The Script Editor copies exec'd code to a temp file in /tmp before running,
# so __file__ may be set but points to e.g. /tmp/carb.xxx/script_nnn.py rather
# than the actual repo. Validate the __file__-derived path and fall back to a
# search if it doesn't contain reload_scene.py.
def _find_scene_gen_dir():
    try:
        candidate = os.path.dirname(os.path.abspath(__file__))
        if os.path.isfile(os.path.join(candidate, "reload_scene.py")):
            return candidate
    except NameError:
        pass
    for candidate in [
        os.environ.get("AIRSTACK_SCENE_GEN_DIR", ""),
        "/isaac-sim/AirStack/scene_gen",
        os.path.expanduser("~/AirStack/scene_gen"),
        "/root/AirStack/scene_gen",
    ]:
        if candidate and os.path.isfile(os.path.join(candidate, "reload_scene.py")):
            return candidate
    raise RuntimeError(
        "Cannot locate AirStack/scene_gen. "
        "Set AIRSTACK_SCENE_GEN_DIR to its full path inside the container.")

_SCENE_GEN_DIR = _find_scene_gen_dir()

# scene_prep lives with the sim, not with the generator: it is Isaac Sim stage
# tooling (colliders, sky, settling) that the plain Pegasus launch scripts use
# too, so it stays under simulation/isaac-sim/utils and is imported from there.
_UTILS_DIR = os.path.join(os.path.dirname(_SCENE_GEN_DIR),
                          "simulation", "isaac-sim", "utils")
for _d in (_SCENE_GEN_DIR, _UTILS_DIR):
    if _d not in sys.path:
        sys.path.insert(0, _d)

import importlib
import compile_disaster
import scene_generator
import scene_prep

importlib.reload(scene_generator)   # picks up edits to scene_generator.py
importlib.reload(scene_prep)
importlib.reload(compile_disaster)  # and to the disaster compilers

import omni.kit.app
import omni.usd
import omni.timeline

# Either level works: a high-level disaster spec (config/presets/) is compiled
# in memory on every reload — so editing severity or the compiler and re-running
# this script rebuilds with the new settings — while a low-level config is
# loaded as is. A bare name ("tornado") is resolved against presets/ then
# low_level/compiled/.
SCENE_CONFIG = os.path.join(_SCENE_GEN_DIR, "config", "presets", "earthquake.yaml")

stage = omni.usd.get_context().get_stage()
omni.timeline.get_timeline_interface().stop()

_, ssf = scene_prep.get_stage_meters_per_unit(stage)
placements = scene_generator.reload_scene_on_stage(
    stage,
    compile_disaster.load_scene_config(SCENE_CONFIG),
    scene_scale_factor=ssf,
    add_colliders_fn=scene_prep.add_colliders,
)

# Physics-settle toppled/strewn props (flipped cars, downed poles) so they
# rest naturally instead of floating at their approximated orientations.
scene_prep.settle_rigid_props(
    stage,
    [p["prim_path"] for p in placements
     if p.get("settle") and p.get("prim_path")],
    ground_path="/World/stage/generated/ground",
)

# Pump the app loop so the renderer processes change notifications for the
# newly created prims. Without this, prims created during a reload are in the
# USD layer but the RTX renderer hasn't picked them up yet — they appear in
# the hierarchy as visible but have no geometry on screen.
app = omni.kit.app.get_app()
for _ in range(10):
    app.update()

omni.timeline.get_timeline_interface().play()
print(f"[reload_scene] Done (loaded from {_SCENE_GEN_DIR})")
