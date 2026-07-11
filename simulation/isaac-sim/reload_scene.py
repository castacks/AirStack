"""
Reload the procedural city on the live Isaac Sim stage without restarting.

Run from the Kit Script Editor (Window → Script Editor) with a single line:
    exec(open("/isaac-sim/AirStack/simulation/isaac-sim/reload_scene.py").read())

Edit this file or scene_generator_config.yaml freely between reloads —
importlib.reload picks up both YAML and Python changes every time.
"""

import os, sys

# The Script Editor copies exec'd code to a temp file in /tmp before running,
# so __file__ may be set but points to e.g. /tmp/carb.xxx/script_nnn.py rather
# than the actual repo. Validate the __file__-derived path and fall back to a
# search if it doesn't contain reload_scene.py.
def _find_isaac_sim_dir():
    try:
        candidate = os.path.dirname(os.path.abspath(__file__))
        if os.path.isfile(os.path.join(candidate, "reload_scene.py")):
            return candidate
    except NameError:
        pass
    for candidate in [
        os.environ.get("AIRSTACK_ISAAC_SIM_DIR", ""),
        "/isaac-sim/AirStack/simulation/isaac-sim",
        os.path.expanduser("~/AirStack/simulation/isaac-sim"),
        "/root/AirStack/simulation/isaac-sim",
    ]:
        if candidate and os.path.isfile(os.path.join(candidate, "reload_scene.py")):
            return candidate
    raise RuntimeError(
        "Cannot locate AirStack/simulation/isaac-sim. "
        "Set AIRSTACK_ISAAC_SIM_DIR to its full path inside the container.")

_ISAAC_SIM_DIR = _find_isaac_sim_dir()

_UTILS_DIR = os.path.join(_ISAAC_SIM_DIR, "utils")
if _UTILS_DIR not in sys.path:
    sys.path.insert(0, _UTILS_DIR)

import importlib
import scene_generator
import scene_prep

importlib.reload(scene_generator)   # picks up edits to scene_generator.py
importlib.reload(scene_prep)

import omni.kit.app
import omni.usd
import omni.timeline

stage = omni.usd.get_context().get_stage()
omni.timeline.get_timeline_interface().stop()

_, ssf = scene_prep.get_stage_meters_per_unit(stage)
scene_generator.reload_scene_on_stage(
    stage,
    os.path.join(_ISAAC_SIM_DIR, "config", "scene_generator_config.yaml"),
    scene_scale_factor=ssf,
    add_colliders_fn=scene_prep.add_colliders,
)

# Pump the app loop so the renderer processes change notifications for the
# newly created prims. Without this, prims created during a reload are in the
# USD layer but the RTX renderer hasn't picked them up yet — they appear in
# the hierarchy as visible but have no geometry on screen.
app = omni.kit.app.get_app()
for _ in range(10):
    app.update()

omni.timeline.get_timeline_interface().play()
print(f"[reload_scene] Done (loaded from {_ISAAC_SIM_DIR})")
