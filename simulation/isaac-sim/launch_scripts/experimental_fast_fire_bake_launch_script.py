#!/usr/bin/env python
"""Isolated fast-slicer entry point for one fire bake experiment.

It patches the imported slicer in memory, enables Fabric settling by default,
and then executes the trusted production fire-bake launcher.  No production
module or cache path is changed.
"""

import os
import runpy
import sys

REPO = os.environ.get("REPO", "/isaac-sim/AirStack")
SCENE_GEN = os.path.join(REPO, "scene_gen")
if SCENE_GEN not in sys.path:
    sys.path.insert(0, SCENE_GEN)

from detail import gac_storey_slice_fast  # noqa: E402
from disaster import soot_bake_fast  # noqa: E402

gac_storey_slice_fast.install()
soot_bake_fast.install()
os.environ.setdefault("SETTLE_FABRIC", "1")
os.environ.setdefault("FB_OUT", "/isaac-sim/.cache/experimental_fast_fire_bakes")
print("[fast_bake] experimental sweep slicer installed; production files untouched")
print("[fast_bake] SETTLE_FABRIC=%s FB_OUT=%s" %
      (os.environ["SETTLE_FABRIC"], os.environ["FB_OUT"]))

runpy.run_path(
    os.path.join(REPO, "simulation/isaac-sim/launch_scripts/fire_bake_launch_script.py"),
    run_name="__main__")
