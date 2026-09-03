#!/usr/bin/env python3
"""Fast-path freeze entry point with a mandatory content-addressed layout gate."""
import os
import runpy
import subprocess
import sys


repo = os.environ.get("REPO", "/isaac-sim/AirStack")
required = {k: os.environ.get(k) for k in
            ("FC_DUMP", "FC_MANIFEST", "FC_LAYOUT_STAMP")}
missing = [k for k, v in required.items() if not v]
if missing:
    raise SystemExit("fast freeze refused: missing " + ", ".join(missing))
cmd = [sys.executable, os.path.join(repo, "scene_gen/tools/urban_fire_fast_preflight.py"),
       "--repo", repo, "--dump", required["FC_DUMP"],
       "--manifest", required["FC_MANIFEST"], "--stamp",
       required["FC_LAYOUT_STAMP"], "--check-stamp"]
subprocess.run(cmd, check=True)
target = os.path.join(repo, "simulation/isaac-sim/launch_scripts",
                      "freeze_urban_fire_city_launch_script.py")
runpy.run_path(target, run_name="__main__")
