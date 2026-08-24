#!/usr/bin/env python
"""Stage A via `airstack up isaac-sim`. A shim over the offline CLI.

Stage A is a batch job — "once; exhaustive; layout-independent" — so its real
entry point is `scene_gen/archetypes/bake_cli.py`, which runs headless, reports
what it baked and EXITS with a status code. Prefer it:

    docker exec isaac-sim bash -c \
      "cd /isaac-sim && ./python.sh AirStack/scene_gen/archetypes/bake_cli.py \
       --config fire --disaster fire"

This file exists only for the `ISAAC_SIM_SCRIPT_NAME` path, which some
workflows are wired to. It forwards the environment to the same CLI:

    SCENE_CONFIG=fire ARCH_DISASTER=fire ARCH_ONLY=ranch,Black_Oak \
    ISAAC_SIM_SCRIPT_NAME=bake_archetypes_launch_script.py airstack up isaac-sim

    SCENE_CONFIG    scene config naming the asset pack (default: fire)
    ARCH_DISASTER   disaster to bake, or 'all' (default: the config's type)
    ARCH_DIR        output root (default: scene_gen/assets/archetypes)
    ARCH_SEED       RNG seed (default: 7)
    ARCH_ONLY       comma-separated type slugs, instead of the whole pack
    ARCH_USED_ONLY  set to 1 for only the types the scene places
    ARCH_GUI        set to 1 to watch it; default is headless
"""

import os
import sys

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
if _SCENE_GEN_DIR not in sys.path:
    sys.path.insert(0, _SCENE_GEN_DIR)

from archetypes.bake_cli import main                             # noqa: E402


def _argv():
    argv = ["--config", os.environ.get("SCENE_CONFIG", "fire"),
            "--seed", os.environ.get("ARCH_SEED", "7")]
    for env, flag in (("ARCH_DISASTER", "--disaster"),
                      ("ARCH_DIR", "--out"),
                      ("ARCH_ONLY", "--only")):
        val = os.environ.get(env, "").strip()
        if val:
            argv += [flag, val]
    if os.environ.get("ARCH_USED_ONLY", "").lower() in ("1", "true", "yes"):
        argv.append("--used-only")
    if os.environ.get("ARCH_GUI", "").lower() in ("1", "true", "yes"):
        argv.append("--gui")
    return argv


if __name__ == "__main__":
    sys.exit(main(_argv()))
