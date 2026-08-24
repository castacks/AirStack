#!/usr/bin/env python3
"""view_usd.py — open written USDs in an Isaac Sim window.

    cd AirStack
    UV_ENV_FILE=$PWD/.env.host uv run python scene_gen/tools/view_usd.py /tmp/ruins
    UV_ENV_FILE=$PWD/.env.host uv run python scene_gen/tools/view_usd.py a.usd b.usd

Takes files or directories (a directory contributes its ``*.usd``/``*.usda``),
references each onto one stage laid out on a grid, and leaves the window up.

There is no ``usdview`` in this environment — the usd-core wheel does not ship
it and Isaac Sim only exposes ``.kit`` apps — so this is the short path from a
written file to looking at it.

The ruins that `quake_preview.py` writes are already frozen (kinematic rigid
bodies) so nothing moves when the sim ticks; this only references and renders
them. Pass ``--drop`` to release them instead and watch them fall, which is a
quick way to see whether a ruin was actually settled or merely looks settled.
"""

from __future__ import annotations

import argparse
import glob
import math
import os
import sys

_SCENE_GEN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("paths", nargs="+", help="USD files, or directories of them")
    p.add_argument("--spacing", type=float, default=0.0,
                   help="metres between items (0 = 1.5x the largest)")
    p.add_argument("--drop", action="store_true",
                   help="release the bodies so they fall, instead of holding "
                        "the settled pose")
    p.add_argument("--shot", default="", help="write a PNG and exit")
    return p.parse_args(argv)


def _frame(stage, set_camera_view, authoring):
    """Point the camera at whatever is actually on the stage."""
    import numpy as np

    box = authoring.content_bounds(stage, "/World")
    if box is None:
        return
    lo, hi = box
    centre = (lo + hi) * 0.5
    reach = float(np.linalg.norm(hi - lo)) * 0.75 + 5.0
    set_camera_view([centre[0] + reach, centre[1] - reach, centre[2] + reach * 0.6],
                    [float(centre[0]), float(centre[1]), float(lo[2])])


def main(argv=None):
    args = parse_args(argv)
    files = []
    for path in args.paths:
        if os.path.isdir(path):
            for ext in ("*.usd", "*.usda", "*.usdc"):
                files += sorted(glob.glob(os.path.join(path, ext)))
        else:
            files.append(path)
    files = [f for f in files if os.path.exists(f)]
    if not files:
        print("[view] nothing to show", flush=True)
        return 1

    from isaacsim import SimulationApp
    simulation_app = SimulationApp(launch_config={"headless": bool(args.shot)})

    import numpy as np
    import omni.kit.app
    from isaacsim.core.api import World
    from isaacsim.core.utils.viewports import set_camera_view
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    from disaster import authoring

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    authoring.add_lighting(world.stage)
    stage = world.stage

    # Measure first: the grid pitch should follow the biggest thing on it.
    sizes = []
    for f in files:
        try:
            sub = Usd.Stage.Open(f)
            cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                      [UsdGeom.Tokens.default_])
            rng = cache.ComputeWorldBound(
                sub.GetDefaultPrim() or sub.GetPseudoRoot()).ComputeAlignedRange()
            sizes.append(float(max(rng.GetSize())))
        except Exception:
            sizes.append(10.0)
    # Median, not max: one outlier (a 30 m sign among 8 m houses) would
    # otherwise set the pitch for everything.
    pitch = args.spacing or (float(np.median(sizes)) * 1.6 if sizes else 15.0)
    cols = max(1, int(math.ceil(math.sqrt(len(files)))))

    for i, f in enumerate(files):
        root = f"/World/view_{i:03d}"
        prim = UsdGeom.Xform.Define(stage, root)
        prim.GetPrim().GetReferences().AddReference(os.path.abspath(f))
        prim.AddTranslateOp().Set(
            Gf.Vec3d((i % cols) * pitch, (i // cols) * pitch, 0.0))
        print(f"[view] {os.path.basename(f)}", flush=True)

    world.reset()
    if not args.drop:
        # Hold whatever pose the file was written with.
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Set(True)

    _frame(stage, set_camera_view, authoring)

    if args.shot:
        import omni.kit.viewport.utility as vu
        for _ in range(90):
            omni.kit.app.get_app().update()
        vu.capture_viewport_to_file(vu.get_active_viewport(),
                                    file_path=args.shot)
        for _ in range(40):
            omni.kit.app.get_app().update()
        print(f"[view] wrote {args.shot}", flush=True)
        simulation_app.close()
        return 0

    print(f"[view] {len(files)} item(s) — close the window to exit", flush=True)
    while simulation_app.is_running():
        world.step(render=True)
    simulation_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
