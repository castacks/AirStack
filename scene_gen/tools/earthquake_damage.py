#!/usr/bin/env python3
"""earthquake_damage.py — shake one asset down, and watch it.

    cd AirStack
    UV_ENV_FILE=$PWD/.env.host uv run python \
        scene_gen/tools/earthquake_damage.py \
        --asset objaverse://6644de89c2f0449db3de934744162b63 \
        --material brick --severity 0.55 --rubble

A demo harness, not the model. Everything that decides what happens lives in
``scene_gen/disaster/``:

    source      one asset -> triangles, UVs, materials
    solids      open sheets -> closed volumes
    fracture    a solid -> chunks, along seeds the caller places
    earthquake  where a building fails, and what lets go
    authoring   chunks, rubble and materials onto a stage

This file only boots Isaac Sim, calls ``earthquake.shake``, authors the result,
runs PhysX until the rubble stops moving, and freezes it — so what comes out is
a static scene ready to be flown through.
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

_SCENE_GEN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

from disaster import authoring, earthquake                      # noqa: E402


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--asset", required=True,
                   help="objaverse://<uid>, omniverse://…, airstack://…, path")
    p.add_argument("--material", nargs="+", default=["concrete"],
                   choices=sorted(earthquake.MATERIALS),
                   help="structural material(s); several are assigned in "
                        "height bands, lowest first")
    p.add_argument("--severity", type=float, default=0.5,
                   help="0 = untouched, 1 = complete collapse")
    p.add_argument("--soft-story", action="store_true",
                   help="open ground floor: the whole base storey fails")
    p.add_argument("--rubble", action="store_true",
                   help="scatter debris blocks around the base")
    p.add_argument("--chunks", type=int, default=90,
                   help="baseline fragment count before material and severity")
    p.add_argument("--dirt", type=float, default=0.07,
                   help="per-face grime variation on cut faces (mostly shade)")
    p.add_argument("--settle", type=float, default=4.0,
                   help="seconds of PhysX before the ruin is frozen")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--target-size", type=float, default=8.0)
    p.add_argument("--up-axis", choices=("z", "y"), default="z")
    p.add_argument("--thickness", type=float, default=0.05,
                   help="metres of shell extrusion for open (sheet) geometry")
    p.add_argument("--headless", action="store_true")
    p.add_argument("--shot", default="",
                   help="with --headless, write <shot>_before/_after.png")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    rng = np.random.default_rng(args.seed)

    from isaacsim import SimulationApp
    simulation_app = SimulationApp(launch_config={"headless": args.headless})

    import omni.kit.app
    from isaacsim.core.api import World
    from isaacsim.core.prims import RigidPrim
    from isaacsim.core.utils.viewports import set_camera_view
    from pxr import UsdPhysics

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    authoring.add_lighting(world.stage)

    ruin = earthquake.shake(
        world.stage, args.asset, severity=args.severity,
        materials=args.material, soft_story=args.soft_story, seed=args.seed,
        target_size=args.target_size, up_axis=args.up_axis,
        thickness=args.thickness, chunks=args.chunks)
    if not ruin.chunks:
        print("[quake] nothing to damage", flush=True)
        simulation_app.close()
        return 1

    authoring.author_ruin(world.stage, "/World/ruin", ruin.chunks,
                          ruin.source.mat_paths, ruin.damage, ruin.materials,
                          args.dirt, rng)
    n_rubble = 0
    if args.rubble:
        n_rubble = authoring.author_rubble(
            world.stage, "/World/rubble", ruin.field,
            [earthquake.MATERIALS[m] for m in args.material], args.severity,
            float(ruin.falls.mean()), ruin.lo, ruin.hi, rng)
        print(f"[quake] {n_rubble} rubble blocks", flush=True)

    world.scene.add(RigidPrim("/World/ruin/chunk_.*", name="ruin"))
    world.reset()
    span = ruin.span
    set_camera_view([span * 1.8, -span * 1.8, span * 0.9],
                    [0.0, 0.0, span * 0.3])

    def shot(tag):
        if not (args.headless and args.shot):
            return
        import omni.kit.viewport.utility as vu
        for _ in range(60):
            omni.kit.app.get_app().update()
        vu.capture_viewport_to_file(vu.get_active_viewport(),
                                    file_path=f"{args.shot}_{tag}.png")
        for _ in range(40):
            omni.kit.app.get_app().update()
        print(f"[quake] wrote {args.shot}_{tag}.png", flush=True)

    def kinematic(root, count, on):
        for i in range(count):
            prim = world.stage.GetPrimAtPath(f"{root}_{i:04d}")
            if prim:
                UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Set(on)

    shot("before")

    # Everything is released, including the parts that did not fail. Gravity
    # is what decides whether a body stays up, and holding survivors kinematic
    # left them floating unsupported once the storey below them collapsed.
    # `earthquake.shake` has already fused touching survivors into single
    # bodies, so an intact wall falls as a wall rather than as loose blocks.
    kinematic("/World/ruin/chunk", len(ruin.chunks), False)
    for _ in range(int(args.settle * 60)):
        world.step(render=not args.headless)

    kinematic("/World/ruin/chunk", len(ruin.chunks), True)
    kinematic("/World/rubble/rubble", n_rubble, True)
    world.step(render=False)
    print("[quake] settled and frozen", flush=True)
    shot("after")

    if args.headless:
        simulation_app.close()
        return 0
    while simulation_app.is_running():
        world.step(render=True)
    simulation_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
