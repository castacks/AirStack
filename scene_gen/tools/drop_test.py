#!/usr/bin/env python3
"""drop_test — would this wreck still be standing if it were not glued down?

    /isaac-sim/python.sh scene_gen/tools/drop_test.py <archetype.usd> [--snap DIR]

A baked archetype carries `UsdPhysics.CollisionAPI` and NO `RigidBodyAPI`: Stage
A exports pre-settled STATIC wrecks so a scene can reference one without paying
for physics at load. That is the whole point of the bake — and it means
`view_usd.py --drop`, which only un-kinematics prims that already have a rigid
body, is a no-op on an archetype. Both of its branches do nothing and the
building sits there, which reads exactly like "it was stable" and is not.

So this makes the body itself. It takes the RETAINED SHELL — the mesh the cut
never broke up, which is everything not named `rubble_*` / `debris_*` — hands
it to PhysX as one dynamic body over the settled rubble as static collision,
and measures where it ends up.

WHAT THE ANSWER MEANS
---------------------
`soft_storey` is supposed to remove a wedge of ground floor and let the mass
above come down on it — `quake.MECHANISMS` says so: "it loses its support
instead, and the graph in `mesh_damage.unsupported` hands it to the settle as
ONE rigid body that topples onto the wreckage." Measured on the v5 library,
that never happens: all twelve baked `soft_storey` archetypes lose 0% of their
height, because the shell still reaches the ground on the part of the footprint
the wedge did not claim, and one rigid mesh standing on part of its base is
infinitely strong.

    drops / tilts  -> the geometry IS unstable and the bug is only that
                      `unsupported` never releases it. Fix the release
                      condition and the existing cut is fine.
    stays put      -> the remaining base genuinely carries it, and no release
                      rule will help; the CUT has to take more of the storey.

Those are different fixes, which is why it is worth ten minutes to tell them
apart before changing anything.
"""

from __future__ import annotations

import argparse
import os
import sys

_SCENE_GEN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

#: Merged-fragment prefixes `disaster/bake.export_object` writes. Everything
#: else under the archetype is the part of the building the cut left whole.
_RUBBLE = ("rubble_", "debris_", "pile_")


def _parse(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("usd", help="a baked archetype")
    p.add_argument("--steps", type=int, default=900)
    p.add_argument("--dt", type=float, default=1.0 / 60.0)
    p.add_argument("--snap", default="",
                   help="write before/after PNGs here")
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = _parse(argv)

    from isaacsim import SimulationApp
    app = SimulationApp(launch_config={"headless": True})

    try:
        import omni.usd
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdLux
        from archetypes import bake as A
        from archetypes import preview as PV
        from disaster import settle as S

        ctx = omni.usd.get_context()
        ctx.new_stage()
        stage = ctx.get_stage()
        A.prepare_stage(stage)          # ground plane at z=0, dome + key light

        root = UsdGeom.Xform.Define(stage, Sdf.Path("/World/subject"))
        root.GetPrim().GetReferences().AddReference(os.path.abspath(args.usd))
        for _ in range(30):
            omni.kit.app.get_app().update()

        shell, rubble = [], []
        for prim in Usd.PrimRange(root.GetPrim()):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            path = str(prim.GetPath())
            (rubble if prim.GetName().startswith(_RUBBLE)
             else shell).append(path)
        if not shell:
            print("[drop] no retained shell in this archetype")
            app.close()
            return 1
        print(f"[drop] shell {len(shell)} mesh(es), rubble {len(rubble)}")

        def bounds(paths):
            b = PV.world_bounds(stage, paths)
            return None if b is None else b

        before = bounds(shell)
        if args.snap:
            PV.capture_cell(stage, shell + rubble,
                            os.path.join(args.snap, "00_before"))

        # The rubble is what it has to land ON; only the shell is released.
        report = S.run(stage, shell, rubble, steps=int(args.steps),
                       dt=float(args.dt), bake_result=True,
                       dynamic_approximation="convexDecomposition")
        after = bounds(shell)
        if args.snap:
            PV.capture_cell(stage, shell + rubble,
                            os.path.join(args.snap, "01_after"))

        print("\n" + "=" * 62)
        print(f"DROP TEST — {os.path.basename(args.usd)}")
        if before and after:
            (bx, by, bz), (bsx, bsy, bsz) = before
            (ax, ay, az), (asx, asy, asz) = after
            print(f"  shell centre z : {bz:8.2f} -> {az:8.2f}   "
                  f"({az - bz:+.2f} m)")
            print(f"  shell top z    : {bz + 0.5 * bsz:8.2f} -> "
                  f"{az + 0.5 * asz:8.2f}   "
                  f"({(az + 0.5 * asz) - (bz + 0.5 * bsz):+.2f} m)")
            # A tilt shows up as the footprint growing while the height falls.
            print(f"  footprint      : {bsx:.1f}x{bsy:.1f} -> "
                  f"{asx:.1f}x{asy:.1f} m")
            print(f"  height         : {bsz:8.2f} -> {asz:8.2f} m")
        print(f"  settle         : {report}")
        print("=" * 62 + "\n")
        sys.stdout.flush()
    except BaseException:
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        app.close()
        raise

    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    sys.exit(main())
