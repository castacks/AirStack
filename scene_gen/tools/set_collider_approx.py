#!/usr/bin/env python3
"""set_collider_approx — change how PhysX approximates archetype colliders.

    AirStack/.venv/bin/python scene_gen/tools/set_collider_approx.py <lib dir> \
        --approx meshSimplification --metric 0.02 --weld 0.25 [--apply]

`disaster/bake.export_object` authors every merged mesh with
`physics:approximation = "none"` -- an EXACT triangle mesh. Measured on
`SM_Building_21_soft_storey`: 15 colliders over 4.15M points / 1.72M faces, and
PhysX builds an acceleration structure over all of it PER PLACEMENT. The run
that recorded 106,773 colliders from 166 references is where the memory went.

This rewrites the approximation on colliders that already exist, so it needs no
re-bake -- the same kind of attribute-only repair as the material rebinds.

WHY NOT `convexHull`: fragments are MERGED BY MATERIAL, so one prim holds
hundreds of scattered shards. A hull around that wraps the whole scatter in one
solid blob and the drone collides with empty air. Convex approximations only
make sense once each fragment is its own collider.
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys

_SCENE_GEN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

APPROX = "physics:approximation"
METRIC = "physxTriangleMeshSimplificationCollision:metric"
WELD = "physxTriangleMeshSimplificationCollision:weldTolerance"


def retarget(path, approx, metric, weld):
    """Set the approximation on every collider in *path*. Returns count."""
    from pxr import Sdf
    lay = Sdf.Layer.FindOrOpen(path)
    if lay is None:
        return 0
    hit = []

    def walk(spec):
        for ch in spec.nameChildren:
            if ch.attributes.get(APPROX) is not None:
                hit.append(ch)
            walk(ch)

    walk(lay.pseudoRoot)
    for spec in hit:
        spec.attributes[APPROX].default = approx
        if approx == "meshSimplification":
            for name, val in ((METRIC, metric), (WELD, weld)):
                a = spec.attributes.get(name)
                if a is None:
                    a = Sdf.AttributeSpec(spec, name,
                                          Sdf.ValueTypeNames.Float)
                a.default = float(val)
    if hit:
        # The baker runs as root in the container, so the file may not be ours
        # to save in place; the directory is, so write beside and move over.
        tmp = path + ".approx.tmp.usd"
        if not lay.Export(tmp):
            return 0
        os.unlink(path)
        shutil.move(tmp, path)
    return len(hit)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("library")
    ap.add_argument("--approx", default="meshSimplification")
    ap.add_argument("--metric", type=float, default=0.02)
    ap.add_argument("--weld", type=float, default=0.25)
    ap.add_argument("--only", default="", help="comma-separated file stems")
    ap.add_argument("--apply", action="store_true")
    a = ap.parse_args(argv)

    only = {x.strip() for x in a.only.split(",") if x.strip()}
    files = sorted(f for f in os.listdir(a.library)
                   if f.endswith(".usd") and ".orig." not in f)
    if only:
        files = [f for f in files if os.path.splitext(f)[0] in only]
    total = touched = 0
    for fn in files:
        p = os.path.join(a.library, fn)
        if not a.apply:
            from pxr import Sdf
            lay = Sdf.Layer.FindOrOpen(p)
            n = 0
            def count(spec):
                nonlocal n
                for ch in spec.nameChildren:
                    if ch.attributes.get(APPROX) is not None:
                        n += 1
                    count(ch)
            count(lay.pseudoRoot)
        else:
            n = retarget(p, a.approx, a.metric, a.weld)
        if n:
            touched += 1
            total += n
    print(f"{'would set' if not a.apply else 'set'} {a.approx} on {total} "
          f"collider(s) across {touched} archetype(s)"
          + (f"  metric={a.metric} weld={a.weld}"
             if a.approx == "meshSimplification" else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
