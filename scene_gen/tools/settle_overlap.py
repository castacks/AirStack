#!/usr/bin/env python3
"""settle_overlap.py — how much of a scene starts inside itself, before Isaac.

    python3 tools/settle_overlap.py --config earthquake

WHAT IT ANSWERS
---------------
`scene_prep.settle_rigid_props` hands every `settle`-marked prop to PhysX as a
dynamic rigid body. PhysX separates overlapping bodies with an impulse
proportional to how deeply they overlap, so a prop that spawns *inside* another
one does not settle — it is fired, and if it is still moving when the 3 s sim
ends it is frozen wherever it got to. That is the mechanism behind debris found
hundreds of metres outside the region.

The overlap is decidable without physics: it is a property of where the
generator put things. This composes the scene on the host and measures it, so
"will this scene explode?" is a question you can answer in seconds instead of
after a container launch.

    settle_overlap.py   -> will it explode?      (host, seconds)
    settle_rigid_props  -> did it explode?       (in sim, reports travel)

WHAT THE NUMBERS MEAN, AND WHERE THEY OVERSTATE
-----------------------------------------------
Depth is metres of AABB overlap. AABBs are looser than the convex hulls PhysX
uses, so small values are noise — 0.1 m between two 2 m planks need not be a
real penetration. What matters is the tail: a p90 above roughly half a prop's
own size means a large minority of the scene starts badly interpenetrating.

**Fracture fragments are the honest exception and are reported separately.**
Voronoi cells are cut from one mesh along shared planes, so neighbours are
face-adjacent by construction: their AABBs always overlap while their hulls
merely touch. A big number in the `debris_fragment` row is therefore expected
and not by itself alarming — what makes fragments fly is the `throw`
displacement pushing them into each other on top of that. The `debris` /
`debris_pile` rows have no such excuse and mean exactly what they say.

Either way this screens; `settle_rigid_props` reporting its travel is what
confirms.

Needs `usd-core` (system `python3`). Composes a stage but calls no renderer.
"""

from __future__ import annotations

import argparse
import collections
import contextlib
import io
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
sys.path.insert(0, _SCENE_GEN)

import numpy as np                                              # noqa: E402
from pxr import Usd, UsdGeom                                    # noqa: E402


def compose(config_name: str):
    """The scene on a stage, via the same pipeline the launch script runs."""
    from compile_disaster import load_scene_config, resolve_config_path
    import generate_scene

    cfg = load_scene_config(resolve_config_path(config_name))
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World")
    with contextlib.redirect_stdout(io.StringIO()):
        placements = generate_scene.generate_scene_on_stage(
            stage, cfg, "/World/gen")
    return stage, placements


def boxes_of(stage, placements):
    """World AABBs and whether each prop is settle-marked."""
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_])
    box, settle, path, cat = [], [], [], []
    for p in placements:
        pp = p.get("prim_path")
        if not pp:
            continue
        prim = stage.GetPrimAtPath(pp)
        if not prim or not prim.IsValid():
            continue
        r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if r.IsEmpty():
            continue
        box.append([*r.GetMin(), *r.GetMax()])
        settle.append(bool(p.get("settle")))
        path.append(pp)
        cat.append(str(p.get("category", "?")))
    return np.asarray(box), np.asarray(settle), path, cat


def overlaps(box, settle, cell: float = 8.0):
    """``{index: (deepest overlap, how many things it overlaps)}``.

    Bucketed into a uniform grid first: the pairwise form is O(n^2) and a
    damaged urban carries tens of thousands of props.
    """
    grid = collections.defaultdict(list)
    for i, b in enumerate(box):
        for gx in range(int(b[0] // cell), int(b[3] // cell) + 1):
            for gy in range(int(b[1] // cell), int(b[4] // cell) + 1):
                grid[(gx, gy)].append(i)

    def depth(a, b):
        d = (min(a[3], b[3]) - max(a[0], b[0]),
             min(a[4], b[4]) - max(a[1], b[1]),
             min(a[5], b[5]) - max(a[2], b[2]))
        return min(d) if all(v > 0 for v in d) else 0.0

    worst: dict = {}
    hits: collections.Counter = collections.Counter()
    seen = set()
    for ids in grid.values():
        for ii in range(len(ids)):
            for jj in range(ii + 1, len(ids)):
                i, j = ids[ii], ids[jj]
                if (i, j) in seen:
                    continue
                seen.add((i, j))
                # Only a settle-marked prop can be launched; two static props
                # overlapping is a cosmetic issue, not a physics one.
                if not (settle[i] or settle[j]):
                    continue
                d = depth(box[i], box[j])
                if d <= 0.0:
                    continue
                for k in (i, j):
                    if settle[k]:
                        worst[k] = max(worst.get(k, 0.0), d)
                        hits[k] += 1
    return worst, hits


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--config", default="earthquake")
    ap.add_argument("--top", type=int, default=5)
    a = ap.parse_args()

    stage, placements = compose(a.config)
    box, settle, path, cat = boxes_of(stage, placements)
    n = int(settle.sum())
    print(f"[settle] {a.config}: {len(box)} prims with bounds, "
          f"{n} settle-marked")
    if not n:
        return 0

    worst, hits = overlaps(box, settle)
    v = np.array(sorted(worst.values())) if worst else np.zeros(0)
    print(f"[settle] starting interpenetrating: {len(worst)} "
          f"({100.0 * len(worst) / n:.0f}% of settle-marked)")
    if len(v):
        print(f"[settle] overlap depth  median {np.median(v):.2f} m   "
              f"p90 {np.percentile(v, 90):.2f} m   max {v.max():.2f} m")
        by = collections.defaultdict(list)
        for i, d in worst.items():
            by[cat[i]].append(d)
        n_by = collections.Counter(c for c, s_ in zip(cat, settle) if s_)
        print("[settle] by category:")
        for c in sorted(by, key=lambda k: -len(by[k])):
            a_ = np.array(by[c])
            print(f"    {c:16s} {len(a_):5d}/{n_by[c]:<5d} overlapping   "
                  f"median {np.median(a_):5.2f} m   p90 "
                  f"{np.percentile(a_, 90):5.2f} m")
        rank = sorted(worst.items(), key=lambda kv: -kv[1])[:a.top]
        print(f"[settle] worst {len(rank)}:")
        for i, d in rank:
            print(f"    {d:6.2f} m deep, inside {hits[i]:3d} others  "
                  f"{path[i]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
