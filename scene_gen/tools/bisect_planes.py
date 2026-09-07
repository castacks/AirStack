#!/usr/bin/env python3
"""bisect_planes — cut every mesh crossed by a plane you placed, into halves.

    /isaac-sim/python.sh tools/bisect_planes.py <session.usda> [--dry-run]

WORKFLOW. In the edit session, drop a plane where you want the cut (Create >
Mesh > Plane, or any prim whose name starts with `Plane` or `cut`), position
and rotate it, save. This splits every mesh the plane actually crosses into two
prims, so you can delete the half you do not want and save again.

The halves are authored as `piece_*` CHILDREN of the mesh they came from, the
same convention `edit_usds.explode_stage` uses, which is what makes the rest
work for free: material and transform inherit from the parent, and the split on
window close merges whatever children survive back into the source. Delete one
half and the source keeps only the other.

THE PLANE is the cutter prim's local XY plane -- normal along its local +Z,
through its origin -- which is how Kit's own Plane primitive is oriented, so a
plane you can see edge-on is the cut you get.
"""

from __future__ import annotations

import argparse
import os
import sys

_SCENE_GEN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

CUTTER_PREFIXES = ("plane", "cut")


def _is_cutter(prim) -> bool:
    n = prim.GetName().lower()
    return any(n.startswith(p) for p in CUTTER_PREFIXES)


def _planes(stage):
    """[(origin, normal)] in world space, one per cutter prim."""
    from pxr import Gf, UsdGeom
    import numpy as np
    cache = UsdGeom.XformCache()
    out = []
    for prim in stage.Traverse():
        if not _is_cutter(prim):
            continue
        M = np.array(Gf.Matrix4d(
            cache.GetLocalToWorldTransform(prim))).reshape(4, 4)
        origin = M[3, :3]
        # The plane's normal is its local +Z carried into world space, and USD
        # is ROW-vector (`p_world = p_local . A`), so the normal transform is
        # `n_local . inv(A).T` -- which for n_local = +Z is COLUMN 2 of
        # `inv(A)`, not a row of it. Taking the row instead mirrors the plane
        # about its own axis: a 45-degree cutter came out as (0, +.707, +.707)
        # where the plane it draws is (0, -.707, +.707), so the cut angle was
        # wrong for every rotated plane while axis-aligned ones looked fine.
        # Inverting (rather than transposing A) is what keeps it perpendicular
        # under the non-uniform scale a dragged-out plane usually has.
        n = np.linalg.inv(M[:3, :3])[:, 2]
        ln = np.linalg.norm(n)
        if ln < 1e-12:
            continue
        out.append((str(prim.GetPath()), origin, n / ln))
    return out


def _tri(counts, idx):
    import numpy as np
    counts = np.asarray(counts, dtype=np.int64)
    idx = np.asarray(idx, dtype=np.int64)
    starts = np.concatenate([[0], np.cumsum(counts)[:-1]])
    out = []
    for c, s in zip(counts, starts):
        f = idx[s:s + c]
        for k in range(1, c - 1):
            out.append((f[0], f[k], f[k + 1]))
    return np.asarray(out, dtype=np.int64) if out else np.zeros((0, 3), np.int64)


def bisect_stage(stage, dry_run=False) -> int:
    from pxr import Gf, Sdf, UsdGeom, Vt
    import numpy as np
    from disaster.mesh_damage import _clip_by_plane
    try:
        from tools.edit_usds import PIECE_PREFIX, EXPLODED_ATTR
    except Exception:                                            # noqa: BLE001
        PIECE_PREFIX, EXPLODED_ATTR = "piece_", "airstack:explodedPieces"

    planes = _planes(stage)
    if not planes:
        print("[bisect] no cutter prims found (name one 'Plane...' or 'cut...')")
        return 0
    for path, o, n in planes:
        print(f"[bisect] plane {path}  origin {np.round(o, 2)}  "
              f"normal {np.round(n, 3)}")

    cache = UsdGeom.XformCache()
    targets = [p for p in stage.Traverse()
               if p.IsA(UsdGeom.Mesh) and not _is_cutter(p)]
    cut = 0
    for prim in targets:
        m = UsdGeom.Mesh(prim)
        pts = m.GetPointsAttr().Get()
        counts = m.GetFaceVertexCountsAttr().Get()
        idx = m.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts or not idx:
            continue
        M = np.array(Gf.Matrix4d(
            cache.GetLocalToWorldTransform(prim))).reshape(4, 4)
        P = np.asarray(pts, dtype=np.float64)
        Pw = (np.c_[P, np.ones(len(P))] @ M)[:, :3]
        F = _tri(counts, idx)
        if not len(F):
            continue

        crossing = [(o, n) for _p, o, n in planes
                    # Only planes that actually CROSS this mesh: everything
                    # else leaves it as one prim, so a plane cuts what it looks
                    # like it cuts.
                    if ((Pw - o) @ n).min() <= -1e-9
                    and ((Pw - o) @ n).max() >= 1e-9]
        if not crossing:
            continue
        if dry_run:
            cut += 1
            print(f"    would cut {prim.GetPath()}  ({len(P):,} verts, "
                  f"{len(crossing)} plane(s))")
            continue

        pv = UsdGeom.PrimvarsAPI(m).GetPrimvar("st")
        st = pv.Get() if pv else None
        UV = (np.asarray(st, dtype=np.float64)
              if st is not None and len(st) == len(P) else None)

        # EVERY crossing plane, not just the first. Stopping after one was an
        # assumption with nothing behind it: two planes through the same mesh
        # then produced a single cut and the second was silently ignored. Each
        # plane splits every surviving fragment, so n planes give up to 2^n.
        halves = [(Pw, F, UV)]
        for o, n in crossing:
            nxt = []
            for hv, hf, huv in halves:
                for sign in (1.0, -1.0):
                    v, f, uv, _ = _clip_by_plane(hv, hf, sign * n, o, uv=huv,
                                                 fmat=None, cap=False)
                    if len(f):
                        nxt.append((v, f, uv))
            halves = nxt or halves
        if len(halves) < 2:
            continue

        inv = np.linalg.inv(M)
        for k, (v, f, uv) in enumerate(halves):
            vl = (np.c_[v, np.ones(len(v))] @ inv)[:, :3]
            part = UsdGeom.Mesh.Define(
                stage, prim.GetPath().AppendChild(f"{PIECE_PREFIX}{k:04d}"))
            part.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(
                vl.astype(np.float32)))
            part.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(
                np.full(len(f), 3, dtype=np.int32)))
            part.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(
                f.reshape(-1).astype(np.int32)))
            part.CreateDoubleSidedAttr(True)
            part.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
            if uv is not None and len(uv) == len(vl):
                UsdGeom.PrimvarsAPI(part).CreatePrimvar(
                    "st", Sdf.ValueTypeNames.TexCoord2fArray,
                    UsdGeom.Tokens.vertex).Set(
                        Vt.Vec2fArray.FromNumpy(uv.astype(np.float32)))
        m.GetPointsAttr().Set(Vt.Vec3fArray())
        m.GetFaceVertexCountsAttr().Set(Vt.IntArray())
        m.GetFaceVertexIndicesAttr().Set(Vt.IntArray())
        prim.CreateAttribute(EXPLODED_ATTR, Sdf.ValueTypeNames.Int,
                             custom=True).Set(len(halves))
        cut += 1

    print(f"[bisect] {'would cut' if dry_run else 'cut'} {cut} mesh(es)")
    return cut


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("session")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--kit", action="store_true",
                    help="start a headless Kit first. Needed whenever the "
                         "session references assets by CONTAINER path: `pxr` "
                         "does not exist under `python.sh` until SimulationApp "
                         "has built the environment, and without Kit the "
                         "references do not resolve so every mesh reads empty")
    a = ap.parse_args(argv)

    app = None
    if a.kit:
        from isaacsim import SimulationApp
        app = SimulationApp(launch_config={"headless": True})

    from pxr import Usd
    stage = Usd.Stage.Open(a.session)
    n = bisect_stage(stage, a.dry_run)
    if n and not a.dry_run:
        stage.GetRootLayer().Save()
        print(f"[bisect] saved {a.session} — RELOAD the stage in Kit to see it")
    sys.stdout.flush()
    if app is not None:
        # `SimulationApp.close()` hard-exits; leave by the same door instead of
        # falling through to a return that would never run.
        os._exit(0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
