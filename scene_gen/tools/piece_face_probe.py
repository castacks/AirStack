#!/usr/bin/env python
"""piece_face_probe — the FACE-LEVEL geometry `_bind_soot`'s outward test
judges, per GeomSubset, on a baked sliced piece.

    usd_python.sh piece_face_probe.py <bake.usd> <sidecar.json> <regex> [...]

For every Mesh under a matched piece, and for every GeomSubset on it, prints
the face count, how many faces pass `_bind_soot`'s test
(`n . out > 0`, `out` = `quake_flow._outward(mass, side)` for the fire's
side), how many are HORIZONTAL (a fake-interior ceiling or floor plane: the
test can never pass one, whichever way it faces), and how deep each face sits
behind the piece's own outward extreme along `out` — the number that
separates a fake interior a camera sees through the opening from genuinely
interior geometry behind an intact shell.

Read-only: opens the bake USD, writes nothing. (fire_dtc3 review, 2026-08-30.)
"""
import json
import re
import sys

import numpy as np

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Usd, UsdGeom                                      # noqa: E402
from disaster import quake_flow as qf                             # noqa: E402

st = Usd.Stage.Open(sys.argv[1])
side_car = json.load(open(sys.argv[2]))
rx = [re.compile(p) for p in sys.argv[3:]]
masses = side_car["masses"]
fire = side_car["fire"]
xfc = UsdGeom.XformCache()


def face_data(prim):
    """(normals, centroids) in WORLD space, one row per face."""
    mesh = UsdGeom.Mesh(prim)
    pts = np.array(mesh.GetPointsAttr().Get(), dtype=np.float64)
    cnt = np.array(mesh.GetFaceVertexCountsAttr().Get(), dtype=np.int64)
    idx = np.array(mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int64)
    if not len(pts) or not len(cnt):
        return None, None
    Mg = xfc.GetLocalToWorldTransform(prim)
    M = np.array([[float(Mg[r][c]) for c in range(4)] for r in range(4)])
    pw = pts @ M[:3, :3] + M[3, :3]
    starts = np.concatenate([[0], np.cumsum(cnt)[:-1]])
    i0 = idx[starts]
    i1 = idx[np.minimum(starts + 1, len(idx) - 1)]
    i2 = idx[np.minimum(starts + 2, len(idx) - 1)]
    nrm = np.cross(pw[i1] - pw[i0], pw[i2] - pw[i0])
    cen = (pw[i0] + pw[i1] + pw[i2]) / 3.0
    return nrm, cen


for pr in st.Traverse():
    p = pr.GetPath().pathString
    if not any(r.search(p) for r in rx) or not pr.IsA(UsdGeom.Mesh):
        continue
    name = p.rsplit("/", 1)[-1]
    m = masses.get("main")
    # the piece's ring side is in its own name (`pier_N_...`, `corner_NE_...`)
    bits = name.split("_")
    # THE SIDE `_bind_soot` JUDGED THIS PIECE ON is the ring side
    # `_r_soot_overlay` was iterating, not the piece's own name: a corner_NE
    # on a fire venting E is visited as "E". Overridable with SIDE=<x>.
    import os as _os
    side = _os.environ.get("SIDE") or next(
        (b for b in bits if b in ("N", "S", "E", "W")), fire["sides"][0])
    ox, oy = qf._outward(m, side)
    nrm, cen = face_data(pr)
    if nrm is None:
        continue
    d = cen[:, 0] * ox + cen[:, 1] * oy          # signed depth along outward
    dmax = float(d.max())
    subs = list(UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(pr)))
    print("%s  side=%s out=(%.2f,%.2f)  %d face(s), outward extent %.2f m"
          % (p, side, ox, oy, len(nrm), dmax - float(d.min())))
    for sub in subs:
        f = np.array(sub.GetIndicesAttr().Get() or [], dtype=np.int64)
        if not len(f):
            continue
        n_, d_ = nrm[f], d[f]
        n_all = np.linalg.norm(n_, axis=1) + 1e-12
        dot = (n_[:, 0] * ox + n_[:, 1] * oy) / n_all
        horiz = np.abs(n_[:, 2]) / n_all > 0.7
        depth = dmax - d_                        # 0 = on the outer face
        print("   %-16s %3d face(s): pass n.out>0 %3d | horizontal %3d | "
              "n.out<0 %3d || depth behind outer face: min %.2f med %.2f "
              "max %.2f m"
              % (sub.GetPrim().GetName(), len(f), int((dot > 1e-9).sum()),
                 int(horiz.sum()), int((dot < -1e-9).sum()),
                 float(depth.min()), float(np.median(depth)),
                 float(depth.max())))
