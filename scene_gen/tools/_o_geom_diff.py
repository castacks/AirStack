#!/usr/bin/env python
"""_o_geom_diff.py — is B the same geometry as A, only packed differently?

    scene_gen/tools/_t_pxr.sh scene_gen/tools/_o_geom_diff.py A=<usd> B=<usd>

Compares two baked archetypes on the things a renderer actually draws, in
WORLD space and per BOUND MATERIAL: triangle count, total surface area, the
centroid and bounding box of the drawn surface. Bare pxr, no Kit.

A merge is a re-packing, so every one of those must match to rounding. Any
row that does not is a lost or moved piece.
"""
import os
import sys
from collections import defaultdict

from pxr import Gf, Usd, UsdGeom, UsdShade


def harvest(path):
    st = Usd.Stage.Open(path)
    xf = UsdGeom.XformCache(Usd.TimeCode.Default())
    per = defaultdict(lambda: [0, 0.0, [0.0, 0.0, 0.0], 0.0,
                               [1e30] * 3, [-1e30] * 3])
    for prim in st.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(prim)
        pts = m.GetPointsAttr().Get() or []
        C = list(m.GetFaceVertexCountsAttr().Get() or [])
        I = list(m.GetFaceVertexIndicesAttr().Get() or [])
        if not pts or not C:
            continue
        M = xf.GetLocalToWorldTransform(prim)
        P = [M.Transform(Gf.Vec3d(p)) for p in pts]
        base = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        base = base.GetPrim().GetName() if base and base.GetPrim().IsValid() \
            else "<none>"
        fmat = [base] * len(C)
        for s in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)):
            b = UsdShade.MaterialBindingAPI(s.GetPrim()).ComputeBoundMaterial()[0]
            nm = b.GetPrim().GetName() if b and b.GetPrim().IsValid() else base
            for f in (s.GetIndicesAttr().Get() or []):
                if 0 <= int(f) < len(fmat):
                    fmat[int(f)] = nm
        o = 0
        for f, c in enumerate(C):
            face = [P[I[o + k]] for k in range(c)]
            o += c
            slot = per[fmat[f]]
            for k in range(1, c - 1):
                p, q, r = face[0], face[k], face[k + 1]
                cr = Gf.Cross(q - p, r - p)
                a = 0.5 * Gf.Vec3d(cr).GetLength()
                slot[0] += 1
                slot[1] += a
                cen = (p + q + r) / 3.0
                for j in range(3):
                    slot[2][j] += a * cen[j]
                slot[3] += a
            for v in face:
                for j in range(3):
                    slot[4][j] = min(slot[4][j], v[j])
                    slot[5][j] = max(slot[5][j], v[j])
    out = {}
    for k, s in per.items():
        c = [(s[2][j] / s[3]) if s[3] else 0.0 for j in range(3)]
        out[k] = {"tris": s[0], "area": s[1], "centroid": c,
                  "lo": s[4], "hi": s[5]}
    return out


def main():
    A, B = os.environ["A"], os.environ["B"]
    a, b = harvest(A), harvest(B)
    keys = sorted(set(a) | set(b))
    bad = 0
    print("{0:<26} {1:>10} {2:>10} {3:>12} {4:>12} {5:>9}".format(
        "material", "tris A", "tris B", "area A", "area B", "cen |d|"))
    for k in keys:
        x, y = a.get(k), b.get(k)
        if x is None or y is None:
            print("{0:<26} {1}".format(k, "ONLY IN " + ("B" if x is None else "A")))
            bad += 1
            continue
        d = max(abs(x["centroid"][j] - y["centroid"][j]) for j in range(3))
        ok = (x["tris"] == y["tris"]
              and abs(x["area"] - y["area"]) <= 1e-3 * max(1.0, x["area"])
              and d < 0.01)
        print("{0:<26} {1:>10d} {2:>10d} {3:>12.2f} {4:>12.2f} {5:>9.4f} {6}"
              .format(k, x["tris"], y["tris"], x["area"], y["area"], d,
                      "" if ok else "  <-- MISMATCH"))
        if not ok:
            bad += 1
    ta, tb = sum(v["tris"] for v in a.values()), sum(v["tris"] for v in b.values())
    aa, ab = sum(v["area"] for v in a.values()), sum(v["area"] for v in b.values())
    print("TOTAL tris {0} vs {1}   area {2:.2f} vs {3:.2f}".format(ta, tb, aa, ab))
    print("{0} material(s) differ".format(bad))
    sys.exit(1 if bad else 0)


main()
