#!/usr/bin/env python
"""Do GAC's windows fall on a clean STOREY x BAY grid?

If they do, a merged GAC mesh can be sliced on that grid into roof + per-storey
corner/wall pieces — the same grammar `detail/urban_building.py` assembles the
ModernCityEnvironment01 kit with (bands x sides x corners, then a roof). The
sliced parts can then be emitted as kit-shaped PLACEMENT dicts, which is all
`quake_flow.classify`/`describe` consume, so the whole fire ladder applies
unchanged.

The grid must be MEASURED, not assumed. `MONO_STOREY_M = 3.4` is a guess that
was fine for spacing plume tongues and is not good enough to cut a building on:
one storey of drift over twelve floors puts the cut through the windows.

Reports, per asset and per elevation: the window-centre rows in z (storey
lines) and columns in the along-face axis (bay lines), with their spacing.
"""
import numpy as np
from pxr import Sdf, Usd, UsdGeom, UsdShade

SEI = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
       "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")
NAMES = ("SM_Building_01", "SM_Building_04", "SM_Building_24")
GLASS = ("glass", "window", "curtain", "glazing")


def tex_of(prim):
    mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    if not mat or not mat.GetPrim().IsValid():
        return ""
    for c in Usd.PrimRange(mat.GetPrim()):
        sh = UsdShade.Shader(c)
        if not sh or sh.GetIdAttr().Get() != "UsdPreviewSurface":
            continue
        d = sh.GetInput("diffuseColor")
        if d is not None and d.HasConnectedSource():
            ts = UsdShade.Shader(d.GetConnectedSource()[0].GetPrim())
            f = ts.GetInput("file")
            v = f.Get() if f else None
            if isinstance(v, Sdf.AssetPath) and v.path:
                return v.path.rsplit("/", 1)[-1]
        break
    return ""


def cluster(vals, tol):
    """1-D agglomerate: sorted values within `tol` collapse to one line."""
    if not len(vals):
        return []
    v = np.sort(np.asarray(vals))
    lines, cur = [], [v[0]]
    for q in v[1:]:
        if q - cur[-1] <= tol:
            cur.append(q)
        else:
            lines.append(float(np.mean(cur)))
            cur = [q]
    lines.append(float(np.mean(cur)))
    return lines


def run(name):
    st = Usd.Stage.Open(SEI + name + ".usd")
    if st is None:
        print("%s OPEN FAILED" % name); return
    mpu = UsdGeom.GetStageMetersPerUnit(st)
    for prim in Usd.PrimRange(st.GetPseudoRoot()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        me = UsdGeom.Mesh(prim)
        pts = me.GetPointsAttr().Get()
        if pts is None or not len(pts):
            continue
        P = np.asarray(pts, dtype=float) * mpu
        counts = np.asarray(me.GetFaceVertexCountsAttr().Get() or [], dtype=np.int64)
        fvi = np.asarray(me.GetFaceVertexIndicesAttr().Get() or [], dtype=np.int64)
        if not len(counts) or len(fvi) != int(counts.sum()):
            continue
        start = np.concatenate([[0], np.cumsum(counts)[:-1]])
        lo, hi = P.min(axis=0), P.max(axis=0)
        for sub in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)):
            if not any(g in tex_of(sub.GetPrim()).lower() for g in GLASS):
                continue
            fi = np.asarray(sub.GetIndicesAttr().Get() or [], dtype=np.int64)
            fi = fi[(fi >= 0) & (fi < len(counts))]
            if not len(fi):
                continue
            # per-face centroid and outward normal
            cen, nrm = [], []
            for f in fi:
                idx = fvi[start[f]:start[f] + counts[f]]
                V = P[idx]
                cen.append(V.mean(axis=0))
                n = np.cross(V[1] - V[0], V[2] - V[0])
                ln = np.linalg.norm(n)
                nrm.append(n / ln if ln > 1e-9 else np.zeros(3))
            cen, nrm = np.array(cen), np.array(nrm)
            print("\n=== %s  (%d glass face(s), building %.1f x %.1f x %.1f m) ==="
                  % (name, len(fi), hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]))
            for side, ax, sgn in (("S", 1, -1), ("N", 1, +1),
                                  ("W", 0, -1), ("E", 0, +1)):
                m = (nrm[:, ax] * sgn) > 0.7
                if m.sum() < 6:
                    continue
                C = cen[m]
                zs = cluster(C[:, 2], 0.9)
                us = cluster(C[:, 1 - ax], 0.7)
                dz = np.diff(zs) if len(zs) > 1 else np.array([0.0])
                du = np.diff(us) if len(us) > 1 else np.array([0.0])
                print("  %s face: %4d window(s)  ->  %2d storey line(s), "
                      "median rise %.2f m (sd %.2f)   %2d bay line(s), "
                      "median pitch %.2f m (sd %.2f)"
                      % (side, m.sum(), len(zs), np.median(dz), dz.std(),
                         len(us), np.median(du), du.std()))


for n in NAMES:
    try:
        run(n)
    except Exception as exc:
        print("%s FAILED %s" % (n, exc))
