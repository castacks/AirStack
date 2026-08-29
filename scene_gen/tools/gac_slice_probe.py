#!/usr/bin/env python
"""Run gac_slice's REAL grid estimator against the REAL GAC window data."""
import os, sys
import numpy as np
from pxr import Sdf, Usd, UsdGeom, UsdShade
sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from detail import gac_slice as gs

SEI = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
       "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")
NAMES = os.environ.get("GS_NAMES", "SM_Building_01,SM_Building_04,"
                       "SM_Building_24,SM_Building_02,SM_Building_09").split(",")


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


for name in [n.strip() for n in NAMES if n.strip()]:
    st = Usd.Stage.Open(SEI + name + ".usd")
    if st is None:
        print("%s OPEN FAILED" % name); continue
    mpu = UsdGeom.GetStageMetersPerUnit(st)
    wins = {}
    bbox = None
    for prim in Usd.PrimRange(st.GetPseudoRoot()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        me = UsdGeom.Mesh(prim)
        pts = me.GetPointsAttr().Get()
        if pts is None or not len(pts):
            continue
        P = np.asarray(pts, dtype=float) * mpu
        bbox = (tuple(P.min(axis=0)), tuple(P.max(axis=0)))
        counts = np.asarray(me.GetFaceVertexCountsAttr().Get() or [], dtype=np.int64)
        fvi = np.asarray(me.GetFaceVertexIndicesAttr().Get() or [], dtype=np.int64)
        if not len(counts) or len(fvi) != int(counts.sum()):
            continue
        start = np.concatenate([[0], np.cumsum(counts)[:-1]])
        for sub in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)):
            if not any(g in tex_of(sub.GetPrim()).lower() for g in gs.GLASS_TEX):
                continue
            for f in (sub.GetIndicesAttr().Get() or []):
                f = int(f)
                if f >= len(counts):
                    continue
                V = P[fvi[start[f]:start[f] + counts[f]]]
                cen = V.mean(axis=0)
                n = np.cross(V[1] - V[0], V[2] - V[0])
                ln = np.linalg.norm(n)
                if ln < 1e-12:
                    continue
                side = gs._side_of(*(n / ln))
                if side is None:
                    continue
                u = cen[1] if side in ("E", "W") else cen[0]
                wins.setdefault(side, []).append((float(u), float(cen[2])))
    if not wins:
        print("%-16s no glazing found" % name); continue
    g = gs.measure_grid(wins, bbox, name=name)
    ok = g.get("confidence", 0) >= gs.MIN_CONFIDENCE
    print("%-16s -> %s\n" % ("", "SLICEABLE" if ok else
                             "REFUSE (confidence %.2f < %.2f)"
                             % (g.get("confidence", 0), gs.MIN_CONFIDENCE)))
