#!/usr/bin/env python
"""_o_usd_stat.py — cost census of a baked archetype USD (bare pxr, no Kit).

    scene_gen/tools/_t_pxr.sh scene_gen/tools/_o_usd_stat.py USD=<path>[,<path>...]

Reports, per file: bytes on disk, `Usd.Stage.Open` wall time, full-traverse
time, prim / mesh / material / subset counts, points and faces, and a
breakdown by the prim-name prefix the quake path authors (`frag`, `lump`,
`heap`, kit `LOD*`, ...) so it is obvious WHICH population dominates.

`REF=<n>` additionally times composing <n> references to the file onto a
fresh stage (composition + traversal only — no Hydra, so this is the USD
half of a city load, not the whole of it).
"""
import json
import os
import sys
import time
from collections import defaultdict

from pxr import Usd, UsdGeom, UsdShade


def _prefix(name):
    """Collapse `frag_0123`, `lump_7_12` to `frag`, `lump`."""
    out = []
    for part in name.split("_"):
        if part.isdigit() or (part[:1].isdigit() and part[1:].isdigit()):
            break
        out.append(part)
    return "_".join(out) or name


def stat_one(path, ref=0):
    size = os.path.getsize(path) if os.path.exists(path) else 0
    t0 = time.time()
    st = Usd.Stage.Open(path)
    t_open = time.time() - t0
    if st is None:
        return {"usd": path, "error": "open failed"}
    t1 = time.time()
    prims = mesh = mat = subset = shader = 0
    pts = faces = fv = 0
    by_pre = defaultdict(lambda: [0, 0, 0])       # prims, points, faces
    by_mat = defaultdict(lambda: [0, 0])          # meshes, faces
    norm_sig = defaultdict(int)
    pvar_sig = defaultdict(int)
    for p in st.Traverse():
        prims += 1
        tn = p.GetTypeName()
        if tn == "Material":
            mat += 1
            continue
        if tn == "Shader":
            shader += 1
            continue
        if tn == "GeomSubset":
            subset += 1
            continue
        if not p.IsA(UsdGeom.Mesh):
            continue
        mesh += 1
        m = UsdGeom.Mesh(p)
        pv = m.GetPointsAttr().Get() or []
        fc = m.GetFaceVertexCountsAttr().Get() or []
        fi = m.GetFaceVertexIndicesAttr().Get() or []
        pts += len(pv)
        faces += len(fc)
        fv += len(fi)
        pre = _prefix(p.GetName())
        by_pre[pre][0] += 1
        by_pre[pre][1] += len(pv)
        by_pre[pre][2] += len(fc)
        na = m.GetNormalsAttr()
        norm_sig["{0}/{1}".format(
            "n" if na.HasAuthoredValue() else "-",
            m.GetNormalsInterpolation() if na.HasAuthoredValue() else "")] += 1
        names = tuple(sorted(
            "{0}:{1}".format(v.GetName(), v.GetInterpolation())
            for v in UsdGeom.PrimvarsAPI(p).GetPrimvars()
            if v.HasAuthoredValue()))
        pvar_sig[names] += 1
        b = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]
        key = b.GetPrim().GetName() if b and b.GetPrim().IsValid() else "<none>"
        by_mat[key][0] += 1
        by_mat[key][1] += len(fc)
    t_trav = time.time() - t1
    out = {"usd": os.path.basename(path), "mb": round(size / 1e6, 2),
           "open_s": round(t_open, 3), "traverse_s": round(t_trav, 3),
           "prims": prims, "meshes": mesh, "materials": mat, "shaders": shader,
           "subsets": subset, "points": pts, "faces": faces, "facevtx": fv,
           "n_mat_bound": len(by_mat),
           "by_prefix": {k: v for k, v in sorted(
               by_pre.items(), key=lambda kv: -kv[1][0])[:14]},
           "by_material": {k: v for k, v in sorted(
               by_mat.items(), key=lambda kv: -kv[1][0])[:14]},
           "normals": dict(norm_sig),
           "primvar_sigs": {"|".join(k) or "<none>": v
                            for k, v in pvar_sig.items()}}
    if ref:
        t2 = time.time()
        s2 = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(s2, "/W")
        for i in range(int(ref)):
            h = UsdGeom.Xform.Define(s2, "/W/b{0}".format(i)).GetPrim()
            h.GetReferences().AddReference(path)
            UsdGeom.Xformable(h).AddTranslateOp().Set((60.0 * i, 0.0, 0.0))
        n = sum(1 for _ in s2.Traverse())
        out["ref{0}_compose_s".format(int(ref))] = round(time.time() - t2, 2)
        out["ref{0}_prims".format(int(ref))] = n
    return out


def main():
    paths = [q for q in os.environ.get("USD", "").split(",") if q.strip()]
    ref = int(os.environ.get("REF") or "0")
    if not paths:
        print("USD=<path[,path]> required")
        return
    rows = []
    for p in paths:
        p = p.strip()
        r = stat_one(p, ref=ref)
        rows.append(r)
        print(json.dumps(r, indent=1))
    out = os.environ.get("OUT", "").strip()
    if out:
        with open(out, "w") as fh:
            json.dump(rows, fh, indent=1)
        print("wrote " + out)


main()
