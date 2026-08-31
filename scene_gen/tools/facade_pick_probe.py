#!/usr/bin/env python
"""facade_pick_probe — WHICH SUBSET OF A TORN PIECE IS ITS FACADE, and which
one `fire_collapse.facade_skin` actually picked?

The offline answer to the row-5 review (2026-08-30):

    "/World/bake/g7/brk_g7_wall_E_4_06_0090/frag_001 ... it's like the interior
     office material not the outside glass window one"

Prints, for every piece matching a regex (INACTIVE ONES INCLUDED — a torn
piece is deactivated by `quake_flow._break_split`, so a plain `Traverse` no
longer sees it; this walks `Usd.TraverseAll`):

  * per GeomSubset: its material, that material's diffuse map, the number of
    OUTWARD-facing triangles it owns (normal . outward > 0.30), their area,
    and how far OUT along the piece's outward axis those faces sit (the
    median and the 90th percentile of face-centroid . outward, in metres).
    That last pair is the whole point: a curtain wall's fake-interior office
    card faces outward too — you are meant to see it THROUGH the glass — so
    an area-only pick can choose the card that sits BEHIND the glass.
  * per `brk_*` group of that piece: what its fragments are actually bound to.

    usd_python.sh facade_pick_probe.py <bake.usd> <regex> [S|E|N|W]

The side letter gives the outward axis; with none it is taken from the piece
name (`wall_E_4_06_0090` -> E -> +x), the same convention `gac_storey_slice`
names its cells by.
"""
import re
import sys

import numpy as np
from pxr import Usd, UsdGeom, UsdShade

st = Usd.Stage.Open(sys.argv[1])
rx = re.compile(sys.argv[2] if len(sys.argv) > 2 else "wall_E_4_06")
SIDE = (sys.argv[3].upper() if len(sys.argv) > 3 else None)
OUT = {"S": (0.0, -1.0, 0.0), "N": (0.0, 1.0, 0.0),
       "E": (1.0, 0.0, 0.0), "W": (-1.0, 0.0, 0.0)}


def diffuse(mpath):
    if not mpath:
        return "(none)", "(none)"
    mp = st.GetPrimAtPath(mpath)
    if not mp or not mp.IsValid():
        return "(expired)", "(none)"
    for c in mp.GetChildren():
        sh = UsdShade.Shader(c)
        if not sh:
            continue
        for nm in ("diffuseColor", "diffuse_color_constant", "base_color",
                   "diffuse_texture"):
            i = sh.GetInput(nm)
            if i is None:
                continue
            try:
                if i.HasConnectedSource():
                    src = i.GetConnectedSource()[0].GetPrim()
                    f = UsdShade.Shader(src).GetInput("file")
                    v = f.Get() if f else None
                    return mp.GetName(), "tex:" + str(v).rsplit("/", 1)[-1].rstrip("@")
                v = i.Get()
            except Exception:
                continue
            if v is not None:
                return mp.GetName(), (("tex:" + str(v).rsplit("/", 1)[-1].rstrip("@"))
                                      if nm == "diffuse_texture"
                                      else "rgb:" + str(v))
    return mp.GetName(), "(no diffuse)"


def bound(prim):
    try:
        m = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    except Exception:
        return None
    return m.GetPrim().GetPath().pathString if (m and m.GetPrim().IsValid()) else None


def side_of(name):
    for tok in ("_S_", "_E_", "_N_", "_W_"):
        if tok in name:
            return tok[1]
    return "S"


xf = UsdGeom.XformCache()
for pr in Usd.PrimRange.Stage(st, Usd.PrimAllPrimsPredicate):
    p = pr.GetPath().pathString
    if not rx.search(p) or pr.GetTypeName() != "Mesh":
        continue
    if "/brk_" in p:
        print("%-62s -> %s %s" % (p[-62:], bound(pr) and
                                  diffuse(bound(pr))[0], diffuse(bound(pr))[1]))
        continue
    nm = pr.GetName()
    out = np.asarray(OUT[SIDE or side_of(nm)], dtype=float)
    print("%s   active=%s   outward=%s%s" % (p, pr.IsActive(),
                                             SIDE or side_of(nm), out))
    m = UsdGeom.Mesh(pr)
    pts = m.GetPointsAttr().Get()
    cnt = m.GetFaceVertexCountsAttr().Get()
    idx = m.GetFaceVertexIndicesAttr().Get()
    if not pts or not cnt or not idx:
        continue
    M = np.array(xf.GetLocalToWorldTransform(pr), dtype=float)
    P = np.asarray([[q[0], q[1], q[2]] for q in pts], dtype=float)
    P = P @ M[:3, :3] + M[3, :3]
    counts = np.asarray(cnt, dtype=np.int64)
    ind = np.asarray(idx, dtype=np.int64)
    starts = np.concatenate(([0], np.cumsum(counts)[:-1]))
    subs = [c for c in pr.GetChildren() if c.GetTypeName() == "GeomSubset"]
    groups = [(list(map(int, s.GetIndicesAttr().Get() or [])), s) for s in subs]
    if not groups:
        groups = [(list(range(len(counts))), pr)]
    for fids, target in groups:
        tri = []
        for f in fids:
            n = int(counts[f])
            s0 = int(starts[f])
            for k in range(1, n - 1):
                tri.append((int(ind[s0]), int(ind[s0 + k]), int(ind[s0 + k + 1])))
        if not tri:
            continue
        T = P[np.asarray(tri, dtype=np.int64)]
        nrm = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        ln = np.linalg.norm(nrm, axis=1)
        keep = (ln > 1e-12) & ((nrm @ out) > 0.30 * ln)
        mp = bound(target)
        mnm, mdf = diffuse(mp)
        if not keep.any():
            print("   sub %-22s %-34s %-40s  outward tris 0" %
                  (target.GetName(), mnm, mdf))
            continue
        d = (T[keep].mean(axis=1) @ out)
        print("   sub %-22s %-34s %-40s  outward tris %5d  area %7.2f m2  "
              "d_out med %7.3f  p90 %7.3f  max %7.3f" %
              (target.GetName(), mnm, mdf, int(keep.sum()),
               0.5 * float(ln[keep].sum()), float(np.median(d)),
               float(np.percentile(d, 90)), float(d.max())))
