#!/usr/bin/env python
"""Can an opening table be DERIVED from a merged building's glass subset?

`r_window_burnout` — the crazed-then-black-void progression that carries the
kit's look — needs to know where the windows ARE. The kit gets that from a
measured table per family. A merged whole-asset mesh has no table, which is
why `burn_monolith` never emptied a single window.

But several packs DO model their glazing as its own material subset. If those
faces cluster into separated islands, each island IS a window, and the table
can be derived geometrically instead of authored. This measures that.

Also reports whether an asset is really one mesh or a bag of INSTANCED parts —
the AEC brownstones failed `monolith_damage.cut_shell` with a Tf error because
it traverses instance proxies and then writes to them, which means they are
composed of instanced sub-prims and may be addressable after de-instancing.

    docker exec isaac-sim bash scene_gen/tools/usd_python.sh \
        scene_gen/tools/openings_probe.py
"""
import numpy as np
from pxr import Sdf, Usd, UsdGeom, UsdShade

SEI = "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
LIB = "omniverse://airlab-nucleus.andrew.cmu.edu:443/Library/Stages/"
GLASS = ("glass", "window", "curtain", "glazing")

TARGETS = [
    ("GAC SM_Building_01", SEI + "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/SM_Building_01.usd"),
    ("GAC SM_Building_04", SEI + "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/SM_Building_04.usd"),
    ("GAC SM_Building_24", SEI + "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/SM_Building_24.usd"),
    ("MuyangDT BG_Building_A", LIB + "Muyang/DownTown/Assets/BG_Building_A.usd"),
    ("Dmytro Building_TypeA_A", LIB + "Dmytro/Assets/Game/FactoryDistrict/Meshes/Building_TypeA_A.usd"),
]
BROWNSTONE = ("AEC Reference_Brownstone02",
              "/isaac-sim/AirStack/scene_gen/assets/aec/brownstone/Assets/"
              "Create_Brownstone02/Reference_Brownstone02.usd")


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


def islands(P, counts, fvi, faces):
    """Cluster the given face indices into connected components by shared
    vertex position (welded to 1 mm), and return each island's bbox."""
    start = np.concatenate([[0], np.cumsum(counts)[:-1]])
    key = {}
    parent = {}

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    for f in faces:
        parent.setdefault(f, f)
    for f in faces:
        for k in range(counts[f]):
            v = fvi[start[f] + k]
            q = tuple(np.round(P[v], 3))
            if q in key:
                union(f, key[q])
            else:
                key[q] = f
    groups = {}
    for f in faces:
        groups.setdefault(find(f), []).append(f)
    out = []
    for g in groups.values():
        vs = np.unique(np.concatenate(
            [fvi[start[f]:start[f] + counts[f]] for f in g]))
        V = P[vs]
        out.append(V.max(axis=0) - V.min(axis=0))
    return out


def report(label, url):
    st = Usd.Stage.Open(url)
    if st is None:
        print("%-26s OPEN FAILED" % label); return
    mpu = UsdGeom.GetStageMetersPerUnit(st)
    n_inst = sum(1 for p in Usd.PrimRange(st.GetPseudoRoot(),
                                          Usd.TraverseInstanceProxies())
                 if p.IsInstanceProxy())
    meshes = [p for p in Usd.PrimRange(st.GetPseudoRoot(),
                                       Usd.TraverseInstanceProxies())
              if p.IsA(UsdGeom.Mesh)]
    print("%-26s mpu=%-5.3g meshes=%-4d instance-proxy prims=%d"
          % (label, mpu, len(meshes), n_inst))
    found = False
    for prim in meshes:
        me = UsdGeom.Mesh(prim)
        pts = me.GetPointsAttr().Get()
        if pts is None or not len(pts):
            continue
        P = np.asarray(pts, dtype=float) * mpu
        counts = np.asarray(me.GetFaceVertexCountsAttr().Get() or [], dtype=np.int64)
        fvi = np.asarray(me.GetFaceVertexIndicesAttr().Get() or [], dtype=np.int64)
        if not len(counts) or len(fvi) != int(counts.sum()):
            continue
        for sub in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)):
            tex = tex_of(sub.GetPrim()).lower()
            if not any(g in tex for g in GLASS):
                continue
            fi = np.asarray(sub.GetIndicesAttr().Get() or [], dtype=np.int64)
            fi = fi[(fi >= 0) & (fi < len(counts))]
            if not len(fi):
                continue
            isl = islands(P, counts, fvi, list(fi))
            sz = np.array([sorted(s)[-2:] for s in isl])  # two largest extents
            print("%-26s   glass subset '%s': %d face(s) -> %d island(s)"
                  % ("", tex[:34], len(fi), len(isl)))
            if len(isl):
                print("%-26s   island size median %.2f x %.2f m, "
                      "%d in the 0.5-4 m window band"
                      % ("", np.median(sz[:, 0]), np.median(sz[:, 1]),
                         sum(1 for s in sz if 0.5 <= s[0] <= 4.0 and 0.5 <= s[1] <= 4.0)))
            found = True
    if not found:
        print("%-26s   NO glass subset — windows are painted into the texture"
              % "")
    print()


for lab, u in TARGETS:
    try:
        report(lab, u)
    except Exception as exc:
        print("%-26s FAILED %s\n" % (lab, exc))
try:
    report(*BROWNSTONE)
except Exception as exc:
    print("brownstone FAILED %s" % exc)
