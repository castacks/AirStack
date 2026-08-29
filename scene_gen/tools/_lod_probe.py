"""The GreatAmericanCity LOD ladder, measured. Bare pxr."""
import os, sys
from pxr import Usd, UsdGeom
NUC = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
       "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")
NAMES = ["SM_Building_23", "SM_Building_11", "SM_Building_08",
         "SM_Building_16", "SM_Building_31", "SM_Building_02"]
n_with = 0
tot = {}
for nm in NAMES:
    st = Usd.Stage.Open(NUC + nm + ".usd"); st.Load()
    S = UsdGeom.GetStageMetersPerUnit(st)
    holder = None
    for p in st.Traverse():
        if p.GetVariantSets().HasVariantSet("LOD"):
            holder = p; break
    if holder is None:
        print("%-18s no LOD variant set" % nm); continue
    n_with += 1
    vs = holder.GetVariantSet("LOD")
    row = []
    for v in vs.GetVariantNames():
        vs.SetVariantSelection(v)
        st.Load()
        pts = tri = 0
        for q in st.Traverse():
            if q.IsA(UsdGeom.Mesh):
                a = UsdGeom.Mesh(q).GetPointsAttr().Get()
                pts += len(a) if a is not None else 0
                c = UsdGeom.Mesh(q).GetFaceVertexCountsAttr().Get()
                if c is not None:
                    tri += sum(max(0, int(k) - 2) for k in c)
        row.append((v, pts, tri))
        tot.setdefault(v, [0, 0])
        tot[v][0] += pts; tot[v][1] += tri
    base = row[0][2] or 1
    print("%-18s %s" % (nm, "  ".join(
        "%s %6.0fk tri (%3.0f%%)" % (v, t/1000.0, 100.0*t/base)
        for v, p, t in row)))
print("\n%d of %d carry the ladder" % (n_with, len(NAMES)))
if tot:
    b = tot[sorted(tot)[0]][1] or 1
    print("summed over those: " + "  ".join(
        "%s %.1fM tri (%.0f%%)" % (v, tot[v][1]/1e6, 100.0*tot[v][1]/b)
        for v in sorted(tot)))
