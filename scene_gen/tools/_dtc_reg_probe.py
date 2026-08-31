"""_dtc_reg_probe — throwaway: what the asset registry needs to know.

  1. metersPerUnit of each pack, read three ways (Sdf layer field, a
     LoadNone stage, a full stage) so the registry can pick the cheap one.
  2. every MATERIAL PRIM NAME on a GAC asset -- the freeze question: does
     adding a material-name test to `gac_slice.is_glazing` change GAC?
  3. Amar_Tower's bbox with and without its baked-in landscaping.
"""
import sys, time
sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
import numpy as np
from pxr import Sdf, Usd, UsdGeom, UsdShade

GAC = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
       "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")
DTC = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
       "scene_gen/assets/downtowncity/")

TARGETS = [(GAC + "SM_Building_02.usd", "gac SM_Building_02"),
           (GAC + "SM_Building_09.usd", "gac SM_Building_09"),
           (DTC + "Amar_Tower.usdc", "dtc Amar_Tower"),
           (DTC + "Building_12.usdc", "dtc Building_12"),
           (DTC + "Carved_13.usdc", "dtc Carved_13")]

print("== 1. metersPerUnit ==")
for url, tag in TARGETS:
    t0 = time.time()
    lay = Sdf.Layer.FindOrOpen(url)
    f = None
    if lay is not None:
        try:
            f = lay.GetField(Sdf.Path.absoluteRootPath, "metersPerUnit")
        except Exception as exc:
            f = "ERR %s" % exc
    t1 = time.time()
    st = Usd.Stage.Open(url, Usd.Stage.LoadNone)
    ln = UsdGeom.GetStageMetersPerUnit(st) if st else None
    t2 = time.time()
    print("  %-22s layer-field=%-8s (%.2fs)  LoadNone=%-8s (%.2fs)"
          % (tag, f, t1 - t0, ln, t2 - t1))

print()
print("== 2. GAC material prim names (is a material-name glazing test a no-op there?) ==")
from detail import gac_slice as gsl
for url, tag in TARGETS:
    st = Usd.Stage.Open(url)
    st.Load()
    names, hits = set(), set()
    for p in Usd.PrimRange(st.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if not p.IsA(UsdShade.Material):
            continue
        n = p.GetName()
        names.add(n)
        if gsl.is_glazing(n):
            hits.add(n)
    print("  %-22s %3d material prim(s); names: %s"
          % (tag, len(names), ", ".join(sorted(names))[:170]))
    print("      is_glazing(NAME) fires on %d: %s" % (len(hits), sorted(hits)))

print()
print("== 3. Amar_Tower bbox with / without baked-in landscaping ==")
GREEN = ("grass", "tree", "leaves", "leaf", "trunk", "bark", "branch",
         "platanus", "robinia", "tilia", "hedge", "shrub", "plant", "foliage")
st = Usd.Stage.Open(DTC + "Amar_Tower.usdc"); st.Load()
S = UsdGeom.GetStageMetersPerUnit(st) or 1.0
xc = UsdGeom.XformCache()
lo_all = np.full(3, np.inf); hi_all = np.full(3, -np.inf)
lo_b = np.full(3, np.inf); hi_b = np.full(3, -np.inf)
area_green = 0.0
def mat_name(prim):
    m = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    return m.GetPrim().GetName() if (m and m.GetPrim().IsValid()) else ""
for prim in Usd.PrimRange(st.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
    if not prim.IsA(UsdGeom.Mesh):
        continue
    me = UsdGeom.Mesh(prim)
    pts = me.GetPointsAttr().Get()
    if pts is None or not len(pts):
        continue
    M = np.asarray(xc.GetLocalToWorldTransform(prim), dtype=float)
    P = np.asarray(pts, dtype=float)
    P = (np.c_[P, np.ones(len(P))] @ M)[:, :3] * S
    counts = np.asarray(me.GetFaceVertexCountsAttr().Get() or [], dtype=np.int64)
    fvi = np.asarray(me.GetFaceVertexIndicesAttr().Get() or [], dtype=np.int64)
    lo_all = np.minimum(lo_all, P.min(0)); hi_all = np.maximum(hi_all, P.max(0))
    if not len(counts) or len(fvi) != int(counts.sum()):
        continue
    start = np.concatenate([[0], np.cumsum(counts)[:-1]])
    subs = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim))
    if not subs:
        nm = mat_name(prim)
        if not any(g in nm.lower() for g in GREEN):
            lo_b = np.minimum(lo_b, P.min(0)); hi_b = np.maximum(hi_b, P.max(0))
        continue
    for s in subs:
        nm = mat_name(s.GetPrim())
        green = any(g in nm.lower() for g in GREEN)
        idx = np.asarray(s.GetIndicesAttr().Get() or [], dtype=np.int64)
        idx = idx[(idx >= 0) & (idx < len(counts))]
        if not len(idx):
            continue
        vids = np.concatenate([fvi[start[f]:start[f] + counts[f]] for f in idx])
        V = P[vids]
        if green:
            area_green += float(len(idx))
            print("     GREEN subset %-34s %6d face(s)  x[%7.1f %7.1f] y[%7.1f %7.1f] z[%6.1f %6.1f]"
                  % (nm, len(idx), V[:,0].min(), V[:,0].max(),
                     V[:,1].min(), V[:,1].max(), V[:,2].min(), V[:,2].max()))
        else:
            lo_b = np.minimum(lo_b, V.min(0)); hi_b = np.maximum(hi_b, V.max(0))
print("  ALL      W=%.2f D=%.2f H=%.2f   x[%.2f %.2f] y[%.2f %.2f] z[%.2f %.2f]"
      % (hi_all[0]-lo_all[0], hi_all[1]-lo_all[1], hi_all[2]-lo_all[2],
         lo_all[0], hi_all[0], lo_all[1], hi_all[1], lo_all[2], hi_all[2]))
print("  NO-GREEN W=%.2f D=%.2f H=%.2f   x[%.2f %.2f] y[%.2f %.2f] z[%.2f %.2f]"
      % (hi_b[0]-lo_b[0], hi_b[1]-lo_b[1], hi_b[2]-lo_b[2],
         lo_b[0], hi_b[0], lo_b[1], hi_b[1], lo_b[2], hi_b[2]))
print("  delta W=%.2f m  D=%.2f m  H=%.2f m   centre dx=%.2f dy=%.2f"
      % ((hi_all[0]-lo_all[0])-(hi_b[0]-lo_b[0]),
         (hi_all[1]-lo_all[1])-(hi_b[1]-lo_b[1]),
         (hi_all[2]-lo_all[2])-(hi_b[2]-lo_b[2]),
         0.5*(lo_all[0]+hi_all[0])-0.5*(lo_b[0]+hi_b[0]),
         0.5*(lo_all[1]+hi_all[1])-0.5*(lo_b[1]+hi_b[1])))
