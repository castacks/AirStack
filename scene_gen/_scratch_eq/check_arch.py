import sys, glob, os
from pxr import Usd, UsdGeom, UsdShade
d = "/isaac-sim/AirStack/scene_gen/assets/archetypes_quake"
for f in sorted(glob.glob(d + "/bld_*.usd")):
    st = Usd.Stage.Open(f)
    dp = st.GetDefaultPrim()
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    r = bc.ComputeWorldBound(dp).ComputeAlignedRange()
    mn, mx = r.GetMin(), r.GetMax()
    n_mesh = sum(1 for p in Usd.PrimRange(dp) if p.IsA(UsdGeom.Mesh))
    n_pts = 0
    unbound = 0
    for p in Usd.PrimRange(dp):
        if p.IsA(UsdGeom.Mesh):
            pts = UsdGeom.Mesh(p).GetPointsAttr().Get()
            n_pts += len(pts) if pts else 0
    print(os.path.basename(f), "up", UsdGeom.GetStageUpAxis(st), "mpu", UsdGeom.GetStageMetersPerUnit(st),
          "bbox min (%.1f %.1f %.2f) max (%.1f %.1f %.1f)" % (mn[0], mn[1], mn[2], mx[0], mx[1], mx[2]),
          "meshes", n_mesh, "pts", n_pts)
