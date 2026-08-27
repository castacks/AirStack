import sys
sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Usd, UsdGeom, Sdf, Gf
st = Usd.Stage.CreateInMemory()
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(st, 1.0)
from disaster import quake_flow as qf
import random
for usd, sc, w in qf.FURNITURE["urm"][:3] + qf.FURNITURE["rc"][:3]:
    path = "/p_" + usd.rsplit("/",1)[-1].split(".")[0]
    r = qf._prop(st, path, usd, 10.0, 5.0, 3.0, 30.0, sc, random.Random(1))
    pr = st.GetPrimAtPath(path)
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    rg = bc.ComputeWorldBound(pr).ComputeAlignedRange()
    print(usd.rsplit("/",1)[-1], "->", r is not None, "empty" if rg.IsEmpty() else (rg.GetMin(), rg.GetMax()), "children", len(list(pr.GetChildren())), pr.GetTypeName())
