import json, sys
from pxr import Usd, UsdGeom
st = Usd.Stage.Open(sys.argv[1])
sc = st.GetPrimAtPath(sys.argv[2] if len(sys.argv) > 2 else "/World/bake/g6/pieces")
bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
out = {}
for c in sc.GetChildren():
    b = bc.ComputeWorldBound(c).ComputeAlignedRange()
    if b.IsEmpty():
        continue
    lo, hi = b.GetMin(), b.GetMax()
    out[c.GetName()] = [float(lo[0]), float(lo[1]), float(lo[2]),
                        float(hi[0]), float(hi[1]), float(hi[2])]
print("JSONSTART")
print(json.dumps(out))
