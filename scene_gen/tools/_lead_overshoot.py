import sys
from pxr import Usd, UsdGeom
stage = Usd.Stage.Open(sys.argv[1])
bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
PL = (-200.0, -120.0, 200.0, 120.0)
hits = {}
for prim in stage.Traverse():
    if prim.GetTypeName() != "Mesh" or not prim.IsActive():
        continue
    r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        continue
    mn, mx = r.GetMin(), r.GetMax()
    over = max(PL[0] - mn[0], PL[1] - mn[1], mx[0] - PL[2], mx[1] - PL[3])
    if over > 0.5:
        path = prim.GetPath().pathString
        segs = path.split("/")
        root = "/".join(segs[:5])
        cur = hits.setdefault(root, [0, 0.0, ""])
        cur[0] += 1
        if over > cur[1]:
            cur[1], cur[2] = over, path + " y1=%.1f x=[%.0f,%.0f]" % (mx[1], mn[0], mx[0])
print("mesh subtrees overshooting the plate by >0.5 m:")
for root, (n, over, worst) in sorted(hits.items()):
    print("  %-55s n=%-4d worst=%.1fm  %s" % (root, n, over, worst))
