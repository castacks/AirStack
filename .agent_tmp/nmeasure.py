"""Measure Nucleus/local USDs with standalone pxr — no SimulationApp.

argv: one or more USD urls. Prints TSV: url<TAB>mpu<TAB>up<TAB>sx<TAB>sy<TAB>sz<TAB>base<TAB>pts
Sizes are in METRES (metersPerUnit applied); no extra scale.
"""
import sys
from pxr import Usd, UsdGeom, Gf


def measure(url):
    st = Usd.Stage.Open(url)
    if st is None:
        return None
    mpu = UsdGeom.GetStageMetersPerUnit(st) or 1.0
    up = UsdGeom.GetStageUpAxis(st) or "Z"
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    bb = cache.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
    if bb.IsEmpty():
        return None
    mn, mx = bb.GetMin(), bb.GetMax()
    d = [(mx[i] - mn[i]) * mpu for i in range(3)]
    lo = [mn[i] * mpu for i in range(3)]
    if str(up) == "Y":
        sx, sy, sz, base = d[0], d[2], d[1], lo[1]
    else:
        sx, sy, sz, base = d[0], d[1], d[2], lo[2]
    pts = 0
    for p in st.Traverse():
        if p.IsA(UsdGeom.Mesh):
            a = UsdGeom.Mesh(p).GetPointsAttr().Get()
            pts += len(a) if a else 0
    return mpu, str(up), sx, sy, sz, base, pts


for u in sys.argv[1:]:
    try:
        m = measure(u)
    except Exception as e:
        print(f"{u}\tERROR\t{e}")
        continue
    if m is None:
        print(f"{u}\tEMPTY")
    else:
        print("%s\t%.6g\t%s\t%.3f\t%.3f\t%.3f\t%.3f\t%d" % (u, *m))
