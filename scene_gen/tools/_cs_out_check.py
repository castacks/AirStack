import glob, os, sys
from pxr import Usd, UsdGeom, UsdShade
d = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "assets", "citysample")
fs = sorted(glob.glob(os.path.join(d, "*.usdc")))
print("%d local modules" % len(fs))
for f in fs[:6]:
    st = Usd.Stage.Open(f)
    m = [p for p in st.Traverse() if p.IsA(UsdGeom.Mesh)][0]
    me = UsdGeom.Mesh(m)
    n = len(me.GetFaceVertexCountsAttr().Get() or [])
    pv = UsdGeom.PrimvarsAPI(m).GetPrimvar("st")
    mat = UsdShade.MaterialBindingAPI(m).ComputeBoundMaterial()[0]
    tex = "-"
    if mat:
        for c in Usd.PrimRange(mat.GetPrim()):
            sh = UsdShade.Shader(c)
            if sh and sh.GetIdAttr().Get() == "UsdUVTexture":
                v = sh.GetInput("file").Get()
                tex = (v.path.rsplit("/",1)[-1] if v else "?")
    print("  %-46s tris %5d  st=%s  tex=%s"
          % (os.path.basename(f)[:46], n, bool(pv and pv.HasValue()), tex[:30]))
