#!/usr/bin/env python
"""piece_soot_probe — what material is ACTUALLY bound on named pieces in a bake USD.

    usd_python.sh piece_soot_probe.py <bake.usd> <regex> [<regex>...]

For every Mesh whose path matches a regex: each GeomSubset's bound material,
its UsdPreviewSurface's diffuse source (texture basename or constant) and
roughness/metallic — enough to tell a sooted copy (fire_/soot atlas png)
from a raw tiled atlas, and a matte from a mirror. (fire_dtc3 review:
corner_NE_0_10_0070 unburnt with a repeating pattern amid burnt walls.)"""
import re, sys
from pxr import Usd, UsdShade

st = Usd.Stage.Open(sys.argv[1])
rx = [re.compile(p) for p in sys.argv[2:]]

def sinfo(mat):
    if not mat:
        return "(none)"
    for c in Usd.PrimRange(mat.GetPrim()):
        sh = UsdShade.Shader(c)
        if sh and sh.GetIdAttr().Get() == "UsdPreviewSurface":
            bits = []
            for name in ("diffuseColor", "roughness", "metallic", "opacity"):
                i = sh.GetInput(name)
                if not i:
                    continue
                if i.HasConnectedSource():
                    ts = UsdShade.Shader(i.GetConnectedSource()[0].GetPrim())
                    f = ts.GetInput("file")
                    v = f.Get() if f else None
                    bits.append("%s=tex:%s" % (name, str(v).rsplit("/", 1)[-1][:64]))
                else:
                    bits.append("%s=%s" % (name, i.Get()))
            return "%s  [%s]" % (mat.GetPrim().GetName(), " ".join(bits))
    return mat.GetPrim().GetName() + " (no preview surface)"

for pr in st.Traverse():
    p = pr.GetPath().pathString
    if not any(r.search(p) for r in rx):
        continue
    if pr.GetTypeName() == "Mesh":
        subs = [c for c in pr.GetChildren() if c.GetTypeName() == "GeomSubset"]
        if subs:
            print(p)
            for c in subs:
                m = UsdShade.MaterialBindingAPI(c).ComputeBoundMaterial()[0]
                print("   sub %-28s -> %s" % (c.GetName(), sinfo(m)))
        else:
            m = UsdShade.MaterialBindingAPI(pr).ComputeBoundMaterial()[0]
            print("%s -> %s" % (p, sinfo(m)))
