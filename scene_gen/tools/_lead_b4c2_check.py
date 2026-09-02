import sys
from pxr import Usd, UsdShade, UsdGeom
stage = Usd.Stage.Open(sys.argv[1])
print("== B4/B5 industrial_debris meshes ==")
for cell in ("B4", "B5"):
    root = stage.GetPrimAtPath("/World/tornado_bench/%s/cell" % cell)
    if not root:
        continue
    for prim in Usd.PrimRange(root):
        if prim.GetTypeName() != "Mesh":
            continue
        mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        mpath = mat.GetPrim().GetPath().pathString if mat else "UNBOUND"
        row = "%s -> %s" % (prim.GetPath().pathString.split("cell/")[-1], mpath)
        if mat:
            sh = UsdShade.Shader.Get(stage, mat.GetPrim().GetPath().AppendChild("Shader"))
            if sh:
                c = sh.GetInput("diffuse_color_constant")
                t = sh.GetInput("diffuse_tint")
                tex = sh.GetInput("diffuse_texture")
                row += "  const=%s tint=%s tex=%s" % (
                    c.Get() if c else None, t.Get() if t else None,
                    bool(tex.Get()) if tex else False)
            else:
                row += "  NO-SHADER-CHILD"
        # displayColor opinion?
        dc = UsdGeom.Gprim(prim).GetDisplayColorAttr().Get()
        if dc:
            row += "  displayColor=%s" % dc[0]
        print(" ", row)
print("== C2 stain prims ==")
root = stage.GetPrimAtPath("/World/tornado_bench/C2/cell")
n = 0
for prim in Usd.PrimRange(root):
    p = prim.GetPath().pathString
    if "stain" in p.lower() or "overlay" in p.lower():
        n += 1
        if prim.GetTypeName() == "Mesh":
            bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
            r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
            box = None if r.IsEmpty() else [round(v, 2) for v in
                  (r.GetMin()[0], r.GetMin()[1], r.GetMin()[2],
                   r.GetMax()[0], r.GetMax()[1], r.GetMax()[2])]
            mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
            sh = UsdShade.Shader.Get(stage, mat.GetPrim().GetPath().AppendChild("Shader")) if mat else None
            op = None
            if sh:
                for name in ("opacity_constant", "cutout_opacity", "opacity"):
                    i = sh.GetInput(name)
                    if i and i.Get() is not None:
                        op = (name, i.Get()); break
            print("  %s box=%s opacity=%s" % (p.split("cell/")[-1], box, op))
print("  (%d stain-ish prims total)" % n)
