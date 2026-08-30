#!/usr/bin/env python
"""Does `soot_plume.piece_material_like` compose on a REAL kit module? Bare
USD in the container (usd_python.sh), no SimulationApp."""
import sys
sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Usd, UsdGeom, UsdShade, Sdf
from detail import urban_building as ub
from disaster import soot_plume as spl

for name in sys.argv[1:] or ["SM_MBuilding04_Facade_A", "SM_MBuilding01_Facade_A"]:
    st = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(st, "/World")
    mod = st.DefinePrim("/World/mod_" + name)
    mod.GetReferences().AddReference(ub._usd(name))
    n_ok = 0
    for prim in Usd.PrimRange(mod):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        subsets = list(UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)))
        targets = [s.GetPrim() for s in subsets] or [prim]
        for t in targets:
            bound = UsdShade.MaterialBindingAPI(t).ComputeBoundMaterial()[0]
            bprim = bound.GetPrim() if bound else None
            sh_path, inp, tex = spl.find_basecolor(bprim)
            print("== {0} target {1}: material {2}".format(name, t.GetName(), bprim.GetPath() if bprim else None))
            print("   basecolor shader {0} input {1} = {2}".format(sh_path, inp, (tex or "").rsplit("/", 1)[-1]))
            if not tex:
                continue
            n_orig = sum(1 for p in Usd.PrimRange(bprim) if UsdShade.Shader(p))
            mp = "/World/FireLooks/soot_{0}".format(n_ok)
            mat = spl.piece_material_like(st, mp, bprim, sh_path, inp, "/tmp/merged_test.png")
            if mat is None:
                print("   piece_material_like -> None (fallback would be used)")
                continue
            n_copy = sum(1 for p in Usd.PrimRange(mat.GetPrim()) if UsdShade.Shader(p))
            rel = Sdf.Path(sh_path).MakeRelativePath(bprim.GetPath())
            cp = st.GetPrimAtPath(Sdf.Path(mp).AppendPath(rel)) if str(rel) not in (".", "") else mat.GetPrim()
            sh = UsdShade.Shader(cp)
            got = sh.GetInput(inp).Get() if sh and sh.GetInput(inp) else None
            others = []
            for p in Usd.PrimRange(mat.GetPrim()):
                s2 = UsdShade.Shader(p)
                if not s2:
                    continue
                for i2 in s2.GetInputs():
                    try:
                        v = i2.Get()
                    except Exception:
                        continue
                    if isinstance(v, Sdf.AssetPath) and v.path and "merged_test" not in v.path:
                        others.append(v.path.rsplit("/", 1)[-1])
            UsdShade.MaterialBindingAPI(t).Bind(mat)
            nb = UsdShade.MaterialBindingAPI(t).ComputeBoundMaterial()[0]
            print("   copy {0}: {1} shader prim(s) (orig {2}); basecolor now = {3}; other maps kept: {4}; rebound -> {5}".format(
                mp, n_copy, n_orig, got.path if got else None, others[:4], nb.GetPath() if nb else None))
            # surface output survives?
            out = mat.GetSurfaceOutput("mdl") or mat.GetSurfaceOutput()
            print("   surface output: {0}".format(out.GetAttr().GetPath() if out else None))
            n_ok += 1
