"""THROWAWAY: what does a KIT (MCE/DW/CIV) window module actually bind?
Builds brownstone_row + dw_terrace kits on a bare stage, then for every
authored module mesh prints: piece name, n subsets, bound material name,
resolved texture basename, is_glazing verdict. Groups by (mat, tex)."""
import collections, sys
sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Usd, UsdGeom, UsdShade
from detail import gac_slice as gsl
from disaster import kit_substitute as ksub, tornado_urban_usd as tuu

for style in (sys.argv[1:] or ["brownstone_row", "dw_terrace"]):
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, "/W"); st.SetDefaultPrim(st.GetPrimAtPath("/W"))
    cell = "/W/cell"; UsdGeom.Xform.Define(st, cell)
    pls = ksub.build_kit(st, cell + "/parts", style, seed=7, ssf=1.0)
    groups = collections.Counter(); win_rows = []
    for p in (pls or []):
        prim = st.GetPrimAtPath(p["prim_path"]) if p.get("prim_path") else None
        if not prim or not prim.IsValid():
            continue
        for mesh in Usd.PrimRange(prim):
            if not mesh.IsA(UsdGeom.Mesh):
                continue
            subs = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(mesh))
            mat = UsdShade.MaterialBindingAPI(mesh).ComputeBoundMaterial()[0]
            tex, mname = tuu._glass_tex_and_name(mat.GetPrim()) if mat else ("", "")
            g = bool(gsl.is_glazing(tex, mat_name=mname))
            groups[(len(subs), mname, tex, g)] += 1
            nm = (p.get("category") or "") + "|" + prim.GetName()
            if any(k in nm.lower() for k in ("win", "wnd", "glass", "door")):
                subrows = []
                for sb in subs:
                    smat = UsdShade.MaterialBindingAPI(sb.GetPrim()).ComputeBoundMaterial()[0]
                    stex, smn = tuu._glass_tex_and_name(smat.GetPrim()) if smat else ("", "")
                    subrows.append((sb.GetPrim().GetName(), smn, stex[:44],
                                    bool(gsl.is_glazing(stex, mat_name=smn))))
                win_rows.append((nm, subrows))
    print("=====", style, len(pls or []), "placements")
    for k, v in sorted(groups.items(), key=lambda kv: -kv[1])[:14]:
        print("  %4d x subsets=%d mat=%-34s tex=%-52s glazing=%s" % (v, k[0], k[1][:34], k[2][:52], k[3]))
    print("  window-ish modules:", len(win_rows))
    for r in win_rows[:6]:
        print("   ", r)
