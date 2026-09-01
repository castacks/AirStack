"""THROWAWAY: on a LIVE slice, do the planner's glass pieces carry any
glazing subset? Slices SM_Building_02, runs the planner at T3, then for each
piece in plan["glass"] lists (subset, tex basename, mat name, is_glazing);
then counts glazing subsets over ALL pieces by role; then lists the SOURCE
mesh's subsets with their texture basenames."""
import collections, json, random, sys
sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Usd, UsdGeom, UsdShade
from detail import gac_slice as gsl, gac_storey_slice as gss
from disaster import fracture, gac_fire as gcf, quake_flow as qf, quake_sliced as qs
from disaster import tornado as tn, tornado_urban as tu, tornado_urban_usd as tuu

fracture.ensure_vtk(verbose=False)
kind, asset = gcf.split_kind(sys.argv[1] if len(sys.argv) > 1 else "SM_Building_02")
url = gcf.asset_url(asset, kind); scale = gcf.asset_scale(url, gcf.PACKS[kind]["scale"], verbose=False)
st = Usd.Stage.CreateInMemory(); UsdGeom.SetStageMetersPerUnit(st, 1.0); UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(st, "/W"); st.SetDefaultPrim(st.GetPrimAtPath("/W")); UsdGeom.Scope.Define(st, "/W/bench")
cell = "/W/bench/g0"; UsdGeom.Xform.Define(st, cell)
src = gcf.place_source(st, cell, url, scale)
btype = qs.construction_type(asset)
pls, grid, measured = gss.slice_to_kit(st, src, cell, "tp2_" + asset.lower(), region=None,
                                       family={"urm": "01", "rc": "02", "rc_glass": "05"}.get(btype, "01"), verbose=False)
info = qf.describe("tp2_" + asset.lower(), pls, 0.0, 0.0, 0.0)
wind = {"bearing_deg": 57.6, "speed_frac": 1.0, "cross_frac": -0.33, "over": False}
plan = tu.plan_damage(info, info["elements"], "T3", btype, random.Random(7), wind,
                      height_class=tu.height_class_for(info["H"]), intensity=0.8)

def subsets_of(path):
    prim = st.GetPrimAtPath(path)
    out = []
    for mesh in Usd.PrimRange(prim):
        if not mesh.IsA(UsdGeom.Mesh):
            continue
        for s in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(mesh)):
            mat = UsdShade.MaterialBindingAPI(s.GetPrim()).ComputeBoundMaterial()[0]
            tex, mname = tuu._glass_tex_and_name(mat.GetPrim()) if mat else ("", "")
            out.append((s.GetPrim().GetName(), tex, mname, bool(gsl.is_glazing(tex, mat_name=mname)),
                        mat.GetPath().pathString if mat else None))
    return out

print("PLAN glass pieces:", len(plan["glass"]))
for p in plan["glass"][:6]:
    print("  ", p.rsplit("/", 1)[-1], subsets_of(p))
by_role = collections.Counter(); glz_role = collections.Counter(); subs_per_piece = collections.Counter()
for pl in pls:
    role = pl.get("_role"); by_role[role] += 1
    subs = subsets_of(pl["prim_path"]); subs_per_piece[len(subs)] += 1
    if any(s[3] for s in subs):
        glz_role[role] += 1
print("pieces by role:", dict(by_role)); print("pieces with a glazing subset by role:", dict(glz_role))
print("subsets per piece histogram:", dict(subs_per_piece))
print("SOURCE subsets:")
for mesh in Usd.PrimRange(st.GetPrimAtPath(src)):
    if not mesh.IsA(UsdGeom.Mesh):
        continue
    for s in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(mesh)):
        mat = UsdShade.MaterialBindingAPI(s.GetPrim()).ComputeBoundMaterial()[0]
        tex, mname = tuu._glass_tex_and_name(mat.GetPrim()) if mat else ("", "")
        n = len(UsdGeom.Subset(s).GetIndicesAttr().Get() or [])
        print("   ", s.GetPrim().GetName(), n, "faces  tex", tex, "mat", mname, "glazing", bool(gsl.is_glazing(tex, mat_name=mname)))
print("apply ->", json.dumps({k: v for k, v in tuu.apply_plan(st, {"stage": st, "parent": cell, "tag": "tp", "mats": {}, "static_extra": [], "loose": [], "authored": [], "info": info}, plan, verbose=False).items() if k != "notes"}))
