"""The "autumn" look: data/recon/disaster_city_autumn.usda -- the same scene, graded to the drone video.

    ~/.venvs/recon/bin/python ground_texture.py --season autumn && ~/.venvs/recon/bin/python autumn.py

disaster_city.usda is "summer" (the Google tiles' look). Autumn is ONLY an aesthetic layer over it:
the stage references summer's /World and overrides looks, never placements -- every building, vehicle,
survivor, rubble piece and tree stays exactly where summer has it. Overrides:
  * light: sun 3000 -> 2200, near neutral; sky dome 1000 -> 750 (the video's hazy, less saturated exposure);
  * ground: the ground material reads ground_tex_autumn/ (dormant straw grass, tan concrete and sand,
    darker; ground_texture.py --season autumn) instead of ground_tex/;
  * trees: each tree instance points at assets/trees/<species>_autumn.usda (written here): a fall oak from
    the NVIDIA vegetation library (the video's trees are post oaks: A01, A04, B05, B11) with KEEP of its leaf
    cards (clumped, so the crown thins rather than speckles) tinted brown, scaled to the summer tree's
    height and set on its base -- so the swap moves nothing.
"""
import os
import numpy as np
from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf, Vt
from _paths import R

KEEP, TINT = 0.05, (0.45, 0.40, 0.33)                                   # the video's oaks are near bare
T = R / "assets/trees"
OAKS = R / "assets/nucleus/NVIDIA/Assets/Vegetation/Trees"
# each summer species -> a fall oak (NVIDIA vegetation library, mirrored from Nucleus): the video's trees are post oaks
SWAP = {"American_Beech": "Shumard_Oak_Fall", "White_Ash": "Scarlet_Oak_fall", "Honey_Locust": "Black_Oak_Fall", "Largetooth_Aspen": "Shumard_Oak_Fall"}

def thin(src, dst, seed):
    """copy src -> dst (same folder, so relative material paths hold) with KEEP of its leaf cards, clumped, tinted"""
    Sdf.Layer.FindOrOpen(str(src)).Export(str(dst))
    st = Usd.Stage.Open(str(dst)); rng = np.random.default_rng(seed); kept = total = 0
    for p in st.Traverse():
        if not p.IsA(UsdGeom.Mesh): continue
        mat = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]
        if not mat or not any(w in mat.GetPath().name.lower() for w in ("leaf", "leaves", "fall")): continue
        if any(a_.IsA(UsdGeom.PointInstancer) for a_ in [p.GetParent()]): continue      # prototype: thinned by instance below
        m = UsdGeom.Mesh(p); cnt = np.array(m.GetFaceVertexCountsAttr().Get()); idx = np.array(m.GetFaceVertexIndicesAttr().Get())
        P = np.array(m.GetPointsAttr().Get()); start = np.r_[0, np.cumsum(cnt)[:-1]]
        cen = np.add.reduceat(P[idx], start) / cnt[:, None]
        cell = np.floor(cen / (np.ptp(P, 0).max() / 12)).astype(np.int64)      # ~12 clumps across the crown
        h = (cell[:, 0] * 73856093 ^ cell[:, 1] * 19349663 ^ cell[:, 2] * 83492791) % 1000 / 1000.0
        keep = (h < KEEP) | (rng.random(len(cnt)) < KEEP * 0.15)         # clumps kept, plus a few stray leaves
        fv = np.concatenate([np.arange(s_, s_ + c) for s_, c, k in zip(start, cnt, keep) if k]) if keep.any() else np.zeros(0, int)
        m.GetFaceVertexCountsAttr().Set(Vt.IntArray(cnt[keep].tolist())); m.GetFaceVertexIndicesAttr().Set(Vt.IntArray(idx[fv].tolist()))
        for pv in UsdGeom.PrimvarsAPI(m).GetPrimvars():                  # face-varying data follows the kept faces
            if pv.GetInterpolation() == UsdGeom.Tokens.faceVarying and pv.Get() is not None and not pv.IsIndexed():
                v_ = pv.Get(); pv.Set(type(v_).FromNumpy(np.asarray(v_)[fv]))
        if m.GetNormalsInterpolation() == UsdGeom.Tokens.faceVarying and m.GetNormalsAttr().Get():
            n_ = m.GetNormalsAttr().Get(); m.GetNormalsAttr().Set(type(n_).FromNumpy(np.asarray(n_)[fv]))
        for sh in Usd.PrimRange(mat.GetPrim()):
            if sh.IsA(UsdShade.Shader): UsdShade.Shader(sh).CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*TINT))
        kept += int(keep.sum()); total += len(cnt)
    leafy = lambda q: any(w in (UsdShade.MaterialBindingAPI(q).ComputeBoundMaterial()[0].GetPath().name.lower() if UsdShade.MaterialBindingAPI(q).ComputeBoundMaterial()[0] else "")
                          for w in ("leaf", "leaves", "fall"))
    for p in st.Traverse():                                          # leaves carried as point-instanced twigs: drop instances
        if not p.IsA(UsdGeom.PointInstancer): continue
        pi = UsdGeom.PointInstancer(p); protos = [st.GetPrimAtPath(t) for t in pi.GetPrototypesRel().GetTargets()]
        if not protos or not all(any(leafy(q) for q in Usd.PrimRange(pr) if q.IsA(UsdGeom.Mesh)) for pr in protos): continue
        P = np.array(pi.GetPositionsAttr().Get()); n_ = len(P)
        if not n_: continue
        cell = np.floor(P / (np.ptp(P, 0).max() / 10 + 1e-6)).astype(np.int64)
        h = (cell[:, 0] * 73856093 ^ cell[:, 1] * 19349663 ^ cell[:, 2] * 83492791) % 1000 / 1000.0
        keep = (h < KEEP) | (rng.random(n_) < KEEP * 0.15)
        for attr in (pi.GetPositionsAttr(), pi.GetOrientationsAttr(), pi.GetScalesAttr(), pi.GetProtoIndicesAttr(), pi.GetIdsAttr(),
                     pi.GetVelocitiesAttr(), pi.GetAngularVelocitiesAttr()):
            v_ = attr.Get()
            if v_ is not None and len(v_) == n_: attr.Set(type(v_).FromNumpy(np.asarray(v_)[keep]))
        kept += int(keep.sum()); total += n_
        for pr in protos:
            for q in Usd.PrimRange(pr):
                if q.IsA(UsdGeom.Mesh) and leafy(q):
                    for sh in Usd.PrimRange(UsdShade.MaterialBindingAPI(q).ComputeBoundMaterial()[0].GetPrim()):
                        if sh.IsA(UsdShade.Shader): UsdShade.Shader(sh).CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*TINT))
    st.Save(); return kept, total

def bound(f):
    st_ = Usd.Stage.Open(str(f)); return UsdGeom.BBoxCache(0, ["default", "render"]).ComputeWorldBound(st_.GetPseudoRoot()).ComputeAlignedRange()
for oak in sorted(set(SWAP.values())):
    k, t = thin(OAKS / f"{oak}.usd", OAKS / f"{oak}_thin.usd", len(oak)); print(f"{oak}: kept {k}/{t} leaf faces")
for sp, oak in SWAP.items():                     # a wrapper in the summer species' own units, scaled to its height
    bs, bo = bound(T / f"{sp}.usd"), bound(OAKS / f"{oak}_thin.usd")
    k = bs.GetSize()[2] / bo.GetSize()[2]; co = bo.GetMidpoint(); cs = bs.GetMidpoint()
    w = Usd.Stage.CreateNew(str(T / f"{sp}_autumn.usda")); UsdGeom.SetStageUpAxis(w, "Z"); UsdGeom.SetStageMetersPerUnit(w, 0.01)
    r = UsdGeom.Xform.Define(w, "/Root"); w.SetDefaultPrim(r.GetPrim())
    a_ = w.DefinePrim("/Root/tree"); a_.GetReferences().AddReference(os.path.relpath(OAKS / f"{oak}_thin.usd", T))
    x = UsdGeom.Xformable(a_)                                            # base on the summer tree's base, centred on its trunk
    x.AddTranslateOp().Set(Gf.Vec3d(cs[0] - k * co[0], cs[1] - k * co[1], bs.GetMin()[2] - k * bo.GetMin()[2])); x.AddScaleOp().Set(Gf.Vec3f(k))
    w.Save(); print(f"{sp}_autumn.usda: {oak} x {k:.2f} (height {bs.GetSize()[2]:.0f} units, as the summer tree)")

# the autumn stage: summer's /World by reference, looks overridden
au = Usd.Stage.CreateNew(str(R / "disaster_city_autumn.usda"))
UsdGeom.SetStageUpAxis(au, "Z"); UsdGeom.SetStageMetersPerUnit(au, 1.0)
w = au.DefinePrim("/World"); w.GetReferences().AddReference("./disaster_city.usda"); au.SetDefaultPrim(w)
sun = UsdLux.DistantLight(au.OverridePrim("/World/Environment/sun")); sun.CreateIntensityAttr(2200.0); sun.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.93))
sky = UsdLux.DomeLight(au.OverridePrim("/World/Environment/sky")); sky.CreateIntensityAttr(750.0)
# (same sky HDR as summer: the clear-sky HDRs tried had fields baked into the horizon)
tex = au.OverridePrim("/World/site/ortho_mat/tex")
tex.CreateAttribute("inputs:file", Sdf.ValueTypeNames.Asset).Set("./ground_tex_autumn/ortho.<UDIM>.jpg")
summer = Usd.Stage.Open(str(R / "disaster_city.usda")); n = 0
for p in summer.GetPrimAtPath("/World/trees/instances").GetChildren() if summer.GetPrimAtPath("/World/trees/instances") else []:
    refs = p.GetMetadata("references")
    items = list(refs.GetAddedOrExplicitItems()) if refs else []
    if not items: continue
    path = str(items[0].assetPath)
    if path.endswith("_autumn.usd"): continue
    o = au.OverridePrim(p.GetPath()); o.GetReferences().SetReferences([Sdf.Reference(path.replace(".usd", "_autumn.usda"))]); n += 1
au.Save()
print(f"wrote {R / 'disaster_city_autumn.usda'}: light, ground and {n} trees overridden; placements are summer's")
