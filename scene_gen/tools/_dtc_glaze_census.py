"""_dtc_glaze_census — which faces did the MATERIAL-NAME glazing test add?

`gac_slice.is_glazing` now takes the bound material's own prim NAME as a
second chance, because a downtowncity window material carries no diffuse map.
This prints, per asset, every material the name test claims that the texture
test did NOT — with its triangle count — so the risk of a false friend
(a window air-conditioner, a rooftop lantern frame) is a number, not a guess.
"""
import sys
sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
import numpy as np
from pxr import Usd, UsdGeom
from detail import gac_slice as gsl, gac_storey_slice as gss
from disaster import gac_fire as gf

for NAME in (sys.argv[1:] or ["dtc:Carved_18"]):
    kind, asset = gf.split_kind(NAME)
    pack = gf.PACKS[kind]
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0); UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, "/W"); st.SetDefaultPrim(st.GetPrimAtPath("/W"))
    UsdGeom.Xform.Define(st, "/W/g0")
    url = gf.asset_url(asset, kind)
    src = gf.place_source(st, "/W/g0", url, gf.asset_scale(url, pack["scale"], verbose=False))
    mesh = gss.read_mesh(st, src, verbose=False)
    MID = mesh["MID"]
    tot = len(mesh["tris"])
    by_tex = by_name = 0
    rows = []
    for k, m in enumerate(mesh["mats"]):
        if m is None or not m.GetPrim().IsValid():
            continue
        n = int((MID == k).sum())
        if not n:
            continue
        nm = m.GetPrim().GetName()
        _sp, _in, url_t = gf._diffuse_of(m.GetPrim())
        tex = (url_t or "").rsplit("/", 1)[-1]
        t_hit = gsl.is_glazing(tex)
        n_hit = gsl.is_glazing("", mat_name=nm)
        if t_hit:
            by_tex += n
        if t_hit or n_hit:
            by_name += n
        if n_hit and not t_hit:
            rows.append((n, nm, tex or "(no diffuse map)"))
    print("\n=== %s: %d tri(s); glazing by TEXTURE %d (%.1f%%), "
          "by TEXTURE-or-NAME %d (%.1f%%)"
          % (asset, tot, by_tex, 100.0 * by_tex / tot, by_name, 100.0 * by_name / tot))
    for n, nm, tex in sorted(rows, reverse=True):
        print("    +%7d tri(s)  %-34s tex=%s" % (n, nm[:34], tex[:46]))
    if not rows:
        print("    (the name test adds nothing on this asset)")
