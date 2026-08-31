"""_dtc_uv_census — how TILED are a merged asset's UVs, per material?

`soot_bake.uv_position_map` rasterises triangle by triangle in Python. For a
triangle whose WRAPPED uv span exceeds 0.5 in either axis it falls back to the
RAW corners and clamps the scan box to one full period — a px x px meshgrid,
4.2M cells at px=2048. A handful of those is fine; thousands is the whole
runtime. GAC atlases are uniquely unwrapped; a downtowncity block tiles brick,
roof tile and marble maps many times over, so this census says how much of
`gac_fire.bake_atlases`' cost is going into texels that the shared-texel test
is about to throw away anyway.
"""
import sys, time
sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
import numpy as np
from pxr import Usd, UsdGeom
from detail import gac_storey_slice as gss
from disaster import gac_fire as gf

NAME = sys.argv[1] if len(sys.argv) > 1 else "dtc:Carved_18"
kind, asset = gf.split_kind(NAME)
pack = gf.PACKS[kind]
url = gf.asset_url(asset, kind)
st = Usd.Stage.CreateInMemory()
UsdGeom.SetStageMetersPerUnit(st, 1.0)
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(st, "/W")
st.SetDefaultPrim(st.GetPrimAtPath("/W"))
UsdGeom.Xform.Define(st, "/W/g0")
t0 = time.time()
src = gf.place_source(st, "/W/g0", url, gf.asset_scale(url, pack["scale"], verbose=False))
mesh = gss.read_mesh(st, src, verbose=False)
print("read_mesh %.1f s: %d tri(s), %d material(s)"
      % (time.time() - t0, len(mesh["tris"]), len(mesh["mats"])))
UV = mesh["UV"]; MID = mesh["MID"]; tris = mesh["tris"]
tu = UV[tris]                                   # (T,3,2)
wrapped = tu - np.floor(tu)
span = wrapped.max(axis=1) - wrapped.min(axis=1)
straddle = (span[:, 0] > 0.5) | (span[:, 1] > 0.5)
raw_span = tu.max(axis=1) - tu.min(axis=1)
print("TOTAL %d tri(s): %d (%.1f%%) take the RAW/seam-straddler path"
      % (len(tris), int(straddle.sum()), 100.0 * straddle.mean()))
print("%-42s %8s %8s %7s %9s" % ("material", "tris", "straddle", "%", "max raw span"))
rows = []
for k, m in enumerate(mesh["mats"]):
    sel = MID == k
    n = int(sel.sum())
    if not n:
        continue
    s = int(straddle[sel].sum())
    rows.append((s, n, k, m))
for s, n, k, m in sorted(rows, reverse=True)[:20]:
    nm = m.GetPrim().GetName() if (m is not None and m.GetPrim().IsValid()) else "(none)"
    sel = MID == k
    print("%-42s %8d %8d %6.1f%% %9.1f"
          % (nm[:42], n, s, 100.0 * s / n, float(raw_span[sel].max())))
