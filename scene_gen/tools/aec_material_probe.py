"""aec_material_probe -- dump the raw shader graph of a handful of AEC
brownstone materials (source stage, not baked) and test `soot_plume.
find_basecolor` / `gac_fire._diffuse_of` against them directly.

    docker exec isaac-sim bash -c "cd /isaac-sim/AirStack && \
      ./scene_gen/tools/usd_python.sh scene_gen/tools/aec_material_probe.py \
      Reference_Brownstone5Row"
"""
import sys

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Usd, UsdGeom, UsdShade                        # noqa: E402
from disaster import gac_fire as gf, soot_plume as spl        # noqa: E402

name = sys.argv[1] if len(sys.argv) > 1 else "Reference_Brownstone5Row"
url = gf.asset_url(name, "aec")
print("opening", url)
st = Usd.Stage.Open(url)
root = st.GetPseudoRoot()

seen = 0
for prim in Usd.PrimRange(root):
    if not prim.IsA(UsdShade.Material):
        continue
    seen += 1
    if seen > 6:
        break
    print("\n=== MATERIAL", prim.GetPath(), "===")
    for c in Usd.PrimRange(prim):
        sh = UsdShade.Shader(c)
        if not sh:
            continue
        sid = sh.GetIdAttr().Get()
        try:
            src_asset = sh.GetSourceAsset("mdl")
        except Exception:
            src_asset = None
        try:
            src_sub = sh.GetSourceAssetSubIdentifier("mdl")
        except Exception:
            src_sub = None
        print("  shader", c.GetPath(), "id=", sid, "mdl_asset=", src_asset,
              "mdl_subident=", src_sub)
        ins = list(sh.GetInputs())
        print("    {0} input(s)".format(len(ins)))
        for inp in ins[:40]:
            try:
                v = inp.Get()
            except Exception as exc:
                v = "ERR({0})".format(exc)
            print("      {0} : {1!r}".format(inp.GetFullName(), v))

    sh_path, inp_name, tex = spl.find_basecolor(prim)
    print("  find_basecolor ->", sh_path, inp_name, tex)
    sh_path2, inp_name2, tex2 = gf._diffuse_of(prim)
    print("  _diffuse_of     ->", sh_path2, inp_name2, tex2)

print("\ntotal materials seen (capped at 6 printed):", seen)
