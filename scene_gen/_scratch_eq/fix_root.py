import glob
from pxr import Usd, UsdGeom
n = 0
for f in sorted(glob.glob("/isaac-sim/AirStack/scene_gen/assets/archetypes_quake/bld_*.usd")):
    st = Usd.Stage.Open(f)
    dp = st.GetDefaultPrim()
    if dp.GetTypeName() != "Xform":
        dp.SetTypeName("Xform")
        st.GetRootLayer().Save()
        n += 1
print("converted", n, "roots to Xform")
