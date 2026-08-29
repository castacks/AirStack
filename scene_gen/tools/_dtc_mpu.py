from pxr import Usd, UsdGeom
R = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
     "scene_gen/assets/downtowncity/")
for n in ("Amar_Tower", "Carved_01", "Building_11"):
    st = Usd.Stage.Open(R + n + ".usdc"); st.Load()
    S = UsdGeom.GetStageMetersPerUnit(st)
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    r = bc.ComputeWorldBound(st.GetPseudoRoot()).ComputeAlignedRange()
    a, b = r.GetMin(), r.GetMax()
    print("%-14s mpu=%-6g RAW %8.1f x %8.1f x %8.1f  -> metres %6.1f x %6.1f x %6.1f"
          % (n, S, b[0]-a[0], b[1]-a[1], b[2]-a[2],
             (b[0]-a[0])*S, (b[1]-a[1])*S, (b[2]-a[2])*S))
