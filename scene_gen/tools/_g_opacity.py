import sys
sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Usd, UsdShade
from detail import urban_building as ub
for nm in ("SM_MBuilding05_SkyscraperFacade_B", "SM_MBuilding04_FirstFloor_B"):
    st = Usd.Stage.Open(ub._usd(nm))
    for p in Usd.PrimRange(st.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        sh = UsdShade.Shader(p)
        if sh and sh.GetIdAttr().Get() == "UsdPreviewSurface":
            print(nm, p.GetPath(), [(i.GetBaseName(), i.Get()) for i in sh.GetInputs()
                                    if i.GetBaseName() in ("opacity", "metallic", "roughness", "ior")])
