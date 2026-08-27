import sys, random
sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
import numpy as np
from pxr import Usd, UsdGeom, Sdf, Gf
from disaster import fracture
st = Usd.Stage.CreateInMemory()
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(st, 1.0)
KIT = "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/ModernCityEnvironment01/Meshes/"
for name, pos in (("SM_MBuilding02_Facade_A", (0, 0, 3)), ("SM_MBuilding02_Facade_A", (420.0, 610.0, 3)),
                  ("SM_MBuilding01_Facade_A", (0, 0, 6)), ("SM_MBuilding01_Facade_A", (400.0, 700.0, 6))):
    path = "/p_%s_%d" % (name, int(pos[0]))
    pr = st.DefinePrim(path)
    pr.GetReferences().AddReference(KIT + name + ".usd")
    pr.Load()
    xf = UsdGeom.Xformable(pr)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    mesh = fracture.prim_to_mesh(st, path)
    if mesh is None:
        print(name, pos, "prim_to_mesh None"); continue
    nrng = np.random.default_rng(3)
    frags = fracture.fracture_mesh(mesh, 10, nrng, mode="uniform", rough=0.012, consume=0.0, min_volume_frac=0.002)
    print(name, pos, "faces", len(mesh.faces), "extents", np.round(mesh.extents, 2), "-> frags", len(frags), "backend", "vtk" if fracture._vtk() else "trimesh")
