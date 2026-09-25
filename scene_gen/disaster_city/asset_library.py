"""Normalise third-party assets into one convention: data/recon/assets/lib/<name>.usda.

    OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh asset_library.py

Each wrapper references the source asset (a Nucleus mirror under assets/nucleus/, or an
Objaverse conversion copied into assets/objaverse/) and applies, in one xform:
metres, Z-up, long horizontal axis along +X, centred in X/Y, base at z = 0.
place_assets.py then scales/rotates every asset the same way. Sizes (m, after
normalisation) and triangle counts go to assets/lib/library.json.

Runs under Kit's USD: the Nucleus crates are kit-written and the pip usd-core cannot
read them.
"""
import json, shutil, sys
from pathlib import Path
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom, Sdf, Gf
sys.path.insert(0, str(Path(__file__).resolve().parent))
from _paths import R, CODE

A = R / "assets"; LIB = A / "lib"; LIB.mkdir(parents=True, exist_ok=True)
NUC = "nucleus/Projects/SEI-COA/"
OBJ_SRC = CODE.parent / "assets/objaverse"                      # scene_gen/assets/objaverse (objaverse_assets.py cache)
# name: (source relative to assets/, class, provenance)
SOURCES = {
    "semi_trailer":     (NUC + "FactoryDistrict/Meshes/SemiTrailer_mdl.usd", "vehicle", "Nucleus FactoryDistrict"),
    "container_6":      (NUC + "FactoryDistrict/Meshes/CargoContainer_Short_mdl.usd", "vehicle", "Nucleus FactoryDistrict"),
    "container_12":     (NUC + "FactoryDistrict/Meshes/CargoContainer_mdl.usd", "vehicle", "Nucleus FactoryDistrict"),
    "container_15":     (NUC + "FactoryDistrict/Meshes/CargoContainer_Long_mdl.usd", "vehicle", "Nucleus FactoryDistrict"),
    "boxcar":           (NUC + "FactoryDistrict/Meshes/CargoCar_mdl.usd", "vehicle", "Nucleus FactoryDistrict"),
    "coalcar":          (NUC + "FactoryDistrict/Meshes/CoalCar_mdl.usd", "vehicle", "Nucleus FactoryDistrict"),
    "flatcar":          (NUC + "FactoryDistrict/Meshes/TrainFlat_mdl.usd", "vehicle", "Nucleus FactoryDistrict"),
    "motorhome":        (NUC + "RetroNeighborhood/FREE_GMC_Motorhome_reimagined_low_poly.usdz", "vehicle", "Nucleus RetroNeighborhood"),
    "construction_truck": (NUC + "ModernCityEnvironment01/Meshes/SK_ConstructionTruck01FullyRigged.usd", "vehicle", "Nucleus ModernCityEnvironment01"),
    "water_tank":       (NUC + "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/SM_Water_Tank.usd", "building", "Nucleus GreatAmericanCity"),
    "tankcar":          ("objaverse/c87b96181fd249ae8de1ac14575ec475/c87b96181fd249ae8de1ac14575ec475.usdc", "vehicle", "Objaverse 'Railway tank'"),
    "tankcar_graffiti": ("objaverse/dda4e35b3d0444ce837f6db3819bd2a0/dda4e35b3d0444ce837f6db3819bd2a0.usdc", "vehicle", "Objaverse 'Graffiti Railway Tank'"),
    "locomotive":       ("objaverse/54b78d37edc248a380c5112935465dbb/54b78d37edc248a380c5112935465dbb.usdc", "vehicle", "Objaverse 'Industrial train Dr14'"),
    "box_truck":        ("objaverse/cbc508a5ad424d79b2f2c8a3de7e1cd3/cbc508a5ad424d79b2f2c8a3de7e1cd3.usdc", "vehicle", "Objaverse 'Light Box Truck'"),
    "delivery_truck":   ("objaverse/1d53f7fa474849db812102dfa5d070d0/1d53f7fa474849db812102dfa5d070d0.usdc", "vehicle", "Objaverse 'DELIVERY TRUCK'"),
    "passenger_car":    ("objaverse/1ec9ef29c1604487a2f498df7e744c88/1ec9ef29c1604487a2f498df7e744c88.usdc", "vehicle", "Objaverse 'Passenger Car(Rail)'"),
    "tankcar_cyl":      ("objaverse/c1f38cc43c694137b711c260445137de/c1f38cc43c694137b711c260445137de.usdc", "vehicle", "Objaverse 'Tank car'"),
    # rejected on the gallery: 656908d5 'Japanese Box Truck' (converted standing on end), 42e23433 'Camper Van' (a cartoon cart)
}

lib = {}
for name, (rel, cls, prov) in SOURCES.items():
    src = A / rel
    if rel.startswith("objaverse/") and not src.exists():                      # copy the conversion into the data tree
        shutil.copytree(OBJ_SRC / Path(rel).parts[1], src.parent, dirs_exist_ok=True)
    s = Usd.Stage.Open(str(src))
    mpu, up = UsdGeom.GetStageMetersPerUnit(s), UsdGeom.GetStageUpAxis(s)
    # bound in source units, then the normalising transform: units -> m, Y-up -> Z-up, long axis -> +X
    b = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(s.GetPseudoRoot()).ComputeAlignedRange()
    lo, hi = Gf.Vec3d(b.GetMin()), Gf.Vec3d(b.GetMax())
    M = Gf.Matrix4d().SetScale(mpu)
    if up == "Y": M = M * Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 90))          # +Y up -> +Z up
    corners = [M.Transform(Gf.Vec3d(x, y, z)) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    ext = [max(c[i] for c in corners) - min(c[i] for c in corners) for i in range(3)]
    if ext[1] > ext[0]: M = M * Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90))    # long side -> +X
    corners = [M.Transform(Gf.Vec3d(x, y, z)) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    mn = [min(c[i] for c in corners) for i in range(3)]; mx = [max(c[i] for c in corners) for i in range(3)]
    M = M * Gf.Matrix4d().SetTranslate(Gf.Vec3d(-(mn[0] + mx[0]) / 2, -(mn[1] + mx[1]) / 2, -mn[2]))
    size = [round(mx[i] - mn[i], 3) for i in range(3)]
    w = Usd.Stage.CreateNew(str(LIB / f"{name}.usda"))
    UsdGeom.SetStageUpAxis(w, "Z"); UsdGeom.SetStageMetersPerUnit(w, 1.0)
    root = UsdGeom.Xform.Define(w, f"/{name}"); w.SetDefaultPrim(root.GetPrim())
    inner = w.DefinePrim(f"/{name}/asset"); inner.GetReferences().AddReference(str(Path("..") / rel))
    UsdGeom.Xformable(inner).AddTransformOp().Set(M)
    root.GetPrim().SetCustomDataByKey("provenance", prov)
    w.Save()
    tris = sum(sum(n - 2 for n in (UsdGeom.Mesh(p).GetFaceVertexCountsAttr().Get() or [])) for p in Usd.PrimRange(s.GetPseudoRoot(), Usd.TraverseInstanceProxies()) if p.IsA(UsdGeom.Mesh))
    lib[name] = {"usd": f"./assets/lib/{name}.usda", "size_m": size, "class": cls, "tris": tris, "provenance": prov}
    print(f"LIB {name:18s} {size} m  {tris} tris  ({prov})", flush=True)
json.dump(lib, open(LIB / "library.json", "w"), indent=1)
app.close()
