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
import numpy as np
from pathlib import Path
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf
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

# rubble: the Nucleus DebrisConcrete set (catalogued in scene_gen/assets/debris_concrete), the
# standalone pack's slabs / chunks / lumps / rebar (already local under assets/debris), textured
# photoscan patches (scene_gen/assets/concrete_rubble_debris/split, copied to assets/fab) and the
# non-concrete things the drone footage shows in the pile: pallets, a concrete pipe, a rusty tank.
for pc in json.load(open(CODE.parent / "assets/debris_concrete/catalogue.json"))["pieces"]:
    SOURCES[f"rubble_{pc['name'].replace('SM_con_debris_', '')}"] = (NUC + pc["url"], "rubble", "Nucleus DebrisConcrete")
for d in sorted((A / "debris").iterdir()):                                     # (its chunks are open shells, its slabs curved sheets)
    if d.is_dir() and d.name.startswith(("lump", "rebar")):
        SOURCES[f"rubble_sa_{d.name}"] = (f"debris/{d.name}/{d.name}.usdc", "rubble", "standalone debris pack")
FAB_SRC = CODE.parent / "assets/concrete_rubble_debris/split"
for n in ("huge_concrete_rubble_pile", "brick_debris_pile", "concrete_slabs", "concrete_debris_elements"):
    SOURCES[f"rubble_fab_{n}"] = (f"fab/{n}/{n}.usdc", "rubble", "photoscan concrete elements pack")
SOURCES.update({
    "rubble_pallet":       ("objaverse/95d1659e3e2b4900a40a650e6906d970/95d1659e3e2b4900a40a650e6906d970.usdc", "rubble", "Objaverse 'Old Wooden Pallet'"),
    "rubble_pallet_stack": ("objaverse/e1b107ddf32f4a3981557b4ba0e63063/e1b107ddf32f4a3981557b4ba0e63063.usdc", "rubble", "Objaverse 'Wooden Pallet'"),
    "rubble_pipe":         ("objaverse/92d1cbc20e8c440aad9be60586d5efa6/92d1cbc20e8c440aad9be60586d5efa6.usdc", "rubble", "Objaverse 'Concrete pipe'"),
    "rubble_tank":         ("objaverse/74c1439281744a809160e50e10a44dca/74c1439281744a809160e50e10a44dca.usdc", "rubble", "Objaverse 'Rusty Tank'"),
    "rubble_tank_storage": ("objaverse/57be2eaf4d6b4d2980bcee4ccb35f780/57be2eaf4d6b4d2980bcee4ccb35f780.usdc", "rubble", "Objaverse 'Rusted Tank storage'"),
    "rubble_slab_plain":   ("objaverse/286a4122730f458a957a7ed6db7a750b/286a4122730f458a957a7ed6db7a750b.usdc", "rubble", "Objaverse 'Concrete slab'"),
})
# precast beams and planks -- the commonest thing in the pile (A01/B01) and in no asset set: UV'd boxes
# (UVs in metres / 2.5, the texture's footprint) written to assets/procedural/, retextured below
BEAMS = {"beam_long": (5.5, 0.5, 0.45), "beam_mid": (3.8, 0.4, 0.4), "beam_short": (2.4, 0.6, 0.5),
         "beam_thin": (4.2, 0.3, 0.3), "plank_wide": (4.5, 1.2, 0.22), "plank_short": (2.8, 1.0, 0.2)}
(A / "procedural").mkdir(exist_ok=True)
for bn, (bl, bw, bh) in BEAMS.items():
    b = Usd.Stage.CreateNew(str(A / f"procedural/{bn}.usda")); UsdGeom.SetStageUpAxis(b, "Z"); UsdGeom.SetStageMetersPerUnit(b, 1.0)
    m = UsdGeom.Mesh.Define(b, f"/{bn}"); b.SetDefaultPrim(m.GetPrim())
    P, F, ST = [], [], []
    for ax in range(3):                                                         # 6 faces, each its own 4 verts
        for sgn in (-1, 1):
            u, v = [i for i in range(3) if i != ax]; e = np.array([bl, bw, bh]) / 2
            q = [(0, 0), (1, 0), (1, 1), (0, 1)] if sgn > 0 else [(0, 0), (0, 1), (1, 1), (1, 0)]
            for cu, cv in q:
                c = np.zeros(3); c[ax] = sgn * e[ax]; c[u] = (2 * cu - 1) * e[u]; c[v] = (2 * cv - 1) * e[v]
                P.append(c + [0, 0, bh / 2]); ST.append((c[u] / 2.5, c[v] / 2.5))
            F.append([len(P) - 4 + k for k in range(4)])
    m.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in P]); m.CreateFaceVertexCountsAttr([4] * 6)
    m.CreateFaceVertexIndicesAttr([i for f in F for i in f]); m.CreateSubdivisionSchemeAttr("none"); m.CreateDoubleSidedAttr(True)
    UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying).Set([Gf.Vec2f(*map(float, t)) for t in ST])
    b.Save()
    SOURCES[f"rubble_{bn}"] = (f"procedural/{bn}.usda", "rubble", "procedural precast beam")
# The DebrisConcrete materials are 128 px, dark (sRGB ~93) and read their normal map without the
# 2x-1 remap; the standalone pieces are flat grey. Both get one tileable concrete texture, lifted to
# the pile's weathered grey-tan (drone clips A01/B01); rebar gets rust.
CONCRETE_TEX = CODE.parent / "assets/standalone/buildings/intact/tower/podium_highrise/textures"
RETEXTURE = lambda n: n.startswith("rubble_") and not n.startswith("rubble_fab_") and SOURCES[n][0].startswith((NUC, "debris/", "procedural/"))

def rubble_material(w, root, name):
    """A UsdPreviewSurface under the wrapper, bound over everything the asset binds itself."""
    mat = UsdShade.Material.Define(w, f"{root}/rubble_mat"); sh = UsdShade.Shader.Define(w, f"{root}/rubble_mat/pbs")
    sh.CreateIdAttr("UsdPreviewSurface"); mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    if name.startswith("rubble_sa_rebar"):
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.16, 0.09, 0.05))
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.6); sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.7)
    else:
        st = UsdShade.Shader.Define(w, f"{root}/rubble_mat/st"); st.CreateIdAttr("UsdPrimvarReader_float2")
        st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        for tex, inp, cs, typ, scale, bias in (
                ("BaseColor", "diffuseColor", "sRGB", "rgb", (1.3, 1.22, 1.1, 1), (0, 0, 0, 0)),
                ("ORM_rough", "roughness", "raw", "r", (1, 1, 1, 1), (0, 0, 0, 0)),
                ("Normal_norm", "normal", "raw", "rgb", (2, 2, 2, 1), (-1, -1, -1, 0))):
            t = UsdShade.Shader.Define(w, f"{root}/rubble_mat/{inp}Tex"); t.CreateIdAttr("UsdUVTexture")
            t.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(f"./textures/Concrete030_4K_{tex}.jpg")
            t.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set(cs)
            t.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*scale)); t.CreateInput("bias", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*bias))
            for k in ("wrapS", "wrapT"): t.CreateInput(k, Sdf.ValueTypeNames.Token).Set("repeat")
            t.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(), "result")
            sh.CreateInput(inp, Sdf.ValueTypeNames.Normal3f if inp == "normal" else Sdf.ValueTypeNames.Color3f if typ == "rgb" else Sdf.ValueTypeNames.Float).ConnectToSource(t.ConnectableAPI(), typ)
    UsdShade.MaterialBindingAPI.Apply(w.GetPrimAtPath(root)).Bind(mat, UsdShade.Tokens.strongerThanDescendants)

(LIB / "textures").mkdir(exist_ok=True)
for tex in ("BaseColor", "ORM_rough", "Normal_norm"):
    shutil.copy(CONCRETE_TEX / f"Concrete030_4K_{tex}.jpg", LIB / "textures")

lib = {}
for name, (rel, cls, prov) in SOURCES.items():
    src = A / rel
    if rel.startswith("objaverse/") and not src.exists():                      # copy the conversion into the data tree
        shutil.copytree(OBJ_SRC / Path(rel).parts[1], src.parent, dirs_exist_ok=True)
    if rel.startswith("fab/") and not src.exists():
        shutil.copytree(FAB_SRC / Path(rel).parts[1], src.parent, dirs_exist_ok=True)
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
    if RETEXTURE(name): rubble_material(w, f"/{name}", name)
    w.Save()
    tris = sum(sum(n - 2 for n in (UsdGeom.Mesh(p).GetFaceVertexCountsAttr().Get() or [])) for p in Usd.PrimRange(s.GetPseudoRoot(), Usd.TraverseInstanceProxies()) if p.IsA(UsdGeom.Mesh))
    lib[name] = {"usd": f"./assets/lib/{name}.usda", "size_m": size, "class": cls, "tris": tris, "provenance": prov}
    print(f"LIB {name:18s} {size} m  {tris} tris  ({prov})", flush=True)
json.dump(lib, open(LIB / "library.json", "w"), indent=1)
app.close()
