from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

from scene_gen.disaster import material_audit, material_repair


def _material(stage, path, color=(.3, .2, .1)):
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def _mesh(stage, path):
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(0, 0, 0), Gf.Vec3f(1, 0, 0),
                           Gf.Vec3f(0, 1, 0)])
    mesh.CreateFaceVertexCountsAttr([3])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2])
    return mesh


def _broken(mesh, target):
    mesh.GetPrim().CreateRelationship("material:binding").SetTargets([target])


def test_known_vendor_defects_are_repaired_without_geometry_changes():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World/stage")

    glass_root = "/World/stage/Brownstone02_Instanced"
    _material(stage, glass_root + "/Looks/Glazed_Glass", (.1, .2, .25))
    glass = _mesh(stage, glass_root + "/Geometry/Windows/glass")
    _broken(glass, glass_root + "/Looks_01/Glazed_Glass")

    tree_root = "/World/stage/tree"
    _material(stage, tree_root + "/Looks/TreeBark_7")
    bark = _mesh(stage, tree_root + "/Black_Oak_branch1")
    _broken(bark, tree_root + "/Looks/Black_Oak_bark_Mat")

    bollard = _mesh(stage, "/World/stage/bollard/g_Bollard_Emissive")
    _broken(bollard, "/World/stage/bollard/Looks/Default")

    factory = UsdGeom.Mesh.Define(stage, "/World/stage/factory")
    factory.CreatePointsAttr([Gf.Vec3f(0, 0, 0), Gf.Vec3f(1, 0, 0),
                              Gf.Vec3f(0, 1, 0)])
    factory.CreateFaceVertexCountsAttr([3])
    factory.CreateFaceVertexIndicesAttr([0, 1, 2])
    section7 = UsdGeom.Subset.Define(stage, "/World/stage/factory/Section7")
    section7.CreateElementTypeAttr(UsdGeom.Tokens.face)
    section7.CreateFamilyNameAttr("materialBind")
    section7.CreateIndicesAttr(list(range(128)))
    UsdShade.MaterialBindingAPI.Apply(section7.GetPrim()).Bind(
        _material(stage, "/World/stage/factory/Section7/UnrealMaterial"))
    section10 = UsdGeom.Subset.Define(stage, "/World/stage/factory/Section10")
    section10.CreateElementTypeAttr(UsdGeom.Tokens.face)
    section10.CreateFamilyNameAttr("materialBind")
    section10.CreateIndicesAttr(list(range(64)))

    before = {str(p.GetPath()): list(UsdGeom.Mesh(p).GetPointsAttr().Get())
              for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)}
    report = material_repair.repair_known(stage)
    assert report["total_repaired"] == 4
    assert report["total_unresolved_known"] == 0
    for prim in (glass.GetPrim(), bark.GetPrim(), bollard.GetPrim(),
                 section10.GetPrim()):
        material = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial(
            materialPurpose=UsdShade.Tokens.full)[0]
        assert material and material.GetPrim().IsA(UsdShade.Material)
    after = {str(p.GetPath()): list(UsdGeom.Mesh(p).GetPointsAttr().Get())
             for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)}
    assert before == after
    assert material_audit.audit(stage)["ok"]


def test_unknown_missing_material_is_not_hidden():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World/stage")
    mesh = _mesh(stage, "/World/stage/unknown")
    _broken(mesh, "/World/stage/Looks/MissingSomethingElse")
    report = material_repair.repair_known(stage)
    assert report["total_repaired"] == 0
    assert not material_audit.audit(stage)["ok"]


def test_cross_scope_material_is_cloned_from_composed_source_and_rebound():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World/stage")
    source = _material(stage, "/World/stage/SharedLooks/BurnBand")
    UsdGeom.Xform.Define(stage, "/World/burnGround")
    band = _mesh(stage, "/World/burnGround/band_0")
    UsdShade.MaterialBindingAPI.Apply(band.GetPrim()).Bind(source)

    report = material_repair.repair_cross_scope(stage)
    assert report == {"looks_moved": 1, "rebound": 1, "unresolved": 0}
    bound = UsdShade.MaterialBindingAPI(
        band.GetPrim()).ComputeBoundMaterial(
            materialPurpose=UsdShade.Tokens.full)[0]
    assert bound
    assert str(bound.GetPath()).startswith("/World/burnGround/Looks/")
    assert material_audit.audit(stage)["ok"]


def test_local_shader_path_is_repaired_with_one_collection_override():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World/stage")
    material = _material(stage, "/World/stage/Tree/Looks/Leaves")
    shader = UsdShade.Shader.Get(
        stage, "/World/stage/Tree/Looks/Leaves/Shader")
    shader.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath("/isaac-sim/AirStack/scene_gen/assets/tree/leaves.png"))
    mesh = _mesh(stage, "/World/stage/Tree/leaves")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)

    report = material_repair.repair_local_material_paths(
        stage, lambda path: "omniverse://server/assets/tree/leaves.png")
    assert report["materials_found"] == 1
    assert report["materials_overridden"] == 1
    assert report["gprims_rebound"] == 1
    assert report["unresolved_paths"] == []
    bound = UsdShade.MaterialBindingAPI(
        mesh.GetPrim()).ComputeBoundMaterial(
            materialPurpose=UsdShade.Tokens.full)[0]
    assert str(bound.GetPath()).startswith(
        "/World/stage/_AssetMaterialRepairs/PortableLooks/")
    assets = []
    for prim in Usd.PrimRange(bound.GetPrim()):
        for attr in prim.GetAttributes():
            if attr.GetTypeName() == Sdf.ValueTypeNames.Asset:
                assets.append(attr.Get().path)
    assert assets == ["omniverse://server/assets/tree/leaves.png"]


def test_instance_proxy_material_overrides_are_scoped_to_instance_roots():
    asset = Usd.Stage.CreateInMemory()
    root = UsdGeom.Xform.Define(asset, "/Asset")
    asset.SetDefaultPrim(root.GetPrim())
    material = _material(asset, "/Asset/Looks/Leaves")
    shader = UsdShade.Shader.Get(asset, "/Asset/Looks/Leaves/Shader")
    shader.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath("/build/tree/leaves.png"))
    mesh = _mesh(asset, "/Asset/leaves")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World/stage")
    for index in range(2):
        instance = stage.DefinePrim("/World/stage/tree_{0}".format(index))
        instance.GetReferences().AddReference(asset.GetRootLayer().identifier)
        instance.SetInstanceable(True)

    report = material_repair.repair_local_material_paths(
        stage, lambda path: "omniverse://server/tree/leaves.png")
    assert report["materials_overridden"] == 1
    assert report["gprims_rebound"] == 2
    assert report["direct_bindings"] == 0
    assert report["instance_collections"] == 2
    # The global stage root no longer carries hundreds of unrelated material
    # collections; each instance sees only the overrides relevant to itself.
    global_names = [rel.GetName() for rel in
                    stage.GetPrimAtPath("/World/stage").GetRelationships()]
    assert not any("portableMaterial" in name for name in global_names)
    for index in range(2):
        proxy = stage.GetPrimAtPath(
            "/World/stage/tree_{0}/leaves".format(index))
        assert proxy.IsInstanceProxy()
        bound = UsdShade.MaterialBindingAPI(proxy).ComputeBoundMaterial(
            materialPurpose=UsdShade.Tokens.full)[0]
        assert bound
        assert str(bound.GetPath()).startswith(
            "/World/stage/_AssetMaterialRepairs/PortableLooks/")


def test_material_prim_asset_input_and_overlay_stay_portable_and_in_scope():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World/stage")
    source = _material(stage, "/World/stage/Looks/BurnBand")
    source.GetPrim().CreateAttribute(
        "inputs:diffuse_texture", Sdf.ValueTypeNames.Asset).Set(
            Sdf.AssetPath("/build/materials/burn.png"))
    UsdGeom.Xform.Define(stage, "/World/burnGround")
    band = _mesh(stage, "/World/burnGround/band_0")
    UsdShade.MaterialBindingAPI.Apply(band.GetPrim()).Bind(source)

    first = material_repair.repair_cross_scope(stage)
    assert first["rebound"] == 1
    portable = material_repair.repair_local_material_paths(
        stage, lambda path: "omniverse://server/materials/burn.png")
    assert portable["unresolved_paths"] == []
    second = material_repair.repair_cross_scope(stage)
    assert second["rebound"] == 1

    bound = UsdShade.MaterialBindingAPI(
        band.GetPrim()).ComputeBoundMaterial(
            materialPurpose=UsdShade.Tokens.full)[0]
    assert str(bound.GetPath()).startswith("/World/burnGround/Looks/")
    raw_assets = []
    for prim in Usd.PrimRange(bound.GetPrim()):
        for attr in prim.GetAttributes():
            if attr.GetTypeName() == Sdf.ValueTypeNames.Asset:
                raw_assets.append(attr.Get().path)
    assert raw_assets == ["omniverse://server/materials/burn.png"]
