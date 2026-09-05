from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

from scene_gen.disaster import material_audit


def _mesh(stage, path="/W/mesh"):
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(0, 0, 0), Gf.Vec3f(1, 0, 0),
                           Gf.Vec3f(0, 1, 0)])
    mesh.CreateFaceVertexCountsAttr([3])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2])
    return mesh


def _material(stage, path="/W/Looks/mat", surface=True):
    mat = UsdShade.Material.Define(stage, path)
    if surface:
        shader = UsdShade.Shader.Define(stage, path + "/shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        mat.CreateSurfaceOutput().ConnectToSource(
            shader.ConnectableAPI(), "surface")
    return mat


def test_valid_material_and_deliberate_display_color_pass():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/W")
    bound = _mesh(stage, "/W/bound")
    UsdShade.MaterialBindingAPI.Apply(bound.GetPrim()).Bind(_material(stage))
    colored = _mesh(stage, "/W/colored")
    UsdGeom.PrimvarsAPI(colored).CreatePrimvar(
        "displayColor", Sdf.ValueTypeNames.Color3fArray,
        UsdGeom.Tokens.constant).Set([Gf.Vec3f(.2, .3, .4)])
    report = material_audit.audit(stage)
    assert report["ok"]
    assert report["counts"]["bound_targets"] == 1
    assert report["counts"]["display_color_only"] == 1


def test_dangling_typeless_and_surface_less_bindings_fail():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/W")
    dangling = _mesh(stage, "/W/dangling")
    dangling.GetPrim().CreateRelationship(
        "material:binding").SetTargets(["/W/Looks/gone"])
    typeless = _mesh(stage, "/W/typeless")
    stage.DefinePrim("/W/Looks/placeholder")
    typeless.GetPrim().CreateRelationship(
        "material:binding").SetTargets(["/W/Looks/placeholder"])
    no_surface = _mesh(stage, "/W/no_surface")
    UsdShade.MaterialBindingAPI.Apply(no_surface.GetPrim()).Bind(
        _material(stage, "/W/Looks/no_surface", surface=False))
    report = material_audit.audit(stage)
    assert not report["ok"]
    assert report["counts"]["dangling_targets"] == 1
    assert report["counts"]["typeless_targets"] == 1
    assert report["counts"]["surface_less_materials"] == 1


def test_unassigned_subset_faces_require_a_mesh_fallback():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/W")
    mesh = UsdGeom.Mesh.Define(stage, "/W/mesh")
    mesh.CreatePointsAttr([Gf.Vec3f(0, 0, 0), Gf.Vec3f(1, 0, 0),
                           Gf.Vec3f(0, 1, 0), Gf.Vec3f(1, 1, 0)])
    mesh.CreateFaceVertexCountsAttr([3, 3])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 1, 3, 2])
    subset = UsdGeom.Subset.Define(stage, "/W/mesh/face0")
    subset.CreateElementTypeAttr(UsdGeom.Tokens.face)
    subset.CreateFamilyNameAttr("materialBind")
    subset.CreateIndicesAttr([0])
    UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(_material(stage))
    report = material_audit.audit(stage)
    assert not report["ok"]
    assert report["counts"]["unassigned_faces"] == 1
    assert report["counts"]["unbound_uncolored"] == 1
