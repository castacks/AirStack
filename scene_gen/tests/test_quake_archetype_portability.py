#!/usr/bin/env python3
"""Portable-path gate for reusable earthquake archetypes."""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(_HERE, "..")))

from disaster import bake  # noqa: E402
from pxr import Sdf, Usd, UsdGeom, UsdShade  # noqa: E402


def test_known_archetype_sources_map_to_their_canonical_nucleus_tree():
    assert bake.portable_archetype_asset_path(
        "/isaac-sim/AirStack/scene_gen/assets/rubble_hd/a.usdc"
    ) == bake._ASSET_MIRROR + "rubble_hd/a.usdc"
    assert bake.portable_archetype_asset_path(
        "/isaac-sim/AirStack/scene_gen/assets/kits/a.usdc"
    ) == bake._KIT_MIRROR + "a.usdc"
    assert bake.portable_archetype_asset_path(
        "./Soil_Mud/T_pjuph20_1K_B.jpg"
    ) == (bake._ASSET_MIRROR +
          "materials/megascans/Soil_Mud/T_pjuph20_1K_B.jpg")
    assert bake.portable_archetype_asset_path(
        "./Brick_Wall_Worn/T_sexkaitb_1K_B.jpg"
    ) == (bake._ASSET_MIRROR +
          "materials/megascans/Brick_Wall_Worn/T_sexkaitb_1K_B.jpg")
    assert bake.portable_archetype_asset_path(
        "Textures/Bed_01_BaseColor.png"
    ) == bake._MODULAR_TEXTURE_MIRROR + "Bed_01_BaseColor.png"
    assert bake.portable_archetype_asset_path(
        "Textures/Game_ModernCityEnvironment01_Materials_M_MBuilding01_"
        "Facades_BaseColor.png"
    ) == (bake._MCE_TEXTURE_MIRROR +
          "Game_ModernCityEnvironment01_Materials_M_MBuilding01_"
          "Facades_BaseColor.png")
    assert bake.portable_archetype_asset_path(
        "/isaac-sim/kit/mdl/core/Base/OmniPBR.mdl"
    ) == "OmniPBR.mdl"


def test_unknown_relative_path_is_not_guessed():
    assert bake.portable_archetype_asset_path("unknown/a.png") == "unknown/a.png"


def test_layer_wide_normalizer_reaches_shader_assets_and_references():
    stage = Usd.Stage.CreateInMemory()
    root = stage.DefinePrim("/Baked", "Xform")
    child = stage.DefinePrim("/Baked/ref", "Xform")
    child.GetReferences().AddReference(
        "/isaac-sim/AirStack/scene_gen/assets/rubble_hd/chunk.usdc")
    mat = UsdShade.Material.Define(stage, "/Baked/Looks/soil")
    shader = UsdShade.Shader.Define(stage, "/Baked/Looks/soil/Shader")
    shader.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath("./Soil_Mud/T_pjuph20_1K_B.jpg"))

    assert bake.normalize_archetype_asset_paths(
        stage.GetRootLayer(), verbose=False) == 2
    assert shader.GetInput("diffuse_texture").Get().path.startswith(
        bake._ASSET_MIRROR)
    refs = root.GetStage().GetPrimAtPath("/Baked/ref").GetMetadata("references")
    assert refs.prependedItems[0].assetPath == (
        bake._ASSET_MIRROR + "rubble_hd/chunk.usdc")


def test_binding_api_repair_is_root_layer_only_and_idempotent():
    stage = Usd.Stage.CreateInMemory()
    mesh = UsdGeom.Mesh.Define(stage, "/Baked/mesh").GetPrim()
    mat = UsdShade.Material.Define(stage, "/Baked/Looks/wall")
    # Deliberately reproduce the old exporter shape: a valid relationship,
    # but no applied MaterialBindingAPI schema.
    UsdShade.MaterialBindingAPI(mesh).Bind(mat)
    assert not mesh.HasAPI(UsdShade.MaterialBindingAPI)

    assert bake.apply_material_binding_api(stage, verbose=False) == 1
    assert mesh.HasAPI(UsdShade.MaterialBindingAPI)
    assert bake.apply_material_binding_api(stage, verbose=False) == 0
