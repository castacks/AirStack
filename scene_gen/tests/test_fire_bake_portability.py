#!/usr/bin/env python3
"""Pure path-policy tests for reusable fire bake portability."""

import os
import sys


sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from disaster import fire_bake as fb  # noqa: E402


def test_shared_asset_path_moves_to_nucleus():
    got = fb.portable_shared_asset_path(
        "/isaac-sim/AirStack/scene_gen/assets/materials/megascans/Brick/a.png")
    assert got == (fb.SHARED_ASSET_MIRROR
                   + "materials/megascans/Brick/a.png")


def test_run_specific_bake_texture_remains_for_final_collect():
    path = "/isaac-sim/.cache/fire_bakes/city_3/textures/soot.png"
    assert fb.portable_shared_asset_path(path) == path


def test_sliced_kit_uses_versioned_nucleus_cache():
    name = "SM_Building_26__66d5f99cf2663d5a.usd"
    got = fb.portable_shared_asset_path(
        "/isaac-sim/AirStack/scene_gen/assets/kits/" + name)
    assert got == fb.SHARED_KIT_MIRROR + name


def test_legacy_migrated_kit_path_is_corrected_without_rebake():
    name = "SM_Building_26__66d5f99cf2663d5a.usd"
    got = fb.portable_shared_asset_path(fb.SHARED_KIT_LEGACY_MIRROR + name)
    assert got == fb.SHARED_KIT_MIRROR + name


def test_malformed_omniverse_url_is_normalized():
    bad = "omniverse:/airlab-nucleus.andrew.cmu.edu:443/Projects/x.usd"
    assert fb.portable_shared_asset_path(bad) == (
        "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/x.usd")
