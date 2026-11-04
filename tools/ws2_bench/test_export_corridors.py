"""Portable USD export checks; optional dependencies are not needed by the bench."""
import hashlib
import json
from pathlib import Path
import shutil

import pytest

pytest.importorskip('pxr.Usd', reason='Scene export requires the optional usd-core package')
pytest.importorskip('numpy')
from pxr import Sdf, Usd, UsdGeom, UsdPhysics
from export_corridors import export


def test_export_matches_suite_and_survives_relocation(tmp_path):
    from corridor_suite import scenario
    from corridors import geometry

    original = tmp_path / 'original'
    manifest = export(original)
    moved = tmp_path / 'moved'
    shutil.move(original, moved)
    assert len(manifest['scenes']) == 12
    for item in manifest['scenes']:
        actual = json.loads((moved / item['geometry']).read_text())
        config = scenario('pulp_dronet', item['shape'], item['difficulty'], 'clean')
        expected = geometry(config['condition']['corridor'], config['condition']['layout_seed'])
        assert actual == expected
        stage = Usd.Stage.Open(str(moved / item['usd']))
        assert UsdGeom.GetStageUpAxis(stage) == 'Z'
        assert UsdGeom.GetStageMetersPerUnit(stage) == 1.0
        assert str(stage.GetDefaultPrim().GetPath()) == '/World'
        meshes, assets, lights = [], [], []
        for prim in stage.Traverse():
            assert not prim.HasAPI(UsdPhysics.RigidBodyAPI)
            if prim.IsA(UsdGeom.Mesh):
                assert prim.HasAPI(UsdPhysics.CollisionAPI)
                meshes.append(prim)
            if prim.GetTypeName().endswith('Light'):
                lights.append(prim)
                assert prim.GetAttribute('inputs:intensity').Get() == 5400.0
            for attr in prim.GetAttributes():
                value = attr.Get()
                if isinstance(value, Sdf.AssetPath) and value.path:
                    assert not Path(value.path).is_absolute()
                    assert value.resolvedPath
                    assert Path(value.resolvedPath).is_relative_to(moved)
                    assets.append(value)
        assert len(meshes) == len(expected['walls']) + len(expected['obstacles']) + 2
        assert len(assets) == 4 and lights
        other = Usd.Stage.CreateInMemory()
        other.DefinePrim('/World/Environment').GetReferences().AddReference(str(moved / item['usd']))
        assert other.GetPrimAtPath('/World/Environment/Corridor/Floor').IsA(UsdGeom.Mesh)
    for path, digest in manifest['files_sha256'].items():
        assert hashlib.sha256((moved / path).read_bytes()).hexdigest() == digest


def test_export_refuses_to_overwrite_existing_assets(tmp_path):
    output = tmp_path / 'user_scenes'
    output.mkdir()
    sentinel = output / 'keep.txt'
    sentinel.write_text('existing user asset')
    with pytest.raises(FileExistsError):
        export(output)
    assert sentinel.read_text() == 'existing user asset'
