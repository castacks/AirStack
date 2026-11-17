"""Authored passive fixtures; expected dispositions are plans, not rendered truth."""
import json
from pathlib import Path

SCENARIOS = ('nominal', 'absent', 'duplicate-red', 'split-occlusion',
             'partial-occlusion', 'full-occlusion')
REFERENCE_CAMERA_M = (.2, 0., .02)


def fixture_spec(scenario='nominal'):
    """Return fresh metadata; importing this module starts no simulator or robot."""
    if not isinstance(scenario, str) or scenario not in SCENARIOS:
        raise ValueError('unknown UAV marker fixture scenario')
    red = dict(id='red_panel', position=[4., 0., 1.2], color=[1., .02, .02],
               size_m=[.6, .6, .6])
    blue = dict(id='blue_distractor', position=[4., -1.5, 1.2], color=[.02, .1, 1.],
                size_m=[.6, .6, .6])
    fixtures = [red, blue]
    expected = 'OBSERVED'
    if scenario == 'absent':
        fixtures = [blue]
        expected = 'ABSENT'
    elif scenario == 'duplicate-red':
        blue.update(id='red_competitor', color=[1., .02, .02])
        expected = 'AMBIGUOUS'
    elif scenario.endswith('occlusion'):
        size = {'split-occlusion': [.1, .8, .12],
                'partial-occlusion': [.1, .8, .3],
                'full-occlusion': [.1, 1., 1.]}[scenario]
        # Aim at target front-face height from the declared reference camera.
        height = 1.4 if scenario == 'partial-occlusion' else 1.2
        ratio = (3.4-REFERENCE_CAMERA_M[0])/(3.7-REFERENCE_CAMERA_M[0])
        z = REFERENCE_CAMERA_M[2]+ratio*(height-REFERENCE_CAMERA_M[2])
        fixtures.append(dict(id='gray_occluder', position=[3.4, 0., z],
                             color=[.05, .05, .05], size_m=size))
        expected = {'split-occlusion': 'AMBIGUOUS', 'partial-occlusion': 'OBSERVED',
                    'full-occlusion': 'ABSENT'}[scenario]
    return dict(schema_version='uav-marker-fixture/v2', scenario=scenario,
                frame='world', units='metres', static_colliders=True, fixtures=fixtures,
                reference_camera_m=list(REFERENCE_CAMERA_M),
                intended_detector_status=expected, rendered_qualified=False,
                expectation_scope='authored geometry only; actual camera/renderer unverified')


def write_manifest(path, manifest):
    """Retain a new manifest; never overwrite an earlier case's evidence."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf-8') as stream:
        stream.write(json.dumps(manifest, indent=2, allow_nan=False)+'\n')


def author_fixtures(stage, spec):
    """Author into the caller's USD stage; never obtain a global live context."""
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade
    for item in spec['fixtures']:
        path = '/World/UAVFixtures/'+item['id']
        cube = UsdGeom.Cube.Define(stage, path)
        size = item['size_m']
        uniform = size[0] == size[1] == size[2]
        cube.CreateSizeAttr(size[0] if uniform else 1.)
        cube.AddTranslateOp().Set(Gf.Vec3d(*item['position']))
        if not uniform:
            cube.AddScaleOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*size))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*item['color'])])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        material = UsdShade.Material.Define(stage, path+'Material')
        shader = UsdShade.Shader.Define(stage, path+'Shader')
        shader.CreateIdAttr('UsdPreviewSurface')
        for name in ('diffuseColor', 'emissiveColor'):
            shader.CreateInput(name, Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*item['color']))
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material)
