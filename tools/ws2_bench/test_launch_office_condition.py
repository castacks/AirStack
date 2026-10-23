"""Execute the real condition callback with USD boundaries stubbed, CPU only."""
import ast
import copy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from conditions import validate
from corridors import geometry, patch_corners


def failed_pilot_condition():
    # Exact clean condition from the first matched pilot's pre-takeoff failure;
    # no runtime artifact dependency, so this regression survives cleanup.
    return dict(name='clean', layout='corridor', layout_seed=594706693, seed=73001,
        light=2200., rgb_noise=0., depth_noise=0., delay=0., patch_enabled=False, patch_size=.6,
        corridor=dict(shape='elbow', width=4., obstacles=2, style='hallway_daylight',
            route_scale=2., difficulty='medium', layout_version=5, bend_radius=4.,
            obstacle_placements=[
                dict(route_s=12.444473644101606, lateral_m=-1.30995856933836,
                     width_m=.8015548037082492, depth_m=.5559535878949424,
                     height_m=2.2573743770001706, yaw_deg=-56.390183405111095),
                dict(route_s=16.48338216856416, lateral_m=-1.34559599400204,
                     width_m=.48593461539866745, depth_m=.6155498738879395,
                     height_m=.7449715941442153, yaw_deg=25.186966984630928)]),
        noise_schedule=dict(start_s=194.92920711898194, duration_s=0., period_s=0.),
        delay_schedule=dict(start_s=0., duration_s=0., period_s=0.))


def callback(raw):
    c = validate(raw)
    data = geometry(c['corridor'], c['layout_seed'])
    scene = SimpleNamespace(data=data, patch_calls=[], light=None)
    quad = SimpleNamespace(points=None, visible=None)
    def corners(size, mount):
        scene.patch_calls.append((size, copy.deepcopy(mount)))
        return patch_corners(data, size, mount)
    scene.patch_corners = corners
    scene.illumination = lambda value: setattr(scene, 'light', value)
    quad.GetPointsAttr = lambda: SimpleNamespace(Set=lambda value: setattr(quad, 'points', value))
    imageable = SimpleNamespace(MakeVisible=lambda: setattr(quad, 'visible', True),
                                MakeInvisible=lambda: setattr(quad, 'visible', False))
    light = SimpleNamespace(GetIntensityAttr=lambda: SimpleNamespace(Set=lambda value: None))
    path = Path(__file__).with_name('launch_office.py')
    node = next(n for n in ast.parse(path.read_text()).body
                if isinstance(n, ast.FunctionDef) and n.name == 'apply_condition')
    namespace = dict(validate=validate, CORRIDOR=True, corridor_scene=scene, dome=light, fill=light,
                     quad=quad, UsdGeom=SimpleNamespace(Imageable=lambda value: imageable),
                     json=json, condition=None)
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace['apply_condition'], scene, quad, namespace


def test_actual_disabled_pulp_patch_does_not_validate_nonexistent_surface():
    raw = failed_pilot_condition(); apply, scene, quad, namespace = callback(raw)
    # This unused default placement caused the original simulator startup error.
    with pytest.raises(ValueError, match='fit completely'):
        patch_corners(scene.data, raw['patch_size'])
    apply(raw)
    assert scene.patch_calls == [] and quad.points is None and quad.visible is False
    assert namespace['condition'] == validate(raw) and scene.light == raw['light']


def test_enabled_invalid_patch_still_rejects_identical_nonfitting_surface():
    raw = failed_pilot_condition(); raw['patch_enabled'] = True
    apply, scene, quad, namespace = callback(raw)
    with pytest.raises(ValueError, match='fit completely'): apply(raw)
    assert scene.patch_calls == [(.6, None)] and namespace['condition'] is None


@pytest.mark.parametrize('enabled,explicit', [(True, False), (True, True), (False, True)])
def test_valid_enabled_and_explicit_clean_mounts_keep_identical_geometry(enabled, explicit):
    raw = failed_pilot_condition(); raw.update(patch_enabled=enabled, patch_size=.3)
    if explicit: raw['patch_mount'] = dict(obstacle_index=0, face='front', height_m=1.2)
    apply, scene, quad, namespace = callback(raw)
    expected = patch_corners(scene.data, .3, raw.get('patch_mount'))
    apply(raw)
    assert quad.points == expected and quad.visible is enabled
    assert scene.patch_calls == [(.3, raw.get('patch_mount'))]
    assert namespace['condition'] == validate(raw)


def test_invalid_explicit_mount_is_not_ignored_even_for_clean():
    raw = failed_pilot_condition()
    raw['patch_mount'] = dict(obstacle_index=1, face='front', height_m=1.2)
    with pytest.raises(ValueError, match='fit completely'): callback(raw)
