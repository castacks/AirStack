import copy
import math

import unittest

try:
    import pytest
except ImportError as error:
    raise unittest.SkipTest("optional UAV pytest suite: " + str(error))

from rrm.uav_viewpoint import optical_forward_in_map, select_viewpoint


def observations(z=3.5):
    return [dict(status='OBSERVED', bbox_xywh=[220, 175, 32, 30],
                 surface_point_optical_m=[0., .7, z], surface_depth_z_m=z)
            for _ in range(3)]


def test_nominal_resolution_seeking_motion():
    result = select_viewpoint(observations(), [480, 300], [1., 0., 0.])
    assert result['status'] == 'SELECTED'
    assert .6 < result['delta_map_m'][0] < .7
    assert result['predicted_bbox_area_ratio'] == pytest.approx(1.5)
    assert result['predicted_surface_depth_z_m'] >= 2.5


def test_camera_forward_defines_map_direction():
    result = select_viewpoint(observations(), [480, 300], [0., -1., 0.])
    assert result['delta_map_m'][0] == 0
    assert result['delta_map_m'][1] < -.6


def test_large_range_is_clipped_to_motion_budget():
    result = select_viewpoint(observations(6), [480, 300], [1., 0., 0.])
    assert result['translation_m'] == .75


@pytest.mark.parametrize('status', ['ABSENT', 'AMBIGUOUS', 'DEPTH_UNKNOWN', 'INVALID'])
def test_unknown_target_abstains(status):
    values = observations()
    values[1]['status'] = status
    assert select_viewpoint(values, [480, 300], [1., 0., 0.])['status'] == 'ABSTAIN'


@pytest.mark.parametrize('change', ['cropped', 'nan', 'unstable_depth', 'unstable_area', 'off_center'])
def test_bad_observation_abstains(change):
    values = copy.deepcopy(observations())
    if change == 'cropped':
        values[0]['bbox_xywh'][0] = 0
    if change == 'nan':
        values[0]['surface_depth_z_m'] = math.nan
    if change == 'unstable_depth':
        values[0]['surface_depth_z_m'] = 3.7
        values[0]['surface_point_optical_m'][2] = 3.7
    if change == 'unstable_area':
        values[0]['bbox_xywh'][2] = 60
    if change == 'off_center':
        values[0]['surface_point_optical_m'][0] = 3
    assert select_viewpoint(values, [480, 300], [1., 0., 0.])['status'] == 'ABSTAIN'


@pytest.mark.parametrize('axis', [[0., 0., 1.], [2., 0., 0.], [math.nan, 0., 0.]])
def test_unqualified_axis_abstains(axis):
    assert select_viewpoint(observations(), [480, 300], axis)['status'] == 'ABSTAIN'


@pytest.mark.parametrize('z', [2.4, 2.55, 2.6])
def test_standoff_and_gain_limit_abstains(z):
    assert select_viewpoint(observations(z), [480, 300], [1., 0., 0.])['status'] == 'ABSTAIN'


def test_optical_forward_rotation():
    assert optical_forward_in_map([0., math.sqrt(.5), 0., math.sqrt(.5)]) == pytest.approx([1., 0., 0.])


@pytest.mark.parametrize('q', [[0., 0., 0., 0.], [math.nan, 0., 0., 1.]])
def test_invalid_quaternion_rejected(q):
    with pytest.raises(ValueError):
        optical_forward_in_map(q)
