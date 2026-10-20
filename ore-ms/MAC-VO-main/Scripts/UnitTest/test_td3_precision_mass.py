import numpy as np
import pytest
import torch

from Scripts.Setup.td3_backend_ablation import fingerprint, solve
from Scripts.Setup.td3_precision_mass import apply_weights, nonpixel_fingerprint
from Scripts.Setup.td3_reference import matrices
from Scripts.UnitTest.test_td3_backend import synthetic_input


def test_mass_preserves_channels_and_all_nonpixel_fields():
    data, _ = synthetic_input()
    uv = data.observations.data['pixel2_uv_cov']
    uv[:, 0] = torch.linspace(.1, 4, len(uv))
    uv[:, 1] = torch.linspace(2, .2, len(uv))
    original = uv.clone()
    before = nonpixel_fingerprint(data)
    report = apply_weights(data, 'fixed_uv_mass', [.4, .6, 2])
    assert nonpixel_fingerprint(data) == before
    assert uv.dtype == original.dtype
    assert (uv[:, 2] == original[:, 2]).all()
    torch.testing.assert_close(uv[:, :2].double().reciprocal().sum(dim=0),
                               original[:, :2].double().reciprocal().sum(dim=0), rtol=1e-6, atol=0)
    torch.testing.assert_close(uv[:, :2], uv[0, :2].expand(len(uv), 2))
    expected = len(uv)/original[:, :2].double().reciprocal().sum(dim=0)
    torch.testing.assert_close(uv[0, :2].double(), expected, rtol=1e-6, atol=0)
    assert report['max_precision_relative_change'] < 1e-6


def test_uniform_source_is_unchanged_and_controls_use_existing_rules():
    data, _ = synthetic_input()
    original = fingerprint(data)
    apply_weights(data, 'fixed_uv_mass', [.4, .6, 2])
    assert fingerprint(data) == original
    report = apply_weights(data, 'learned', [.4, .6, 2])
    np.testing.assert_allclose(report['precision_ratio_u_v'], [1, 1])
    assert fingerprint(data) == original
    apply_weights(data, 'fixed_uv', [.4, .6, 2])
    torch.testing.assert_close(data.observations.data['pixel2_uv_cov'][:, :2],
                               torch.tensor([.4, .6]).expand(40, 2))
    assert (data.observations.data['pixel2_disp_cov'] == .5).all()


@pytest.mark.parametrize('bad', [0., -1., np.nan, np.inf])
def test_invalid_pixel_variance_rejected(bad):
    data, _ = synthetic_input()
    data.observations.data['pixel2_uv_cov'][0, 0] = bad
    with pytest.raises(ValueError):
        apply_weights(data, 'fixed_uv_mass', [.4, .6, 2])


def test_nonzero_cross_covariance_and_invalid_disparity_rejected():
    data, _ = synthetic_input()
    data.observations.data['pixel2_uv_cov'][0, 2] = .1
    with pytest.raises(ValueError):
        apply_weights(data, 'fixed_uv_mass', [.4, .6, 2])
    data.observations.data['pixel2_uv_cov'][0, 2] = 0
    data.observations.data['pixel2_disp_cov'][0, 0] = 0
    with pytest.raises(ValueError):
        apply_weights(data, 'fixed_uv_mass', [.4, .6, 2])


def test_mass_preserving_weights_recover_consistent_motion_without_mutation():
    data, expected = synthetic_input()
    uv = data.observations.data['pixel2_uv_cov']
    uv[:, 0] = torch.linspace(.1, 4, len(uv))
    apply_weights(data, 'fixed_uv_mass', [.4, .6, 2])
    before = fingerprint(data)
    pose, _ = solve(data, 'disp')
    np.testing.assert_allclose(matrices(pose[None])[0], expected, atol=1e-3)
    assert fingerprint(data) == before
