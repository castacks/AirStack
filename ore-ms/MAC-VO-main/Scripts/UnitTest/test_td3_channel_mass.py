import numpy as np
import pytest
import torch

from Scripts.Setup.td3_channel_mass import apply_weights
from Scripts.Setup.td3_precision_mass import apply_weights as previous_weights, nonpixel_fingerprint
from Scripts.Setup.td3_backend_ablation import fingerprint, solve
from Scripts.Setup.td3_reference import matrices
from Scripts.UnitTest.test_td3_backend import synthetic_input


@pytest.mark.parametrize('arm,channel', [('fixed_u_mass', 0), ('fixed_v_mass', 1)])
def test_single_channel_uniformity_conservation_and_untouched_channel(arm, channel):
    data, _ = synthetic_input()
    uv = data.observations.data['pixel2_uv_cov']
    uv[:, 0] = torch.linspace(.1, 4, len(uv))
    uv[:, 1] = torch.linspace(2, .2, len(uv))
    before = uv.clone(); signature = nonpixel_fingerprint(data)
    report = apply_weights(data, arm, [.4, .6, 2])
    assert report['uniform_channels'] == [channel]
    assert report['unselected_channels_bitwise_unchanged']
    assert uv.dtype == before.dtype and torch.equal(uv[:, 2], before[:, 2])
    assert torch.equal(uv[:, 1-channel], before[:, 1-channel])
    assert torch.equal(uv[:, channel], uv[0, channel].expand(len(uv)))
    expected = len(uv)/before[:, channel].double().reciprocal().sum()
    torch.testing.assert_close(uv[0, channel].double(), expected, rtol=1e-6, atol=0)
    torch.testing.assert_close(uv[:, :2].double().reciprocal().sum(dim=0),
                               before[:, :2].double().reciprocal().sum(dim=0), rtol=1e-6, atol=0)
    assert nonpixel_fingerprint(data) == signature


@pytest.mark.parametrize('arm', ['learned', 'fixed_uv_mass'])
def test_controls_match_predecessor_exactly(arm):
    data, _ = synthetic_input(); old, _ = synthetic_input()
    for item in [data, old]:
        item.observations.data['pixel2_uv_cov'][:, 0] = torch.linspace(.1, 4, 40)
    apply_weights(data, arm, [.4, .6, 2]); previous_weights(old, arm, [.4, .6, 2])
    assert fingerprint(data) == fingerprint(old)


@pytest.mark.parametrize('arm', ['fixed_u_mass', 'fixed_v_mass'])
def test_one_channel_recovers_consistent_motion_without_input_mutation(arm):
    data, expected = synthetic_input()
    uv = data.observations.data['pixel2_uv_cov']
    uv[:, 0] = torch.linspace(.1, 4, len(uv)); uv[:, 1] = torch.linspace(2, .2, len(uv))
    apply_weights(data, arm, [.4, .6, 2]); before = fingerprint(data)
    pose, _ = solve(data, 'disp')
    np.testing.assert_allclose(matrices(pose[None])[0], expected, atol=1e-3)
    assert fingerprint(data) == before


@pytest.mark.parametrize('bad', [0., -1., np.nan, np.inf])
def test_invalid_variance_in_unselected_channel_is_rejected(bad):
    data, _ = synthetic_input()
    data.observations.data['pixel2_uv_cov'][0, 1] = bad
    with pytest.raises(ValueError):
        apply_weights(data, 'fixed_u_mass', [.4, .6, 2])
