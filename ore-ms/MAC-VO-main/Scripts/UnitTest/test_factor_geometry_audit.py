"""Known covariance/projector fixtures for objective geometry accounting."""
import numpy as np
import pytest
from Scripts.Setup.factor_geometry_audit import decompose


def fixture():
    n = 9; rng = np.random.default_rng(10)
    depth = np.array([1., 5, 20, 80, 4, 19, 79, 100, 10])
    pixels = np.array([[x, y] for y in (15, 45, 75) for x in (15, 45, 75)])
    r = rng.normal(size=(n, 3)); j = rng.normal(size=(n, 3, 6)); c = np.tile(np.eye(3), (n, 1, 1))
    return depth, pixels, [90, 90], r, j, c


def test_fixed_boundaries_and_cells():
    result = decompose(*fixture())
    assert [v['count'] for v in result['depth_bins'].values()] == [2, 3, 2, 2]
    assert [v['count'] for v in result['target_3x3_cells'].values()] == [1]*9
    assert result['top1pct_factor_budget'] == 1


def test_energy_and_information_partitions():
    args = fixture(); result = decompose(*args)
    for groups in ('depth_bins', 'target_3x3_cells'):
        for name, total in result['totals'].items():
            np.testing.assert_allclose(sum(v['sums'][name] for v in result[groups].values()), total)
            np.testing.assert_allclose(sum(v['shares'][name] for v in result[groups].values()), 1.)
    np.testing.assert_allclose(result['totals']['translation_information'], (args[4][:, :, :3]**2).sum())
    np.testing.assert_allclose(result['totals']['rotation_information'], (args[4][:, :, 3:]**2).sum())


def test_projector_via_qr():
    args = fixture(); result = decompose(*args); q, _ = np.linalg.qr(args[4].reshape(-1, 6))
    leverage = (q*q).sum(axis=1).reshape(9, 3).sum(1)
    assert result['numerical_rank'] == 6
    np.testing.assert_allclose(result['totals']['leverage'], 6.)
    np.testing.assert_allclose(result['descriptors']['top1pct_leverage_share'], max(leverage)/6.)


def test_rank_deficient_projector():
    args = list(fixture()); args[4][:] = 0; args[4][:, :, :3] = np.eye(3)
    result = decompose(*args)
    assert result['numerical_rank'] == 3
    np.testing.assert_allclose(result['totals']['leverage'], 3.)
    np.testing.assert_allclose(result['descriptors']['top1pct_leverage_share'], 1/9)


def test_zero_residual_null_shares():
    args = list(fixture()); args[3][:] = 0; result = decompose(*args)
    assert result['descriptors']['top1pct_energy_share'] is None
    assert result['totals']['uv_energy'] == result['totals']['disparity_energy'] == 0
    assert all(v['shares']['uv_energy'] is None for v in result['depth_bins'].values())


def test_zero_jacobian():
    args = list(fixture()); args[4][:] = 0; result = decompose(*args)
    assert result['numerical_rank'] == 0 and result['descriptors']['top1pct_leverage_share'] is None


def test_correlated_uv_energy():
    args = list(fixture()); args[5][:, :2, :2] = [[2, .3], [.3, 4]]
    result = decompose(*args)
    expected = np.einsum('ni,nij,nj->', args[3][:, :2], np.linalg.inv(args[5][:, :2, :2]), args[3][:, :2])
    np.testing.assert_allclose(result['totals']['uv_energy'], expected)


def test_covariance_scaling():
    args = list(fixture()); first = decompose(*args); args[5] *= 4; second = decompose(*args)
    for key in first['totals']:
        np.testing.assert_allclose(second['totals'][key], first['totals'][key] if key == 'leverage' else first['totals'][key]/4)
    np.testing.assert_allclose(second['descriptors']['top1pct_leverage_share'], first['descriptors']['top1pct_leverage_share'])


@pytest.mark.parametrize('bad', [0, -1, float('nan')])
def test_invalid_depth(bad):
    args = list(fixture()); args[0][0] = bad
    with pytest.raises(ValueError): decompose(*args)


@pytest.mark.parametrize('bad', [0, -1, float('nan')])
def test_invalid_covariance(bad):
    args = list(fixture()); args[5][0, 0, 0] = bad
    with pytest.raises(ValueError): decompose(*args)


def test_unsupported_cross_covariance():
    args = list(fixture()); args[5][0, 0, 2] = args[5][0, 2, 0] = .1
    with pytest.raises(ValueError): decompose(*args)


def test_out_of_bounds():
    args = list(fixture()); args[1][0, 0] = 90
    with pytest.raises(ValueError): decompose(*args)


def test_tail_budget_rounds_up():
    args = list(fixture()); args = [np.concatenate([a]*12) if isinstance(a, np.ndarray) else a for a in args]
    assert decompose(*args)['top1pct_factor_budget'] == 2
