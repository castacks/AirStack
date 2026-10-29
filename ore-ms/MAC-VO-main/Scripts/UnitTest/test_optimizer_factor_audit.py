"""Analytic fixtures for saved-factor diagnostics, independent forward differences."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from Scripts.Setup.audit_optimizer_factors import camera_points, residual_jacobian, summarize


def fixture():
    rng = np.random.default_rng(41)
    p = rng.uniform([-2, -2, -2], [2, 2, 2], (20, 3)); p[:, 0] += 7
    k = np.array([[410., 0, 320], [0, 390, 240], [0, 0, 1]])
    uv = p[:, 1:]/p[:, :1]*[410, 390]+[320, 240]
    disp = 410*.22/p[:, 0]
    return p, uv, disp, k


def test_exact_residual():
    p, uv, disp, k = fixture()
    r, j = residual_jacobian(p, uv, disp, k, .22)
    np.testing.assert_allclose(r, 0, atol=1e-13)
    assert j.shape == (20, 3, 6)


def test_nonzero_residual():
    p, uv, disp, k = fixture()
    r, _ = residual_jacobian(p, uv+[1, -2], disp+3, k, .22)
    np.testing.assert_allclose(r, np.tile([-1, 2, -3], (20, 1)), atol=1e-13)


def test_right_tangent_finite_difference():
    p, uv, disp, k = fixture()
    _, j = residual_jacobian(p, uv, disp, k, .22)
    for axis in range(6):
        d = np.eye(6)[axis]*1e-6
        def forward(sign):
            q = (p-sign*d[:3]) @ Rotation.from_rotvec(sign*d[3:]).as_matrix()
            return np.column_stack((q[:, 1:]/q[:, :1]*[410, 390]+[320, 240], 410*.22/q[:, 0]))
        np.testing.assert_allclose((forward(1)-forward(-1))/2e-6, j[:, :, axis], rtol=2e-6, atol=1e-7)


def test_correlated_weighting():
    p, uv, disp, k = fixture()
    r, j = residual_jacobian(p, uv+1, disp+2, k, .22)
    c = np.tile([[2., .3, 0], [.3, 4, 0], [0, 0, 5]], (20, 1, 1))
    result = summarize(r, j, c)
    inv = np.linalg.inv(c)
    np.testing.assert_allclose(result['information_right_camera_m_rad'], np.einsum('nki,nkl,nlj->ij', j, inv, j))
    np.testing.assert_allclose(result['whitened_sum_squares'], np.einsum('ni,nij,nj->', r, inv, r))
    assert result['numerical_rank'] == 6 and result['pose_covariance'] is None


def test_covariance_scaling():
    p, uv, disp, k = fixture(); r, j = residual_jacobian(p, uv+1, disp, k, .22)
    c = np.tile(np.eye(3), (20, 1, 1))
    a, b = summarize(r, j, c), summarize(r, j, c*4)
    np.testing.assert_allclose(b['information_right_camera_m_rad'], np.array(a['information_right_camera_m_rad'])/4)
    assert b['whitened_sum_squares'] == a['whitened_sum_squares']/4


def test_rank_deficient():
    p, uv, disp, k = fixture(); p[:] = p[0]
    r, j = residual_jacobian(p, uv, disp, k, .22)
    a = summarize(r, j, np.tile(np.eye(3), (20, 1, 1)))
    assert a['numerical_rank'] == 3 and a['mixed_units_condition_number'] is None


@pytest.mark.parametrize('bad', [0., -1., float('nan')])
def test_invalid_forward_depth(bad):
    p, uv, disp, k = fixture(); p[0, 0] = bad
    with pytest.raises(ValueError): residual_jacobian(p, uv, disp, k, .22)


@pytest.mark.parametrize('bad', [0., -1., float('nan')])
def test_invalid_covariance(bad):
    p, uv, disp, k = fixture(); r, j = residual_jacobian(p, uv, disp, k, .22)
    c = np.tile(np.eye(3), (20, 1, 1)); c[0, 0, 0] = bad
    with pytest.raises(ValueError): summarize(r, j, c)


def test_invalid_pose():
    with pytest.raises(ValueError): camera_points([[1, 0, 0]], [0]*7)


def test_rotated_pose():
    r = Rotation.from_rotvec([.1, .3, -.2]); t = np.array([2, -1, .5])
    p, _, _, _ = fixture()
    np.testing.assert_allclose(camera_points(r.apply(p)+t, np.r_[t, r.as_quat()]), p, atol=1e-14)


def test_asymmetric_covariance():
    p, uv, disp, k = fixture(); r, j = residual_jacobian(p, uv, disp, k, .22)
    c = np.tile(np.eye(3), (20, 1, 1)); c[0, 1, 0] = .1
    with pytest.raises(ValueError): summarize(r, j, c)
