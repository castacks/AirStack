"""Synthetic geometry/contract tests, not evidence of real-data improvement."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import bearing_rotation as br

K = np.array([[500., 0, 320], [0, 480., 128], [0, 0, 1]])


def scene(seed=0, translation=True):
    rng = np.random.default_rng(seed)
    points = rng.uniform([-3, -2, 5], [3, 2, 20], (64, 3))
    rotation = Rotation.from_rotvec([.012, -.02, .008]).as_matrix()
    t = np.array([.3, .05, .1]) if translation else np.zeros(3)
    other = (points-t)@rotation
    uv = lambda p: (p@K.T)[:, :2]/p[:, 2:]
    a, b = uv(points), uv(other)
    covariance = np.broadcast_to([[.7, .2], [.2, 1.3]], (len(a), 2, 2)).copy()
    f = br.bearings(a, K)[0]
    g, _, c = br.bearings(b, K, covariance)
    return f, g, c, rotation, t, b, covariance


def test_bearing_jacobian_and_covariance():
    _, g, c, _, _, uv, covariance = scene()
    _, jac, _ = br.bearings(uv, K)
    numeric = []
    for d in np.eye(2):
        numeric.append((br.bearings(uv+1e-3*d, K)[0]-br.bearings(uv-1e-3*d, K)[0])/.002)
    np.testing.assert_allclose(jac, np.stack(numeric, axis=-1), atol=1e-12, rtol=1e-7)
    np.testing.assert_allclose(c, jac@covariance@jac.swapaxes(1, 2), atol=1e-15)
    np.testing.assert_allclose(np.einsum('ni,nij->nj', g, c), 0, atol=1e-20)


def test_scalar_formula_and_sign_invariance():
    f, g, c, r, t, *_ = scene()
    t /= np.linalg.norm(t)
    r = Rotation.from_rotvec([.004, -.003, .002]).as_matrix()@r
    values = br.residuals(f, g, r, t, c)
    for i in range(len(f)):
        skew = np.array([[0, -f[i, 2], f[i, 1]], [f[i, 2], 0, -f[i, 0]], [-f[i, 1], f[i, 0], 0]])
        variance = t@skew@r@c[i]@r.T@skew.T@t
        expected = (t@np.cross(f[i], r@g[i]))/np.sqrt(variance+br.VARIANCE_REGULARIZER)
        np.testing.assert_allclose(values[i], expected, atol=1e-12)
    np.testing.assert_allclose(br.residuals(f, g, r, -t, c), -values, atol=1e-12)


@pytest.mark.parametrize('seed', [0, 1, 2])
@pytest.mark.parametrize('method', ['nec', 'anisotropic', 'isotropic'])
def test_exact_scene_solve(seed, method):
    f, g, c, r, t, uv, _ = scene(seed)
    if method == 'isotropic':
        c = br.bearings(uv, K, np.broadcast_to(np.eye(2), (len(uv), 2, 2)))[2]
    result = br.solve(f, g, None if method == 'nec' else c)
    assert result['status'] == 'CONVERGED'
    assert Rotation.from_matrix(result['rotation'].T@r).magnitude() < 1e-6
    assert abs(result['translation_direction']@(t/np.linalg.norm(t))) > 1-1e-8
    assert result['final_cost'] <= result['initial_cost']+1e-20


def test_pure_rotation_is_not_a_metric_pose():
    f, g, c, *_ = scene(translation=False)
    assert br.solve(f, g, c)['status'].startswith('DEGENERATE')


def test_near_epipole_regularizer():
    t = np.array([0., 0, 1])
    f = np.array([[0., 0, 1], [1e-12, 0, 1]])
    f /= np.linalg.norm(f, axis=1)[:, None]
    g = f.copy(); g[:, 0] += 1e-6; g /= np.linalg.norm(g, axis=1)[:, None]
    c = np.broadcast_to(np.diag([1e-6, 1e-6, 0]), (2, 3, 3))
    assert np.isfinite(br.residuals(f, g, np.eye(3), t, c)).all()


def test_isotropic_pixel_variance_is_not_constant_nec_weight():
    f, g, _, r, t, uv, _ = scene()
    c = br.bearings(uv, K, np.broadcast_to(np.eye(2), (len(uv), 2, 2)))[2]
    t /= np.linalg.norm(t)
    r = Rotation.from_rotvec([.01, 0, 0]).as_matrix()@r
    ratio = br.residuals(f, g, r, t, c)/br.residuals(f, g, r, t)
    assert np.ptp(ratio) > 1


def test_rejects_bad_covariance():
    with pytest.raises(ValueError, match='SPD'):
        br.bearings(np.array([[2., 3]]), K, np.array([[[1., 2], [2, 1]]]))
    with pytest.raises(ValueError, match='finite'):
        br.bearings(np.array([[float('nan'), 3.]]), K)


def test_covariance_scale_and_regularizer_scale():
    f, g, c, r, t, *_ = scene()
    t /= np.linalg.norm(t)
    r = Rotation.from_rotvec([.01, 0, 0]).as_matrix()@r
    a = br.residuals(f, g, r, t, c)
    b = br.residuals(f, g, r, t, 4*c, 4*br.VARIANCE_REGULARIZER)
    np.testing.assert_allclose(a, 2*b, atol=1e-12)


def test_budget_is_reported():
    f, g, c, *_ = scene()
    rng = np.random.default_rng(4)
    g += rng.normal(0, 1e-4, g.shape); g /= np.linalg.norm(g, axis=1)[:, None]
    assert br.solve(f, g, c, max_evaluations=1)['status'] == 'BUDGET_EXHAUSTED'
