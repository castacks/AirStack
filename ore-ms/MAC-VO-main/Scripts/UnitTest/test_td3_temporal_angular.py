import numpy as np
from Scripts.Setup.td3_temporal_angular import rays, ray_jacobian, angular_variance, angle

K = np.array([[400., 0, 320], [0, 500, 256], [0, 0, 1]])

def test_center_and_exact_angle():
    uv = np.array([[320., 256.]])
    result, valid = angular_variance(uv, np.array([[4., 9., 1.]]), K)
    np.testing.assert_allclose(result, 4/400**2+9/500**2)
    assert valid.all()
    np.testing.assert_allclose(angle(uv, uv+[4, 0], K), np.arctan(.01))

def test_finite_difference_and_cross_covariance():
    uv = np.array([[40., 70.], [500., 450.]])
    j = ray_jacobian(uv, K)
    for axis in range(2):
        delta = np.eye(2)[axis]*1e-3
        np.testing.assert_allclose(j[:, :, axis], (rays(uv+delta, K)-rays(uv-delta, K))/.002, atol=2e-12, rtol=1e-8)
    c = np.array([[4., 9., 3.], [1., 4., -1.]])
    actual, valid = angular_variance(uv, c, K)
    expected = [np.trace(jj@np.array([[cc[0], cc[2]], [cc[2], cc[1]]])@jj.T) for jj, cc in zip(j, c)]
    np.testing.assert_allclose(actual, expected); assert valid.all()

def test_resolution_scaling_invariance():
    uv = np.array([[40., 70.], [500., 450.]])
    c = np.array([[4., 9., 3.], [1., 4., -1.]])
    scaled = K.copy(); scaled[:2] *= 2
    np.testing.assert_allclose(angular_variance(uv, c, K)[0], angular_variance(uv*2, c*4, scaled)[0])
    np.testing.assert_allclose(angle(uv, uv+[2, 3], K), angle(uv*2, (uv+[2, 3])*2, scaled))

def test_invalid_covariance_preserved_as_missing():
    c = np.array([[1., 1., 2.], [-1., 1., 0.], [np.nan, 1., 0.], [0., 0., 0.]])
    result, valid = angular_variance(np.tile([320., 256.], (4, 1)), c, K)
    assert valid.tolist() == [False, False, False, True]
    assert np.isnan(result[:3]).all() and result[3] == 0
