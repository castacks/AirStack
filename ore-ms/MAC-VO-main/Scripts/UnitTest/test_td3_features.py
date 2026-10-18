import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from Scripts.Setup.td3_feature_consistency import (
    unproject_ned, consistency, rank_association, depth_scale_fit, fit_trajectory_scale,
)


def test_rigid_residual_and_rotated_covariance():
    relative = np.eye(4)[None]
    relative[0,:3,:3] = Rotation.from_euler('z', 90, degrees=True).as_matrix()
    relative[0,:3,3] = [.3,.2,0]
    p0 = np.array([[1.,2,3]])
    p1 = np.einsum('nij,nj->ni',relative[:,:3,:3],p0) + relative[:,:3,3]
    c0 = np.diag([1.,2,3])[None]; c1 = np.eye(3)[None]
    result = consistency(p0,p1,c0,c1,relative)
    np.testing.assert_allclose(result['residual'],0,atol=1e-12)
    np.testing.assert_allclose(result['covariance'][0],np.diag([3.,2,4]),atol=1e-12)
    assert result['valid'].tolist() == [True]


def test_normalization_and_singular_rejection():
    p0=np.zeros((1,3));p1=np.array([[2.,0,0]]);relative=np.eye(4)[None]
    result=consistency(p0,p1,np.eye(3)[None],np.eye(3)[None],relative)
    assert result['mahalanobis_squared_approx'][0] == pytest.approx(2.)
    singular=consistency(p0,p1,np.zeros((1,3,3)),np.zeros((1,3,3)),relative)
    assert not singular['valid'][0]
    assert np.isnan(singular['mahalanobis_squared_approx'][0])


def test_known_depth_scale_and_projection_basis():
    p0=np.array([[4.,1,1],[8.,2,2]])
    relative=np.repeat(np.eye(4)[None],2,axis=0);relative[:,:3,3]=[1.,0,0]
    p1=p0+2*relative[:,:3,3]  # both endpoint model depths scaled by 2
    fit=depth_scale_fit(p0,p1,relative,np.array([True,True]))
    assert fit['unconstrained_scale'] == pytest.approx(.5)
    assert fit['fitted_residual_rms_m'] == pytest.approx(0)
    k=np.array([[[100.,0,50.],[0,100.,40.],[0,0,1]]])
    point=unproject_ned(np.array([[60.,60.]]),[10.],k)
    np.testing.assert_allclose(point,[[10.,1.,2.]])


def test_rank_constant_and_in_sample_trajectory_scale():
    assert rank_association([1,2,3,4],[2,4,6,8]) == pytest.approx(1)
    assert rank_association([1,1,1],[1,2,3]) is None
    assert rank_association([1,2,3,4],[2,4,6,8],control=[1,2,3,4]) is None
    reference=np.repeat(np.eye(4)[None],3,axis=0);reference[:,0,3]=[0,1,2]
    estimated=reference.copy();estimated[:,0,3]*=2
    result=fit_trajectory_scale(estimated,reference)
    assert result['fitted_scale'] == pytest.approx(.5)
    assert result['fitted_translation_rmse_m'] == pytest.approx(0)


def test_scale_fit_uses_only_first_half_for_sequential_diagnostic():
    reference=np.repeat(np.eye(4)[None],4,axis=0);reference[:,0,3]=[0,1,2,3]
    estimated=reference.copy();estimated[:,0,3]=[0,2,6,8]
    result=fit_trajectory_scale(estimated,reference)['sequential_split_diagnostic']
    assert result['first_half_fit_scale'] == pytest.approx(.5)
    # Held-out reference [2,3], scaled estimate [3,4]; no test-half fitting.
    assert result['second_half_scaled_translation_rmse_m'] == pytest.approx(1.)
