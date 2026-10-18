import numpy as np
from Scripts.Setup.td3_reprojection import pair_geometry,project_ned


def fixture():
    k=np.array([[200.,0,160.],[0,200.,120.],[0,0,1.]])
    p0=np.array([[5.,.5,.2],[8.,-.5,.4],[10.,1.,-.5]])
    relative=np.eye(4);relative[:3,3]=[-.4,.1,0]
    p1=p0+relative[:3,3]
    return k,p0,p1,relative,project_ned(p0,k),project_ned(p1,k)


def test_exact_rigid_geometry():
    k,p0,p1,t,u0,u1=fixture()
    r=pair_geometry(u0,u1,p0[:,0],p1[:,0],k,t)
    np.testing.assert_allclose(r['forward_error_px'],0,atol=1e-10)
    np.testing.assert_allclose(r['reverse_error_px'],0,atol=1e-10)
    np.testing.assert_allclose(r['sampson_px'],0,atol=1e-10)


def test_wrong_depth_does_not_change_epipolar_residual():
    k,p0,p1,t,u0,u1=fixture()
    r=pair_geometry(u0,u1,2*p0[:,0],2*p1[:,0],k,t)
    assert np.linalg.norm(r['forward_error_px'],axis=1).min()>.1
    np.testing.assert_allclose(r['sampson_px'],0,atol=1e-10)
    u1[:,1]+=3
    perturbed=pair_geometry(u0,u1,p0[:,0],p1[:,0],k,t)
    assert perturbed['sampson_px'].min()>.1


def test_behind_camera_and_zero_translation_are_invalid():
    k,p0,p1,t,u0,u1=fixture()
    assert np.isnan(project_ned(np.array([[-1.,0,0],[0.,0,0]]),k)).all()
    r=pair_geometry(u0,u0,p0[:,0],p0[:,0],k,np.eye(4))
    assert np.isnan(r['sampson_px']).all()


def test_rotation_basis_and_inverse_direction():
    from scipy.spatial.transform import Rotation
    k,p0,_,t,_,_=fixture()
    t[:3,:3]=Rotation.from_euler('z',8,degrees=True).as_matrix()
    p1=p0@t[:3,:3].T+t[:3,3]
    u0,u1=project_ned(p0,k),project_ned(p1,k)
    r=pair_geometry(u0,u1,p0[:,0],p1[:,0],k,t)
    np.testing.assert_allclose(r['forward_error_px'],0,atol=1e-10)
    np.testing.assert_allclose(r['sampson_px'],0,atol=1e-10)
    backward=pair_geometry(u1,u0,p1[:,0],p0[:,0],k,np.linalg.inv(t))
    np.testing.assert_allclose(backward['forward_error_px'],0,atol=1e-10)


def test_offline_residual_matches_active_backend_and_3d_covariance_is_unused():
    import torch
    import pypose as pp
    from Module.Map import MatchObs,PointNode
    from Module.Optimization.TwoFramePGO.Graphs import GraphInput,Analytic_ReprojDisp_TwoFramePGO
    k,p0,p1,t,u0,u1=fixture()
    u1=u1+np.array([.5,-.2])
    def tensor(a):return torch.as_tensor(a,dtype=torch.float32)
    obs=MatchObs.init({'pixel2_uv':tensor(u1),
        'pixel2_uv_cov':tensor(np.tile([.2,.3,0],(3,1))),
        'pixel2_disp':tensor((k[0,0]*.3/p1[:,0])[:,None]),
        'pixel2_disp_cov':torch.ones(3,1)*.7,
        'obs2_covTc':torch.eye(3).repeat(3,1,1)})
    points=PointNode.init({'pos_Tw':tensor(p0),'cov_Tw':torch.eye(3).repeat(3,1,1)})
    graph=Analytic_ReprojDisp_TwoFramePGO(GraphInput(
        frame_idx=torch.tensor([1]),from_idx=torch.tensor([0]),
        init_motion=pp.mat2SE3(tensor(np.linalg.inv(t)))[None],baseline=torch.tensor(.3),
        observations=obs,points=points,images_intrinsic=tensor(k),
        edges_index=torch.zeros(3,dtype=torch.long),device='cpu'))
    offline=pair_geometry(u0,u1,p0[:,0],p1[:,0],k,t)
    expected=np.column_stack([-offline['forward_error_px'],np.zeros(3)])
    np.testing.assert_allclose(graph().detach().numpy(),expected,atol=2e-5)
    before=graph.covariance_array().clone()
    obs.data['obs2_covTc'].mul_(1000)
    points.data['cov_Tw'].mul_(1000)
    torch.testing.assert_close(graph.covariance_array(),before,rtol=0,atol=0)
    np.testing.assert_allclose(before[0].numpy(),np.diag([.2,.3,.7]),atol=1e-7)
