import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from Scripts.Setup.td3_calibration_reconcile import (
    raw_rays, rectified_pixels, implied_transform, sampson_pixels, triangulation,
    fit_pose, pose_difference, support_gate,
)


def synthetic():
    rng=np.random.default_rng(42)
    xyz=rng.uniform([-2,-1,3],[2,1,20],(120,3))
    t=np.eye(4);t[:3,:3]=Rotation.from_euler('xyz',[.2,-.7,.1],degrees=True).as_matrix();t[:3,3]=[-.33,.001,.002]
    target=xyz@t[:3,:3].T+t[:3,3]
    a=xyz/xyz[:,2:3];b=target/target[:,2:3]
    k=np.array([[400.,0,320],[0,400,256],[0,0,1]])
    cfg={'calib':{'cam0':{'intrinsics':[400,400,320,256]},'cam1':{'intrinsics':[400,400,320,256],'T_cn_cnm1':t.tolist()}}}
    for name in ['cam0','cam1']:
        cfg['calib'][name].update(distortion_coeffs=[0,0,0,0],resolution=[640,512])
    return a,b,t,k,cfg


def test_rectification_ray_roundtrip_and_implied_direction():
    a,_,t,k,_=synthetic();r=Rotation.from_euler('xyz',[1,2,-1],degrees=True).as_matrix()
    p=np.column_stack([k,np.zeros(3)])
    uv=rectified_pixels(a,r,p)
    np.testing.assert_allclose(raw_rays(uv,r,p),a,atol=1e-14)
    left={'r':r,'p':p};right_r=Rotation.from_euler('y',.4,degrees=True).as_matrix();q=p.copy();q[0,3]=-132
    implied=implied_transform(left,{'r':right_r,'p':q})
    np.testing.assert_allclose(implied[:3,:3],right_r.T@r,atol=1e-14)
    np.testing.assert_allclose(implied[:3,3],right_r.T@[-.33,0,0],atol=1e-14)


def test_epipolar_residual_and_scale_sign_invariance():
    a,b,t,k,_=synthetic()
    assert sampson_pixels(a,b,t,k,k).max()<1e-10
    noisy=b.copy();noisy[:,1]+=.01
    error=sampson_pixels(a,noisy,t,k,k)
    assert np.median(error)>1
    other=t.copy();other[:3,3]*=-3
    np.testing.assert_allclose(sampson_pixels(a,noisy,other,k,k),error,atol=1e-12)


def test_triangulation_and_translation_sign_cheirality():
    a,b,t,k,_=synthetic();result=triangulation(a,b,t,k,k)
    assert result['finite'].all() and result['positive'].all() and result['eligible'].all()
    assert result['reprojection_px'].max()<1e-10
    other=t.copy();other[:3,3]*=-1
    result2=triangulation(a,b,other,k,k)
    assert not result2['positive'].any()
    np.testing.assert_allclose(result['xyz'],-result2['xyz'],atol=1e-10)
    identical=triangulation(a,a,np.diag([1.,1.,1.,1.])+np.array([[0,0,0,-.33],[0,0,0,0],[0,0,0,0],[0,0,0,0]]),k,k)
    assert not identical['eligible'].any()
    assert triangulation(a[:0],b[:0],t,k,k)['xyz'].shape==(0,3)


def test_calibrated_pose_fit_recovers_known_pose():
    a,b,t,_,cfg=synthetic();estimated,masks=fit_pose(a,b,cfg)
    diff=pose_difference(estimated,t)
    assert diff['rotation_deg']<1e-4 and diff['translation_direction_deg']<1e-3
    assert masks['ransac_mask'].all()
    with pytest.raises(ValueError):fit_pose(a[:3],b[:3],cfg)


def test_image_gate_requires_every_holdout_and_stable_pose():
    def row(i):
        candidate={'parallax_eligible_count':30,'positive_eligible_fraction':.98,'sampson_px':{'p10_p50_p90':[.1,.2,.3]}}
        return {'case':str(i),'candidates':{'pooled_image_candidate':candidate,'producer_inverse':{'sampson_px':{'p10_p50_p90':[1,1,1]}},'kalibr_as_stored':{'sampson_px':{'p10_p50_p90':[1,1,1]}}}}
    rows=[row(i) for i in range(10)];stable={'rotation_deg':.1,'translation_direction_deg':1}
    assert support_gate(rows,stable)['passed']
    assert not support_gate(rows[:9],stable)['passed']
    assert not support_gate(rows,None)['passed']
    rows[0]['candidates']['pooled_image_candidate']['positive_eligible_fraction']=.94
    assert not support_gate(rows,stable)['passed']
