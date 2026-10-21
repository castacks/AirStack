import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from Scripts.Setup.td3_calibration_refine import centered_pose, signed_residual, fit_rotation, gate


def test_center_constraint_and_synthetic_rotation_fit():
    rng=np.random.default_rng(4);xyz=rng.uniform([-2,-1,3],[2,1,20],(150,3));a=xyz/xyz[:,2:3]
    center=np.array([.335,.001,-.002]);rot=Rotation.from_euler('xyz',[.3,-.8,.15],degrees=True).as_rotvec()
    t=centered_pose(rot,center);target=xyz@t[:3,:3].T+t[:3,3];b=target/target[:,2:3]
    np.testing.assert_allclose(-t[:3,:3].T@t[:3,3],center,atol=1e-14)
    cfg={'calib':{name:{'intrinsics':[400,400,320,256],'distortion_coeffs':[0,0,0,0]} for name in ['cam0','cam1']}}
    fitted,report=fit_rotation(a,b,center,cfg)
    assert report['success'] and not report['bound_saturated']
    np.testing.assert_allclose(fitted,t,atol=1e-8)
    k=np.array([[400,0,320],[0,400,256],[0,0,1.]])
    assert np.max(np.abs(signed_residual(rot,center,a,b,k,k)))<1e-10
    with pytest.raises(ValueError):fit_rotation(a[:2],b[:2],center,cfg)


def test_fresh_gate_retains_failures_and_rejects_bound_solution():
    def row(i):return {'case':str(i),'candidates':{'body_centers_rotation_fit':{'parallax_eligible_count':30,'positive_eligible_fraction':1.,'sampson_px':{'p10_p50_p90':[.1,.2,.3]}},'producer_inverse':{'sampson_px':{'p10_p50_p90':[1,1,1]}},'kalibr_as_stored':{'sampson_px':{'p10_p50_p90':[1,1,1]}}}}
    rows=[row(i) for i in range(12)];stability={'rotation_deg':.1};fits=[{'bound_saturated':False} for i in range(3)]
    assert gate(rows,stability,fits)['passed']
    assert not gate(rows[:11],stability,fits)['passed']
    fits[0]['bound_saturated']=True
    assert not gate(rows,stability,fits)['passed']
    fits[0]['bound_saturated']=False;rows[0]['candidates']['body_centers_rotation_fit']['parallax_eligible_count']=0
    assert not gate(rows,stability,fits)['passed']
