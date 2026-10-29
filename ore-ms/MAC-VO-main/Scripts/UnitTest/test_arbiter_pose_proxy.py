"""Nominal SE3 conjugation and offline bridge controls; no physical GT assumptions."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from Scripts.Setup.arbiter_pose_proxy import body_delta, transport, losses, se3, at, summarize


def pose(t=(0,0,0),rot=(0,0,0)):
    p=np.eye(4);p[:3,3]=t;p[:3,:3]=Rotation.from_rotvec(rot).as_matrix();return p


def test_perfect_body_prediction_with_lever_arm():
    e=pose((1,0,0));d=pose(rot=(0,0,np.pi/2))
    camera=np.linalg.inv(e)@d@e
    assert np.allclose(camera[:3,3],[-1,1,0])
    recovered=body_delta(camera,e)
    np.testing.assert_allclose(recovered,d,atol=1e-14)
    assert losses(recovered,d)['translation_m']<1e-14


def test_rotated_camera_axes():
    e=pose(rot=(0,0,np.pi/2));d=pose((1,0,0))
    np.testing.assert_allclose(body_delta(d,e)[:3,3],[0,1,0],atol=1e-14)


def test_asynchronous_perfect_motion_and_world_anchor_invariance():
    r0=pose((1,2,3),(.3,.2,-.1));a=pose((2,1,3),(.1,-.4,.2))
    b=pose((4,2,1),(-.2,.3,.4));r1=pose((4,3,2),(.4,.2,.1))
    d=np.linalg.inv(a)@b;expected=np.linalg.inv(r0)@r1
    np.testing.assert_allclose(transport(d,r0,a,b,r1),expected,atol=1e-14)
    # A non-perfect predictor also remains invariant under a world-frame change.
    d=d@pose((.1,.2,-.1),(.1,.1,.2));g=pose((30,-20,10),(.7,-.3,.2))
    np.testing.assert_allclose(transport(d,g@r0,g@a,g@b,g@r1),transport(d,r0,a,b,r1),atol=1e-13)


def test_noncommuting_bridge_order_and_no_bridge_identity():
    r0=pose();a=pose((1,0,0),(0,0,.4));b=pose((2,1,0),(.2,0,0));r1=pose((3,1,2),(0,.3,0));d=pose((.2,.4,.5),(.1,.2,.3))
    actual=transport(d,r0,a,b,r1)
    reverse=a@np.linalg.inv(r0)@d@r1@np.linalg.inv(b)
    assert not np.allclose(actual,reverse)
    np.testing.assert_allclose(transport(d,a,a,b,b),d,atol=1e-14)


def test_loss_units_and_no_mixed_score():
    l=losses(pose((3,4,0),(0,0,np.pi/3)),pose())
    assert l['translation_m']==5 and l['rotation_deg']==pytest.approx(60)
    assert set(l)=={'translation_m','rotation_deg'}


@pytest.mark.parametrize('fault',['reflection','shape','nan','last_row','nonorthogonal'])
def test_bad_se3_rejected(fault):
    p=pose()
    if fault=='reflection':p[0,0]=-1
    elif fault=='shape':p=p[:3]
    elif fault=='nan':p[0,3]=np.nan
    elif fault=='last_row':p[3,0]=1
    else:p[0,0]=2
    with pytest.raises(ValueError):se3(p)


def test_query_does_not_round_large_timestamps_or_allow_float():
    class Reference:
        def at(self,t):return t
    value=1761615290010015011
    assert at(Reference(),[value])[0]==value
    with pytest.raises(ValueError):at(Reference(),[float(value)])


def test_empty_population_never_gets_zero_loss():
    row={'window':'test','action':'ABSTAIN','experts':{'rgb':None,'thermal':None}}
    s=summarize([row])['test']
    assert s['common_eligible_count']==0
    assert s['policies']['rgb_only']['translation_m']['mean'] is None
    assert s['policies']['advisory']['available']==0 and s['policies']['advisory']['abstain']==1


def test_serialization_roundoff_projection_is_small_and_explicit():
    from Scripts.Setup.arbiter_pose_proxy import rectification_rotation
    r=pose(rot=(.1,.2,.3))[:3,:3];raw=r.copy();raw[0,0]+=2e-8
    projected=rectification_rotation(raw)
    np.testing.assert_allclose(projected.T@projected,np.eye(3),atol=1e-14)
    assert np.max(np.abs(projected-raw))<3e-8
    with pytest.raises(ValueError):rectification_rotation(np.diag([1,1,1.001]))
    with pytest.raises(ValueError):rectification_rotation(np.diag([1,1,-1]))
