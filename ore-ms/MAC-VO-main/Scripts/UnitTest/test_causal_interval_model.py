"""Independent exponential controls and explicit motion-model limitations."""
import copy
import math
import numpy as np
import pytest
from scipy.linalg import expm, logm

from Scripts.Setup.causal_interval_model import project, scale_motion


def algebra(w, v):
    # Independent direct 4x4 Lie algebra, not production Jacobian code.
    x,y,z = w
    out = np.zeros((4,4));out[:3,:3]=[[0,-z,y],[z,0,-x],[-y,x,0]];out[:3,3]=v
    return out


@pytest.mark.parametrize('w',[[0,0,0],[1e-10,-2e-10,3e-10],[.2,-.3,.4],[0,0,math.pi-1e-9]])
@pytest.mark.parametrize('ratio',[.5,1.,1.5,2.])
def test_full_screw_against_independent_matrix_exponential(w,ratio):
    xi = algebra(w,[.3,-.7,.2]); native = expm(xi)
    np.testing.assert_allclose(scale_motion(native,ratio),expm(xi*ratio),atol=3e-10,rtol=3e-10)


def test_half_turn_about_offset_axis_is_not_linear_translation_scaling():
    # Rotate about point(1,0): a half turn translates(2,0), a quarter(1,-1).
    native = [[-1.,0.,0.,2.],[0.,-1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]]
    quarter = scale_motion(native,.5)
    np.testing.assert_allclose(quarter[:3,3],[1,-1,0],atol=1e-12)
    np.testing.assert_allclose(quarter@quarter,native,atol=1e-12)


def test_bounded_screws_match_independent_logm_expm():
    rng=np.random.default_rng(88)
    for _ in range(25):
        native=expm(algebra(rng.uniform(-.5,.5,3),rng.uniform(-1,1,3)))
        np.testing.assert_allclose(scale_motion(native,1.3),expm(logm(native)*1.3).real,atol=1e-10)


def test_epoch_integer_shift_and_stationary_twist_exactness():
    t=1761615290010015011; native=expm(algebra([0,0,.3],[1,.1,0]))
    before=native.copy()
    out=project(native,[t,t+100],[t+20,t+170])
    np.testing.assert_allclose(out['delta_model'],expm(algebra([0,0,.3],[1,.1,0])*1.5),atol=1e-12)
    assert out['duration_ratio']==1.5 and out['max_outside_native_ns']==70
    assert not out['admission_ready'] and not out['reference_used_for_prediction']
    np.testing.assert_array_equal(native,before)


def test_acceleration_is_a_counterexample_even_with_perfect_native_motion():
    # x(t)=t^2: native[0,1] displacement1; target[1,2] displacement3.
    native=np.eye(4);native[0,3]=1
    out=project(native,[0,100],[100,200])
    assert out['valid_model_projection'] and out['delta_model'][0][3]==1
    assert abs(out['delta_model'][0][3]-3)==2


def test_horizon_is_fixed_and_rejection_has_no_motion():
    assert project(np.eye(4),[0,100],[100,200])['valid_model_projection']
    out=project(np.eye(4),[0,100],[101,201])
    assert not out['valid_model_projection'] and out['delta_model'] is None


@pytest.mark.parametrize('case',['reflection','nonfinite','scale','bottom','bool'])
def test_bad_geometry_rejected(case):
    matrix=np.eye(4).tolist()
    if case=='reflection':matrix[0][0]=-1
    elif case=='nonfinite':matrix[0][3]=float('nan')
    elif case=='scale':matrix[0][0]=2
    elif case=='bool':matrix[0][0]=True
    else:matrix[3][0]=1
    with pytest.raises(ValueError):scale_motion(matrix,1.)


@pytest.mark.parametrize('ratio',[True,0,-1,float('inf')])
def test_bad_ratio_rejected(ratio):
    with pytest.raises(ValueError):scale_motion(np.eye(4),ratio)


@pytest.mark.parametrize('times',[[True,100],[0.,100],[100,100]])
def test_bad_timestamps_rejected(times):
    with pytest.raises(ValueError):project(np.eye(4),times,[0,100])
