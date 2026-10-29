import copy
import numpy as np
import pytest
from Scripts.Setup.arbiter_quality_features import covariance_features,make_features
from Scripts.Setup.analyze_arbiter_quality import preference,rank_association
from Scripts.UnitTest.test_pair_arbiter_events import fixture


def grid():
    return {'grid_uv1':np.array([[0.,0.]]),'grid_raw_flow_cov':np.array([[4.,16.,0.]]),'K':np.diag([2.,4.,1.])}


def test_analytic_center_and_resolution_invariance():
    raw=grid();r=covariance_features(raw)
    assert r['flow_trace_px2']==20 and r['ray_trace_rad2']==pytest.approx(2)
    scaled=copy.deepcopy(raw);scaled['grid_uv1']*=3;scaled['grid_raw_flow_cov']*=9;scaled['K']=np.diag([3,3,1])@scaled['K']
    rr=covariance_features(scaled)
    assert rr['flow_trace_px2']==180 and rr['ray_trace_rad2']==pytest.approx(r['ray_trace_rad2'])


def test_cross_term_retained_off_axis():
    raw=grid();raw['grid_uv1'][:]=[1,2];raw['grid_raw_flow_cov'][:]=[4,16,2]
    a=covariance_features(raw)['ray_trace_rad2'];raw['grid_raw_flow_cov'][0,2]=-2
    assert a!=pytest.approx(covariance_features(raw)['ray_trace_rad2'])


@pytest.mark.parametrize('fault',['nonpsd','nan_endpoint','zero_diagonal','empty'])
def test_invalid_grid_not_replaced_by_zero_uncertainty(fault):
    raw=grid()
    if fault=='nonpsd':raw['grid_raw_flow_cov'][0,2]=100
    elif fault=='nan_endpoint':raw['grid_uv1'][0,0]=np.nan
    elif fault=='zero_diagonal':raw['grid_raw_flow_cov'][0,0]=0
    else:raw['grid_uv1']=np.empty((0,2));raw['grid_raw_flow_cov']=np.empty((0,3))
    result=covariance_features(raw)
    assert result['flow_trace_px2'] is None and result['ray_trace_rad2'] is None and result['valid_covariance_grid_count']==0


def features_input():
    reports,events=fixture();rows=reports['rgb']['input_rows']
    for c in events:
        for e in events[c]:
            if e['kind']=='pair_end':e['occupied_3x3_cells']=1
    raw={'time0_ns':np.array(rows[0]['rgb_left']['time_ns'],np.int64),
         'grid_uv1':np.zeros((20,2)),'grid_raw_flow_cov':np.tile([1.,2.,0.],(20,1)),'K':np.eye(3),
         'selected_uv0':np.zeros((12,2)),'selected_inbound':np.array([True]*11+[False]),
         'outlier_keep_inbound':np.array([True]*10+[False]),'height_width':np.array([100,100])}
    return rows,events['rgb'],raw


def test_current_prior_only_and_unknown_cold_start():
    rows,events,raw=features_input();before=make_features(rows,0,'rgb',events,raw)
    assert before['values']['prior_supported'] is None and before['values']['retained_fraction']==pytest.approx(10/12)
    events[4]['original_need_interp']=True
    assert before==make_features(rows,0,'rgb',events,raw)
    raw['time0_ns']=np.array(rows[1]['rgb_left']['time_ns'],np.int64)
    assert make_features(rows,1,'rgb',events,raw)['values']['prior_supported']==1


@pytest.mark.parametrize('fault',['wrong_time','float_time','count','coverage','duplicate_health'])
def test_bad_snapshot_contract_rejected(fault):
    rows,events,raw=features_input()
    if fault=='wrong_time':raw['time0_ns']+=1
    elif fault=='float_time':raw['time0_ns']=np.array(float(raw['time0_ns']))
    elif fault=='count':events[1]['retained_count']=9
    elif fault=='coverage':events[1]['occupied_3x3_cells']=2
    else:events.append(copy.deepcopy(events[1]))
    with pytest.raises(ValueError):make_features(rows,0,'rgb',events,raw)


def test_preference_is_fixed_and_missing_is_not_confidence():
    assert preference(1,2,'lower')==('rgb','decisive')
    assert preference(1,2,'higher')==('thermal','decisive')
    assert preference(1,1,'higher')==('rgb','tie_rgb_preference')
    assert preference(None,2,'lower')==(None,'missing')


def test_constant_and_small_rank_populations_are_undefined():
    assert rank_association([1,1,1],[1,2,3])['reason']=='constant_ranks'
    assert rank_association([1,2],[2,1])['rho'] is None
    assert rank_association([1,2,3],[3,2,1])['rho']==pytest.approx(-1)
