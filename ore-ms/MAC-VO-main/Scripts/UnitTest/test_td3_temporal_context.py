import numpy as np
import pytest
from Scripts.Setup.td3_temporal_context import partial_rank, temporal_checks, pair_summary


def test_nuisance_only_is_undefined_after_adjustment():
    x=np.arange(19.);y=x+np.sin(x)*.1
    result,residuals,valid=partial_rank(x,y,x[:,None])
    assert result['status']=='zero_rank_residual' and result['rho'] is None
    assert result['design_rank']==2 and valid.sum()==19
    np.testing.assert_allclose(residuals,0,atol=1e-11)


def test_remaining_signal_and_rank_deficiency():
    rng=np.random.default_rng(17);x=rng.permutation(19).astype(float)
    c=np.column_stack([np.arange(19.),np.arange(19.),np.ones(19)])
    result,_,_=partial_rank(x,x,c)
    assert result['rho']==pytest.approx(1.)
    assert result['design_rank']==2 and result['residual_dimension']==17


def test_missing_rows_small_samples_and_constants():
    x=np.arange(10.);y=x.copy();c=np.zeros((10,1));x[0]=np.nan;c[2]=np.nan
    result,r,valid=partial_rank(x,y,c)
    assert result['n']==8 and result['rho']==pytest.approx(1.) and not valid[0] and not valid[2]
    assert np.isnan(r[[0,2]]).all()
    assert partial_rank(x[:3],y[:3],c[:3])[0]['status']=='insufficient_samples'
    assert partial_rank(np.ones(10),y,np.zeros((10,0)))[0]['status']=='zero_rank_residual'
    assert partial_rank(x* np.nan,y,c)[0]['n']==0


def test_first_differences_do_not_bridge_missing_or_gaps():
    x=np.arange(19.)**2;y=x.copy();x[5]=np.nan
    result=temporal_checks(x,y,np.arange(19))
    assert result['difference_n']==16 and result['difference_rho']==pytest.approx(1.)
    assert result['first9_n']==8 and result['last10_n']==10
    index=np.array([0,1,4,5,6]); result=temporal_checks(index**2,index**2,index)
    assert result['difference_n']==3


def test_empty_inverse_depth_and_inbound_summary():
    data=dict(trace=np.arange(3.),uv0=np.zeros((3,2)),uv1=np.ones((3,2)),depth0=np.array([0.,np.nan,-1.]),
              forward_px=np.arange(3.),sampson_px=np.arange(3.))
    result=pair_summary(data,np.ones(3,bool))
    assert result['inverse_depth_m'] is None and result['inverse_depth_m_finite_n']==0
    assert result['flow_px']==pytest.approx(np.sqrt(2))
    result=pair_summary(data,np.zeros(3,bool))
    assert result['points']==0 and result['trace'] is None


def test_shapes_and_chronology():
    with pytest.raises(ValueError):partial_rank(np.ones(3),np.ones(4),np.ones((3,1)))
    with pytest.raises(ValueError):temporal_checks(np.ones(3),np.ones(3),np.array([0,0,1]))
