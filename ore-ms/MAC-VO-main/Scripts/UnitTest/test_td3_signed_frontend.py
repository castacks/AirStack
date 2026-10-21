import numpy as np
from Scripts.Setup.td3_signed_frontend import signed_metrics, depth_reproduction


def test_signed_convention_and_missing_correspondences():
    observed=np.array([[4.,1.],[-3.,-.5],[.1,0],[9.,0]])
    model=np.array([[-4.,-1.],[3.,.5],[.1,0],[np.nan,0]])
    result=signed_metrics(observed,model,np.ones(4,bool))
    assert result['count']==3 and result['signed_population_count']==2
    assert result['sign_agreement']==1
    assert result['horizontal_error_px']['p10_p50_p90'][1]==0
    assert signed_metrics(observed,-model,np.ones(4,bool))['sign_agreement']==0
    result=signed_metrics(np.empty((0,2)),np.empty((0,2)),np.empty(0,bool))
    assert result['sign_agreement'] is None


def test_control_rejects_mask_change_and_excess_error():
    saved=np.array([1.,10.,np.nan])
    assert depth_reproduction(saved,saved.copy())['pass_control']
    assert depth_reproduction(saved,np.array([1.,10.0001,np.nan]))['pass_control']
    assert not depth_reproduction(saved,np.array([1.01,10.,np.nan]))['pass_control']
    assert not depth_reproduction(saved,np.array([1.,10.,1.]))['pass_control']
    assert not depth_reproduction(np.array([np.nan]),np.array([np.nan]))['pass_control']
