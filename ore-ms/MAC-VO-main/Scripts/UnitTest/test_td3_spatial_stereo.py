import numpy as np
from Scripts.Setup.td3_spatial_stereo import grid, patch_metrics, groups, summarize, measure


def test_known_two_dimensional_translation_and_wrong_target():
    rng=np.random.default_rng(0);left=rng.uniform(0,255,(80,96));right=np.roll(left,(-2,-3),(0,1))
    uv=np.array([[30.,30.],[50.,40.]])
    correct=patch_metrics(left,right,uv,np.tile([-3.,-2.],(2,1)))
    np.testing.assert_allclose(correct['ncc'],1,atol=1e-12)
    assert correct['ncc_valid'].all()
    wrong=patch_metrics(left,right,uv,np.zeros((2,2)))
    assert np.max(np.abs(wrong['ncc']))<.3


def test_constant_and_outside_patches_remain_undefined():
    image=np.full((80,96),80.)
    result=patch_metrics(image,image,np.array([[30.,30.],[1.,1.]]),np.array([[0.,0.],[100.,0.]]))
    assert result['patch_inbound'].tolist()==[True,False]
    assert not result['ncc_valid'].any() and np.isnan(result['ncc']).all()
    empty=patch_metrics(image,image,np.empty((0,2)),np.empty((0,2)))
    assert empty['ncc'].size==0


def test_fixed_grid_counts_and_spatial_texture_partitions():
    assert grid(512,640).shape==(1008,2)
    assert grid(544,1024).shape==(1800,2)
    uv=np.array([[0.,0.],[32.,30.],[65.,60.],[95.,79.]])
    data=dict(uv=uv,source_std=np.array([1.,5.,6.,np.nan]),near_inlier=np.array([True,False,False,True]),
        endpoint_inbound=np.ones(4,bool),vertical_abs=np.array([0.,1.5,2.,3.]),ncc_valid=np.ones(4,bool))
    m=groups(data,80,96)
    for key in ['texture_le1','texture_1_to5','texture_gt5','texture_invalid']:assert m[key].sum()==1
    assert m['vertical_le_1_5'].sum()==2
    assert sum(m[f'tile_{i}_{j}'].sum() for i in range(3) for j in range(3))==4
    data['source_std'][-1]=np.inf
    assert groups(data,80,96)['texture_invalid'].sum()==1


def test_measure_no_features_and_outside_endpoint_do_not_drop_points():
    rng=np.random.default_rng(1);left=rng.uniform(0,255,(80,96));flow=np.zeros((2,80,96));flow[0]=-3;flow[1]=2
    cov=np.ones_like(flow);depth=np.ones((80,96))*10
    p=np.array([[100.,0,40,0],[0,100,40,0],[0,0,1,0]]);q=p.copy();q[0,3]=-30
    uv=np.array([[32.,32.],[1.,1.]])
    d=measure(left,left,flow,cov,depth,p,q,uv,np.empty((0,2)))
    assert not d['near_inlier'].any() and np.isinf(d['nearest_inlier_distance']).all()
    assert d['endpoint_inbound'].tolist()==[True,False]
    summary=summarize(d,np.ones(2,bool))
    assert summary['count']==2 and summary['positive_gt1_count']==2
    assert summary['endpoint_inbound_count']==1 and summary['vertical_le1_5_count']==0
    assert summary['projection_sign_agreement']==1
    features=np.array([[32.,32.],[17.,1.]])
    d=measure(left,left,flow,cov,depth,p,q,uv,features)
    assert d['near_inlier'].all()
