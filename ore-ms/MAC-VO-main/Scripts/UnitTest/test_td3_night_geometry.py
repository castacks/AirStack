import numpy as np
from Scripts.Setup.td3_night_geometry import measure, masks, summarize


def fixture():
    uv = np.array([[10.,10.],[20.,20.],[30.,30.]])
    return {'grid_uv0':uv, 'grid_uv1':uv+np.array([0.,5.]),
            'grid_depth0':np.ones(3)*2, 'grid_depth1':np.ones(3)*2,
            'grid_raw_flow_cov':np.ones((3,3)), 'grid_inbound':np.array([True,False,True]),
            'K':np.array([[100.,0,0],[0,100.,0],[0,0,1.]])}


def test_exact_geometry_and_wrong_flow():
    p=fixture(); relative=np.eye(4); relative[2,3]=.1
    d=measure(p,'grid',relative)
    np.testing.assert_allclose(d['forward_px'],0,atol=1e-12)
    np.testing.assert_allclose(d['sampson_px'],0,atol=1e-12)
    p['grid_uv1'][:,0]+=3
    d=measure(p,'grid',relative)
    np.testing.assert_allclose(d['forward_px'],3)
    assert np.all(d['sampson_px']>0)


def test_epipolar_depth_independence_and_zero_translation():
    p=fixture(); relative=np.eye(4); relative[1,3]=.1
    first=measure(p,'grid',relative)
    p['grid_depth0'][:]=np.nan; p['grid_depth1'][:]=np.nan
    second=measure(p,'grid',relative)
    np.testing.assert_equal(first['sampson_px'],second['sampson_px'])
    assert np.isnan(second['forward_px']).all()
    assert np.isnan(measure(p,'grid',np.eye(4))['sampson_px']).all()


def test_behind_camera_and_masks():
    p=fixture(); p['grid_depth0'][0]=-1; p['grid_depth0'][1]=np.nan
    relative=np.eye(4); relative[0,3]=-3
    d=measure(p,'grid',relative)
    assert np.isnan(d['forward_px']).all()
    groups=masks(d,40)
    assert groups['all'].sum()==3 and groups['inbound'].sum()==2
    assert groups['depth_invalid'].sum()==2
    assert groups['inbound_both_depths_max50m'].sum()==1


def test_empty_and_constant_associations():
    d=measure(fixture(),'grid',np.eye(4))
    empty=summarize(d,np.zeros(3,bool))
    assert empty['count']==0 and empty['forward_px_rho'] is None
    assert summarize(d,np.ones(3,bool))['forward_px_rho'] is None
