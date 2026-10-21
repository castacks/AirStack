import numpy as np
import pytest
from Scripts.Setup.td3_stereo_sign import bilinear, magnitude_from_depth, projected_disparity, comparison


def test_bilinear_plane_and_invalid_support():
    y,x=np.mgrid[:4,:5];image=(2*x+3*y+7).astype(float)
    uv=np.array([[1.25,1.5],[-.1,0],[4,1],[np.nan,0]])
    result=bilinear(image,uv)
    assert result[0]==pytest.approx(14)
    assert np.isnan(result[1:]).all()
    image[1,1]=np.nan
    assert np.isnan(bilinear(image,uv[:1])[0])


def test_projection_offset_and_sign_reversal():
    p=np.array([[100.,0,12,0],[0,100,10,0],[0,0,1,0]])
    q=p.copy();q[0,2]=10;q[0,3]=-20
    assert projected_disparity(np.array([2.]),p,q)[0]==pytest.approx(12)
    q[0,3]=20
    assert projected_disparity(np.array([2.]),p,q)[0]==pytest.approx(-8)
    # Camera swap reverses the complete signed displacement, including offset.
    assert projected_disparity(np.array([2.]),q,p)[0]==pytest.approx(8)


def test_depth_erases_flow_sign():
    flow=np.array([-7.,2.,-1.])
    for candidate in [flow,-flow]:
        depth=100*.3/np.abs(candidate)
        np.testing.assert_allclose(magnitude_from_depth(depth,100,.3),np.abs(flow))
    result=magnitude_from_depth(np.array([0.,-1.,np.nan,np.inf]),100,.3)
    assert np.isnan(result).all()
    with pytest.raises(ValueError):magnitude_from_depth(np.ones(1),100,-.3)


def test_comparison_missing_and_nearzero_disparity():
    result=comparison(np.array([0.,-4.,6.,8.]),np.array([2.,5.,np.nan,9.]),np.array([-2.,-5.,-6.,-9.]),np.array([True,True,True,False]))
    assert result['count']==2
    assert result['ratio_population_count']==1
    assert result['model_over_observed_magnitude']['p10_p50_p90'][1]==pytest.approx(1.25)
    assert result['positive_depth_projection_sign_agreement']==1
    empty=comparison(np.empty(0),np.empty(0),np.empty(0),np.empty(0,bool))
    assert empty['count']==0 and empty['positive_observed_fraction'] is None
