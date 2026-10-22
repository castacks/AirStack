import numpy as np
import pytest
from Scripts.Setup.td3_gnss_translation import positions_at,magnitudes


def test_large_utm_precision_and_local_interpolation():
    t=np.array([1761610000000000000,1761610000020000000],np.int64)
    p=np.array([[4480000.,-602000.,300.],[4480000.02,-601999.96,300.006]])
    q=t[:1]+10000000;result=positions_at(t,p,q)
    np.testing.assert_allclose(result,(p[1:]-p[:1])/2,atol=1e-12)
    assert np.array_equal(p[:,0].astype(np.float32),np.array([4480000.,4480000.],np.float32))
    assert result[0,0]>.009


def test_translation_and_signed_axis_invariance():
    p=np.array([[0.,0.,0.],[3.,4.,2.],[-2.,5.,3.]])
    for transformed in [p+[5000000,-600000,400],p[:,[1,0,2]]*[-1,1,-1]]:
        for key in ['xy','xyz','vertical']:np.testing.assert_allclose(magnitudes(p)[key],magnitudes(transformed)[key])


def test_boundaries_offsets_and_no_extrapolation():
    t=np.array([100,200],np.int64);p=np.array([[0.,0.,0.],[1.,2.,3.]])
    np.testing.assert_array_equal(positions_at(t,p,t),p)
    for q in [np.array([99],np.int64),np.array([201],np.int64)]:
        with pytest.raises(ValueError,match='extrapolation'):positions_at(t,p,q)


def test_invalid_sources_and_queries_are_explicit():
    t=np.array([100,200],np.int64);p=np.zeros((2,3))
    for ts,ps,qs in [(t[::-1],p,t),(np.array([100,100]),p,t),(t,np.full((2,3),np.nan),t),(t,p,t.astype(float)),(t,np.zeros((2,2)),t)]:
        with pytest.raises(ValueError):positions_at(ts,ps,qs)
