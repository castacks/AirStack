import unittest

try:
    import numpy as np
    import pytest
    __import__("cv2")
except ImportError as error:
    raise unittest.SkipTest("optional UAV perception tests: " + str(error))
from rrm.uav_marker import detect_red_surface

K=[100.,0.,50.,0.,100.,40.,0.,0.,1.]


def fixture():
    rgb=np.zeros((80,100,3),np.uint8);rgb[30:50,40:60]=[255,0,0]
    return rgb,np.full((80,100),3.,np.float32)


def test_calibrated_surface():
    v=detect_red_surface(*fixture(),K)
    assert v['status']=='OBSERVED' and v['surface_depth_z_m']==3
    assert v['surface_point_optical_m']==pytest.approx([-.03,-.03,3.])


def test_blue_distractor_absent():
    rgb,d=fixture();rgb[:]=[0,0,255]
    assert detect_red_surface(rgb,d,K)['status']=='ABSENT'


def test_two_red_regions_ambiguous():
    rgb,d=fixture();rgb[5:15,5:15]=[255,0,0]
    assert detect_red_surface(rgb,d,K)['status']=='AMBIGUOUS'


@pytest.mark.parametrize('bad',[0.,float('nan'),float('inf'),-1.,25.])
def test_unknown_depth_stays_unknown(bad):
    rgb,d=fixture();d[:]=bad
    v=detect_red_surface(rgb,d,K)
    assert v['status']=='DEPTH_UNKNOWN' and v['surface_point_optical_m'] is None


def test_dimensions_rejected():
    rgb,d=fixture()
    with pytest.raises(ValueError):detect_red_surface(rgb,d[:5],K)


def test_invalid_calibration():
    rgb,d=fixture();k=K.copy();k[0]=0
    with pytest.raises(ValueError):detect_red_surface(rgb,d,k)


def test_surface_pixel_is_observed_even_with_center_hole():
    rgb,d=fixture();rgb[36:44,46:54]=0
    v=detect_red_surface(rgb,d,K);u,y=v['surface_pixel_uv']
    assert tuple(rgb[y,u])==(255,0,0)


def test_integer_depth_units_rejected():
    rgb,d=fixture()
    with pytest.raises(ValueError):detect_red_surface(rgb,d.astype(np.uint16),K)


def test_nonstandard_projection_rejected():
    rgb,d=fixture();k=K.copy();k[1]=.2
    with pytest.raises(ValueError):detect_red_surface(rgb,d,k)
