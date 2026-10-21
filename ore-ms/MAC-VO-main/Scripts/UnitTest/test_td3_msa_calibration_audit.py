import cv2
import numpy as np
import pytest
import yaml

from Scripts.Setup.td3_msa_calibration_audit import msa_camera, reinterpret_rays


def test_fixed_raw_pixels_reinterpreted_between_lens_models():
    k=np.array([[413.,0,314],[0,414,251],[0,0,1]])
    old_d=np.array([-.36,.12,-.0006,.0004])
    new_d=np.array([-.06,-.03,-.015,.01])
    # Input observations are defined independently as fixed distorted pixels.
    xy=np.array([[20.,30.],[310,250],[620,480],[120,400],[480,80]])
    undistorted=cv2.undistortPointsIter(xy[:,None],k,old_d,None,None,
        (cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,100,1e-13)).reshape(-1,2)
    rays=np.column_stack([undistorted,np.ones(len(xy))])
    actual,pixels,checks=reinterpret_rays(rays,k,old_d,k,new_d)
    np.testing.assert_allclose(pixels,xy,atol=1e-9)
    expected=cv2.fisheye.undistortPoints(xy[:,None],k,new_d).reshape(-1,2)
    np.testing.assert_allclose(actual[:,:2],expected,atol=1e-10)
    assert max(checks.values())<1e-8


@pytest.mark.parametrize('model,last', [('radtan',0),('equidistant',.1)])
def test_reject_incompatible_msa_models(tmp_path,model,last):
    data={'distortion_model':model,'camera_matrix':{'data':np.eye(3).ravel().tolist()},
          'distortion_coefficients':{'data':[0,0,0,0,last]}}
    p=tmp_path/'camera.yaml';p.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):msa_camera(p)
