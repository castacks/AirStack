from pathlib import Path
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from droid_ms2_development import native_times,metric_poses,canonical_pair,valid_native_vectors


def test_source_clock_is_relative_before_float_conversion():
    epoch=1_628_857_086_260_566_695
    times=[epoch,epoch+66_661_111,epoch+199_993_333]
    got=native_times(times)
    assert got[0]==0 and np.all(np.diff(got)>0)
    np.testing.assert_allclose(got.astype(float)*1e8,[0,66_661_111,199_993_333],atol=10)


@pytest.mark.parametrize('times',[[False,1],[0.,1.],[1,1],[2,1],[-1,1],[0,2**63],[0,2**62,2**62+1]])
def test_invalid_or_colliding_clocks_rejected(times):
    with pytest.raises(ValueError):native_times(times)


def test_translation_uses_provided_baseline_without_rotation_or_scale_fit():
    matrix=np.array([[0.,-1,0,.2],[1,0,0,-.1],[0,0,1,.3],[0,0,0,1]])
    poses,segments,reasons=metric_poses([matrix],.3,True)
    np.testing.assert_allclose(np.asarray(poses[0])[:3,3],[.6,-.3,.9])
    np.testing.assert_array_equal(np.asarray(poses[0])[:3,:3],matrix[:3,:3])
    assert segments==[0] and reasons==[None]


def test_finite_filler_is_not_admitted_without_frontend_initialization():
    poses,segments,reasons=metric_poses([np.eye(4)]*3,.3,False)
    assert poses==segments==[None]*3
    assert reasons==['frontend_not_initialized']*3


def test_invalid_native_outputs_remain_null_and_break_segments():
    bad=np.eye(4);bad[0,0]=2
    poses,segments,reasons=metric_poses([np.eye(4),bad,np.eye(4),None],.3,True)
    assert segments==[0,None,1,None] and poses[1] is None and poses[3] is None
    assert reasons[1]=='nonrigid_native_matrix'


@pytest.mark.parametrize('baseline',[True,0.,-.3,float('nan')])
def test_bad_baseline_rejected(baseline):
    with pytest.raises(ValueError):metric_poses([np.eye(4)],baseline,True)


def test_zero_nonfinite_and_nonunit_quaternions_are_not_poses():
    vectors=np.array([[0,0,0,0,0,0,1],[0,0,0,0,0,0,0],[0,0,0,0,0,0,2],[np.nan,0,0,0,0,0,1]])
    assert valid_native_vectors(vectors).tolist()==[True,False,False,False]


def test_canonical_thermal_replicates_channels_without_resizing(tmp_path):
    raw=np.arange(256,dtype=np.uint8).reshape(16,16)
    for side in ['left','right']:
        (tmp_path/side).mkdir();assert cv2.imwrite(str(tmp_path/side/'000000.png'),raw)
    images=canonical_pair(tmp_path,'000000',[16,16])
    assert images.shape==(2,16,16,3)
    for side in range(2):
        for channel in range(3):np.testing.assert_array_equal(images[side,:,:,channel],raw)
    with pytest.raises(ValueError):canonical_pair(tmp_path,'000000',[8,8])
