from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from native_depth_proxy import feature_result,stats

def feature(u=3,v=2,z=4,right=2):return [u,v,0,right,z,False,False]
def test_depth_units_and_exact_agreement():
 a=np.zeros((5,8),np.uint16);a[2,3]=1024;r=feature_result(feature(),a,8,.5,8)
 assert r['reason']=='compared' and r['reference_m']==4 and r['metrics']['depth_ratio']==1 and r['metrics']['depth_error_m']==0 and r['metrics']['disparity_error_px']==0

def test_nearest_half_rounds_up_without_interpolation():
 a=np.zeros((5,8),np.uint16);a[2,3]=1024
 assert feature_result(feature(2.5,1.5),a,8,.5,8)['pixel']==[3,2]
 assert feature_result(feature(2.499,1.499),a,8,.5,8)['reason']=='zero_reference'

def test_native_missing_retains_reference_supported_reason():
 a=np.full((5,8),1024,np.uint16);r=feature_result(feature(z=-1,right=-1),a,8,.5,8)
 assert r['reason']=='native_stereo_missing' and r['reference_m']==4 and r['metrics'] is None

def test_proxy_right_boundary_and_empty_label():
 a=np.full((5,8),256,np.uint16)
 assert feature_result(feature(u=1),a,8,.5,8)['reason']=='proxy_right_outside'
 assert feature_result(feature(u=7.5),a,8,.5,8)['reason']=='reference_pixel_outside'
 assert feature_result(feature(),np.zeros((5,8),np.uint16),8,.5,8)['reason']=='zero_reference'

def test_signed_depth_shortfall_and_disparity_excess():
 a=np.full((5,8),2048,np.uint16);r=feature_result(feature(z=4,right=2),a,8,.5,8)['metrics']
 assert r['depth_ratio']==.5 and r['depth_error_m']==-4 and r['abs_relative_error']==.5 and r['disparity_error_px']==.5

def test_empty_statistics_are_null():
 assert stats([])['n']==0 and stats([])['median'] is None
