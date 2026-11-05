from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import pytest
from native_stereo_history import frame_stats

def test_unmatched_close_far_and_flags():
 row=dict(index=5,N=4,right_keypoints=8,close_depth_m=3,features=[[1,2,0,-1,-1,None,None],[10,3,1,8,2,True,False],[20,4,0,19,4,False,True],[30,4,0,28,3,True,False]])
 s=frame_stats(row);assert s['accepted']==3 and s['features']==4 and s['close']==1 and s['far']==2
 assert s['associated_accepted']==2 and s['flagged_outlier_accepted']==1
 assert s['median_disparity_px']==2 and s['median_depth_m']==3

def test_empty_native_frame_retained():
 s=frame_stats(dict(index=0,N=0,right_keypoints=0,close_depth_m=4,features=[]));assert s['accepted']==0 and s['acceptance'] is None and s['median_depth_m'] is None

@pytest.mark.parametrize('features',[[[10,2,0,-1,2,False,False]],[[10,2,0,11,2,False,False]],[[10,2,0,-2,-1,False,False]],[[10,2,0,9,float('nan'),False,False]]])
def test_malformed_stereo_rejected(features):
 with pytest.raises(ValueError):frame_stats(dict(index=0,N=1,right_keypoints=1,close_depth_m=4,features=features))

def test_omitted_feature_rejected():
 with pytest.raises(ValueError):frame_stats(dict(index=0,N=1,right_keypoints=1,close_depth_m=4,features=[]))
