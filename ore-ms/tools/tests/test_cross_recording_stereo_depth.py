from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from cross_recording_stereo_depth import feature_result,neighborhood,correlation,ambiguity

def test_quantized_depth_and_missing_geometry():
 d=np.zeros((20,20),np.uint16);d[10,10]=2560;f=[10,10,0,9,10,None,None]
 r=feature_result(f,d,10,1,20);assert r['reason']=='compared' and r['metrics']['depth_ratio']==1
 assert feature_result([10,10,0,-1,-1,None,None],d,10,1,20)['reason']=='native_stereo_missing'
 assert feature_result([0,0,0,0,1,None,None],d,10,1,20)['reason']=='zero_reference'

def test_reference_only_group_does_not_depend_on_native_depth():
 d=np.full((20,20),2560,np.uint16);a=neighborhood([10,10,0,9,5,None,None],d,10,1,2);b=neighborhood([10,10,0,8,15,None,None],d,10,1,2)
 assert a['group']==b['group']=='locally_consistent' and a['candidates']==b['candidates'] and a['interval_class']!=b['interval_class']

def test_rank_guard_and_tied_ambiguity():
 a=ambiguity(dict(candidates=[[0,10],[1,10]],costs=[8,4,2,4,8,9,10,11,12,13,14]));assert a['descriptor_margin']==0 and a['descriptor_ratio']==1 and a['patch_relative_gap']==3
 assert correlation([], 'patch_relative_gap')==dict(points=0,rho=None,reason='fewer_than_10')
 points=[dict(faithful=True,ambiguity=dict(patch_relative_gap=i//2),original_metrics=dict(abs_relative_error=9-i//2)) for i in range(10)]
 assert abs(correlation(points,'patch_relative_gap')['rho']+1)<1e-12
