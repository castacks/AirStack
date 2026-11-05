from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import pytest
from stereo_reference_diagnostic import decompose,extract,match_summary,summarize

def test_exact_motion():
 r=decompose([1,2,3],[1,2,3]);assert r['ratio']==1 and r['error_m']==0

def test_parallel_shortfall():
 r=decompose([0,0,4],[0,0,1]);assert r['parallel_error_m']==-3 and r['perpendicular_error_m']==0 and r['ratio']==.25

def test_perpendicular_error():
 r=decompose([2,0,0],[2,3,0]);assert r['parallel_error_m']==0 and r['perpendicular_error_m']==3 and r['error_m']==3

def test_reverse_direction():
 r=decompose([1,0,0],[-1,0,0]);assert r['cosine']==-1 and r['ratio']==1 and r['parallel_error_m']==-2

def test_rotated_coordinates_preserve_decomposition():
 q=np.array([[0,-1,0],[1,0,0],[0,0,1]])
 a=np.array([1,2,3]);b=np.array([4,-2,1])
 for k,v in decompose(a,b).items():assert np.isclose(v,decompose(q@a,q@b)[k])

def test_zero_motion_is_retained_not_divided():
 r=decompose([0,0,0],[1,0,0]);assert r['ratio'] is None and r['cosine'] is None and r['error_m']==1
 assert summarize([r])['zero_reference_pairs']==1

def test_invalid_vectors_rejected():
 with pytest.raises(ValueError):decompose([1,2,float('nan')],[1,2,3])

def test_signed_disparity_and_empty_populations():
 matches=[dict(left=[10,5],right=[7,6]),dict(left=[2,5],right=[4,9])]
 s=match_summary(matches,100);assert s['positive_disparity']==1 and s['within_vertical']==1 and s['positive_and_vertical']==1
 assert match_summary([],100)['abs_vertical_px']['median'] is None

def test_textureless_pair_is_explicit_empty():
 n,m,rows=extract(np.zeros((100,100),np.uint8),np.zeros((100,100),np.uint8));assert (n,m,rows)==(0,0,[])
