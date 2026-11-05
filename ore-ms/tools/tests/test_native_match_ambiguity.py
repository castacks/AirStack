from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from native_match_ambiguity import ambiguity,reconstruct,correlation

def test_descriptor_gap_and_separated_patch_competitor():
 r=ambiguity(dict(candidates=[[1,20],[2,30]],costs=[50,50,50,2,1,2,10,20,30,40,50]))
 assert r['descriptor_margin']==10 and r['descriptor_ratio']==2/3 and r['patch_relative_gap']==9

def test_zero_descriptor_tie_and_patch_tie():
 r=ambiguity(dict(candidates=[[1,0],[2,0]],costs=[50,50,50,2,1,2,1,20,30,40,50]))
 assert r['descriptor_tie'] and r['descriptor_ratio'] is None and r['patch_separated_tie'] and r['patch_relative_gap']==0

def test_single_candidate_and_missing_costs_are_null():
 r=ambiguity(dict(candidates=[[1,30]],costs=None));assert r['descriptor_margin'] is None and r['descriptor_ratio'] is None and r['patch_relative_gap'] is None

def test_symmetric_subpixel_known_depth():
 p=dict(patch_status='ok',costs=[20,16,12,8,2,0,2,8,12,16,20],scaled_right_center=8)
 assert reconstruct(p,dict(scale_factors=[1],max_disparity=100,bf=20),[10,10,0,8,10])==[8,10]

def test_boundary_minimum_not_accepted():
 p=dict(patch_status='ok',costs=list(range(11)),scaled_right_center=8)
 assert reconstruct(p,dict(scale_factors=[1],max_disparity=100,bf=20),[10,10,0,8,10]) is None

def test_rank_guard_and_expected_negative_direction():
 rows=[dict(faithful=True,ambiguity=dict(descriptor_margin=i),original_metrics=dict(abs_relative_error=10-i)) for i in range(10)]
 assert correlation(rows,'descriptor_margin')['rho'] < -.999
 assert correlation(rows[:-1],'descriptor_margin')['reason']=='fewer_than_10'
 for r in rows:r['ambiguity']['descriptor_margin']=1
 assert correlation(rows,'descriptor_margin')['reason']=='constant'
