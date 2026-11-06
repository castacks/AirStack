from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from copy import deepcopy
from stereo_baseline_initialization import union_views,compare_frames

def test_union_boundaries_and_holes_are_preserved():
 eye=[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
 a=dict(poses=[eye]*5,segments=[0,0,0,0,0],time_ns=list(range(5)),source_ids=list(range(5)),terminal_success=True);b=deepcopy(a);b['segments']=[0,0,1,1,1];c=deepcopy(a);c['poses'][3]=None;c['segments'][3]=None
 views,mask=union_views([a,b,c]);assert mask==[True,True,True,False,True] and views[0]['segments']==[0,0,1,None,2]

def test_geometry_and_acceptance_differences():
 a=dict(features=[[10,5,0,9,10,True,False],[20,5,0,-1,-1,False,False]]);b=deepcopy(a);b['features'][0][3]=8;b['features'][0][4]=5;b['features'][1][3]=19;b['features'][1][4]=10
 r=compare_frames(a,b);assert r['coordinates_equal'] and r['accepted_changes']==1 and r['right_changes']==2 and r['depth_changes']==2

def test_unavailable_not_zero_or_equal():
 assert compare_frames(dict(available=False),dict(features=[]))['comparable'] is False
 assert compare_frames(dict(features=[[10,5,0,9,10]]),dict(features=[[11,5,0,9,10]]))['coordinates_equal'] is False
