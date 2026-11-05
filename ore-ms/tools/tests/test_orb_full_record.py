from copy import deepcopy
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from orb_full_record import evaluation_view,failure_view

def fixture():
    times=[0,100000000,400000000,500000000,600000000]
    p=dict(time_ns=times,right_time_ns=[t+1 for t in times],stems=[f'{i:06}' for i in range(5)])
    pose=[[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]]
    rows=[dict(index=i,time_ns=t,right_time_ns=t+1,state=2,map_id=7,T_camera_from_world=deepcopy(pose)) for i,t in enumerate(times)]
    legacy=dict(time_ns=times,poses=[deepcopy(pose) for _ in times],segments=[0,0,1,1,1],
                rows=rows,segment_count=2,coordinate_frame='rgb_rectified_left_camera',terminal_success=True)
    return p,legacy

def test_time_gap_does_not_create_map_reset_or_change_poses():
    p,l=fixture();before=deepcopy(l);view,b=evaluation_view(p,l)
    assert view['segments']==[0]*5 and view['poses']==l['poses'] and view['time_ns']==p['time_ns']
    assert l==before and b['recorded_time_gaps']==1 and b['legacy_segment_count']==2
    assert b['evaluation_segment_count']==1 and b['observed_map_identity_changes']==0

def test_missing_output_breaks_even_if_same_map_recovers():
    p,l=fixture();l['poses'][2]=None;l['rows'][2]['T_camera_from_world']=None
    v,b=evaluation_view(p,l)
    assert v['segments']==[0,0,None,1,1] and b['recovery_after_missing_splits']==1

def test_map_identity_change_breaks_continuous_valid_output():
    p,l=fixture()
    for r in l['rows'][3:]:r['map_id']=8
    v,b=evaluation_view(p,l)
    assert v['segments']==[0,0,0,1,1] and b['observed_map_identity_changes']==1

@pytest.mark.parametrize('kind',['float_time','wrong_time','index','map_float','map_negative','state','missing_row','native_pose','parallel'])
def test_corrupt_source_or_map_evidence_refused(kind):
    p,l=fixture()
    if kind=='float_time':l['time_ns']=list(l['time_ns']);l['time_ns'][0]=0.
    elif kind=='wrong_time':l['rows'][0]['right_time_ns']=2
    elif kind=='index':l['rows'][0]['index']=1
    elif kind=='map_float':l['rows'][0]['map_id']=7.
    elif kind=='map_negative':l['rows'][0]['map_id']=-1
    elif kind=='state':l['rows'][0]['state']=3
    elif kind=='missing_row':l['rows'][0]=None
    elif kind=='native_pose':l['poses'][0]=None
    else:l['poses'].pop()
    with pytest.raises(ValueError):evaluation_view(p,l)

def test_fail_closed_publication_retains_every_expected_slot():
    p,_=fixture();v=failure_view(p,'malformed native receipt')
    assert len(v['poses'])==5 and all(vv is None for vv in v['poses'])
    assert v['time_ns']==p['time_ns'] and v['source_ids']==p['stems'] and not v['terminal_success']
