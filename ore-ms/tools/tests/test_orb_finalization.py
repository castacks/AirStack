from copy import deepcopy
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from orb_finalization import common_views,final_view,segments

def fixture():
    eye=np.eye(4).tolist();p=dict(stems=['000000','000001','000002'],time_ns=[0,100,300],right_time_ns=[1,101,301])
    e=dict(time_ns=p['time_ns'],source_ids=p['stems'],poses=[deepcopy(eye),None,deepcopy(eye)],segments=[0,None,1],terminal_success=True,rows=[dict(map_id=7),None,dict(map_id=8)])
    rows=[dict(index=i,source_id=p['stems'][i],time_ns=p['time_ns'][i],right_time_ns=p['right_time_ns'][i],received_valid=i!=1,candidate_count=1,native_lost=False,final_map_id=7 if i<2 else 8,reason=None,T_world_from_camera=deepcopy(e['poses'][i])) for i in range(3)]
    return p,e,rows

def test_missing_and_map_boundaries_not_stitched():
    p,e,r=fixture();f=final_view(p,e,r)
    assert f['poses'][1] is None and f['segments']==[0,None,1]

@pytest.mark.parametrize('kind',['resurrect','map','ambiguous','native_lost','clock','source'])
def test_bad_final_binding_rejected(kind):
    p,e,r=fixture()
    if kind=='resurrect':r[1]['T_world_from_camera']=np.eye(4).tolist()
    elif kind=='map':r[0]['final_map_id']=8
    elif kind=='ambiguous':r[0]['candidate_count']=2
    elif kind=='native_lost':r[0]['native_lost']=True
    elif kind=='clock':r[0]['time_ns']=1
    else:r[0]['source_id']='000009'
    with pytest.raises(ValueError):final_view(p,e,r)

def test_additional_final_hole_stays_in_common_denominator():
    p,e,r=fixture();r[2]['T_world_from_camera']=None;r[2]['reason']='native_lost_or_missing_history'
    final=final_view(p,e,r);d=deepcopy(e);views,mask=common_views(e,final,d)
    assert mask==[True,False,False]
    assert all(v['segments']==[0,None,None] and len(v['poses'])==3 for v in views)

def test_same_clock_axis_required():
    p,e,r=fixture();f=final_view(p,e,r);d=deepcopy(e);d['time_ns']=[0,101,300]
    with pytest.raises(ValueError):common_views(e,f,d)

def test_homogeneous_float_roundoff_only_is_canonicalized():
    p,e,r=fixture();r[0]['T_world_from_camera'][3][0]=1e-16
    v=final_view(p,e,r);assert v['poses'][0][3]==[0,0,0,1]
    r[0]['T_world_from_camera'][3][0]=1e-5
    with pytest.raises(ValueError):final_view(p,e,r)

def test_time_gap_only_keeps_map_segment():
    assert segments([True,True,True],[0,0,1])==[0,0,1]

def test_double_factor_chain_and_map_gauge_direction():
    a=np.eye(4);a[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]];a[0,3]=2
    b=np.eye(4);b[1,3]=3;c=np.eye(4);c[2,3]=5
    world=np.linalg.inv(a@b@c)
    np.testing.assert_allclose((a@b@c)@world,np.eye(4),atol=1e-14)
    assert not np.allclose(world,np.linalg.inv(c@b@a))
