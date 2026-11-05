from copy import deepcopy
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from provided_reference_full_record import bind
from independent_benchmark_metrics_v2 import score

def fixture():
    n=31;times=[i*100000000 for i in range(n)];stems=[f'{i:06}' for i in range(n)]
    poses=[]
    for i in range(n):
        p=np.eye(4);p[:3,3]=[i*.2,(i*.1)**2,np.sin(i*.1)];poses.append(p.tolist())
    e=dict(time_ns=times,source_ids=stems,poses=poses,segments=[0]*n,terminal_success=True,
           scale_fitted=False,reference_used=False,physical_frame_or_clock_verified=False)
    m=dict(records=[dict(stem=s,cameras={k:dict(left=dict(recorded_time_ns=t)) for k in ['rgb','thr']}) for s,t in zip(stems,times)])
    return e,m

def test_complete_camera_local_axis():
    e,m=fixture();assert bind(e,m,'rgb')['time_ns']==e['time_ns']

@pytest.mark.parametrize('kind',['float','bool','time','stem','source_order','missing','scale','labels','physical','camera'])
def test_axis_and_scope_reject_corruption(kind):
    e,m=fixture();mode='rgb'
    if kind=='float':e['time_ns'][0]=0.
    elif kind=='bool':e['time_ns'][0]=False
    elif kind=='time':e['time_ns'][3]+=1
    elif kind=='stem':m['records'][0]['stem']='000001'
    elif kind=='source_order':e['source_ids'][0]='000001'
    elif kind=='missing':e['poses'].pop()
    elif kind=='scale':e['scale_fitted']=True
    elif kind=='labels':e['reference_used']=True
    elif kind=='physical':e['physical_frame_or_clock_verified']=True
    else:mode='nir'
    with pytest.raises(ValueError):bind(e,m,mode)

def test_rigid_world_gauge_and_scale_control():
    e,_=fixture();ref=dict(time_ns=e['time_ns'],poses=deepcopy(e['poses']))
    t=np.eye(4);t[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]];t[:3,3]=[4,-2,7]
    e['poses']=[(t@np.asarray(p)).tolist() for p in e['poses']]
    assert score(ref,e)['ate_se3_rmse_m']<1e-12
    e['poses']=[np.asarray(p).copy() for p in ref['poses']]
    for p in e['poses']:p[:3,3]*=2
    assert score(ref,e)['ate_se3_rmse_m']>.1

def test_invalid_reference_hole_and_gap_denominators():
    e,_=fixture();ref=dict(time_ns=e['time_ns'],poses=deepcopy(e['poses']))
    ref['poses'][15][0][0]=2
    e['poses'][7]=None;e['segments']=[0]*7+[None]+[1]*23
    r=score(ref,e)
    assert r['source_frames']==31 and r['reference_valid_frames']==30
    assert r['published_pose_frames']==30 and r['reset_count']==1 and r['rpe_coverage']<1
    assert r['reference_exclusions'][0]['index']==15
