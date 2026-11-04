"""Synthetic publisher/common-scorer integration; no real reference outcomes."""
from pathlib import Path
import sys
import math

import numpy as np
import pytest

pytest.importorskip('scipy')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from selected_motion_trajectory import publish
from independent_benchmark_metrics_v2 import score


def fixture():
    times=[i*100_000_000 for i in range(31)];poses=[]
    for i in range(31):
        t=i*.1;c,s=math.cos(t),math.sin(t)
        poses.append([[c,-s,0.,math.sin(t)],[s,c,0.,1-math.cos(t)],
                      [0.,0.,1.,t*t*.1],[0.,0.,0.,1.]])
    slots=[]
    for i in range(30):
        slots.append({'target_interval_ns':times[i:i+2],'deadline_ns':i+20,'action':'RGB',
            'selected':{'received_ns':i+20,'controller_clock_id':'analytic_controller',
                'payload':{'id':str(i),'camera':'rgb','available':True,'reference_transport_used':False,
                    'native_interval_ns':times[i:i+2],'sensor_clock_id':'analytic_sensor',
                    'source_frame':'rgb','native_segment_id':0,'reset_before':False,
                    'delta_camera':np.linalg.solve(np.asarray(poses[i]),np.asarray(poses[i+1])).tolist()}}})
    contract={'time_ns':times,'source_ids':[str(i) for i in range(31)],'target_frame':'rgb',
        'sensor_clock_id':'analytic_sensor','sensor_clock_evidence_ref':'analytic_definition',
        'controller_clock_id':'analytic_controller','max_source_gap_ns':150_000_000,
        'initialization':{'source_time_ns':0,'deadline_ns':10,'received_ns':10,
            'controller_clock_id':'analytic_controller','target_frame':'rgb','evidence_ref':'analytic_origin'},
        'frame_contracts':{'rgb':{'source_frame':'rgb','target_frame':'rgb',
            'target_from_camera':np.eye(4).tolist(),'frame_evidence_ref':'analytic_identity'}},
        'slots':slots,'terminal_success':True}
    return {'time_ns':times,'poses':poses},contract


def test_common_v2_scorer_accepts_exact_publisher_clock_and_pose_convention():
    reference,contract=fixture();published=publish(contract);result=score(reference,published)
    assert result['engineering_success']
    assert result['pose_coverage']==result['rpe_coverage']==1.
    assert result['ate_origin_rmse_m']<1e-12
    assert result['rpe_translation_rmse_m']<1e-12 and result['rpe_rotation_rmse_deg']<1e-10


def test_common_v2_scorer_retains_publisher_hole_and_reset_penalty():
    reference,contract=fixture();contract['slots'][14]['action']='ABSTAIN'
    published=publish(contract);result=score(reference,published)
    assert published['poses'][15] is None
    assert result['published_pose_frames']==30 and result['source_frames']==31
    assert result['reset_count']==1 and not result['engineering_success']
    assert all(not(i<=15<=j) for i,j in [r['source_indices'] for r in result['rpe_rows']])
