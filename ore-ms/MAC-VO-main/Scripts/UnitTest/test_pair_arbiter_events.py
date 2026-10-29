"""Contract and failure cases for pairing separate causal event captures."""
import copy
import hashlib
import json

import pytest

from Scripts.Setup.pair_arbiter_events import build_pairs, load_verified_run, HEALTH_FIELDS


def fixture(offset=8):
    start=1761615290010015011
    rows=[{'rgb_left':{'time_ns':start+i*100},'thermal_left':{'time_ns':start+i*100-offset}} for i in range(3)]
    reports={c:{'modality':c,'input_rows':copy.deepcopy(rows),'coordinate_frame':c+' camera NED','status':'PASS_EXECUTION_ONLY'} for c in ('rgb','thermal')}
    streams={}
    for c in reports:
        e=[]
        for i in range(2):
            t0,t1=[rows[j][c+'_left']['time_ns'] for j in [i,i+1]]
            e += [{'kind':'optimization_submitted','submission_id':i+1,'from_idx':i,'frame_idx':i+1,'time0_ns':t0,'time1_ns':t1},
                  {'kind':'pair_end','time0_ns':t0,'time1_ns':t1,'selected_count':12,'inbound_count':11,'retained_count':10,
                   'occupied_3x3_cells':7,'grid_count':20,'grid_positive_uu_vv_count':20,'grid_trace_q10_q50_q90_px2':[1.,2.,3.],
                   'original_need_interp':False,'execution_error':None,'motion_status':'submitted'},
                  {'kind':'motion_result_written','submission_id':i+1,'from_idx':i,'frame_idx':i+1,'time0_ns':t0,'time1_ns':t1,
                   'first_write_for_submission':True,'stage':'pair_processing' if i==0 else 'termination','original_need_interp':False,
                   'parent_pose_camera_map_xyzw':[0.,0.,0.,0.,0.,0.,1.],
                   'committed_pose_camera_map_xyzw':[1.,0.,0.,0.,0.,0.,1.],
                   'derived_camera_delta_xyzw':[1.,0.,0.,0.,0.,0.,1.],'derived_delta_unavailable_reason':None}]
        for i,event in enumerate(e):event.update(sequence=i,observation_monotonic_ns=1000+i)
        streams[c]=e
    return reports,streams


def build(reports,streams):
    return build_pairs(reports['rgb'],streams['rgb'],reports['thermal'],streams['thermal'])


def resequence(events):
    for i,e in enumerate(events):e.update(sequence=i,observation_monotonic_ns=1000+i)


def test_exact_large_timestamps_offsets_and_no_pose_promotion():
    reports,streams=fixture();r=build(reports,streams)
    assert r['alignment']['pair_count']==2 and r['alignment']['exact_same_intervals']==0
    assert r['pairs'][0]['rgb_minus_thermal_endpoint_ns']==[8,8]
    assert r['pairs'][0]['cameras']['rgb']['sensor_interval_ns'][0]==1761615290010015011
    assert r['availability']['rgb']['camera_local_motion_pairs']==2
    assert r['eligibility']['health_schema_replay_complete'] and not r['eligibility']['pose_gate_training_ready']
    assert not r['pairs'][0]['cameras']['thermal']['body_transform_verified']


def test_exact_timing_does_not_certify_frame_or_pose_labels():
    reports,streams=fixture(offset=0);r=build(reports,streams)
    assert r['alignment']['exact_same_intervals']==2 and not r['eligibility']['pose_gate_training_ready']


def test_missing_pair_is_retained_and_incomplete():
    reports,streams=fixture();streams['thermal'].pop(1);resequence(streams['thermal']);r=build(reports,streams)
    assert len(r['pairs'])==2 and not r['eligibility']['health_schema_replay_complete']
    row=r['pairs'][0]['cameras']['thermal'];assert row['health_status']=='MISSING_PAIR_END' and not row['motion_available_camera_local']


def test_unsupported_without_motion_not_hidden_or_assigned_zero_error():
    reports,streams=fixture();e=streams['thermal'];e[1].update(original_need_interp=True,selected_count=0,inbound_count=0,retained_count=0,occupied_3x3_cells=0,motion_status='not_submitted')
    e.pop(2);e.pop(0);resequence(e);r=build(reports,streams)
    row=r['pairs'][0]['cameras']['thermal'];assert row['health_status']=='COMPLETE' and row['motion_status']=='NO_FIRST_WRITE'
    assert row['motion'] is None and not row['motion_available_camera_local']


def test_repeated_write_is_not_new_motion_or_new_availability():
    reports,streams=fixture();repeat=copy.deepcopy(streams['rgb'][-1]);repeat['first_write_for_submission']=False
    streams['rgb'].append(repeat);resequence(streams['rgb']);r=build(reports,streams)
    assert r['availability']['rgb']['repeated_writes_not_counted_as_new_motion']==1
    assert r['availability']['rgb']['camera_local_motion_pairs']==2


def test_graph_interval_mismatch_preserved_and_not_available():
    reports,streams=fixture();streams['rgb'][0]['time0_ns']-=10;streams['rgb'][2]['time0_ns']-=10
    r=build(reports,streams);row=r['pairs'][0]['cameras']['rgb']
    assert row['motion_status']=='INTERVAL_MISMATCH' and not row['motion_available_camera_local']
    assert row['motion']['time0_ns'] != row['sensor_interval_ns'][0]


@pytest.mark.parametrize('case',['duplicate_end','duplicate_first','orphan','different_manifest','float_time','backwards_clock','invalid_counts'])
def test_bad_contracts_rejected(case):
    reports,streams=fixture()
    if case=='duplicate_end':streams['rgb'].append(copy.deepcopy(streams['rgb'][1]));resequence(streams['rgb'])
    elif case=='duplicate_first':streams['rgb'].append(copy.deepcopy(streams['rgb'][2]));resequence(streams['rgb'])
    elif case=='orphan':streams['rgb'][2]['submission_id']=99
    elif case=='different_manifest':reports['thermal']['input_rows'][0]['thermal_left']['time_ns']+=1
    elif case=='float_time':streams['rgb'][1]['time0_ns']=float(streams['rgb'][1]['time0_ns'])
    elif case=='backwards_clock':streams['rgb'][-1]['observation_monotonic_ns']=0
    else:streams['rgb'][1]['retained_count']=13
    with pytest.raises(ValueError):build(reports,streams)


def test_final_run_outcome_is_not_a_past_predictor_feature():
    reports,streams=fixture();first=build(reports,streams)
    reports['rgb']['status']='FAILED';reports['rgb']['final_tracking_loss_count']=999
    second=build(reports,streams)
    assert first==second
    assert 'status' not in HEALTH_FIELDS and 'final_tracking_loss_count' not in HEALTH_FIELDS


def test_loader_rejects_incomplete_or_tampered_journal(tmp_path):
    base=tmp_path/'run';events=base/'arbiter-events';events.mkdir(parents=True)
    (base/'report.json').write_text('{}')
    (events/'events.jsonl').write_text('')
    summary=events/'summary.json';summary.write_text(json.dumps({'status':'COMPLETE','observer_errors':[]}))
    digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    proof=tmp_path/'proof.json';proof.write_text(json.dumps({'status':'PASS','events_sha256':digest(events/'events.jsonl'),'summary_sha256':digest(summary)}))
    assert load_verified_run(base,proof)==({},[])
    (events/'events.jsonl').write_text('{}\n')
    with pytest.raises(ValueError,match='hash mismatch'):load_verified_run(base,proof)
    summary.write_text(json.dumps({'status':'OBSERVER_ERRORS','observer_errors':['failure']}))
    with pytest.raises(ValueError,match='not complete'):load_verified_run(base,proof)
