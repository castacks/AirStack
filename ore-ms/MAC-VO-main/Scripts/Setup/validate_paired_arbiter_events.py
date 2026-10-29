"""Independent audit of the paired early-window contract, with original journals."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import statistics
from zoneinfo import ZoneInfo


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate(root, run, manifest):
    protocol=json.loads((run/'protocol.json').read_text())
    paired=json.loads((run/'paired-events.json').read_text())
    original=json.loads(manifest.read_text())['pairs'][:20]
    assert len(original)==20 and len(paired['pairs'])==19
    source_hashes={str(manifest):sha(manifest)}
    for path,digest in protocol['input_sha256'].items():assert sha(path)==digest
    plan=json.loads((root/'results/paired-arbiter-events/thermal-plan.json').read_text())
    prior=root/'results/arbiter-event-recorder/gpu-plan.json';prior_plan=json.loads(prior.read_text())
    assert sha(prior)==plan['rgb_existing_plan_sha256'] and prior_plan['image_id']==plan['image_id']
    for rel,digest in plan['source_sha256'].items():assert sha(root/'MAC-VO-main'/rel)==digest
    for rel,digest in plan['input_sha256'].items():assert sha(root.parent/rel)==digest
    reused=json.loads((root/'results/paired-arbiter-events/rgb-reuse-validation.json').read_text());assert reused['status']=='PASS'
    for rel,digest in reused['verified_prior_member_sha256'].items():assert sha(root.parent/rel)==digest
    expected_health=['selected_count','inbound_count','retained_count','occupied_3x3_cells','grid_count',
        'grid_positive_uu_vv_count','grid_trace_q10_q50_q90_px2','original_need_interp','execution_error','motion_status']
    totals={};checks=0
    for camera,path in [('rgb',protocol['rgb_run']),('thermal',protocol['thermal_run'])]:
        base=Path(path);report=json.loads((base/'report.json').read_text())
        assert report['input_rows']==original and report['manifest_start']==0 and report['completed_frames']==20
        assert report['modality']==camera and report['status']=='PASS_EXECUTION_ONLY'
        assert protocol['checkpoint_sha256'][camera]==report['checkpoint_sha256']
        journal=base/'arbiter-events/events.jsonl';events=[json.loads(line) for line in journal.read_text().splitlines()]
        source_hashes[str(journal)]=sha(journal);source_hashes[str(base/'report.json')]=sha(base/'report.json')
        summary=json.loads((base/'arbiter-events/summary.json').read_text());assert summary['status']=='COMPLETE' and summary['observer_errors']==[]
        ends=[e for e in events if e['kind']=='pair_end'];motions=[e for e in events if e['kind']=='motion_result_written' and e['first_write_for_submission']]
        assert len(ends)==len(motions)==19
        assert all(a['observation_monotonic_ns']<=b['observation_monotonic_ns'] for a,b in zip(events,events[1:]))
        for i,row in enumerate(paired['pairs']):
            assert row['pair_ordinal']==i
            expected=[original[j][camera+'_left']['time_ns'] for j in [i,i+1]]
            out=row['cameras'][camera];assert out['sensor_interval_ns']==expected
            assert all(type(t) is int for t in out['sensor_interval_ns'])
            end=ends[i];assert [end['time0_ns'],end['time1_ns']]==expected
            assert out['health']=={key:end[key] for key in expected_health}
            assert out['health_status']=='COMPLETE' and out['health_observation_monotonic_ns']==end['observation_monotonic_ns']
            match=[e for e in motions if [e['time0_ns'],e['time1_ns']]==expected];assert len(match)==1;motion=match[0]
            for key,value in out['motion'].items():assert motion[key]==value
            assert out['motion_status']=='INTERVAL_MATCH' and out['motion_interval_matches_pair'] is True
            assert out['motion_available_camera_local'] and not end['original_need_interp']
            assert out['pose_frame']==report['coordinate_frame'] and not out['body_transform_verified']
            checks+=len(expected_health)+len(out['motion'])+12
        totals[camera]={'pairs':len(ends),'first_motions':len(motions),
            'repeated_writes':sum(e['kind']=='motion_result_written' and not e['first_write_for_submission'] for e in events),
            'retained_count_min_median_max':[min(e['retained_count'] for e in ends),statistics.median(e['retained_count'] for e in ends),max(e['retained_count'] for e in ends)],
            'unsupported_pairs':sum(e['original_need_interp'] for e in ends)}
        expected_availability={'complete_health_pairs':19,'camera_local_motion_pairs':19,'repeated_writes_not_counted_as_new_motion':totals[camera]['repeated_writes']}
        assert paired['availability'][camera]==expected_availability
    offsets=[]
    for i,row in enumerate(paired['pairs']):
        delta=[original[j]['rgb_left']['time_ns']-original[j]['thermal_left']['time_ns'] for j in [i,i+1]]
        assert row['rgb_minus_thermal_endpoint_ns']==delta and row['exact_same_interval']==(delta==[0,0])
        assert all(type(x) is int for x in row['rgb_minus_thermal_endpoint_ns']);offsets.append(delta);checks+=5
    exact=sum(d==[0,0] for d in offsets);maximum=max(abs(x) for d in offsets for x in d)
    assert paired['alignment']=={'paired_source_rows_identical':True,'pair_count':19,'exact_same_intervals':exact,'max_abs_endpoint_offset_ns':maximum}
    assert paired['capture_mode']=='separate_replay_processes' and paired['eligibility']['health_schema_replay_complete']
    assert paired['eligibility']['pose_gate_training_ready'] is False and len(paired['eligibility']['reasons'])==4
    assert paired['feature_contract']['allowed_health_fields']==expected_health
    for path,digest in {**protocol['input_sha256'],**source_hashes}.items():assert sha(path)==digest
    result={'status':'PASS','created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),
        'script_sha256':sha(__file__),'source_sha256':source_hashes,'paired_report_sha256':sha(run/'paired-events.json'),
        'checked_fields':checks,'paired_intervals':19,'camera_summaries':totals,
        'offset_start_ns_min_median_max':[min(d[0] for d in offsets),statistics.median(d[0] for d in offsets),max(d[0] for d in offsets)],
        'offset_end_ns_min_median_max':[min(d[1] for d in offsets),statistics.median(d[1] for d in offsets),max(d[1] for d in offsets)],
        'scope':'exact source-manifest/journal pairing and contract arithmetic;not simultaneous availability, body-frame pose accuracy or gate performance'}
    (run/'validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='source_sha256'},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--run',type=Path,required=True);p.add_argument('--manifest',type=Path,required=True);a=p.parse_args();validate(a.root,a.run,a.manifest)
