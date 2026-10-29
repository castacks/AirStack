"""Independently audit the live timeline without importing scheduler/policy code."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

FIELDS=('selected_count','inbound_count','retained_count','occupied_3x3_cells',
        'grid_count','grid_positive_uu_vv_count','grid_trace_q10_q50_q90_px2',
        'original_need_interp','execution_error','motion_status')
CAMERAS=('rgb','thermal')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate(run, root):
    protocol=json.loads((run/'protocol.json').read_text())
    summary=json.loads((run/'summary.json').read_text())
    assert summary['status']=='PASS_REPLAY_ONLY'
    assert summary['timeline_sha256']==sha(run/'timeline.jsonl')
    assert protocol['mode']=='offline_serial_sensor_ordered' and protocol['group_lag']==1
    assert protocol['priority']==list(CAMERAS) and not protocol['policy_fitted']
    assert not protocol['body_transform_verified'] and not protocol['estimator_feedback'] and not protocol['wall_clock_paced']
    rows=protocol['input_rows'];n=len(rows)
    plan=json.loads((run.parent/'capture-plan.json').read_text())
    # EXP074 is deliberately locked to the previously examined long middle window.
    manifest=root/'results/long-night-validation/prepared-001/datasets/middle/manifest.json'
    original=json.loads(manifest.read_text())['pairs']
    assert rows==original[:100] and n==100
    assert plan['manifest_relative']==str(manifest.relative_to(root))
    assert plan['frames']==100 and plan['start']==0
    for rel,digest in plan['source_sha256'].items():assert sha(root/'MAC-VO-main'/rel)==digest,rel
    for rel,digest in plan['input_sha256'].items():assert sha(root/Path(rel).relative_to('ore-ms'))==digest,rel
    for rel,digest in plan['checkpoint_sha256'].items():assert sha(root/rel)==digest,rel
    for name,digest in protocol['source_sha256'].items():assert sha(root/'MAC-VO-main/Scripts/Setup'/name)==digest
    records=[json.loads(s) for s in (run/'timeline.jsonl').read_text().splitlines()]
    expected=sorted((r[c+'_left']['time_ns'],c,i) for i,r in enumerate(rows) for c in CAMERAS)
    releases=[];completed=set();active=None;drained=set();prefix={c:[] for c in CAMERAS};decisions=[]
    previous=-1;checked=0;last_group=-1
    for i,r in enumerate(records):
        assert r['sequence']==i and type(r['controller_monotonic_ns']) is int
        assert r['controller_monotonic_ns']>=previous;previous=r['controller_monotonic_ns']
        kind=r['kind'];c=r.get('camera')
        assert kind in {'ready','release','completed','drain_ready','drain_release','finished','observer_event','decision'}
        if kind=='release':
            assert active is None and not drained
            item=(r['sensor_time_ns'],c,r['frame'])
            assert type(item[0]) is int and item==expected[len(releases)]
            releases.append(item);active=(c,r['frame'])
        elif kind=='completed':
            assert active==(c,r['frame']) and active not in completed
            completed.add(active);active=None
            group=r['frame']
            if all((expert,group) in completed for expert in CAMERAS):last_group=group
        elif kind=='observer_event':
            event=r['event']
            assert event['modality']==c and event['sequence']==len(prefix[c])
            assert type(event['observation_monotonic_ns']) is int and event['observation_monotonic_ns']<=r['controller_monotonic_ns']
            assert active is not None and active[0]==c or c in drained
            prefix[c].append(event)
        elif kind=='drain_release':
            assert active is None and len(completed)==2*n and len(decisions)==n-1 and c not in drained
            drained.add(c)
        elif kind=='decision':
            pair=len(decisions)
            assert r['pair_ordinal']==pair and active is None and not drained
            expected_boundary={'kind':'group_complete','group':pair+2} if pair<n-2 else {'kind':'final_frame_before_drain','group':n-1}
            assert r['boundary']==expected_boundary and last_group==expected_boundary['group']
            assert r['observed_event_counts']=={c:len(prefix[c]) for c in CAMERAS}
            available={}
            for expert in CAMERAS:
                t0,t1=[rows[j][expert+'_left']['time_ns'] for j in (pair,pair+1)]
                ends=[e for e in prefix[expert] if e['kind']=='pair_end' and (e['time0_ns'],e['time1_ns'])==(t0,t1)]
                assert len(ends)<=1
                health=ends[0] if ends else None
                motions=[e for e in prefix[expert] if e['kind']=='motion_result_written' and e['first_write_for_submission'] and e['time1_ns']==t1]
                motion=motions[0] if len(motions)==1 else None
                complete=health is not None and all(health.get(k) is not None for k in FIELDS if k not in ('execution_error','grid_trace_q10_q50_q90_px2'))
                match=motion is not None and motion['time0_ns']==t0
                ok=bool(complete and match and health['original_need_interp'] is False and motion['original_need_interp'] is False and health['execution_error'] is None and motion['derived_camera_delta_xyzw'] is not None)
                status='AMBIGUOUS_FIRST_WRITES' if len(motions)>1 else 'NO_FIRST_WRITE' if motion is None else 'INTERVAL_MATCH' if match else 'INTERVAL_MISMATCH'
                evidence={'sensor_interval_ns':[t0,t1], 'health_status':'COMPLETE' if complete else 'MISSING_PAIR_END' if health is None else 'PARTIAL_HEALTH',
                    'health':{k:health.get(k) if health else None for k in FIELDS}, 'motion_status':status,
                    'motion_available_camera_local':bool(match and health is not None and health.get('original_need_interp') is False and motion.get('original_need_interp') is False and health.get('execution_error') is None and motion['derived_camera_delta_xyzw'] is not None),
                    'body_transform_verified':False,'health_event_sequence':health['sequence'] if health else None,
                    'motion_event_sequence':motion['sequence'] if motion else None}
                assert r['evidence'][expert]==evidence
                available[expert]=ok;checked+=len(evidence)+len(FIELDS)
            assert r['availability']==available
            action='RGB' if available['rgb'] else 'THERMAL' if available['thermal'] else 'ABSTAIN'
            reason='both_available_fixed_rgb_preference' if all(available.values()) else 'only_rgb_available' if available['rgb'] else 'only_thermal_available' if available['thermal'] else 'no_eligible_result'
            assert r['action']==action and r['reason']==reason
            assert r['camera_local_advice_only'] and not r['pose_gate_training_ready']
            decisions.append(r);checked+=6
    assert len(releases)==len(completed)==2*n and len(decisions)==n-1 and drained==set(CAMERAS)
    for expert in CAMERAS:
        base=run/'experts'/expert
        saved=[json.loads(line) for line in (base/'arbiter-events/events.jsonl').read_text().splitlines()]
        assert prefix[expert]==saved
        report=json.loads((base/'report.json').read_text())
        assert report['input_rows']==rows and report['status']=='PASS_EXECUTION_ONLY'
        proof=json.loads((run.parent/(expert+'-preservation.json')).read_text())
        assert proof['status']=='PASS' and proof['events_sha256']==sha(base/'arbiter-events/events.jsonl')
        assert proof['summary_sha256']==sha(base/'arbiter-events/summary.json')
    actions=dict(Counter(d['action'] for d in decisions))
    comparisons={c+'_only':{'available':sum(d['availability'][c] for d in decisions),
        'abstain':sum(not d['availability'][c] for d in decisions)} for c in CAMERAS}
    comparisons['advisory']={'available':sum(d['action']!='ABSTAIN' for d in decisions),
        'abstain':sum(d['action']=='ABSTAIN' for d in decisions)}
    unsupported_choices=sum(d['action']!='ABSTAIN' and d['evidence'][d['action'].lower()]['health']['original_need_interp'] is not False for d in decisions)
    assert unsupported_choices==0
    assert summary['actions']==actions and summary['decisions']==len(decisions)
    assert summary['streamed_events']=={c:len(prefix[c]) for c in CAMERAS}
    result={'status':'PASS','checked_decision_fields':checked,'scheduled_frames':len(releases),
        'decisions':len(decisions),'actions':actions,'prefixes_match_saved_journals':True,
        'fixed_deadline_availability':comparisons,'unsupported_choices':unsupported_choices,
        'timeline_sha256':sha(run/'timeline.jsonl'),'validator_sha256':sha(Path(__file__)),
        'scope':'independent offline deadline/prefix audit; numerical preservation checked separately; no VO gain claim'}
    (run/'validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True);p.add_argument('--root',type=Path,required=True)
    a=p.parse_args();validate(a.run,a.root)
