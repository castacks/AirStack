"""Independent frozen inventory/source audit; imports no validator or replay."""
from collections import Counter
from copy import deepcopy
import hashlib,json
from pathlib import Path
ROOT=Path('/root/AirStack/ore-ms');OUT=ROOT/'results/predecessor-state-validator/run-001'
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
checks=0

def equal(a,b):
    global checks
    checks+=1
    if isinstance(a,dict):
        assert isinstance(b,dict) and set(a)==set(b)
        for k in a:equal(a[k],b[k])
    elif isinstance(a,list):
        assert isinstance(b,list) and len(a)==len(b)
        for x,y in zip(a,b):equal(x,y)
    else:assert type(a) is type(b) and a==b,(a,b)


def reconstruct():
    inv=read(ROOT/'results/predecessor-evidence/run-001/inventory.json')
    contract=read(ROOT/'results/prefix-model-pipeline/run-001/predictor-contract.json');cache={};rows=[]
    for original in inv['rows']:
        sample=original['sample_id'];prefix,window,_=sample.split('/');camera=original['camera'];scope=f'{prefix}/{window}'
        if scope not in cache:
            p=(ROOT/contract[window]['run_relative']/'timeline.jsonl' if prefix=='EXP089' else ROOT/'results/third-recording-shared-capture/run-001/windows'/window/'shared/timeline.jsonl')
            cache[scope]=[json.loads(s) for s in p.read_text().splitlines()]
        timeline=cache[scope];decision=timeline[original['decision_sequence']];assert decision['kind']=='decision' and decision['pair_ordinal']==original['pair_ordinal']
        parent=original['parent'];child=original['motion']
        if original['candidate_status']!='EXACT_FIRST_WRITE':
            result={'status':'NOT_CHECKED','reason':'no_unique_exact_child','physical_motion_admitted':False};child_sequence=None
        else:
            assert child==timeline[child['sequence']] and child['camera']==camera and child['controller_monotonic_ns']<=decision['controller_monotonic_ns']
            child_sequence=child['sequence'];m=child['event']
            journal=[e for e in timeline[:decision['sequence']] if e['kind']=='observer_event' and e['camera']==camera]
            submissions=[e for e in journal if e['sequence']<child_sequence and e['event']['kind']=='optimization_submitted' and e['event']['submission_id']==m['submission_id']]
            assert len(submissions)==1
            for k in ('from_idx','frame_idx','time0_ns','time1_ns'):assert submissions[0]['event'][k]==m[k]
            assert sum(e['event']['kind']=='motion_result_written' and e['event']['submission_id']==m['submission_id'] and e['event']['first_write_for_submission'] is True for e in journal)==1
            result={'schema_version':1,'status':'REJECT_PREDECESSOR_EVIDENCE','reason':'missing_root_initialization' if parent['root_frame_zero'] else 'unsupported_parent_health',
                    'scope_id':scope,'camera':camera,'physical_motion_admitted':False,'root_evidence_used':False,'source':None}
            if not parent['root_frame_zero'] and parent['health_at_child_write']['status']=='SUPPORTED_HEALTH':
                h=parent['health_at_child_write']['evidence'];last=parent['last_observed_parent_write'];assert last is not None
                assert h==timeline[h['sequence']] and last==timeline[last['sequence']] and h['camera']==last['camera']==camera
                assert h['sequence']<child_sequence and last['sequence']<child_sequence
                assert h['event']['new_frame_idx']==last['event']['frame_idx']==m['from_idx']
                assert h['event']['time1_ns']==last['event']['time1_ns']==m['time0_ns']
                assert 0<=h['event']['time0_ns']<h['event']['time1_ns'] and h['event']['stage']=='pair_processing'
                assert h['event']['original_need_interp'] is False and h['event']['execution_error'] is None
                assert last['event']['committed_pose_camera_map_xyzw']==m['parent_pose_camera_map_xyzw']
                prior=[e for e in journal if e['sequence']<child_sequence and e['event']['kind']=='motion_result_written' and e['event']['frame_idx']==m['from_idx']]
                firsts=[e for e in prior if e['event']['first_write_for_submission'] is True];assert len(firsts)==1
                job=firsts[0]['event'];parent_sub=[e for e in journal if e['sequence']<firsts[0]['sequence'] and e['event']['kind']=='optimization_submitted' and e['event']['submission_id']==job['submission_id']]
                assert len(parent_sub)==1
                assert [e['event']['write_ordinal_for_submission'] for e in prior]==list(range(1,len(prior)+1))
                for e in prior:
                    assert e['event']['stage']=='pair_processing' and e['event']['original_need_interp'] is False
                    for k in ('submission_id','from_idx','frame_idx','time0_ns','time1_ns'):assert e['event'][k]==job[k]
                for k in ('from_idx','frame_idx','time0_ns','time1_ns'):assert parent_sub[0]['event'][k]==job[k]
                result.update(status='PASS_PREDECESSOR_EVIDENCE',reason='supported_parent_with_observed_pose',
                    source={'child_timeline_sequence':child_sequence,'child_event_sequence':m['sequence'],'parent_frame_idx':m['from_idx'],
                            'parent_time_ns':m['time0_ns'],'parent_health_sequence':h['sequence'],'parent_write_sequence':last['sequence'],'initialization_evidence_ref':None})
            elif not parent['root_frame_zero']:assert parent['health_at_child_write']['status']=='UNSUPPORTED_HEALTH'
        rows.append({'sample_id':sample,'recording_id':original['recording_id'],'camera':camera,'child_timeline_sequence':child_sequence,
                     'original_advice':decision['action'],'original_available':original['current_available'],'result':result})
    assert len(rows)==548 and len({(r['sample_id'],r['camera']) for r in rows})==548
    counts=lambda rs:dict(Counter(r['result']['reason'] for r in rs))
    return {'protocol_sha256':sha(OUT/'protocol.json'),'rows':rows,'counts':counts(rows),
            'recordings':{g:counts([r for r in rows if r['recording_id']==g]) for g in sorted({r['recording_id'] for r in rows})}}


def main():
    p=read(OUT/'protocol.json')
    for group in ('input_sha256','source_sha256','protected_source_sha256'):assert all(sha(ROOT/f)==h for f,h in p[group].items())
    assert p['declaration_sha256']==sha(OUT/'declaration.md') and p['root_initialization_supplied'] is False and p['labels_read'] is False
    expected=reconstruct();actual=read(OUT/'results.json');equal(expected,actual);base_checks=checks
    accepted=next(i for i,r in enumerate(actual['rows']) if r['result']['status']=='PASS_PREDECESSOR_EVIDENCE')
    root=next(i for i,r in enumerate(actual['rows']) if r['result']['reason']=='missing_root_initialization')
    recovery=next(i for i,r in enumerate(actual['rows']) if r['result']['reason']=='unsupported_parent_health')
    changes={
        'root_promoted':lambda x:x['rows'][root]['result'].__setitem__('status','PASS_PREDECESSOR_EVIDENCE'),
        'recovery_promoted':lambda x:x['rows'][recovery]['result'].__setitem__('reason','supported_parent_with_observed_pose'),
        'parent_write':lambda x:x['rows'][accepted]['result']['source'].__setitem__('parent_write_sequence',0),
        'future_health':lambda x:x['rows'][accepted]['result']['source'].__setitem__('parent_health_sequence',999999),
        'scope_alias':lambda x:x['rows'][accepted]['result'].__setitem__('scope_id','different/window'),
        'changed_advice':lambda x:x['rows'][accepted].__setitem__('original_advice','ABSTAIN'),
        'physical_admission':lambda x:x['rows'][accepted]['result'].__setitem__('physical_motion_admitted',True),
        'child_binding':lambda x:x['rows'][accepted].__setitem__('child_timeline_sequence',0),
        'missing_slot':lambda x:x['rows'].pop(),
        'missing_recording':lambda x:x['recordings'].pop(next(iter(x['recordings']))),
        'count':lambda x:x['counts'].__setitem__('supported_parent_with_observed_pose',439),
    }
    rejected=[]
    for name,mutate in changes.items():
        candidate=deepcopy(actual);mutate(candidate)
        try:equal(expected,candidate)
        except AssertionError:rejected.append(name)
        else:raise AssertionError('mutation accepted: '+name)
    equal(expected,{k:actual[k] for k in reversed(actual)})
    report={'status':'PASS_INDEPENDENT_PREDECESSOR_REPLAY','slots':548,'base_recursive_checks':base_checks,
            'counts':expected['counts'],'mutation_rejections':rejected,'positive_key_reorder':'PASS',
            'results_sha256':sha(OUT/'results.json'),'protocol_sha256':sha(OUT/'protocol.json'),'auditor_sha256':sha(Path(__file__))}
    with (OUT/'validation.json').open('x') as f:json.dump(report,f,indent=2);f.write('\n')
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
