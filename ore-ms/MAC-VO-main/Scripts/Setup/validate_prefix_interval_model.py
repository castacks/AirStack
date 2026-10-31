"""Independent prefix/rule/SE3 and matched-population audit for EXP089.

Does not import prediction, scheduling, history-policy or production model code.
"""
import argparse
from itertools import combinations
import json
from pathlib import Path
import numpy as np
from scipy.linalg import expm, logm
from scipy.spatial.transform import Rotation
from Scripts.Setup.validate_interval_model import independent_loss, sha, read

FIELDS=('selected_count','inbound_count','retained_count','occupied_3x3_cells','grid_count',
        'grid_positive_uu_vv_count','original_need_interp','motion_status')
POLICIES=('rgb_only','thermal_only','raw_advice','history')


def available(rows,pair,events,camera):
    a,b=[rows[j][camera+'_left']['time_ns'] for j in (pair,pair+1)]
    health=[e for e in events if e['kind']=='pair_end' and [e['time0_ns'],e['time1_ns']]==[a,b]]
    motions=[e for e in events if e['kind']=='motion_result_written' and e['first_write_for_submission'] and e['time1_ns']==b]
    h=health[0] if len(health)==1 else None;m=motions[0] if len(motions)==1 else None
    good=bool(h and m and all(h.get(f) is not None for f in FIELDS) and m['time0_ns']==a
        and h['original_need_interp'] is False and m['original_need_interp'] is False
        and h.get('execution_error') is None and m['derived_camera_delta_xyzw'] is not None)
    return good,h,m


def audit_payload(contract,timelines,predictions,scores,source):
    assert len(predictions)==len(scores['rows'])==217
    expected_keys={(w,i) for w,n in (('early',19),('middle',99),('day',99)) for i in range(n)}
    pred={(r['window'],r['pair_ordinal']):r for r in predictions};scored={(r['window'],r['pair_ordinal']):r for r in scores['rows']}
    assert set(pred)==set(scored)==expected_keys
    labels={(r['window'],r['camera'],r['pair_ordinal']):r for r in source}
    projected=checks=0;largest=0.;expected_losses={}
    for window,c in contract.items():
        events={cam:[] for cam in ('rgb','thermal')};receipts={cam:{} for cam in events};previous=-1;next_pair=0
        for sequence,entry in enumerate(timelines[window]):
            assert entry['sequence']==sequence and type(entry['controller_monotonic_ns']) is int and entry['controller_monotonic_ns']>=previous
            previous=entry['controller_monotonic_ns']
            if entry['kind']=='observer_event':
                camera=entry['camera'];e=entry['event'];assert e['sequence']==len(events[camera]) and e['modality']==camera
                events[camera].append(e);receipts[camera][e['sequence']]=previous
            elif entry['kind']=='decision':
                pair=entry['pair_ordinal'];assert pair==next_pair;next_pair+=1
                r=pred[window,pair];s=scored[window,pair]
                assert r['source_decision_sequence']==sequence and r['controller_deadline_ns']==previous
                assert r['observed_event_counts']=={cam:len(e) for cam,e in events.items()}
                assert r['common_interval_ns']==[c['input_rows'][j]['rgb_left']['time_ns'] for j in (pair,pair+1)]
                assert all(r[f] is False for f in ('admission_ready','physical_frame_verified','acquisition_clock_verified','measured_live_latency_verified','reference_used_for_prediction'))
                eligibility={};confirmation={};model_ok={};camera_losses={}
                for camera in events:
                    ok,h,m=available(c['input_rows'],pair,events[camera],camera);eligibility[camera]=ok
                    prior=available(c['input_rows'],pair-1,events[camera],camera)[0] if pair else False
                    confirmation[camera]=ok and prior
                    p=r['experts'][camera];assert p['available_by_prefix'] is ok
                    valid=bool(ok and m['stage']=='pair_processing');model_ok[camera]=valid
                    assert p['valid_model_projection'] is valid
                    if not valid:
                        assert p['delta_model'] is None and p['source'] is None;camera_losses[camera]=None
                        continue
                    receipt=max(receipts[camera][h['sequence']],receipts[camera][m['sequence']])
                    assert p['source']['received_controller_ns']==receipt<=previous
                    assert p['source']['motion_event_sequence']==m['sequence'] and p['source']['health_event_sequence']==h['sequence']
                    vector=m['derived_camera_delta_xyzw'];native=np.eye(4);native[:3,:3]=Rotation.from_quat(vector[3:]).as_matrix();native[:3,3]=vector[:3]
                    transform=np.array(c['nominal_body_from_camera'][camera]);native=transform@native@np.linalg.inv(transform)
                    np.testing.assert_allclose(native,p['source']['native_body_delta'],rtol=1e-8,atol=1e-9)
                    a,b=[c['input_rows'][j][camera+'_left']['time_ns'] for j in (pair,pair+1)];r0,r1=r['common_interval_ns']
                    assert p['native_interval_ns']==[a,b] and p['target_interval_ns']==[r0,r1]
                    assert p['duration_ratio']==(r1-r0)/(b-a) and p['horizon_ns']==b-a
                    assert max(a-r0,r0-b,a-r1,r1-b,0)<=b-a
                    assert not p['reference_used_for_prediction'] and not p['admission_ready']
                    model=expm(logm(native)*(r1-r0)/(b-a));assert np.max(np.abs(model.imag))<1e-9;model=model.real
                    largest=max(largest,float(np.max(np.abs(model-np.array(p['delta_model'])))))
                    np.testing.assert_allclose(model,p['delta_model'],rtol=1e-8,atol=1e-9);projected+=1
                    refs=labels[window,camera,pair]['reference_poses'];reference=np.linalg.solve(np.array(refs['common_start']),np.array(refs['common_end']))
                    camera_losses[camera]=independent_loss(model,reference)
                choose=lambda mask:'RGB' if mask['rgb'] else 'THERMAL' if mask['thermal'] else 'ABSTAIN'
                advice={'rgb_only':'RGB' if eligibility['rgb'] else 'ABSTAIN','thermal_only':'THERMAL' if eligibility['thermal'] else 'ABSTAIN',
                        'raw_advice':choose(eligibility),'history':choose(confirmation)}
                actions={name:action if action!='ABSTAIN' and model_ok[action.lower()] else 'ABSTAIN' for name,action in advice.items()}
                assert r['advice_actions']==advice and r['model_actions']==s['actions']==actions
                expected_losses[window,pair]={}
                for policy,action in actions.items():
                    value=None if action=='ABSTAIN' else camera_losses[action.lower()];expected_losses[window,pair][policy]=value
                    if value is None:assert s['losses'][policy] is None
                    else:
                        for metric,v in value.items():np.testing.assert_allclose(v,s['losses'][policy][metric],atol=2e-9,rtol=1e-7);checks+=1
    def stats(values,actual):
        nonlocal checks
        assert actual['count']==len(values)
        if not values:assert all(actual[k] is None for k in ('mean','median','rmse','max'));return
        a=np.array(values);expected={'mean':a.mean(),'median':np.median(a),'rmse':np.linalg.norm(a)/np.sqrt(len(a)),'max':a.max()}
        for k,v in expected.items():np.testing.assert_allclose(v,actual[k],atol=2e-9,rtol=1e-7);checks+=1
    assert set(scores['groups'])==set(contract)=={'early','middle','day'}
    for window,g in scores['groups'].items():
        assert set(g['policies'])==set(POLICIES)
        assert set(g['exact_matched_comparisons'])=={a+'__'+b for a,b in combinations(POLICIES,2)}
        pairs=sorted(i for w,i in expected_losses if w==window);assert g['decisions']==len(pairs)
        for policy,p in g['policies'].items():
            valid=[expected_losses[window,i][policy] for i in pairs if expected_losses[window,i][policy] is not None]
            assert p['available']==len(valid) and p['abstain']==len(pairs)-len(valid)
            for metric,st in p['conditional_losses'].items():stats([v[metric] for v in valid],st)
        for name,comparison in g['exact_matched_comparisons'].items():
            a,b=name.split('__');matched=[i for i in pairs if all(expected_losses[window,i][p] is not None for p in (a,b))]
            assert comparison['matched_pair_ordinals']==matched
            for policy,metrics in comparison['policies'].items():
                for metric,st in metrics.items():stats([expected_losses[window,i][policy][metric] for i in matched],st)
    assert projected==331
    return {'status':'PASS','decisions':217,'camera_slots':434,'independent_projections':projected,'scalar_checks':checks,'max_model_matrix_absolute_difference':largest}


def audit(root,out):
    protocol=read(out/'protocol.json');lock=read(out/'prediction-proof.json');pred=read(out/'predictions.json');scores=read(out/'scores.json')
    assert lock['predictions_sha256']==scores['predictions_sha256']==sha(out/'predictions.json')
    assert lock['protocol_sha256']==pred['protocol_sha256']==sha(out/'protocol.json')
    assert protocol['contract_sha256']==sha(out/'predictor-contract.json')
    assert protocol['declaration_sha256']==sha(out/'declaration.json')
    assert all(sha(root/p)==h for p,h in protocol['input_sha256'].items())
    contract=read(out/'predictor-contract.json');timelines={w:[json.loads(s) for s in (root/c['run_relative']/'timeline.jsonl').read_text().splitlines()] for w,c in contract.items()}
    source=[]
    for w,name in [('early','end-to-end-solver-control'),('middle','solver-collapse-replication'),('day','solver-day-replication')]:
        source += [dict(r,window=w) for r in read(root/'results'/name/'run-001/evaluation.json')['rows'] if r['arm']=='original']
    result=audit_payload(contract,timelines,pred['rows'],scores,source)
    result.update(predictions_sha256=sha(out/'predictions.json'),scores_sha256=sha(out/'scores.json'),auditor_sha256=sha(__file__))
    with (out/'validation.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();audit(a.root,a.output)
