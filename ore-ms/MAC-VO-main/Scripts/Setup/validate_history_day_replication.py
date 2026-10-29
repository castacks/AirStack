"""Independent bounded shared-event, nominal-proxy and history audits for EXP077.

Stage logic preserves EXP074-076 validators, with only the declared recording,
manifest and row-population binding changed. No prediction/score functions imported.
"""
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


def validate_shared(run, root):
    protocol=json.loads((run/'protocol.json').read_text())
    summary=json.loads((run/'summary.json').read_text())
    assert summary['status']=='PASS_REPLAY_ONLY'
    assert summary['timeline_sha256']==sha(run/'timeline.jsonl')
    assert protocol['mode']=='offline_serial_sensor_ordered' and protocol['group_lag']==1
    assert protocol['priority']==list(CAMERAS) and not protocol['policy_fitted']
    assert not protocol['body_transform_verified'] and not protocol['estimator_feedback'] and not protocol['wall_clock_paced']
    rows=protocol['input_rows'];n=len(rows)
    plan=json.loads((run.parent/'capture-plan.json').read_text())
    # EXP077 is deliberately locked to the previously examined separate day middle window.
    manifest=root/'results/long-salon/prepared-001/datasets/middle/manifest.json'
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



"""Independent source-journal reconstruction of conditional offline pose labels."""
import argparse
from collections import defaultdict
from decimal import Decimal
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from Scripts.Setup.td3_reference import load_reference, camera_reference


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def matrix(v):
    result=np.eye(4);result[:3,:3]=Rotation.from_quat(v[3:7]).as_matrix();result[:3,3]=v[:3];return result


def metrics(estimate,truth):
    # Direct residual components instead of composing the producer's loss matrix.
    rt=truth[:3,:3].T
    return {'translation_m':float(np.linalg.norm(rt@(estimate[:3,3]-truth[:3,3]))),
            'rotation_deg':float(np.degrees(Rotation.from_matrix(rt@estimate[:3,:3]).magnitude()))}


def stats(v):
    v=np.asarray(v,float)
    if not len(v):return {'count':0,'mean':None,'median':None,'rmse':None}
    return dict(count=len(v),mean=float(np.sum(v)/len(v)),median=float(np.median(v)),rmse=float(np.linalg.norm(v)/np.sqrt(len(v))))


def validate_proxy(root,run):
    protocol=json.loads((run/'protocol.json').read_text());report=json.loads((run/'pose-proxy.json').read_text())
    assert not protocol['physical_body_transform_verified'] and not protocol['independent_ground_truth']
    assert not protocol['causal_feature_eligible'] and not protocol['gate_fitted']
    assert protocol['reference_used_for_offline_transport']
    for rel,digest in protocol['input_sha256'].items():assert sha(root/rel)==digest,rel
    source=root/'data/tartandrive3/20251027/0_salon_so_day'
    d=source/'super_odometry/odometry';times=np.array([int(Decimal(s)*1000000000) for s in (d/'interp_timestamps.txt').read_text().splitlines()],np.int64)
    poses=np.loadtxt(d/'interp_data.txt');seconds=(times-times[0])/1e9
    slerp=Slerp(seconds,Rotation.from_quat(poses[:,3:7]));oldref,_=load_reference(source)
    def reference(t):
        assert type(t) is int and times[0]<=t<=times[-1]
        q=(t-int(times[0]))/1e9
        result=np.eye(4);result[:3,:3]=slerp([q]).as_matrix()[0]
        result[:3,3]=[np.interp(q,seconds,poses[:,i]) for i in range(3)]
        return result
    expected_rows=[];checked=0
    for window,rel,datarel in (('day-middle','history-day-replication','long-salon/prepared-001/datasets/middle'),):
        path=root/'results'/rel/'run-001';dataset=root/'results'/datarel
        timeline=[json.loads(s) for s in (path/'timeline.jsonl').read_text().splitlines()]
        groups=json.loads((path/'protocol.json').read_text())['input_rows']
        events={c:[json.loads(s) for s in (path/'experts'/c/'arbiter-events/events.jsonl').read_text().splitlines()] for c in ('rgb','thermal')}
        for decision in (r for r in timeline if r['kind']=='decision'):
            i=decision['pair_ordinal'];row=report['rows'][len(expected_rows)]
            assert row['window']==window and row['pair_ordinal']==i and row['source_decision_sequence']==decision['sequence'] and row['action']==decision['action']
            common=[groups[j]['rgb_left']['time_ns'] for j in (i,i+1)];r0,r1=map(reference,common);truth=np.linalg.solve(r0,r1)
            assert row['common_interval_ns']==common
            np.testing.assert_allclose(row['reference_delta'],truth,atol=1e-11,rtol=1e-11)
            for c in events:
                actual=row['experts'][c]
                if not decision['availability'][c]:assert actual is None;continue
                index=decision['evidence'][c]['motion_event_sequence'];event=events[c][index]
                assert actual['motion_event_sequence']==index<decision['observed_event_counts'][c]
                own=[groups[j][c+'_left']['time_ns'] for j in (i,i+1)];assert actual['sensor_interval_ns']==own
                a,b=map(reference,own)
                # Recover nominal E with the independently existing camera-reference path.
                camera=camera_reference(oldref,np.array([own[0]],np.int64),dataset,root/'data/calibration/yamaha.yaml',c)[0]
                e=np.linalg.solve(a,camera)
                # Independently normalize the rounded composite rotation by polar SVD.
                u,_,vt=np.linalg.svd(e[:3,:3]);e[:3,:3]=u@vt
                dc=matrix(event['derived_camera_delta_xyzw'])
                er,et=e[:3,:3],e[:3,3]
                db=np.eye(4);db[:3,:3]=er@dc[:3,:3]@er.T
                db[:3,3]=er@dc[:3,3]+et-db[:3,:3]@et
                aligned=np.linalg.solve(r0,a)@db@np.linalg.solve(b,r1)
                for name,value in [('nominal_body_from_camera',e),('own_body_delta',db),('common_body_delta',aligned)]:
                    np.testing.assert_allclose(actual[name],value,atol=1e-10,rtol=1e-10);checked+=16
                for name,pred,ref in [('own_loss',db,np.linalg.solve(a,b)),('common_loss',aligned,truth)]:
                    expected=metrics(pred,ref)
                    assert set(actual[name])==set(expected)
                    for key,value in expected.items():np.testing.assert_allclose(actual[name][key],value,atol=1e-9,rtol=1e-9);checked+=1
            expected_rows.append(row)
    assert len(report['rows'])==len(expected_rows)==99
    for window in ('day-middle',):
        rows=[r for r in expected_rows if r['window']==window];summary=report['windows'][window]
        assert summary['decisions']==len(rows)
        common=[r for r in rows if all(r['experts'][c] is not None for c in ('rgb','thermal'))]
        assert summary['common_eligible_count']==len(common)
        wins=defaultdict(int)
        for row in common:
            a,b=[row['experts'][c]['common_loss']['translation_m'] for c in ('rgb','thermal')]
            wins['tie' if abs(a-b)<=1e-12 else 'rgb' if a<b else 'thermal']+=1
        assert summary['common_translation_wins']=={c:wins[c] for c in ('rgb','thermal','tie')}
        def compare(actual,expected):
            assert actual['count']==expected['count']
            for k in ('mean','median','rmse'):
                if expected[k] is None:assert actual[k] is None
                else:np.testing.assert_allclose(actual[k],expected[k],atol=1e-11,rtol=1e-11)
        for c in ('rgb','thermal'):
            for k in ('translation_m','rotation_deg'):compare(summary['common_eligible_losses'][c][k],stats([r['experts'][c]['common_loss'][k] for r in common]))
        compare(summary['common_translation_oracle'],stats([min(r['experts'][c]['common_loss']['translation_m'] for c in ('rgb','thermal')) for r in common]))
        for policy in ('rgb_only','thermal_only','advisory'):
            values=[]
            for row in rows:
                c=row['action'].lower() if policy=='advisory' else policy.split('_')[0]
                if c in row['experts'] and row['experts'][c] is not None:values.append(row['experts'][c]['common_loss'])
            p=summary['policies'][policy];assert p['available']==len(values) and p['abstain']==len(rows)-len(values)
            for k in ('translation_m','rotation_deg'):compare(p[k],stats([v[k] for v in values]))
    result={'status':'PASS','rows':len(expected_rows),'numeric_components_checked':checked,
        'report_sha256':sha(run/'pose-proxy.json'),'protocol_sha256':sha(run/'protocol.json'),'validator_sha256':sha(Path(__file__)),
        'scope':'independent numeric/population check of nominal SLAM-proxy labels; no physical validation'}
    (run/'validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))



"""Independent prefix/history/population audit; imports neither policy nor scorer."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
from statistics import mean,median

C=('rgb','thermal')
F=('selected_count','inbound_count','retained_count','occupied_3x3_cells','grid_count',
   'grid_positive_uu_vv_count','grid_trace_q10_q50_q90_px2','original_need_interp','execution_error','motion_status')
P=('rgb_only','thermal_only','fixed_rgb_preference','history')


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def evidence(groups,i,c,events):
    times=[groups[j][c+'_left']['time_ns'] for j in (i,i+1)]
    healths=[e for e in events if e['kind']=='pair_end' and [e['time0_ns'],e['time1_ns']]==times]
    assert len(healths)<=1;h=healths[0] if healths else None
    motions=[e for e in events if e['kind']=='motion_result_written' and e['first_write_for_submission'] and e['time1_ns']==times[1]]
    m=motions[0] if len(motions)==1 else None
    complete=h is not None and all(h.get(k) is not None for k in F if k not in ('execution_error','grid_trace_q10_q50_q90_px2'))
    match=m is not None and m['time0_ns']==times[0]
    available=bool(match and h is not None and h.get('original_need_interp') is False and m.get('original_need_interp') is False and h.get('execution_error') is None and m['derived_camera_delta_xyzw'] is not None)
    record={'sensor_interval_ns':times,'health_status':'COMPLETE' if complete else 'MISSING_PAIR_END' if h is None else 'PARTIAL_HEALTH',
        'health':{k:h.get(k) if h else None for k in F},
        'motion_status':'AMBIGUOUS_FIRST_WRITES' if len(motions)>1 else 'NO_FIRST_WRITE' if m is None else 'INTERVAL_MATCH' if match else 'INTERVAL_MISMATCH',
        'motion_available_camera_local':available,'body_transform_verified':False,
        'health_event_sequence':h['sequence'] if h else None,'motion_event_sequence':m['sequence'] if m else None}
    return bool(complete and available),record


def close(actual,expected):
    if expected is None:assert actual is None
    else:assert type(actual) in (float,int) and math.isclose(actual,expected,rel_tol=1e-11,abs_tol=1e-12),(actual,expected)


def stat(actual,values):
    assert actual['count']==len(values)
    for k,v in {'mean':mean(values) if values else None,'median':median(values) if values else None,
                'rmse':math.sqrt(sum(v*v for v in values)/len(values)) if values else None}.items():close(actual[k],v)


def validate_history(root,run):
    protocol=json.loads((run/'protocol.json').read_text());pred=json.loads((run/'predictions.json').read_text());scores=json.loads((run/'scores.json').read_text())
    assert protocol['confirmation_intervals']==2 and not protocol['labels_used_for_prediction'] and not protocol['newer_than_target_intervals_used'] and not protocol['fitted']
    assert pred['protocol_sha256']==sha(run/'protocol.json') and scores['predictions_sha256']==sha(run/'predictions.json')
    for rel,digest in protocol['input_sha256'].items():assert sha(root/rel)==digest
    assert all('pose-proxy' not in rel for rel in protocol['input_sha256'])
    checked=0;expected=[]
    for window in protocol['windows']:
        path=root/window['run_relative'];groups=json.loads((path/'protocol.json').read_text())['input_rows'];prefix={c:[] for c in C}
        for line in (path/'timeline.jsonl').read_text().splitlines():
            r=json.loads(line)
            if r['kind']=='observer_event':prefix[r['camera']].append(r['event'])
            if r['kind']!='decision':continue
            i=r['pair_ordinal'];d=pred['decisions'][len(expected)]
            assert d['window']==window['window'] and d['pair_ordinal']==i
            assert d['source_decision_sequence']==r['sequence'] and d['source_deadline_monotonic_ns']==r['controller_monotonic_ns'] and d['boundary']==r['boundary']
            assert d['observed_event_counts']=={c:len(prefix[c]) for c in C}
            current={c:evidence(groups,i,c,prefix[c]) for c in C}
            previous={c:evidence(groups,i-1,c,prefix[c]) for c in C} if i else None
            available={c:current[c][0] for c in C};prior={c:previous[c][0] if previous else False for c in C}
            eligible={c:available[c] and prior[c] for c in C}
            assert d['base_availability']==available==r['availability']
            assert d['current_evidence']=={c:current[c][1] for c in C}==r['evidence']
            assert d['previous_availability']==prior and d['confirmed_availability']==eligible
            assert d['previous_evidence']==({c:previous[c][1] for c in C} if previous else None)
            assert d['previous_pair_ordinal']==(i-1 if i else None)
            action='RGB' if eligible['rgb'] else 'THERMAL' if eligible['thermal'] else 'ABSTAIN'
            reason='cold_start' if previous is None else 'both_confirmed_fixed_rgb_preference' if all(eligible.values()) else 'only_rgb_confirmed' if eligible['rgb'] else 'only_thermal_confirmed' if eligible['thermal'] else 'no_confirmed_expert'
            assert d['action']==action and d['reason']==reason and d['baseline_action']==r['action']
            assert d['confirmation_intervals']==2 and not d['reference_or_loss_used'] and not d['pose_gate_training_ready']
            expected.append(d);checked+=20
    assert len(expected)==len(pred['decisions'])==len(scores['rows'])==99
    labels=root/scores['label_directory_relative'];proof=json.loads((labels/'validation.json').read_text())
    assert proof['status']=='PASS' and scores['label_report_sha256']==sha(labels/'pose-proxy.json')==proof['report_sha256']
    lookup={(r['window'],r['pair_ordinal']):r for r in json.loads((labels/'pose-proxy.json').read_text())['rows']}
    for d,row in zip(expected,scores['rows']):
        label=lookup[(d['window'],d['pair_ordinal'])]
        for k in ('window','pair_ordinal','source_decision_sequence','action','baseline_action'):assert row[k]==d[k]
        assert label['source_decision_sequence']==d['source_decision_sequence']
        for policy in P:
            c=d['action'].lower() if policy=='history' else d['baseline_action'].lower() if policy=='fixed_rgb_preference' else policy.split('_')[0]
            expert=label['experts'].get(c);assert row['losses'][policy]==(expert['common_loss'] if expert else None)
            checked+=1
    for window in ('day-middle',):
        rows=[r for r in scores['rows'] if r['window']==window];s=scores['windows'][window]
        assert s['decisions']==len(rows) and s['actions']==dict(Counter(r['action'] for r in rows))
        assert s['action_changes']==dict(Counter(r['baseline_action']+'->'+r['action'] for r in rows if r['baseline_action']!=r['action']))
        for policy in P:
            available=[r for r in rows if r['losses'][policy] is not None];full=s['full_available'][policy]
            assert full['available']==len(available) and full['abstain']==len(rows)-len(available)
            assert full['pair_ordinals']==[r['pair_ordinal'] for r in available]
            for k in ('translation_m','rotation_deg'):stat(full[k],[r['losses'][policy][k] for r in available])
            common=[r for r in available if r['losses']['history'] is not None];matched=s['matched_with_history'][policy]
            assert matched['count']==len(common) and matched['pair_ordinals']==[r['pair_ordinal'] for r in common]
            for name,p in (('history','history'),('baseline',policy)):
                for k in ('translation_m','rotation_deg'):stat(matched[name][k],[r['losses'][p][k] for r in common])
    result={'status':'PASS','decisions':len(expected),'decision_and_loss_fields_checked':checked,
        'predictions_sha256':sha(run/'predictions.json'),'scores_sha256':sha(run/'scores.json'),
        'validator_sha256':sha(Path(__file__)),'scope':'independent causal prefix/history and conditional proxy population audit'}
    (run/'validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))



if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('stage',choices=('shared','proxy','history'))
    p.add_argument('--root',type=Path,required=True);p.add_argument('--run',type=Path,required=True)
    a=p.parse_args()
    if a.stage=='shared':validate_shared(a.run,a.root)
    elif a.stage=='proxy':validate_proxy(a.root,a.run)
    else:validate_history(a.root,a.run)
