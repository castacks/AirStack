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


def validate(root,run):
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
    assert len(expected)==len(pred['decisions'])==len(scores['rows'])==118
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
    for window in ('early','middle'):
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
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--run',type=Path,required=True)
    a=p.parse_args();validate(a.root,a.run)
