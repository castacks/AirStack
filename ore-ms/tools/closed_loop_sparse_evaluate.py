"""Own-prefix replay, ordinary adjacent delivery and separate proxy scoring."""
import argparse
import collections
import json
from pathlib import Path
import sys
import numpy as np
from closed_loop_sparse_study import ROOT,OUT,PLAN,read,write,sha,verify
sys.path.insert(0,str(ROOT/'MAC-VO-main'))
from Scripts.Setup.sparse_feedback import SparseFeedback
from Scripts.Setup.causal_interval_model import project
from Scripts.Setup.td3_reference import matrices,load_reference
from Scripts.Setup.evaluate_interval_model import loss,summary
from native_gap_evaluate import repeat_comparison


def replay(window,base):
    timeline=[json.loads(l) for l in (base/'timeline.jsonl').read_text().splitlines()]
    mode=read(base/'summary.json')['mode'];state=SparseFeedback(window['rows'],mode);decisions={};ingress=None;checks=0
    for e in timeline:
        if e['kind']=='preselection':
            predicted=state.plan(e['pair'],e['decision_ns'])
            assert predicted=={k:e[k] for k in predicted};decisions[e['pair']]=e;checks+=1
        elif e['kind']=='release' and e['source']>0:
            assert state.declare(e['camera'],e['source'])==e['binding'];checks+=1
        elif e['kind']=='observer_event':
            ingress=state.ingest(e['camera'],e['event'],e['controller_monotonic_ns'],e['trigger_source'])
        elif e['kind']=='ingress':
            assert ingress=={k:e[k] for k in ingress};checks+=1
    return timeline,decisions,checks


def camera_output(window,timeline,pair,camera):
    rows=window['rows'];native=[rows[j][camera+'_left']['time_ns'] for j in [pair,pair+1]]
    common=[rows[j]['rgb_left']['time_ns'] for j in [pair,pair+1]]
    deadline=next(e for e in timeline if e['kind']=='group_complete' and e['source']==pair+2)['controller_monotonic_ns']
    events=[e for e in timeline if e['kind']=='observer_event' and e['camera']==camera]
    health=[e for e in events if e['event']['kind']=='pair_end' and e['event']['time1_ns']==native[1]]
    writes=[e for e in events if e['event']['kind']=='motion_result_written' and e['event']['time1_ns']==native[1] and e['event']['first_write_for_submission']]
    h=health[0] if health else None;w=writes[0] if writes else None
    release=next(e for e in timeline if e['kind'] in ['release','skip_release'] and e['source']==pair+1 and e['camera']==camera)
    result={'camera':camera,'requested':release['kind']=='release','native_interval_ns':native,'common_interval_ns':common,
            'actual_interval_ns':[h['event']['time0_ns'],h['event']['time1_ns']] if h else None,
            'deadline_ns':deadline,'health_event_sequence':h['event']['sequence'] if h else None,
            'motion_event_sequence':w['event']['sequence'] if w else None,'motion_receipt_ns':w['controller_monotonic_ns'] if w else None,
            'write_stage':w['event']['stage'] if w else None,'native_eligible':False,'model_eligible':False,'model':None}
    if not result['requested']:reason='not_requested'
    elif h is None:reason='missing_health'
    elif result['actual_interval_ns']!=native:reason='native_gap_not_adjacent'
    elif h['event']['execution_error'] is not None or h['event']['original_need_interp'] is not False:reason='native_unsupported'
    elif w is None:reason='missing_motion'
    elif w['event']['stage']!='pair_processing':reason='non_processing_first_write'
    elif w['controller_monotonic_ns']>deadline or h['controller_monotonic_ns']>deadline:reason='after_ordinary_deadline'
    elif [w['event']['time0_ns'],w['event']['time1_ns']]!=native:reason='motion_interval_mismatch'
    elif w['event']['original_need_interp'] is not False or w['event']['derived_camera_delta_xyzw'] is None:reason='invalid_motion'
    else:
        reason='eligible_adjacent_native';result['native_eligible']=True
        T=np.asarray(window['nominal_body_from_camera'][camera],float)
        body=T@matrices(np.asarray([w['event']['derived_camera_delta_xyzw']],float))[0]@np.linalg.inv(T)
        result['body_delta']=body.tolist();result['model']=project(body,native,common)
        result['model_eligible']=result['model']['valid_model_projection']
    result['reason']=reason;return result


def costs(base):
    result={};total=0;seconds=0.
    for c in ['rgb','thermal']:
        path=base/'experts'/c;report=read(path/'report.json');assert report['status']=='PASS_EXECUTION_ONLY'
        mapping=read(path/'source-mapping.json');cost=read(path/'native-cost.json');runs=[r for r in cost['native_phases'] if r['kind']=='run']
        assert [r['source'] for r in runs]==mapping['source_indices']==cost['processed_source_indices']
        counts=collections.Counter(e['kind'] for e in cost['frontend_calls']);assert counts['frontend_depth']==2 and counts['frontend_pair']==len(runs)-1
        result[c]={'frames':len(runs),'pair_calls':counts['frontend_pair'],'depth_calls':counts['frontend_depth'],
                   'run_seconds':sum(r['duration_ns'] for r in runs)/1e9,
                   'termination_seconds':sum(r['duration_ns'] for r in cost['native_phases'] if r['kind']=='terminate')/1e9}
        total+=result[c]['pair_calls'];seconds+=result[c]['run_seconds']
    result['total_pair_calls']=total;result['total_run_seconds']=seconds;return result


def predict():
    plan=verify();lock=read(OUT/'capture-lock.json');assert lock['plan_sha256']==sha(PLAN)
    for n,h in lock['sha256'].items():assert sha(ROOT/n)==h,n
    rows=[];checks=0;cost={};parity={};old=read(ROOT/'experiments/native-gap-replay-2026-11-03.json')
    for n,h in plan['prior_dense_npz_sha256'].items():assert sha(ROOT/n)==h,n
    for w in plan['windows']:
        for arm in plan['arms']:
            base=OUT/'captures'/w['window']/arm;timeline,decisions,count=replay(w,base);checks+=count
            cost[w['window']+'/'+arm]=costs(base)
            # Last two source frames are guards; the last scored target has an ordinary next-group boundary.
            for pair in range(2,w['frames']-3):
                d=decisions[pair];experts={c:camera_output(w,timeline,pair,c) for c in ['rgb','thermal']}
                camera=d['action'].lower() if d['action']!='ABSTAIN' else None
                selected=experts[camera] if camera else None
                rows.append({'window':w['window'],'arm':arm,'pair':pair,'decision_ns':d['decision_ns'],
                             'action':d['action'],'requests':d['requests'],'experts':experts,
                             'opportunity':any(e['native_eligible'] for e in experts.values()),
                             'selected_native_eligible':bool(selected and selected['native_eligible']),
                             'selected_model_eligible':bool(selected and selected['model_eligible']),
                             'common_interval_ns':[w['rows'][j]['rgb_left']['time_ns'] for j in [pair,pair+1]]})
        base=OUT/'captures'/w['window']/'full_read_cache'
        for camera in ['rgb','thermal']:
            c=next(c for c in old['cases'] if c['window']==w['window'] and c['camera']==camera and c['arm']=='dense')
            parity[w['window']+'/'+camera+'/EXP118']=repeat_comparison(base/'experts'/camera,ROOT/c['capture']/camera)
            if w['window']=='early':parity['early/'+camera+'/repeat']=repeat_comparison(base/'experts'/camera,OUT/'captures/early/full-read-repeat/experts'/camera)
    assert len(rows)==410
    write(OUT/'predictions.json',{'rows':rows,'costs':cost,'strict_array_parity':parity,
                                'prefix_replay_checks':checks,'references_read':False,'capture_lock_sha256':sha(OUT/'capture-lock.json')})
    write(OUT/'prediction-lock.json',{'predictions_sha256':sha(OUT/'predictions.json'),'source_sha256':sha(__file__),'references_read':False})
    print('LOCKED',len(rows),'core source slots;',checks,'own-prefix replay checks; no reference reads')


def score():
    plan=verify();lock=read(OUT/'prediction-lock.json');assert lock['predictions_sha256']==sha(OUT/'predictions.json') and lock['source_sha256']==sha(__file__)
    predictions=read(OUT/'predictions.json');rows=predictions['rows'];references={}
    for n,h in plan['scorer_only_reference_sha256'].items():assert sha(ROOT/n)==h,n
    for w in plan['windows']:
        reference=load_reference(ROOT/w['reference_dataset'])[0]
        for row in [r for r in rows if r['window']==w['window']]:
            poses=reference.at(np.asarray(row['common_interval_ns'],np.int64));target=np.linalg.solve(poses[0],poses[1])
            row['reference_delta']=target.tolist();row['selected_loss']=None
            for e in row['experts'].values():e['proxy_loss']=loss(e['model']['delta_model'],target) if e['model_eligible'] else None
            if row['selected_model_eligible']:row['selected_loss']=row['experts'][row['action'].lower()]['proxy_loss']
    groups={}
    for w in plan['windows']:
        groups[w['window']]={}
        for arm in plan['arms']:
            rr=[r for r in rows if r['window']==w['window'] and r['arm']==arm]
            groups[w['window']][arm]={'slots':len(rr),'eligible':sum(r['selected_native_eligible'] for r in rr),
              'unavailable_selected':sum(r['action']!='ABSTAIN' and not r['selected_native_eligible'] for r in rr),
              'uncommitted_while_opportunity':sum(r['action']=='ABSTAIN' and r['opportunity'] for r in rr),
              'uncommitted_none_available':sum(r['action']=='ABSTAIN' and not r['opportunity'] for r in rr),
              'model_eligible':sum(r['selected_model_eligible'] for r in rr),
              'actions':dict(collections.Counter(r['action'] for r in rr)),
              'expert_eligible':{c:sum(r['experts'][c]['native_eligible'] for r in rr) for c in ['rgb','thermal']},
              'expert_reasons':{c:dict(collections.Counter(r['experts'][c]['reason'] for r in rr)) for c in ['rgb','thermal']},
              'cost':predictions['costs'][w['window']+'/'+arm]}
        d=groups[w['window']]['full_read_cache'];b=groups[w['window']]['bounded_probe']
        passed=b['eligible']>=d['eligible'] and b['unavailable_selected']<=d['unavailable_selected'] and b['uncommitted_while_opportunity']<=d['uncommitted_while_opportunity'] and b['cost']['total_pair_calls']<d['cost']['total_pair_calls']
        perarm={arm:{r['pair']:r for r in rows if r['window']==w['window'] and r['arm']==arm} for arm in plan['arms']}
        common=[i for i in perarm['full_read_cache'] if all(perarm[a][i]['selected_model_eligible'] for a in plan['arms'])]
        matched={arm:{metric:summary([perarm[arm][i]['selected_loss'][metric] for i in common]) for metric in ['translation_m','rotation_deg']} for arm in plan['arms']}
        groups[w['window']].update(development_contract_passed=passed,matched_common_slots=common,matched_proxy_metrics=matched)
    write(OUT/'scores.json',{'scope':'own-clock offline native feedback; nominal-body stationary-twist / recorded-SLAM proxy',
         'groups':groups,'rows':rows,'strict_array_parity':predictions['strict_array_parity'],
         'prefix_replay_checks':predictions['prefix_replay_checks'],'primary_contract_passed':all(g['development_contract_passed'] for g in groups.values()),
         'prediction_lock_sha256':sha(OUT/'prediction-lock.json'),'policy_adopted':False})
    print(json.dumps(groups,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['predict','score']);a=p.parse_args();globals()[a.action]()
