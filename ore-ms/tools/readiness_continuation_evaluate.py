"""Post-lock transition delivery, latency and qualified proxy evaluation."""
import argparse
import collections
import json
from pathlib import Path
import sys
import numpy as np
from readiness_continuation_study import ROOT,OUT,PLAN,read,write,sha,verify
sys.path.insert(0,str(ROOT/'MAC-VO-main'))
from Scripts.Setup.transition_feedback import TransitionFeedback
from Scripts.Setup.readiness_feedback import ReadinessFeedback
from Scripts.Setup.causal_interval_model import project
from Scripts.Setup.td3_reference import matrices,load_reference
from Scripts.Setup.evaluate_interval_model import loss,summary
from closed_loop_sparse_evaluate import costs
from native_gap_evaluate import repeat_comparison

def camera_output(window,timeline,pair,camera):
    rows=window['rows'];native=[rows[j][camera+'_left']['time_ns'] for j in [pair,pair+1]]
    common=[rows[j]['rgb_left']['time_ns'] for j in [pair,pair+1]]
    deadline=next(e for e in timeline if e['kind']=='group_complete' and e['source']==pair+2)['controller_monotonic_ns']
    events=[e for e in timeline if e['kind']=='observer_event' and e['camera']==camera]
    health=[e for e in events if e['event']['kind']=='pair_end' and e['event']['time1_ns']==native[1]]
    writes=[e for e in events if e['event']['kind']=='motion_result_written' and e['event']['time1_ns']==native[1] and e['event']['first_write_for_submission']]
    h=health[0] if health else None;w=writes[0] if writes else None
    release=next(e for e in timeline if e['kind'] in ['release','skip_release','transport_drop'] and e['source']==pair+1 and e['camera']==camera)
    result={'camera':camera,'requested':release['policy_requested'],'delivered':release['delivered'],'native_interval_ns':native,'common_interval_ns':common,
            'actual_interval_ns':[h['event']['time0_ns'],h['event']['time1_ns']] if h else None,
            'deadline_ns':deadline,'health_event_sequence':h['event']['sequence'] if h else None,
            'motion_event_sequence':w['event']['sequence'] if w else None,'motion_receipt_ns':w['controller_monotonic_ns'] if w else None,
            'write_stage':w['event']['stage'] if w else None,'motion_trigger_source':w['trigger_source'] if w else None,'native_eligible':False,'model_eligible':False,'model':None}
    if not result['requested']:reason='not_requested'
    elif not result['delivered']:reason='synthetic_transport_unavailable'
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


def replay(w,base):
    timeline=[json.loads(l) for l in (base/'timeline.jsonl').read_text().splitlines()]
    meta=read(base/'summary.json')
    state=ReadinessFeedback(w['rows'],meta['mode']) if meta['readiness_enabled'] else TransitionFeedback(w['rows'],meta['mode'])
    decisions={};checks=0;ingress=None;transport=None
    for e in timeline:
        if e['kind']=='preselection':
            d=state.plan(e['pair'],e['decision_ns']);assert d=={k:e[k] for k in d};decisions[e['pair']]=e;checks+=1
        elif e['kind']=='release' and e['source']>0:
            assert state.declare(e['camera'],e['source'])==e['binding'];checks+=1
        elif e['kind']=='transport_unavailable':
            transport=state.delivery_failure(e['camera'],e['source'],e['controller_monotonic_ns'])
        elif e['kind']=='transport_ingress':
            assert transport=={k:e[k] for k in transport};checks+=1
        elif e['kind']=='observer_event':
            ingress=state.ingest(e['camera'],e['event'],e['controller_monotonic_ns'],e['trigger_source'])
        elif e['kind']=='ingress':
            assert ingress=={k:e[k] for k in ingress};checks+=1
    if meta['readiness_enabled']:
        assert state.session_history==meta['readiness_sessions'] and state.extra_used==meta['readiness_extra_used']
    return timeline,decisions,checks


def latencies(plan,arm,timeline,decisions,rows):
    result=[];N=plan['window']['frames']
    def event_summary(e,start,anchor):
        if e is None:return None
        source=e.get('source',e.get('pair',-1)+1)
        t=e.get('decision_ns',e.get('controller_monotonic_ns'))
        return {'source':source,'source_groups_after_anchor':source-start,'controller_ms_after_anchor':(t-anchor)/1e6}
    def delivery_summary(row,camera,start,anchor):
        if row is None:return None
        expert=row['experts'][camera];source=expert['motion_trigger_source']
        return {'target_ending_source':row['pair']+1,'delivery_source':source,
                'source_groups_after_anchor':source-start,
                'controller_ms_after_anchor':(expert['motion_receipt_ns']-anchor)/1e6}
    for index,o in enumerate(plan['outages']):
        start=o['start_source'];returned=o['end_source']+1
        horizon=plan['outages'][index+1]['start_source']-1 if index+1<len(plan['outages']) else N-3
        onset=decisions[start-1]['decision_ns'];return_clock=decisions[returned-1]['decision_ns']
        before=decisions[start-2]['action'];active=before.lower() if before!='ABSTAIN' else None
        for c in o['cameras']:
            failures=[e for e in timeline if e['kind']=='transport_unavailable' and e['camera']==c and start<=e['source']<=o['end_source']]
            first_failure=failures[0] if failures else None
            choices=[d for pair,d in decisions.items() if start<=pair+1<=horizon]
            switched=next((d for d in choices if first_failure and d['decision_ns']>first_failure['controller_monotonic_ns'] and d['action'] not in ['ABSTAIN',c.upper()]),None)
            delivered_alt=next((r for r in rows if first_failure and r['decision_ns']>first_failure['controller_monotonic_ns'] and r['pair']+1<=horizon and r['action'] not in ['ABSTAIN',c.upper()] and r['selected_native_eligible']),None)
            restored=[e for e in timeline if e['kind']=='release' and e['camera']==c and returned<=e['source']<=horizon]
            first_input=restored[0] if restored else None
            fresh=next((d for pair,d in decisions.items() if returned<=pair+1<=horizon and d['views'][c]['value'] is True and d['views'][c]['pair']>=returned-1),None)
            adjacent=next((r for r in rows if returned<=r['pair']+1<=horizon and r['experts'][c]['native_eligible']),None)
            selected_return=next((r for r in rows if returned<=r['pair']+1<=horizon and r['action']==c.upper() and r['selected_native_eligible']),None)
            result.append({'outage':o['name'],'camera':c,'start_source':start,'return_source':returned,'horizon_source':horizon,
                'active_before_outage':active==c,'before_action':before,
                'first_failure':event_summary(first_failure,start,onset),
                'first_alternate_preselection':event_summary(switched,start,onset) if active==c else None,
                'first_eligible_alternate':delivery_summary(delivered_alt,delivered_alt['action'].lower(),start,onset) if active==c and delivered_alt else None,
                'first_returned_input_request':event_summary(first_input,returned,return_clock),
                'first_fresh_true_preselection_view':event_summary(fresh,returned,return_clock),
                'fresh_view_native_pair':fresh['views'][c]['pair'] if fresh else None,
                'first_on_time_adjacent_return':delivery_summary(adjacent,c,returned,return_clock),
                'first_selected_adjacent_return':delivery_summary(selected_return,c,returned,return_clock),
                'note':'null is censored/not required; sticky healthy alternative does not require switching back'})
    return result


def predict():
    plan=verify();w=plan['window'];lock=read(OUT/'capture-lock.json');assert lock['plan_sha256']==sha(PLAN)
    for n,h in lock['sha256'].items():assert sha(ROOT/n)==h,n
    for n,h in plan['prior_control_sha256'].items():assert sha(ROOT/n)==h,n
    results={};rows=[];checks=0
    for arm in ['unperturbed']+plan['arms']:
        base=OUT/'captures'/arm;timeline,decisions,count=replay(w,base);checks+=count
        stats=costs(base)
        stats['requested_inputs']={c:sum(e['kind'] in ['release','transport_drop'] and e['camera']==c and e['source']>0 for e in timeline) for c in ['rgb','thermal']}
        stats['transport_drops']={c:sum(e['kind']=='transport_drop' and e['camera']==c for e in timeline) for c in ['rgb','thermal']}
        result={'cost':stats,'actions_all':dict(collections.Counter(d['action'] for d in decisions.values()))}
        if arm!='unperturbed':
            armrows=[]
            for pair in range(2,w['frames']-3):
                d=decisions[pair];experts={c:camera_output(w,timeline,pair,c) for c in ['rgb','thermal']}
                camera=d['action'].lower() if d['action']!='ABSTAIN' else None;e=experts.get(camera)
                row={'arm':arm,'pair':pair,'decision_ns':d['decision_ns'],'action':d['action'],'requests':d['requests'],
                     'experts':experts,'selected_native_eligible':bool(e and e['native_eligible']),
                     'selected_model_eligible':bool(e and e['model_eligible']),'opportunity':any(x['native_eligible'] for x in experts.values()),
                     'common_interval_ns':[w['rows'][j]['rgb_left']['time_ns'] for j in [pair,pair+1]]}
                armrows.append(row);rows.append(row)
            result['transition_latency']=latencies(plan,arm,timeline,decisions,armrows)
            result['joint_outage_actions']={str(source):decisions[source-1]['action'] for source in range(75,80)}
        meta=read(base/'summary.json');result['readiness_sessions']=meta['readiness_sessions'];result['readiness_extra_used']=meta['readiness_extra_used']
        results[arm]=result
    parity={c:repeat_comparison(OUT/'captures/unperturbed/experts'/c,ROOT/'results/availability-transitions/run-001/captures/unperturbed/experts'/c) for c in ['rgb','thermal']}
    assert len(rows)==285
    write(OUT/'predictions.json',{'rows':rows,'arms':results,'unperturbed_parity':parity,'prefix_checks':checks,
                                'references_read':False,'capture_lock_sha256':sha(OUT/'capture-lock.json')})
    write(OUT/'prediction-lock.json',{'predictions_sha256':sha(OUT/'predictions.json'),'source_sha256':sha(__file__),'references_read':False})
    print('LOCKED',len(rows),'core slots;',checks,'prefix checks; labels unread')


def score():
    plan=verify();lock=read(OUT/'prediction-lock.json');assert lock['predictions_sha256']==sha(OUT/'predictions.json') and lock['source_sha256']==sha(__file__)
    pred=read(OUT/'predictions.json');rows=pred['rows'];w=plan['window']
    for n,h in plan['scorer_only_reference_sha256'].items():assert sha(ROOT/n)==h,n
    reference=load_reference(ROOT/w['reference_dataset'])[0]
    for r in rows:
        poses=reference.at(np.asarray(r['common_interval_ns'],np.int64));target=np.linalg.solve(poses[0],poses[1]);r['reference_delta']=target.tolist()
        for e in r['experts'].values():e['proxy_loss']=loss(e['model']['delta_model'],target) if e['model_eligible'] else None
        r['selected_loss']=r['experts'][r['action'].lower()]['proxy_loss'] if r['selected_model_eligible'] else None
    groups={}
    for arm in plan['arms']:
        rr=[r for r in rows if r['arm']==arm];g=dict(pred['arms'][arm])
        g.update(slots=len(rr),eligible=sum(r['selected_native_eligible'] for r in rr),
                 unavailable_selected=sum(r['action']!='ABSTAIN' and not r['selected_native_eligible'] for r in rr),
                 uncommitted_while_opportunity=sum(r['action']=='ABSTAIN' and r['opportunity'] for r in rr),
                 uncommitted_none_available=sum(r['action']=='ABSTAIN' and not r['opportunity'] for r in rr),
                 actions=dict(collections.Counter(r['action'] for r in rr)),
                 expert_reasons={c:dict(collections.Counter(r['experts'][c]['reason'] for r in rr)) for c in ['rgb','thermal']})
        groups[arm]=g
    common=sorted(set.intersection(*[{r['pair'] for r in rows if r['arm']==arm and r['selected_model_eligible']} for arm in plan['arms']]))
    for arm,g in groups.items():
        rr=[r for r in rows if r['arm']==arm and r['pair'] in common]
        g['matched_proxy']={m:summary([r['selected_loss'][m] for r in rr]) for m in ['translation_m','rotation_deg']}
    candidate=groups['readiness_probe'];base=groups['bounded_probe'];full=groups['full_read_cache']
    criteria={'all_four_return_episodes':all(l['first_on_time_adjacent_return'] is not None for l in candidate['transition_latency']),
              'eligible_noninferior':candidate['eligible']>=base['eligible'],
              'unavailable_noninferior':candidate['unavailable_selected']<=base['unavailable_selected'],
              'uncommitted_opportunity_noninferior':candidate['uncommitted_while_opportunity']<=base['uncommitted_while_opportunity'],
              'fewer_pair_calls_than_full':candidate['cost']['total_pair_calls']<full['cost']['total_pair_calls'],
              'global_extra_cap':candidate['readiness_extra_used']<=20,
              'session_attempt_cap':all(x['attempts']<=4 for x in candidate['readiness_sessions']),
              'strict_unperturbed_parity':all(x['all_arrays_exact'] for x in pred['unperturbed_parity'].values())}
    write(OUT/'scores.json',{'scope':'explicit synthetic input availability; serial offline qualified first-write SLAM proxy',
         'groups':groups,'rows':rows,'primary_criteria':criteria,'primary_contract_passed':all(criteria.values()),'common_all_arms_slots':common,'unperturbed':pred['arms']['unperturbed'],
         'unperturbed_parity':pred['unperturbed_parity'],'prefix_checks':pred['prefix_checks'],
         'prediction_lock_sha256':sha(OUT/'prediction-lock.json'),'synthetic':True,'policy_adopted':False})
    print(json.dumps(groups,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['predict','score']);a=p.parse_args();globals()[a.action]()
