"""Independent synthetic environment, prefix, endpoint, projection and latency audit."""
import collections
import functools
import json
import math
import numpy as np
from scipy.linalg import expm,logm
from buffered_handover_study import ROOT,OUT,PLAN,read,write,sha
from native_gap_audit import pose,inverse,reference_at
CAMERAS=['rgb','thermal']

def prefix(plan,base,perturb):
    rows=plan['window']['rows'];N=len(rows);mode=read(base/'summary.json')['mode']
    log=[json.loads(l) for l in (base/'timeline.jsonl').read_text().splitlines()];active=None;checks=0;decision=None
    meta=read(base/'summary.json');ready=meta['readiness_enabled'];fixture=read(ROOT/plan['fixture'])['decisions'] if meta['frozen_request_fixture'] else None
    queries=[];live={c:None for c in CAMERAS};history=[];extra_used=0
    health={c:{} for c in CAMERAS};motion={c:{} for c in CAMERAS};resolved={c:{} for c in CAMERAS}
    requested={c:set() for c in CAMERAS};last={c:None for c in CAMERAS};burst={c:0 for c in CAMERAS};prior={c:0 for c in CAMERAS};bounds={c:{} for c in CAMERAS}
    endmaps={c:{r[c+'_left']['time_ns']:i for i,r in enumerate(rows)} for c in CAMERAS}
    for e in log:
        if e['kind']=='preselection':
            record=e;e=e['base_decision'] if ready else e
            i=e['pair'];now=e['decision_ns'];views={}
            for c in CAMERAS:
                if not resolved[c]:v={'status':'UNKNOWN','value':None,'last_value':None,'pair':None,'receipt_ns':None,'age_ns':None}
                else:
                    pair=max(resolved[c]);value,t=resolved[c][pair];age=now-t;assert age>=0;stale=age>=1_000_000_000
                    v={'status':'STALE' if stale else 'READY' if value else 'FAILED','value':None if stale else value,
                       'last_value':value,'pair':pair,'receipt_ns':t,'age_ns':age}
                views[c]=v
            assert views==e['views'];checks+=14
            if fixture is not None:
                declared=fixture[i]
                assert record['action']==declared['action'] and record['requests']==declared['requests'] and record['frozen_request_fixture']
                for c in declared['requests']:requested[c].add(i);last[c]=now
                active=declared['action'].lower() if declared['action']!='ABSTAIN' else None
                decision=record;checks+=3;continue
            good=[c for c in CAMERAS if views[c]['value'] is True]
            if mode.startswith('fixed_'):chosen=mode.removeprefix('fixed_');reason='fixed_camera'
            else:chosen=active if active in good else good[0] if good else None;reason='keep_fresh_active' if chosen==active and chosen is not None else 'switch_fresh_available' if chosen else 'no_fresh_available'
            active=chosen
            if i<2 or i>=N-3:req=set(CAMERAS);reason='common_bootstrap_or_guard'
            elif mode=='full_read_cache':req=set(CAMERAS);reason='full_read_control'
            elif chosen is None:req=set(CAMERAS);reason='refresh_unknown_stale_or_failed'
            else:
                req={chosen}
                if mode=='bounded_probe':
                    other=next(c for c in CAMERAS if c!=chosen);due=views[other]['status'] in ['UNKNOWN','STALE'] or last[other] is None or now-last[other]>=1_000_000_000
                    if due and burst[other]==0:burst[other]=2
                    if burst[other]>0:req.add(other)
            assert e['action']==(chosen.upper() if chosen else 'ABSTAIN') and e['reason']==reason and e['burst_before']==burst
            assert e['requests']==[c for c in CAMERAS if c in req];checks+=5
            if chosen and not mode.startswith('fixed_'):assert views[chosen]['value'] is True;checks+=1
            for c in req:requested[c].add(i);last[c]=now;burst[c]=max(0,burst[c]-1)
            assert e['burst_after']==burst;decision=record;checks+=1
            if ready:
                source=i+1;common=reason in ['common_bootstrap_or_guard','refresh_unknown_stale_or_failed']
                for c in CAMERAS:
                    session=live[c]
                    if session is None:continue
                    if session['status']=='pending':
                        status='became_active' if c==chosen else 'shared_refresh' if common else 'attempt_cap' if session['attempts']>=4 else 'extra_budget' if extra_used>=20 and c not in req else None
                        if status:session.update(status=status,closed_source=source)
                    if session['status']!='pending':live[c]=None
                for c in CAMERAS:
                    if not common and c!=chosen and c in req and e['burst_before'][c]==2 and live[c] is None:
                        session={'id':len(history),'camera':c,'start_source':source,'start_ns':now,'attempts':0,
                                 'requested_sources':[],'extra_sources':[],'status':'pending','evidence':None}
                        live[c]=session;history.append(session)
                extras=[]
                for c in CAMERAS:
                    session=live[c]
                    if session is not None and session['status']=='pending' and c not in req and session['attempts']<4 and extra_used<20:
                        extras.append(c);extra_used+=1;session['extra_sources'].append(source)
                        requested[c].add(i);last[c]=now
                actual=req|set(extras)
                for c in CAMERAS:
                    session=live[c]
                    if session is not None and session['status']=='pending' and c in actual:
                        session['attempts']+=1;session['requested_sources'].append(source);assert session['attempts']<=4
                assert record['requests']==[c for c in CAMERAS if c in actual]
                assert record['readiness_overlay']=={'extra_requests':extras,'extra_used':extra_used,'attempt_limit':4,'extra_limit':20,'sessions':live}
                checks+=8
        elif e['kind']=='buffer_query':
            c=e['camera'];group=e['delivery_group'];source=e['source'];queries.append(e)
            assert meta['catchup_enabled'] and len(queries)<=2 and sum(q['delivery_group']==group for q in queries)<=1
            assert source==group-1 and prior[c]==group-2 and decision['action']==c.upper()
            assert not (perturb and any(c in o['cameras'] and o['start_source']<=group<=o['end_source'] for o in plan['outages']))
            checks+=5
        elif e['kind']=='buffer_lookup':
            expected=not (perturb and any(e['camera'] in o['cameras'] and o['start_source']<=e['source']<=o['end_source'] for o in plan['outages']))
            assert e['retained']==expected;checks+=1
        elif e['kind']=='catchup_release':
            c=e['camera'];source=e['source'];pair=source-1
            assert source==prior[c]+1 and pair not in bounds[c] and pair not in health[c]
            requested[c].add(pair)
            bound={'pair':pair,'source_start':prior[c],'source_end':source,'sensor_interval_ns':[rows[j][c+'_left']['time_ns'] for j in [prior[c],source]]}
            assert e['binding']==bound;bounds[c][pair]=bound;prior[c]=source;checks+=4
        elif e['kind'] in ['release','skip_release','transport_drop']:
            c=e['camera'];source=e['source'];req=source==0 or source-1 in requested[c]
            unavailable=perturb and any(c in o['cameras'] and o['start_source']<=source<=o['end_source'] for o in plan['outages'])
            delivered=req and not unavailable
            assert e['policy_requested']==req and e['availability_observed']==req and e['delivered']==delivered
            assert e['kind']==('release' if delivered else 'transport_drop' if req else 'skip_release');checks+=4
            if source:
                assert decision['pair']==source-1 and decision['sequence']<e['sequence'];checks+=2
                if delivered:
                    bound={'pair':source-1,'source_start':prior[c],'source_end':source,'sensor_interval_ns':[rows[j][c+'_left']['time_ns'] for j in [prior[c],source]]}
                    assert bound==e['binding'];bounds[c][source-1]=bound;prior[c]=source;checks+=1
                else:assert e['binding'] is None;checks+=1
        elif e['kind']=='transport_unavailable':
            c=e['camera'];source=e['source'];pair=source-1
            assert pair in requested[c] and pair not in bounds[c] and pair not in health[c]
            health[c][pair]=(False,e['controller_monotonic_ns']);resolved[c][pair]=health[c][pair];checks+=3
            if ready and live[c] is not None and live[c]['status']=='pending':
                live[c].update(status='transport_failed',closed_source=source)
        elif e['kind']=='transport_ingress':
            assert prior[e['camera']]==e['native_previous_source'] and not e['native_health_fabricated'];checks+=2
        elif e['kind']=='observer_event':
            ev=e['event'];c=e['camera'];kind=ev['kind']
            if kind not in ['pair_end','motion_result_written'] or ev['stage']!='pair_processing':continue
            if kind=='motion_result_written' and not ev['first_write_for_submission']:continue
            pair=endmaps[c][ev['time1_ns']]-1;bound=bounds[c][pair]
            assert bound['sensor_interval_ns']==[ev['time0_ns'],ev['time1_ns']] and e['trigger_source']-1 in bounds[c];checks+=2
            if kind=='pair_end':assert pair not in health[c];health[c][pair]=(ev['execution_error'] is None and not ev['original_need_interp'],e['controller_monotonic_ns'])
            else:assert pair not in motion[c];motion[c][pair]=(not ev['original_need_interp'] and ev['derived_camera_delta_xyzw'] is not None,e['controller_monotonic_ns'])
            if pair in health[c]:
                value,t=health[c][pair]
                if not value:resolved[c][pair]=(False,t)
                elif pair in motion[c]:resolved[c][pair]=(motion[c][pair][0],max(t,motion[c][pair][1]))
            checks+=1
            if ready:
                session=live[c]
                valid=kind=='motion_result_written' and not ev['original_need_interp'] and ev['derived_camera_delta_xyzw'] is not None
                if session is not None and session['status']=='pending' and valid and pair in health[c] and health[c][pair][0] and bound['source_end']>=session['start_source'] and bound['source_start']+1==bound['source_end'] and e['trigger_source']==bound['source_end']+1 and e['delivery_group']<=bound['source_end']+1:
                    session['evidence']={'binding':bound,'trigger_source':e['trigger_source'],'receipt_ns':e['controller_monotonic_ns'],'event_sequence':ev['sequence']}
                    session.update(status='satisfied',closed_source=e['trigger_source'])
                    checks+=4
    meta=read(base/'summary.json')
    if ready:
        assert history==meta['readiness_sessions'] and extra_used==meta['readiness_extra_used']
        assert extra_used<=20 and all(session['attempts']<=4 for session in history);checks+=4
    assert len(queries)==meta['catchup_queries'];checks+=1
    return log,checks

@functools.lru_cache(None)
def reference(directory,times):
    r=reference_at(directory,times);return inverse(r[0])@r[1]

def main():
    plan=read(PLAN);scores=read(OUT/'scores.json');lock=read(OUT/'capture-lock.json');checks=0;w=plan['window']
    assert sha(PLAN)==sha(OUT/'plan.json')==lock['plan_sha256']
    for mapping in [plan['input_source_sha256'],plan['prior_control_sha256'],plan['scorer_only_reference_sha256'],lock['sha256']]:
        for n,h in mapping.items():assert sha(ROOT/n)==h,n;checks+=1
    predlock=read(OUT/'prediction-lock.json');assert predlock['predictions_sha256']==sha(OUT/'predictions.json') and predlock['source_sha256']==sha(ROOT/'tools/buffered_handover_evaluate.py')
    logs={}
    for arm in ['unperturbed']+plan['arms']:
        base=OUT/'captures'/arm;log,count=prefix(plan,base,arm!='unperturbed');logs[arm]=log;checks+=count
        for c in CAMERAS:
            path=base/'experts'/c;mapping=read(path/'source-mapping.json');indices=[e['source'] for e in log if e['kind'] in ['release','catchup_release'] and e['camera']==c]
            assert indices==mapping['source_indices'];assert mapping['input_rows']==[w['rows'][i] for i in indices];checks+=len(indices)*2
            cost=read(path/'native-cost.json');runs=[e for e in cost['native_phases'] if e['kind']=='run'];assert [e['source'] for e in runs]==indices
            assert sum(e['kind']=='frontend_pair' for e in cost['frontend_calls'])==len(indices)-1 and sum(e['kind']=='frontend_depth' for e in cost['frontend_calls'])==2;checks+=2
            with np.load(path/'trajectory.npz') as x:assert x['time_ns'].reshape(-1).tolist()==[w['rows'][i][c+'_left']['time_ns'] for i in indices];checks+=len(indices)
            for e in cost['native_phases']+cost['frontend_calls']:assert e['status']=='PASS' and e['duration_ns']==e['end_ns']-e['start_ns'];checks+=1
    for arm,log in logs.items():
        report=read(OUT/'captures'/arm/'retained-inputs.json');entries={c:{} for c in CAMERAS};total=peak=0;metadata={}
        assert len(report['arrivals'])==200
        expected_order=[(e['camera'],e['source']) for e in log if e['kind'] in ['release','skip_release','transport_drop']]
        assert [(r['camera'],r['source']) for r in report['arrivals']]==expected_order
        latest={c:-1 for c in CAMERAS}
        for row in report['arrivals']:
            c=row['camera'];source=row['source'];assert source==latest[c]+1;latest[c]=source;checks+=1
            expired=[j for j in entries[c] if j<source-1]
            assert expired==row['expired_sources']
            for j in expired:del entries[c][j]
            available=arm=='unperturbed' or not any(c in o['cameras'] and o['start_source']<=source<=o['end_source'] for o in plan['outages'])
            assert available==row['stored'];checks+=2
            if available:
                images={}
                for side in ['left','right']:
                    original=w['rows'][source][c+'_'+side];path=ROOT/w['dataset']/original['path']
                    images[side]={'path':original['path'],'sha256':sha(path),'bytes':path.stat().st_size,'time_ns':original['time_ns']}
                assert images==row['images'];entries[c][source]=sum(v['bytes'] for v in images.values());metadata[c,source]=images
                total+=entries[c][source];checks+=2
            assert sorted(entries[c])==row['retained_sources']
            peak=max(peak,sum(sum(items.values()) for items in entries.values()));checks+=1
        assert total==report['ingress_bytes'] and peak==report['peak_retained_bytes'] and report['buffer_io_ns']>=0;checks+=3
        retrieved=0
        for release in [e for e in log if e['kind']=='catchup_release']:
            assert release['encoded']==metadata[release['camera'],release['source']]
            retrieved+=sum(v['bytes'] for v in release['encoded'].values());checks+=1
        assert retrieved==report['retrieved_bytes'];checks+=1
    for r in scores['rows']:
        log=logs[r['arm']];i=r['pair'];assert 2<=i<=96
        d=next(e for e in log if e['kind']=='preselection' and e['pair']==i);assert r['action']==d['action'] and r['decision_ns']==d['decision_ns'];checks+=2
        deadline=next(e['controller_monotonic_ns'] for e in log if e['kind']=='group_complete' and e['source']==i+2)
        target=reference(w['reference_dataset'],tuple(r['common_interval_ns']));assert np.allclose(target,r['reference_delta'],atol=1e-8);checks+=1
        for c,e in r['experts'].items():
            ts=[w['rows'][j][c+'_left']['time_ns'] for j in [i,i+1]]
            events=[a for a in log if a['kind']=='observer_event' and a['camera']==c]
            hh=[a for a in events if a['event']['kind']=='pair_end' and a['event']['time1_ns']==ts[1]]
            ww=[a for a in events if a['event']['kind']=='motion_result_written' and a['event']['time1_ns']==ts[1] and a['event']['first_write_for_submission']]
            h=hh[0] if hh else None;m=ww[0] if ww else None
            rel=next(a for a in log if a['kind'] in ['release','skip_release','transport_drop'] and a['camera']==c and a['source']==i+1)
            historical=[a for a in log if a['kind']=='catchup_release' and a['camera']==c and a['source']==i+1]
            if historical:rel=historical[0]
            ok=bool(rel['kind'] in ['release','catchup_release'] and h and m and [h['event']['time0_ns'],h['event']['time1_ns']]==ts and
                h['event']['execution_error'] is None and not h['event']['original_need_interp'] and m['event']['stage']=='pair_processing' and
                h['controller_monotonic_ns']<=deadline and m['controller_monotonic_ns']<=deadline and
                [m['event']['time0_ns'],m['event']['time1_ns']]==ts and not m['event']['original_need_interp'] and m['event']['derived_camera_delta_xyzw'] is not None)
            assert ok==e['native_eligible'];checks+=1
            if not ok:assert e['proxy_loss'] is None;continue
            assert e['motion_trigger_source']==m['delivery_group'] and e['motion_invocation_source']==m['trigger_source'];checks+=2
            T=np.asarray(w['nominal_body_from_camera'][c]);body=T@pose(m['event']['derived_camera_delta_xyzw'])@inverse(T)
            assert np.allclose(body,e['body_delta'],atol=1e-10);checks+=1
            common=r['common_interval_ns'];outside=max(ts[0]-common[0],common[0]-ts[1],ts[0]-common[1],common[1]-ts[1],0)
            good=outside<=ts[1]-ts[0];assert good==e['model_eligible'];checks+=1
            if not good:continue
            model=expm(logm(body)*((common[1]-common[0])/(ts[1]-ts[0])));assert np.max(np.abs(np.imag(model)))<1e-9;model=np.real(model)
            assert np.allclose(model,e['model']['delta_model'],atol=1e-8,rtol=1e-8);checks+=1
            t=np.linalg.norm(model[:3,3]-target[:3,3]);rot=target[:3,:3].T@model[:3,:3]
            angle=math.degrees(math.atan2(np.linalg.norm([rot[2,1]-rot[1,2],rot[0,2]-rot[2,0],rot[1,0]-rot[0,1]])/2,(np.trace(rot)-1)/2))
            assert math.isclose(float(t),e['proxy_loss']['translation_m'],rel_tol=1e-7,abs_tol=1e-8) and math.isclose(angle,e['proxy_loss']['rotation_deg'],rel_tol=1e-6,abs_tol=1e-7);checks+=2
        chosen=r['experts'].get(r['action'].lower());assert r['selected_native_eligible']==bool(chosen and chosen['native_eligible'])
        assert r['opportunity']==any(e['native_eligible'] for e in r['experts'].values());assert r['selected_loss']==(chosen['proxy_loss'] if chosen and chosen['model_eligible'] else None);checks+=3
    both=sorted(set.intersection(*[{r['pair'] for r in scores['rows'] if r['arm']==arm and r['selected_model_eligible']} for arm in plan['arms']]))
    assert both==scores['common_all_arms_slots']
    for arm,g in scores['groups'].items():
        rr=[r for r in scores['rows'] if r['arm']==arm];log=logs[arm];dec={e['pair']+1:e for e in log if e['kind']=='preselection'}
        assert len(rr)==g['slots']==95 and g['eligible']==sum(r['selected_native_eligible'] for r in rr);checks+=2
        assert g['unavailable_selected']==sum(r['action']!='ABSTAIN' and not r['selected_native_eligible'] for r in rr);checks+=1
        calls=ns=0
        for c in CAMERAS:
            cost=read(OUT/'captures'/arm/'experts'/c/'native-cost.json')
            calls+=sum(e['kind']=='frontend_pair' for e in cost['frontend_calls']);ns+=sum(e['duration_ns'] for e in cost['native_phases'] if e['kind']=='run')
        assert calls==g['cost']['total_pair_calls'] and math.isclose(ns/1e9,g['cost']['total_run_seconds'],rel_tol=1e-12);checks+=2
        for metric in ['translation_m','rotation_deg']:
            vals=[r['selected_loss'][metric] for r in rr if r['pair'] in both];summary=g['matched_proxy'][metric];assert len(vals)==summary['count']
            if vals:assert math.isclose(math.sqrt(math.fsum(x*x for x in vals)/len(vals)),summary['rmse'],abs_tol=1e-12,rel_tol=1e-12)
            else:assert summary['rmse'] is None
            checks+=2
        for lat in g['transition_latency']:
            c=lat['camera'];start=lat['start_source'];ret=lat['return_source'];horizon=lat['horizon_source']
            active=dec[start-1]['action']==c.upper();assert active==lat['active_before_outage'];checks+=1
            failure=next((e for e in log if e['kind']=='transport_unavailable' and e['camera']==c and start<=e['source']<ret),None)
            candidates={
                'first_on_time_adjacent_return':next((r for r in rr if ret<=r['pair']+1<=horizon and r['experts'][c]['native_eligible']),None),
                'first_selected_adjacent_return':next((r for r in rr if ret<=r['pair']+1<=horizon and r['action']==c.upper() and r['selected_native_eligible']),None),
                'first_eligible_alternate':next((r for r in rr if active and failure and r['decision_ns']>failure['controller_monotonic_ns'] and r['pair']+1<=horizon and r['action'] not in ['ABSTAIN',c.upper()] and r['selected_native_eligible']),None)}
            for key,row in candidates.items():
                value=lat[key]
                if row is None:assert value is None;checks+=1;continue
                camera=row['action'].lower() if key=='first_eligible_alternate' else c
                expert=row['experts'][camera];anchor=start if key=='first_eligible_alternate' else ret
                assert value['target_ending_source']==row['pair']+1 and value['delivery_source']==expert['motion_trigger_source']
                assert value['source_groups_after_anchor']==expert['motion_trigger_source']-anchor
                assert math.isclose(value['controller_ms_after_anchor'],(expert['motion_receipt_ns']-dec[anchor]['decision_ns'])/1e6,abs_tol=1e-9);checks+=4
            firstinput=next((e for e in log if e['kind']=='release' and e['camera']==c and ret<=e['source']<=horizon),None)
            v=lat['first_returned_input_request'];assert (v is None)==(firstinput is None)
            if v:assert v['source']==firstinput['source'];checks+=1
            fresh=next((d for source,d in dec.items() if ret<=source<=horizon and d['views'][c]['value'] is True and d['views'][c]['pair']>=ret-1),None)
            v=lat['first_fresh_true_preselection_view'];assert (v is None)==(fresh is None)
            if v:assert v['source']==fresh['pair']+1 and lat['fresh_view_native_pair']==fresh['views'][c]['pair'];checks+=2
        for source,action in g['joint_outage_actions'].items():assert dec[int(source)]['action']==action;checks+=1
    c=scores['groups']['buffered_probe'];b=scores['groups']['bounded_probe'];f=scores['groups']['full_read_cache']
    off=next(r for r in scores['rows'] if r['arm']=='fixture_off' and r['pair']==50)
    on=next(r for r in scores['rows'] if r['arm']=='fixture_on' and r['pair']==50)
    criteria={'fixture_off_exact':all(x['all_arrays_exact'] for x in scores['fixture_parity'].values()),
              'fixture_off_gap_preserved':off['action']=='RGB' and off['experts']['rgb']['reason']=='native_gap_not_adjacent',
              'fixture_on_adjacent_target51':on['action']=='RGB' and on['selected_native_eligible'] and on['experts']['rgb']['motion_trigger_source']<=52,
              'all_four_return_episodes':all(l['first_on_time_adjacent_return'] is not None for l in c['transition_latency']),
              'eligible_noninferior':c['eligible']>=b['eligible'],
              'unavailable_noninferior':c['unavailable_selected']<=b['unavailable_selected'],
              'uncommitted_opportunity_noninferior':c['uncommitted_while_opportunity']<=b['uncommitted_while_opportunity'],
              'fewer_pair_calls_than_full':c['cost']['total_pair_calls']<f['cost']['total_pair_calls'],
              'global_extra_cap':c['readiness_extra_used']<=20,
              'session_attempt_cap':all(x['attempts']<=4 for x in c['readiness_sessions']),
              'catchup_query_cap':all(g['catchup_queries']<=2 for g in scores['groups'].values()),
              'catchup_group_cap':all(g['max_queries_per_group']<=1 for g in scores['groups'].values()),
              'strict_unperturbed_parity':all(x['all_arrays_exact'] for x in scores['unperturbed_parity'].values())}
    assert criteria==scores['primary_criteria'] and scores['primary_contract_passed']==all(criteria.values());checks+=9
    result={'status':'PASS_INDEPENDENT_RETAINED_INPUT_CATCHUP_PREFIX_ORIGINAL_DEADLINE_AUDIT','checks':checks,
            'core_rows':len(scores['rows']),'dual_timelines':len(logs),
            'scores_sha256':sha(OUT/'scores.json'),'auditor_sha256':sha(__file__)}
    write(OUT/'audit.json',result);print(json.dumps(result,indent=2))

if __name__=='__main__':main()
