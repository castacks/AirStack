"""Independent prefix, interval/deadline, matrix-log projection and score audit."""
import collections
import functools
import json
import math
from pathlib import Path
import sys
import numpy as np
from scipy.linalg import expm,logm
from closed_loop_sparse_study import ROOT,OUT,PLAN,read,write,sha
from native_gap_audit import pose,inverse,reference_at
CAMERAS=['rgb','thermal']


def prefix(window,base):
    timeline=[json.loads(l) for l in (base/'timeline.jsonl').read_text().splitlines()]
    mode=read(base/'summary.json')['mode'];rows=window['rows'];N=len(rows);active=None;checks=0
    health={c:{} for c in CAMERAS};motions={c:{} for c in CAMERAS};resolved={c:{} for c in CAMERAS}
    requests={c:set() for c in CAMERAS};last={c:None for c in CAMERAS};burst={c:0 for c in CAMERAS};prior={c:0 for c in CAMERAS};bounds={c:{} for c in CAMERAS}
    endmaps={c:{r[c+'_left']['time_ns']:i for i,r in enumerate(rows)} for c in CAMERAS};decision=None
    for e in timeline:
        if e['kind']=='preselection':
            i=e['pair'];now=e['decision_ns'];views={}
            for c in CAMERAS:
                if not resolved[c]:v={'status':'UNKNOWN','value':None,'last_value':None,'pair':None,'receipt_ns':None,'age_ns':None}
                else:
                    pair=max(resolved[c]);good,t=resolved[c][pair];age=now-t;assert age>=0;stale=age>=1_000_000_000
                    v={'status':'STALE' if stale else 'READY' if good else 'FAILED','value':None if stale else good,
                       'last_value':good,'pair':pair,'receipt_ns':t,'age_ns':age}
                views[c]=v
            assert views==e['views'];checks+=14
            good=[c for c in CAMERAS if views[c]['value'] is True];chosen=active if active in good else good[0] if good else None
            reason='keep_fresh_active' if chosen==active and chosen is not None else 'switch_fresh_available' if chosen else 'no_fresh_available';active=chosen
            if i<2 or i>=N-3:req=set(CAMERAS);reason='common_bootstrap_or_guard'
            elif mode=='full_read_cache':req=set(CAMERAS);reason='full_read_control'
            elif chosen is None:req=set(CAMERAS);reason='refresh_unknown_stale_or_failed'
            else:
                req={chosen};other=next(c for c in CAMERAS if c!=chosen)
                due=views[other]['status'] in ['UNKNOWN','STALE'] or last[other] is None or now-last[other]>=1_000_000_000
                if due and burst[other]==0:burst[other]=2
                if burst[other]>0:req.add(other)
            assert e['action']==(chosen.upper() if chosen else 'ABSTAIN') and e['reason']==reason and e['burst_before']==burst
            assert e['requests']==[c for c in CAMERAS if c in req];checks+=5
            for c in req:requests[c].add(i);last[c]=now;burst[c]=max(0,burst[c]-1)
            assert e['burst_after']==burst;decision=e;checks+=2
        elif e['kind'] in ['release','skip_release']:
            source=e['source'];c=e['camera'];requested=source==0 or source-1 in requests[c]
            assert requested==(e['kind']=='release');checks+=1
            if source:
                assert decision['pair']==source-1 and decision['sequence']<e['sequence'];checks+=2
                if requested:
                    bound={'pair':source-1,'source_start':prior[c],'source_end':source,
                           'sensor_interval_ns':[rows[j][c+'_left']['time_ns'] for j in [prior[c],source]]}
                    assert bound==e['binding'];bounds[c][source-1]=bound;prior[c]=source;checks+=1
        elif e['kind']=='observer_event':
            ev=e['event'];c=e['camera'];kind=ev['kind']
            if kind not in ['pair_end','motion_result_written'] or ev['stage']!='pair_processing':continue
            if kind=='motion_result_written' and not ev['first_write_for_submission']:continue
            end=endmaps[c][ev['time1_ns']];pair=end-1;bound=bounds[c][pair]
            assert bound['sensor_interval_ns']==[ev['time0_ns'],ev['time1_ns']]
            assert e['trigger_source']-1 in bounds[c];receipt=e['controller_monotonic_ns'];checks+=2
            if kind=='pair_end':
                value=ev['execution_error'] is None and not ev['original_need_interp'];assert pair not in health[c];health[c][pair]=(value,receipt)
            else:
                value=not ev['original_need_interp'] and ev['derived_camera_delta_xyzw'] is not None;assert pair not in motions[c];motions[c][pair]=(value,receipt)
            if pair in health[c]:
                good,t=health[c][pair]
                if not good:resolved[c][pair]=(False,t)
                elif pair in motions[c]:resolved[c][pair]=(motions[c][pair][0],max(t,motions[c][pair][1]))
            checks+=2
    return timeline,checks


@functools.lru_cache(None)
def independent_reference(directory,times):
    r=reference_at(directory,times);return inverse(r[0])@r[1]


def main():
    plan=read(PLAN);scores=read(OUT/'scores.json');pred=read(OUT/'predictions.json');lock=read(OUT/'capture-lock.json');checks=0
    assert sha(PLAN)==sha(OUT/'plan.json') and lock['plan_sha256']==sha(PLAN)
    for mapping in [plan['input_source_sha256'],plan['scorer_only_reference_sha256'],plan['prior_dense_npz_sha256'],lock['sha256']]:
        for n,h in mapping.items():assert sha(ROOT/n)==h,n;checks+=1
    predictionlock=read(OUT/'prediction-lock.json');assert predictionlock['source_sha256']==sha(ROOT/'tools/closed_loop_sparse_evaluate.py') and predictionlock['predictions_sha256']==sha(OUT/'predictions.json')
    timelines={};windows={w['window']:w for w in plan['windows']}
    for w in plan['windows']:
        for arm in plan['arms']+(['full-read-repeat'] if w['window']=='early' else []):
            base=OUT/'captures'/w['window']/arm;timeline,count=prefix(w,base);timelines[w['window'],arm]=timeline;checks+=count
            for c in CAMERAS:
                p=base/'experts'/c;mapping=read(p/'source-mapping.json');indices=[e['source'] for e in timeline if e['kind']=='release' and e['camera']==c]
                assert indices==mapping['source_indices'];assert mapping['input_rows']==[w['rows'][i] for i in indices];checks+=len(indices)*2
                with np.load(p/'trajectory.npz') as x:assert x['time_ns'].reshape(-1).tolist()==[w['rows'][i][c+'_left']['time_ns'] for i in indices]
                cost=read(p/'native-cost.json');runs=[e for e in cost['native_phases'] if e['kind']=='run'];assert [e['source'] for e in runs]==indices;checks+=len(indices)
                assert sum(e['kind']=='frontend_pair' for e in cost['frontend_calls'])==len(indices)-1
                assert sum(e['kind']=='frontend_depth' for e in cost['frontend_calls'])==2;checks+=2
                for e in cost['frontend_calls']+cost['native_phases']:assert e['status']=='PASS' and e['duration_ns']==e['end_ns']-e['start_ns'];checks+=1
    for r in scores['rows']:
        w=windows[r['window']];i=r['pair'];timeline=timelines[r['window'],r['arm']]
        assert 2<=i<=w['frames']-4
        d=next(e for e in timeline if e['kind']=='preselection' and e['pair']==i);assert d['action']==r['action'] and d['decision_ns']==r['decision_ns'];checks+=2
        deadline=next(e['controller_monotonic_ns'] for e in timeline if e['kind']=='group_complete' and e['source']==i+2)
        target=independent_reference(w['reference_dataset'],tuple(r['common_interval_ns']));assert np.allclose(target,r['reference_delta'],atol=1e-8,rtol=1e-8);checks+=1
        for c,e in r['experts'].items():
            ts=[w['rows'][j][c+'_left']['time_ns'] for j in [i,i+1]]
            event=[a for a in timeline if a['kind']=='observer_event' and a['camera']==c]
            hh=[a for a in event if a['event']['kind']=='pair_end' and a['event']['time1_ns']==ts[1]]
            ww=[a for a in event if a['event']['kind']=='motion_result_written' and a['event']['time1_ns']==ts[1] and a['event']['first_write_for_submission']]
            h=hh[0] if hh else None;writeevent=ww[0] if ww else None
            ok=bool(h and writeevent and [h['event']['time0_ns'],h['event']['time1_ns']]==ts and
                    h['event']['execution_error'] is None and not h['event']['original_need_interp'] and
                    writeevent['event']['stage']=='pair_processing' and writeevent['controller_monotonic_ns']<=deadline and
                    h['controller_monotonic_ns']<=deadline and [writeevent['event']['time0_ns'],writeevent['event']['time1_ns']]==ts and
                    not writeevent['event']['original_need_interp'] and writeevent['event']['derived_camera_delta_xyzw'] is not None)
            assert ok==e['native_eligible'] and e['deadline_ns']==deadline;checks+=2
            if not ok:assert e['proxy_loss'] is None;continue
            T=np.asarray(w['nominal_body_from_camera'][c]);body=T@pose(writeevent['event']['derived_camera_delta_xyzw'])@inverse(T)
            assert np.allclose(body,e['body_delta'],atol=1e-10);checks+=1
            common=r['common_interval_ns'];outside=max(ts[0]-common[0],common[0]-ts[1],ts[0]-common[1],common[1]-ts[1],0)
            model_ok=outside<=ts[1]-ts[0];assert model_ok==e['model_eligible'];checks+=1
            if not model_ok:continue
            ratio=(common[1]-common[0])/(ts[1]-ts[0]);model=expm(logm(body)*ratio)
            assert np.max(np.abs(np.imag(model)))<1e-9;model=np.real(model)
            assert np.allclose(model,e['model']['delta_model'],atol=1e-8,rtol=1e-8);checks+=1
            translation=float(np.linalg.norm(model[:3,3]-target[:3,3]));rotation=target[:3,:3].T@model[:3,:3]
            sine=np.linalg.norm([rotation[2,1]-rotation[1,2],rotation[0,2]-rotation[2,0],rotation[1,0]-rotation[0,1]])/2
            angle=math.degrees(math.atan2(sine,(np.trace(rotation)-1)/2))
            assert math.isclose(translation,e['proxy_loss']['translation_m'],abs_tol=1e-8,rel_tol=1e-7)
            assert math.isclose(angle,e['proxy_loss']['rotation_deg'],abs_tol=1e-7,rel_tol=1e-6);checks+=2
        chosen=r['experts'].get(r['action'].lower());assert r['selected_native_eligible']==bool(chosen and chosen['native_eligible']);assert r['opportunity']==any(e['native_eligible'] for e in r['experts'].values());checks+=2
        assert r['selected_loss']==(chosen['proxy_loss'] if chosen and chosen['model_eligible'] else None);checks+=1
    for window,g in scores['groups'].items():
        arms={a:[r for r in scores['rows'] if r['window']==window and r['arm']==a] for a in plan['arms']}
        for arm,rr in arms.items():
            summary=g[arm];assert len(rr)==windows[window]['frames']-5==summary['slots']
            assert summary['eligible']==sum(r['selected_native_eligible'] for r in rr)
            assert summary['unavailable_selected']==sum(r['action']!='ABSTAIN' and not r['selected_native_eligible'] for r in rr)
            assert summary['uncommitted_while_opportunity']==sum(r['action']=='ABSTAIN' and r['opportunity'] for r in rr);checks+=4
            totalpairs=totalns=0
            for c in CAMERAS:
                cost=read(OUT/'captures'/window/arm/'experts'/c/'native-cost.json')
                totalpairs+=sum(e['kind']=='frontend_pair' for e in cost['frontend_calls']);totalns+=sum(e['duration_ns'] for e in cost['native_phases'] if e['kind']=='run')
            assert totalpairs==summary['cost']['total_pair_calls'] and math.isclose(totalns/1e9,summary['cost']['total_run_seconds'],rel_tol=1e-12);checks+=2
        d=g['full_read_cache'];b=g['bounded_probe'];passed=b['eligible']>=d['eligible'] and b['unavailable_selected']<=d['unavailable_selected'] and b['uncommitted_while_opportunity']<=d['uncommitted_while_opportunity'] and b['cost']['total_pair_calls']<d['cost']['total_pair_calls']
        assert passed==g['development_contract_passed'];checks+=1
        both=sorted(set(r['pair'] for r in arms['full_read_cache'] if r['selected_model_eligible'])&set(r['pair'] for r in arms['bounded_probe'] if r['selected_model_eligible']))
        assert both==g['matched_common_slots'];checks+=1
        for arm,rr in arms.items():
            for metric in ['translation_m','rotation_deg']:
                values=[r['selected_loss'][metric] for r in rr if r['pair'] in both];summary=g['matched_proxy_metrics'][arm][metric];assert summary['count']==len(values)
                if values:assert math.isclose(math.sqrt(math.fsum(x*x for x in values)/len(values)),summary['rmse'],rel_tol=1e-12,abs_tol=1e-12)
                else:assert summary['rmse'] is None
                checks+=2
    assert scores['primary_contract_passed']==all(g['development_contract_passed'] for g in scores['groups'].values())
    result={'status':'PASS_INDEPENDENT_OWN_PREFIX_ADJACENT_DEADLINE_PROJECTION_SCORE_AUDIT',
            'checks':checks,'core_rows':len(scores['rows']),'dual_timelines':len(timelines),
            'scores_sha256':sha(OUT/'scores.json'),'auditor_sha256':sha(__file__),
            'independent_reference_helper_sha256':sha(ROOT/'tools/native_gap_audit.py')}
    write(OUT/'audit.json',result);print(json.dumps(result,indent=2))

if __name__=='__main__':main()
