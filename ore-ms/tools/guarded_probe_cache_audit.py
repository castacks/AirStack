"""Independent accepted-prefix reconstruction, hidden-event mutation and count checks."""
import json,math,statistics,collections
from guarded_probe_cache import ROOT,OUT,read,write,sha,verify,replay

def main():
 plan=verify();outputs=read(OUT/'locked-schedules.json')['windows_policies'];targets=read(OUT/'guarded-native-truth.json');scores=read(OUT/'scores.json');cases={s['tag']+'/'+c['window']:(s,c) for s in plan['sources'] for c in s['cases']};checks=0;mutation=0;age_violations=[]
 assert len(outputs)==108
 for o in outputs:
  s,c=cases[o['window']];raw=c['rows'];log=[json.loads(x) for x in (ROOT/s['base']/'captures'/c['case']/'on/timeline.jsonl').read_text().splitlines()];events={(e['camera'],e['event']['sequence']):e for e in log if e['kind']=='observer_event'};accepted=o['accepted'];hidden_ids={(e['camera'],e['event_sequence']) for e in o['hidden']};changed=[]
  for entry in log:
   if entry['kind']=='observer_event' and (entry['camera'],entry['event']['sequence']) in hidden_ids:
    entry=dict(entry,event=dict(entry['event']));e=entry['event']
    if e['kind']=='pair_end':e['original_need_interp']=not e['original_need_interp'];e['execution_error']='hidden_error'
    elif e['kind']=='motion_result_written':e['derived_camera_delta_xyzw']=None;e['original_need_interp']=True
   changed.append(entry)
  assert replay(c,changed,o['mode'])=={k:o[k] for k in ['decisions','accepted','hidden']};mutation+=1
  requested={cam:set() for cam in ['rgb','thermal']};active=None;last_request={cam:None for cam in requested};burst={cam:0 for cam in requested}
  for d in o['decisions']:
   i=d['pair'];now=d['decision_ns'];source=log[d['source_release_sequence']];assert source['kind']=='release' and source['frame']==i+1 and source['controller_monotonic_ns']==now;assert d['score_core']==(2<=i<=len(raw)-4);assert d['guard']==(i>=len(raw)-3)
   views={}
   for cam in requested:
    observations=[a for a in accepted if a['camera']==cam and a['receipt_ns']<now];health={};motion={}
    for a in observations:
     e=events[cam,a['event_sequence']]['event'];assert a['pair'] in requested[cam]
     if a['kind']=='pair_end':health[a['pair']]=(e.get('execution_error') is None and e['original_need_interp'] is False,a['receipt_ns'])
     else:
      assert a['trigger_pair'] in requested[cam] and e['stage']=='pair_processing';valid=e['time0_ns']==raw[a['pair']][cam+'_left']['time_ns'] and e.get('original_need_interp') is False and e.get('derived_camera_delta_xyzw') is not None;motion[a['pair']]=(valid,a['receipt_ns'])
    resolved={}
    for pair,(good,t) in health.items():
     if not good:resolved[pair]=(False,t)
     elif pair in motion:resolved[pair]=(motion[pair][0],max(t,motion[pair][1]))
    if not resolved:v={'status':'UNKNOWN','value':None,'last_value':None,'pair':None,'receipt_ns':None,'age_ns':None}
    else:
     pair=max(resolved);good,t=resolved[pair];age=now-t;stale=o['mode']!='unbounded_hold' and age>=1_000_000_000;v={'status':'STALE' if stale else 'READY' if good else 'FAILED','value':None if stale else good,'last_value':good,'pair':pair,'receipt_ns':t,'age_ns':age}
    assert v==d['views'][cam];views[cam]=v;checks+=7
   good=[cam for cam in requested if views[cam]['value'] is True]
   if o['mode'].startswith('fixed_'):choice=o['mode'].removeprefix('fixed_')
   else:choice=active if active in good else good[0] if good else None
   assert d['action']==(choice.upper() if choice else 'ABSTAIN');active=choice
   force=i<2 or i>=len(raw)-3
   if force or o['mode']=='full_read_cache' or choice is None:req=set(requested)
   else:
    req={choice}
    if o['mode']=='bounded_probe':
     other=next(cam for cam in requested if cam!=choice);due=views[other]['status'] in ['UNKNOWN','STALE'] or last_request[other] is None or now-last_request[other]>=1_000_000_000
     if due and burst[other]==0:burst[other]=2
     if burst[other]>0:req.add(other)
   assert set(d['requests'])==req and d['burst_before']==burst;checks+=3
   for cam in req:requested[cam].add(i);last_request[cam]=now;burst[cam]=max(0,burst[cam]-1)
   assert d['burst_after']==burst
   if o['mode']=='bounded_probe' and d['score_core'] and choice is not None:assert views[choice]['age_ns']<1_000_000_000
  for a in accepted:
   assert a['pair'] in requested[a['camera']]
   if a['kind']=='motion_result_written':assert a['trigger_pair'] in requested[a['camera']]
   checks+=1
 # All core native truth uses ordinary (not terminal drain) decision boundaries.
 proof=read(OUT/'guard-deadline-proof.json')['bindings']
 for p in proof:assert p['received_ns']<=p['deadline_ns'];checks+=1
 def check_stats(group,result):
  nonlocal checks
  count=collections.Counter();ages=[]
  for o in group:
   for d in o['decisions']:
    count['all_requests_including_guards']+=len(d['requests']);count['guard_requests' if d['guard'] else 'bootstrap_and_core_requests']+=len(d['requests'])
    if not d['score_core']:continue
    t=targets[o['window']+'/'+str(d['pair'])]['projected'];a=d['action'];count['core_pairs']+=1;count['core_requests']+=len(d['requests']);count['requested_some_eligible']+=int(any(t[c] for c in d['requests']));count['refresh_decisions']+=int(d['reason']=='refresh_unknown_stale_or_failed')
    if a=='ABSTAIN':count['uncommitted_while_available' if any(t.values()) else 'uncommitted_none_available']+=1
    else:count['selected_eligible' if t[a.lower()] else 'selected_unavailable']+=1
    for cam,v in d['views'].items():
     count['view_'+v['status']]+=1
     if v['age_ns'] is not None:ages.append(v['age_ns']/1e9)
     if cam in d['requests'] and a!='ABSTAIN' and cam!=a.lower():count['inactive_probe_requests']+=1
  for k,v in result.items():
   if k in ['resolved_age_median_s','resolved_age_max_s','request_fraction_of_dense_core','actual_inference_savings_measured']:continue
   assert count[k]==v;checks+=1
  assert result['resolved_age_median_s']==statistics.median(ages) and result['resolved_age_max_s']==max(ages);assert result['request_fraction_of_dense_core']==count['core_requests']/(2*count['core_pairs']);checks+=3
 for domain,result in scores['domains'].items():
  for mode,stats in result['policies'].items():check_stats([o for o in outputs if o['domain']==domain and o['mode']==mode],stats)
  a,b=[result['policies'][m] for m in ['bounded_probe','full_read_cache']];passed=a.get('selected_eligible',0)>=b.get('selected_eligible',0) and a.get('selected_unavailable',0)<=b.get('selected_unavailable',0) and a.get('uncommitted_while_available',0)<=b.get('uncommitted_while_available',0) and a['bootstrap_and_core_requests']<b['bootstrap_and_core_requests'];assert result['development_contract_passed']==passed
 for record,result in scores['records'].items():
  for mode,stats in result.items():check_stats([o for o in outputs if o['recording_id']==record and o['mode']==mode],stats)
 assert scores['primary_contract_passed']==all(d['development_contract_passed'] for d in scores['domains'].values());verify();write(OUT/'audit.json',{'status':'PASS_INDEPENDENT_GUARDED_MASK_CACHE_AUDIT','checks':checks,'actual_hidden_outcome_mutation_invariant_schedules':mutation,'window_policy_schedules':len(outputs),'guarded_deadline_write_bindings':len(proof),'scores_sha256':sha(OUT/'scores.json'),'auditor_sha256':sha(__file__),'native_sparse_counterfactual_validated':False});print('AUDITED',checks,'checks;',mutation,'hidden-event mutation invariances')
if __name__=='__main__':main()
