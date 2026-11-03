"""Independent chronological pending-state resolver, future truth and population audit."""
from collections import Counter
import json,copy,math
import numpy as np
from early_health_execution import ROOT,OUT,read,write,sha,verify
from early_health_study import sources,METHODS
from Scripts.Setup.early_availability import snapshot

def chronological(raw,i,timeline,epoch,cam):
 slots={(raw[k][cam+'_left']['time_ns'],raw[k+1][cam+'_left']['time_ns']):k for k in range(i+1)};health={};motion={};resolved={}
 for entry in timeline:
  if entry['kind']!='observer_event' or entry['camera']!=cam or entry['controller_monotonic_ns']>epoch:continue
  e=entry['event']
  if e['kind']=='pair_end' and (e['time0_ns'],e['time1_ns']) in slots:health[slots[e['time0_ns'],e['time1_ns']]]=e
  if e['kind']=='motion_result_written' and e['first_write_for_submission']:motion.setdefault(e['time1_ns'],[]).append(e)
  for k,h in health.items():
   if h.get('execution_error') is not None or h['original_need_interp'] is True:resolved[k]=False;continue
   if h['original_need_interp'] is not False:continue
   ms=motion.get(raw[k+1][cam+'_left']['time_ns'],[])
   if not ms:continue
   valid=len(ms)==1
   if valid:
    m=ms[0];valid=m['time0_ns']==raw[k][cam+'_left']['time_ns'] and m['time1_ns']==raw[k+1][cam+'_left']['time_ns'] and m['stage']=='pair_processing' and m.get('original_need_interp') is False and m.get('derived_camera_delta_xyzw') is not None
   resolved[k]=valid
 return resolved[max(resolved)] if resolved else None,max(resolved) if resolved else None

def main():
 plan=verify();rows=read(OUT/'locked-availability-predictions.json')['rows'];truth=read(OUT/'availability-truth.json');scores=read(OUT/'availability-scores.json');lookup,controls=sources(plan);checks=0;cache={};boundary=[]
 assert len(rows)==1955 and len({r['forecast_id'] for r in rows})==1955
 for r in rows:
  case=lookup[r['window']];raw=case['case']['rows'];timeline=case['timeline'];i=r['origin_pair'];j=r['target_pair'];s=r['early_prefix'];key=(r['window'],i)
  if key not in cache:
   health={}
   for cam in ['rgb','thermal']:
    endings=[e for e in timeline if e['kind']=='observer_event' and e['camera']==cam and e['event']['kind']=='pair_end' and [e['event']['time0_ns'],e['event']['time1_ns']]==[raw[i][cam+'_left']['time_ns'],raw[i+1][cam+'_left']['time_ns']]];assert len(endings)==1;health[cam]=endings[0]
   epoch=max(e['controller_monotonic_ns'] for e in health.values());assert epoch==s['epoch_ns']
   for cam in health:
    value,k=chronological(raw,i,timeline,epoch,cam);assert value is s['states'][cam]['resolved'];source=s['provenance'][cam]['resolved'];assert (source['pair'] if source else None)==k
    h=health[cam]['event'];supported=h.get('execution_error') is None and h['original_need_interp'] is False;assert supported is s['states'][cam]['health'];checks+=3
   future=[]
   for entry in timeline:
    if entry['kind']=='observer_event' and entry['controller_monotonic_ns']>epoch:
     entry=dict(entry,event=dict(entry['event']));e=entry['event']
     if e['kind']=='pair_end':e['original_need_interp']=not e['original_need_interp'];e['execution_error']='injected_future_error'
     elif e['kind']=='motion_result_written':e['derived_camera_delta_xyzw']=None;e['original_need_interp']=True
    future.append(entry)
   changed=snapshot(raw,i,future);assert changed=={k:v for k,v in s.items() if k!='offline_reconstruction_cpu_s'};checks+=1;cache[key]=True
  releases=[e for e in timeline if e['kind']=='release' and e['controller_monotonic_ns']<=s['epoch_ns']];assert max(e['frame'] for e in releases)==r['latest_released_at_early_prefix']<j;release=min(e['controller_monotonic_ns'] for e in releases if e['frame']==i+1);assert abs((s['epoch_ns']-release)/1e9-r['early_pair_end_delay_s'])<1e-12;checks+=2
  predicted=r['availability_predictions']
  for name,field in [('early_health','health'),('early_confirmed','confirmed'),('early_resolved','resolved')]:
   values={cam:s['states'][cam][field] for cam in ['rgb','thermal']};assert values==predicted[name];choice='RGB' if values['rgb'] is True else 'THERMAL' if values['thermal'] is True else 'ABSTAIN';assert choice==r['actions'][name];checks+=2
  original_decision,=[e for e in timeline if e['kind']=='decision' and e['pair_ordinal']==j];p=case['prefix'][case['case']['window'],j]
  assert truth[r['forecast_id']]['camera_local']==original_decision['availability'];assert truth[r['forecast_id']]['projected']=={c:bool(p['experts'][c]['available_by_prefix'] and p['experts'][c]['valid_model_projection']) for c in ['rgb','thermal']};checks+=4
  for cam in ['rgb','thermal']:
   health,=[e['event'] for e in timeline if e['kind']=='observer_event' and e['camera']==cam and e['event']['kind']=='pair_end' and [e['event']['time0_ns'],e['event']['time1_ns']]==[raw[j][cam+'_left']['time_ns'],raw[j+1][cam+'_left']['time_ns']]];assert truth[r['forecast_id']]['tracking'][cam]==(health['original_need_interp'] is False and health['execution_error'] is None);checks+=1
  action=r['actions']['early_resolved'];bad=action!='ABSTAIN' and not truth[r['forecast_id']]['projected'][action.lower()]
  if bad:
   cam=action.lower();health,=[e['event'] for e in timeline if e['kind']=='observer_event' and e['camera']==cam and e['event']['kind']=='pair_end' and [e['event']['time0_ns'],e['event']['time1_ns']]==[raw[j][cam+'_left']['time_ns'],raw[j+1][cam+'_left']['time_ns']]];writes=[e for e in timeline if e['kind']=='observer_event' and e['camera']==cam and e['event']['kind']=='motion_result_written' and e['event']['first_write_for_submission'] and e['event']['time1_ns']==raw[j+1][cam+'_left']['time_ns']]
   boundary.append({'forecast_id':r['forecast_id'],'horizon_ns':r['horizon_ns'],'window':r['window'],'target_pair':j,'terminal_target_pair':j==len(raw)-2,'tracking_supported':not health['original_need_interp'],'future_first_writes':[{'stage':e['event']['stage'],'received_ns':e['controller_monotonic_ns'],'after_target_deadline':e['controller_monotonic_ns']>original_decision['controller_monotonic_ns']} for e in writes]})
 def groupcheck(result,subset):
  nonlocal checks
  for target,output in result['targets'].items():
   for method,stat in output['selection'].items():
    ids={k:[] for k in ['selected_available','selected_unavailable','abstain_while_available','abstain_none_available']}
    for r in subset:
     t=truth[r['forecast_id']][target];a=r['actions'][method]
     if a=='ABSTAIN':k='abstain_while_available' if any(t.values()) else 'abstain_none_available'
     else:k='selected_available' if t[a.lower()] else 'selected_unavailable'
     ids[k].append(r['forecast_id'])
    assert ids==stat['ids'];assert all(stat[k]==len(v) for k,v in ids.items());assert sum(map(len,ids.values()))==stat['forecasts']==len(subset);assert stat['unique_targets']==len({r['target_id'] for r in subset});checks+=7
   for method,cameras in output['confusion'].items():
    for cam,stat in cameras.items():
     counts=Counter()
     for r in subset:
      p=r['availability_predictions'][method][cam];t=truth[r['forecast_id']][target][cam]
      if p is None:counts['unknown']+=1;counts['unknown_but_available']+=int(t)
      else:counts[['TN','FP','FN','TP'][2*int(t)+int(p)]]+=1
     assert stat=={k:counts[k] for k in stat};checks+=6
  delays=[r['early_pair_end_delay_s'] for r in subset];assert abs(result['timing']['early_pair_end_delay_median_s']-__import__('statistics').median(delays))<1e-12;assert abs(result['timing']['early_pair_end_delay_p95_s']-float(np.quantile(delays,.95)))<1e-12;assert result['timing']['positive_early_reconstruction_budgets']==sum(r['remaining_early_reconstruction_budget_s']>0 for r in subset);checks+=3
 for h,result in scores['horizons'].items():
  selected=[r for r in rows if str(r['horizon_ns'])==h];groupcheck(result['aggregate'],selected)
  for domain,g in result['domains'].items():
   groupcheck(g,[r for r in selected if r['domain']==domain]);a,b=[g['targets']['projected']['selection'][m] for m in ['early_resolved','late_information']];assert g['availability_noninferiority_passed']==(a['selected_available']>=b['selected_available'] and a['selected_unavailable']<=b['selected_unavailable'] and a['abstentions']<=b['abstentions'])
  for record,g in result['records'].items():groupcheck(g,[r for r in selected if r['recording_id']==record])
 verify();write(OUT/'availability-audit.json',{'status':'PASS_INDEPENDENT_EARLY_AVAILABILITY_AUDIT','checks':checks,'forecasts':len(rows),'unique_origin_prefixes':len(cache),'actual_future_event_mutation_invariant':True,'truth_sha256':sha(OUT/'availability-truth.json'),'scores_sha256':sha(OUT/'availability-scores.json'),'auditor_sha256':sha(__file__),'pose_error_labels_read':False});write(OUT/'terminal-boundary-posthoc.json',{'scope':'posthoc metadata stratification; primary counts/predictions unchanged','unavailable_early_resolved_forecasts':boundary,'all_are_terminal':all(r['terminal_target_pair'] for r in boundary),'all_have_supported_tracking':all(r['tracking_supported'] for r in boundary),'all_first_writes_after_target_deadline':all(r['future_first_writes'] and all(w['after_target_deadline'] for w in r['future_first_writes']) for r in boundary),'primary_scores_sha256':sha(OUT/'availability-scores.json')});print('AUDITED',checks,'checks;',len(boundary),'selected-unavailable forecasts across two horizons; terminal',all(r['terminal_target_pair'] for r in boundary))
if __name__=='__main__':main()
