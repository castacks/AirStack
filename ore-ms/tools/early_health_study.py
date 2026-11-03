"""Fixed no-fit availability persistence; lock actions before future-state scoring."""
import argparse,collections,time,statistics
from early_health_execution import ROOT,OUT,read,write,sha,verify
import sys
sys.path.insert(0,str(ROOT/'MAC-VO-main'))
from Scripts.Setup.early_availability import snapshot,choose,supported
METHODS=['early_health','early_confirmed','early_resolved','late_eligible','late_information','rgb_only','thermal_only']

def sources(plan):
 out={};controls={};prefix={}
 for s in plan['sources']:
  for r in read(ROOT/s['controls'])['rows']:controls[s['tag']+'/'+r['sample_id']]=r
  p={(r['window'],r['pair_ordinal']):r for r in read(ROOT/s['base']/s['prefix'])['rows']}
  for c in s['cases']:
   timeline=[__import__('json').loads(e) for e in (ROOT/s['base']/'captures'/c['case']/'on/timeline.jsonl').read_text().splitlines()];out[s['tag']+'/'+c['window']]={'case':c,'timeline':timeline,'prefix':p,'tag':s['tag']}
 return out,controls

def predict():
 plan=verify();bindings=read(ROOT/plan['forecast_bindings'])['rows'];lookup,controls=sources(plan);cache={};rows=[]
 for r in bindings:
  case=lookup[r['window']];raw=case['case']['rows'];i=r['origin_pair'];key=(r['window'],i)
  if key not in cache:
   start=time.perf_counter_ns();s=snapshot(raw,i,case['timeline']);s['offline_reconstruction_cpu_s']=(time.perf_counter_ns()-start)/1e9;cache[key]=s
  s=cache[key];releases=[e for e in case['timeline'] if e['kind']=='release' and e['controller_monotonic_ns']<=s['epoch_ns']];latest=max(e['frame'] for e in releases);assert latest<r['target_pair'];release=min(e['controller_monotonic_ns'] for e in releases if e['frame']==i+1);delay=(s['epoch_ns']-release)/1e9;origin=controls[r['origin_id']];late={cam:origin['native_available'][cam] for cam in ['rgb','thermal']}
  rows.append({**{k:r[k] for k in ['forecast_id','origin_id','target_id','domain','recording_id','window','origin_pair','target_pair','target_start_ns','horizon_ns','realized_start_lead_s','observed_offline_prefix_delay_s']},'early_prefix':s,'latest_released_at_early_prefix':latest,'early_pair_end_delay_s':delay,'remaining_early_observation_budget_s':r['realized_start_lead_s']-delay,'remaining_early_reconstruction_budget_s':r['realized_start_lead_s']-delay-s['offline_reconstruction_cpu_s'],'availability_predictions':{'early_health':{cam:s['states'][cam]['health'] for cam in late},'early_confirmed':{cam:s['states'][cam]['confirmed'] for cam in late},'early_resolved':{cam:s['states'][cam]['resolved'] for cam in late},'late_eligible':late},'actions':{'early_health':s['actions']['health'],'early_confirmed':s['actions']['confirmed'],'early_resolved':s['actions']['resolved'],'late_eligible':choose(late),'late_information':origin['actions']['information_rank'],'rgb_only':'RGB','thermal_only':'THERMAL'}})
 assert len(rows)==1955
 write(OUT/'locked-availability-predictions.json',{'rows':rows,'future_outcomes_used':False,'pose_error_labels_read':False,'fits':0});write(OUT/'prediction-lock.json',{'predictions_sha256':sha(OUT/'locked-availability-predictions.json'),'forecast_bindings_sha256':sha(ROOT/plan['forecast_bindings']),'source_sha256':sha(__file__),'future_outcomes_used':False});verify();print('LOCKED',len(rows),'no-fit availability forecasts;',len(cache),'early prefixes')

def selection(rows,truth,method,target):
 counts=collections.Counter();ids={k:[] for k in ['selected_available','selected_unavailable','abstain_while_available','abstain_none_available']}
 for r in rows:
  t=truth[r['forecast_id']][target];action=r['actions'][method];anygood=any(t.values())
  if action=='ABSTAIN':state='abstain_while_available' if anygood else 'abstain_none_available'
  else:state='selected_available' if t[action.lower()] else 'selected_unavailable'
  counts[state]+=1;ids[state].append(r['forecast_id'])
 return {'forecasts':len(rows),'unique_targets':len({r['target_id'] for r in rows}),**{k:counts[k] for k in ids},'abstentions':counts['abstain_while_available']+counts['abstain_none_available'],'ids':ids}

def confusion(rows,truth,method,target,camera):
 out=collections.Counter()
 for r in rows:
  p=r['availability_predictions'][method][camera];t=truth[r['forecast_id']][target][camera]
  if p is None:out['unknown']+=1;out['unknown_but_available']+=int(t)
  else:out[('TP' if t else 'FP') if p else ('FN' if t else 'TN')]+=1
 return {k:out[k] for k in ['TP','FP','TN','FN','unknown','unknown_but_available']}

def recovery(rows,truth,method,target,camera):
 runs=[]
 for window in sorted({r['window'] for r in rows}):
  bytarget={}
  for r in sorted([r for r in rows if r['window']==window],key=lambda r:r['origin_pair']):bytarget.setdefault(r['target_pair'],r)
  ordered=sorted(bytarget);k=0
  while k<len(ordered):
   j=ordered[k];r=bytarget[j];ready=truth[r['forecast_id']][target][camera];previous=bytarget.get(j-1)
   if ready and previous is not None and not truth[previous['forecast_id']][target][camera]:
    run=[r];q=k+1
    while q<len(ordered) and ordered[q]==ordered[q-1]+1 and truth[bytarget[ordered[q]]['forecast_id']][target][camera]:run.append(bytarget[ordered[q]]);q+=1
    found=[r for r in run if r['availability_predictions'][method][camera] is True];runs.append({'window':window,'camera':camera,'start_pair':j,'pairs':len(run),'detected':bool(found),'delay_s':(found[0]['target_start_ns']-run[0]['target_start_ns'])/1e9 if found else None,'undetected_right_censored':not found});k=q
   else:k+=1
 detected=[r['delay_s'] for r in runs if r['detected']]
 return {'observed_false_to_true_runs':len(runs),'undetected':sum(not r['detected'] for r in runs),'detected_median_delay_s':statistics.median(detected) if detected else None,'runs':runs,'closed_loop_recovery':False}

def summary(rows,truth):
 return {'targets':{target:{'selection':{m:selection(rows,truth,m,target) for m in METHODS},'confusion':{m:{c:confusion(rows,truth,m,target,c) for c in ['rgb','thermal']} for m in ['early_health','early_confirmed','early_resolved','late_eligible']},'recovery':{m:{c:recovery(rows,truth,m,target,c) for c in ['rgb','thermal']} for m in ['early_health','early_confirmed','early_resolved','late_eligible']}} for target in ['tracking','camera_local','projected']},'timing':{'early_pair_end_delay_median_s':statistics.median(r['early_pair_end_delay_s'] for r in rows),'early_pair_end_delay_p95_s':float(__import__('numpy').quantile([r['early_pair_end_delay_s'] for r in rows],.95)),'old_decision_delay_median_s':statistics.median(r['observed_offline_prefix_delay_s'] for r in rows),'positive_early_reconstruction_budgets':sum(r['remaining_early_reconstruction_budget_s']>0 for r in rows),'forecasts':len(rows),'live_deadline_verified':False}}

def score():
 plan=verify();lock=read(OUT/'prediction-lock.json');assert lock['predictions_sha256']==sha(OUT/'locked-availability-predictions.json');rows=read(OUT/'locked-availability-predictions.json')['rows'];lookup,controls=sources(plan);truth={}
 for r in rows:
  case=lookup[r['window']];target=controls[r['target_id']];p=case['prefix'][case['case']['window'],r['target_pair']];raw=case['case']['rows'];j=r['target_pair'];tracking={}
  for cam in ['rgb','thermal']:
   first,last=raw[j][cam+'_left']['time_ns'],raw[j+1][cam+'_left']['time_ns'];end,=[e['event'] for e in case['timeline'] if e['kind']=='observer_event' and e['camera']==cam and e['event']['kind']=='pair_end' and [e['event']['time0_ns'],e['event']['time1_ns']]==[first,last]];value=supported(end);assert type(value) is bool;tracking[cam]=value
  truth[r['forecast_id']]={'tracking':tracking,'camera_local':{c:p['experts'][c]['available_by_prefix'] for c in ['rgb','thermal']},'projected':target['native_available']}
 write(OUT/'availability-truth.json',truth);results={}
 for h in [500_000_000,1_000_000_000]:
  selected=[r for r in rows if r['horizon_ns']==h];domains={}
  for domain in ['td3','ms2']:
   rr=[r for r in selected if r['domain']==domain];s=summary(rr,truth);a=s['targets']['projected']['selection']['early_resolved'];b=s['targets']['projected']['selection']['late_information'];s['availability_noninferiority_passed']=a['selected_available']>=b['selected_available'] and a['selected_unavailable']<=b['selected_unavailable'] and a['abstentions']<=b['abstentions'];domains[domain]=s
  results[str(h)]={'domains':domains,'records':{rec:summary([r for r in selected if r['recording_id']==rec],truth) for rec in sorted({r['recording_id'] for r in selected})},'aggregate':summary(selected,truth),'availability_noninferiority_passed':all(s['availability_noninferiority_passed'] for s in domains.values())}
 write(OUT/'availability-scores.json',{'horizons':results,'primary_horizon_ns':500_000_000,'primary_availability_noninferiority_passed':results['500000000']['availability_noninferiority_passed'],'future_fallback_used':False,'pose_error_labels_read':False,'fits':0,'prospective_validation':False,'policy_adopted':False});verify();print(__import__('json').dumps({'primary_availability_noninferiority_passed':results['500000000']['availability_noninferiority_passed'],'horizons':{h:{'domains':{d:{'gate':s['availability_noninferiority_passed'],'projected_selections':{m:{k:v for k,v in x.items() if k!='ids'} for m,x in s['targets']['projected']['selection'].items()},'timing':s['timing']} for d,s in v['domains'].items()}} for h,v in results.items()}},indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['predict','score']);a=p.parse_args();globals()[a.action]()
