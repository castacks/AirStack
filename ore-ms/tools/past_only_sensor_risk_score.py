"""Held-out-record scoring after prediction lock; no predictor inputs from labels."""
from collections import Counter
import argparse,json,math,statistics
from past_only_sensor_risk import ROOT,OUT,read,write,sha,verify,load_labels
METHODS=['current','history','rgb_only','thermal_only','origin_information']
def stats(rows,labels,method):
 valid=[];unavailable=0;abstain=0;missing=0;losses=[];miss={'thermal':0,'rgb':0}
 for r in rows:
  label=labels.get(r['target_id']);action=r['actions'][method]
  if label is None:missing+=1;continue
  if action=='ABSTAIN':abstain+=1;continue
  loss=label.get(action.lower())
  if loss is None:unavailable+=1;continue
  valid.append(r['forecast_id']);losses.append(loss)
  if all(label.get(c) is not None for c in ['rgb','thermal']):
   winner='rgb' if label['rgb']['translation_m']<=label['thermal']['translation_m'] else 'thermal'
   if action.lower()!=winner:miss[winner]+=1
 return {'forecasts':len(rows),'unique_targets':len({r['target_id'] for r in rows}),'scored':len(valid),'ids':valid,'selected_unavailable':unavailable,'abstentions':abstain,'missing_reference':missing,'missed_thermal_opportunities':miss['thermal'],'missed_rgb_opportunities':miss['rgb'],**{m:math.sqrt(math.fsum(v[m]**2 for v in losses)/len(losses)) if losses else None for m in ['translation_m','rotation_deg']}}
def delays(rows,labels,method):
 runs=[]
 for window in sorted({r['window'] for r in rows}):
  unique={}
  for r in sorted([r for r in rows if r['window']==window],key=lambda r:r['origin_pair']):unique.setdefault(r['target_pair'],r)
  last=None;winner=None;run=[]
  def finish():
   if not run:return
   good=[r for r in run if r['actions'][method].lower()==winner];runs.append({'window':window,'winner':winner,'start_target_pair':run[0]['target_pair'],'pairs':len(run),'detected':bool(good),'delay_s':(good[0]['target_start_ns']-run[0]['target_start_ns'])/1e9 if good else None,'right_censored':not good})
  for i,r in sorted(unique.items()):
   label=labels.get(r['target_id']);new=None
   if label and all(label.get(c) is not None for c in ['rgb','thermal']):new='rgb' if label['rgb']['translation_m']<=label['thermal']['translation_m'] else 'thermal'
   if new is None or new!=winner or last is None or i!=last+1:finish();run=[]
   if new is not None:run.append(r)
   winner,last=new,i
  finish()
 detected=[r['delay_s'] for r in runs if r['detected']]
 return {'runs':runs,'run_count':len(runs),'undetected_runs':sum(not r['detected'] for r in runs),'detected_run_median_delay_s':statistics.median(detected) if detected else None,'actual_closed_loop_recovery':False}
def report(rows,labels):
 both=[r for r in rows if r['target_id'] in labels and all(labels[r['target_id']].get(c) is not None for c in ['rgb','thermal'])]
 matched=[r for r in both if r['actions']['current']!='ABSTAIN' and r['actions']['history']!='ABSTAIN']
 out={'all':{m:stats(rows,labels,m) for m in METHODS},'both_supported':{m:stats(both,labels,m) for m in METHODS},'current_history_matched':{m:stats(matched,labels,m) for m in ['current','history']},'winner_delay':{m:delays(rows,labels,m) for m in ['current','history']},'target_translation_winners':dict(Counter('rgb' if labels[r['target_id']]['rgb']['translation_m']<=labels[r['target_id']]['thermal']['translation_m'] else 'thermal' for r in both))}
 return out
def score():
 plan=verify();lock=read(OUT/'prediction-lock.json');assert lock['predictions_sha256']==sha(OUT/'locked-predictions.json') and lock['models_sha256']==sha(OUT/'models.json') and not lock['test_labels_used_for_prediction'];rows=read(OUT/'locked-predictions.json')['rows'];labels=load_labels(plan);horizons={}
 for h in plan['horizons_ns']:
  selected=[r for r in rows if r['horizon_ns']==h];records={record:report([r for r in selected if r['recording_id']==record],labels) for record in sorted({r['recording_id'] for r in selected})};domains={}
  for domain in ['td3','ms2']:
   members=sorted({r['recording_id'] for r in selected if r['domain']==domain});macro={}
   for method in ['current','history']:
    values=[records[m]['current_history_matched'][method] for m in members];valid=[v for v in values if v['scored']]
    macro[method]={'recordings_with_joint_scores':len(valid),'translation_mse':statistics.mean(v['translation_m']**2 for v in valid) if valid else None,'rotation_mse':statistics.mean(v['rotation_deg']**2 for v in valid) if valid else None,'selected_unavailable':sum(records[m]['all'][method]['selected_unavailable'] for m in members),'scored':sum(records[m]['all'][method]['scored'] for m in members)}
   a,b=macro['current'],macro['history'];positive=a['translation_mse'] is not None and b['translation_mse'] is not None and b['translation_mse']<a['translation_mse'] and b['rotation_mse']<=a['rotation_mse'] and b['selected_unavailable']<=a['selected_unavailable'] and b['scored']>=a['scored'];domains[domain]={'recordings':members,'macro':macro,'development_increment_passed':bool(positive)}
  horizons[str(h)]={'records':records,'domains':domains,'development_increment_passed':all(d['development_increment_passed'] for d in domains.values()),'aggregate':report(selected,labels),'budget':{d:{'forecasts':len([r for r in selected if r['domain']==d]),'positive_remaining_instrumented_budget':sum(r['remaining_instrumented_budget_including_prediction_s']>0 for r in selected if r['domain']==d),'median_prefix_delay_s':statistics.median(r['observed_offline_prefix_delay_s'] for r in selected if r['domain']==d),'p95_prefix_delay_s':float(__import__('numpy').quantile([r['observed_offline_prefix_delay_s'] for r in selected if r['domain']==d],.95)),'live_deadline_verified':False} for d in ['td3','ms2']}}
 result={'horizons':horizons,'primary_horizon_ns':plan['primary_horizon_ns'],'primary_development_increment_passed':horizons[str(plan['primary_horizon_ns'])]['development_increment_passed'],'model_adopted':False,'prospective_validation':False,'physical_accuracy_verified':False,'prediction_lock_sha256':sha(OUT/'prediction-lock.json')};write(OUT/'scores.json',result);verify();print(json.dumps({'primary_development_increment_passed':result['primary_development_increment_passed'],'horizons':{h:{'domains':s['domains'],'budget':s['budget'],'records':{r:{m:{k:v for k,v in s['current_history_matched'][m].items() if k!='ids'} for m in ['current','history']} for r,s in s['records'].items()}} for h,s in horizons.items()}},indent=2))
if __name__=='__main__':score()
