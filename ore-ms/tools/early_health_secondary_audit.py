"""Independent recovery/censoring reconstruction and fresh receipt timing audit."""
import collections,statistics,math
from early_health_execution import ROOT,OUT,read,write,sha,verify
from early_health_study import sources
from early_health_capture_audit import journal

def percentile(values,q):
 a=sorted(values);z=(len(a)-1)*q;i=int(z);return a[i]+(a[min(i+1,len(a)-1)]-a[i])*(z-i)
def main():
 plan=verify();pred=read(OUT/'locked-availability-predictions.json')['rows'];truth=read(OUT/'availability-truth.json');scores=read(OUT/'availability-scores.json');fresh=read(OUT/'fresh-stage-timings.json');checks=0
 def recovery_check(result,rows):
  nonlocal checks
  for target,r in result['targets'].items():
   for method,cameras in r['recovery'].items():
    for cam,saved in cameras.items():
     expected=[]
     for window in sorted({p['window'] for p in rows}):
      targets={}
      for p in sorted([p for p in rows if p['window']==window],key=lambda p:p['origin_pair']):targets.setdefault(p['target_pair'],p)
      ordered=sorted(targets);runs=[];run=[]
      for j in ordered:
       p=targets[j]
       if truth[p['forecast_id']][target][cam]:
        if run and j!=run[-1]['target_pair']+1:runs.append(run);run=[]
        run.append(p)
       elif run:runs.append(run);run=[]
      if run:runs.append(run)
      for run in runs:
       first=run[0]['target_pair'];previous=targets.get(first-1)
       if previous is None or truth[previous['forecast_id']][target][cam]:continue
       good=next((p for p in run if p['availability_predictions'][method][cam] is True),None)
       expected.append({'window':window,'camera':cam,'start_pair':first,'pairs':len(run),'detected':good is not None,'delay_s':(good['target_start_ns']-run[0]['target_start_ns'])/1e9 if good else None,'undetected_right_censored':good is None})
     assert expected==saved['runs'];assert saved['observed_false_to_true_runs']==len(expected) and saved['undetected']==sum(r['undetected_right_censored'] for r in expected);detected=[r['delay_s'] for r in expected if r['detected']];assert saved['detected_median_delay_s']==(statistics.median(detected) if detected else None);checks+=4+len(expected)
 for h,result in scores['horizons'].items():
  rows=[p for p in pred if str(p['horizon_ns'])==h];recovery_check(result['aggregate'],rows)
  for domain,r in result['domains'].items():recovery_check(r,[p for p in rows if p['domain']==domain])
  for record,r in result['records'].items():recovery_check(r,[p for p in rows if p['recording_id']==record])
 for c in plan['fresh_cases']:
  for arm in ['off','on']:
   log=journal(OUT/'captures'/c['case']/arm/'timeline.jsonl');stored={p['pair']:p for p in fresh['rows'] if p['case']==c['case'] and p['arm']==arm}
   for i in range(c['frames']-1):
    p=stored[i];firstrelease=min(e['controller_monotonic_ns'] for e in log if e['kind']=='release' and e['frame']==i+1)
    for cam in ['rgb','thermal']:
     for name,kind in [('covariance','early_covariance_available'),('tracking','early_tracking_available'),('pair_end','pair_end'),('information','native_disparity_information')]:
      matches=[e for e in log if e['kind']=='observer_event' and e['camera']==cam and e['event']['kind']==kind and [e['event']['time0_ns'],e['event']['time1_ns']]==[c['rows'][i][cam+'_left']['time_ns'],c['rows'][i+1][cam+'_left']['time_ns']]];value=(matches[0]['controller_monotonic_ns']-firstrelease)/1e9 if matches else None;assert value==p['per_camera'][cam][name];checks+=1
    decision,=[e for e in log if e['kind']=='decision' and e['pair_ordinal']==i];assert p['old_decision_prefix_s']==(decision['controller_monotonic_ns']-firstrelease)/1e9;checks+=1
 for domain,s in fresh['summaries'].items():
  rows=[r for r in fresh['rows'] if r['domain']==domain and r['arm']=='on']
  for name in ['joint_covariance_s','joint_tracking_s','joint_pair_end_s','joint_information_s','old_decision_prefix_s']:
   v=[r[name] for r in rows if r[name] is not None];expected={'n':len(v),'min':min(v),'median':statistics.median(v),'p95':percentile(v,.95),'max':max(v)}
   for key,value in expected.items():assert math.isclose(s[name][key],value,abs_tol=1e-12);checks+=1
  assert fresh['fresh_instrumented_readiness_gate'][domain]==(s['joint_tracking_s']['p95']<.5)
 verdict=read(OUT/'primary-verdict.json');assert verdict['primary_combined_passed']==(scores['primary_availability_noninferiority_passed'] and all(fresh['fresh_instrumented_readiness_gate'].values()));verify();write(OUT/'secondary-audit.json',{'status':'PASS_INDEPENDENT_RECOVERY_AND_FRESH_TIMING_AUDIT','checks':checks,'fresh_pair_arm_rows':len(fresh['rows']),'source_sha256':sha(__file__),'fresh_timing_sha256':sha(OUT/'fresh-stage-timings.json'),'availability_scores_sha256':sha(OUT/'availability-scores.json')});print('PASS',checks,'recovery/timing checks')
if __name__=='__main__':main()
