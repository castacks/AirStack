"""Independent real-data weighted least squares, causal bindings and score audit."""
from pathlib import Path
from collections import Counter
import copy,json,math,statistics
import numpy as np
from scipy.linalg import lstsq
from past_only_sensor_risk import ROOT,OUT,PLAN,read,sha,verify,load_labels
from Scripts.Setup.future_sensor_risk import fit_fold,predict

def main():
 plan=verify();rows=read(OUT/'source-features.json')['rows'];byid={r['forecast_id']:r for r in rows};labels=load_labels(plan);models=read(OUT/'models.json')['models'];predictions=read(OUT/'locked-predictions.json')['rows'];scores=read(OUT/'scores.json');checks=0
 assert len(rows)==len(byid)==len(predictions)==1955 and len(models)==28 and all(m['status']=='PASS_FIT' for m in models)
 def close(a,b,rtol=1e-9,atol=1e-11):
  nonlocal checks
  np.testing.assert_allclose(a,b,rtol=rtol,atol=atol);checks+=np.asarray(a).size
 for m in models:
  train=[byid[i] for i in m['train_ids']];assert all(r['recording_id']!=m['excluded_recording'] for r in train);groups=Counter(r['recording_id'] for r in train);w=np.asarray([1/len(groups)/groups[r['recording_id']] for r in train]);close(w,m['weights']);x=np.asarray([r[m['family']] for r in train]);y=np.asarray([math.log1p(labels[r['target_id']]['rgb']['translation_m']/.1)-math.log1p(labels[r['target_id']]['thermal']['translation_m']/.1) for r in train]);mean=np.asarray([math.fsum(a*b for a,b in zip(w,x[:,j])) for j in range(x.shape[1])]);constant=np.ptp(x,axis=0)==0;mean[constant]=x[0,constant];scale=np.sqrt(np.asarray([math.fsum(a*(b-c)**2 for a,b in zip(w,x[:,j])) for j,c in enumerate(mean)]));scale[constant]=1;close(mean,m['mean']);close(scale,m['scale']);z=(x-mean)/scale;design=np.c_[z,np.ones(len(x))];reg=np.c_[np.eye(x.shape[1]),np.zeros(x.shape[1])];solution=lstsq(np.r_[w[:,None]**.5*design,reg],np.r_[w**.5*y,np.zeros(x.shape[1])],lapack_driver='gelsd')[0];close(solution,np.r_[m['coefficients'],m['intercept']])
  subset=[r for r in rows if r['horizon_ns']==m['horizon_ns']];altered=copy.deepcopy(subset);guarded={k:v for k,v in labels.items() if k not in {r['target_id'] for r in subset if r['recording_id']==m['excluded_recording']}}
  for r in altered:
   if r['recording_id']==m['excluded_recording']:r[m['family']]=[1e20]*len(r[m['family']])
  before=fit_fold(subset,labels,m['excluded_recording'],m['family']);after=fit_fold(altered,guarded,m['excluded_recording'],m['family']);assert before==after;close(before['coefficients'],m['coefficients']);checks+=1
  for p in predictions:
   if p['horizon_ns']==m['horizon_ns'] and p['recording_id']==m['excluded_recording']:
    r=byid[p['forecast_id']];margin=float((np.asarray(r[m['family']])-mean)/scale@solution[:-1]+solution[-1]);close(margin,p['predictions'][m['family']]['margin']);assert p['actions'][m['family']]==('THERMAL' if margin>0 else 'RGB')
 # Receipt and target-time binding reconstructed from unmodified source timelines.
 for source in plan['sources']:
  base=ROOT/source['base']
  for c in source['cases']:
   timeline=[json.loads(s) for s in (base/'captures'/c['case']/'on/timeline.jsonl').read_text().splitlines()];raw=c['rows'];selected=[r for r in rows if r['window']==source['tag']+'/'+c['window']];cov={}
   for cam in ['rgb','thermal']:
    cov[cam]=[]
    for i in range(len(raw)-1):
     with np.load(base/'captures'/c['case']/'on/experts'/cam/'uncertainty'/f'pair-{i:03d}.npz',allow_pickle=False) as a:
      assert int(a['time0_ns'])==raw[i][cam+'_left']['time_ns'];K=a['K'].reshape(3,3).astype(float);v=a['grid_raw_flow_cov'].astype(float);values=[uu/K[0,0]**2+vv/K[1,1]**2 for uu,vv,uv in v];values=[v for v in values if math.isfinite(v) and v>=0];cov[cam].append(statistics.median(values));checks+=1
   for r in selected:
    i=r['origin_pair'];j=r['target_pair'];end=raw[i+1]['rgb_left']['time_ns'];possible=[k for k in range(i+1,len(raw)-1) if raw[k]['rgb_left']['time_ns']>=end];nearest=min(possible,key=lambda k:(abs(raw[k]['rgb_left']['time_ns']-end-r['horizon_ns']),raw[k]['rgb_left']['time_ns']));assert nearest==j and abs(raw[j]['rgb_left']['time_ns']-end-r['horizon_ns'])<=50_000_000;assert len(r['receipts'])==6
    for receipt in r['receipts']:
     assert receipt['pair'] in [i,i-1,i-2];e,=[e for e in timeline if e['kind']=='observer_event' and e['camera']==receipt['camera'] and e['event']['sequence']==receipt['event_sequence']];assert e['event']['kind']=='pair_end' and e['controller_monotonic_ns']==receipt['received_ns']<=r['origin_controller_deadline_ns'];cam=receipt['camera'];k=receipt['pair'];assert [e['event']['time0_ns'],e['event']['time1_ns']]==[raw[k][cam+'_left']['time_ns'],raw[k+1][cam+'_left']['time_ns']];checks+=3
    releases=[e for e in timeline if e['kind']=='release' and e['controller_monotonic_ns']<=r['origin_controller_deadline_ns']];assert max(e['frame'] for e in releases)==r['latest_released_frame']<j
    earliest=min(e['controller_monotonic_ns'] for e in releases if e['frame']==i+1);close((r['origin_controller_deadline_ns']-earliest)/1e9,r['observed_offline_prefix_delay_s']);close((raw[j]['rgb_left']['time_ns']-end)/1e9-r['observed_offline_prefix_delay_s'],r['remaining_instrumented_budget_s'])
    for n,cam in enumerate(['rgb','thermal']):
     rate=[math.log(max(cov[cam][i-k]/((raw[i-k+1][cam+'_left']['time_ns']-raw[i-k][cam+'_left']['time_ns'])/1e9)**2,1e-30)) for k in range(3)];close(r['current'][n*8],rate[0],rtol=1e-6,atol=1e-6);close(r['history'][16+n*2:18+n*2],[rate[1]-rate[0],rate[2]-rate[0]],rtol=1e-5,atol=1e-6)
 # Independent every-record/horizon population and scalar error computation.
 for h,result in scores['horizons'].items():
  selected=[r for r in predictions if str(r['horizon_ns'])==h]
  for record,s in result['records'].items():
   allrows=[r for r in selected if r['recording_id']==record];both=[r for r in allrows if r['target_id'] in labels and all(labels[r['target_id']].get(c) is not None for c in ['rgb','thermal'])];matched=[r for r in both if r['actions']['current']!='ABSTAIN' and r['actions']['history']!='ABSTAIN']
   for scope,chosen in [('all',allrows),('both_supported',both),('current_history_matched',matched)]:
    for method,stat in s[scope].items():
     valid=[];missing=unavailable=abstain=0;values=[];miss=Counter()
     for r in chosen:
      label=labels.get(r['target_id']);action=r['actions'][method]
      if label is None:missing+=1;continue
      if action=='ABSTAIN':abstain+=1;continue
      if label.get(action.lower()) is None:unavailable+=1;continue
      valid.append(r['forecast_id']);values.append(label[action.lower()])
      if all(label.get(c) is not None for c in ['rgb','thermal']):
       winner='rgb' if label['rgb']['translation_m']<=label['thermal']['translation_m'] else 'thermal'
       if action.lower()!=winner:miss[winner]+=1
     assert stat['ids']==valid and stat['scored']==len(valid) and stat['forecasts']==len(chosen) and stat['unique_targets']==len({r['target_id'] for r in chosen});assert [stat[k] for k in ['missing_reference','selected_unavailable','abstentions','missed_thermal_opportunities','missed_rgb_opportunities']]==[missing,unavailable,abstain,miss['thermal'],miss['rgb']];checks+=9
     for metric in ['translation_m','rotation_deg']:
      if values:close(stat[metric],math.sqrt(math.fsum(v[metric]**2 for v in values)/len(values)))
      else:assert stat[metric] is None
  for domain,d in result['domains'].items():
   for method,m in d['macro'].items():
    chosen=[result['records'][rec]['current_history_matched'][method] for rec in d['recordings']];chosen=[s for s in chosen if s['scored']];close(m['translation_mse'],statistics.mean(s['translation_m']**2 for s in chosen));close(m['rotation_mse'],statistics.mean(s['rotation_deg']**2 for s in chosen));assert m['selected_unavailable']==sum(result['records'][rec]['all'][method]['selected_unavailable'] for rec in d['recordings'])
   a,b=[d['macro'][m] for m in ['current','history']];passed=b['translation_mse']<a['translation_mse'] and b['rotation_mse']<=a['rotation_mse'] and b['selected_unavailable']<=a['selected_unavailable'] and b['scored']>=a['scored'];assert bool(passed)==d['development_increment_passed']
  assert result['development_increment_passed']==all(d['development_increment_passed'] for d in result['domains'].values())
 verify();out={'status':'PASS_INDEPENDENT_PAST_ONLY_RISK_AUDIT','numeric_and_population_checks':int(checks),'models':len(models),'forecasts':len(rows),'heldout_label_deletion_and_feature_mutation_invariant':True,'target_unreleased_at_all_prefixes':True,'scores_sha256':sha(OUT/'scores.json'),'auditor_sha256':sha(__file__),'live_deadline_verified':False}
 with (OUT/'audit.json').open('x') as f:json.dump(out,f,indent=2);f.write('\n')
 print(out)
if __name__=='__main__':main()
