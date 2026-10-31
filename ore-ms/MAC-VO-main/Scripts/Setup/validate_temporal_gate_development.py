"""Independent EXP091 source, scalar fit, inference and matched-score audit.

Uses only the standard library; imports no feature, fit, inference or scorer code.
Semantic mutations run after hash verification, so rejection cannot rely on hashes.
"""
import argparse
from collections import Counter
from copy import deepcopy
import hashlib
from itertools import combinations
import json
import math
from pathlib import Path
import statistics

FAMILIES={'combined':list(range(42)), 'uncertainty':[i for i in range(42) if i%7 in (4,5)],
 'health':[i for i in range(42) if i%7 not in (4,5)], 'current':list(range(7))+list(range(21,28))}
BASE=('rgb_only','thermal_only','raw_advice','history')
METHODS=BASE+tuple(FAMILIES)
OPTIONS={'maxiter':2000,'gtol':1e-9,'ftol':0.,'maxls':20,'maxcor':10,'maxfun':100000}
NAMES=('retained_fraction','inbound_fraction','occupied_fraction','positive_diagonal_fraction','log_median_trace','log_trace_spread','supported')

def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def near(a,b,tol=2e-12):
 assert math.isfinite(a) and math.isfinite(b) and abs(a-b)<=tol+abs(a)*1e-10,(a,b)
def equal(a,b):
 if isinstance(a,dict):
  assert isinstance(b,dict) and set(a)==set(b),(set(a),set(b))
  for k in a:equal(a[k],b[k])
 elif isinstance(a,list):
  assert isinstance(b,list) and len(a)==len(b)
  for x,y in zip(a,b):equal(x,y)
 elif isinstance(a,bool):assert type(b) is bool and a==b
 elif isinstance(a,int):assert type(b) is int and a==b
 elif isinstance(a,float):near(a,b)
 else:assert a==b,(a,b)


def native_vector(rows,pair,events):
 if pair<2:return None,{'reason':'cold_start','health_sequences':{}}
 values=[];evidence={}
 for camera in ('rgb','thermal'):
  evidence[camera]=[]
  for lag in (0,1,2):
   times=[rows[pair-lag+j][camera+'_left']['time_ns'] for j in (0,1)]
   assert all(type(t) is int for t in times) and times[1]>times[0]
   found=[e for e in events[camera] if e['kind']=='pair_end' and [e['time0_ns'],e['time1_ns']]==times]
   assert len(found)<=1
   e=found[0] if found else None
   fields=('selected_count','inbound_count','retained_count','occupied_3x3_cells','grid_count','grid_positive_uu_vv_count','original_need_interp','execution_error','grid_trace_q10_q50_q90_px2')
   q=e.get('grid_trace_q10_q50_q90_px2') if e else None
   if e is None or any(k not in e for k in fields) or not isinstance(q,list) or len(q)!=3 or any(type(x) not in (int,float) or not math.isfinite(x) or x<0 for x in q):
    return None,{'reason':'missing_observed_features','health_sequences':evidence}
   s,i,r,n,g,p=[e[k] for k in fields[:6]]
   assert all(type(x) is int and x>=0 for x in (s,i,r,n,g,p)) and s>=i>=r and n<=9 and p<=g
   assert q==sorted(q) and type(e['original_need_interp']) is bool
   values.extend([r/max(s,1),i/max(s,1),n/9,p/max(g,1),math.log1p(q[1]),math.log((1+q[2])/(1+q[0])),float(e['original_need_interp'] is False and e['execution_error'] is None)])
   evidence[camera].append(e['sequence'])
 return values,{'reason':'complete','health_sequences':evidence}


def source_rows(root):
 old=root/'results/prefix-model-pipeline/run-001';new=root/'results/third-recording-shared-capture/run-001'
 proof=read(old/'validation.json');assert proof['status']=='PASS'
 assert proof['predictions_sha256']==sha(old/'predictions.json') and proof['scores_sha256']==sha(old/'scores.json')
 assert read(old/'protocol.json')['contract_sha256']==sha(old/'predictor-contract.json')
 assert read(old/'predictions.json')['protocol_sha256']==sha(old/'protocol.json')
 lock=read(new/'prediction-lock.json');annotation=read(new/'annotation-validation.json')
 assert annotation['status']=='PASS_CONDITIONAL_ANNOTATION' and annotation['annotation_sha256']==sha(new/'annotation.json')
 assert lock['capture_plan_sha256']==sha(new/'capture-plan.json')
 specs=[]
 for window,c in read(old/'predictor-contract.json').items():
  specs.append(('EXP089',window,'td3/20251027/0_salon_so_day' if window=='day' else 'td3/20251027/night_mac_salon_warehousefig8',c['input_rows'],root/c['run_relative']/'timeline.jsonl',[p for p in read(old/'predictions.json')['rows'] if p['window']==window]))
 for c in read(new/'capture-plan.json')['windows']:
  case=c['case'];p=new/'windows'/case;model=read(p/'model-predictions.json')
  assert sha(p/'model-predictions.json')==lock['model_prediction_sha256'][str((p/'model-predictions.json').relative_to(root))]
  assert model['timeline_sha256']==sha(p/'shared/timeline.jsonl')==read(p/'shared/validation.json')['timeline_sha256']
  specs.append(('EXP090',case,'td3/20251027/1_salon_mac_cov_day',c['input_rows'],p/'shared/timeline.jsonl',model['rows']))
 result=[]
 for prefix,window,recording,rows,path,model_rows in specs:
  events={'rgb':[],'thermal':[]};receipts={'rgb':{},'thermal':{}};models={p['pair_ordinal']:p for p in model_rows};previous=-1;pair_count=0
  for sequence,line in enumerate(path.read_text().splitlines()):
   entry=json.loads(line);now=entry['controller_monotonic_ns'];assert entry['sequence']==sequence and type(now) is int and now>=previous;previous=now
   if entry['kind']=='observer_event':
    c=entry['camera'];e=entry['event'];assert e['sequence']==len(events[c]);events[c].append(e);receipts[c][e['sequence']]=now
   elif entry['kind']=='decision':
    pair=entry['pair_ordinal'];assert pair==pair_count;pair_count+=1;p=models[pair]
    assert p['source_decision_sequence']==sequence and p['controller_deadline_ns']==now
    assert p['observed_event_counts']=={c:len(es) for c,es in events.items()}
    vector,evidence=native_vector(rows,pair,events)
    assert all(receipts[c][n]<=now for c,ns in evidence['health_sequences'].items() for n in ns)
    both=all(v['valid_model_projection'] for v in p['experts'].values())
    result.append({'sample_id':f'{prefix}/{window}/{pair}','recording_id':recording,'window':window,'pair_ordinal':pair,
     'common_interval_ns':p['common_interval_ns'],'vector':vector,'feature_evidence':evidence,'both_supported':both,
     'eligible':bool(both and vector is not None),'baseline_actions':p['model_actions'],'source_controller_sequence':sequence})
  assert pair_count==len(rows)-1
 labels={f"EXP089/{r['window']}/{r['pair_ordinal']}":{'rgb':r['losses']['rgb_only'],'thermal':r['losses']['thermal_only']} for r in read(old/'scores.json')['rows']}
 labels.update({f"EXP090/{r['case']}/{r['pair_ordinal']}":r['camera_losses'] for r in read(new/'annotation.json')['rows']})
 index={(r['recording_id'],tuple(r['common_interval_ns'])):r for r in read(new/'development-index.json')['entries']}
 eligible=[r for r in result if r['eligible']];assert len(result)==274 and len(eligible)==len(index)==161
 assert len({(r['recording_id'],tuple(r['common_interval_ns'])) for r in eligible})==161
 for r in eligible:
  e=index[r['recording_id'],tuple(r['common_interval_ns'])];l=labels[r['sample_id']]
  assert e['rgb_translation_proxy_loss_m']==l['rgb']['translation_m'] and e['thermal_translation_proxy_loss_m']==l['thermal']['translation_m']
 return result,labels


def load(root,out):
 p=read(out/'protocol.json');assert sha(out/'features.json')==p['feature_sha256'] and sha(out/'labels.json')==p['labels_sha256'] and sha(out/'declaration.md')==p['declaration_sha256']
 assert all(sha(root/f)==h for f,h in p['input_sha256'].items()) and all(sha(root/f)==h for f,h in p['source_sha256'].items())
 lock=read(out/'models-lock.json');assert lock['protocol_sha256']==sha(out/'protocol.json')
 assert len(lock['models_sha256'])==16 and all(sha(out/f)==h for f,h in lock['models_sha256'].items())
 proof=read(out/'prediction-proof.json');assert proof['predictions_sha256']==sha(out/'predictions.json')
 pred=read(out/'predictions.json');assert pred['models_lock_sha256']==sha(out/'models-lock.json') and pred['features_sha256']==p['feature_sha256']
 scores=read(out/'scores.json');assert scores['predictions_sha256']==sha(out/'predictions.json')
 expected,labels=source_rows(root)
 return {'protocol':p,'protocol_hash':sha(out/'protocol.json'),'features':read(out/'features.json'),'labels':read(out/'labels.json'),
  'models':{f:read(out/f) for f in lock['models_sha256']},'lock':lock,'predictions':pred,'proof':proof,'scores':scores,'expected':expected,'expected_labels':labels}


def summary(values):
 if not values:return {'count':0,'mean':None,'median':None,'rmse':None,'max':None}
 return {'count':len(values),'mean':math.fsum(values)/len(values),'median':float(statistics.median(values)),
  'rmse':math.sqrt(math.fsum(v*v for v in values)/len(values)),'max':max(values)}


def validate(d):
 p=d['protocol'];rows=d['features']['rows'];labels=d['labels'];recordings=sorted({r['recording_id'] for r in d['expected']})
 equal(d['expected'],rows);equal(d['expected_labels'],labels)
 equal([f'{c}/lag{lag}/{n}' for c in ('rgb','thermal') for lag in range(3) for n in NAMES],d['features']['feature_names'])
 equal(recordings,p['recordings']);equal(FAMILIES,p['families']);equal(OPTIONS,p['solver_options']);assert p['ridge']==.01 and p['primary']=='combined' and p['no_hyperparameter_search'] is True
 assert p['rows']==274 and p['eligible']==161
 objective_checks=0
 for family,indices in FAMILIES.items():
  for fold,excluded in enumerate(recordings+[None]):
   name=f'fold-{fold}' if excluded is not None else 'all-development';m=d['models'][f'models/{family}/{name}.json']
   assert m['protocol_sha256']==d['protocol_hash'] and m['family']==family and m['feature_indices']==indices
   assert m['excluded_recording']==excluded and m['training_recordings']==[g for g in recordings if g!=excluded]
   train=[r for r in rows if r['eligible'] and r['recording_id']!=excluded]
   assert m['train_sample_ids']==[r['sample_id'] for r in train]
   x=[[r['vector'][j] for j in indices] for r in train];n=len(x)
   errors=[(labels[r['sample_id']]['rgb']['translation_m'],labels[r['sample_id']]['thermal']['translation_m']) for r in train]
   costs=[abs(b*b-a*a) for a,b in errors];y=[float(a<b) for a,b in errors]
   totals={g:math.fsum(c for r,c in zip(train,costs) if r['recording_id']==g) for g in recordings if g!=excluded}
   active=sum(t>0 for t in totals.values());weights=[c/totals[r['recording_id']]/active if totals[r['recording_id']]>0 else 0. for r,c in zip(train,costs)]
   equal(totals,m['record_cost_totals']);equal(weights,m['train_weights']);equal(OPTIONS,m['options']);assert m['ridge']==.01 and m['solver']=='L-BFGS-B'
   means=[];scales=[];constant=[]
   for j in range(len(indices)):
    column=[v[j] for v in x];same=min(column)==max(column);mean=column[0] if same else math.fsum(column)/n
    means.append(mean);scales.append(1. if same else math.sqrt(math.fsum((v-mean)**2 for v in column)/n));constant.append(same)
   equal(means,m['mean']);equal(scales,m['scale']);equal(constant,m['constant_columns'])
   assert len(m['coefficients'])==len(indices)
   z=[[(v-a)/s for v,a,s in zip(vs,m['mean'],m['scale'])] for vs in x]
   margins=[math.fsum(a*b for a,b in zip(v,m['coefficients']))+m['intercept'] for v in z]
   probability=[1/(1+math.exp(-s)) if s>=0 else math.exp(s)/(1+math.exp(s)) for s in margins]
   residual=[w*(q-t) for w,q,t in zip(weights,probability,y)]
   objective=math.fsum(w*(math.log1p(math.exp(-abs(s)))+max(s,0)-t*s) for w,s,t in zip(weights,margins,y))+.005*math.fsum(a*a for a in m['coefficients'])
   gradient=[math.fsum(r*v[j] for r,v in zip(residual,z))+.01*m['coefficients'][j] for j in range(len(indices))]+[math.fsum(residual)]
   near(objective,m['objective']);near(max(abs(g) for g in gradient),m['gradient_inf'])
   passed=m['native_success'] is True and m['gradient_inf']<=1e-9
   assert m['status']==('PASS_FIT' if passed else 'FAILED_FIT') and m['reason']==('converged' if passed else 'solver_or_stationarity_failure')
   if passed:assert max(abs(g) for g in gradient)<=1e-9+2e-12
   equal({'status':m['status'],'reason':m['reason'],'gradient_inf':m['gradient_inf']},d['lock']['statuses'][family+'/'+name]);objective_checks+=1
 expected_pred=[]
 for r in rows:
  decisions={};fold=recordings.index(r['recording_id'])
  for f,indices in FAMILIES.items():
   m=d['models'][f'models/{f}/fold-{fold}.json'];fallback=r['baseline_actions']['raw_advice']
   assert m['excluded_recording']==r['recording_id'] and r['recording_id'] not in m['training_recordings']
   if not r['eligible'] or r['vector'] is None:v={'action':fallback,'reason':'original_fallback','margin':None}
   elif m['status']!='PASS_FIT':v={'action':fallback,'reason':'failed_fit_fallback','margin':None}
   else:
    margin=math.fsum((r['vector'][j]-mean)/scale*c for j,mean,scale,c in zip(indices,m['mean'],m['scale'],m['coefficients']))+m['intercept']
    v={'action':'RGB' if margin>=0 else 'THERMAL','reason':'learned_margin','margin':margin}
   assert (v['action']=='ABSTAIN')==(fallback=='ABSTAIN');decisions[f]=v
  expected_pred.append({'sample_id':r['sample_id'],'recording_id':r['recording_id'],'actions':r['baseline_actions']|{f:v['action'] for f,v in decisions.items()},'learned_decisions':decisions})
 equal(expected_pred,d['predictions']['rows'])
 coverage={m:sum(r['actions'][m]!='ABSTAIN' for r in expected_pred) for m in METHODS}
 equal(coverage,d['proof']['coverage']);assert all(coverage[f]==268 for f in FAMILIES) and d['proof']['labels_read_at_prediction'] is False
 scored=[]
 for r in expected_pred:
  losses={m:(None if a=='ABSTAIN' else labels[r['sample_id']][a.lower()]) for m,a in r['actions'].items()}
  assert all((losses[m] is None)==(a=='ABSTAIN') for m,a in r['actions'].items())
  scored.append({'sample_id':r['sample_id'],'recording_id':r['recording_id'],'losses':losses})
 equal(scored,d['scores']['rows']);groups={}
 for g in recordings:
  rs=[r for r in scored if r['recording_id']==g];policies={};matched={}
  for m in METHODS:
   valid=[r['losses'][m] for r in rs if r['losses'][m] is not None]
   policies[m]={'available':len(valid),'abstain':len(rs)-len(valid),'conditional':{k:summary([v[k] for v in valid]) for k in ('translation_m','rotation_deg')}}
  for a,b in combinations(METHODS,2):
   both=[r for r in rs if all(r['losses'][m] is not None for m in (a,b))]
   matched[a+'__'+b]={'matched_sample_ids':[r['sample_id'] for r in both],'policies':{m:{k:summary([r['losses'][m][k] for r in both]) for k in ('translation_m','rotation_deg')} for m in (a,b)}}
  groups[g]={'population':len(rs),'policies':policies,'matched_comparisons':matched}
 equal(groups,d['scores']['groups']);diagnostics={}
 for f in FAMILIES:
  per={}
  for g in recordings:
   per[g]={}
   for b in BASE[:3]:
    c=groups[g]['matched_comparisons'][b+'__'+f];n=len(c['matched_sample_ids'])
    per[g][b]={'matched':n,'strict_translation_improvement':bool(n and c['policies'][f]['translation_m']['rmse']<c['policies'][b]['translation_m']['rmse'])}
  diagnostics[f]={'development_consistent_translation_diagnostic':all(v['strict_translation_improvement'] for checks in per.values() for v in checks.values()),'per_record':per}
 equal(diagnostics,d['scores']['development_diagnostic'])
 equal({m:{k:math.fsum(groups[g]['policies'][m]['conditional'][k]['rmse'] for g in recordings)/3 for k in ('translation_m','rotation_deg')} for m in METHODS},d['scores']['equal_recording_conditional_rmse_mean'])
 return {'rows':len(rows),'eligible':sum(r['eligible'] for r in rows),'feature_coordinates':sum(len(r['vector']) for r in rows if r['vector'] is not None),
  'recordings':len(recordings),'fit_objective_and_stationarity_checks':objective_checks,'coverage':coverage,'matched_comparison_groups':3*28,
  'fit_statuses':dict(Counter(m['status'] for m in d['models'].values())),'diagnostics':{f:v['development_consistent_translation_diagnostic'] for f,v in diagnostics.items()}}


def challenges(d):
 i=next(i for i,r in enumerate(d['features']['rows']) if r['eligible']);sid=d['features']['rows'][i]['sample_id']
 g=d['protocol']['recordings'][0];model='models/combined/fold-0.json'
 excluded_sid=next(r['sample_id'] for r in d['features']['rows'] if r['recording_id']==g and r['eligible'])
 changes={
  'feature_value':lambda x:x['features']['rows'][i]['vector'].__setitem__(0,x['features']['rows'][i]['vector'][0]+.1),
  'recording_alias':lambda x:x['features']['rows'][i].__setitem__('recording_id','unknown/alias'),
  'training_weight':lambda x:x['models'][model]['train_weights'].__setitem__(0,x['models'][model]['train_weights'][0]+.01),
  'training_mean':lambda x:x['models'][model]['mean'].__setitem__(0,x['models'][model]['mean'][0]+.01),
  'excluded_training_id':lambda x:x['models'][model]['train_sample_ids'].append(excluded_sid),
  'coefficient':lambda x:x['models'][model]['coefficients'].__setitem__(0,x['models'][model]['coefficients'][0]+.1),
  'objective':lambda x:x['models'][model].__setitem__('objective',x['models'][model]['objective']+.01),
  'full_fit_substitution':lambda x:x['models'].__setitem__(model,deepcopy(x['models']['models/combined/all-development.json'])),
  'prediction_margin':lambda x:x['predictions']['rows'][i]['learned_decisions']['combined'].__setitem__('margin',x['predictions']['rows'][i]['learned_decisions']['combined']['margin']+.1),
  'prediction_action':lambda x:x['predictions']['rows'][i]['actions'].__setitem__('combined','ABSTAIN'),
  'label_value':lambda x:x['labels'][sid]['rgb'].__setitem__('translation_m',x['labels'][sid]['rgb']['translation_m']+.1),
  'score_value':lambda x:x['scores']['rows'][i]['losses']['combined'].__setitem__('translation_m',x['scores']['rows'][i]['losses']['combined']['translation_m']+.1),
  'matched_population':lambda x:x['scores']['groups'][g]['matched_comparisons']['rgb_only__combined']['matched_sample_ids'].pop(),
  'missing_recording':lambda x:x['scores']['groups'].pop(g),
  'coverage_count':lambda x:x['proof']['coverage'].__setitem__('combined',267),
 }
 rejected=[]
 for name,change in changes.items():
  candidate=deepcopy(d);change(candidate)
  try:validate(candidate)
  except (AssertionError,KeyError,ValueError,OverflowError):rejected.append(name)
  else:raise AssertionError('semantic mutation accepted: '+name)
 reordered=deepcopy(d)
 def reverse_keys(v):
  if isinstance(v,dict):return {k:reverse_keys(v[k]) for k in reversed(v)}
  if isinstance(v,list):return [reverse_keys(x) for x in v]
  return v
 validate(reverse_keys(reordered))
 return {'rejected':rejected,'count':len(rejected),'positive_json_key_reorder':'PASS','hash_checks_bypassed_for_semantic_challenges':True}


if __name__=='__main__':
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--root',type=Path,required=True);parser.add_argument('--run',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
 args=parser.parse_args();data=load(args.root,args.run);result=validate(data)
 result.update({'status':'PASS_INDEPENDENT_DEVELOPMENT_AUDIT','semantic_challenges':challenges(data),'auditor_sha256':sha(Path(__file__)),
  'protocol_sha256':sha(args.run/'protocol.json'),'scores_sha256':sha(args.run/'scores.json'),'scope':'retrospective nominal-body recorded-SLAM development proxy; no reserve or production adoption'})
 with args.output.open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
 print(json.dumps(result,indent=2))
