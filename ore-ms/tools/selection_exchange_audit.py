"""Independent threshold/tie, raw-sum and aggregation audit of saved exchanges."""
from pathlib import Path
import copy
import hashlib
import json
import math
import numpy as np
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/selection-exchange-diagnostic/run-001'
p=json.loads((OUT/'protocol.json').read_text());d=json.loads((OUT/'diagnostic.json').read_text());checks=0

def check(value,label):
 global checks
 if not value:raise AssertionError(label)
 checks+=1

def close(a,b,label):check(math.isclose(a,b,rel_tol=2e-12,abs_tol=2e-12),label)

def lower(values,k):
 threshold=np.partition(values,k-1)[k-1];chosen=values<threshold;ties=np.flatnonzero(values==threshold);chosen[ties[:k-int(chosen.sum())]]=True
 return chosen,threshold

for name,digest in p['pins'].items():check(hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name)
check(len(d['rows'])==196 and len(d['records'])==13,'completepopulation');check(d['uniform_scale_exact_order_controls']==784,'allscalarcontrols')
seen=set();control_count=0
for row in d['rows']:
 key=row['recording_id'],row['sample_sha256'];check(key not in seen,'noduplicates');seen.add(key)
 with np.load(ROOT/row['array_path'],allow_pickle=False) as a:
  residual=a['prediction'].astype('float64')-a['target'].astype('float64');loss=np.einsum('ij,ij->i',residual,residual);n=len(loss);check(n==row['pixels'],'samepopulation');check(np.all(np.diff(a['flat_index'])>0),'nativetieorder');q=max(1,int(p['tail_fraction']*n));tail,_=lower(-loss,q);check(q==row['tail_pixels'],'tailmass')
  score={arm:np.exp(2*a[arm].astype('float64')).sum(1) for arm in p['models']}
  for arm,v in score.items():check(np.array_equal((2*v)/2,v) and np.isfinite(2*v).all() and (v>0).all(),'exactpower2scaling');control_count+=1
  for arm in p['adapted_arms']:
   entry=row['arms'][arm];logratio=np.log(score[arm])-np.log(score['baseline']);ordered=np.sort(logratio)
   values=[]
   for fraction in [.25,.5,.75]:
    index=(n-1)*fraction;lo=int(index);hi=int(math.ceil(index));values.append(float(ordered[lo]+(ordered[hi]-ordered[lo])*(index-lo)))
   for key,value in zip(('log_trace_ratio_q25','log_trace_ratio_median','log_trace_ratio_q75'),values):close(entry[key],value,'logratioquantile')
   close(entry['centered_log_trace_ratio_rms'],float(np.sqrt(np.dot(logratio-values[1],logratio-values[1])/n)),'spatialdeparture')
   for i,fraction in enumerate(p['retained_fractions']):
    result=entry['retentions'][i];k=max(1,int(n*fraction));baseline,cutoff=lower(score['baseline'],k);adapted,adapt_cutoff=lower(score[arm],k)
    for label,mask,t in [('baseline',baseline,cutoff),(arm,adapted,adapt_cutoff)]:
     scaled=2*score[label];same=scaled<2*t;ties=np.flatnonzero(scaled==2*t);same[ties[:k-int(same.sum())]]=True;check(np.array_equal(mask,same),'scalarselectioninvariance')
    plus=adapted&~baseline;minus=baseline&~adapted;shared=adapted&baseline
    for key,expected in [('retained',k),('entrants',int(plus.sum())),('exits',int(minus.sum())),('common',int(shared.sum())),('entrant_tail_pixels',int((plus&tail).sum())),('exit_tail_pixels',int((minus&tail).sum()))]:check(result[key]==expected,'exact/'+key)
    for key,mask in [('baseline_error_sum',baseline),('adapted_error_sum',adapted),('common_error_sum',shared),('entrant_error_sum',plus),('exit_error_sum',minus),('entrant_tail_error_sum',plus&tail),('exit_tail_error_sum',minus&tail),('entrant_nontail_error_sum',plus&~tail),('exit_nontail_error_sum',minus&~tail)]:close(result[key],float(loss[mask].sum()),'sum/'+key)
    close(result['adapted_error_sum'],result['common_error_sum']+result['entrant_error_sum'],'adaptedpartition');close(result['baseline_error_sum'],result['common_error_sum']+result['exit_error_sum'],'originalpartition')
    close(result['delta_mse'],(float(loss[adapted].sum())-float(loss[baseline].sum()))/k,'riskdelta');close(result['delta_mse'],result['exchange_delta_mse'],'exchangeidentity');close(result['delta_mse'],result['tail_delta_mse']+result['nontail_delta_mse'],'tailpartition')
    if fraction==1:check(not plus.any() and not minus.any() and result['delta_mse']==0,'fullretentionzero')
check(control_count==784,'allcontrolpopulation')

def aggregate_check(value):
 for identity,record in value['records'].items():
  rr=[r for r in value['rows'] if r['recording_id']==identity];check(len(rr)==record['pairs'],'recordpopulation')
  for arm in p['adapted_arms']:
   for i,fraction in enumerate(p['retained_fractions']):
    entries=[r['arms'][arm]['retentions'][i] for r in rr];n=sum(r['retained'] for r in entries);entry_n=sum(r['entrants'] for r in entries);exit_n=sum(r['exits'] for r in entries);entry_sum=sum(r['entrant_error_sum'] for r in entries);exit_sum=sum(r['exit_error_sum'] for r in entries);summary=record['arms'][arm]['retentions'][i]
    check(summary['retained']==n and summary['entrants']==entry_n==exit_n==summary['exits'],'pooledcounts');close(summary['turnover_fraction'],entry_n/n,'turnover');close(summary['baseline_mse'],sum(r['baseline_error_sum'] for r in entries)/n,'originalrisk');close(summary['adapted_mse'],sum(r['adapted_error_sum'] for r in entries)/n,'adaptedrisk');close(summary['delta_mse'],(entry_sum-exit_sum)/n,'pooledidentity')
    for key,source in [('tail_delta_mse','tail'),('nontail_delta_mse','nontail')]:close(summary[key],sum(r[f'entrant_{source}_error_sum']-r[f'exit_{source}_error_sum'] for r in entries)/n,'pooledtail')
    if entry_n:close(summary['entrant_mean_squared_error'],entry_sum/entry_n,'entrantmean');close(summary['exit_mean_squared_error'],exit_sum/exit_n,'exitmean')
    else:check(summary['entrant_mean_squared_error'] is None and summary['exit_mean_squared_error'] is None,'emptyexchangeexplicit')
 for group,g in value['groups'].items():
  ids=g['recordings'];check(len(ids)==len(set(ids)),'grouprecorduniqueness')
  for arm in p['adapted_arms']:
   for i in range(5):
    for key in ('baseline_mse','adapted_mse','delta_mse','tail_delta_mse','nontail_delta_mse','turnover_fraction'):close(g['macro'][arm]['retentions'][i][key],sum(value['records'][identity]['arms'][arm]['retentions'][i][key] for identity in ids)/len(ids),'macro/'+key)
    k=sum(value['records'][identity]['arms'][arm]['retentions'][i]['retained'] for identity in ids)
    for key in ('baseline_mse','adapted_mse','delta_mse','tail_delta_mse','nontail_delta_mse','turnover_fraction'):close(g['micro']['arms'][arm]['retentions'][i][key],sum(value['records'][identity]['arms'][arm]['retentions'][i][key]*value['records'][identity]['arms'][arm]['retentions'][i]['retained'] for identity in ids)/k,'micro/'+key)
aggregate_check(d)
mutations=[]
for name in ('exchange_sign','retained_mass','tail_cost','record_macro'):
 corrupt=copy.deepcopy(d)
 if name=='record_macro':corrupt['groups']['expanded_eight']['macro']['recurrent_diagonal_nll']['retentions'][2]['delta_mse']+=.01
 else:
  first=corrupt['records'][next(iter(corrupt['records']))]['arms']['recurrent_diagonal_nll']['retentions'][2]
  if name=='exchange_sign':first['delta_mse']+=.01
  elif name=='retained_mass':first['entrants']+=1
  else:first['tail_delta_mse']+=.01
 try:aggregate_check(corrupt)
 except AssertionError:mutations.append({'name':name,'rejected':True})
 else:raise AssertionError('escaped mutation: '+name)
for name,digest in p['pins'].items():check(hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,'final/'+name)
with (OUT/'audit.json').open('x') as stream:json.dump({'status':'PASS','checks':checks,'mutations':mutations,'pairs':196,'records':13,'uniform_scale_controls':control_count,'source_arrays_unchanged':True,'optimizer_steps':0,'new_inference_calls':0},stream,indent=2);stream.write('\n')
print('PASS',checks,'checks',len(mutations),'mutations')
