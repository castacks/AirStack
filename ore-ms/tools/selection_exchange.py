"""Descriptive saved-output set-exchange accounting; no model execution."""
from pathlib import Path
import datetime
import hashlib
import json
import math
import numpy as np
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/selection-exchange-diagnostic/run-001';PROTOCOL=ROOT/'docs/research/selection-exchange-protocol-2026-11-02.json'
sha=lambda path:hashlib.sha256(Path(path).read_bytes()).hexdigest()
p=json.loads(PROTOCOL.read_text())
for name,digest in p['pins'].items():assert sha(ROOT/name)==digest,name
OUT.mkdir(parents=True,exist_ok=False);(OUT/'protocol.json').write_bytes(PROTOCOL.read_bytes());rows=[];control_count=0
for source in p['sources']:
 capture=json.loads((ROOT/source['capture']['path']).read_text());metrics=json.loads((ROOT/source['metrics']['path']).read_text())
 for source_row,prior in zip(capture['rows'],metrics['rows']):
  path=ROOT/source_row['path'];assert sha(path)==source_row['sha256']
  with np.load(path,allow_pickle=False) as a:
   assert np.all(np.diff(a['flat_index'])>0)
   error=a['prediction'].astype('float64')-a['target'].astype('float64');loss=np.sum(error**2,axis=1);n=len(loss);assert n==source_row['pixels'] and np.isfinite(loss).all()
   tail_count=max(1,int(math.floor(p['tail_fraction']*n)));tail=np.zeros(n,dtype=bool);tail[np.argsort(-loss,kind='stable')[:tail_count]]=True
   scores={};orders={}
   for arm in p['models']:
    scores[arm]=np.exp(2*a[arm].astype('float64')).sum(1);score=scores[arm];assert np.isfinite(score).all() and (score>0).all() and score.max()<=np.finfo(np.float64).max/2
    orders[arm]=np.argsort(score,kind='stable');assert np.array_equal(orders[arm],np.argsort(2*score,kind='stable'));control_count+=1
   row={'source_group':source['group'],'source_index':source_row['index'],'recording_id':source_row['recording_id'],'source_cohort':source_row['cohort'],'sample_sha256':source_row['sample_sha256'],'array_path':source_row['path'],'array_sha256':source_row['sha256'],'pixels':n,'tail_pixels':tail_count,'arms':{}}
   for arm in p['adapted_arms']:
    ratio=np.log(scores[arm])-np.log(scores['baseline']);q=np.quantile(ratio,[.25,.5,.75]);entry={'log_trace_ratio_q25':float(q[0]),'log_trace_ratio_median':float(q[1]),'log_trace_ratio_q75':float(q[2]),'centered_log_trace_ratio_rms':float(np.sqrt(np.mean((ratio-q[1])**2))),'retentions':[]}
    for i,fraction in enumerate(p['retained_fractions']):
     k=max(1,int(math.floor(fraction*n)));baseline=np.zeros(n,dtype=bool);adapted=np.zeros(n,dtype=bool);baseline[orders['baseline'][:k]]=True;adapted[orders[arm][:k]]=True
     entering=adapted&~baseline;leaving=baseline&~adapted;common=adapted&baseline;assert entering.sum()==leaving.sum();base_sum=float(loss[baseline].sum());adapt_sum=float(loss[adapted].sum());entry_sum=float(loss[entering].sum());exit_sum=float(loss[leaving].sum())
     for label,value in [('baseline',base_sum/k),(arm,adapt_sum/k)]:assert math.isclose(value,prior['arms'][label]['risks'][i]['selected_error_sum']/k,rel_tol=2e-12,abs_tol=2e-12)
     diagnostic={'fraction':fraction,'retained':k,'entrants':int(entering.sum()),'exits':int(leaving.sum()),'common':int(common.sum()),'baseline_error_sum':base_sum,'adapted_error_sum':adapt_sum,'common_error_sum':float(loss[common].sum()),'entrant_error_sum':entry_sum,'exit_error_sum':exit_sum,'entrant_tail_pixels':int((entering&tail).sum()),'exit_tail_pixels':int((leaving&tail).sum()),'entrant_tail_error_sum':float(loss[entering&tail].sum()),'exit_tail_error_sum':float(loss[leaving&tail].sum()),'entrant_nontail_error_sum':float(loss[entering&~tail].sum()),'exit_nontail_error_sum':float(loss[leaving&~tail].sum())}
     delta=(adapt_sum-base_sum)/k;net=(entry_sum-exit_sum)/k;assert math.isclose(delta,net,rel_tol=2e-12,abs_tol=2e-12)
     diagnostic.update(delta_mse=delta,exchange_delta_mse=net,tail_delta_mse=(diagnostic['entrant_tail_error_sum']-diagnostic['exit_tail_error_sum'])/k,nontail_delta_mse=(diagnostic['entrant_nontail_error_sum']-diagnostic['exit_nontail_error_sum'])/k)
     if fraction==1:assert not diagnostic['entrants'] and not diagnostic['exits'] and delta==0
     entry['retentions'].append(diagnostic)
    row['arms'][arm]=entry
  rows.append(row);print(source['group'],source_row['index'],source_row['recording_id'],flush=True)
assert len(rows)==196

def aggregate(selected):
 result={'pairs':len(selected),'pixels':sum(r['pixels'] for r in selected),'arms':{}}
 for arm in p['adapted_arms']:
  entries=[r['arms'][arm] for r in selected];curves=[]
  for i,fraction in enumerate(p['retained_fractions']):
   rr=[v['retentions'][i] for v in entries];k=sum(v['retained'] for v in rr);entrants=sum(v['entrants'] for v in rr);exits=sum(v['exits'] for v in rr);entry_loss=sum(v['entrant_error_sum'] for v in rr);exit_loss=sum(v['exit_error_sum'] for v in rr)
   curves.append({'fraction':fraction,'retained':k,'entrants':entrants,'exits':exits,'turnover_fraction':entrants/k,'baseline_mse':sum(v['baseline_error_sum'] for v in rr)/k,'adapted_mse':sum(v['adapted_error_sum'] for v in rr)/k,'delta_mse':(entry_loss-exit_loss)/k,'tail_delta_mse':sum(v['entrant_tail_error_sum']-v['exit_tail_error_sum'] for v in rr)/k,'nontail_delta_mse':sum(v['entrant_nontail_error_sum']-v['exit_nontail_error_sum'] for v in rr)/k,'entrant_mean_squared_error':entry_loss/entrants if entrants else None,'exit_mean_squared_error':exit_loss/exits if exits else None,'entrant_tail_pixels':sum(v['entrant_tail_pixels'] for v in rr),'exit_tail_pixels':sum(v['exit_tail_pixels'] for v in rr)})
  result['arms'][arm]={'mean_pair_median_log_trace_ratio':sum(v['log_trace_ratio_median'] for v in entries)/len(entries),'mean_pair_iqr_log_trace_ratio':sum(v['log_trace_ratio_q75']-v['log_trace_ratio_q25'] for v in entries)/len(entries),'mean_pair_centered_log_trace_ratio_rms':sum(v['centered_log_trace_ratio_rms'] for v in entries)/len(entries),'retentions':curves}
 return result
records={identity:aggregate([r for r in rows if r['recording_id']==identity]) for identity in dict.fromkeys(r['recording_id'] for r in rows)};groups={}
for source_group in [s['group'] for s in p['sources']]+['all_thirteen']:
 ids=[identity for identity in records if source_group=='all_thirteen' or next(r for r in rows if r['recording_id']==identity)['source_group']==source_group];group={'recordings':ids,'micro':aggregate([r for r in rows if r['recording_id'] in ids]),'macro':{}}
 for arm in p['adapted_arms']:
  group['macro'][arm]={'retentions':[{key:sum(records[identity]['arms'][arm]['retentions'][i][key] for identity in ids)/len(ids) for key in ('fraction','baseline_mse','adapted_mse','delta_mse','tail_delta_mse','nontail_delta_mse','turnover_fraction')} for i in range(len(p['retained_fractions']))]}
 groups[source_group]=group
result={'status':'PASS_SAVED_SELECTION_EXCHANGE','created_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'protocol_sha256':sha(PROTOCOL),'rows':rows,'records':records,'groups':groups,'uniform_scale_exact_order_controls':control_count,'optimizer_steps':0,'new_inference_calls':0,'new_payloads':0,'candidate_promoted':False}
with (OUT/'diagnostic.json').open('x') as stream:json.dump(result,stream,indent=2);stream.write('\n')
print('PASS',len(rows),'pairs',len(records),'records',control_count,'scale controls',flush=True)
