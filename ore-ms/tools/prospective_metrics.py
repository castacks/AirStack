from pathlib import Path
import datetime,hashlib,json,math
import numpy as np
ROOT=Path('/root/AirStack/ore-ms');OUT=ROOT/'results/prospective-objective-evaluation/run-001';p=json.loads((OUT/'protocol.json').read_text());capture=json.loads((OUT/'capture.json').read_text());arms=list(p['models']);rows=[]
for row in capture['rows']:
 f=ROOT/row['path'];assert hashlib.sha256(f.read_bytes()).hexdigest()==row['sha256']
 with np.load(f,allow_pickle=False) as a:
  err=a['prediction'].astype(np.float64)-a['target'].astype(np.float64);sq=np.sum(err*err,axis=1);n=len(sq);oracle=np.argsort(sq,kind='stable');out={'index':row['index'],'recording_id':row['recording_id'],'cohort':row['cohort'],'pixels':n,'error_sum':float(sq.sum()),'arms':{}}
  for arm in arms:
   raw=a[arm].astype(np.float64);variance=np.exp(2*raw);norm=err/np.sqrt(variance);q=np.sum(norm**2,axis=1);trace=variance.sum(axis=1);assert np.isfinite(variance).all() and (variance>0).all();assert np.isfinite(q).all();order=np.argsort(trace,kind='stable');risks=[]
   for fraction in p['retained_fractions']:
    k=max(1,int(math.floor(fraction*n)));risks.append({'fraction':fraction,'retained':k,'selected_error_sum':float(sq[order[:k]].sum()),'oracle_error_sum':float(sq[oracle[:k]].sum()),'random_expected_error_sum':float(sq.mean()*k)})
   out['arms'][arm]={'coverage_counts':{str(level):int(np.count_nonzero(q<=-2*math.log1p(-level))) for level in p['coverage_levels']},'mahalanobis_sum':float(q.sum()),'normalized_sum':norm.sum(axis=0).tolist(),'normalized_squared_sum':(norm**2).sum(axis=0).tolist(),'variance_sum':variance.sum(axis=0).tolist(),'trace_min':float(trace.min()),'trace_max':float(trace.max()),'final_nll':float((.5*(err**2/variance+2*raw)).mean()),'risks':risks}
  rows.append(out)
def aggregate(selected):
 n=sum(r['pixels'] for r in selected);out={'pixels':n,'pairs':len(selected),'arms':{}}
 for arm in arms:
  entries=[r['arms'][arm] for r in selected];risk=[]
  for i,f in enumerate(p['retained_fractions']):
   k=sum(e['risks'][i]['retained'] for e in entries);risk.append({'fraction':f,'retained':k,**{name:sum(e['risks'][i][key] for e in entries)/k for name,key in [('selected_mse','selected_error_sum'),('oracle_mse','oracle_error_sum'),('random_mse','random_expected_error_sum')]}})
  out['arms'][arm]={'coverage':{str(l):sum(e['coverage_counts'][str(l)] for e in entries)/n for l in p['coverage_levels']},'mahalanobis_mean':sum(e['mahalanobis_sum'] for e in entries)/n,'standardized_bias':(sum(np.array(e['normalized_sum']) for e in entries)/n).tolist(),'standardized_second_moment':(sum(np.array(e['normalized_squared_sum']) for e in entries)/n).tolist(),'mean_variance':(sum(np.array(e['variance_sum']) for e in entries)/n).tolist(),'final_nll':sum(e['final_nll']*r['pixels'] for e,r in zip(entries,selected))/n,'risks':risk}
 return out
records={identity:aggregate([r for r in rows if r['recording_id']==identity]) for identity in dict.fromkeys(r['recording_id'] for r in rows)};groups={}
for group in ('prospective_only',):
 selected=[i for i in records if group=='all_three' or next(r for r in rows if r['recording_id']==i)['cohort']==group];groups[group]={'recordings':selected,'arms':{}}
 for arm in arms:
  rr=[records[i]['arms'][arm] for i in selected];groups[group]['arms'][arm]={'coverage':{str(l):sum(r['coverage'][str(l)] for r in rr)/len(rr) for l in p['coverage_levels']},'mahalanobis_mean':sum(r['mahalanobis_mean'] for r in rr)/len(rr),'risks':[{'fraction':f,**{k:sum(r['risks'][j][k] for r in rr)/len(rr) for k in ('selected_mse','oracle_mse','random_mse')}} for j,f in enumerate(p['retained_fractions'])]}
result={'status':'PASS_FROZEN_CALIBRATION_RANKING','created_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'rows':rows,'records':records,'groups':groups,'optimizer_steps':0,'candidate_promoted':False}
with (OUT/'metrics.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
for g in groups:print(g,json.dumps(groups[g]),flush=True)
