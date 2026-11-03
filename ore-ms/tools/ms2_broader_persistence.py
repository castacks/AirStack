"""Predeclared descriptive covariance persistence; no model fitting or selection."""
from pathlib import Path
import argparse,json,hashlib
import numpy as np
from scipy.stats import spearmanr
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/ms2-broader-range/run-001';DECL=ROOT/'docs/research/ms2-broader-range-persistence-declaration-2026-11-02.md'
def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,x):
 with Path(p).open('x') as f:json.dump(x,f,indent=2,allow_nan=False);f.write('\n')
def freeze():
 assert not (OUT/'fresh-labels.json').exists() and not (OUT/'scores.json').exists()
 write(OUT/'persistence-declaration-lock.json',{'declaration_sha256':sha(DECL),'source_sha256':sha(__file__),'sampling_sha256':sha(ROOT/'experiments/ms2-broader-range-2026-11-02.json'),'execution_sha256':sha(OUT/'execution-plan.json'),'new_labels_computed':False,'primary_changed':False})
 print('FROZEN supplementary diagnostics before new labels')
def corr(x,y):
 x=np.asarray(x,float);y=np.asarray(y,float);mask=np.isfinite(x)&np.isfinite(y);x=x[mask];y=y[mask]
 return {'n':len(x),'rho':float(spearmanr(x,y).statistic) if len(x)>=10 and np.ptp(x)>0 and np.ptp(y)>0 else None}
def run():
 lock=read(OUT/'persistence-declaration-lock.json');assert lock['source_sha256']==sha(__file__) and lock['declaration_sha256']==sha(DECL) and lock['execution_sha256']==sha(OUT/'execution-plan.json')
 proof=read(OUT/'prediction-lock.json');assert proof['actions_sha256']==sha(OUT/'locked-actions.json') and proof['labels_read'] is False
 plan=read(OUT/'execution-plan.json');labels=read(OUT/'fresh-labels.json');actions=read(OUT/'locked-actions.json')['rows'];windows={}
 for c in plan['cases']:
  n=c['frames']-1;window={};raw=[]
  for camera in ['rgb','thermal']:
   trace=np.full(n,np.nan);counts=[];errors={m:np.full(n,np.nan) for m in ['translation_m','rotation_deg']}
   for i in range(n):
    path=OUT/'captures'/c['case']/'on/experts'/camera/'uncertainty'/f'pair-{i:03d}.npz'
    with np.load(path,allow_pickle=False) as a:
     assert int(a['time0_ns'])==c['rows'][i][camera+'_left']['time_ns'];K=a['K'].reshape(3,3);v=a['grid_raw_flow_cov'];values=v[:,0]/K[0,0]**2+v[:,1]/K[1,1]**2;good=values[np.isfinite(values)&(values>=0)];counts.append(len(good))
     if len(good):trace[i]=np.median(good)
    loss=labels.get(f"{c['window']}/{i}",{}).get(camera)
    if loss:
     for m in errors:errors[m][i]=loss[m]
   times=[r[camera+'_left']['time_ns'] for r in c['rows']];lags={}
   for lag in [1,3,10]:
    leads=[(times[i+lag]-times[i+1])/1e9 for i in range(n-lag)]
    lags[str(lag)]={'covariance_persistence':corr(trace[:-lag],trace[lag:]),'future_error_association':{m:corr(trace[:-lag],v[lag:]) for m,v in errors.items()},'target_start_lead_seconds':{'min':min(leads),'median':float(np.median(leads)),'max':max(leads)}}
   window[camera]={'median_angular_covariance_trace_rad2':trace.tolist(),'finite_grid_counts':counts,'lags':lags};assert np.isfinite(trace).all()
  runs=[];last_i=None;last=None;length=0
  for r in [r for r in actions if r['window']==c['window']]:
   i=r['pair_ordinal'];l=labels.get(r['sample_id']);winner=None
   if r['both_supported'] and l:
    a,b=l['rgb']['translation_m'],l['thermal']['translation_m'];winner='rgb' if a<b else 'thermal' if b<a else 'tie'
   if winner is None or winner!=last or last_i is None or i!=last_i+1:
    if length:runs.append({'winner':last,'pairs':length})
    length=0
   if winner:length+=1
   last,last_i=winner,i
  if length:runs.append({'winner':last,'pairs':length})
  window['translation_winner_runs']=runs;windows[c['case']]=window
 write(OUT/'persistence.json',{'windows':windows,'declaration_lock_sha256':sha(OUT/'persistence-declaration-lock.json'),'prediction_lock_sha256':sha(OUT/'prediction-lock.json'),'labels_sha256':sha(OUT/'fresh-labels.json'),'forecast_model_selected':False,'wall_clock_deadline_verified':False});print('PERSISTENCE',len(windows),'windows; fixed lags 1/3/10')
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['freeze','run']);a=p.parse_args();globals()[a.action]()
