"""Posthoc fixed dt² rescaling; never changes primary predictions or results."""
from pathlib import Path
import json,hashlib
import numpy as np
from scipy.stats import spearmanr
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/ms2-broader-range/run-001'
def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def corr(x,y):
 x,y=np.asarray(x,float),np.asarray(y,float);mask=np.isfinite(x)&np.isfinite(y);x,y=x[mask],y[mask]
 return {'n':len(x),'rho':float(spearmanr(x,y).statistic) if len(x)>=10 and np.ptp(x)>0 and np.ptp(y)>0 else None}
def main():
 plan=read(OUT/'execution-plan.json');persistence=read(OUT/'persistence.json');labels=read(OUT/'fresh-labels.json');result={}
 before={n:sha(OUT/n) for n in ['locked-actions.json','prediction-lock.json','scores.json','persistence.json']}
 for c in plan['cases']:
  window={};common=np.diff(np.asarray([r['rgb_left']['time_ns'] for r in c['rows']],np.int64))/1e9
  for cam in ['rgb','thermal']:
   dt=np.diff(np.asarray([r[cam+'_left']['time_ns'] for r in c['rows']],np.int64))/1e9;trace=np.asarray(persistence['windows'][c['case']][cam]['median_angular_covariance_trace_rad2']);rate=trace/dt**2
   errors={m:np.asarray([labels.get(f"{c['window']}/{i}",{}).get(cam,{}).get(m,np.nan) if labels.get(f"{c['window']}/{i}",{}).get(cam) is not None else np.nan for i in range(99)],float) for m in ['translation_m','rotation_deg']}
   lags={}
   for lag in [1,3,10]:lags[str(lag)]={'raw_persistence':corr(trace[:-lag],trace[lag:]),'rate_persistence':corr(rate[:-lag],rate[lag:]),'rate_covariance_future_pose_error':{m:corr(rate[:-lag],v[lag:]) for m,v in errors.items()},'rate_covariance_future_error_rate':{m:corr(rate[:-lag],(v/common)[lag:]) for m,v in errors.items()}}
   window[cam]={'recorded_native_dt_seconds_min_median_max':np.quantile(dt,[0,.5,1]).tolist(),'covariance_interval_association':corr(trace,dt),'lags':lags}
  result[c['case']]=window
 assert all(sha(OUT/n)==h for n,h in before.items())
 report={'scope':'posthoc, all windows; no primary change or selected predictor','declaration_sha256':sha(ROOT/'docs/research/ms2-broader-cadence-posthoc-2026-11-02.md'),'source_sha256':sha(__file__),'unchanged_primary_artifact_sha256':before,'physical_exposure_intervals_verified':False,'windows':result}
 with (OUT/'cadence-posthoc.json').open('x') as f:json.dump(report,f,indent=2,allow_nan=False);f.write('\n')
 print('POSTHOC CADENCE all9windows, fixed dt² and lags1/3/10; primary hashes unchanged')
if __name__=='__main__':main()
