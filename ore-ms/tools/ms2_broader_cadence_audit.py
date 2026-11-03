"""Independent tied-rank audit of the explicitly posthoc dt² diagnostic."""
from pathlib import Path
import json,math,statistics
from ms2_broader_persistence_audit import correlation,sha
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/ms2-broader-range/run-001'
def read(p):return json.loads(Path(p).read_text())
def main():
 report=read(OUT/'cadence-posthoc.json');plan=read(OUT/'execution-plan.json');persistence=read(OUT/'persistence.json')['windows'];labels=read(OUT/'fresh-labels.json');checks=0
 def check(x,y,saved):
  nonlocal checks
  n,r=correlation(x,y);assert n==saved['n']
  if r is None:assert saved['rho'] is None
  else:assert math.isclose(r,saved['rho'],abs_tol=1e-12)
  checks+=2
 for c in plan['cases']:
  common=[(b['rgb_left']['time_ns']-a['rgb_left']['time_ns'])/1e9 for a,b in zip(c['rows'],c['rows'][1:])]
  for cam in ['rgb','thermal']:
   saved=report['windows'][c['case']][cam];dt=[(b[cam+'_left']['time_ns']-a[cam+'_left']['time_ns'])/1e9 for a,b in zip(c['rows'],c['rows'][1:])];trace=persistence[c['case']][cam]['median_angular_covariance_trace_rad2'];rate=[v/t**2 for v,t in zip(trace,dt)]
   assert saved['recorded_native_dt_seconds_min_median_max']==[min(dt),statistics.median(dt),max(dt)];checks+=3;check(trace,dt,saved['covariance_interval_association'])
   for lag in [1,3,10]:
    r=saved['lags'][str(lag)];check(trace[:-lag],trace[lag:],r['raw_persistence']);check(rate[:-lag],rate[lag:],r['rate_persistence'])
    for metric in ['translation_m','rotation_deg']:
     errors=[labels.get(f"{c['window']}/{i}",{}).get(cam) for i in range(99)];values=[v[metric] if v is not None else None for v in errors];rates=[v/t if v is not None else None for v,t in zip(values,common)];check(rate[:-lag],values[lag:],r['rate_covariance_future_pose_error'][metric]);check(rate[:-lag],rates[lag:],r['rate_covariance_future_error_rate'][metric])
 for n,h in report['unchanged_primary_artifact_sha256'].items():assert sha(OUT/n)==h
 result={'status':'PASS_INDEPENDENT_POSTHOC_CADENCE_AUDIT','checks':checks,'source_sha256':sha(__file__),'cadence_posthoc_sha256':sha(OUT/'cadence-posthoc.json'),'primary_unchanged':True,'model_selected':False}
 with (OUT/'cadence-posthoc-audit.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
 print(result)
if __name__=='__main__':main()
