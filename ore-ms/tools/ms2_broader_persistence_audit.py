"""Independent scalar covariance medians, tied ranks, leads and future-target masks."""
from pathlib import Path
from itertools import groupby
import hashlib,json,math,statistics
import numpy as np
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/ms2-broader-range/run-001'
def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def ranks(values):
 out=[0.]*len(values);ordered=sorted(enumerate(values),key=lambda z:z[1]);offset=0
 for _,group in groupby(ordered,key=lambda z:z[1]):
  group=list(group);rank=offset+(len(group)+1)/2
  for i,_ in group:out[i]=rank
  offset+=len(group)
 return out
def correlation(x,y):
 pairs=[(a,b) for a,b in zip(x,y) if a is not None and b is not None and math.isfinite(a) and math.isfinite(b)]
 n=len(pairs)
 if n<10:return n,None
 a,b=[ranks([p[i] for p in pairs]) for i in [0,1]];av=sum(a)/n;bv=sum(b)/n;cov=math.fsum((x-av)*(y-bv) for x,y in zip(a,b));norm=math.sqrt(math.fsum((x-av)**2 for x in a)*math.fsum((y-bv)**2 for y in b));return n,cov/norm if norm else None
def main():
 result=read(OUT/'persistence.json');labels=read(OUT/'fresh-labels.json');plan=read(OUT/'execution-plan.json');checks=0
 assert result['labels_sha256']==sha(OUT/'fresh-labels.json') and not result['forecast_model_selected']
 for c in plan['cases']:
  for camera in ['rgb','thermal']:
   report=result['windows'][c['case']][camera];trace=report['median_angular_covariance_trace_rad2'];assert len(trace)==99
   for i,saved in enumerate(trace):
    path=OUT/'captures'/c['case']/'on/experts'/camera/'uncertainty'/f'pair-{i:03d}.npz'
    with np.load(path,allow_pickle=False) as a:
     assert int(a['time0_ns'])==c['rows'][i][camera+'_left']['time_ns'];K=a['K'].reshape(3,3).astype(float);v=a['grid_raw_flow_cov'].astype(float);values=[uu/K[0,0]**2+vv/K[1,1]**2 for uu,vv,uv in v];values=[x for x in values if math.isfinite(x) and x>=0]
     expected=statistics.median(values);assert math.isclose(expected,saved,rel_tol=5e-7,abs_tol=1e-15);assert len(values)==report['finite_grid_counts'][i];checks+=3
   for lag in [1,3,10]:
    rr=report['lags'][str(lag)];n,rho=correlation(trace[:-lag],trace[lag:]);assert n==rr['covariance_persistence']['n']
    if rho is None:assert rr['covariance_persistence']['rho'] is None
    else:assert math.isclose(rho,rr['covariance_persistence']['rho'],abs_tol=1e-12)
    checks+=2
    for metric in ['translation_m','rotation_deg']:
     errors=[labels.get(f"{c['window']}/{i}",{}).get(camera) for i in range(99)];target=[x[metric] if x is not None else None for x in errors[lag:]];n,rho=correlation(trace[:-lag],target);saved=rr['future_error_association'][metric];assert n==saved['n']
     if rho is None:assert saved['rho'] is None
     else:assert math.isclose(rho,saved['rho'],abs_tol=1e-12)
     checks+=2
    leads=[(c['rows'][i+lag][camera+'_left']['time_ns']-c['rows'][i+1][camera+'_left']['time_ns'])/1e9 for i in range(99-lag)]
    assert rr['target_start_lead_seconds']=={'min':min(leads),'median':statistics.median(leads),'max':max(leads)};checks+=3
 out={'status':'PASS_INDEPENDENT_COVARIANCE_PERSISTENCE_AUDIT','checks':checks,'windows':9,'sensor_series':18,'persistence_sha256':sha(OUT/'persistence.json'),'auditor_sha256':sha(__file__),'forecast_claim':False}
 with (OUT/'persistence-audit.json').open('x') as f:json.dump(out,f,indent=2);f.write('\n')
 print(out)
if __name__=='__main__':main()
