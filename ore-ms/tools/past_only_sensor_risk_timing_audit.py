"""Independent tied-rank replication and source timestamp correspondence."""
from decimal import Decimal
import json,statistics
from past_only_sensor_risk import ROOT,OUT,read,write,sha,verify
from ms2_broader_persistence_audit import correlation

def main():
 plan=verify();report=read(OUT/'persistence-replication.json');checks=0
 for s in report['series']:
  raw=s['covariance_trace'];dt=s['native_dt_s'];rate=[v/t**2 for v,t in zip(raw,dt)]
  assert s['dt_min_median_max']==[min(dt),statistics.median(dt),max(dt)];checks+=3
  for lag in [1,3,10]:
   for name,v in [('raw',raw),('rate',rate)]:
    n,r=correlation(v[:-lag],v[lag:]);saved=s['lags'][str(lag)][name];assert n==saved['n']
    if r is None:assert saved['rho'] is None
    else:assert abs(r-saved['rho'])<1e-12
    checks+=2
 for row in report['sensor_timestamp_audit']:
  path=ROOT/row['path'];assert path.exists()==row['source_available']
  if not path.exists():continue
  source=next(s for s in plan['sources'] if any(c['recording_id']==row['recording_id'] and c['window']==row['window'] for c in s['cases']));c=next(c for c in source['cases'] if c['recording_id']==row['recording_id'] and c['window']==row['window']);times=path.read_text().split();diff=[]
  for r in c['rows']:
   camera=r[row['camera']+'_'+row['side']];expected=int((Decimal(times[camera['index']])*10**9).to_integral_value());diff.append(abs(expected-camera['time_ns']));checks+=1
  assert max(diff)==row['max_absolute_serialization_ns']<=1000 and sha(path)==row['source_sha256'];checks+=2
 write(OUT/'timing-audit.json',{'status':'PASS_INDEPENDENT_TIMING_REPLICATION_AUDIT','checks':checks,'series':len(report['series']),'source_timestamp_stream_windows':len(report['sensor_timestamp_audit']),'source_sha256':sha(__file__),'replication_sha256':sha(OUT/'persistence-replication.json'),'physical_exposure_clocks_verified':False});print('PASS',checks,'timing/persistence checks')
if __name__=='__main__':main()
