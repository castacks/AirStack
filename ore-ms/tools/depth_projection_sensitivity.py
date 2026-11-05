"""EXP148 reference-only local projection sensitivity, preserving primary errors."""
import argparse,hashlib,json,math
from pathlib import Path
import cv2
import numpy as np
from native_depth_proxy import stats,METRICS
ROOT=Path(__file__).resolve().parents[1];PRIOR=ROOT/'results/native-depth-proxy/run-001';OUT=ROOT/'results/depth-projection-sensitivity/run-001';PLAN=ROOT/'experiments/depth-projection-sensitivity-2026-11-05.json'
PARAMS=dict(radii=[1,2],minimum_candidates=3,maximum_relative_span=.10,groups=['insufficient','locally_consistent','mixed_depth'])
def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def write(p,v):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')

def neighborhood(feature,depth,fx,baseline,radius):
 u,v,_,right,native,_,_=feature;x=math.floor(u+.5);y=math.floor(v+.5);candidates=[];rejected=dict(outside=0,zero=0,proxy_right_outside=0)
 for yy in range(y-radius,y+radius+1):
  for xx in range(x-radius,x+radius+1):
   if not(0<=xx<depth.shape[1] and 0<=yy<depth.shape[0]):rejected['outside']+=1;continue
   code=int(depth[yy,xx])
   if not code:rejected['zero']+=1;continue
   z=code/256
   if not(0<=u-fx*baseline/z<depth.shape[1]-1):rejected['proxy_right_outside']+=1;continue
   candidates.append([xx,yy,code])
 values=[v[2]/256 for v in candidates]
 if not values:return dict(center=[x,y],candidates=candidates,rejected=rejected,group='insufficient',reference_min_m=None,reference_max_m=None,reference_median_m=None,relative_span=None,ratio_lower=None,ratio_upper=None,interval_class='unavailable')
 low=min(values);high=max(values);median=float(np.median(values));span=(high-low)/median
 group='insufficient' if len(values)<3 else 'locally_consistent' if span<=.10 else 'mixed_depth'
 lower=native/high;upper=native/low;classification='short_for_all' if upper<1 else 'long_for_all' if lower>1 else 'contains_unit'
 return dict(center=[x,y],candidates=candidates,rejected=rejected,group=group,reference_min_m=low,reference_max_m=high,reference_median_m=median,relative_span=span,ratio_lower=lower,ratio_upper=upper,interval_class=classification)

def freeze():
 source=read(PRIOR/'native-selected.json');acq=read(PRIOR/'acquisition.json');files=[Path(__file__),ROOT/'tools/depth_projection_sensitivity_audit.py',ROOT/'tools/tests/test_depth_projection_sensitivity.py',ROOT/'tools/native_depth_proxy.py',ROOT/'tools/native_depth_proxy_audit.py',ROOT/'tools/ms2_broader_intake.py',ROOT/'docs/research/depth-projection-sensitivity-protocol-2026-11-05.md',PRIOR/'summary.json',PRIOR/'native-selected.json',PRIOR/'acquisition.json',PRIOR/'independent-audit.json',PRIOR/'study-lock.json',PRIOR/'source/qualification.json']
 for r in source['records']:files.append(PRIOR/'points'/r['modality']/(r['stem']+'.json'))
 for member in acq['members']:
  f=ROOT/member['path']
  if sha(f)!=member['sha256']:raise ValueError('original label pin')
  files.append(f)
 p=dict(parent='229eeb87ffe2c688e51bf46dbb301db3a6e72751',parameters=PARAMS,source_sha256={str(f.relative_to(ROOT)):sha(f) for f in files},runtime='sha256:d3cf8a41dd8090c8939cd840b4735d526cec9a54e48ac36d506f91d13ee56e1c',physical_reference_qualified=False,physical_sync_verified=False,benchmark_execution_ready=False,scale_fitted=False,time_fitted=False)
 write(PLAN,p);write(OUT/'study-lock.json',p);print('FROZEN',len(p['source_sha256']),'pins')

def verify():
 p=read(PLAN)
 if p['parameters']!=PARAMS or sha(PLAN)!=sha(OUT/'study-lock.json'):raise ValueError('study lock/parameters')
 for n,h in p['source_sha256'].items():
  if sha(ROOT/n)!=h:raise ValueError('changed input '+n)
 return p

def reduce_points(points):
 return dict(points=len(points),metrics={key:stats([p['original_metrics'][key] for p in points]) for key in METRICS},interval_counts={key:sum(p['interval_class']==key for p in points) for key in ['short_for_all','long_for_all','contains_unit','unavailable']},ratio_lower=stats([p['ratio_lower'] for p in points if p['ratio_lower'] is not None]),ratio_upper=stats([p['ratio_upper'] for p in points if p['ratio_upper'] is not None]))

def run():
 verify();source=read(PRIOR/'native-selected.json');acq=read(PRIOR/'acquisition.json');members={v['name']:v for v in acq['members']};cases=[]
 for r in source['records']:
  name=source['sequence']+'/'+r['camera']+'/depth/'+r['stem']+'.png';depth=cv2.imread(str(ROOT/members[name]['path']),-1);old=read(PRIOR/'points'/r['modality']/(r['stem']+'.json'));primary=[p for p in old['points'] if p['reason']=='compared']
  for radius in PARAMS['radii']:
   points=[]
   for p in primary:
    result=neighborhood(p['native'],depth,r['K'][0][0],r['baseline_m'],radius)
    if not any(v[:2]==p['pixel'] and v[2]==p['depth_code'] for v in result['candidates']):raise ValueError('primary central pixel lost')
    result.update(feature_index=p['feature_index'],original_metrics=p['metrics'],native=p['native']);points.append(result)
   groups={group:reduce_points([p for p in points if p['group']==group]) for group in PARAMS['groups']};case=dict(modality=r['modality'],index=r['index'],stem=r['stem'],radius=radius,primary_points=len(primary),all=reduce_points(points),groups=groups)
   write(OUT/'points'/r['modality']/(r['stem']+'-r'+str(radius)+'.json'),dict(case=case,points=points));cases.append(case)
 agg=[]
 for mode in ['rgb','thermal']:
  for radius in PARAMS['radii']:
   rows=[c for c in cases if c['modality']==mode and c['radius']==radius];groups={}
   for name in ['all']+PARAMS['groups']:
    data=[c['all'] if name=='all' else c['groups'][name] for c in rows]
    groups[name]=dict(points=sum(v['points'] for v in data),images_nonempty=sum(v['points']>0 for v in data),images_empty=sum(v['points']==0 for v in data),metrics={k:stats([v['metrics'][k]['median'] for v in data if v['metrics'][k]['median'] is not None]) for k in METRICS},interval_counts={k:sum(v['interval_counts'][k] for v in data) for k in ['short_for_all','long_for_all','contains_unit','unavailable']})
   agg.append(dict(modality=mode,radius=radius,images=20,groups=groups))
 write(OUT/'summary.json',dict(cases=cases,aggregates=agg,physical_reference_qualified=False,physical_sync_verified=False,benchmark_execution_ready=False,scale_fitted=False,time_fitted=False,source_sha256=sha(PLAN),primary_errors_changed=False))
 verify();print('COMPLETE80image/radius cases')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('action',choices=['freeze','verify','run']);x=p.parse_args();{'freeze':freeze,'verify':verify,'run':run}[x.action]()
