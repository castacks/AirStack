"""EXP152 frozen depth/location controls for the existing patch-gap cue."""
import argparse,json,hashlib,math,itertools
from pathlib import Path
from collections import Counter
import numpy as np
from scipy.stats import rankdata,spearmanr
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'results/cross-recording-stereo-depth/run-001';OUT=ROOT/'results/stereo-ambiguity-controls/run-001';PLAN=ROOT/'experiments/stereo-ambiguity-controls-2026-11-05.json'
PARAMS=dict(indices=list(range(0,2000,100)),recordings=['day','night'],modalities=['rgb','thermal'],groups=['all_primary','locally_consistent'],controls=['depth','spatial','joint'],depth_edges_m=[5,10,20,40,80],grid=3,minimum_per_stratum=[30,10],minimum_raw_points=10,variable='patch_relative_gap')
RUNTIME='sha256:d3cf8a41dd8090c8939cd840b4735d526cec9a54e48ac36d506f91d13ee56e1c'
def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def write(p,v):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')
def stats(values):
 if not values:return dict(n=0,median=None,min=None,max=None)
 return dict(n=len(values),median=float(np.median(values)),min=min(values),max=max(values))
def rank_correlation(points):
 x=[p['gap'] for p in points];y=[p['error'] for p in points];reason='fewer_than_10' if len(points)<10 else 'constant' if len(set(x))<2 or len(set(y))<2 else None
 return dict(points=len(points),rho=float(spearmanr(x,y).statistic) if reason is None else None,reason=reason)
def bin_key(point,shape,control):
 depth=sum(point['reference_m']>=e for e in PARAMS['depth_edges_m']);h,w=shape;u,v=point['uv'];y=math.floor(3*v/h);x=math.floor(3*u/w)
 if not(0<=x<3 and 0<=y<3 and point['reference_m']>0):raise ValueError('source spatial/depth extent')
 return [depth] if control=='depth' else [y,x] if control=='spatial' else [depth,y,x]
def keys(control):return [[d] for d in range(6)] if control=='depth' else [list(k) for k in itertools.product(range(3),range(3))] if control=='spatial' else [list(k) for k in itertools.product(range(6),range(3),range(3))]
def calculate(points,shape,control,minimum):
 valid=[p for p in points if p['faithful'] and p['gap'] is not None and math.isfinite(p['gap']) and math.isfinite(p['error'])];bins=[];retained=[];xx=yy=xy=0.
 for key in keys(control):
  group=[p for p in valid if bin_key(p,shape,control)==key];x=[p['gap'] for p in group];y=[p['error'] for p in group];reason='empty' if not group else 'below_minimum' if len(group)<minimum else 'constant' if len(set(x))<2 or len(set(y))<2 else None;ranks=[]
  if reason is None:
   n=len(group);a=(rankdata(x,method='average')-(n+1)/2)/n;b=(rankdata(y,method='average')-(n+1)/2)/n
   xx+=float(a@a);yy+=float(b@b);xy+=float(a@b);retained.extend(p['feature_index'] for p in group);ranks=[dict(feature_index=p['feature_index'],gap_rank=float(ar),error_rank=float(br)) for p,ar,br in zip(group,a,b)]
  bins.append(dict(key=key,feature_indices=[p['feature_index'] for p in group],points=len(group),eligible=reason is None,reason=reason,ranks=ranks))
 kept=set(retained);same=[p for p in valid if p['feature_index'] in kept]
 return dict(cohort_points=len(points),valid_points=len(valid),excluded_feature_indices=[p['feature_index'] for p in points if p not in valid],full_raw=rank_correlation(valid),retained_raw=rank_correlation(same),retained_feature_indices=[p['feature_index'] for p in same],retained_points=len(same),retained_fraction=len(same)/len(valid) if valid else None,within_bin=xy/math.sqrt(xx*yy) if xx>0 and yy>0 else None,rank_sums=dict(xx=xx,yy=yy,xy=xy),eligible_bins=sum(v['eligible'] for v in bins),bins=bins)
def prepare():
 records=[];pins={}
 for r in read(BASE/'prepared.json')['records']:
  rec=r['recording'];mode=r['modality'];stem=r['stem'];df=BASE/'depth'/rec/mode/(stem+'.json');af=BASE/'ambiguity'/rec/mode/(stem+'.json');d=read(df);a=read(af);primary={p['feature_index']:p for p in d['points'] if p['reason']=='compared'};points=[]
  for p in a['points']:
   source=primary[p['feature_index']]
   if p['original_metrics']!=source['metrics'] or p['reference_group']!=source['local']['group']:raise ValueError('prior immutable join')
   points.append(dict(feature_index=p['feature_index'],uv=source['native'][:2],reference_m=source['reference_m'],error=source['metrics']['abs_relative_error'],gap=p['ambiguity']['patch_relative_gap'],faithful=p['faithful'],reference_group=p['reference_group']))
  if [p['feature_index'] for p in points]!=list(primary):raise ValueError('full primary source population')
  records.append(dict(recording=rec,modality=mode,index=r['index'],shape=r['shape'],points=points));pins[str(df.relative_to(ROOT))]=sha(df);pins[str(af.relative_to(ROOT))]=sha(af)
 write(OUT/'prepared.json',dict(records=records,source_sha256=pins));print('PREPARED80cases',sum(len(r['points']) for r in records),'original primary points')
def freeze():
 files=[Path(__file__),ROOT/'tools/stereo_ambiguity_controls_plot.py',ROOT/'tools/stereo_ambiguity_controls_audit.py',ROOT/'tools/tests/test_stereo_ambiguity_controls.py',ROOT/'docs/research/stereo-ambiguity-controls-protocol-2026-11-05.md',ROOT/'experiments/EXP-047-ms2-depth-spatial-controls.md',OUT/'prepared.json']+[BASE/n for n in ['prepared.json','summary.json','fidelity.json','independent-audit.json','study-lock.json']]
 pins={str(f.relative_to(ROOT)):sha(f) for f in files};pins.update(read(OUT/'prepared.json')['source_sha256']);p=dict(parent='6b984d4ef27423e12e07c56776c2a489ca5a2431',parameters=PARAMS,runtime=RUNTIME,source_sha256=pins,new_inference=False,new_acquisition=False,physical_reference_qualified=False,policy_fitted=False)
 write(PLAN,p);write(OUT/'study-lock.json',p);print('FROZEN',len(pins),'pins')
def verify():
 p=read(PLAN)
 if p['parameters']!=PARAMS or sha(PLAN)!=sha(OUT/'study-lock.json'):raise ValueError('study/parameter binding')
 for n,h in p['source_sha256'].items():
  if sha(ROOT/n)!=h:raise ValueError('source changed '+n)
 return p
def run():
 verify();rows=[]
 for r in read(OUT/'prepared.json')['records']:
  cases=[]
  for group in PARAMS['groups']:
   data=r['points'] if group=='all_primary' else [p for p in r['points'] if p['reference_group']=='locally_consistent']
   for control in PARAMS['controls']:
    for minimum in PARAMS['minimum_per_stratum']:
     row=dict(recording=r['recording'],modality=r['modality'],index=r['index'],group=group,control=control,minimum=minimum,result=calculate(data,r['shape'],control,minimum));cases.append(row);rows.append(row)
  write(OUT/'cases'/r['recording']/r['modality']/(f"{r['index']:06d}.json"),dict(rows=cases))
 aggregates=[]
 for rec,mode,group,control,minimum in itertools.product(PARAMS['recordings'],PARAMS['modalities'],PARAMS['groups'],PARAMS['controls'],PARAMS['minimum_per_stratum']):
  data=[r['result'] for r in rows if (r['recording'],r['modality'],r['group'],r['control'],r['minimum'])==(rec,mode,group,control,minimum)];aggregates.append(dict(recording=rec,modality=mode,group=group,control=control,minimum=minimum,image_cases=len(data),full_raw=stats([r['full_raw']['rho'] for r in data if r['full_raw']['rho'] is not None]),full_raw_defined=stats([r['full_raw']['rho'] for r in data if r['within_bin'] is not None]),retained_raw=stats([r['retained_raw']['rho'] for r in data if r['retained_raw']['rho'] is not None]),within_bin=stats([r['within_bin'] for r in data if r['within_bin'] is not None]),negative_images=sum(r['within_bin'] is not None and r['within_bin']<0 for r in data),retained_fraction_all_nonempty=stats([r['retained_fraction'] for r in data if r['retained_fraction'] is not None]),retained_fraction_defined=stats([r['retained_fraction'] for r in data if r['within_bin'] is not None]),total_valid_points=sum(r['valid_points'] for r in data),total_retained_points=sum(r['retained_points'] for r in data),bin_reasons=dict(Counter(b['reason'] or 'eligible' for r in data for b in r['bins']))))
 write(OUT/'summary.json',dict(image_pairs=80,case_control_rows=len(rows),aggregates=aggregates,new_inference=False,new_acquisition=False,physical_reference_qualified=False,policy_fitted=False,partial_spearman=False,causal_adjustment=False))
 write(OUT/'output-lock.json',dict(source_sha256={str(f.relative_to(ROOT)):sha(f) for f in [OUT/'summary.json',*sorted((OUT/'cases').rglob('*.json'))]}));verify();print('COMPLETE',len(rows),'controlled cases;',len(aggregates),'aggregates')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('action',choices=['prepare','freeze','verify','run']);a=p.parse_args();{'prepare':prepare,'freeze':freeze,'verify':verify,'run':run}[a.action]()
