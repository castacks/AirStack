"""EXP152 independent source joins, pair-count midranks and stratified audit."""
import json,hashlib,math,itertools
from pathlib import Path
from collections import Counter
from copy import deepcopy
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'results/cross-recording-stereo-depth/run-001';OUT=ROOT/'results/stereo-ambiguity-controls/run-001';COUNT=0
CONTROLS=['depth','spatial','joint'];GROUPS=['all_primary','locally_consistent']
def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def need(v,label):
 global COUNT
 COUNT+=1
 if not v:raise ValueError(label)
def close(a,b,label):
 if a is None or b is None:need(a is b,label)
 else:need(math.isclose(a,b,rel_tol=2e-10,abs_tol=2e-10),label)
def midranks(values):return [1+sum(y<x for y in values)+.5*(sum(y==x for y in values)-1) for x in values]
def dot(x,y):return sum(a*b for a,b in zip(x,y))
def correlation(data):
 x=[p['gap'] for p in data];y=[p['error'] for p in data]
 reason='fewer_than_10' if len(data)<10 else 'constant' if len(set(x))<2 or len(set(y))<2 else None
 if reason:return dict(points=len(data),rho=None,reason=reason)
 a=midranks(x);b=midranks(y);center=(len(data)+1)/2;a=[v-center for v in a];b=[v-center for v in b]
 return dict(points=len(data),rho=dot(a,b)/math.sqrt(dot(a,a)*dot(b,b)),reason=None)
def key(p,shape,control):
 z=p['reference_m'];band=next((i for i,upper in enumerate([5,10,20,40,80]) if z<upper),5);u,v=p['uv'];h,w=shape;y=0 if v<h/3 else 1 if v<2*h/3 else 2;x=0 if u<w/3 else 1 if u<2*w/3 else 2
 return [band] if control=='depth' else [y,x] if control=='spatial' else [band,y,x]
def possible(control):return [[i] for i in range(6)] if control=='depth' else [list(v) for v in itertools.product(range(3),range(3))] if control=='spatial' else [list(v) for v in itertools.product(range(6),range(3),range(3))]
def check_correlation(saved,data,label):
 want=correlation(data);need(saved['points']==want['points'] and saved['reason']==want['reason'],label+' population/availability');close(saved['rho'],want['rho'],label+' rho')
def check_result(saved,points,shape,control,minimum):
 valid=[p for p in points if p['faithful'] and p['gap'] is not None and math.isfinite(p['gap']) and math.isfinite(p['error'])];need(saved['cohort_points']==len(points) and saved['valid_points']==len(valid),'cohort/variable population');need(saved['excluded_feature_indices']==[p['feature_index'] for p in points if p not in valid],'explicit invalid variables');check_correlation(saved['full_raw'],valid,'full')
 want_keys=possible(control);need([b['key'] for b in saved['bins']]==want_keys,'all fixed bins');retained=[];xx=yy=xy=0.;eligible=0
 for b,k in zip(saved['bins'],want_keys):
  group=[p for p in valid if key(p,shape,control)==k];need(b['feature_indices']==[p['feature_index'] for p in group] and b['points']==len(group),'exact independent bin membership')
  x=[p['gap'] for p in group];y=[p['error'] for p in group];reason='empty' if len(group)==0 else 'below_minimum' if len(group)<minimum else 'constant' if len(set(x))<2 or len(set(y))<2 else None
  need(b['reason']==reason and b['eligible'] is (reason is None),'predeclared bin eligibility')
  if reason is not None:need(b['ranks']==[],'unavailable bin has no ranks');continue
  eligible+=1;retained.extend(p['feature_index'] for p in group);n=len(group);a=[(v-(n+1)/2)/n for v in midranks(x)];c=[(v-(n+1)/2)/n for v in midranks(y)];need(len(b['ranks'])==n,'all normalized rank slots')
  for rank,p,ar,br in zip(b['ranks'],group,a,c):
   need(rank['feature_index']==p['feature_index'],'rank source identity');close(rank['gap_rank'],ar,'pair-count cue midrank');close(rank['error_rank'],br,'pair-count error midrank')
  xx+=dot(a,a);yy+=dot(c,c);xy+=dot(a,c)
 same=[p for p in valid if p['feature_index'] in set(retained)];need(saved['retained_feature_indices']==[p['feature_index'] for p in same] and saved['retained_points']==len(same),'retained source population')
 close(saved['retained_fraction'],len(same)/len(valid) if valid else None,'retention fraction');need(saved['eligible_bins']==eligible,'eligible bin total');check_correlation(saved['retained_raw'],same,'same-retained')
 for field,want in [('xx',xx),('yy',yy),('xy',xy)]:close(saved['rank_sums'][field],want,'independent rank sum '+field)
 close(saved['within_bin'],xy/math.sqrt(xx*yy) if xx>0 and yy>0 else None,'stratified rank coefficient')
def check_stats(saved,values):
 need(saved['n']==len(values),'equal-image statistic population')
 if not values:need(saved==dict(n=0,median=None,min=None,max=None),'null empty aggregate');return
 ordered=sorted(values);n=len(values);median=(ordered[(n-1)//2]+ordered[n//2])/2
 for k,v in [('median',median),('min',min(values)),('max',max(values))]:close(saved[k],v,'scalar image '+k)
def audit():
 plan=read(ROOT/'experiments/stereo-ambiguity-controls-2026-11-05.json');need(sha(ROOT/'experiments/stereo-ambiguity-controls-2026-11-05.json')==sha(OUT/'study-lock.json'),'study lock')
 for lock in [plan,read(OUT/'output-lock.json')]:
  for n,h in lock['source_sha256'].items():need(sha(ROOT/n)==h,'frozen source/output '+n)
 prepared=read(OUT/'prepared.json');summary=read(OUT/'summary.json');prior=read(BASE/'summary.json');need(len(prepared['records'])==summary['image_pairs']==80 and summary['case_control_rows']==960 and len(summary['aggregates'])==48,'all frozen output populations')
 for k in ['new_inference','new_acquisition','physical_reference_qualified','policy_fitted','partial_spearman','causal_adjustment']:need(summary[k] is False,'scope '+k)
 allrows=[];sample=None;total=0
 for r in prepared['records']:
  rec=r['recording'];mode=r['modality'];stem=f"{r['index']:06d}";depth=read(BASE/'depth'/rec/mode/(stem+'.json'));amb=read(BASE/'ambiguity'/rec/mode/(stem+'.json'));metadata=next(v for v in read(BASE/'prepared.json')['records'] if (v['recording'],v['modality'],v['index'])==(rec,mode,r['index']));need(r['shape']==metadata['shape'],'source dimensions');primary={v['feature_index']:v for v in depth['points'] if v['reason']=='compared'}
  need([v['feature_index'] for v in r['points']]==[v['feature_index'] for v in amb['points']]==list(primary),'complete original source IDs')
  for p,q in zip(r['points'],amb['points']):
   d=primary[p['feature_index']];need(p['uv']==d['native'][:2] and p['reference_m']==d['reference_m'] and p['error']==d['metrics']['abs_relative_error'],'original geometry/depth/error unchanged');need(p['gap']==q['ambiguity']['patch_relative_gap'] and p['faithful']==q['faithful'] and p['reference_group']==q['reference_group']==d['local']['group'],'original gap/fidelity/group unchanged');need(p['reference_m']>0 and 0<=p['uv'][0]<r['shape'][1] and 0<=p['uv'][1]<r['shape'][0],'valid native geometry');total+=1
  rows=read(OUT/'cases'/rec/mode/(stem+'.json'))['rows'];need(len(rows)==12,'all cohort/control/threshold cases');wantlabels=list(itertools.product(GROUPS,CONTROLS,[30,10]));need([(v['group'],v['control'],v['minimum']) for v in rows]==wantlabels,'frozen output order')
  for row in rows:
   need((row['recording'],row['modality'],row['index'])==(rec,mode,r['index']),'case source identity');data=r['points'] if row['group']=='all_primary' else [p for p in r['points'] if p['reference_group']=='locally_consistent'];check_result(row['result'],data,r['shape'],row['control'],row['minimum']);old=next(c for c in prior['ambiguity']['cases'] if (c['recording'],c['modality'],c['index'])==(rec,mode,r['index']))['groups'][row['group']]['correlations']['patch_relative_gap'];need(row['result']['full_raw']['points']==old['points'] and row['result']['full_raw']['reason']==old['reason'],'EXP151 raw support reproduced');close(row['result']['full_raw']['rho'],old['rho'],'EXP151 raw rho reproduced');allrows.append(row)
   if sample is None and row['control']=='depth' and row['result']['within_bin'] is not None:sample=(row,data,r['shape'])
 need(total==2938,'all original2938primary points')
 for a in summary['aggregates']:
  data=[r['result'] for r in allrows if all(r[k]==a[k] for k in ['recording','modality','group','control','minimum'])];need(a['image_cases']==len(data)==20,'all20image aggregate cases')
  for key2 in ['full_raw','retained_raw']:check_stats(a[key2],[r[key2]['rho'] for r in data if r[key2]['rho'] is not None])
  check_stats(a['full_raw_defined'],[r['full_raw']['rho'] for r in data if r['within_bin'] is not None]);check_stats(a['within_bin'],[r['within_bin'] for r in data if r['within_bin'] is not None]);need(a['negative_images']==sum(r['within_bin'] is not None and r['within_bin']<0 for r in data),'negative coefficient images');check_stats(a['retained_fraction_all_nonempty'],[r['retained_fraction'] for r in data if r['retained_fraction'] is not None]);check_stats(a['retained_fraction_defined'],[r['retained_fraction'] for r in data if r['within_bin'] is not None]);need(a['total_valid_points']==sum(r['valid_points'] for r in data) and a['total_retained_points']==sum(r['retained_points'] for r in data),'aggregate point totals');need(a['bin_reasons']==dict(Counter(b['reason'] or 'eligible' for r in data for b in r['bins'])),'all unavailable bin reasons')
 mutations=0;row,source,shape=sample
 for kind in ['membership','eligibility','rank','support','coefficient','source_depth']:
  bad=deepcopy(row['result']);points=deepcopy(source);b=next(v for v in bad['bins'] if v['eligible'])
  if kind=='membership':b['feature_indices'].pop()
  elif kind=='eligibility':b['eligible']=False
  elif kind=='rank':b['ranks'][0]['gap_rank']+=.1
  elif kind=='support':bad['retained_feature_indices'].pop()
  elif kind=='coefficient':bad['within_bin']+=.1
  else:
   target=b['feature_indices'][0];next(p for p in points if p['feature_index']==target)['reference_m']=1000 if b['key'][0]<5 else 1
  try:check_result(bad,points,shape,row['control'],row['minimum'])
  except ValueError:mutations+=1
  else:raise ValueError('undetected corruption '+kind)
 result=dict(passed=True,independent_checks=COUNT,mutations_rejected=mutations,image_pairs=80,original_primary_points=total,controlled_rows=len(allrows),aggregates=48,raw_EXP151_reproduced=True,auditor_sha256=sha(__file__),physical_reference_qualified=False)
 with (OUT/'independent-audit.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
 print(json.dumps(result,indent=2))
if __name__=='__main__':audit()
