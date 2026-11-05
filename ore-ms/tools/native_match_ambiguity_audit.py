"""Independent native descriptor/patch/rank audit for initialized EXP149 replay."""
import json,hashlib,math
from pathlib import Path
from copy import deepcopy
import numpy as np
import native_depth_proxy_audit as scalar
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/native-match-ambiguity/run-001';PLAN=ROOT/'experiments/native-match-ambiguity-2026-11-05.json';COUNT=0
VARIABLES=['descriptor_margin','descriptor_ratio','patch_relative_gap']
def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def need(v,label):
 global COUNT
 COUNT+=1
 if not v:raise ValueError(label)
def close(a,b,label):scalar.close(a,b,label)
def ranks(values):
 order=sorted(range(len(values)),key=lambda i:values[i]);result=[0.]*len(values);start=0
 while start<len(order):
  end=start+1
  while end<len(order) and values[order[end]]==values[order[start]]:end+=1
  rank=(start+1+end)/2
  for j in range(start,end):result[order[j]]=rank
  start=end
 return result
def rho(x,y):
 a=ranks(x);b=ranks(y);am=sum(a)/len(a);bm=sum(b)/len(b);numer=sum((u-am)*(v-bm) for u,v in zip(a,b));return numer/math.sqrt(sum((u-am)**2 for u in a)*sum((v-bm)**2 for v in b))

def check_costs(point,frame):
 idx=point['feature_index'];feature=frame['features'][idx];u,v,level=feature[:3];need(len(point['descriptor'])==32,'left descriptor bytes')
 cand=[];lower=float(np.float32(np.float32(u)-np.float32(frame['max_disparity'])))
 for i,right in enumerate(frame['right_features']):
  x,y,octave=right['key'];rad=np.float32(2)*np.float32(frame['scale_factors'][int(octave)]);low=math.floor(float(np.float32(np.float32(y)-rad)));high=math.ceil(float(np.float32(np.float32(y)+rad)))
  if low<=int(v)<=high and level-1<=octave<=level+1 and lower<=x<=u:
   need(len(right['descriptor'])==32,'right descriptor bytes');distance=sum((int(a)^int(b)).bit_count() for a,b in zip(point['descriptor'],right['descriptor']));cand.append([i,distance])
 need(point['candidates']==cand,'all descriptor candidates/Hamming costs')
 passing=[c for c in cand if c[1]<100];selected=min(passing,key=lambda c:c[1]) if passing else [-1,100]
 need(point['selected_right_id']==selected[0] and point['best_hamming']==selected[1],'native strict first descriptor best')
 distances=sorted(c[1] for c in cand);gap=distances[1]-distances[0] if len(distances)>1 else None;ratio=distances[0]/distances[1] if len(distances)>1 and distances[1] else None
 patchgap=tie=None
 if point['patch_status']=='ok':
  need(selected[1]<75,'native patch descriptor threshold');need(len(point['left_patch'])==121 and len(point['right_patches'])==11,'native patch shape')
  costs=[]
  for p in point['right_patches']:
   need(len(p)==121,'each native right patch');costs.append(sum(abs(int(a)-int(b)) for a,b in zip(point['left_patch'],p)))
  need(point['costs']==costs,'scalar uint8 patch L1 costs')
  inv=np.float32(frame['inverse_scales'][int(level)]);ul=math.floor(float(np.float32(np.float32(u)*inv))+.5);vl=math.floor(float(np.float32(np.float32(v)*inv))+.5);ur=math.floor(float(np.float32(np.float32(frame['right_features'][selected[0]]['key'][0])*inv))+.5)
  need(point['scaled_left']==[ul,vl] and point['scaled_right_center']==ur,'native pyramid sampling coordinates')
  i=min(range(11),key=lambda j:costs[j]);alts=[c for j,c in enumerate(costs) if abs(j-i)>=2];patchgap=(min(alts)-costs[i])/max(1,costs[i]);tie=min(alts)==costs[i]
  if 0<i<10:
   a=np.float32(costs[i-1]);b=np.float32(costs[i]);c=np.float32(costs[i+1]);den=np.float32(2)*np.float32(np.float32(a+c)-np.float32(2)*b)
   if den:
    delta=np.float32(np.float32(a-c)/den);scaled=np.float32(np.float32(np.float32(ur)+np.float32(i-5))+delta);right=np.float32(np.float32(frame['scale_factors'][int(level)])*scaled);disparity=np.float32(np.float32(u)-right)
    if -1<=delta<=1 and 0<=disparity<np.float32(frame['max_disparity']):
     if disparity<=0:disparity=np.float32(.01);right=np.float32(float(u)-.01)
     depth=np.float32(np.float32(frame['bf'])/disparity);need([float(right),float(depth)]==feature[3:5],'native float32 parabolic right/depth result')
 return dict(descriptor_margin=gap,descriptor_ratio=ratio,patch_relative_gap=patchgap,descriptor_tie=gap==0 if gap is not None else None,patch_separated_tie=tie)

def audit():
 p=read(PLAN);need(sha(PLAN)==sha(OUT/'study-lock.json'),'study lock')
 for n,h in p['source_sha256'].items():need(sha(ROOT/n)==h,'source input pin')
 prepared=read(OUT/'prepared.json');summary=read(OUT/'summary.json');need(len(summary['cases'])==40 and len(summary['aggregates'])==4,'all40cases/4cohorts')
 for key in ['physical_reference_qualified','benchmark_execution_ready','matcher_tuned','new_trajectory']:need(summary[key] is False,'scope '+key)
 reports=[];samples=None;point_total=faithful_total=0;cases=[]
 for mode in ['rgb','thermal']:
  status=read(OUT/(mode+'-status.json'));need(status['terminal_success'] is True and status['returncode']==0,'single native replay success')
  frames=[json.loads(s) for s in (OUT/(mode+'.jsonl')).read_text().splitlines()];expected=[r for r in prepared['records'] if r['modality']==mode];need([v['index'] for v in frames]==[r['index'] for r in expected]==list(range(0,2000,100)),'exact fixed image grid')
  for frame,r in zip(frames,expected):
   need(len(frame['features'])==len(r['historical']['features']),'all left features in replay');need(all(a[:3]==b[:3] for a,b in zip(frame['features'],r['historical']['features'])),'historical coordinate identity')
   need(len(frame['points'])==len(r['targets']),'all primary feature targets');report=read(OUT/'analysis'/mode/(r['stem']+'.json'));need(len(report['points'])==len(r['targets']),'all output target rows')
   # Rebind the selected target metrics/reference cohort to original inputs.
   original=read(ROOT/'results/native-depth-proxy/run-001/points'/mode/(r['stem']+'.json'));original={q['feature_index']:q for q in original['points'] if q['reason']=='compared'}
   reference=read(ROOT/'results/depth-projection-sensitivity/run-001/points'/mode/(r['stem']+'-r2.json'));reference={q['feature_index']:q['group'] for q in reference['points']}
   for target,point,row in zip(r['targets'],frame['points'],report['points']):
    i=target['feature_index'];need(i==point['feature_index']==row['feature_index'],'retained feature identity');need(target['metrics']==row['original_metrics']==original[i]['metrics'] and target['reference_group']==row['reference_group']==reference[i],'original target/group unchanged')
    want=check_costs(point,frame)
    for key,value in want.items():
     if isinstance(value,bool) or value is None:need(row['ambiguity'][key] is value,'ambiguity tie/null '+key)
     else:close(row['ambiguity'][key],value,'independent ambiguity '+key)
    historical=frame['features'][i]==target['native'][:5];need(row['historical_exact'] is historical,'per-point historical reproduction')
    need(row['native_cost_reconstruction_exact'] is True and row['faithful'] is historical,'native replay cost fidelity')
    point_total+=1;faithful_total+=historical
    if samples is None:samples=(deepcopy(point),deepcopy(frame))
   case=report['case'];need(case==next(v for v in summary['cases'] if v['modality']==mode and v['index']==r['index']),'case summary binding')
   for group,g in case['groups'].items():
    cohort=report['points'] if group=='all_primary' else [q for q in report['points'] if q['reference_group']=='locally_consistent'];valid=[q for q in cohort if q['faithful']]
    need(g['points']==len(cohort) and g['faithful']==len(valid),'cohort/fidelity population');need(g['descriptor_ties']==sum(q['ambiguity']['descriptor_tie'] is True for q in valid) and g['patch_ties']==sum(q['ambiguity']['patch_separated_tie'] is True for q in valid),'exact ambiguity ties')
    for key in VARIABLES:
     scalar.check_stats(g['variables'][key],[q['ambiguity'][key] for q in valid if q['ambiguity'][key] is not None]);selected=[q for q in valid if q['ambiguity'][key] is not None];x=[q['ambiguity'][key] for q in selected];y=[q['original_metrics']['abs_relative_error'] for q in selected];reason='fewer_than_10' if len(selected)<10 else 'constant' if len(set(x))<2 or len(set(y))<2 else None
     correlation=g['correlations'][key];need(correlation['points']==len(selected) and correlation['reason']==reason,'per-image rank cohort/availability');close(correlation['rho'],rho(x,y) if reason is None else None,'scalar tied-rank rho')
    scalar.check_stats(g['all_original_absrel'],[q['original_metrics']['abs_relative_error'] for q in cohort]);scalar.check_stats(g['faithful_original_absrel'],[q['original_metrics']['abs_relative_error'] for q in valid])
   cases.append(case)
 for a in summary['aggregates']:
  data=[v['groups'][a['group']] for v in cases if v['modality']==a['modality']];need(a['points']==sum(v['points'] for v in data) and a['faithful']==sum(v['faithful'] for v in data),'aggregate points/fidelity')
  need(a['descriptor_ties']==sum(v['descriptor_ties'] for v in data) and a['patch_ties']==sum(v['patch_ties'] for v in data),'aggregate ties')
  for key in VARIABLES:
   values=[v['correlations'][key]['rho'] for v in data if v['correlations'][key]['rho'] is not None];s=a['correlations'][key];need(s['images_defined']==len(values) and s['images_unavailable']==20-len(values) and s['negative_images']==sum(v<0 for v in values),'image-rank aggregate coverage');scalar.check_stats(s['stats'],values)
 mutations=0;point,frame=samples
 for kind in ['hamming','patch','candidate','depth']:
  bad=deepcopy(point);changed=deepcopy(frame) if kind=='depth' else frame
  if kind=='hamming':bad['candidates'][0][1]+=1
  elif kind=='patch':bad['costs'][0]+=1
  elif kind=='candidate':bad['candidates'].pop()
  else:changed['features'][bad['feature_index']][4]*=2
  try:check_costs(bad,changed)
  except ValueError:mutations+=1
  else:raise ValueError('missed corruption '+kind)
 # Independent rank fixture/mutation ensures ordering is material.
 need(abs(rho(list(range(10)),list(range(9,-1,-1)))+1)<1e-12,'known negative ranks')
 for value,expected in [(rho([0,0,1,1],[3,3,1,1]),-1)]:close(value,expected,'tied negative rank fixture')
 result=dict(passed=True,independent_checks=COUNT+scalar.COUNT,mutations_rejected=mutations,primary_points=point_total,historical_and_cost_faithful=faithful_total,image_pairs=40,physical_reference_qualified=False,scope='raw native descriptor/patch costs, initialized replay geometry and original proxy ranks; not physical correspondence truth',auditor_sha256=sha(__file__),plan_sha256=sha(PLAN))
 with (OUT/'independent-audit.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
 print(json.dumps(result,indent=2))
if __name__=='__main__':audit()
