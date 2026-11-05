"""Independent scalar window, reference-only stratum and interval audit."""
import hashlib,json,math
from pathlib import Path
from copy import deepcopy
from collections import Counter
import cv2
import native_depth_proxy_audit as old
ROOT=Path(__file__).resolve().parents[1];PRIOR=ROOT/'results/native-depth-proxy/run-001';OUT=ROOT/'results/depth-projection-sensitivity/run-001';PLAN=ROOT/'experiments/depth-projection-sensitivity-2026-11-05.json';COUNT=0
GROUPS=['insufficient','locally_consistent','mixed_depth'];CLASSES=['short_for_all','long_for_all','contains_unit','unavailable'];KEYS=old.KEYS

def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def need(v,label):
 global COUNT
 COUNT+=1
 if not v:raise ValueError(label)
def close(a,b,label):old.close(a,b,label)

def derive(feature,depth,fx,b,radius):
 u,v,_,right,z,_,_=feature;x=math.floor(u+.5);y=math.floor(v+.5);candidates=[];rejected=Counter(outside=0,zero=0,proxy_right_outside=0)
 for dx in range(-radius,radius+1):
  for dy in range(-radius,radius+1):
   xx=x+dx;yy=y+dy
   if xx<0 or yy<0 or xx>=len(depth[0]) or yy>=len(depth):rejected['outside']+=1;continue
   code=int(depth[yy,xx])
   if code==0:rejected['zero']+=1;continue
   reference=code/256
   if not(u*reference>=fx*b and (u-(len(depth[0])-1))*reference<fx*b):rejected['proxy_right_outside']+=1;continue
   candidates.append([xx,yy,code])
 candidates.sort(key=lambda q:(q[1],q[0]));values=[v[2]/256 for v in candidates]
 if values:
  lo=min(values);hi=max(values);middle=old.quantile(values,.5);span=(hi-lo)/middle
  group='insufficient' if len(values)<3 else 'locally_consistent' if span<=.10 else 'mixed_depth';lower=z/hi;upper=z/lo
  classification='short_for_all' if z<lo else 'long_for_all' if z>hi else 'contains_unit'
 else:lo=hi=middle=span=lower=upper=None;group='insufficient';classification='unavailable'
 return dict(center=[x,y],candidates=candidates,rejected=dict(rejected),group=group,reference_min_m=lo,reference_max_m=hi,reference_median_m=middle,relative_span=span,ratio_lower=lower,ratio_upper=upper,interval_class=classification)

def check_point(point,original,depth,r, radius):
 need(point['feature_index']==original['feature_index'] and point['native']==original['native'],'fixed original feature')
 want=derive(original['native'],depth,r['K'][0][0],r['baseline_m'],radius)
 for key in ['center','candidates','rejected','group','interval_class']:need(point[key]==want[key],'scalar window '+key)
 for key in ['reference_min_m','reference_max_m','reference_median_m','relative_span','ratio_lower','ratio_upper']:close(point[key],want[key],'scalar neighborhood '+key)
 need(any(q[:2]==original['pixel'] and q[2]==original['depth_code'] for q in want['candidates']),'central reference retained')
 need(point['original_metrics']==original['metrics'],'primary metrics unchanged')
 _,reason,_,_,metrics=old.expect(original['native'],depth,r['K'][0][0],r['baseline_m'],r['shape'][1]);need(reason=='compared','original primary still valid')
 for key,val in metrics.items():close(point['original_metrics'][key],val,'independent original central error')
 return want

def check_reduced(record,points):
 need(record['points']==len(points),'full group population')
 for key in KEYS:old.check_stats(record['metrics'][key],[v['original_metrics'][key] for v in points])
 need(record['interval_counts']=={key:sum(v['interval_class']==key for v in points) for key in CLASSES},'interval sign counts')
 old.check_stats(record['ratio_lower'],[v['ratio_lower'] for v in points if v['ratio_lower'] is not None]);old.check_stats(record['ratio_upper'],[v['ratio_upper'] for v in points if v['ratio_upper'] is not None])

def audit():
 p=read(PLAN);need(sha(PLAN)==sha(OUT/'study-lock.json'),'study lock')
 for n,h in p['source_sha256'].items():need(sha(ROOT/n)==h,'frozen input '+n)
 summary=read(OUT/'summary.json');need(len(summary['cases'])==80 and len(summary['aggregates'])==4,'all declared cases/radii')
 for k in ['physical_reference_qualified','physical_sync_verified','benchmark_execution_ready','scale_fitted','time_fitted','primary_errors_changed']:need(summary[k] is False,'scope '+k)
 source=read(PRIOR/'native-selected.json');members={v['name']:v for v in read(PRIOR/'acquisition.json')['members']};records={};mutation_sample=None
 for r in source['records']:
  path=ROOT/members[source['sequence']+'/'+r['camera']+'/depth/'+r['stem']+'.png']['path'];depth=cv2.imread(str(path),-1);primary=read(PRIOR/'points'/r['modality']/(r['stem']+'.json'));original=[v for v in primary['points'] if v['reason']=='compared']
  for radius in [1,2]:
   result=read(OUT/'points'/r['modality']/(r['stem']+'-r'+str(radius)+'.json'));case=result['case'];points=result['points'];need(len(points)==len(original)==case['primary_points'],'fixed primary point support')
   for point,previous in zip(points,original):check_point(point,previous,depth,r,radius)
   check_reduced(case['all'],points);need(case['all']['metrics']==primary['record']['metrics'],'all-point primary summary unchanged')
   for group in GROUPS:check_reduced(case['groups'][group],[v for v in points if v['group']==group])
   need(sum(v['points'] for v in case['groups'].values())==len(original),'strata partition all primary points')
   need(case['all']['interval_counts']['unavailable']==0,'primary central reference guarantees candidate')
   need(case==next(v for v in summary['cases'] if (v['modality'],v['index'],v['radius'])==(r['modality'],r['index'],radius)),'case summary exact')
   records[(r['modality'],r['index'],radius)]=case
   if points and mutation_sample is None:mutation_sample=(points[0],original[0],depth,r,radius)
 for mode in ['rgb','thermal']:
  for radius in [1,2]:
   aggregate=next(v for v in summary['aggregates'] if v['modality']==mode and v['radius']==radius);rows=[records[(mode,i,radius)] for i in range(0,2000,100)];need(aggregate['images']==20,'all20 images retained')
   expected_total=309 if mode=='rgb' else 1089;need(aggregate['groups']['all']['points']==expected_total,'fixed full primary cohort')
   for name in ['all']+GROUPS:
    data=[c['all'] if name=='all' else c['groups'][name] for c in rows];saved=aggregate['groups'][name]
    need(saved['points']==sum(c['points'] for c in data) and saved['images_nonempty']==sum(c['points']>0 for c in data) and saved['images_empty']==sum(c['points']==0 for c in data),'group/image coverage')
    need(saved['interval_counts']=={key:sum(c['interval_counts'][key] for c in data) for key in CLASSES},'aggregate interval signs')
    for key in KEYS:old.check_stats(saved['metrics'][key],[v['metrics'][key]['median'] for v in data if v['metrics'][key]['median'] is not None])
 mutations=0;point,original,depth,r,radius=mutation_sample
 for kind in ['candidate','stratum','primary','interval']:
  bad=deepcopy(point)
  if kind=='candidate':bad['candidates'].pop()
  elif kind=='stratum':bad['group']='mixed_depth' if bad['group']!='mixed_depth' else 'insufficient'
  elif kind=='primary':bad['original_metrics']['depth_ratio']*=2
  else:bad['ratio_lower']+=.1
  try:check_point(bad,original,depth,r,radius)
  except ValueError:mutations+=1
  else:raise ValueError('missed corruption '+kind)
 result=dict(passed=True,independent_checks=COUNT+old.COUNT,mutations_rejected=mutations,image_radius_cases=80,primary_features=1398,point_radius_memberships=2796,reference_visibility_qualified=False,auditor_sha256=sha(__file__),plan_sha256=sha(PLAN))
 with (OUT/'independent-audit.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
 print(json.dumps(result,indent=2))
if __name__=='__main__':audit()
