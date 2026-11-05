"""EXP145 fixed-prefix, descriptive stereo/input/reference diagnostics; never fits."""
import argparse
from copy import deepcopy
import hashlib,json
from pathlib import Path
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
PRIOR=ROOT/'results/orb-finalization/run-001'
OUT=ROOT/'results/stereo-reference-diagnostic/run-001'
PLAN=ROOT/'experiments/stereo-reference-diagnostic-2026-11-05.json'
PARAMS=dict(indices=list(range(0,2000,100)),features=2000,scale_factor=1.2,levels=8,fast=20,edge=31,patch=31,wta=2,ratio=.75,vertical_px=2.,bins=[[0,500],[500,1000],[1000,1500],[1500,2000]])

def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def write(p,v):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')
def local(s):
 p=Path(s)
 if p.is_absolute():
  for base in ['/workspace','/root/AirStack/ore-ms']:
   if str(p).startswith(base+'/'):return ROOT/str(p)[len(base)+1:]
  raise ValueError('unrecognized source root')
 return ROOT/p

def freeze():
 pins={}
 def add(p):
  p=local(p);pins[str(p.relative_to(ROOT))]=sha(p)
 for p in [Path(__file__),ROOT/'tools/stereo_reference_audit.py',ROOT/'tools/tests/test_stereo_reference_diagnostic.py',ROOT/'docs/research/stereo-reference-diagnostic-protocol-2026-11-05.md',PRIOR/'source-manifest.json',PRIOR/'prediction-lock.json',PRIOR/'study-lock.json',PRIOR/'summary.json',ROOT/'MAC-VO-main/DataLoader/Dataset/MS2.py',ROOT/'tools/orb_ms2_development.py',ROOT/'results/ms2-night-pilot/calibration.json',ROOT/'results/provided-reference-full-record/run-001/reference-lock.json']:
  add(p)
 for p in (ROOT/'results/provided-reference-full-record/run-001/source-001').glob('*'):add(p)
 locked=read(PRIOR/'prediction-lock.json')
 for name,h in locked['source_sha256'].items():
  if sha(ROOT/name)!=h:raise ValueError('prior prediction pin')
  add(name)
 source=read(PRIOR/'source-manifest.json');refs=read(ROOT/'results/provided-reference-full-record/run-001/reference-lock.json')
 for mode,m in [('rgb','rgb'),('thermal','thr')]:
  d=PRIOR/'captures'/(mode+'-paired')
  for name in ['plan.json','settings.yaml','inputs.tsv']:add(d/name)
  add(PRIOR/'protocols'/('droid-'+mode+'.json'))
  for arm in ['received','final','droid']:
   for suffix in ['.json','-view.json']:add(PRIOR/'scores'/mode/(arm+suffix))
  prepared={Path(x['path']).name+':'+Path(x['path']).parent.name:x for x in read(d/'plan.json')['prepared_images']}
  for i,row in enumerate(source['records']):
   ref=ROOT/'results/provided-reference-full-record/run-001/reference'/source['sequence']/m/(row['stem']+'.txt')
   if sha(ref)!=refs['member_sha256'][str(ref.relative_to(ROOT))]:raise ValueError('reference pin')
   add(ref)
   if i in PARAMS['indices']:
    for side in ['left','right']:
     original=row['cameras'][m][side];f=ROOT/original['path']
     if sha(f)!=original['sha256']:raise ValueError('raw source pin')
     add(f);item=prepared[row['stem']+'.png:'+side];f=local(item['path'])
     if sha(f)!=item['sha256']:raise ValueError('canonical pin')
     add(f)
 write(PLAN,dict(parent='08c1583f5d73a6c8353c251c427b9dee42e763c3',source_frames=2000,sequence=source['sequence'],parameters=PARAMS,runtime='sha256:d3cf8a41dd8090c8939cd840b4735d526cec9a54e48ac36d506f91d13ee56e1c',source_sha256=pins,scale_fitted=False,time_fitted=False,physical_sync_verified=False,new_native_inference=False,benchmark_execution_ready=False))
 print('Frozen',len(pins),'source pins')

def verify():
 p=read(PLAN)
 if p['parameters']!=PARAMS:raise ValueError('parameter change')
 for n,h in p['source_sha256'].items():
  if sha(ROOT/n)!=h:raise ValueError('changed pin '+n)
 return p

def stats(values):
 a=np.asarray(values,dtype=float)
 if not len(a):return dict(n=0,min=None,median=None,p90=None,max=None,rms=None)
 return dict(n=len(a),min=float(a.min()),median=float(np.median(a)),p90=float(np.percentile(a,90)),max=float(a.max()),rms=float(np.sqrt(np.mean(a*a))))

def decompose(g,e):
 g=np.asarray(g,dtype=float);e=np.asarray(e,dtype=float)
 if g.shape!=(3,) or e.shape!=(3,) or not np.isfinite(g).all() or not np.isfinite(e).all():raise ValueError('finite translation vectors required')
 gn=float(np.linalg.norm(g));en=float(np.linalg.norm(e));delta=e-g
 parallel=float(np.dot(delta,g/gn)) if gn>0 else None
 perp=float(np.linalg.norm(delta-parallel*g/gn)) if gn>0 else None
 return dict(reference_m=gn,prediction_m=en,ratio=en/gn if gn>0 else None,cosine=float(np.dot(g,e)/(gn*en)) if gn>0 and en>0 else None,parallel_error_m=parallel,perpendicular_error_m=perp,error_m=float(np.linalg.norm(delta)))

def summarize(rows):
 result={'pairs':len(rows)}
 for k in ['reference_m','prediction_m','ratio','cosine','parallel_error_m','perpendicular_error_m','error_m']:
  result[k]=stats([r[k] for r in rows if r[k] is not None])
 total=sum(r['error_m']**2 for r in rows)
 result['parallel_squared_error_share']=sum(r['parallel_error_m']**2 for r in rows if r['parallel_error_m'] is not None)/total if total else None
 result['zero_reference_pairs']=sum(r['reference_m']==0 for r in rows)
 return result

def match_summary(matches,fx):
 dx=np.asarray([v['left'][0]-v['right'][0] for v in matches]);dy=np.asarray([v['left'][1]-v['right'][1] for v in matches])
 return dict(matches=len(matches),positive_disparity=int(sum(dx>0)),within_vertical=int(sum(abs(dy)<=PARAMS['vertical_px'])),positive_and_vertical=int(sum((dx>0)&(abs(dy)<=PARAMS['vertical_px']))),signed_disparity_px=stats(dx),abs_vertical_px=stats(abs(dy)),abs_vertical_normalized=stats(abs(dy)/fx))

def extract(left,right):
 orb=cv2.ORB_create(nfeatures=PARAMS['features'],scaleFactor=PARAMS['scale_factor'],nlevels=PARAMS['levels'],edgeThreshold=PARAMS['edge'],patchSize=PARAMS['patch'],WTA_K=PARAMS['wta'],scoreType=cv2.ORB_HARRIS_SCORE,fastThreshold=PARAMS['fast'])
 kl,dl=orb.detectAndCompute(left,None);kr,dr=orb.detectAndCompute(right,None)
 if dl is None or dr is None or len(dl)<2 or len(dr)<2:return len(kl),len(kr),[]
 matcher=cv2.BFMatcher(cv2.NORM_HAMMING)
 def ratios(a,b):
  return {v[0].queryIdx:v[0] for v in matcher.knnMatch(a,b,k=2) if len(v)==2 and v[0].distance<PARAMS['ratio']*v[1].distance}
 lr=ratios(dl,dr);rl=ratios(dr,dl);matches=[]
 for i,v in sorted(lr.items()):
  if v.trainIdx in rl and rl[v.trainIdx].trainIdx==i:
   matches.append(dict(left_id=i,right_id=v.trainIdx,left=list(kl[i].pt),right=list(kr[v.trainIdx].pt),distance=float(v.distance)))
 return len(kl),len(kr),matches

def run():
 p=verify();cv2.setNumThreads(1);cv2.setRNGSeed(0)
 from orb_ms2_development import thermal_function
 replay=thermal_function();source=read(PRIOR/'source-manifest.json');cal=read(ROOT/'results/ms2-night-pilot/calibration.json')
 report=[];motion=[];images=[];clocks=[]
 for mode,m in [('rgb','rgb'),('thermal','thr')]:
  rows=source['records'];d=PRIOR/'captures'/(mode+'-paired');plan=read(d/'plan.json');proto=read(PRIOR/'protocols'/('droid-'+mode+'.json'))['runs'][0]
  k=np.asarray(cal['K_'+m+'L']['values']);kr=np.asarray(cal['K_'+m+'R']['values']);r=np.asarray(cal['R_'+m+'R']['values']);t=np.asarray(cal['T_'+m+'R']['values']).reshape(3);b=-float(t[0])*.001
  if not (np.array_equal(k,kr) and np.array_equal(r,np.eye(3)) and np.array_equal(t[1:],[0,0]) and b==proto['baseline_m'] and b==plan['calibration']['baseline_m'] and np.allclose(k,np.asarray(rows[0]['cameras'][m]['K']),rtol=0,atol=0)):raise ValueError('supplied rectified contract')
  report.append(dict(modality=mode,K=k.tolist(),baseline_m=b,bf=float(k[0,0]*b),orb_calibration=plan['calibration'],droid_geometry=proto['image_geometry'],source_shape=proto['source_shape'],metric_factor=b/.1,source_stereo_R_identity=True,source_stereo_K_equal=True,source_translation_horizontal=True))
  times=[row['cameras'][m]['left']['recorded_time_ns'] for row in rows];right=[row['cameras'][m]['right']['recorded_time_ns'] for row in rows]
  delta=[r-l for l,r in zip(times,right)]
  clocks.append(dict(modality=mode,source_frames=len(rows),signed_right_minus_left_ns=delta,left_times_ns=times,right_times_ns=right,signed_ms=stats(np.array(delta)*1e-6),absolute_ms=stats(abs(np.array(delta))*1e-6),left_cadence_ms=stats(np.diff(times)*1e-6),within_2ms=sum(abs(x)<=2000000 for x in delta),right_before_left=sum(x<0 for x in delta),equal=sum(x==0 for x in delta),right_after_left=sum(x>0 for x in delta),physical_exposure_offset_known=False))
  refs=[np.loadtxt(ROOT/'results/provided-reference-full-record/run-001/reference'/source['sequence']/m/(row['stem']+'.txt')).reshape(4,4) for row in rows]
  from independent_benchmark_metrics_v2 import pose
  refs=[pose(g)[0] for g in refs]
  for arm in ['received','final','droid']:
   view=read(PRIOR/'scores'/mode/(arm+'-view.json'));score=read(PRIOR/'scores'/mode/(arm+'.json'));pairs=[]
   for score_row in score['rpe_rows']:
    i,j=score_row['source_indices'];g=np.linalg.solve(refs[i],refs[j]);e=np.linalg.solve(pose(view['poses'][i])[0],pose(view['poses'][j])[0]);v=decompose(g[:3,3],e[:3,3]);v.update(indices=[i,j],duration_ns=times[j]-times[i]);pairs.append(v)
   motion.append(dict(modality=mode,arm=arm,rows=pairs,summary=summarize(pairs),bins=[dict(start=a,end_exclusive=b,summary=summarize([x for x in pairs if a<=x['indices'][0]<b])) for a,b in PARAMS['bins']]))
  prepared={Path(x['path']).name+':'+Path(x['path']).parent.name:x for x in plan['prepared_images']}
  for i in PARAMS['indices']:
   row=rows[i];pair=[];intensity=[]
   for side in ['left','right']:
    raw=cv2.imread(str(ROOT/row['cameras'][m][side]['path']),cv2.IMREAD_UNCHANGED);item=prepared[row['stem']+'.png:'+side];image=cv2.imread(str(local(item['path'])),cv2.IMREAD_UNCHANGED)
    expected=raw if m=='rgb' else replay(raw)
    if not np.array_equal(image,expected):raise ValueError('canonical producer replay')
    pair.append(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY) if m=='rgb' else image);intensity.append(dict(side=side,raw_dtype=str(raw.dtype),raw_shape=list(raw.shape),canonical_shape=list(image.shape),mean=float(image.mean()),std=float(image.std()),min=int(image.min()),max=int(image.max())))
   nl,nr,matches=extract(*pair)
   images.append(dict(modality=mode,index=i,stem=row['stem'],fx=float(k[0,0]),features_left=nl,features_right=nr,intensity=intensity,matches=matches,summary=match_summary(matches,float(k[0,0]))))
   print('IMAGE',mode,i,'mutual ratio matches',len(matches),flush=True)
 write(OUT/'contract.json',dict(runs=report,scope='numeric provided model consistency, not surveyed physical geometry'))
 write(OUT/'clocks.json',dict(runs=clocks,physical_sync_verified=False))
 write(OUT/'motion.json',dict(runs=motion,scale_fitted=False,time_fitted=False,physical_sync_verified=False,benchmark_execution_ready=False))
 write(OUT/'images.json',dict(runs=images,parameters=PARAMS,native_tracker_correspondences=False,physical_rectification_verified=False))
 summary=[]
 for mode in ['rgb','thermal']:
  cohort=[v for v in images if v['modality']==mode];matches=[v for row in cohort for v in row['matches']];agg=match_summary(matches,cohort[0]['fx'])
  summary.append(dict(modality=mode,image_pairs=len(cohort),empty_pairs=sum(not v['matches'] for v in cohort),image_summary=agg,motion={x['arm']:x['summary'] for x in motion if x['modality']==mode},clock={k:v for k,v in next(x for x in clocks if x['modality']==mode).items() if k not in ['signed_right_minus_left_ns','left_times_ns','right_times_ns']}))
 write(OUT/'summary.json',dict(runs=summary,study_sha256=sha(PLAN),physical_sync_verified=False,benchmark_execution_ready=False,scale_fitted=False,time_fitted=False,new_native_inference=False))
 verify();print('DIAGNOSTIC COMPLETE')

if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('action',choices=['freeze','verify','run']);x=a.parse_args();{'freeze':freeze,'verify':verify,'run':run}[x.action]()
