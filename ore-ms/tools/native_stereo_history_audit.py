"""Independent EXP146 native stereo-array, factor/export and score audit."""
import hashlib,json,math
from pathlib import Path
from copy import deepcopy
import numpy as np
from scipy.stats import rankdata
import orb_finalization_audit as finalaudit
import provided_reference_full_record_audit as metric
import stereo_reference_audit as decomposition
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/native-stereo-history/run-002';PLAN=ROOT/'experiments/native-stereo-history-2026-11-05.json';COUNT=0

def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def need(v,label):
 global COUNT
 COUNT+=1
 if not v:raise ValueError(label)
def close(a,b,label):
 if a is None or b is None:need(a is b,label)
 else:need(math.isclose(a,b,rel_tol=2e-7,abs_tol=2e-8),label)

def native_frame(row,plan,index):
 need(row['index']==index and row['source_id']==plan['stems'][index],'native stereo source identity')
 need(row['time_ns']==plan['time_ns'][index] and row['right_time_ns']==plan['right_time_ns'][index],'native stereo integer clocks')
 if row.get('available') is False:
  need(row['features']==[] and row['reason']=='native_frame_empty_after_return','explicit unavailable post-return frame')
  return dict(index=index,**{k:None for k in ['features','right_features','accepted','acceptance','median_disparity_px','median_depth_m','close','far','associated_accepted','flagged_outlier_accepted','missing_map_flags','missing_outlier_flags']})
 need(row['native_time']==(plan['time_ns'][index]-plan['time_ns'][0])*1e-9,'native frame exact elapsed clock')
 f=np.asarray([v[:5] for v in row['features']],float).reshape(-1,5)
 need(len(f)==row['N'] and row['N']>=0,'all native feature slots')
 need(np.isfinite(f).all(),'finite native features')
 positive=f[:,4]>0
 need(np.array_equal(positive,f[:,3]>=0) and np.all(f[~positive,3:5]==-1),'unmatched sentinel and accepted mask')
 d=f[positive,0]-f[positive,3];z=f[positive,4]
 need(np.all(d>0),'positive accepted disparity')
 # Native depth division uses float disparity before a floored right coordinate
 # rounds. Bound that coordinate subtraction from the serialized float snapshots.
 ul=np.abs(np.spacing(f[positive,0].astype(np.float32)).astype(float));ur=np.abs(np.spacing(f[positive,3].astype(np.float32)).astype(float))
 eps=4*(ul+ur)
 lo=np.maximum(d-eps,np.finfo(np.float32).tiny);hi=d+eps
 need(np.all(z>=row['bf']/hi*(1-2e-5)-2e-5) and np.all(z<=row['bf']/lo*(1+2e-5)+2e-5),'native float32 bf/disparity depth')
 need(np.isclose(row['bf'],plan['calibration']['bf'],rtol=5e-7,atol=1e-7) and np.isclose(row['baseline_m'],plan['calibration']['baseline_m'],rtol=5e-7,atol=1e-7),'native baseline/focal calibration')
 need(np.isclose(row['close_depth_m'],35*row['baseline_m'],rtol=5e-7,atol=1e-6),'native close threshold')
 need(np.all((f[:,2]>=0)&(f[:,2]<8)&(f[:,2]==np.floor(f[:,2]))),'native pyramid octaves')
 for j,v in enumerate(row['features']):
  need((v[5] is None)==(j>=row['map_array_size']) and (v[6] is None)==(j>=row['outlier_array_size']),'native flag array availability')
  need((v[5] is None or type(v[5]) is bool) and (v[6] is None or type(v[6]) is bool),'native snapshot flag types')
 def median(v):return float(np.median(v)) if len(v) else None
 return dict(index=index,features=len(f),right_features=row['right_keypoints'],accepted=int(sum(positive)),acceptance=float(sum(positive)/len(f)) if len(f) else None,median_disparity_px=median(d),median_depth_m=median(z),close=int(sum(z<row['close_depth_m'])),far=int(sum(z>=row['close_depth_m'])),associated_accepted=sum(v[4]>0 and v[5] is True for v in row['features']),flagged_outlier_accepted=sum(v[4]>0 and v[6] is True for v in row['features']),missing_map_flags=sum(v[4]>0 and v[5] is None for v in row['features']),missing_outlier_flags=sum(v[4]>0 and v[6] is None for v in row['features']))

def audit():
 p=read(PLAN);lock=read(OUT/'prediction-lock.json')
 for n,h in {**p['source_sha256'],**lock['source_sha256']}.items():need(sha(ROOT/n)==h,'frozen source/output pin')
 summary=read(OUT/'summary.json')
 for key in ['physical_sync_verified','benchmark_execution_ready','scale_fitted','time_fitted','causal_attribution']:need(summary[key] is False,'scope qualifier')
 source=read(OUT/'source-manifest.json');need(len(source['records'])==2000,'complete2000source')
 reports=[]
 for mode,m in [('rgb','rgb'),('thermal','thr')]:
  d=OUT/'captures'/(mode+'-paired');plan=read(d/'plan.json');frames=[];saved=read(OUT/'analysis'/mode/'frames.json')['rows'];entry=next(x for x in summary['runs'] if x['modality']==mode)
  with (d/'native.jsonl.stereo.jsonl').open() as f:
   for i,line in enumerate(f):
    want=native_frame(json.loads(line),plan,i);need(want==saved[i],'independent all-feature frame summary');frames.append(want)
  need(len(frames)==len(saved)==2000,'all native stereo frames')
  need(entry['total_features']==sum(r['features'] or 0 for r in frames) and entry['total_accepted']==sum(r['accepted'] or 0 for r in frames),'full feature totals')
  need(entry['stereo_available_frames']==sum(v['features'] is not None for v in frames) and entry['unavailable_source_indices']==[v['index'] for v in frames if v['features'] is None],'unavailable source frames explicit')
  for key,s in entry['frame_summary'].items():decomposition.check_stats(s,[v[key] for v in frames if v[key] is not None])
  report=finalaudit.export_audit(d);report['modality']=mode;reports.append(report)
  reflock=read(ROOT/'results/provided-reference-full-record/run-001/reference-lock.json')
  for v in source['records']:
   refpath=ROOT/'results/provided-reference-full-record/run-001/reference'/source['sequence']/m/(v['stem']+'.txt')
   need(sha(refpath)==reflock['member_sha256'][str(refpath.relative_to(ROOT))],'raw reference pin')
  for arm in ['control','paired']:
   directory=OUT/'captures'/(mode+'-'+arm);view0=read(directory/'received.json');status=read(directory/'run-status.json')
   native=[json.loads(line) for line in (directory/'native.jsonl').read_text().splitlines()];native=[r for r in native if 'index' in r]
   need(len(native)==2000 and status['terminal_success'] is True,'all native calls/terminal status')
   sid0=-1;active0=False;last0=None
   for i,row0 in enumerate(native):
    need(row0['index']==i and row0['time_ns']==plan['time_ns'][i] and row0['right_time_ns']==plan['right_time_ns'][i],'original native axis')
    if row0['T_camera_from_world'] is None:
     need(view0['poses'][i] is None and view0['segments'][i] is None,'received missing unchanged');active0=False;continue
    t=np.asarray(row0['T_camera_from_world']);inv=np.eye(4);inv[:3,:3]=t[:3,:3].T;inv[:3,3]=-t[:3,:3].T@t[:3,3]
    need(np.allclose(inv,view0['poses'][i],rtol=0,atol=1e-10),'native received rigid transpose')
    if not active0 or row0['map_id']!=last0:sid0+=1
    need(view0['segments'][i]==sid0,'native map/missing segment binding');active0=True;last0=row0['map_id']
  raw=[np.loadtxt(ROOT/'results/provided-reference-full-record/run-001/reference'/source['sequence']/m/(v['stem']+'.txt')).reshape(4,4) for v in source['records']]
  motions=read(OUT/'analysis'/mode/'motion.json');received=read(d/'received.json');final=read(d/'final.json');mask=[a is not None and b is not None for a,b in zip(received['poses'],final['poses'])]
  seg=[];sid=-1;active=False;previous=None
  for ok,original in zip(mask,received['segments']):
   if not ok:seg.append(None);active=False;continue
   if not active or original!=previous:sid+=1
   seg.append(sid);active=True;previous=original
  for name in ['control','received','final']:
   view=read(OUT/'scores'/mode/(name+'-view.json'));score=read(OUT/'scores'/mode/(name+'.json'));want=metric.independently_derive(view['time_ns'],raw,view);metric.compare(score,want)
   if name!='control':
    original=received if name=='received' else final
    need(view['segments']==seg and view['poses']==[v if ok else None for v,ok in zip(original['poses'],mask)],'exact common mask/boundaries/no pose edit')
   motion=motions[name];need([r['indices'] for r in motion['rows']]==[r['source_indices'] for r in score['rpe_rows']],'same motion pairs')
   for row in motion['rows']:
    i,j=row['indices'];g=decomposition.relative_translation(metric.rigid(raw[i]),metric.rigid(raw[j]));e=decomposition.relative_translation(metric.rigid(view['poses'][i]),metric.rigid(view['poses'][j]));wantrow=decomposition.deriv(g,e)
    for k,v in wantrow.items():close(row[k],v,'independent scalar motion '+k)
   decomposition.check_summary(motion['summary'],motion['rows']);need(entry['motion_summary'][name]==motion['summary'],'summary motion binding')
  for b in entry['bins']:
   a,end=b['start'],b['end_exclusive'];need([a,end] in [[0,500],[500,1000],[1000,1500],[1500,2000]],'fixed bins')
   need(b['available_frames']==sum(v['features'] is not None for v in frames[a:end]),'bin available population')
   for key in ['accepted','acceptance','median_disparity_px','median_depth_m']:decomposition.check_stats(b[key],[v[key] for v in frames[a:end] if v[key] is not None])
   decomposition.check_summary(b['final_motion'],[r for r in motions['final']['rows'] if a<=r['indices'][0]<end])
  delta=[r-l for r,l in zip(plan['right_time_ns'],plan['time_ns'])]
  for s in entry['recorded_sign_strata']:
   cohort=[r for r in motions['final']['rows'] if ('negative' if delta[r['indices'][0]]<0 else 'positive' if delta[r['indices'][0]]>0 else 'zero')==s['recorded_delta_sign']];decomposition.check_summary(s['motion'],cohort)
  for key,s in entry['spearman'].items():
   cohort=[r for r in motions['final']['rows'] if key=='signed_delta_ms' or frames[r['indices'][0]][key] is not None]
   x=[delta[r['indices'][0]]/1e6 if key=='signed_delta_ms' else frames[r['indices'][0]][key] for r in cohort];y=[r['ratio'] for r in cohort]
   rho=float(np.corrcoef(rankdata(x),rankdata(y))[0,1]) if len(set(x))>1 and len(set(y))>1 else None
   need(s['pairs']==len(cohort) and s['causal'] is False,'correlation population/qualifier');close(s['rho'],rho,'independent rank correlation')
 mutations=0
 # Mutate native sentinels, depth scale, omitted feature and source clock.
 sample=json.loads((OUT/'captures/thermal-paired/native.jsonl.stereo.jsonl').open().readline());plan=read(OUT/'captures/thermal-paired/plan.json')
 for kind in ['depth','sentinel','population','clock']:
  bad=deepcopy(sample)
  if kind=='depth':
   j=next(i for i,v in enumerate(bad['features']) if v[4]>0);bad['features'][j][4]*=2
  elif kind=='sentinel':bad['features'][0][3]=-2
  elif kind=='population':bad['features'].pop()
  else:bad['time_ns']+=1
  try:native_frame(bad,plan,0)
  except ValueError:mutations+=1
  else:raise ValueError('undetected native corruption '+kind)
 result=dict(passed=True,independent_checks=COUNT+metric.COUNT+finalaudit.COUNT+decomposition.COUNT,mutations_rejected=mutations,runs=reports,auditor_sha256=sha(__file__),physical_sync_verified=False,benchmark_execution_ready=False)
 with (OUT/'independent-audit.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
 print(json.dumps(result,indent=2))
if __name__=='__main__':audit()
