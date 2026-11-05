"""EXP146 read-only native frame stereo history with fixed exposed prefix."""
import argparse,json,hashlib,subprocess,time,traceback
from pathlib import Path
from copy import deepcopy
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/native-stereo-history/run-001'
PRIOR=ROOT/'results/orb-finalization/run-001'
PLAN=ROOT/'experiments/native-stereo-history-2026-11-05.json'
BASE='sha256:1542a9a2fe4f645aa7b3db1d4d002c0a0b5941a91db105c5587db62b68bd8445'
PARENT_RUNTIME='sha256:38b15cbcaf5d9f292d5279b4583f124e893003df26bfc93de16898f4d9abd688'
SCORE_RUNTIME='sha256:d3cf8a41dd8090c8939cd840b4735d526cec9a54e48ac36d506f91d13ee56e1c'
PARAMS=dict(source_groups=2000,seed=0,threads=1,timeout_seconds=3600,bins=[[0,500],[500,1000],[1000,1500],[1500,2000]],depth_rtol=2e-5,depth_atol_m=2e-5,coordinate_ulp_multiplier=4,arms=['control','paired'])
def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def write(p,v):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')
def local(s):
 p=str(s)
 for base in ['/workspace/','/root/AirStack/ore-ms/']:
  if p.startswith(base):return ROOT/p[len(base):]
 if p.startswith('/'):raise ValueError('unrecognized root')
 return ROOT/p

def prepare(count=2000,namespace=OUT):
 source=read(PRIOR/'source-manifest.json');source['records']=source['records'][:count]
 source['source_frames']=count;source['scope']='EXP146 fixed original night prefix'
 write(namespace/'source-manifest.json',source)
 for mode in ['rgb','thermal']:
  for arm in PARAMS['arms']:
   old=PRIOR/'captures'/(mode+'-paired');d=namespace/'captures'/(mode+'-'+arm);d.mkdir(parents=True)
   p=read(old/'plan.json')
   for k in ['time_ns','right_time_ns','stems']:p[k]=p[k][:count]
   p['prepared_images']=p['prepared_images'][:count*2]
   (d/'settings.yaml').write_bytes((old/'settings.yaml').read_bytes())
   (d/'inputs.tsv').write_text(''.join((old/'inputs.tsv').read_text().splitlines(keepends=True)[:count]))
   pins={str(ROOT/'MAC-VO-main/DataLoader/Dataset/MS2.py'):sha(ROOT/'MAC-VO-main/DataLoader/Dataset/MS2.py'),str(ROOT/'tools/orb_ms2_development.py'):sha(ROOT/'tools/orb_ms2_development.py')}
   for row in source['records']:
    for m in ['rgb','thr']:
     for side in ['left','right']:
      item=row['cameras'][m][side];f=ROOT/item['path']
      if sha(f)!=item['sha256']:raise ValueError('raw source hash')
      pins[str(f)]=item['sha256']
   for item in p['prepared_images']:
    f=local(item['path'])
    if sha(f)!=item['sha256']:raise ValueError('canonical image hash')
    pins[str(f)]=item['sha256']
   for f in [namespace/'source-manifest.json',d/'settings.yaml',d/'inputs.tsv']:pins[str(f)]=sha(f)
   p.update(frozen_files=pins,source_manifest_sha256=sha(namespace/'source-manifest.json'),scope='EXP146 fixed native observation prefix')
   write(d/'plan.json',p)
 print('PREPARED',count,namespace,flush=True)

def freeze(runtime):
 pins={}
 files=[ROOT/'tools/native_stereo_history.py',ROOT/'tools/native_stereo_history_audit.py',ROOT/'tools/tests/test_native_stereo_history.py',ROOT/'environment/orb_stereo_history_runner.cc',ROOT/'environment/orb_stereo_history_CMakeLists.txt',ROOT/'environment/Dockerfile.orb-stereo-history',ROOT/'docs/research/native-stereo-history-protocol-2026-11-05.md',OUT/'source-manifest.json',OUT/'runtime-identity.json']
 prior=read(ROOT/'experiments/orb-finalization-2026-11-05.json');files += [ROOT/n for n in prior['source_sha256'] if not n.startswith('results/')]
 files += [ROOT/'tools/stereo_reference_diagnostic.py',ROOT/'tools/stereo_reference_audit.py',ROOT/'results/ms2-night-pilot/calibration.json',ROOT/'results/provided-reference-full-record/run-001/reference-lock.json']
 for d in (OUT/'captures').iterdir():
  files += [d/n for n in ['plan.json','settings.yaml','inputs.tsv']]
 for f in files:pins[str(f.relative_to(ROOT))]=sha(f)
 write(PLAN,dict(parent='89574849b402e15220396ccc4a9be2e1054c3e43',parameters=PARAMS,source_groups=2000,base_runtime=BASE,observed_runtime=runtime,parent_runtime=PARENT_RUNTIME,score_runtime=SCORE_RUNTIME,source_sha256=pins,physical_sync_verified=False,benchmark_execution_ready=False,scale_fitted=False,time_fitted=False,reference_outcomes_previously_exposed=True))
 (OUT/'study-lock.json').write_bytes(PLAN.read_bytes());print('FROZEN',len(pins),runtime)

def verify():
 p=read(PLAN)
 if p['parameters']!=PARAMS or sha(PLAN)!=sha(OUT/'study-lock.json'):raise ValueError('study lock/parameters')
 for n,h in p['source_sha256'].items():
  if sha(ROOT/n)!=h:raise ValueError('changed source '+n)
 return p

def run_arm(mode,arm,namespace=OUT):
 from orb_ms2_development import run as old_run,publish
 from orb_full_record import evaluation_view,failure_view
 from orb_finalization import final_view
 d=namespace/'captures'/(mode+'-'+arm);p=read(d/'plan.json');write(d/'attempt-lock.json',dict(mode=mode,arm=arm,start_utc_ns=time.time_ns()))
 start=time.monotonic_ns();error=None;legacy=None
 try:
  if arm=='control':legacy=old_run(d,timeout=3600)
  else:
   for n,h in p['frozen_files'].items():
    if sha(n)!=h:raise ValueError('source pin '+n)
   runner='/opt/ore-ms-stereo-adapter/build/orb_stereo_history_runner';cmd=[runner,'/opt/ORB_SLAM3/Vocabulary/ORBvoc.txt',str(d/'settings.yaml'),str(d/'inputs.tsv'),str(d/'native.jsonl'),'0','1']
   write(d/'execution-lock.json',dict(command=cmd,runner_sha256=sha(runner),library_sha256=sha('/opt/ORB_SLAM3/lib/libORB_SLAM3.so'),plan_sha256=sha(d/'plan.json')))
   with (d/'native.log').open('x') as f:
    try:result=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,timeout=3600);code=result.returncode
    except subprocess.TimeoutExpired:code=124
   native=[json.loads(s) for s in (d/'native.jsonl').read_text().splitlines()];legacy=publish(p,native,code)
   configs=[r['configuration'] for r in native if 'configuration' in r]
   legacy['native_calibration_verified']=len(configs)==1 and configs[0]['rerectifies'] is False and configs[0]['resizes'] is False and all(np.isclose(configs[0][k],v,rtol=5e-7,atol=1e-7) for k,v in p['calibration'].items())
   if not legacy['native_calibration_verified']:legacy['terminal_success']=False
   write(d/'estimate.json',legacy)
  received,boundaries=evaluation_view(p,legacy)
  if arm=='paired':
   rows=[json.loads(s) for s in (d/'native.jsonl.final.jsonl').read_text().splitlines()];final=final_view(p,{**received,'rows':legacy['rows']},rows);write(d/'final.json',final)
 except Exception:
  error=traceback.format_exc();received=failure_view(p,error);boundaries={}
 write(d/'received.json',received)
 write(d/'run-status.json',dict(mode=mode,arm=arm,wall_ns=time.monotonic_ns()-start,source_frames=len(p['stems']),received_poses=sum(v is not None for v in received['poses']),terminal_success=received['terminal_success'],boundaries=boundaries,error=error))
 print('FINISHED',mode,arm,'supported',sum(v is not None for v in received['poses']),'terminal',received['terminal_success'],flush=True)

def frame_stats(row):
 features=np.asarray([v[:5] for v in row['features']],dtype=float).reshape(-1,5)
 if len(features)!=row['N']:raise ValueError('native feature population')
 z=features[:,4];u=features[:,3];valid=z>0
 if np.any(valid!=(u>=0)) or np.any(z[~valid]!=-1) or np.any(u[~valid]!=-1):raise ValueError('native stereo sentinels')
 d=features[valid,0]-u[valid]
 if np.any(d<=0) or not np.isfinite(features).all():raise ValueError('positive finite disparity')
 def med(x):return float(np.median(x)) if len(x) else None
 flags=[v for v in row['features'] if v[4]>0]
 return dict(index=row['index'],features=row['N'],right_features=row['right_keypoints'],accepted=int(sum(valid)),acceptance=float(np.mean(valid)) if len(valid) else None,median_disparity_px=med(d),median_depth_m=med(z[valid]),close=int(sum(z[valid]<row['close_depth_m'])),far=int(sum(z[valid]>=row['close_depth_m'])),associated_accepted=sum(v[5] is True for v in flags),flagged_outlier_accepted=sum(v[6] is True for v in flags),missing_map_flags=sum(v[5] is None for v in flags),missing_outlier_flags=sum(v[6] is None for v in flags))

def lock():
 verify();pins={}
 for d in (OUT/'captures').iterdir():
  for f in d.iterdir():
   if f.is_file():pins[str(f.relative_to(ROOT))]=sha(f)
 write(OUT/'prediction-lock.json',dict(source_sha256=pins,study_sha256=sha(PLAN),all_four_attempts_retained=True,new_reference_scores_read=False))

def analyze():
 from independent_benchmark_metrics_v2 import score,pose
 from orb_finalization import common_views
 from stereo_reference_diagnostic import decompose,summarize,stats
 from scipy.stats import spearmanr
 p=verify();locked=read(OUT/'prediction-lock.json')
 for n,h in locked['source_sha256'].items():
  if sha(ROOT/n)!=h:raise ValueError('prediction changed')
 source=read(OUT/'source-manifest.json');refslock=read(ROOT/'results/provided-reference-full-record/run-001/reference-lock.json');summaries=[]
 for mode,m in [('rgb','rgb'),('thermal','thr')]:
  d=OUT/'captures'/(mode+'-paired');received=read(d/'received.json');final=read(d/'final.json');control=read(OUT/'captures'/(mode+'-control')/'received.json')
  views,mask=common_views(received,final,final);views=views[:2];raw=[]
  for row in source['records']:
   f=ROOT/'results/provided-reference-full-record/run-001/reference'/source['sequence']/m/(row['stem']+'.txt')
   if sha(f)!=refslock['member_sha256'][str(f.relative_to(ROOT))]:raise ValueError('reference pin')
   raw.append(np.loadtxt(f).reshape(4,4).tolist())
  scores={};motions={}
  for name,view in [('control',control),('received',views[0]),('final',views[1])]:
   scoreout=score(dict(time_ns=view['time_ns'],poses=raw),view);write(OUT/'scores'/mode/(name+'.json'),scoreout);write(OUT/'scores'/mode/(name+'-view.json'),view)
   scores[name]={k:v for k,v in scoreout.items() if k not in ['rpe_rows','segment_ate','reference_exclusions']};pairs=[]
   for pair in scoreout['rpe_rows']:
    i,j=pair['source_indices'];g=np.linalg.solve(pose(raw[i])[0],pose(raw[j])[0]);e=np.linalg.solve(pose(view['poses'][i])[0],pose(view['poses'][j])[0]);v=decompose(g[:3,3],e[:3,3]);v.update(indices=[i,j]);pairs.append(v)
   motions[name]=dict(rows=pairs,summary=summarize(pairs))
  frames=[]
  with (d/'native.jsonl.stereo.jsonl').open() as f:
   for line in f:frames.append(frame_stats(json.loads(line)))
  if len(frames)!=2000:raise ValueError('all native stereo source slots required')
  bins=[]
  for a,b in PARAMS['bins']:
   cohort=frames[a:b];bins.append(dict(start=a,end_exclusive=b,frames=b-a,accepted=stats([v['accepted'] for v in cohort]),acceptance=stats([v['acceptance'] for v in cohort if v['acceptance'] is not None]),median_disparity_px=stats([v['median_disparity_px'] for v in cohort if v['median_disparity_px'] is not None]),median_depth_m=stats([v['median_depth_m'] for v in cohort if v['median_depth_m'] is not None]),final_motion=summarize([r for r in motions['final']['rows'] if a<=r['indices'][0]<b])))
  times=received['time_ns'];right=read(d/'plan.json')['right_time_ns'];delta=[r-l for r,l in zip(right,times)];strata=[]
  for label in ['negative','zero','positive']:
   pairs=[r for r in motions['final']['rows'] if ('negative' if delta[r['indices'][0]]<0 else 'positive' if delta[r['indices'][0]]>0 else 'zero')==label];strata.append(dict(recorded_delta_sign=label,motion=summarize(pairs)))
  correlations={}
  for key in ['accepted','acceptance','median_disparity_px','median_depth_m','signed_delta_ms']:
   cohort=[r for r in motions['final']['rows'] if key=='signed_delta_ms' or frames[r['indices'][0]][key] is not None]
   xs=[delta[r['indices'][0]]/1e6 if key=='signed_delta_ms' else frames[r['indices'][0]][key] for r in cohort];ys=[r['ratio'] for r in cohort]
   result=spearmanr(xs,ys).statistic if len(set(xs))>1 and len(set(ys))>1 else None
   correlations[key]=dict(pairs=len(cohort),rho=float(result) if result is not None and np.isfinite(result) else None,causal=False)
  write(OUT/'analysis'/mode/'frames.json',dict(rows=frames));write(OUT/'analysis'/mode/'motion.json',motions)
  summaries.append(dict(modality=mode,source_frames=2000,observed_pose_frames=sum(v is not None for v in received['poses']),common_pose_frames=sum(mask),scores=scores,frame_summary={key:stats([v[key] for v in frames if v[key] is not None]) for key in ['features','right_features','accepted','acceptance','median_disparity_px','median_depth_m','close','far','associated_accepted','flagged_outlier_accepted']},total_features=sum(v['features'] for v in frames),total_accepted=sum(v['accepted'] for v in frames),bins=bins,recorded_sign_strata=strata,spearman=correlations,motion_summary={k:v['summary'] for k,v in motions.items()}))
 write(OUT/'summary.json',dict(runs=summaries,physical_sync_verified=False,benchmark_execution_ready=False,scale_fitted=False,time_fitted=False,causal_attribution=False))
 verify();print('ANALYZED BOTH CAMERAS')

if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('action',choices=['prepare','freeze','verify','run','lock','analyze']);a.add_argument('--runtime');a.add_argument('--mode');a.add_argument('--arm');a.add_argument('--count',type=int,default=2000);a.add_argument('--namespace');x=a.parse_args();ns=ROOT/x.namespace if x.namespace else OUT
 {'prepare':lambda:prepare(x.count,ns),'freeze':lambda:freeze(x.runtime),'verify':verify,'run':lambda:run_arm(x.mode,x.arm,ns),'lock':lock,'analyze':analyze}[x.action]()
