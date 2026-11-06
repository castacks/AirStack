"""EXP150 same-observer original/rebuild/repaired stereo initialization study."""
import argparse,json,hashlib,subprocess,time,traceback
from pathlib import Path
from copy import deepcopy
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/stereo-baseline-initialization/run-001'
PRIOR=ROOT/'results/orb-finalization/run-001'
PLAN=ROOT/'experiments/stereo-baseline-initialization-2026-11-05.json'
ORIGINAL='sha256:83739d72791f554a61e063c7868fe982a687c89e121129a2bc8d3819a532e50a'
PARAMS=dict(arms=['original','rebuilt','fixed'],source_groups=2000,seed=0,threads=1,timeout_seconds=3600,bins=[[0,500],[500,1000],[1000,1500],[1500,2000]])
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
 source['source_frames']=count;source['scope']='EXP150 fixed original night prefix'
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
   p.update(frozen_files=pins,source_manifest_sha256=sha(namespace/'source-manifest.json'),scope='EXP150 fixed native observation prefix')
   write(d/'plan.json',p)
 print('PREPARED',count,namespace,flush=True)

def run_arm(mode,arm,namespace=OUT):
 from orb_ms2_development import publish
 from orb_full_record import evaluation_view,failure_view
 from orb_finalization import final_view
 d=namespace/'captures'/(mode+'-'+arm);p=read(d/'plan.json');write(d/'attempt-lock.json',dict(mode=mode,arm=arm,start_utc_ns=time.time_ns()))
 start=time.monotonic_ns();error=None;legacy=None
 try:
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
  rows=[json.loads(s) for s in (d/'native.jsonl.final.jsonl').read_text().splitlines()];final=final_view(p,{**received,'rows':legacy['rows']},rows);write(d/'final.json',final)
 except Exception:
  error=traceback.format_exc();received=failure_view(p,error);boundaries={}
 write(d/'received.json',received)
 write(d/'run-status.json',dict(mode=mode,arm=arm,wall_ns=time.monotonic_ns()-start,source_frames=len(p['stems']),received_poses=sum(v is not None for v in received['poses']),terminal_success=received['terminal_success'],boundaries=boundaries,error=error))
 print('FINISHED',mode,arm,'supported',sum(v is not None for v in received['poses']),'terminal',received['terminal_success'],flush=True)

def union_views(streams):
 n=len(streams[0]['poses']);mask=[all(s['poses'][i] is not None for s in streams) for i in range(n)];segments=[];sid=-1;active=False;previous=None
 for i,ok in enumerate(mask):
  if not ok:segments.append(None);active=False;continue
  signature=tuple(s['segments'][i] for s in streams)
  if not active or signature!=previous:sid+=1
  segments.append(sid);active=True;previous=signature
 views=[]
 for s in streams:views.append(dict(time_ns=s['time_ns'],source_ids=s['source_ids'],poses=[p if ok else None for p,ok in zip(s['poses'],mask)],segments=segments,terminal_success=s['terminal_success']))
 return views,mask

def compare_frames(a,b):
 if a.get('available') is False or b.get('available') is False:return dict(comparable=False,coordinates_equal=None,accepted_changes=None,right_changes=None,depth_changes=None)
 x=np.asarray([v[:5] for v in a['features']],float).reshape(-1,5);y=np.asarray([v[:5] for v in b['features']],float).reshape(-1,5);coords=x.shape==y.shape and np.array_equal(x[:,:3],y[:,:3])
 if not coords:return dict(comparable=True,coordinates_equal=False,accepted_changes=None,right_changes=None,depth_changes=None)
 return dict(comparable=True,coordinates_equal=True,accepted_changes=int(sum((x[:,4]>0)!=(y[:,4]>0))),right_changes=int(sum(x[:,3]!=y[:,3])),depth_changes=int(sum(x[:,4]!=y[:,4])))

def validate_probes():
 reports=read(OUT/'probes/driver.json');patched=[]
 for r in reports:
  if r['arm']=='fixed':
   if r['returncode']!=0:raise ValueError('fixed constructor probe failed')
   patched.append(read(ROOT/r['output']))
 if len(patched)!=3 or any(p['features']!=patched[0]['features'] for p in patched):raise ValueError('fixed geometry depends on storage pattern')
 accepted=[p for p in patched[0]['features'] if p[4]>0];disparity=[p[0]-p[3] for p in accepted]
 if not accepted or abs(float(np.median(disparity))-4)>.5:raise ValueError('synthetic known-disparity check')
 write(OUT/'probe-validation.json',dict(passed=True,fixed_patterns=[p['pattern'] for p in patched],fixed_features=patched[0]['N'],fixed_accepted=len(accepted),median_disparity=float(np.median(disparity)),fixed_geometry_identical=True,original_characterization=[r for r in reports if r['arm']!='fixed']))
 print('PROBES fixed invariant and known disparity valid')

def freeze(rebuilt,fixed):
 files=[Path(__file__),ROOT/'tools/stereo_baseline_initialization_audit.py',ROOT/'tools/tests/test_stereo_baseline_initialization.py',ROOT/'environment/patch_orb_stereo_baseline.py',ROOT/'environment/orb_initialization_probe.cc',ROOT/'environment/orb_initialization_probe_CMakeLists.txt',ROOT/'environment/Dockerfile.orb-baseline-initialization',ROOT/'docs/research/stereo-baseline-initialization-protocol-2026-11-05.md',OUT/'source-manifest.json',OUT/'runtime-identities.json',OUT/'probe-validation.json',ROOT/'results/provided-reference-full-record/run-001/reference-lock.json']
 files += [ROOT/'tools/stereo_baseline_initialization_plot.py',OUT.parent/'integration-001/independent-audit.json']
 files += sorted((OUT/'probes').glob('*.json'))
 files += sorted((OUT/'validation').glob('*.py'))
 for name in ['orb_ms2_development.py','orb_full_record.py','orb_finalization.py','orb_finalization_audit.py','independent_benchmark_metrics_v2.py','provided_reference_full_record_audit.py','native_stereo_history.py','native_stereo_history_audit.py','stereo_reference_diagnostic.py','stereo_reference_audit.py']:files.append(ROOT/'tools'/name)
 for d in (OUT/'captures').iterdir():files += [d/n for n in ['plan.json','settings.yaml','inputs.tsv']]
 p=dict(parent='ed971d3cb70446f6884bb1745732fd9d5088dbd2',parameters=PARAMS,source_sha256={str(f.relative_to(ROOT)):sha(f) for f in files},runtimes=dict(original=ORIGINAL,rebuilt=rebuilt,fixed=fixed),score_runtime='sha256:d3cf8a41dd8090c8939cd840b4735d526cec9a54e48ac36d506f91d13ee56e1c',physical_sync_verified=False,benchmark_execution_ready=False,scale_fitted=False,time_fitted=False,reference_outcomes_previously_exposed=True,default_runtime_changed=False)
 write(PLAN,p);write(OUT/'study-lock.json',p);print('FROZEN',len(files),'source pins')

def verify():
 p=read(PLAN)
 if sha(PLAN)!=sha(OUT/'study-lock.json') or p['parameters']!=PARAMS:raise ValueError('study/parameter lock')
 for n,h in p['source_sha256'].items():
  if sha(ROOT/n)!=h:raise ValueError('source changed '+n)
 return p

def lock():
 verify();pins={}
 for d in (OUT/'captures').iterdir():
  for f in d.iterdir():
   if f.is_file():pins[str(f.relative_to(ROOT))]=sha(f)
 write(OUT/'prediction-lock.json',dict(source_sha256=pins,all_six_attempts_retained=True,new_reference_scores_read=False,study_sha256=sha(PLAN)))

def analyze():
 from independent_benchmark_metrics_v2 import score,pose
 from stereo_reference_diagnostic import decompose,summarize,stats
 from native_stereo_history import frame_stats
 p=verify();lock=read(OUT/'prediction-lock.json')
 for n,h in lock['source_sha256'].items():
  if sha(ROOT/n)!=h:raise ValueError('prediction changed')
 source=read(OUT/'source-manifest.json');refs=read(ROOT/'results/provided-reference-full-record/run-001/reference-lock.json');reports=[]
 for mode,m in [('rgb','rgb'),('thermal','thr')]:
  streams=[];names=[];raw=[]
  for arm in PARAMS['arms']:
   for stage in ['received','final']:
    streams.append(read(OUT/'captures'/(mode+'-'+arm)/(stage+'.json')));names.append(arm+'-'+stage)
  common,mask=union_views(streams)
  for r in source['records']:
   path=ROOT/'results/provided-reference-full-record/run-001/reference'/source['sequence']/m/(r['stem']+'.txt')
   if sha(path)!=refs['member_sha256'][str(path.relative_to(ROOT))]:raise ValueError('reference pin')
   raw.append(np.loadtxt(path).reshape(4,4).tolist())
  scores={};motions={}
  for name,own,matched in zip(names,streams,common):
   for scope,view in [('own',own),('common',matched)]:
    result=score(dict(time_ns=view['time_ns'],poses=raw),view);write(OUT/'scores'/mode/scope/(name+'.json'),result);write(OUT/'scores'/mode/scope/(name+'-view.json'),view)
    scores[scope+'-'+name]={k:v for k,v in result.items() if k not in ['rpe_rows','segment_ate','reference_exclusions']}
    if scope=='common':
     rows=[]
     for pair in result['rpe_rows']:
      i,j=pair['source_indices'];g=np.linalg.solve(pose(raw[i])[0],pose(raw[j])[0]);e=np.linalg.solve(pose(view['poses'][i])[0],pose(view['poses'][j])[0]);v=decompose(g[:3,3],e[:3,3]);v['indices']=[i,j];rows.append(v)
     motions[name]=dict(rows=rows,summary=summarize(rows),bins=[dict(start=a,end_exclusive=b,summary=summarize([r for r in rows if a<=r['indices'][0]<b])) for a,b in PARAMS['bins']])
  stats_by_arm={}
  for arm in PARAMS['arms']:
   records=[]
   with (OUT/'captures'/(mode+'-'+arm)/'native.jsonl.stereo.jsonl').open() as f:
    for line in f:records.append(frame_stats(json.loads(line)))
   if len(records)!=2000:raise ValueError('full native stereo population')
   write(OUT/'analysis'/mode/(arm+'-frames.json'),dict(rows=records))
   stats_by_arm[arm]=dict(available=sum(v['features'] is not None for v in records),missing_indices=[v['index'] for v in records if v['features'] is None],observed_features=sum(v['features'] or 0 for v in records),observed_accepted=sum(v['accepted'] or 0 for v in records),frame_medians={k:stats([v[k] for v in records if v[k] is not None]) for k in ['accepted','median_depth_m','median_disparity_px']})
  comparisons={}
  for a,b in [('original','rebuilt'),('rebuilt','fixed'),('original','fixed')]:
   rows=[]
   with (OUT/'captures'/(mode+'-'+a)/'native.jsonl.stereo.jsonl').open() as fa,(OUT/'captures'/(mode+'-'+b)/'native.jsonl.stereo.jsonl').open() as fb:
    for i,(la,lb) in enumerate(zip(fa,fb)):v=compare_frames(json.loads(la),json.loads(lb));v['index']=i;rows.append(v)
   label=a+'-vs-'+b;write(OUT/'analysis'/mode/(label+'.json'),dict(rows=rows));comparisons[label]=dict(source_frames=len(rows),comparable=sum(v['comparable'] for v in rows),coordinate_mismatches=sum(v['coordinates_equal'] is False for v in rows),accepted_changes=sum(v['accepted_changes'] or 0 for v in rows),right_changes=sum(v['right_changes'] or 0 for v in rows),depth_changes=sum(v['depth_changes'] or 0 for v in rows),changed_source_indices=[v['index'] for v in rows if (v['right_changes'] or v['depth_changes'])])
  write(OUT/'analysis'/mode/'motion.json',motions);reports.append(dict(modality=mode,source_frames=2000,common_pose_frames=sum(mask),scores=scores,motion_summary={k:v['summary'] for k,v in motions.items()},native_stats=stats_by_arm,stereo_comparisons=comparisons))
 write(OUT/'summary.json',dict(runs=reports,physical_sync_verified=False,benchmark_execution_ready=False,scale_fitted=False,time_fitted=False,default_runtime_changed=False))
 verify();print('ANALYZED original/rebuild/fixed, both cameras')

if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('action',choices=['prepare','run','probes','freeze','verify','lock','analyze']);a.add_argument('--count',type=int,default=2000);a.add_argument('--namespace');a.add_argument('--mode');a.add_argument('--arm');a.add_argument('--rebuilt');a.add_argument('--fixed');x=a.parse_args();ns=ROOT/x.namespace if x.namespace else OUT
 {'prepare':lambda:prepare(x.count,ns),'run':lambda:run_arm(x.mode,x.arm,ns),'probes':validate_probes,'freeze':lambda:freeze(x.rebuilt,x.fixed),'verify':verify,'lock':lock,'analyze':analyze}[x.action]()
