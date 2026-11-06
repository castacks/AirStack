"""EXP151 fixed initialized day/night stereo, conditional depth and ambiguity."""
import argparse,json,hashlib,subprocess,urllib.request,time,math
from pathlib import Path
from collections import Counter
import numpy as np
import cv2
from native_depth_proxy import feature_result,stats,METRICS
from depth_projection_sensitivity import neighborhood
from native_match_ambiguity import ambiguity,reconstruct,correlation
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/cross-recording-stereo-depth/run-001';PLAN=ROOT/'experiments/cross-recording-stereo-depth-2026-11-06.json'
PARAMS=dict(recordings=['day','night'],indices=list(range(0,2000,100)),modalities=['rgb','thermal'],units_per_metre=256,local_radius=2,minimum_candidates=3,maximum_relative_span=.10,minimum_rank_points=10,variables=['descriptor_margin','descriptor_ratio','patch_relative_gap'],groups=['all_primary','locally_consistent'],compressed_budget_bytes=1024**3,timeout_seconds=600)
SCORE='sha256:d3cf8a41dd8090c8939cd840b4735d526cec9a54e48ac36d506f91d13ee56e1c'
def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def write(p,v):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')
def local(s):
 for prefix in ['/workspace/','/root/AirStack/ore-ms/']:
  if s.startswith(prefix):return ROOT/s[len(prefix):]
 return ROOT/s
def prepare():
 records=[];pins={}
 for recording in PARAMS['recordings']:
  manifest=ROOT/'results/bounded-full-record/manifests'/(recording+'.json');source=read(manifest);pins[str(manifest.relative_to(ROOT))]=sha(manifest)
  for mode,camera in [('rgb','rgb'),('thermal','thr')]:
   old=ROOT/'results/bounded-full-record/canonical'/(recording+'-'+mode);plan=read(old/'plan.json');pins[str((old/'plan.json').relative_to(ROOT))]=sha(old/'plan.json');images={(Path(v['path']).parent.name,Path(v['path']).stem):v for v in plan['prepared_images']};lines=[]
   key=recording+'-'+mode;settings=OUT/'settings'/(key+'.yaml');settings.parent.mkdir(parents=True,exist_ok=True);settings.write_bytes((old/'settings.yaml').read_bytes())
   for index in PARAMS['indices']:
    r=source['records'][index];stem=r['stem'];item=r['cameras'][camera];paths=[]
    for side in ['left','right']:
     im=images[(side,stem)];f=local(im['path']);assert sha(f)==im['sha256'];pins[str(f.relative_to(ROOT))]=im['sha256'];paths.append('/workspace/'+str(f.relative_to(ROOT)))
     raw=ROOT/item[side]['path'];assert sha(raw)==item[side]['sha256'];pins[str(raw.relative_to(ROOT))]=item[side]['sha256']
    records.append(dict(recording=recording,modality=mode,camera=camera,index=index,stem=stem,sequence=source['sequence'],shape=item['left']['shape'][:2],K=item['K'],baseline_m=item['baseline_m'],time_ns=plan['time_ns'][index],right_time_ns=plan['right_time_ns'][index]))
    lines.append('\t'.join([str(index),*paths,str(plan['time_ns'][index]),','])+'\n')
   f=OUT/'inputs'/(key+'.tsv');f.parent.mkdir(exist_ok=True);f.write_text(''.join(lines))
 for name in ['setup','data-format']:
  url='https://sites.google.com/view/multi-spectral-stereo-dataset/'+name
  with urllib.request.urlopen(url,timeout=45) as response:data=response.read(5_000_001)
  assert len(data)<=5_000_000;dest=OUT/'source'/(name+'.html');dest.write_bytes(data)
  write(OUT/'source'/(name+'.json'),dict(url=url,sha256=sha(dest),retrieved_utc_ns=time.time_ns(),physical_export_semantics_resolved=False))
 write(OUT/'prepared.json',dict(records=records,image_sha256=pins));print('PREPARED80fixed image pairs; no new depth numeric decode',flush=True)
def freeze(runtime):
 files=[Path(__file__),ROOT/'tools/cross_recording_stereo_depth_acquire.py',ROOT/'tools/cross_recording_stereo_depth_audit.py',ROOT/'tools/tests/test_cross_recording_stereo_depth.py',ROOT/'tools/native_depth_proxy.py',ROOT/'tools/native_depth_proxy_audit.py',ROOT/'tools/native_match_ambiguity.py',ROOT/'tools/native_match_ambiguity_audit.py',ROOT/'tools/depth_projection_sensitivity.py',ROOT/'tools/depth_projection_sensitivity_audit.py',ROOT/'tools/ms2_broader_intake.py',ROOT/'environment/orb_match_ambiguity_runner.cc',ROOT/'environment/orb_match_ambiguity_CMakeLists.txt',ROOT/'environment/Dockerfile.orb-cross-recording-depth',ROOT/'docs/research/cross-recording-stereo-depth-protocol-2026-11-06.md',ROOT/'docs/research/ms2-clock-evidence-findings-2026-11-04.md',OUT/'prepared.json',OUT/'runtime-identity.json',ROOT/'results/stereo-baseline-initialization/run-001/runtime-identities.json',ROOT/'results/ms2-day-replication/archive-manifest.json',ROOT/'results/ms2-day-replication/proj_depth-tar-index.json',ROOT/'results/ms2-day-replication/archives/proj_depth-block-offsets.json',ROOT/'results/public-data-audit/ms2-access.json',ROOT/'results/native-depth-proxy/run-001/acquisition.json']
 files+=list((OUT/'source').glob('*'))+list((OUT/'settings').glob('*'))+list((OUT/'inputs').glob('*'))+list((ROOT/'results/bounded-full-record/source-001/wheels').glob('*.whl'))
 prepared=read(OUT/'prepared.json');members=[];base=ROOT/'results/ms2-day-replication';index={v['name']:v for v in read(base/'proj_depth-tar-index.json') if v['regular']}
 for r in prepared['records']:
  name=r['sequence']+'/'+r['camera']+'/depth/'+r['stem']+'.png'
  if r['recording']=='day':
   members.append(dict(modality=r['modality'],index=r['index'],shape=r['shape'],**index[name]));old=base/'pilot/proj_depth'/name
   if old.exists():files.append(old)
  else:
   item=next(v for v in read(ROOT/'results/native-depth-proxy/run-001/acquisition.json')['members'] if v['name']==name);files.append(ROOT/item['path'])
 pins={str(f.relative_to(ROOT)):sha(f) for f in files};pins.update(prepared['image_sha256']);seq=next(r['sequence'] for r in prepared['records'] if r['recording']=='day');archive=next(v for v in read(base/'archive-manifest.json') if v['kind']=='proj_depth');url=next(v['url'] for v in read(ROOT/'results/public-data-audit/ms2-access.json') if v['kind']=='proj_depth' and v['sequence']==seq)
 p=dict(parent='ac162c341908fa8a23d32839b8fbeaa8a42b3f91',parameters=PARAMS,runtime=runtime,score_runtime=SCORE,source_sha256=pins,members=members,archive_url=url,archive_bytes=archive['bytes'],historical_full_archive_sha256=archive['sha256'],block_offsets='results/ms2-day-replication/archives/proj_depth-block-offsets.json',source_base='results/ms2-day-replication',physical_reference_qualified=False,physical_sync_verified=False,benchmark_execution_ready=False,matcher_tuned=False,new_trajectory=False)
 write(PLAN,p);write(OUT/'study-lock.json',p)
 write(ROOT/'experiments/recording-exposure-ledger-2026-11-06-cross-recording-depth.json',dict(recordings=sorted({r['sequence'] for r in prepared['records']}),development_exposed=True,indices=PARAMS['indices'],new_day_depth_maps=40,night_maps_previously_exposed=40,reserves_unchanged=True,new_numeric_depth_read=False))
 print('FROZEN',len(pins),'pins',runtime,flush=True)
def verify():
 p=read(PLAN);assert sha(PLAN)==sha(OUT/'study-lock.json') and p['parameters']==PARAMS
 for n,h in p['source_sha256'].items():
  if sha(ROOT/n)!=h:raise ValueError('frozen source '+n)
 return p
def replay(stage):
 verify();targets=read(OUT/'targets.json') if stage=='costs' else None
 if targets:
  for n,h in read(OUT/'geometry-lock.json')['source_sha256'].items():assert sha(ROOT/n)==h
 for recording in PARAMS['recordings']:
  for mode in PARAMS['modalities']:
   key=recording+'-'+mode;input0=OUT/'inputs'/(key+'.tsv');dest=OUT/stage;dest.mkdir(exist_ok=True)
   if targets:
    input0=dest/(key+'.tsv');lines=[]
    for line,r in zip((OUT/'inputs'/(key+'.tsv')).read_text().splitlines(),[v for v in targets['records'] if v['recording']==recording and v['modality']==mode]):
     fields=line.split('\t');assert int(fields[0])==r['index'];fields[4]=','.join(str(i) for i in r['indices']) or ',';lines.append('\t'.join(fields)+'\n')
    input0.write_text(''.join(lines))
   runner='/opt/ore-ms-ambiguity/build/orb_match_ambiguity_runner';cmd=[runner,str(OUT/'settings'/(key+'.yaml')),str(input0),str(dest/(key+'.jsonl'))]
   write(dest/(key+'-execution.json'),dict(command=cmd,runner_sha256=sha(runner),library_sha256=sha('/opt/ORB_SLAM3/lib/libORB_SLAM3.so'),inputs_sha256=sha(input0)))
   with (dest/(key+'.log')).open('x') as f:
    try:code=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,timeout=600).returncode
    except subprocess.TimeoutExpired:code=124
   write(dest/(key+'-status.json'),dict(returncode=code,terminal_success=code==0));assert code==0
   print(stage,key,'completed',flush=True)
 if stage=='geometry':write(OUT/'geometry-lock.json',dict(source_sha256={str(f.relative_to(ROOT)):sha(f) for f in sorted((OUT/stage).glob('*'))},new_day_depth_numeric_read=False))
 else:write(OUT/'cost-lock.json',dict(source_sha256={str(f.relative_to(ROOT)):sha(f) for f in sorted((OUT/stage).glob('*'))},targets_sha256=sha(OUT/'targets.json'),new_ambiguity_associations_read=False))
def label(r):
 if r['recording']=='day':acq=read(OUT/'acquisition.json')
 else:acq=read(ROOT/'results/native-depth-proxy/run-001/acquisition.json')
 name=r['sequence']+'/'+r['camera']+'/depth/'+r['stem']+'.png';item=next(v for v in acq['members'] if v['name']==name);f=ROOT/item['path'];assert sha(f)==item['sha256'];depth=cv2.imread(str(f),-1);assert depth.dtype==np.uint16 and list(depth.shape)==r['shape'];return depth,item

def evaluate_depth():
 verify()
 for n,h in read(OUT/'geometry-lock.json')['source_sha256'].items():assert sha(ROOT/n)==h
 records=read(OUT/'prepared.json')['records'];targets=[];cases=[]
 for recording in PARAMS['recordings']:
  for mode in PARAMS['modalities']:
   frames=[json.loads(l) for l in (OUT/'geometry'/(recording+'-'+mode+'.jsonl')).read_text().splitlines()];source=[r for r in records if r['recording']==recording and r['modality']==mode];assert [v['index'] for v in frames]==[r['index'] for r in source]==PARAMS['indices']
   for frame,r in zip(frames,source):
    depth,item=label(r);points=[]
    for i,five in enumerate(frame['features']):
     f=five+[None,None];v=feature_result(f,depth,r['K'][0][0],r['baseline_m'],r['shape'][1]);v.update(feature_index=i,native=f)
     if v['reason']=='compared':v['local']=neighborhood(f,depth,r['K'][0][0],r['baseline_m'],2)
     points.append(v)
    primary=[v for v in points if v['reason']=='compared'];groups={}
    for group in PARAMS['groups']:
     data=primary if group=='all_primary' else [v for v in primary if v['local']['group']=='locally_consistent']
     groups[group]=dict(points=len(data),metrics={k:stats([v['metrics'][k] for v in data]) for k in METRICS})
    case=dict(recording=recording,modality=mode,index=r['index'],features=len(points),accepted=sum(v['native_accepted'] for v in points),reasons=dict(Counter(v['reason'] for v in points)),unique_reference_pixels=len({tuple(v['pixel']) for v in primary}),groups=groups)
    write(OUT/'depth'/recording/mode/(r['stem']+'.json'),dict(case=case,points=points,label_sha256=item['sha256']));cases.append(case);targets.append(dict(recording=recording,modality=mode,index=r['index'],indices=[v['feature_index'] for v in primary]))
 agg=[]
 for rec in PARAMS['recordings']:
  for mode in PARAMS['modalities']:
   for group in PARAMS['groups']:
    data=[c['groups'][group] for c in cases if c['recording']==rec and c['modality']==mode];agg.append(dict(recording=rec,modality=mode,group=group,image_cases=len(data),images_nonempty=sum(v['points']>0 for v in data),points=sum(v['points'] for v in data),per_image_median={k:stats([v['metrics'][k]['median'] for v in data if v['metrics'][k]['median'] is not None]) for k in METRICS}))
 write(OUT/'targets.json',dict(records=targets,rule='all primary supported accepted features; no error or ambiguity filter'))
 write(OUT/'depth-summary.json',dict(cases=cases,aggregates=agg,physical_reference_qualified=False,physical_sync_verified=False,benchmark_execution_ready=False,scale_fitted=False,time_fitted=False))
 print('DEPTH80cases; targets frozen',sum(len(v['indices']) for v in targets),flush=True)
def analyze_costs():
 verify();lock=read(OUT/'cost-lock.json');assert sha(OUT/'targets.json')==lock['targets_sha256']
 for n,h in lock['source_sha256'].items():assert sha(ROOT/n)==h
 cases=[];fidelity=[]
 for recording in PARAMS['recordings']:
  for mode in PARAMS['modalities']:
   key=recording+'-'+mode;geometry=list(map(json.loads,(OUT/'geometry'/(key+'.jsonl')).read_text().splitlines()));frames=list(map(json.loads,(OUT/'costs'/(key+'.jsonl')).read_text().splitlines()))
   for frame,original in zip(frames,geometry):
    assert frame['index']==original['index'] and frame['features']==original['features'] and frame['right_features']==original['right_features'];index=frame['index'];depth=read(OUT/'depth'/recording/mode/(f'{index:06d}.json'));primary=[v for v in depth['points'] if v['reason']=='compared'];assert [p['feature_index'] for p in frame['points']]==[p['feature_index'] for p in primary];points=[]
    for native,target in zip(frame['points'],primary):
     exact=reconstruct(native,frame,frame['features'][target['feature_index']])==target['native'][3:5]
     points.append(dict(feature_index=target['feature_index'],faithful=exact,reference_group=target['local']['group'],original_metrics=target['metrics'],ambiguity=ambiguity(native)))
    groups={}
    for group in PARAMS['groups']:
     subset=points if group=='all_primary' else [p for p in points if p['reference_group']=='locally_consistent'];groups[group]=dict(points=len(subset),faithful=sum(p['faithful'] for p in subset),correlations={key:correlation(subset,key) for key in PARAMS['variables']})
    case=dict(recording=recording,modality=mode,index=index,groups=groups);write(OUT/'ambiguity'/recording/mode/(f'{index:06d}.json'),dict(case=case,points=points));cases.append(case);fidelity.append(dict(recording=recording,modality=mode,index=index,geometry_exact=True,points=len(points),native_cost_exact=sum(p['faithful'] for p in points)))
 agg=[]
 for rec in PARAMS['recordings']:
  for mode in PARAMS['modalities']:
   for group in PARAMS['groups']:
    data=[c['groups'][group] for c in cases if c['recording']==rec and c['modality']==mode];agg.append(dict(recording=rec,modality=mode,group=group,points=sum(v['points'] for v in data),faithful=sum(v['faithful'] for v in data),correlations={key:dict(images_defined=sum(v['correlations'][key]['rho'] is not None for v in data),negative_images=sum(v['correlations'][key]['rho'] is not None and v['correlations'][key]['rho']<0 for v in data),stats=stats([v['correlations'][key]['rho'] for v in data if v['correlations'][key]['rho'] is not None])) for key in PARAMS['variables']}))
 write(OUT/'fidelity.json',dict(cases=fidelity,all_geometry_exact=True,points=sum(v['points'] for v in fidelity),native_cost_exact=sum(v['native_cost_exact'] for v in fidelity)))
 write(OUT/'summary.json',dict(depth=read(OUT/'depth-summary.json'),ambiguity=dict(cases=cases,aggregates=agg),physical_reference_qualified=False,physical_sync_verified=False,benchmark_execution_ready=False,matcher_tuned=False,new_trajectory=False))
 print('ANALYZED80cross-recording cases',flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('action',choices=['prepare','freeze','verify','geometry','acquire','depth','costs','analyze']);p.add_argument('--runtime');a=p.parse_args()
 if a.action=='acquire':
  from cross_recording_stereo_depth_acquire import acquire
  acquire()
 else:{'prepare':prepare,'freeze':lambda:freeze(a.runtime),'verify':verify,'geometry':lambda:replay('geometry'),'depth':evaluate_depth,'costs':lambda:replay('costs'),'analyze':analyze_costs}[a.action]()
