"""Independent bounded-run auditing; uses earlier independent metric quaternion checker.
No native worker, capacity helper, inference, reference pose or clock fit imports.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from droid_ms2_audit import check_output

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/bounded-full-record'
CHECKS=0
NORMAL_MODES={'quarter_memory_persistent_state_cap','half_memory_persistent_state_cap'}


def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def need(value,label):
 global CHECKS
 CHECKS+=1
 if not value:raise ValueError(label)


def run_case(p,case,directory,protocol_path,compare=False):
 global CHECKS
 src=read(ROOT/case['manifest']);rows=src['records'];n=len(rows)
 need(n==case['source_frames'] and [r['stem'] for r in rows]==[f'{i:06}' for i in range(n)],'complete original axis')
 v=read(directory/'estimate.json');cap=read(directory/'capacity-status.json');lock=read(directory/'execution-lock.json')
 need(lock['protocol_sha256']==sha(protocol_path) and lock['runtime_image']==p['runtime_image'],'execution identity')
 need({k:lock['settings'][k] for k in p['settings']}==p['settings'],'only allocation changed')
 need(cap['policy']==p['bounded_capacity'] and cap['all_requested_source_slots_retained'] is True,'declared capacity')
 need(cap['original_adapter_sha256']==sha(ROOT/'tools/droid_ms2_development.py') and cap['wrapper_sha256']==sha(ROOT/p.get('worker_source','tools/droid_bounded_worker.py')),'actual source identity')
 slots=p['bounded_capacity']['slots'];limit=slots-17
 if p['bounded_capacity']['mode'] in NORMAL_MODES:
  divisor=2 if p['bounded_capacity']['mode']=='half_memory_persistent_state_cap' else 4
  states=(p['bounded_capacity']['preflight_gpu_total_bytes']//divisor)//4496290
  expected=1
  while 2*expected<=states:expected*=2
  need(slots==expected and slots==(4096 if divisor==2 else 2048),'metadata-derived state budget')
 else:need(case['case']=='day-rgb' and n==101 and slots==32,'declared overflow control')
 vectors=matrices=None
 if (directory/'native-trajectory.npz').exists():
  with np.load(directory/'native-trajectory.npz',allow_pickle=False) as a:vectors=a['xyz_xyzw'].copy();matrices=a['world_from_camera_native'].copy()
 CHECKS+=check_output(v,src,case,vectors,matrices)
 calls=[json.loads(l) for l in (directory/'tracking.jsonl').read_text().splitlines()]
 need(len(calls)==v['native_track_calls_returned']<=n,'returned source calls')
 m='rgb' if case['modality']=='rgb' else 'thr'
 times=[r['cameras'][m]['left']['recorded_time_ns'] for r in rows]
 stamps=np.asarray([(t-times[0])/100000000 for t in times],dtype=np.float32)
 need(np.all(np.diff(stamps)>0),'native clock no collisions')
 for i,r in enumerate(calls):
  need(r['source_index']==i and r['source_stem']==rows[i]['stem'] and r['time_ns']==times[i],'tracking source identity')
  need(r['right_time_ns']==rows[i]['cameras'][m]['right']['recorded_time_ns'] and r['native_time']==float(stamps[i]),'stereo/native clocks')
  need(0<=r['native_keyframe_count']<=limit and r['processing_ns']>=0 and r['receipt_elapsed_ns']>=r['processing_ns'],'state bound and timings')
 compaction=cap.get('feature_buffer_compaction')
 if compaction is not None:
  shape=compaction['shape_per_slot'];active=compaction['active_keyframes'];retained=active+17
  need(shape==[2,128,case['shape'][0]//8,case['shape'][1]//8],'native feature shape')
  need(compaction['original_slots']==4096 and compaction['retained_feature_slots']==retained,'no active/reserve pruning')
  need(compaction['removed_unused_slots']==4096-retained,'unused capacity only')
  per=2*128*shape[2]*shape[3]
  need(compaction['original_feature_elements']==4096*per and compaction['retained_feature_elements']==retained*per,'feature element accounting')
  need(retained*per<2**31-1 and compaction['native32_bit_element_limit']==2**31-1,'native accessor guard')
  need(compaction['all_active_and_reserved_feature_bits_preserved'] is True and compaction['counter_unchanged'] is True,'feature bits/counter preservation')
 if v['native_execution_completed']:need(compaction is not None,'compaction proof mandatory for corrected finalization')
 status=cap['source_statuses'];need(len(status)==n,'all requested status slots')
 events=cap['events'];blocked=next((e['attempt_index'] for e in events if e['kind']=='capacity_blocked_before_native_track'),None)
 if p['bounded_capacity']['mode'] not in NORMAL_MODES:
  need(blocked is not None and v['terminal_success'] is False,'real overflow control actually blocks')
 if blocked is not None:
  need(len(events)==1 and blocked==len(calls)<n and events[0]['native_keyframe_count']>=limit,'guard before native call')
  need(cap['native_track_attempts']==len(calls)+1 and 'NativeCapacityExceeded' in v['failure'],'explicit capacity failure')
  need(v['native_execution_completed'] is False and all(x is None for x in v['poses']),'no salvaged unfinished offline poses')
 for i,s in enumerate(status):
  need(s['source_index']==i and s['source_id']==rows[i]['stem'] and s['time_ns']==times[i],'status lineage')
  need(s['native_track_returned'] is (i<len(calls)),'returned vs attempted distinction')
  need(s['capacity_blocked_before_native_track'] is (i==blocked),'blocked slot identity')
  need(s['not_attempted_after_capacity_abort'] is (blocked is not None and i>blocked),'unattempted suffix retained')
  need(s['final_offline_pose_available'] is (v['poses'][i] is not None),'pose availability separate')
 for f in ['online_output_latency_available','physical_frame_or_clock_verified','reference_scoring']:need(cap[f] is False,'no extra claim '+f)
 reads=read(directory/'input-reads.json')['reads'];phases={'tracking':[],'filler':[]};expected={}
 cal=read(ROOT/case['calibration']);K=np.asarray(cal['K_'+m+'L']['values'])
 h0,w0=case['source_shape'];scale=min(1.,np.sqrt(196608/(h0*w0)));h1,w1=int(h0*scale),int(w0*scale);hf,wf=h1-h1%8,w1-w1%8
 intrinsics=np.asarray([K[0,0]*w1/w0,K[1,1]*h1/h0,K[0,2]*w1/w0,K[1,2]*h1/h0],dtype=np.float32).tolist()
 need(case['shape']==[hf,wf] and np.allclose(case['intrinsics'],intrinsics,atol=5e-5,rtol=0),'geometry proof')
 need(case['baseline_m']==-float(cal['T_'+m+'R']['values'][0][0])*.001,'metric baseline')
 for r in reads:
  i=r['source_index'];phases[r['phase']].append(i)
  if i not in expected:
   pair=[]
   for side in ['left','right']:
    path=ROOT/case['canonical']/side/(rows[i]['stem']+'.png')
    need(sha(path)==p['source_sha256'][str(path.relative_to(ROOT))],'actual canonical input bytes')
    image=cv2.imread(str(path),cv2.IMREAD_UNCHANGED)
    if image.ndim==2:image=np.tile(image[...,None],(1,1,3))
    if [h0,w0]!=[h1,w1]:image=cv2.resize(image,(w1,h1),interpolation=cv2.INTER_LINEAR)
    pair.append(image[:hf,:wf])
   tensor=np.stack(pair).transpose(0,3,1,2).astype(np.float32)
   expected[i]=hashlib.sha256(tensor.tobytes()).hexdigest()
  need(r['native_time']==float(stamps[i]) and r['image_shape']==[2,3,hf,wf],'actual tensor geometry/time')
  need(r['intrinsics']==intrinsics and r['image_tensor_sha256']==expected[i],'independent actual input tensor')
 for phase,idx in phases.items():need(idx==list(range(len(idx))),'no missing/reordered input '+phase)
 need(len(phases['tracking']) in [len(calls),min(n,len(calls)+1)],'input/returned-call accounting')
 if v['native_execution_completed']:need(len(calls)==n and all(len(x)==n for x in phases.values()),'complete offline trajectory')
 comparison=None
 if compare:
  prior=ROOT/'results/droid-longer-prefix/run-001/captures'/case['case']
  old_calls=[json.loads(l) for l in (prior/'tracking.jsonl').read_text().splitlines()]
  fields=['source_index','source_stem','time_ns','right_time_ns','native_time','native_keyframe_count','frontend_initialized']
  need(len(old_calls)==len(calls)==101,'prefix returned population')
  differences=[{'index':i,'field':k} for i,(a,b) in enumerate(zip(calls,old_calls)) for k in fields if a[k]!=b[k]]
  baseline_repeat=None
  if differences:
   reference=OUT/'reference-repeat'/case['case']
   need((reference/'estimate.json').exists(),'unchanged baseline repeat required for any divergent prefix')
   repeated_calls=[json.loads(l) for l in (reference/'tracking.jsonl').read_text().splitlines()]
   need(len(repeated_calls)==101,'unchanged baseline repeat complete')
   repeated_input=read(reference/'input-reads.json')['reads']
   need(repeated_input==old_reads_for_repeat(prior),'unchanged baseline repeat actual inputs exact')
   repeated_diffs=[{'index':i,'field':k} for i,(a,b) in enumerate(zip(repeated_calls,old_calls)) for k in ['native_keyframe_count','frontend_initialized'] if a[k]!=b[k]]
   need(bool(repeated_diffs),'cannot attribute mismatch to unverified baseline variability')
   baseline_repeat={'tracking_divergences':len(repeated_diffs),'details':repeated_diffs,'estimate_sha256':sha(reference/'estimate.json')}
  need(all(a[k]==b[k] for a,b in zip(calls,old_calls) for k in ['source_index','source_stem','time_ns','right_time_ns','native_time']),'all prefix source/time fields exact')
  old_reads=read(prior/'input-reads.json')['reads']
  need(len(old_reads)==len(reads),'prefix input count')
  for a,b in zip(reads,old_reads):need(a==b,'prefix actual input metadata/tensor exact')
  old=read(prior/'estimate.json');maxdiff=float(np.max(np.abs(np.asarray(old['poses'])-np.asarray(v['poses']))))
  comparison={'tracking_categorical_divergences':len(differences),'tracking_divergence_details':differences,'unchanged_reference_repeat':baseline_repeat,'all_input_reads_exact':True,'max_offline_pose_matrix_difference':maxdiff}
 report={'case':case['case'],'source_frames':n,'returned_track_calls':len(calls),'capacity_events':events,
 'finite_offline_poses':v['finite_admitted_pose_frames'],'terminal_success':v['terminal_success'],
 'last_observed_native_keyframe_count':cap['last_observed_keyframe_count'],'native_keyframes':v['native_keyframe_count'],'peak_cuda_memory_allocated':v['peak_cuda_memory_allocated'],
 'wall_runtime_ns':v['wall_runtime_ns'],'failure':v['failure'],'comparison':comparison,
 'estimate_sha256':sha(directory/'estimate.json')}
 return report


def old_reads_for_repeat(prior):return read(prior/'input-reads.json')['reads']


def main(mode,output):
 cv2.setNumThreads(1);reports=[]
 if mode=='prefix':
  paths=[ROOT/'experiments/droid-bounded-prefix-2026-11-04-v2.json',ROOT/'experiments/droid-bounded-overflow-control-2026-11-04-v2.json']
 else:paths=[ROOT/r['protocol'] for r in read(ROOT/'experiments/droid-bounded-feature-layout-retries-2026-11-04.json')['runs']]
 for path in paths:
  p=read(path)
  for rel,digest in p['source_sha256'].items():need(sha(ROOT/rel)==digest,'frozen native input '+rel)
  control=p['bounded_capacity']['mode'] not in NORMAL_MODES
  for case in p['runs']:
   name='overflow-control' if control else case['case']
   directory=OUT/('prefix-v2' if mode=='prefix' else 'captures-v4')/name
   report=run_case(p,case,directory,path,compare=mode=='prefix' and not control);report['capture_name']=name;reports.append(report)
   print('AUDITED',mode,name,flush=True)
 mutations=0
 # Independent metric checker negative controls, using each output's actual native arrays.
 for report in reports:
  case_id=report['case'];control=report['capture_name']=='overflow-control'
  path=next(path for path in paths if any(c['case']==case_id for c in read(path)['runs']) and
    (read(path)['bounded_capacity']['mode'] not in NORMAL_MODES) is control)
  p=read(path);case=next(c for c in p['runs'] if c['case']==case_id)
  directory=OUT/('prefix-v2' if mode=='prefix' else 'captures-v4')/report['capture_name']
  v=read(directory/'estimate.json');src=read(ROOT/case['manifest']);vectors=matrices=None
  if (directory/'native-trajectory.npz').exists():
   with np.load(directory/'native-trajectory.npz',allow_pickle=False) as a:vectors=a['xyz_xyzw'].copy();matrices=a['world_from_camera_native'].copy()
  for key,new in [('metric_translation_factor',1.10),('online_output_latency_available',True),('scale_fitted',True)]:
   corrupt=deepcopy(v);corrupt[key]=new
   try:check_output(corrupt,src,case,vectors,matrices)
   except ValueError:mutations+=1
   else:raise ValueError('undetected metric mutation '+key)
 result={'passed':True,'auditor_version':4,'auditor_sha256':sha(Path(__file__)),'mode':mode,'independent_checks':CHECKS,'metric_mutations_rejected':mutations,'runs':reports,
 'reference_scoring':False,'physical_sync_verified':False,'benchmark_execution_ready':False}
 with Path(output).open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
 print(json.dumps(result,indent=2))


if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['prefix','full']);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();main(a.mode,a.output)
