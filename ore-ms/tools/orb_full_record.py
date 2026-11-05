"""EXP143 complete-record native ORB execution and matched map-segment scoring."""
import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import resource
import shutil
import time
import traceback

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/orb-full-record/run-001'
PLAN=ROOT/'experiments/orb-full-record-2026-11-05.json'
RUNTIME='sha256:1542a9a2fe4f645aa7b3db1d4d002c0a0b5941a91db105c5587db62b68bd8445'
THREAD_ENV={k:'1' for k in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']}

def read(p):return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def write(p,value):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
    with p.open('x') as f:json.dump(value,f,indent=2,allow_nan=False);f.write('\n')

def evaluation_view(plan,legacy):
    """Keep received poses unchanged; source-time gaps are not map resets."""
    n=len(plan['time_ns']);rows=legacy['rows']
    if any(len(legacy[k])!=n for k in ['time_ns','poses','segments','rows']):raise ValueError('parallel source population')
    if legacy['time_ns']!=plan['time_ns'] or any(type(t) is not int for t in legacy['time_ns']):raise ValueError('exact source clocks')
    segments=[];sid=-1;previous_valid=False;previous_map=None;missing_splits=map_changes=0
    seen_valid=False
    for i,(pose,row) in enumerate(zip(legacy['poses'],rows)):
        if row is not None:
            if type(row['index']) is not int or row['index']!=i or type(row['time_ns']) is not int or type(row['right_time_ns']) is not int or row['time_ns']!=plan['time_ns'][i] or row['right_time_ns']!=plan['right_time_ns'][i]:
                raise ValueError('native source binding')
        if pose is None:
            if row is not None and row['T_camera_from_world'] is not None:raise ValueError('legacy omitted available native pose')
            segments.append(None);previous_valid=False;continue
        if row is None or row['T_camera_from_world'] is None or row['state']!=2 or type(row['map_id']) is not int or row['map_id']<0:
            raise ValueError('pose without tracking/map evidence')
        if not previous_valid or row['map_id']!=previous_map:
            sid+=1
            if seen_valid:
                if not previous_valid:missing_splits+=1
                if previous_map is not None and row['map_id']!=previous_map:map_changes+=1
        segments.append(sid);previous_valid=True;previous_map=row['map_id'];seen_valid=True
    result={k:deepcopy(legacy[k]) for k in ['time_ns','poses','terminal_success']}
    result.update(segments=segments,source_ids=deepcopy(plan['stems']),
                  coordinate_frame=legacy['coordinate_frame'],source_frames=n,
                  reference_used=False,scale_fitted=False,physical_frame_or_clock_verified=False,
                  scope='camera-local received ORB poses; map-continuity segments; conditional provided references')
    boundaries=dict(legacy_segment_count=legacy['segment_count'],evaluation_segment_count=sid+1,
                    observed_map_identity_changes=map_changes,recovery_after_missing_splits=missing_splits,
                    recorded_time_gaps=sum(b-a>150000000 for a,b in zip(plan['time_ns'],plan['time_ns'][1:])))
    return result,boundaries

def failure_view(plan,error):
    n=len(plan['time_ns'])
    return dict(time_ns=deepcopy(plan['time_ns']),source_ids=deepcopy(plan['stems']),poses=[None]*n,
                segments=[None]*n,terminal_success=False,source_frames=n,reference_used=False,
                scale_fitted=False,physical_frame_or_clock_verified=False,failure=error,
                scope='fail-closed publication failure; original native receipts retained')

def freeze():
    import subprocess
    from orb_ms2_development import validate_build_flags
    old=read(ROOT/'experiments/orb-ms2-development-2026-11-04-v4.json')
    intake=read(ROOT/'experiments/full-record-intake-2026-11-04.json')
    reference=ROOT/'results/provided-reference-full-record/run-001/reference-lock.json'
    files=[Path(__file__),ROOT/'tools/orb_full_record_audit.py',ROOT/'tools/tests/test_orb_full_record.py',
           ROOT/'docs/research/orb-full-record-protocol-2026-11-05.md',ROOT/'tools/orb_ms2_development.py',
           ROOT/'tools/independent_benchmark_metrics_v2.py',ROOT/'tools/provided_reference_full_record_audit.py',
           ROOT/'experiments/orb-ms2-development-2026-11-04-v4.json',reference,
           ROOT/'results/provided-reference-full-record/run-001/summary.json']
    native_paths=['/opt/ore-ms-adapter/build/orb_ms2_runner','/opt/ORB_SLAM3/Vocabulary/ORBvoc.txt','/opt/ORB_SLAM3/lib/libORB_SLAM3.so']
    flag_paths={k:f'/opt/{p}' for k,p in dict(orb='ORB_SLAM3/build/CMakeFiles/ORB_SLAM3.dir/flags.make',g2o='ORB_SLAM3/Thirdparty/g2o/build/CMakeFiles/g2o.dir/flags.make',adapter='ore-ms-adapter/build/CMakeFiles/orb_ms2_runner.dir/flags.make').items()}
    flags={k:Path(v).read_text() for k,v in flag_paths.items()};validate_build_flags(flags)
    revision=subprocess.check_output(['git','-C','/opt/ORB_SLAM3','rev-parse','HEAD'],text=True).strip()
    if revision!='4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4':raise ValueError('native revision')
    runs=[]
    for condition in ['night','day','rain']:
        source=next(c for c in intake['cases'] if c['condition']==condition)
        for mode in ['rgb','thermal']:
            case=condition+'-'+mode;canonical=ROOT/'results/bounded-full-record/canonical'/case
            directory=OUT/'captures'/case;directory.mkdir(parents=True,exist_ok=False)
            for name in ['plan.json','settings.yaml','inputs.tsv']:
                shutil.copy2(canonical/name,directory/name);files.extend([canonical/name,directory/name])
            plan=read(directory/'plan.json')
            if len(plan['stems'])!=source['source_groups'] or plan['seed']!=0:raise ValueError('complete original source axis/seed')
            files.extend([ROOT/'results/bounded-full-record/manifests'/(condition+'.json'),ROOT/source['calibration']])
            runs.append(dict(case=case,condition=condition,modality='rgb' if mode=='rgb' else 'thr',
                             sequence=source['sequence'],source_frames=source['source_groups'],
                             manifest='results/bounded-full-record/manifests/'+condition+'.json',
                             calibration=source['calibration'],directory=str(directory.relative_to(ROOT))))
    p=dict(date='2026-11-05 America/New_York',parent_commit='bf0e1493e',runtime_image=RUNTIME,
           runs=runs,source_sha256={str(f.relative_to(ROOT)):sha(f) for f in files},
           native_sha256={f:sha(f) for f in native_paths},native_revision=revision,compile_flags=flags,
           native_patch_sha256=hashlib.sha256(subprocess.check_output(['git','-C','/opt/ORB_SLAM3','diff'])).hexdigest(),
           timeout_seconds=3600,thread_environment=THREAD_ENV,seed=0,sequential=True,attempts_per_case=1,
           reference_lock_sha256=sha(reference),references_previously_development_exposed=True,
           segment_policy='map continuity; split on missing output or native map identity change, never timestamp-only gap',
           physical_sync_verified=False,benchmark_execution_ready=False,reserve_access=False,new_native_inference=True,
           quality_driven_retries=False,original_adapter_sha256=old['source_sha256']['tools/orb_ms2_development.py'])
    if sha(ROOT/'tools/orb_ms2_development.py')!=p['original_adapter_sha256']:raise ValueError('original adapter changed')
    write(PLAN,p);write(OUT/'study-lock.json',p);print('FROZEN all six complete-record runs',flush=True)

def verify(native=False):
    p=read(PLAN)
    if sha(PLAN)!=sha(OUT/'study-lock.json'):raise ValueError('study lock')
    for n,h in p['source_sha256'].items():
        if sha(ROOT/n)!=h:raise ValueError('source/config changed '+n)
    for flag in ['physical_sync_verified','benchmark_execution_ready','reserve_access','quality_driven_retries']:
        if p[flag] is not False:raise ValueError('scope '+flag)
    if native:
        for n,h in p['native_sha256'].items():
            if sha(n)!=h:raise ValueError('native binary changed '+n)
        if any(os.environ.get(k)!=v for k,v in p['thread_environment'].items()):raise ValueError('thread environment')
    return p

def run(case):
    from orb_ms2_development import run as original_run
    p=verify(native=True);r=next(r for r in p['runs'] if r['case']==case);directory=ROOT/r['directory']
    if (directory/'attempt-lock.json').exists():raise ValueError('one declared native attempt only')
    write(directory/'attempt-lock.json',dict(plan_sha256=sha(PLAN),case=case,start_utc_ns=time.time_ns(),runtime=p['runtime_image'],thread_environment=THREAD_ENV))
    start=time.monotonic_ns();error=None;legacy=None
    try:
        legacy=original_run(directory,timeout=p['timeout_seconds'])
        view,boundaries=evaluation_view(read(directory/'plan.json'),legacy)
    except Exception:
        error=traceback.format_exc();view=failure_view(read(directory/'plan.json'),error);boundaries=None
    wall=time.monotonic_ns()-start
    rows=legacy['rows'] if legacy else []
    times=[r['processing_ns'] for r in rows if r is not None]
    write(directory/'evaluation-estimate.json',view)
    write(directory/'run-status.json',dict(case=case,wrapper_wall_ns_including_input_hash_verification=wall,
          native_adapter_wall_ns_including_startup_shutdown=legacy.get('wall_runtime_ns_including_startup_shutdown') if legacy else None,
          peak_child_maxrss_kib=resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss,
          returned_source_calls=len(times),native_processing_sum_ns=sum(times),
          native_processing_max_ns=max(times) if times else None,
          native_returncode=legacy.get('native_returncode') if legacy else None,
          native_calibration_verified=legacy.get('native_calibration_verified') if legacy else False,
          terminal_success=view['terminal_success'],pose_frames=sum(x is not None for x in view['poses']),
          source_frames=r['source_frames'],segment_boundaries=boundaries,publication_error=error,
          evaluation_estimate_sha256=sha(directory/'evaluation-estimate.json'),plan_sha256=sha(PLAN)))
    print('COMPLETED',case,'terminal',view['terminal_success'],'poses',sum(x is not None for x in view['poses']), '/',r['source_frames'],flush=True)

def lock_predictions():
    p=verify();pins={}
    for r in p['runs']:
        directory=ROOT/r['directory']
        for name in ['attempt-lock.json','evaluation-estimate.json','run-status.json','execution-lock.json','native.jsonl','estimate.json']:
            if (directory/name).exists():pins[str((directory/name).relative_to(ROOT))]=sha(directory/name)
        if not (directory/'run-status.json').exists():raise ValueError('all six attempts required before scoring')
    write(OUT/'prediction-lock.json',dict(plan_sha256=sha(PLAN),source_sha256=pins,all_six_attempts_retained=True,reference_score_computed=False))

def score_all():
    import numpy as np
    from independent_benchmark_metrics_v2 import score
    p=verify();lock=read(OUT/'prediction-lock.json')
    if lock['plan_sha256']!=sha(PLAN):raise ValueError('prediction parent')
    for n,h in lock['source_sha256'].items():
        if sha(ROOT/n)!=h:raise ValueError('prediction/native outputs changed')
    refs=read(ROOT/'results/provided-reference-full-record/run-001/reference-lock.json')
    for n,h in refs['member_sha256'].items():
        if sha(ROOT/n)!=h:raise ValueError('reference changed')
    results=[]
    for r in p['runs']:
        directory=ROOT/r['directory'];e=read(directory/'evaluation-estimate.json');source=read(ROOT/r['manifest'])
        poses=[np.loadtxt(ROOT/'results/provided-reference-full-record/run-001/reference'/r['sequence']/r['modality']/(row['stem']+'.txt')).reshape(4,4).tolist() for row in source['records']]
        result=score(dict(time_ns=e['time_ns'],poses=poses),e);write(OUT/'scores'/(r['case']+'.json'),result)
        results.append(dict(case=r['case'],condition=r['condition'],modality=r['modality'],
                            **{k:v for k,v in result.items() if k not in ['rpe_rows','reference_exclusions','segment_ate']},
                            run_status=read(directory/'run-status.json')))
        print('SCORED',r['case'],'ATE',result['ate_se3_rmse_m'],'coverage',result['pose_coverage'],'segments',result['segment_count'],flush=True)
    write(OUT/'summary.json',dict(runs=results,droid_context=read(ROOT/'results/provided-reference-full-record/run-001/summary.json'),
          physical_sync_verified=False,benchmark_execution_ready=False,adaptive_advantage_tested=False,
          scale_fitted=False,hardware_matched=False,output_latency_matched=False))

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('action',choices=['freeze','verify','run','lock','score']);a.add_argument('--case');args=a.parse_args()
    {'freeze':freeze,'verify':verify,'run':lambda:run(args.case),'lock':lock_predictions,'score':score_all}[args.action]()
