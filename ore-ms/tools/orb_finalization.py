"""EXP144 same-prefix received/final ORB and offline DROID control."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import subprocess
import time
import traceback

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/orb-finalization/run-001'
PLAN=ROOT/'experiments/orb-finalization-2026-11-05.json'
BASE='sha256:1542a9a2fe4f645aa7b3db1d4d002c0a0b5941a91db105c5587db62b68bd8445'
DROID='sha256:d3cf8a41dd8090c8939cd840b4735d526cec9a54e48ac36d506f91d13ee56e1c'
N=2000
def read(p):return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def write(p,v):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')

def segments(valid,received_segments):
    result=[];sid=-1;active=False;previous=None
    for ok,original in zip(valid,received_segments):
        if not ok:result.append(None);active=False;continue
        if type(original) is not int:raise ValueError('valid output without received segment')
        if not active or original!=previous:sid+=1
        result.append(sid);active=True;previous=original
    return result

def final_view(plan,received,rows):
    import numpy as np
    n=len(plan['stems'])
    if len(rows)!=n:raise ValueError('all final source slots required')
    poses=[];reasons=[]
    for i,row in enumerate(rows):
        if (type(row['index']) is not int or row['index']!=i or row['source_id']!=plan['stems'][i]
            or type(row['time_ns']) is not int or row['time_ns']!=plan['time_ns'][i]
            or row['right_time_ns']!=plan['right_time_ns'][i]):raise ValueError('final source/clock binding')
        pose=row['T_world_from_camera'];reason=row.get('reason')
        if pose is not None:
            if received['poses'][i] is None or row['received_valid'] is not True:raise ValueError('received missing output resurrected')
            native=received['rows'][i]
            if row['candidate_count']!=1 or row['native_lost'] is not False or row['final_map_id']!=native['map_id']:
                raise ValueError('ambiguous/lost/wrong-map final pose')
            a=np.asarray(pose,dtype=float)
            if a.shape!=(4,4) or not np.isfinite(a).all() or not np.allclose(a[3],[0,0,0,1],atol=1e-10,rtol=0):raise ValueError('final homogeneous pose')
            a[3]=[0,0,0,1]
            if np.linalg.norm(a[:3,:3].T@a[:3,:3]-np.eye(3))>1e-5 or abs(np.linalg.det(a[:3,:3])-1)>1e-5:
                pose=None;reason='nonrigid_final_snapshot'
            else:pose=a.tolist()
        poses.append(pose);reasons.append(reason)
    result={k:deepcopy(received[k]) for k in ['time_ns','source_ids','terminal_success']}
    result.update(poses=poses,segments=segments([x is not None for x in poses],received['segments']),
                  rejection_reasons=reasons,reference_used=False,scale_fitted=False,
                  scope='post-worker native factor snapshot; no missing output filling or map stitching')
    return result

def common_views(received,final,droid):
    if not (received['time_ns']==final['time_ns']==droid['time_ns'] and received['source_ids']==final['source_ids']==droid['source_ids']):
        raise ValueError('exact same prefix/source clocks required')
    mask=[all(e['poses'][i] is not None for e in [received,final,droid]) for i in range(len(received['poses']))]
    shared=segments(mask,received['segments']);views=[]
    for e in [received,final,droid]:
        views.append(dict(time_ns=deepcopy(e['time_ns']),source_ids=deepcopy(e['source_ids']),
                          poses=[deepcopy(x) if ok else None for x,ok in zip(e['poses'],mask)],
                          segments=deepcopy(shared),terminal_success=e['terminal_success']))
    return views,mask

def prepare(count=N,namespace=OUT):
    source=read(ROOT/'results/bounded-full-record/manifests/night.json');source['records']=source['records'][:count]
    source.update(scope='fixed first '+str(count)+' original source groups; not the complete recording',source_frames=count)
    write(namespace/'source-manifest.json',source)
    for mode in ['rgb','thermal']:
        old=ROOT/'results/bounded-full-record/canonical'/('night-'+mode);full=read(old/'plan.json')
        for arm in ['control','repeat','paired']:
            dest=namespace/'captures'/(mode+'-'+arm);dest.mkdir(parents=True,exist_ok=False)
            (dest/'settings.yaml').write_bytes((old/'settings.yaml').read_bytes())
            (dest/'inputs.tsv').write_text(''.join((old/'inputs.tsv').read_text().splitlines(keepends=True)[:count]))
            p=deepcopy(full)
            for name in ['time_ns','right_time_ns','stems']:p[name]=p[name][:count]
            p['prepared_images']=p['prepared_images'][:count*2]
            files={str(namespace/'source-manifest.json'):sha(namespace/'source-manifest.json'),
                   str(ROOT/'tools/orb_ms2_development.py'):sha(ROOT/'tools/orb_ms2_development.py'),
                   str(ROOT/'MAC-VO-main/DataLoader/Dataset/MS2.py'):sha(ROOT/'MAC-VO-main/DataLoader/Dataset/MS2.py')}
            for row in source['records']:
                for m in ['rgb','thr']:
                    for side in ['left','right']:
                        rec=row['cameras'][m][side];files[str(ROOT/rec['path'])]=rec['sha256']
            for item in p['prepared_images']:files[item['path']]=item['sha256']
            for name in ['settings.yaml','inputs.tsv']:files[str(dest/name)]=sha(dest/name)
            p['frozen_files']=files;p['source_manifest_sha256']=sha(namespace/'source-manifest.json');p['scope']='fixed first '+str(count)+' original night source groups'
            write(dest/'plan.json',p)
    if count!=N:return
    for mode in ['rgb','thermal']:
        p=read(ROOT/'results/bounded-full-record/protocols'/('night-'+mode+'.json'))
        c=p['runs'][0];c['manifest']=str((namespace/'source-manifest.json').relative_to(ROOT));c['source_frames']=count
        paths=[ROOT/'tools/droid_ms2_development.py',ROOT/'tools/droid_bounded_worker.py',ROOT/p['weights'],namespace/'source-manifest.json',ROOT/c['calibration']]
        for side in ['left','right']:paths += [ROOT/c['canonical']/side/(f'{i:06}.png') for i in range(count)]
        p['source_frames']=count
        p['source_sha256']={str(f.relative_to(ROOT)):sha(f) for f in paths};p['scope']='same-horizon '+str(count)+' source offline DROID control'
        write(namespace/'protocols'/('droid-'+mode+'.json'),p)

def freeze(runtime):
    files=[Path(__file__),ROOT/'tools/orb_finalization_audit.py',ROOT/'tools/tests/test_orb_finalization.py',
           ROOT/'docs/research/orb-finalization-protocol-2026-11-05.md',ROOT/'tools/orb_ms2_development.py',
           ROOT/'tools/orb_full_record.py',ROOT/'tools/independent_benchmark_metrics_v2.py',
           ROOT/'tools/provided_reference_full_record_audit.py',ROOT/'tools/droid_ms2_development.py',ROOT/'tools/droid_bounded_worker.py',OUT/'runtime-identity.json']
    files+=list((ROOT/'environment').glob('*orb_final*'));files.extend([ROOT/'environment/add_orb_export_accessors.py',ROOT/'environment/Dockerfile.orb-finalized'])
    files+=list(OUT.glob('source-manifest.json'))+list((OUT/'captures').glob('*/*'))+list((OUT/'protocols').glob('*.json'))
    p=dict(date='2026-11-05 America/New_York',parent_commit='33bc82f81',source_groups=N,modalities=['rgb','thermal'],
           source_sha256={str(f.relative_to(ROOT)):sha(f) for f in files if f.is_file()},base_runtime=BASE,paired_runtime=runtime,droid_runtime=DROID,
           timeout_seconds=3600,worker_finish_timeout_seconds=60,attempts_per_arm=1,
           arms=['control','repeat','paired','droid'],seed=0,thread_environment={k:'1' for k in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']},
           references_previously_exposed=True,source_selection='first 2000 original night source groups, engineering prefix, not new quality outcomes',
           reference_lock_sha256=sha(ROOT/'results/provided-reference-full-record/run-001/reference-lock.json'),
           native_identity=read(OUT/'runtime-identity.json'),physical_sync_verified=False,benchmark_execution_ready=False,
           scale_fit=False,time_fit=False,reserve_access=False)
    write(PLAN,p);write(OUT/'study-lock.json',p);print('FROZEN eight native arms on the same 2000-group prefix')

def verify():
    p=read(PLAN)
    if sha(PLAN)!=sha(OUT/'study-lock.json'):raise ValueError('study lock')
    for n,h in p['source_sha256'].items():
        if sha(ROOT/n)!=h:raise ValueError('source/config changed '+n)
    return p

def run_orb(mode,arm,namespace=OUT):
    from orb_ms2_development import run as old_run,publish
    from orb_full_record import evaluation_view
    directory=namespace/'captures'/(mode+'-'+arm);p=read(directory/'plan.json')
    if (directory/'attempt-lock.json').exists():raise ValueError('one declared attempt')
    write(directory/'attempt-lock.json',dict(mode=mode,arm=arm,start_utc_ns=time.time_ns()))
    error=None;start=time.monotonic_ns()
    if arm!='paired':legacy=old_run(directory,timeout=3600)
    else:
        for n,h in p['frozen_files'].items():
            if sha(n)!=h:raise ValueError('prepared/source changed')
        runner='/opt/ore-ms-final-adapter/build/orb_finalized_runner'
        cmd=[runner,'/opt/ORB_SLAM3/Vocabulary/ORBvoc.txt',str(directory/'settings.yaml'),str(directory/'inputs.tsv'),str(directory/'native.jsonl'),'0','1']
        write(directory/'execution-lock.json',dict(command=cmd,runner_sha256=sha(runner),library_sha256=sha('/opt/ORB_SLAM3/lib/libORB_SLAM3.so'),plan_sha256=sha(directory/'plan.json')))
        with (directory/'native.log').open('x') as f:
            try:result=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,timeout=3600);code=result.returncode
            except subprocess.TimeoutExpired:code=124
        native=[json.loads(s) for s in (directory/'native.jsonl').read_text().splitlines()]
        import numpy as np
        legacy=publish(p,native,code)
        configurations=[row['configuration'] for row in native if 'configuration' in row]
        legacy['native_calibration_verified']=(len(configurations)==1 and configurations[0]['rerectifies'] is False
             and configurations[0]['resizes'] is False and all(np.isclose(configurations[0][key],value,rtol=5e-7,atol=1e-7) for key,value in p['calibration'].items()))
        if not legacy['native_calibration_verified']:legacy['terminal_success']=False
        write(directory/'estimate.json',legacy)
    received,boundaries=evaluation_view(p,legacy);write(directory/'received.json',received)
    if arm=='paired':
        rows=[json.loads(s) for s in (directory/'native.jsonl.final.jsonl').read_text().splitlines()]
        final=final_view(p,{**received,'rows':legacy['rows']},rows);write(directory/'final.json',final)
    write(directory/'run-status.json',dict(mode=mode,arm=arm,wall_ns=time.monotonic_ns()-start,source_frames=len(p['stems']),received_poses=sum(v is not None for v in received['poses']),terminal_success=received['terminal_success'],boundaries=boundaries,error=error))
    print('FINISHED',mode,arm,'poses',sum(v is not None for v in received['poses']),flush=True)

def lock():
    p=verify();pins={}
    for mode in ['rgb','thermal']:
        for arm in p['arms']:
            d=OUT/'captures'/(mode+'-'+arm)
            if arm=='droid':needed=['estimate.json','execution-lock.json','capacity-status.json','tracking.jsonl','input-reads.json','native-trajectory.npz']
            else:needed=['estimate.json','execution-lock.json','received.json','native.jsonl','run-status.json']
            if arm=='paired':needed+=['final.json','native.jsonl.final.jsonl','native.jsonl.maps.json']
            for name in needed:
                if not (d/name).exists():raise ValueError('missing arm output '+str(d/name))
                pins[str((d/name).relative_to(ROOT))]=sha(d/name)
    write(OUT/'prediction-lock.json',dict(study_sha256=sha(PLAN),source_sha256=pins,all_eight_arms_retained=True,new_reference_scores_read=False))

def score():
    import numpy as np
    from independent_benchmark_metrics_v2 import score as metric
    p=verify();locked=read(OUT/'prediction-lock.json')
    for n,h in locked['source_sha256'].items():
        if sha(ROOT/n)!=h:raise ValueError('prediction changed')
    refs=read(ROOT/'results/provided-reference-full-record/run-001/reference-lock.json')
    if sha(ROOT/'results/provided-reference-full-record/run-001/reference-lock.json')!=p['reference_lock_sha256']:raise ValueError('reference lock')
    source=read(OUT/'source-manifest.json');summary=[]
    for mode in ['rgb','thermal']:
        directory=OUT/'captures'/(mode+'-paired');received=read(directory/'received.json');final=read(directory/'final.json');droid=read(OUT/'captures'/(mode+'-droid')/'estimate.json')
        views,mask=common_views(received,final,droid);m='rgb' if mode=='rgb' else 'thr';raw=[]
        for row in source['records']:
            path=ROOT/'results/provided-reference-full-record/run-001/reference'/source['sequence']/m/(row['stem']+'.txt')
            if sha(path)!=refs['member_sha256'][str(path.relative_to(ROOT))]:raise ValueError('reference member')
            raw.append(np.loadtxt(path).reshape(4,4).tolist())
        results={}
        for arm,view in zip(['received','final','droid'],views):
            dst=OUT/'scores'/mode;write(dst/(arm+'-view.json'),view);result=metric(dict(time_ns=view['time_ns'],poses=raw),view);write(dst/(arm+'.json'),result)
            results[arm]={k:v for k,v in result.items() if k not in ['rpe_rows','segment_ate','reference_exclusions']}
        summary.append(dict(modality=mode,source_frames=N,individual_pose_frames={a:sum(v is not None for v in e['poses']) for a,e in [('received',received),('final',final),('droid',droid)]},common_pose_frames=sum(mask),results=results))
    write(OUT/'summary.json',dict(runs=summary,common_future_horizon=N,physical_sync_verified=False,benchmark_execution_ready=False,scale_fitted=False,output_stage_effect_only=True))
    print(json.dumps(summary,indent=2))

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('action',choices=['prepare','freeze','verify','run-orb','lock','score']);a.add_argument('--runtime');a.add_argument('--mode');a.add_argument('--arm');a.add_argument('--smoke',action='store_true');x=a.parse_args()
    ns=ROOT/'results/orb-finalization/integration-020' if x.smoke else OUT
    {'prepare':lambda:prepare(20 if x.smoke else N,ns),'freeze':lambda:freeze(x.runtime),'verify':verify,'run-orb':lambda:run_orb(x.mode,x.arm,ns),'lock':lock,'score':score}[x.action]()
