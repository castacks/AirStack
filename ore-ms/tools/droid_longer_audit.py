"""Independent long-prefix and EXP136-prefix comparison; no native worker helpers."""
import argparse
from copy import deepcopy
from pathlib import Path
import hashlib
import json

import cv2
import numpy as np

from droid_ms2_audit import check_output

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/droid-longer-prefix/run-001'
PLAN=ROOT/'experiments/droid-longer-prefix-2026-11-04.json'


def read(path):return json.loads(Path(path).read_text())
def sha(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def main(output):
    plan=read(PLAN);assert sha(PLAN)==sha(OUT/'plan.json');checks=0;summary=[];mutations=[]
    for path,digest in plan['source_sha256'].items():assert sha(ROOT/path)==digest,path
    changed={k for k,v in plan['settings'].items() if v!=plan['prior_settings'][k]}
    assert changed=={'buffer'} and plan['settings']['buffer']==128;checks+=2
    prior_poses=0;long_poses=0
    for case in plan['runs']:
        source=read(ROOT/case['manifest']);directory=OUT/'captures'/case['case']
        value=read(directory/'estimate.json');execution=read(directory/'execution-lock.json')
        assert execution['settings']['buffer']==128
        assert {k:v for k,v in execution['settings'].items() if k in plan['prior_settings'] and k!='buffer'}=={k:v for k,v in plan['prior_settings'].items() if k!='buffer'}
        assert [r['stem'] for r in source['records']]==[f'{i:06d}' for i in range(101)]
        arrays_path=directory/'native-trajectory.npz';vectors=matrices=None
        if arrays_path.exists():
            with np.load(arrays_path,allow_pickle=False) as arrays:
                vectors=arrays['xyz_xyzw'].copy();matrices=arrays['world_from_camera_native'].copy()
        check=lambda v:check_output(v,source,case,vectors,matrices)
        checks+=check(value)
        calls=[json.loads(line) for line in (directory/'tracking.jsonl').read_text().splitlines()]
        old_calls=[json.loads(line) for line in (ROOT/case['prior_tracking']).read_text().splitlines()]
        assert len(calls)==value['native_track_calls_returned']<=101
        m='rgb' if case['modality']=='rgb' else 'thr'
        original_times=[r['cameras'][m]['left']['recorded_time_ns'] for r in source['records']]
        native_times=np.asarray([(t-original_times[0])/100_000_000 for t in original_times],dtype=np.float32)
        for index,row in enumerate(calls):
            assert row['source_index']==index and row['source_stem']==f'{index:06d}'
            assert row['time_ns']==original_times[index] and type(row['time_ns']) is int
            assert row['right_time_ns']==source['records'][index]['cameras'][m]['right']['recorded_time_ns']
            assert row['native_time']==float(native_times[index]);checks+=4
        prefix_divergences=[]
        for index,(a,b) in enumerate(zip(calls[:20],old_calls)):
            for field in ['source_index','source_stem','time_ns','right_time_ns','native_time']:
                assert a[field]==b[field];checks+=1
            for field in ['native_keyframe_count','frontend_initialized']:
                if a[field]!=b[field]:prefix_divergences.append({'source_index':index,'field':field,'short':b[field],'long':a[field]})
        reads=read(directory/'input-reads.json')['reads'];old_reads=read(ROOT/case['prior_input_reads'])['reads']
        prior_tracking={r['source_index']:r for r in old_reads if r['phase']=='tracking'}
        phase_indices={'tracking':[],'filler':[]}
        for row in reads:
            i=row['source_index'];phase_indices[row['phase']].append(i)
            assert row['native_time']==float(native_times[i]);parts=[]
            h0,w0=case['source_shape'];scale=min(1.,np.sqrt(196608/(h0*w0)))
            h1,w1=int(h0*scale),int(w0*scale);final=[h1-h1%8,w1-w1%8]
            assert case['shape']==final
            for side in ['left','right']:
                path=ROOT/case['canonical']/side/f'{i:06d}.png'
                assert sha(path)==plan['source_sha256'][str(path.relative_to(ROOT))]
                image=cv2.imread(str(path),cv2.IMREAD_UNCHANGED)
                if image.ndim==2:image=np.repeat(image[...,None],3,axis=2)
                if [h1,w1]!=[h0,w0]:image=cv2.resize(image,(w1,h1),interpolation=cv2.INTER_LINEAR)
                parts.append(image[:final[0],:final[1]])
            tensor=np.stack(parts).transpose(0,3,1,2).astype(np.float32)
            assert row['image_shape']==list(tensor.shape)
            assert row['image_tensor_sha256']==hashlib.sha256(tensor.tobytes()).hexdigest()
            assert row['intrinsics']==np.asarray(case['intrinsics'],dtype=np.float32).tolist();checks+=6
            if row['phase']=='tracking' and i<20:
                for field in ['native_time','image_shape','image_tensor_sha256','intrinsics']:
                    assert row[field]==prior_tracking[i][field];checks+=1
        for indices in phase_indices.values():assert indices==list(range(len(indices)))
        if value['native_execution_completed']:assert all(len(indices)==101 for indices in phase_indices.values())
        initialization=next((r for r in calls if r['frontend_initialized']),None)
        prior=read(ROOT/case['prior_estimate']);prior_poses+=prior['finite_admitted_pose_frames'];long_poses+=value['finite_admitted_pose_frames']
        changes=[('wrong_scale',lambda v:v.update(metric_translation_factor=1.10)),
            ('float_clock',lambda v:v['time_ns'].__setitem__(0,float(v['time_ns'][0]))),
            ('online_latency_claim',lambda v:v.update(online_output_latency_available=True)),
            ('hidden_scale_fit',lambda v:v.update(scale_fitted=True))]
        if not value['frontend_initialized']:
            changes.append(('uninitialized_pose',lambda v:v['poses'].__setitem__(0,np.eye(4).tolist())))
        elif value['finite_admitted_pose_frames']:
            index=next(i for i,p in enumerate(value['poses']) if p is not None)
            changes.append(('changed_metric_pose',lambda v:v['poses'][index][0].__setitem__(3,v['poses'][index][0][3]+1)))
        for name,modify in changes:
            corrupted=deepcopy(value);modify(corrupted)
            try:check(corrupted)
            except ValueError:mutations.append(case['case']+'/'+name)
            else:raise ValueError('undetected mutation '+name)
        summary.append({'case':case['case'],'prior_initialized':prior['frontend_initialized'],
            'first_initialization_source_index':initialization['source_index'] if initialization else None,
            'first_initialization_time_ns':initialization['time_ns'] if initialization else None,
            'first_initialization_source_elapsed_ns':initialization['time_ns']-original_times[0] if initialization else None,
            'first20_frontend_divergences':prefix_divergences,
            **{k:value[k] for k in ['source_frames','native_track_calls_returned','native_execution_completed',
                'frontend_initialized','finite_admitted_pose_frames','terminal_success','native_keyframe_count',
                'wall_runtime_ns','peak_cuda_memory_allocated','failure']},'estimate_sha256':sha(directory/'estimate.json')})
    report={'pass':True,'independent_checks':checks,'semantic_mutations_rejected':len(mutations),'mutations':mutations,
            'runs':summary,'prior_20frame_admitted_poses':prior_poses,'long_prefix_admitted_poses':long_poses,
            'all_first20_inputs_exact':True,'categorical_prefix_divergences':sum(len(r['first20_frontend_divergences']) for r in summary),
            'reference_scoring':False,'protected_payload_access':False,'benchmark_execution_ready':False}
    with Path(output).open('x') as stream:json.dump(report,stream,indent=2);stream.write('\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);args=p.parse_args();main(args.output)
