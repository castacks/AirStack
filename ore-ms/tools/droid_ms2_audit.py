"""Independent input/pose/metric/time audit of the DROID stereo development runs."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
PLAN=ROOT/'experiments/droid-ms2-development-2026-11-04.json'
OUT=ROOT/'results/droid-ms2-development/run-001'


def read(path):return json.loads(Path(path).read_text())
def sha(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def quaternion_pose(vector):
    tx,ty,tz,x,y,z,w=map(float,vector)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),tx],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),ty],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y),tz],[0,0,0,1]])


def check_output(value,source,case,vectors,matrices):
    count=0
    def need(condition,label):
        nonlocal count
        if not condition:raise ValueError(label)
        count+=1
    modality='rgb' if case['modality']=='rgb' else 'thr'
    times=[row['cameras'][modality]['left']['recorded_time_ns'] for row in source['records']]
    right=[row['cameras'][modality]['right']['recorded_time_ns'] for row in source['records']]
    need(all(type(t) is int for t in value['time_ns']+value['right_time_ns']),'strict source integer clocks')
    need(value['time_ns']==times and value['right_time_ns']==right,'source clock identity')
    need(value['source_ids']==[r['stem'] for r in source['records']],'original source stems')
    need(value['source_frames']==len(times)==len(value['poses'])==len(value['segments']),'all source slots')
    need(value['metric_translation_factor']==case['baseline_m']/.1,'provided baseline metric factor')
    need(value['scale_fitted'] is False and value['reference_used'] is False,'no fitted scale/reference')
    need(value['online_output_latency_available'] is False,'no native offline/online conflation')
    need(value['native_tracking_failure_status_available'] is False,'no invented tracking status')
    need(value['native_filler_uses_interpolated_initialization'] is True,'filler interpolation explicit')
    valid=[];expected_segment=-1;active=False
    for index,pose in enumerate(value['poses']):
        vector=vectors[index] if vectors is not None else None
        finite=(vector is not None and np.isfinite(vector).all() and abs(np.linalg.norm(vector[3:])-1)<=1e-5)
        need(value['native_vector_valid'][index] is bool(finite),'native quaternion validity')
        proper=False
        if finite and matrices is not None:
            matrix=matrices[index]
            proper=(np.isfinite(matrix).all() and np.array_equal(matrix[3],[0,0,0,1])
                    and np.linalg.norm(matrix[:3,:3].T@matrix[:3,:3]-np.eye(3))<=1e-5
                    and abs(np.linalg.det(matrix[:3,:3])-1)<=1e-5)
        admitted=finite and proper and value['frontend_initialized'] is True
        if not admitted:
            need(pose is None and value['segments'][index] is None,'failed/uninitialized output stays null')
            active=False;valid.append(False);continue
        native=quaternion_pose(vector)
        need(np.allclose(native,matrices[index],atol=2e-5,rtol=0),'independent xyzw-to-matrix direction')
        native[:3,3]*=case['baseline_m']/.1
        need(np.allclose(pose,native,atol=2e-5,rtol=0),'independent metric output conversion')
        need(np.array_equal(np.asarray(pose)[3],[0,0,0,1]),'homogeneous row')
        if not active:expected_segment+=1
        need(type(value['segments'][index]) is int and value['segments'][index]==expected_segment,'segment continuity')
        active=True;valid.append(True)
    need(value['finite_admitted_pose_frames']==sum(valid),'finite pose count')
    need(value['terminal_success'] is (value['native_execution_completed'] is True and
          value['frontend_initialized'] is True and all(valid)),'terminal initialization/finite gate')
    return count


def main(output):
    p=read(PLAN);checks=0;mutations=[];summary=[]
    assert sha(PLAN)==sha(OUT/'plan.json')
    for path,digest in p['source_sha256'].items():assert sha(ROOT/path)==digest,path
    for case in p['runs']:
        source=read(ROOT/case['manifest']);calibration=read(ROOT/case['calibration'])
        m='rgb' if case['modality']=='rgb' else 'thr';k=np.asarray(calibration['K_'+m+'L']['values'])
        assert case['baseline_m']==-calibration['T_'+m+'R']['values'][0][0]/1000
        assert case['intrinsics']==[k[0,0],k[1,1],k[0,2],k[1,2]]
        directory=OUT/case['case'];value=read(directory/'estimate.json');execution=read(directory/'execution-lock.json')
        assert execution['runtime_image']==p['runtime_image'] and execution['settings']['stereo'] is True
        assert execution['settings']['image_size']==case['shape']
        assert execution['metric_translation_factor']==case['baseline_m']/.1
        vectors=matrices=None
        if (directory/'native-trajectory.npz').exists():
            with np.load(directory/'native-trajectory.npz',allow_pickle=False) as arrays:
                vectors=arrays['xyz_xyzw'].copy();matrices=arrays['world_from_camera_native'].copy()
        check=lambda x:check_output(x,source,case,vectors,matrices)
        checks+=check(value)
        expected_times=np.asarray([(t-value['time_ns'][0])/100_000_000 for t in value['time_ns']],dtype=np.float32)
        calls=[json.loads(line) for line in (directory/'tracking.jsonl').read_text().splitlines()]
        assert len(calls)==value['native_track_calls_returned']<=len(source['records'])
        for index,row in enumerate(calls):
            assert row['source_index']==index and row['source_stem']==source['records'][index]['stem']
            assert type(row['time_ns']) is int and row['time_ns']==value['time_ns'][index]
            assert type(row['right_time_ns']) is int and row['right_time_ns']==value['right_time_ns'][index]
            assert row['native_time']==float(expected_times[index])
            assert row['processing_ns']>=0 and row['receipt_elapsed_ns']>=row['processing_ns']
            checks+=5
        if calls:assert calls[-1]['frontend_initialized']==value['frontend_initialized']
        keyframes=value['native_keyframe_times']
        if keyframes is not None:
            assert all(t in expected_times for t in keyframes)
            assert all(a<b for a,b in zip(keyframes,keyframes[1:]));checks+=2
        reads=read(directory/'input-reads.json')['reads'];phase_indices={'tracking':[],'filler':[]}
        for row in reads:
            index=row['source_index'];phase_indices[row['phase']].append(index)
            assert row['native_time']==float(expected_times[index])
            parts=[]
            for side in ['left','right']:
                path=ROOT/case['canonical']/side/(source['records'][index]['stem']+'.png')
                assert sha(path)==p['source_sha256'][str(path.relative_to(ROOT))]
                raw=cv2.imread(str(path),cv2.IMREAD_UNCHANGED)
                if raw.ndim==2:raw=np.tile(raw[...,None],(1,1,3))
                parts.append(raw)
            expected=np.transpose(np.asarray(parts),(0,3,1,2)).astype(np.float32)
            assert row['image_shape']==list(expected.shape)
            assert row['image_tensor_sha256']==hashlib.sha256(expected.tobytes()).hexdigest()
            assert row['intrinsics']==np.asarray(case['intrinsics'],dtype=np.float32).tolist()
            checks+=6
        for phase,indices in phase_indices.items():assert indices==list(range(len(indices)))
        if value['native_execution_completed']:assert all(len(v)==20 for v in phase_indices.values())
        alterations=[('wrong_scale',lambda x:x.update(metric_translation_factor=1.10)),
            ('float_time',lambda x:x['time_ns'].__setitem__(0,float(x['time_ns'][0]))),
            ('invented_online_latency',lambda x:x.update(online_output_latency_available=True)),
            ('hidden_scale_fit',lambda x:x.update(scale_fitted=True))]
        if not value['frontend_initialized']:
            alterations.append(('uninitialized_filler_admission',lambda x:x['poses'].__setitem__(0,np.eye(4).tolist())))
        elif value['finite_admitted_pose_frames']:
            first=next(i for i,pose in enumerate(value['poses']) if pose is not None)
            alterations.append(('altered_metric_pose',lambda x:x['poses'][first][0].__setitem__(3,x['poses'][first][0][3]+1)))
        for label,modify in alterations:
            changed=deepcopy(value);modify(changed)
            try:check(changed)
            except ValueError:mutations.append(case['case']+'/'+label)
            else:raise ValueError('failed to reject '+label)
        summary.append({'case':case['case'],**{key:value[key] for key in ['source_frames','native_track_calls_returned',
            'native_execution_completed','frontend_initialized','finite_admitted_pose_frames','terminal_success',
            'native_keyframe_count','wall_runtime_ns','peak_cuda_memory_allocated','failure']},
            'estimate_sha256':sha(directory/'estimate.json')})
    report={'pass':True,'independent_checks':checks,'semantic_mutations_rejected':len(mutations),
            'mutations':mutations,'runs':summary,'reference_used':False,'scale_fitted':False,
            'protected_payload_access':False,'benchmark_execution_ready':False}
    with Path(output).open('x') as stream:json.dump(report,stream,indent=2);stream.write('\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();main(args.output)
