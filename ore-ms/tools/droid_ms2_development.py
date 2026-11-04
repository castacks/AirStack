"""Pinned DROID offline stereo adapter; exact source clocks remain in sidecars."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
REVISION='2dfd39f0dcad44012ca7bbb8aa70b55edbfa9c99'
INTERNAL_BASELINE=.1
NATIVE_TIME_UNIT_NS=100_000_000
# Pinned native demo defaults, with explicit stereo/buffer/no-viewer mode.
SETTINGS={'buffer':64,'stereo':True,'disable_vis':True,'beta':.3,'filter_thresh':2.4,
          'warmup':8,'keyframe_thresh':4.,'frontend_thresh':16.,'frontend_window':25,
          'frontend_radius':2,'frontend_nms':1,'backend_thresh':22.,
          'backend_radius':2,'backend_nms':3,'upsample':False,
          'frontend_device':'cuda:0','backend_device':'cuda:0','asynchronous':False}


def read(path):return json.loads(Path(path).read_text())
def sha(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()
def write(path,value):
    with Path(path).open('x') as stream:json.dump(value,stream,indent=2,allow_nan=False);stream.write('\n')


def native_times(times):
    if (not times or any(type(t) is not int or not 0<=t<2**63 for t in times)
            or any(a>=b for a,b in zip(times,times[1:]))):
        raise ValueError('strict chronological integer nanosecond source times required')
    relative=np.asarray([(t-times[0])/NATIVE_TIME_UNIT_NS for t in times],dtype=np.float32)
    if not np.isfinite(relative).all() or not np.all(np.diff(relative)>0):
        raise ValueError('native float clock collisions/nonfinite values')
    return relative


def canonical_pair(directory,stem,shape):
    images=[]
    for side in ['left','right']:
        raw=cv2.imread(str(Path(directory)/side/(stem+'.png')),cv2.IMREAD_UNCHANGED)
        if raw is None or raw.dtype!=np.uint8 or list(raw.shape[:2])!=list(shape):
            raise ValueError('invalid canonical source image')
        if raw.ndim==2:raw=np.repeat(raw[...,None],3,axis=2)
        if raw.ndim!=3 or raw.shape[2]!=3:raise ValueError('expected canonical BGR/grayscale')
        images.append(raw)
    return np.stack(images,axis=0)


def model_geometry(shape):
    height,width=shape
    if any(type(v) is not int or v<=0 for v in shape):raise ValueError('positive integer image dimensions required')
    scale=min(1.,np.sqrt((384*512)/(height*width)))
    resized=[int(height*scale),int(width*scale)]
    final=[v-v%8 for v in resized]
    if min(final)<8:raise ValueError('image too small for native8x feature grid')
    if 48*(final[0]//8*final[1]//8)**2>=2**31:
        raise ValueError('native32-bit correlation capacity exceeded')
    return {'source_shape':list(shape),'resize_shape':resized,'shape':final,
            'scale_xy':[resized[1]/width,resized[0]/height],'crop_origin_xy':[0,0]}


def transform_pair(images,geometry):
    result=[]
    for image in images:
        if list(image.shape[:2])!=geometry['source_shape']:raise ValueError('source geometry mismatch')
        height,width=geometry['resize_shape']
        if [height,width]!=geometry['source_shape']:
            image=cv2.resize(image,(width,height),interpolation=cv2.INTER_LINEAR)
        final_height,final_width=geometry['shape']
        result.append(image[:final_height,:final_width].copy())
    return np.stack(result)


def metric_poses(native_matrices,baseline_m,initialized):
    if (type(baseline_m) not in (int,float) or not np.isfinite(baseline_m) or baseline_m<=0
            or type(initialized) is not bool):
        raise ValueError('positive provided metric baseline and explicit initialization required')
    ratio=baseline_m/INTERNAL_BASELINE
    poses=[];segments=[];reasons=[];active=False;segment=-1
    for value in native_matrices:
        reason=None
        if not initialized:reason='frontend_not_initialized'
        elif value is None:reason='missing_native_output'
        else:
            matrix=np.asarray(value,dtype=float)
            if (matrix.shape!=(4,4) or not np.isfinite(matrix).all()
                    or not np.array_equal(matrix[3],[0,0,0,1])):reason='invalid_native_matrix'
            elif (np.linalg.norm(matrix[:3,:3].T@matrix[:3,:3]-np.eye(3))>1e-5
                    or abs(np.linalg.det(matrix[:3,:3])-1)>1e-5):reason='nonrigid_native_matrix'
        if reason is not None:
            poses.append(None);segments.append(None);reasons.append(reason);active=False
            continue
        if not active:segment+=1
        converted=matrix.copy();converted[:3,3]*=ratio
        poses.append(converted.tolist());segments.append(segment);reasons.append(None);active=True
    return poses,segments,reasons


def valid_native_vectors(vectors):
    values=np.asarray(vectors)
    if values.ndim!=2 or values.shape[1]!=7:raise ValueError('native xyz+xyzw output required')
    return np.isfinite(values).all(axis=1)&(np.abs(np.linalg.norm(values[:,3:],axis=1)-1)<=1e-5)


def execute(protocol,case_id,output):
    import subprocess
    import torch
    import lietorch
    protocol_path=Path(protocol);plan=read(protocol_path)
    for name,expected in plan['source_sha256'].items():
        if sha(ROOT/name)!=expected:raise ValueError('changed frozen source/input '+name)
    case=next(row for row in plan['runs'] if row['case']==case_id)
    source=read(ROOT/case['manifest']);camera='rgb' if case['modality']=='rgb' else 'thr'
    times=[r['cameras'][camera]['left']['recorded_time_ns'] for r in source['records']]
    right=[r['cameras'][camera]['right']['recorded_time_ns'] for r in source['records']]
    stamps=native_times(times);shape=case['shape'];baseline=case['baseline_m']
    if any(v%8 for v in shape):raise ValueError('canonical dimensions must be divisible by8; no implicit crop')
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    torch.set_num_threads(1);cv2.setNumThreads(1);torch.manual_seed(0);np.random.seed(0)
    torch.cuda.manual_seed_all(0)
    sys.path.insert(0,'/opt/DROID-SLAM/droid_slam')
    from droid import Droid
    revision=subprocess.check_output(['git','-C','/opt/DROID-SLAM','rev-parse','HEAD'],text=True).strip()
    if revision!=REVISION:raise ValueError('wrong DROIDsource revision')
    k=np.asarray(case['intrinsics'],dtype=np.float32)
    options={**SETTINGS,'image_size':shape,'weights':str(ROOT/plan['weights'])}
    write(output/'execution-lock.json',{'protocol_sha256':sha(protocol_path),'case':case,
        'settings':options,'runtime_image':plan['runtime_image'],'droid_revision':revision,
        'droid_diff':subprocess.check_output(['git','-C','/opt/DROID-SLAM','diff'],text=True),
        'lietorch_diff':subprocess.check_output(['git','-C','/opt/DROID-SLAM/thirdparty/lietorch','diff'],text=True),
        'weights_sha256':sha(ROOT/plan['weights']),'torch':torch.__version__,'cuda':torch.version.cuda,
        'opencv':cv2.__version__,'numpy':np.__version__,'gpu':torch.cuda.get_device_name(),
        'tf32_matmul':torch.backends.cuda.matmul.allow_tf32,
        'tf32_cudnn':torch.backends.cudnn.allow_tf32,
        'cudnn_benchmark':torch.backends.cudnn.benchmark,
        'deterministic_algorithms':torch.are_deterministic_algorithms_enabled(),
        'native_baseline':INTERNAL_BASELINE,'metric_translation_factor':baseline/INTERNAL_BASELINE,
        'native_time_unit_ns':NATIVE_TIME_UNIT_NS,'seed':0,
        'native_time_max_quantization_error_ns':float(max(abs(float(t)*NATIVE_TIME_UNIT_NS-(s-times[0])) for t,s in zip(stamps,times)))})
    native=None;calls=[];initialized=False;failure=None;completed=False;keyframes=None
    raw_matrices=[None]*len(times);vector_valid=[False]*len(times);input_reads=[]
    begin=time.monotonic_ns();torch.cuda.reset_peak_memory_stats()
    def stream(phase):
        for index,(stamp,row) in enumerate(zip(stamps,source['records'])):
            raw=canonical_pair(ROOT/case['canonical'],row['stem'],case.get('source_shape',shape))
            if 'image_geometry' in case:raw=transform_pair(raw,case['image_geometry'])
            tensor=torch.from_numpy(raw).permute(0,3,1,2).float()
            input_reads.append({'phase':phase,'source_index':index,'native_time':float(stamp),
                'image_shape':list(tensor.shape),'image_tensor_sha256':hashlib.sha256(tensor.numpy().tobytes()).hexdigest(),
                'intrinsics':k.tolist()})
            yield float(stamp),tensor,torch.from_numpy(k.copy())
    with (output/'tracking.jsonl').open('x') as journal:
        try:
            native=Droid(SimpleNamespace(**options))
            for index,(stamp,images,intrinsics) in enumerate(stream('tracking')):
                torch.cuda.synchronize();before=time.monotonic_ns()
                native.track(stamp,images,intrinsics=intrinsics)
                torch.cuda.synchronize();received=time.monotonic_ns()
                initialized=bool(native.frontend.is_initialized)
                row={'source_index':index,'source_stem':source['records'][index]['stem'],
                     'time_ns':times[index],'right_time_ns':right[index],
                     'native_time':stamp,'native_keyframe_count':native.video.counter.value,
                     'frontend_initialized':initialized,
                     'processing_ns':received-before,'receipt_elapsed_ns':received-begin}
                calls.append(row);journal.write(json.dumps(row)+'\n');journal.flush()
            initialized=bool(native.frontend.is_initialized)
            keyframes=native.video.tstamp[:native.video.counter.value].cpu().tolist()
            if plan.get('uninitialized_backend_guard') and not initialized:
                failure='frontend_not_initialized; offline backend deliberately skipped'
                vectors=None
            else:vectors=native.terminate(stream('filler'))
            torch.cuda.synchronize()
            if vectors is None:raise RuntimeError(failure)
            if vectors.shape!=(len(times),7):raise ValueError('native terminate output count/shape mismatch')
            # terminate() already inverts T_camera_from_world. Do not invert twice.
            mask=valid_native_vectors(vectors);vector_valid=mask.tolist()
            matrices=np.full((len(times),4,4),np.nan,dtype=np.float32)
            if mask.any():matrices[mask]=lietorch.SE3(torch.from_numpy(vectors[mask].copy())).matrix().numpy()
            raw_matrices=[matrix.tolist() if valid else None for matrix,valid in zip(matrices,mask)]
            np.savez_compressed(output/'native-trajectory.npz',xyz_xyzw=vectors,
                                world_from_camera_native=matrices,native_time=stamps)
            completed=True
        except Exception as error:
            failure=type(error).__name__+': '+str(error)
    poses,segments,reasons=metric_poses(raw_matrices,baseline,initialized)
    write(output/'input-reads.json',{'reads':input_reads,'reference_used':False})
    success=completed and initialized and all(p is not None for p in poses)
    result={'time_ns':times,'right_time_ns':right,'source_ids':[r['stem'] for r in source['records']],
        'poses':poses,'segments':segments,'pose_rejection_reasons':reasons,
        'terminal_success':success,'native_execution_completed':completed,
        'backend_skipped_for_uninitialized':bool(plan.get('uninitialized_backend_guard') and not initialized and len(calls)==len(times)),
        'frontend_initialized':initialized,'failure':failure,
        'source_frames':len(times),'native_track_calls_returned':len(calls),
        'finite_admitted_pose_frames':sum(p is not None for p in poses),
        'native_keyframe_times':keyframes,'native_keyframe_count':len(keyframes) if keyframes is not None else None,
        'native_vector_valid':vector_valid,
        'wall_runtime_ns':time.monotonic_ns()-begin,
        'peak_cuda_memory_allocated':torch.cuda.max_memory_allocated(),
        'metric_translation_factor':baseline/INTERNAL_BASELINE,
        'coordinate_frame':case['modality']+'_rectified_left_camera',
        'output_provenance':'offline native backend plus trajectory filler image-based refinement',
        'native_filler_uses_interpolated_initialization':True,
        'native_tracking_failure_status_available':False,
        'online_output_latency_available':False,'reference_used':False,'scale_fitted':False,
        'physical_frame_or_clock_verified':False,'scope':'exposed development offline stereo context only'}
    write(output/'estimate.json',result)
    print(json.dumps({key:result[key] for key in ['terminal_success','frontend_initialized','native_execution_completed','native_track_calls_returned','finite_admitted_pose_frames','failure']}))
    return success


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--protocol',type=Path,required=True);parser.add_argument('--case',required=True)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    raise SystemExit(0 if execute(args.protocol,args.case,args.output) else 1)
