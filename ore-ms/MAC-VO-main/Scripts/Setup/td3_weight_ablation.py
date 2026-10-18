"""Frozen per-channel covariance ablation on saved graph inputs; no inference/scale fit."""
import argparse
from dataclasses import replace
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
import torch
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from Module.Map import MatchObs
try:
    from .td3_backend_ablation import pair_input,fingerprint,solve,compose,local_error
    from .td3_reference import load_reference,camera_reference,matrices,trajectory_metrics
except ImportError:
    from td3_backend_ablation import pair_input,fingerprint,solve,compose,local_error
    from td3_reference import load_reference,camera_reference,matrices,trajectory_metrics

ARMS=('learned','fixed_uv','fixed_disp','fixed_both')
WEIGHT_FIELDS={'pixel2_uv_cov','pixel2_disp_cov'}


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def geometry_fingerprint(data):
    obs=MatchObs.init({k:v for k,v in data.observations.data.items() if k not in WEIGHT_FIELDS})
    return fingerprint(replace(data,observations=obs))


def set_weights(data,arm,constants):
    if arm not in ARMS:raise ValueError('Unknown weight arm')
    values=np.asarray(constants,dtype=float)
    if values.shape!=(3,) or not np.isfinite(values).all() or np.any(values<=0):
        raise ValueError('Three finite positive channel variances required')
    uv=data.observations.data['pixel2_uv_cov'];disp=data.observations.data['pixel2_disp_cov']
    if not torch.isfinite(uv).all() or not torch.isfinite(disp).all() or (uv[:,:2]<=0).any() or (disp<=0).any():
        raise ValueError('Source covariance must be finite positive')
    if (uv[:,2]!=0).any():raise ValueError('Protocol requires zero uv cross-covariance')
    before=geometry_fingerprint(data)
    if arm in ('fixed_uv','fixed_both'):
        uv[:,0].fill_(float(values[0]));uv[:,1].fill_(float(values[1]))
    if arm in ('fixed_disp','fixed_both'):disp.fill_(float(values[2]))
    if geometry_fingerprint(data)!=before:raise ValueError('Weight arm changed nonweight inputs')
    return before


def prepare(args):
    constants={}
    for m in ['rgb','thermal']:
        with np.load(args.source/'original'/m/'tensor-map.npz') as graph:
            first,second=graph['edge/match2frame1/mapping'],graph['edge/match2frame2/mapping']
            mask=(first>=0)&(second==first+1)&(second<10)
            if len(np.unique(second[mask]))!=9:raise ValueError('Expected first nine training pairs')
            uv=graph['match//pixel2_uv_cov'][mask];disp=graph['match//pixel2_disp_cov'][mask].ravel()
            if np.any(uv[:,2]!=0):raise ValueError('Nonzero source cross-covariance')
            channels=np.column_stack([uv[:,:2],disp])
            if not np.isfinite(channels).all() or np.any(channels<=0):raise ValueError('Invalid source variances')
            constants[m]={'variance_u_v_disp_px2':np.median(channels,axis=0).tolist(),
                'observations':int(mask.sum()),'pairs':9,'source_frame_range_inclusive':[0,9]}
    paths=[args.source/n/m/f for n in ['early','original','late'] for m in ['rgb','thermal']
           for f in ['tensor-map.npz','trajectory.npz','effective-config.json']]
    plan={'created_at_utc':datetime.now(timezone.utc).isoformat(),'arms':ARMS,
          'constants_rule':'channelwise median active covariance from original frames0..9; no reference data or evaluation-window tuning',
          'constants':constants,'backend':'disp','threads':args.threads,
          'calibration_sha256':sha(args.calibration),
          'source_inputs_sha256':{str(p.relative_to(args.source)):sha(p) for p in paths},
          'scope':'fixed matches/depth/initialization; separate same-clip windows and SLAM proxy, not GT or calibrated noise'}
    args.output.mkdir(parents=True,exist_ok=False)
    with (args.output/'plan.json').open('x') as stream:json.dump(plan,stream,indent=2)
    print(json.dumps(plan['constants'],indent=2),flush=True)


def run(directory,reference,args,output,constants):
    with np.load(directory/'tensor-map.npz') as saved,np.load(directory/'trajectory.npz') as source:
        times=source['time_ns'].reshape(-1);old=matrices(saved['frames//pose'])
        if not np.array_equal(times,saved['frames//time_ns'].reshape(-1)) or not np.array_equal(source['poses_camera_ned'],saved['frames//pose']):
            raise ValueError('Source timestamp/pose association mismatch')
        if saved['frames//need_interp'].any():raise ValueError('Interpolated source unsupported')
        config=json.loads((directory/'effective-config.json').read_text())
        expected={'device':'cpu','vectorize':True,'parallel':False,'graph_type':'disp','autodiff':False}
        if config['Odometry']['optimizer']['args']!=expected:raise ValueError('Unexpected source optimizer')
        camera=camera_reference(reference,times,args.dataset,args.calibration,directory.name)
        reference_delta=np.linalg.inv(camera[:-1])@camera[1:]
        old_delta=np.linalg.inv(old[:-1])@old[1:]
        reports={};geometry_hashes=[]
        output.mkdir(parents=True,exist_ok=False)
        for arm in ARMS:
            motions=[];pairs=[];signatures=[]
            for i in range(len(times)-1):
                data=pair_input(saved,i);signature=set_weights(data,arm,constants)
                pose,metrics=solve(data,'disp')
                signatures.append(signature);metrics['geometry_sha256']=signature
                motions.append(np.linalg.inv(old[i])@matrices(pose[None])[0]);pairs.append(metrics)
            geometry_hashes.append(signatures)
            increments=np.stack(motions);trajectory=compose(increments)
            t,r=local_error(increments,reference_delta);rt,rr=local_error(increments,old_delta)
            conditions=[p['conditioning']['condition_number'] for p in pairs]
            reports[arm]={'trajectory':trajectory_metrics(trajectory,camera),
                'local_translation_rmse_m':float(np.sqrt(np.mean(t*t))),
                'local_rotation_rmse_deg':float(np.sqrt(np.mean(r*r))),
                'max_local_translation_difference_from_original_m':float(rt.max()),
                'max_local_rotation_difference_from_original_deg':float(rr.max()),
                'median_local_jacobian_condition_number':float(np.median(conditions)) if all(c is not None for c in conditions) else None,
                'pairs':pairs}
            np.savez_compressed(output/(arm+'.npz'),T_composed_camera=trajectory,T_camera0_from_camera1=increments,
                time_ns=times,local_reference_translation_error_m=t,local_reference_rotation_error_deg=r)
            print(directory.parent.name,directory.name,arm,'RMS',reports[arm]['trajectory']['anchored_translation_rmse_m'],flush=True)
        if not all(hashes==geometry_hashes[0] for hashes in geometry_hashes):raise ValueError('Arms changed geometry inputs')
        control=reports['learned']
        reports['validation']={'identical_nonweight_inputs':True,
            'learned_replay_within_tolerance':control['max_local_translation_difference_from_original_m']<=1e-3 and control['max_local_rotation_difference_from_original_deg']<=.01,
            'tolerances':{'translation_m':1e-3,'rotation_deg':.01}}
        return reports


def plot(results,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(12,4),layout='constrained')
    names=['early','original','late'];x=np.arange(3)
    for ax,m in zip(axes,['rgb','thermal']):
        for i,arm in enumerate(ARMS):
            ax.bar(x+(i-1.5)*.19,[results[n][m][arm]['trajectory']['anchored_translation_rmse_m'] for n in names],.19,label=arm)
        ax.set_xticks(x,names);ax.set_title(m.upper());ax.set_ylabel('Recorded SLAM disagreement RMS (m)');ax.legend(fontsize=8)
    fig.suptitle('Fixed matches/depth/initialization: only covariance weights differ')
    fig.savefig(output/'weight-comparison.png',dpi=160);plt.close(fig)


def evaluate(args):
    plan=json.loads((args.output/'plan.json').read_text())
    if plan['threads']!=args.threads or plan['calibration_sha256']!=sha(args.calibration):raise ValueError('Plan setting mismatch')
    for rel,digest in plan['source_inputs_sha256'].items():
        if sha(args.source/rel)!=digest:raise ValueError('Frozen input hash mismatch')
    torch.set_num_threads(args.threads);torch.manual_seed(0);np.random.seed(0)
    reference,_=load_reference(args.dataset);results={}
    for name in ['early','original','late']:
        results[name]={}
        for modality in ['rgb','thermal']:
            results[name][modality]=run(args.source/name/modality,reference,args,args.output/name/modality,
                plan['constants'][modality]['variance_u_v_disp_px2'])
    files=[Path(__file__),Path(__file__).with_name('td3_backend_ablation.py'),Path(__file__).with_name('td3_reference.py'),
           ROOT/'Module/Optimization/PyposeOptimizers.py',ROOT/'Module/Optimization/TwoFramePGO/Graphs.py',ROOT/'Module/Optimization/TwoFramePGO/Optimizer.py']
    refdir=args.dataset/'super_odometry/odometry'
    report={'created_at_utc':datetime.now(timezone.utc).isoformat(),'plan':plan,'plan_sha256':sha(args.output/'plan.json'),
        'torch':torch.__version__,'torch_threads':torch.get_num_threads(),
        'source_sha256':{str(p.relative_to(ROOT)):sha(p) for p in files},
        'reference_input_sha256':{str(p.relative_to(args.dataset)):sha(p) for p in [refdir/'interp_data.txt',refdir/'interp_timestamps.txt',refdir/'info.yaml']},
        'results':results}
    with (args.output/'analysis.json').open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    plot(results,args.output)
    if not all(results[n][m]['validation']['learned_replay_within_tolerance'] for n in results for m in results[n]):
        raise RuntimeError('Learned control replay failed; investigate before interpreting')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['prepare','evaluate'])
    for key in ['source','dataset','calibration','output']:parser.add_argument('--'+key,type=Path,required=True)
    parser.add_argument('--threads',type=int,default=1)
    args=parser.parse_args()
    if args.threads<1:parser.error('threads must be positive')
    (prepare if args.action=='prepare' else evaluate)(args)
