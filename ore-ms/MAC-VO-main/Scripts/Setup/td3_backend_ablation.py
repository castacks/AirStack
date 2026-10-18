"""Fixed-input analytic reprojection/disparity ablation; no inference or scale fit."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
import numpy as np
import torch
import pypose as pp
from scipy.spatial.transform import Rotation
from pypose.optim.scheduler import StopOnPlateau
from Module.Map import MatchObs,PointNode
from Module.Optimization.TwoFramePGO.Graphs import GraphInput
from Module.Optimization.TwoFramePGO.Optimizer import TwoFrame_PGO
from Module.Optimization.PyposeOptimizers import LM_analytic
try:
    from .td3_reference import load_reference,camera_reference,matrices,trajectory_metrics
except ImportError:
    from td3_reference import load_reference,camera_reference,matrices,trajectory_metrics


def pair_input(saved,index):
    first,second=saved['edge/match2frame1/mapping'],saved['edge/match2frame2/mapping']
    mask=(first==index)&(second==index+1)
    if not mask.any():raise ValueError('Missing consecutive pair')
    ids=saved['edge/match2point/mapping'][mask]
    obs=MatchObs.init({key.split('//')[1]:torch.from_numpy(saved[key][mask].copy())
                       for key in saved.files if key.startswith('match//')})
    pts=PointNode.init({key.split('//')[1]:torch.from_numpy(saved[key][ids].copy())
                       for key in saved.files if key.startswith('points//')})
    return GraphInput(frame_idx=torch.tensor([index+1]),from_idx=torch.tensor([index]),
        init_motion=pp.SE3(torch.from_numpy(saved['frames//pose'][index:index+1].copy())),
        baseline=torch.from_numpy(saved['frames//baseline'][index+1:index+2].copy()),
        observations=obs,points=pts,images_intrinsic=torch.from_numpy(saved['frames//K'][index+1].copy()),
        edges_index=torch.zeros(int(mask.sum()),dtype=torch.long),device='cpu')


def fingerprint(data):
    digest=hashlib.sha256()
    values={'initial_pose':data.init_motion.tensor(),'baseline':data.baseline,'K':data.images_intrinsic,
            **{'obs/'+k:v for k,v in data.observations.data.items()},
            **{'points/'+k:v for k,v in data.points.data.items()}}
    for key,value in sorted(values.items()):
        a=value.detach().cpu().numpy();digest.update(key.encode());digest.update(str(a.dtype).encode());digest.update(str(a.shape).encode());digest.update(a.tobytes())
    return digest.hexdigest()


def conditioning(graph):
    # Local camera-pose tangent, translation in meters and rotation in radians.
    graph()
    p=graph.pos_Tc.detach();n=len(p);fx,fy=graph.K[0,0],graph.K[1,1]
    x,y,z=p.unbind(-1)
    j=torch.zeros(n,graph().shape[1],3,dtype=p.dtype)
    j[:,0,0]=-fx*y/x.square();j[:,0,1]=fx/x
    j[:,1,0]=-fy*z/x.square();j[:,1,2]=fy/x
    if j.shape[1]==3:j[:,2,0]=-fx*graph.baseline.reshape(-1)[0]/x.square()
    motion=torch.cat([-torch.eye(3,dtype=p.dtype).repeat(n,1,1),pp.vec2skew(p)],dim=2)
    jac=j@motion
    cov=graph.covariance_array().detach()
    whitened=torch.linalg.solve_triangular(torch.linalg.cholesky(cov),jac,upper=False).reshape(-1,6)
    singular=torch.linalg.svdvals(whitened).cpu().numpy()
    return {'scope':'local camera tangent; meters/radians, inverse variance before Huber; not optimizer Hessian',
            'singular_values':singular.tolist(),'condition_number':float(singular[0]/singular[-1]) if singular[-1]>1e-12 else None}


def solve(data,backend):
    before=fingerprint(data)
    context=TwoFrame_PGO.init_context(SimpleNamespace(graph_type=backend,autodiff=False,
                                                     device='cpu',vectorize=True,parallel=False))
    graph=context['pose_graph_class'](data).to(device='cpu',dtype=torch.double)
    optimizer=LM_analytic(graph,min=1e-6,**context['optimizer_cfg'])
    scheduler=StopOnPlateau(optimizer,steps=10,patience=2,decreasing=1e-5,verbose=False)
    losses=[]
    while scheduler.continual():
        weight=torch.block_diag(*(torch.pinverse(graph.covariance_array().double())))
        loss=optimizer.step(input=(),weight=weight)
        losses.append(float(loss));scheduler.step(loss)
    result=graph.pose2opt.detach().tensor().cpu().numpy()[0]
    if not np.isfinite(result).all() or not np.isfinite(losses).all():raise ValueError('Nonfinite optimization')
    if not np.isclose(np.linalg.norm(result[3:]),1,atol=1e-5):raise ValueError('Non-unit pose')
    residual=graph().detach().cpu().numpy()
    if fingerprint(data)!=before:raise ValueError('Backend mutated frozen input')
    metrics={'input_sha256':before,'iterations':len(losses),'loss_history':losses,
        'loss_scope':'native solver-reported robust loss; different residual dimensions, not comparable between arms',
        'median_reprojection_px':float(np.median(np.linalg.norm(residual[:,:2],axis=1))),
        'median_disparity_abs_residual_px':float(np.median(np.abs(residual[:,2]))) if residual.shape[1]==3 else None,
        'conditioning':conditioning(graph)}
    return result,metrics


def compose(increments):
    poses=[np.eye(4)]
    for delta in increments:poses.append(poses[-1]@delta)
    return np.stack(poses)


def local_error(estimated,reference):
    delta=np.linalg.inv(reference)@estimated
    return np.linalg.norm(delta[:,:3,3],axis=1),np.degrees(Rotation.from_matrix(delta[:,:3,:3]).magnitude())


def run(directory,reference,args,output):
    saved=np.load(directory/'tensor-map.npz');source=np.load(directory/'trajectory.npz')
    times=source['time_ns'].reshape(-1);old=matrices(saved['frames//pose'])
    if not np.array_equal(times,saved['frames//time_ns'].reshape(-1)):raise ValueError('Timestamp mismatch')
    if not np.array_equal(source['poses_camera_ned'],saved['frames//pose']):raise ValueError('Pose mismatch')
    config=json.loads((directory/'effective-config.json').read_text())
    if config['Odometry']['optimizer']['args']!={'device':'cpu','vectorize':True,'parallel':False,'graph_type':'disp','autodiff':False}:raise ValueError('Unexpected source optimizer')
    if saved['frames//need_interp'].any():raise ValueError('Interpolated source unsupported')
    modality=directory.name
    camera=camera_reference(reference,times,args.dataset,args.calibration,modality)
    reference_delta=np.linalg.inv(camera[:-1])@camera[1:]
    old_delta=np.linalg.inv(old[:-1])@old[1:]
    output.mkdir(parents=True,exist_ok=False)
    reports={};arms={}
    for backend in ['disp','reproj']:
        motions=[];pairs=[]
        for i in range(len(times)-1):
            pose,metrics=solve(pair_input(saved,i),backend)
            motion=np.linalg.inv(old[i])@matrices(pose[None])[0]
            motions.append(motion);pairs.append(metrics)
        increments=np.stack(motions);trajectory=compose(increments)
        t,r=local_error(increments,reference_delta)
        replay_t,replay_r=local_error(increments,old_delta)
        reports[backend]={'trajectory':trajectory_metrics(trajectory,camera),
            'local_translation_rmse_m':float(np.sqrt(np.mean(t*t))),
            'local_rotation_rmse_deg':float(np.sqrt(np.mean(r*r))),
            'max_local_translation_difference_from_original_m':float(replay_t.max()),
            'max_local_rotation_difference_from_original_deg':float(replay_r.max()),
            'median_local_jacobian_condition_number':float(np.median([p['conditioning']['condition_number'] for p in pairs])),
            'pairs':pairs}
        np.savez_compressed(output/(backend+'.npz'),T_composed_camera=trajectory,
            T_camera0_from_camera1=increments,time_ns=times,
            local_reference_translation_error_m=t,local_reference_rotation_error_deg=r)
        arms[backend]=reports[backend]
        print(directory.parent.name,modality,backend,'RMS',reports[backend]['trajectory']['anchored_translation_rmse_m'],flush=True)
    hashes_match=all(a['input_sha256']==b['input_sha256'] for a,b in zip(arms['disp']['pairs'],arms['reproj']['pairs']))
    if not hashes_match:raise ValueError('Arms received different inputs')
    reports['validation']={'identical_arm_inputs':hashes_match,
        'disp_replay_within_tolerance':reports['disp']['max_local_translation_difference_from_original_m']<=1e-3 and reports['disp']['max_local_rotation_difference_from_original_deg']<=.01,
        'tolerances':{'translation_m':1e-3,'rotation_deg':.01}}
    return reports


def plot(results,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(11,4),layout='constrained')
    names=['early','original','late'];x=np.arange(3)
    for ax,m in zip(axes,['rgb','thermal']):
        for offset,arm in [(-.18,'disp'),(.18,'reproj')]:
            vals=[results[n][m][arm]['trajectory']['anchored_translation_rmse_m'] for n in names]
            ax.bar(x+offset,vals,.36,label=arm)
        ax.set_xticks(x,names);ax.set_title(m.upper());ax.set_ylabel('Recorded SLAM disagreement RMS (m)');ax.legend()
    fig.suptitle('Fixed matches and initialization: composed independent pair estimates')
    fig.savefig(output/'backend-comparison.png',dpi=160);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for k in ['source','dataset','calibration','output']:parser.add_argument('--'+k,type=Path,required=True)
    parser.add_argument('--threads',type=int,default=1,help='CPU threads, recorded in provenance; replay tolerance must pass')
    args=parser.parse_args()
    if args.threads<1:parser.error('threads must be positive')
    torch.set_num_threads(args.threads)
    args.output.mkdir(parents=True,exist_ok=False)
    # Both arms use the same thread count; verify disparity replay against original.
    torch.manual_seed(0);np.random.seed(0)
    reference,_=load_reference(args.dataset);results={}
    for name in ['early','original','late']:
        results[name]={}
        for modality in ['rgb','thermal']:
            results[name][modality]=run(args.source/name/modality,reference,args,args.output/name/modality)
    files=[Path(__file__),ROOT/'Module/Optimization/PyposeOptimizers.py',ROOT/'Module/Optimization/TwoFramePGO/Graphs.py',ROOT/'Module/Optimization/TwoFramePGO/Optimizer.py']
    inputs=[args.calibration]
    for name in results:
        for m in results[name]:
            inputs.extend(args.source/name/m/f for f in ['tensor-map.npz','trajectory.npz','effective-config.json'])
    report={'created_at_utc':datetime.now(timezone.utc).isoformat(),
        'scope':'fixed-input objective ablation with independent frozen-prior pairs; same clip/provisional calibration/SLAM proxy, no GT',
        'torch':torch.__version__,'torch_threads':torch.get_num_threads(),
        'source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        'input_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},'results':results}
    (args.output/'analysis.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    plot(results,args.output)
    if not all(results[n][m]['validation']['disp_replay_within_tolerance'] for n in results for m in results[n]):
        raise RuntimeError('Disp replay tolerance exceeded; do not interpret ablation until investigated')


if __name__=='__main__':main()
