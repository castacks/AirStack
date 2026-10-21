"""Retain raw signed stereo flow on the fixed EXP-018 first-frame inputs."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import cv2
import numpy as np

from .td3_stereo_sign import bilinear
from .td3_tracking_uncertainty import sha, percentiles
from .td3_night_geometry import verify


def signed_metrics(observed_left_minus_right, model_flow, mask):
    observed=np.asarray(observed_left_minus_right,float);flow=np.asarray(model_flow,float)
    if observed.shape!=flow.shape or observed.ndim!=2 or observed.shape[1]!=2:
        raise ValueError('Expected matching Nx2 displacement arrays')
    valid=mask&np.isfinite(observed).all(axis=1)&np.isfinite(flow).all(axis=1)
    observed=observed[valid];prediction=-flow[valid]
    signed=(np.abs(observed[:,0])>1)&(np.abs(prediction[:,0])>1)
    return dict(count=int(valid.sum()),signed_population_count=int(signed.sum()),
        observed_disparity_px=percentiles(observed[:,0]),model_disparity_px=percentiles(prediction[:,0]),
        horizontal_error_px=percentiles(np.abs(prediction[:,0]-observed[:,0])),
        vertical_error_px=percentiles(np.abs(prediction[:,1]-observed[:,1])),
        sign_agreement=float((np.sign(prediction[signed,0])==np.sign(observed[signed,0])).mean()) if signed.any() else None)


def depth_reproduction(saved, current, tolerance=1e-4):
    saved=np.asarray(saved);current=np.asarray(current)
    if saved.shape!=current.shape:raise ValueError('Depth shape mismatch')
    old=np.isfinite(saved)&(saved>0);new=np.isfinite(current)&(current>0)
    common=old&new
    relative=np.abs(current[common].astype(float)-saved[common])/saved[common]
    maximum=float(relative.max()) if relative.size else None
    return dict(positive_masks_equal=bool(np.array_equal(old,new)),count=int(common.sum()),
        max_relative_error=maximum,max_absolute_error_m=float(np.max(np.abs(current[common].astype(float)-saved[common]))) if common.any() else None,
        bitwise_equal=bool(np.array_equal(saved,current)),tolerance=tolerance,
        pass_control=bool(np.array_equal(old,new) and maximum is not None and maximum<=tolerance))


def prepare(source,output):
    inherited=json.loads((source/'plan.json').read_text());files=dict(inherited['files_sha256']);verify(files)
    paths=[Path(__file__),Path(__file__).parents[1]/'UnitTest/test_td3_signed_frontend.py',source/'analysis.json',source/'plan.json']
    paths.extend((source/'arrays').glob('*.npz'))
    for case in inherited['cases']:
        weight=Path('/workspace/models/macslam')/('MACT_Offroad.pth' if case['modality']=='thermal' else 'MACVO_FrontendCov.pth')
        report=json.loads(Path(case['report']).read_text())
        if sha(weight)!=report['checkpoint_sha256']:raise ValueError('Original checkpoint hash mismatch')
        paths.append(weight)
    paths.extend(Path('/workspace/MAC-VO-main/Module/Network').rglob('*.py'))
    paths.extend([Path('/workspace/MAC-VO-main/DataLoader/Dataset/TD3.py'),Path('/workspace/MAC-VO-main/Module/Frontend/Matching.py'),
        Path('/workspace/MAC-VO-main/Module/Frontend/StereoDepth.py'),Path('/workspace/MAC-VO-main/Docker/Dockerfile.blackwell')])
    files.update({str(p.resolve()):sha(p) for p in paths if p.is_file()});verify(files)
    output.mkdir(parents=True,exist_ok=False)
    plan=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),base_commit='75222af3e',source=str(source),cases=inherited['cases'],files_sha256=files,
        protocol=dict(seed=0,native_fp32=True,depth_control_max_relative_error=1e-4,
            preprocessing='Provided uint8 PNG; BGR to RGB, CPU float32 /255, native resolution',
            signed_convention='Model flow maps source left coordinates to target right; left-minus-right disparity = -flow_x',
            selection='Same 12 first stereo pairs fixed before outcomes; no additional windows or VO',
            scope='Signed model correspondence consistency; not independent calibration/depth truth'))
    (output/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')


def run(output):
    import torch
    from Module.Network.FlowFormer.configs.submission import get_cfg
    from Module.Network.FlowFormerCov import build_flowformer
    from Module.Network.FlowFormerCov.checkpoint import load_cov_checkpoint
    plan=json.loads((output/'plan.json').read_text());verify(plan['files_sha256'])
    records=[];(output/'arrays').mkdir();torch.set_num_threads(1)
    for case in plan['cases']:
        tag='-'.join(case[k] for k in ['clip','window','modality'])
        record=dict(tag=tag,status='RUNNING');model=None
        try:
            torch.manual_seed(0);np.random.seed(0)
            config=json.loads(Path(case['config']).read_text())['Odometry']['frontend']['args']
            cfg=get_cfg();cfg.latentcostformer.pretrain=False;cfg.latentcostformer.decoder_depth=config['decoder_depth']
            model=build_flowformer(cfg,torch.float32,torch.float32).eval().cuda()
            checkpoint=Path('/workspace/models/macslam')/('MACT_Offroad.pth' if case['modality']=='thermal' else 'MACVO_FrontendCov.pth')
            migration=load_cov_checkpoint(model,torch.load(checkpoint,map_location='cpu',weights_only=True),allow_legacy=case['modality']=='rgb')
            images=[]
            for p in case['images']:
                image=cv2.imread(p,cv2.IMREAD_UNCHANGED)
                if image is None or image.dtype!=np.uint8:raise ValueError('Expected provided uint8 stereo PNG')
                image=cv2.cvtColor(image,cv2.COLOR_GRAY2RGB if image.ndim==2 else cv2.COLOR_BGR2RGB)
                images.append((torch.from_numpy(image.copy()).permute(2,0,1).unsqueeze(0).float()/255.).cuda())
            with torch.inference_mode():flow,cov=model.inference(*images)
            torch.cuda.synchronize()
            if not torch.isfinite(flow).all() or not torch.isfinite(cov).all() or not (cov>0).all():raise ValueError('Invalid model output')
            report=json.loads(Path(case['report']).read_text())
            depth=((report['execution_baseline_m']*report['K'][0][0])*flow[:,:1].abs().reciprocal()).cpu().numpy()
            flow=flow[0].cpu().numpy();cov=cov[0].cpu().numpy()
            with np.load(case['depth']) as saved:control=depth_reproduction(saved['depth'],depth)
            with np.load(Path(plan['source'])/'arrays'/(tag+'.npz')) as matches:
                uv=matches['uv0'][matches['inlier']];observed=matches['observed_inlier_delta']
                sampled=np.column_stack([bilinear(channel,uv) for channel in flow])
                populations={name:signed_metrics(observed,sampled,matches['mask_'+name]) for name in ['all_inliers','vertical_le_1.5px']}
            np.savez_compressed(output/'arrays'/(tag+'.npz'),flow=flow,covariance=cov,depth=depth,
                flow_at_inliers=sampled,observed_inlier_delta=observed)
            record.update(status='PASS' if control['pass_control'] else 'DEPTH_CONTROL_FAILED',migration=migration,
                depth_control=control,populations=populations,
                model_left_minus_right_disparity_px=percentiles(-flow[0]),
                positive_model_disparity_fraction=float((-flow[0]>0).mean()),
                checkpoint_sha256=sha(checkpoint),shape=list(flow.shape))
        except Exception as error:
            record.update(status='FAILED',error_type=type(error).__name__,error=str(error))
        finally:
            del model;torch.cuda.empty_cache()
        records.append(record);print(tag,record['status'],flush=True)
        (output/'progress.json').write_text(json.dumps(records,indent=2)+'\n')
    result=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),base_commit=plan['base_commit'],
        torch=torch.__version__,cuda=torch.version.cuda,opencv=cv2.__version__,numpy=np.__version__,
        gpu=torch.cuda.get_device_name(0),cases=records,verified_hashes=verify(plan['files_sha256']))
    (output/'analysis.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    if any(r['status']!='PASS' for r in records):raise RuntimeError('Signed inference has failed/control-failed cases; inspect retained records')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=['prepare','run'])
    parser.add_argument('--source',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.action=='prepare':prepare(args.source,args.output)
    else:run(args.output)
