"""Separate raw-first MSA preprocessing and fixed-window execution artifacts."""
import argparse
from copy import deepcopy
from datetime import datetime, timezone
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

from .audit_td3_geometry import pose_matrix
from .td3_msa_calibration_audit import msa_camera
from .td3_thermal_provenance import producer_functions, preprocess, compare_image, match_stereo
from .td3_tracking_uncertainty import sha
from .td3_night_geometry import verify


def workspace_path(path):
    return Path(str(path).replace('/root/AirStack/ore-ms/', '/workspace/', 1))


def msa_rectification(source):
    cameras=[msa_camera(source/f'thermal_{side}.intrinsics.yaml') for side in ['left','right']]
    ext=yaml.safe_load((source/'extrinsics.yaml').read_text())
    t=np.linalg.inv(pose_matrix(ext['thermal_right']['value']))@pose_matrix(ext['thermal_left']['value'])
    k=[c[1] for c in cameras];d=[c[2] for c in cameras]
    rect=cv2.fisheye.stereoRectify(k[0],d[0],k[1],d[1],(640,512),
        t[:3,:3].copy(),t[:3,3].copy(),flags=cv2.CALIB_ZERO_DISPARITY,balance=0,fov_scale=1)
    maps=[cv2.fisheye.initUndistortRectifyMap(k[i],d[i],rect[i],rect[i+2],(640,512),cv2.CV_32FC1) for i in range(2)]
    return cameras,t,rect,maps


def prepare(root, output):
    output.mkdir(parents=True,exist_ok=False);cv2.setNumThreads(1)
    extraction_path=root/'results/msa-replay/inputs/extraction.json'
    extraction=json.loads(extraction_path.read_text())
    for f in extraction['files']:
        if sha(workspace_path(f['path']))!=f['sha256']:raise ValueError('Extracted file hash mismatch')
    source=root/'data/calibration/msa_cal';cameras,t,rect,maps=msa_rectification(source)
    config_path=root/'data/calibration/upstream/get_thermal_data.yaml';cfg=yaml.safe_load(config_path.read_text())
    producer=root/'data/calibration/upstream/get_thermal_data.py';functions=producer_functions(producer)
    old_cal=functions['read_kalibr_stereo'](cfg);records=[];replays=[];geometry=[];files={}
    info_paths=[root/f'results/stereo-sign/inputs/day/sensors/thermal_{side}_processed_camera_info/data.yaml' for side in ['left','right']]
    for case in extraction['cases']:
        tag=case['case'];dataset=output/'datasets'/tag;rows=deepcopy(case['rows']);original=workspace_path(case['original'])
        old_report=json.loads((original/'report.json').read_text())
        if sha(root/'models/macslam/MACT_Offroad.pth')!=old_report['checkpoint_sha256']:
            raise ValueError('Checkpoint differs from baseline')
        for i,(side,cam,info_path) in enumerate(zip(['left','right'],cameras,info_paths)):
            info=yaml.safe_load(info_path.read_text());info.update(height=512,width=640,
                distortion_model='equidistant',k=cam[1].tolist(),d=np.r_[cam[2],0.].tolist(),r=rect[i].tolist(),p=rect[i+2].tolist())
            dest=dataset/f'sensors/thermal_{side}_processed_camera_info/data.yaml';dest.parent.mkdir(parents=True,exist_ok=True)
            dest.write_text(yaml.safe_dump(info,sort_keys=False))
        for position,(row,pair) in enumerate(zip(rows,case['raw'])):
            raw_paths=[workspace_path(pair[s]['path']) for s in ['left','right']]
            raw=[cv2.imread(str(p),cv2.IMREAD_UNCHANGED) for p in raw_paths]
            if any(im is None or im.dtype!=np.uint16 or im.shape!=(512,640) for im in raw):raise ValueError('Expected native raw thermal uint16')
            previous=functions['stereo_rectify'](*[im.copy() for im in raw],old_cal)
            for i,side in enumerate(['left','right']):
                rel=row['thermal_'+side]['path']
                old_image=(root/'results/msa-replay/inputs/day'/rel if case['clip']=='day' else root/'data/tartandrive3/20251027/night_thermal_so_salon2'/rel)
                provided=cv2.imread(str(old_image),cv2.IMREAD_UNCHANGED)
                control=compare_image(provided,preprocess(previous[i],functions,cfg['process'],cfg['enhance']))
                replays.append(dict(case=tag,position=position,side=side,association=pair[side],control=control))
                if not control['exact_full_array_equal']:raise ValueError(f'Original producer replay differs: {tag}/{position}/{side}')
                processed=preprocess(cv2.remap(raw[i],*maps[i],cv2.INTER_LINEAR),functions,cfg['process'],cfg['enhance'])
                # Preserve the provided three replicated channels used by the original adapter.
                processed=np.repeat(processed[:,:,None],3,axis=2)
                dest=dataset/rel;dest.parent.mkdir(parents=True,exist_ok=True)
                if not cv2.imwrite(str(dest),processed):raise RuntimeError('Processed image write failed')
                files.update({str(p):sha(p) for p in [*raw_paths,old_image,dest]})
            if position==0:
                images=[cv2.imread(str(dataset/row['thermal_'+s]['path']),cv2.IMREAD_UNCHANGED) for s in ['left','right']]
                metrics,arrays=match_stereo(*images,rect[2],rect[3])
                dest=output/'stereo-matches';dest.mkdir(exist_ok=True)
                np.savez_compressed(dest/f'{tag}.npz',**arrays)
                geometry.append(dict(case=tag,**metrics))
                preview=np.hstack(images)
                for y in range(16,512,32):cv2.line(preview,(0,y),(1279,y),(0,200,255),1)
                cv2.imwrite(str(dest/f'{tag}.png'),preview)
        manifest=dataset/'manifest.json';manifest.write_text(json.dumps({'pairs':rows,'source':'MSA separate replay; original acquisition rows unchanged'},indent=2)+'\n')
        records.append(dict(case=tag,clip=case['clip'],window=case['window'],dataset=str(dataset),manifest=str(manifest),
            original=str(original),rows=rows,reference_dataset=str(root/'data/tartandrive3/20251027'/('1_salon_mac_cov_day' if case['clip']=='day' else 'night_thermal_so_salon2'))))
        files.update({str(p):sha(p) for p in [manifest,original/'report.json',original/'effective-config.json',original/'first-depth.npz',original/'trajectory.npz']})
        print('Prepared',tag,flush=True)
    paths=[Path(__file__),extraction_path,config_path,producer,root/'models/macslam/MACT_Offroad.pth',
        root/'MAC-VO-main/Scripts/Setup/run_td3_execution.py',root/'data/calibration/yamaha.yaml']
    paths.extend(p for p in source.iterdir() if p.is_file())
    for p in output.rglob('*'):
        if p.is_file():paths.append(p)
    files.update({str(p):sha(p) for p in paths});verify(files)
    plan=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),base_commit='90759fae1',cases=records,
        geometry=geometry,original_replay_checks=replays,files_sha256=files,T_right_from_left=t.tolist(),
        rectification=[r.tolist() for r in rect[:4]],
        protocol=dict(frames=20,seed=0,checkpoint='unchanged',intensity_processing='unchanged firestereo_v2/enhance',
            mapping=False,fp32=True,optimizer_parallel=False,reference_anchor='Original nominal vehicle-from-raw-left retained for BOTH calibrations',
            scope='Source-calibration image replay and recorded-SLAM proxy comparison, not independent metric/body ground truth'))
    (output/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')


def execute(case_path, output, case_name):
    """Use existing runner and defaults; passively retain first signed stereo output."""
    import torch
    from Module.Frontend.Frontend import FlowFormerCovFrontend
    from .run_td3_execution import run
    from .td3_tracking_uncertainty import percentiles
    plan=json.loads(case_path.read_text());verify(plan['files_sha256'])
    case=next(c for c in plan['cases'] if c['case']==case_name)
    first=True;records=[];original=FlowFormerCovFrontend.inference_2_depth
    def record(flow,cov,frame,enforce):
        nonlocal first
        result=original(flow,cov,frame,enforce)
        disparity=-flow[0,0].detach().cpu().numpy()
        records.append(dict(positive_disparity_fraction=float((disparity>0).mean()),disparity_px=percentiles(disparity)))
        if first:
            dest=output/'thermal'/'first-signed-stereo.npz'
            np.savez_compressed(dest,flow=flow[0].detach().cpu().numpy(),covariance=cov[0].detach().cpu().numpy(),
                depth=result.depth.detach().cpu().numpy())
            first=False
        return result
    FlowFormerCovFrontend.inference_2_depth=staticmethod(record)
    run_args=argparse.Namespace(dataset=Path(case['dataset']),manifest=Path(case['manifest']),output=output,
        modality='thermal',frames=20,start=0,acknowledge_provisional_geometry=True,record_uncertainty=True)
    try:run(run_args,'thermal')
    finally:
        output.mkdir(parents=True,exist_ok=True)
        (output/'stereo-sign-summary.json').write_text(json.dumps(records,indent=2)+'\n')
        FlowFormerCovFrontend.inference_2_depth=staticmethod(original)
    current=json.loads((output/'thermal/effective-config.json').read_text())
    baseline=json.loads((Path(case['original'])/'effective-config.json').read_text())
    if current!=baseline:raise ValueError('Effective configuration changed from baseline')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['prepare','execute'])
    parser.add_argument('--root',type=Path,default=Path('/workspace'))
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--plan',type=Path)
    parser.add_argument('--case')
    args=parser.parse_args()
    if args.action=='prepare':prepare(args.root,args.output)
    else:execute(args.plan,args.output,args.case)
