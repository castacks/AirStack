"""Nonoverlapping fixed100-group night windows with original timestamp gaps."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil

import cv2
import numpy as np
import yaml

from .td3_msa_replay import msa_rectification, workspace_path
from .td3_thermal_provenance import producer_functions, preprocess, compare_image
from .td3_tracking_uncertainty import sha, percentiles
from .td3_night_geometry import verify


def prepare(root,output):
    output.mkdir(parents=True,exist_ok=False);cv2.setNumThreads(1)
    base=root/'results/long-night-validation';source=root/'data/tartandrive3/20251027/night_mac_salon_warehousefig8'
    extraction=json.loads((base/'image-extraction.json').read_text())
    for rel,rec in extraction['files'].items():
        if sha(source/rel)!=rec['sha256']:raise ValueError('Image changed')
    cameras,t,rect,maps=msa_rectification(root/'data/calibration/msa_cal')
    cfgpath=root/'data/calibration/upstream/get_thermal_data.yaml';cfg=yaml.safe_load(cfgpath.read_text())
    producer=root/'data/calibration/upstream/get_thermal_data.py';funcs=producer_functions(producer);old=funcs['read_kalibr_stereo'](cfg)
    cases=[];controls=[]
    for c in extraction['cases']:
        dataset=output/'datasets'/c['case']
        for name in ['multisense_left_rect_color_camera_info','multisense_right_camera_info','thermal_left_processed_camera_info','thermal_right_processed_camera_info']:
            dest=dataset/'sensors'/name/'data.yaml';dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source/'sensors'/name/'data.yaml',dest)
        for i,side in enumerate(['left','right']):
            dest=dataset/f'sensors/thermal_{side}_processed_camera_info/data.yaml';info=yaml.safe_load(dest.read_text())
            info.update(k=cameras[i][1].tolist(),d=np.r_[cameras[i][2],0.].tolist(),distortion_model='equidistant',r=rect[i].tolist(),p=rect[i+2].tolist())
            dest.write_text(yaml.safe_dump(info,sort_keys=False))
        for position,(row,pair) in enumerate(zip(c['rows'],c['raw'])):
            raw=[cv2.imread(str(workspace_path(pair[s]['path'])),cv2.IMREAD_UNCHANGED) for s in ['left','right']]
            if any(im is None or im.dtype!=np.uint16 or im.shape!=(512,640) for im in raw):raise ValueError('Bad raw encoding')
            replay=funcs['stereo_rectify'](*[im.copy() for im in raw],old)
            for i,side in enumerate(['left','right']):
                rel=row['thermal_'+side]['path'];provided=cv2.imread(str(source/rel),cv2.IMREAD_UNCHANGED)
                control=compare_image(provided,preprocess(replay[i],funcs,cfg['process'],cfg['enhance']))
                controls.append(dict(case=c['case'],position=position,side=side,control=control))
                if not control['exact_full_array_equal']:raise ValueError('Original producer replay mismatch')
                result=preprocess(cv2.remap(raw[i],*maps[i],cv2.INTER_LINEAR),funcs,cfg['process'],cfg['enhance'])
                dest=dataset/rel;dest.parent.mkdir(parents=True,exist_ok=True)
                if not cv2.imwrite(str(dest),np.repeat(result[:,:,None],3,axis=2)):raise RuntimeError('Image write failed')
                rel=row['rgb_'+side]['path'];dest=dataset/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source/rel,dest)
        manifest=dataset/'manifest.json';manifest.write_text(json.dumps({'pairs':c['rows']},indent=2)+'\n')
        cases.append(dict(case=c['case'],dataset=str(dataset),manifest=str(manifest),reference_dataset=str(source),rows=c['rows'],selection=c['selection']))
        print('Prepared',c['case'],len(c['rows']),flush=True)
    paths=[Path(__file__),root/'MAC-VO-main/Scripts/UnitTest/test_td3_long_night.py',root/'MAC-VO-main/Scripts/Setup/td3_long_recording.py',base/'selection.json',base/'image-extraction.json',cfgpath,producer,root/'data/calibration/yamaha.yaml',
        root/'models/macslam/MACT_Offroad.pth',root/'models/macslam/MACVO_FrontendCov.pth',root/'MAC-VO-main/Scripts/Setup/run_td3_execution.py']
    paths.extend(root/'MAC-VO-main/Scripts/Setup'/name for name in ['td3_reference.py','td3_msa_replay.py','td3_uncertainty.py'])
    paths.extend(p for p in (root/'data/calibration/msa_cal').iterdir() if p.is_file())
    paths.extend(p for p in (source/'super_odometry/odometry').iterdir() if p.is_file())
    paths.extend(source/rel for rel in extraction['files'])
    paths.extend(p for p in output.rglob('*') if p.is_file())
    files={str(p):sha(p) for p in paths};verify(files)
    plan=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),base_commit='68517e81f',cases=cases,files_sha256=files,
        original_replay_controls=controls,rectification=[r.tolist() for r in rect[:4]],
        protocol=dict(frames=100,seed=0,mapping=False,optimizer_parallel=False,fp32=True,reference='Same SLAM proxy/nominal body anchor, own rectification',
            scope='Three fixed100-group night blocks excluding EXP-031; original variable timestamp gaps; no physical/reference validation'))
    (output/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')



def select_windows(rows, reference_start, reference_end, excluded, frames=100):
    """Nearest midpoint in each chronological third, with no positive-gap cutoff."""
    if frames < 2 or len(rows) < 3*frames:
        raise ValueError('Insufficient rows for three windows')
    times = {m: np.array([r[m+'_left']['time_ns'] for r in rows], np.int64) for m in ['rgb', 'thermal']}
    bounds = [0, len(rows)//3, 2*len(rows)//3, len(rows)]
    result = []
    for name, lo, hi in zip(['early', 'middle', 'late'], bounds[:-1], bounds[1:]):
        candidates = []
        for start in range(lo, hi-frames+1):
            if any(i in excluded for i in range(start, start+frames)):
                continue
            if any(np.any(t[start:start+frames] < reference_start) or np.any(t[start:start+frames] > reference_end)
                   or np.any(np.diff(t[start:start+frames]) <= 0) for t in times.values()):
                continue
            candidates.append((abs(start+(frames-1)/2-(lo+hi-1)/2), start))
        if not candidates:
            raise ValueError('No eligible window in '+name)
        _, start = min(candidates)
        result.append(dict(name=name, manifest_start=start, frames=frames, region=[lo, hi],
            duration_seconds={m: float((t[start+frames-1]-t[start])/1e9) for m, t in times.items()},
            gaps_ns={m: np.diff(t[start:start+frames]).tolist() for m, t in times.items()}))
    return result


if __name__ == '__main__':
    from .td3_long_recording import execute
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=['prepare','execute'])
    parser.add_argument('--root',type=Path,default=Path('/workspace'));parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--plan',type=Path);parser.add_argument('--case');parser.add_argument('--modality',choices=['rgb','thermal'])
    args=parser.parse_args()
    if args.action=='prepare':prepare(args.root,args.output)
    else:execute(args.plan,args.output,args.case,args.modality)
