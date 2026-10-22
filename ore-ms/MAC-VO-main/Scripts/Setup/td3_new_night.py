"""Fixed chronological-window RGB/MSA-thermal validation on a new night recording."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil

import cv2
import numpy as np
import yaml

from .td3_msa_replay import msa_rectification, workspace_path
from .td3_thermal_provenance import producer_functions, preprocess, compare_image, match_stereo
from .td3_tracking_uncertainty import sha, percentiles
from .td3_night_geometry import verify


def prepare(root, output):
    output.mkdir(parents=True,exist_ok=False);cv2.setNumThreads(1)
    base=root/'results/new-night-validation';source=root/'data/tartandrive3/20251027/night_mac_salon_warehousefig8'
    extraction=json.loads((base/'image-extraction.json').read_text())
    for rel,rec in extraction['files'].items():
        if sha(source/rel)!=rec['sha256']:raise ValueError('Extracted image changed')
    preflight=json.loads((base/'preflight.json').read_text())
    if not all(c['equal_to_previous_day'] for c in preflight['calibration_checks']):raise ValueError('New camera metadata differs; review applicability before inference')
    cameras,t,rect,maps=msa_rectification(root/'data/calibration/msa_cal')
    config_path=root/'data/calibration/upstream/get_thermal_data.yaml';cfg=yaml.safe_load(config_path.read_text())
    producer=root/'data/calibration/upstream/get_thermal_data.py';funcs=producer_functions(producer);old=funcs['read_kalibr_stereo'](cfg)
    controls=[];cases=[];geometry=[]
    for c in extraction['cases']:
        dataset=output/'datasets'/c['case']
        for name in ['multisense_left_rect_color_camera_info','multisense_right_camera_info','thermal_left_processed_camera_info','thermal_right_processed_camera_info']:
            path=dataset/'sensors'/name/'data.yaml';path.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source/'sensors'/name/'data.yaml',path)
        for i,side in enumerate(['left','right']):
            path=dataset/f'sensors/thermal_{side}_processed_camera_info/data.yaml';info=yaml.safe_load(path.read_text())
            info.update(k=cameras[i][1].tolist(),d=np.r_[cameras[i][2],0.].tolist(),distortion_model='equidistant',r=rect[i].tolist(),p=rect[i+2].tolist())
            path.write_text(yaml.safe_dump(info,sort_keys=False))
        for position,(row,raw_pair) in enumerate(zip(c['rows'],c['raw'])):
            raw=[cv2.imread(str(workspace_path(raw_pair[s]['path'])),cv2.IMREAD_UNCHANGED) for s in ['left','right']]
            if any(im is None or im.dtype!=np.uint16 or im.shape!=(512,640) for im in raw):raise ValueError('Unexpected raw encoding')
            old_rect=funcs['stereo_rectify'](*[im.copy() for im in raw],old)
            for i,side in enumerate(['left','right']):
                rel=row['thermal_'+side]['path'];provided=cv2.imread(str(source/rel),cv2.IMREAD_UNCHANGED)
                control=compare_image(provided,preprocess(old_rect[i],funcs,cfg['process'],cfg['enhance']))
                controls.append(dict(case=c['case'],position=position,side=side,control=control))
                if not control['exact_full_array_equal']:raise ValueError('Old producer replay mismatch')
                processed=preprocess(cv2.remap(raw[i],*maps[i],cv2.INTER_LINEAR),funcs,cfg['process'],cfg['enhance'])
                target=dataset/rel;target.parent.mkdir(parents=True,exist_ok=True)
                if not cv2.imwrite(str(target),np.repeat(processed[:,:,None],3,axis=2)):raise RuntimeError('Image write failed')
                rgb=row['rgb_'+side]['path'];target=dataset/rgb;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source/rgb,target)
            if position==0:
                for modality,names in [('rgb',['multisense_left_rect_color_camera_info','multisense_right_camera_info']),('thermal',['thermal_left_processed_camera_info','thermal_right_processed_camera_info'])]:
                    images=[cv2.imread(str(dataset/row[modality+'_'+s]['path']),cv2.IMREAD_UNCHANGED) for s in ['left','right']]
                    projections=[np.array(yaml.safe_load((dataset/'sensors'/name/'data.yaml').read_text())['p']) for name in names]
                    metrics,arrays=match_stereo(*images,*projections);geometry.append(dict(case=c['case'],modality=modality,**metrics))
                    dest=output/'stereo';dest.mkdir(exist_ok=True);np.savez_compressed(dest/f'{c["case"]}-{modality}.npz',**arrays)
                    preview=np.hstack([cv2.cvtColor(im,cv2.COLOR_GRAY2BGR) if im.ndim==2 else im for im in images])
                    for y in range(16,preview.shape[0],32):cv2.line(preview,(0,y),(preview.shape[1]-1,y),(0,200,255),1)
                    cv2.imwrite(str(dest/f'{c["case"]}-{modality}.png'),preview)
        manifest=dataset/'manifest.json';manifest.write_text(json.dumps({'pairs':c['rows']},indent=2)+'\n')
        cases.append(dict(case=c['case'],dataset=str(dataset),manifest=str(manifest),reference_dataset=str(source),rows=c['rows'],selection=c['selection']))
        print('Prepared',c['case'],flush=True)
    paths=[Path(__file__),base/'preflight.json',base/'paired-manifest.json',base/'image-extraction.json',base/'metadata-extraction.json',producer,config_path,
        root/'data/calibration/yamaha.yaml',root/'models/macslam/MACT_Offroad.pth',root/'models/macslam/MACVO_FrontendCov.pth',
        root/'MAC-VO-main/Scripts/Setup/run_td3_execution.py',root/'MAC-VO-main/Scripts/Setup/td3_msa_replay.py',
        root/'MAC-VO-main/Scripts/Setup/td3_additional_recording.py',
        root/'MAC-VO-main/Scripts/UnitTest/test_td3_new_night.py']
    paths.extend(p for p in (root/'data/calibration/msa_cal').iterdir() if p.is_file())
    paths.extend(p for p in source.rglob('*') if p.is_file());paths.extend(p for p in output.rglob('*') if p.is_file())
    files={str(p):sha(p) for p in paths};verify(files)
    plan=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),base_commit='859a58b44',cases=cases,files_sha256=files,
        old_producer_controls=controls,geometry=geometry,T_right_from_left=t.tolist(),rectification=[r.tolist() for r in rect[:4]],
        protocol=dict(seed=0,frames=20,mapping=False,optimizer_parallel=False,fp32=True,reference='Recorded SLAM proxy, original nominal body anchor; MSA thermal rectification',
            selection='First eligible20 frames per chronological third; fixed before images/outcomes',scope='Separate nonoverlapping same-date capture, no calibrated fusion or independent accuracy claim'))
    (output/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')



def first_windows(times, lower, upper, frames=20, max_gap_ns=150000000):
    """First eligible block in each chronological third; never use image outcomes."""
    count = len(times[0])
    if len(times) != 2 or len(times[1]) != count:
        raise ValueError('Two aligned modality timelines required')
    valid = np.ones(count, bool)
    for ts in times:
        valid &= (ts >= lower) & (ts <= upper)
    bounds = [0, count//3, 2*count//3, count]
    windows = []
    for name, lo, hi in zip(['early', 'middle', 'late'], bounds[:-1], bounds[1:]):
        for start in range(lo, hi-frames+1):
            gaps = [np.diff(ts[start:start+frames]) for ts in times]
            if valid[start:start+frames].all() and all(np.all((g > 0) & (g <= max_gap_ns)) for g in gaps):
                windows.append(dict(name=name, manifest_start=start, frames=frames, region=[lo, hi]))
                break
        else:
            raise ValueError('No eligible window in '+name)
    return windows


if __name__ == '__main__':
    from .td3_additional_recording import execute
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=['prepare','execute'])
    parser.add_argument('--root',type=Path,default=Path('/workspace'));parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--plan',type=Path);parser.add_argument('--case');parser.add_argument('--modality',choices=['rgb','thermal'])
    args=parser.parse_args()
    if args.action=='prepare':prepare(args.root,args.output)
    else:execute(args.plan,args.output,args.case,args.modality)
