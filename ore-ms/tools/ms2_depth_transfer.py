"""Fixed60-frame, nominal-source-convention depth-structure transfer test."""
import argparse
import ast
import hashlib
import json
import os
from pathlib import Path
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
AUTHOR=Path(os.environ.get('THERMAL_SFM_AUTHOR','/author'))
ASSETS=Path(os.environ.get('THERMAL_SFM_ASSETS','/assets'))
DATA=ROOT/'results/stereo-mean-adaptation/run-001/dataset.json'
OUT=ROOT/'results/ms2-depth-transfer/run-001'
PLAN=ROOT/'experiments/ms2-depth-transfer-freeze-2026-11-12.json'
CASES=('day','night','rain')


def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()


def write(p,j):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
    with p.open('x') as f:json.dump(j,f,indent=2,allow_nan=False)


def verify():
    roots={'workspace':ROOT,'author':AUTHOR,'assets':ASSETS}
    plan=json.loads(PLAN.read_text())
    for n,d in plan['pins'].items():
        kind,path=n.split('/',1);assert sha(roots[kind]/path)==d,n


def scaled_metrics(gt,baseline,candidate):
    """Identical medians/support; finite/positive coverage costs remain explicit."""
    if gt.shape!=baseline.shape or gt.shape!=candidate.shape:raise ValueError('equal native grids required')
    reference=np.isfinite(gt)&(gt>1e-3)&(gt<80)
    valid_b=reference&np.isfinite(baseline)&(baseline>0)
    valid_c=reference&np.isfinite(candidate)&(candidate>0)
    common=valid_b&valid_c
    report={'reference_pixels':int(reference.sum()),'baseline_pixels':int(valid_b.sum()),
        'candidate_pixels':int(valid_c.sum()),'common_pixels':int(common.sum()),
        'candidate_preserves_baseline_support':not bool(np.any(valid_b&~valid_c))}
    if not common.any():return {**report,'metrics':None}
    target=gt[common].astype(np.float64)
    metrics={}
    for name,arr in [('mact',baseline),('mono',candidate)]:
        raw=arr[common].astype(np.float64)
        scale=float(np.median(target)/np.median(raw));values=raw*scale
        difference=values-target
        metrics[name]={'scale':scale,'abs_rel':float(np.mean(np.abs(difference)/target)),
            'rmse':float(np.sqrt(np.mean(difference**2)))}
    raw=baseline[common].astype(np.float64)-target
    metrics['unscaled_mact']={'abs_rel':float(np.mean(np.abs(raw)/target)),
                              'rmse':float(np.sqrt(np.mean(raw**2)))}
    return {**report,'metrics':metrics}


def decision(rows):
    usable=[r for r in rows if r['metrics'] is not None]
    means={name:{k:float(np.mean([r['metrics'][name][k] for r in usable])) if usable else None
                 for k in ['abs_rel','rmse']} for name in ['mact','mono','unscaled_mact']}
    checks={'all20frames_scored':len(usable)==len(rows)==20,
            'no_support_loss':all(r['candidate_preserves_baseline_support'] for r in rows)}
    for k in ['abs_rel','rmse']:
        checks[k+'_improves']=bool(usable) and means['mono'][k]<means['mact'][k]
    return {'checks':checks,'pass':all(checks.values()),'equal_frame_metrics':means}


def predict():
    verify()
    from types import SimpleNamespace
    import torch
    from skimage.transform import resize
    from imageio.v2 import imread
    from thermal_sfm_reproduction import compatibility
    compatibility()
    sys.path.insert(0,str(AUTHOR/'common'))
    import models
    mono=models.DispResNet(18,False,num_channel=1).cuda()
    mono.load_state_dict(torch.load(ASSETS/'dispnet_disp_model_best.pth.tar',map_location='cpu')['state_dict'],strict=True)
    mono.eval()
    # Exact author scalar/input function bodies, with no source file mutations.
    scope={'np':np,'torch':torch,'imresize':resize,'imread':imread,'device':torch.device('cuda')}
    for path,name in [(AUTHOR/'common/utils/custom_transforms.py','Celsius2Raw'),(AUTHOR/'test_disp.py','load_tensor_Timage_outdoor')]:
        tree=ast.parse(path.read_text());fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name==name)
        exec(compile(ast.Module(body=[fn],type_ignores=[]),str(path),'exec'),scope)
    sys.path.insert(0,str(ROOT/'MAC-VO-main'))
    from Module.Frontend.Frontend import FlowFormerCovFrontend
    from Utility.Config import asNamespace
    from DataLoader.Dataset.MS2 import MS2ManifestSequence,THERMAL_RECIPE
    cfg=json.loads((ROOT/'results/radiometric-native/run-001/day/original/thermal/effective-config.json').read_text())['Odometry']['frontend']['args']
    cfg['weight']=str(ROOT/'models/macslam/MACT_Offroad.pth')
    frontend=FlowFormerCovFrontend(asNamespace(cfg))
    data=json.loads(DATA.read_text())
    for record in data['records']:
        condition=record['condition'];dest=OUT/'predictions'/condition;dest.mkdir(parents=True,exist_ok=False)
        source=json.loads((ROOT/'results/unweighted-longer/run-001/inputs'/condition/'source-manifest.json').read_text())
        for i,row in enumerate(record['rows']):
            assert row['source_id']==f'{i:06d}'
            for side in ['left','right']:assert row[side]['sha256']==source['records'][i]['cameras']['thr'][side]['sha256']
        sequence=MS2ManifestSequence(ROOT,ROOT/'results/unweighted-longer/run-001/inputs'/condition/'source-manifest.json','thr',thermal_transform=THERMAL_RECIPE)
        with torch.inference_mode():
            for i,row in enumerate(record['rows']):
                raw_input=scope['load_tensor_Timage_outdoor'](str(ROOT/row['left']['path']),SimpleNamespace(img_height=256,img_width=320))
                monocular=1/mono(raw_input)[0,0].cpu().numpy()
                mact=frontend.estimate_depth(sequence[i].stereo).depth[0,0].cpu().numpy()
                assert monocular.shape==(256,320) and mact.shape==(256,640)
                with (dest/(row['source_id']+'.npz')).open('xb') as f:np.savez_compressed(f,mono_native=monocular,mact_native=mact)
                print(condition,i+1,'/20',flush=True)
    verify()
    pins={str(p.relative_to(OUT)):sha(p) for p in (OUT/'predictions').rglob('*.npz')}
    assert len(pins)==60
    write(OUT/'prediction-lock.json',{'pins':pins,'freeze_sha256':sha(PLAN),'ground_truth_values_read_for_prediction':False})


def score():
    verify()
    import cv2
    lock=json.loads((OUT/'prediction-lock.json').read_text());assert lock['freeze_sha256']==sha(PLAN)
    for n,d in lock['pins'].items():assert sha(OUT/n)==d
    reports={}
    for record in json.loads(DATA.read_text())['records']:
        c=record['condition'];rows=[]
        for row in record['rows']:
            gt=cv2.imread(str(ROOT/row['depth']['path']),cv2.IMREAD_UNCHANGED).astype(np.float64)/256
            with np.load(OUT/'predictions'/c/(row['source_id']+'.npz')) as z:
                # Preserve author inverse-depth resize convention, fixed epsilon.
                candidate=1/(cv2.resize(1/(z['mono_native'].astype(np.float64)+1e-6),(640,256))+1e-6)
                baseline=z['mact_native'].astype(np.float64)
            rows.append({'source_id':row['source_id'],**scaled_metrics(gt,baseline,candidate)})
        report={'rows':rows,**decision(rows)};reports[c]=report
        write(OUT/(c+'-scores.json'),report)
    write(OUT/'summary.json',{'records':reports,'decision':'ADVANCE_TO_SEPARATE_SCALE_AND_VO_CONTRACT' if all(r['pass'] for r in reports.values()) else 'CLOSE_THIS_CHECKPOINT_TRANSFER_CONFIGURATION_NO_RETUNING',
        'scope':'Both models individually median-scaled on identical reference-supported pixels; depth structure only; exposed development data'})


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('action',choices=['predict','score']);args=parser.parse_args()
    {'predict':predict,'score':score}[args.action]()
