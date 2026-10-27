"""Stage and evaluate one declared recording with frozen pre-acquisition selectors.

Numerical features are prediction-based; support remains reference-selected.
"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
from zoneinfo import ZoneInfo
import cv2
import numpy as np
from audit_ms2_pilot import read_calibration
from ms2_geometry_risk_selection import extract_features,predict,summarize,FAMILIES,sha

FROZEN_SHA256='51b2b9a15d1951c2902e6ef8a0a5e45dcae3d1713d428e36d9545b530e0ae377'


def dump(p,value):p.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def prepare(root,base):
    frozen=base/'frozen-models.json';assert sha(frozen)==FROZEN_SHA256;plan=json.loads(frozen.read_text());seq=plan['sequence'];dataset=base/'dataset'
    assert not (dataset/'source-manifest.json').exists()
    calibration=read_calibration(dataset/'pilot/sync_data'/seq/'calib.npy')
    dump(dataset/'calibration.json',calibration)
    inputs={};times={};metadata={}
    for kind in ['sync_data','proj_depth']:
        for item in json.loads((dataset/(kind+'-pilot-manifest.json')).read_text()):
            path=dataset/item['path'];assert path.stat().st_size==item['bytes'] and sha(path)==item['sha256'];inputs[str(path)]=sha(path)
    for mod in ['rgb','thr']:
        for side in ['left','right']:
            p=dataset/'pilot/sync_data'/seq/mod/f'img_{side}_timestamp.txt';values=list(map(int,p.read_text().split()));assert len(values)>=20 and all(a<b for a,b in zip(values,values[1:]))
            times[mod+'_'+side]=values;metadata[mod+'_'+side]={'rows':len(values),'duration_ns':values[-1]-values[0]}
    records=[]
    for stem in plan['ids']:
        i=int(stem);row={'stem':stem,'cameras':{},'references':{}}
        for mod in ['rgb','thr']:
            k=np.array(calibration[f'K_{mod}L']['values']);rotation=np.array(calibration[f'R_{mod}R']['values']);t=np.array(calibration[f'T_{mod}R']['values']).reshape(3)/1000
            assert np.array_equal(rotation,np.eye(3)) and t[0]<0 and np.array_equal(t[1:],[0,0])
            camera={'K':k.tolist(),'baseline_m':float(-t[0])}
            for side in ['left','right']:
                p=dataset/'pilot/sync_data'/seq/mod/('img_'+side)/(stem+'.png');raw=cv2.imread(str(p),cv2.IMREAD_UNCHANGED)
                assert raw is not None and str(raw.dtype)==('uint8' if mod=='rgb' else 'uint16')
                assert list(raw.shape)==([384,1224,3] if mod=='rgb' else [256,640])
                camera[side]={'path':str(p.relative_to(root)),'sha256':sha(p),'shape':list(raw.shape),'dtype':str(raw.dtype),'recorded_time_ns':times[mod+'_'+side][i]}
            p=dataset/'pilot/proj_depth'/seq/mod/'depth'/(stem+'.png');z=cv2.imread(str(p),cv2.IMREAD_UNCHANGED)
            assert z.dtype==np.uint16 and list(z.shape)==camera['left']['shape'][:2]
            row['cameras'][mod]=camera;row['references'][mod]={'depth':{'path':str(p.relative_to(root)),'sha256':sha(p),'shape':list(z.shape),'dtype':'uint16','meters_per_code':1/256,'zero_invalid':True}}
        records.append(row)
    dump(dataset/'source-manifest.json',{'sequence':seq,'scope':'fixed new-recording first20 supplied pairs; pose/reference physical accuracy not certified','records':records})
    for p in [dataset/'source-manifest.json',dataset/'calibration.json']:inputs[str(p)]=sha(p)
    matches=[];historical=set()
    # Old source manifests cover all four model camera inputs, including right cameras.
    historical_files=[root/'results/ms2-day-replication/source-manifest.json',root/'results/ms2-rain-replication/source-manifest.json',root/'results/ms2-timestamp-semantics/source-protocol-input-manifest.json']
    for p in historical_files:
        old=json.loads(p.read_text())
        for row in old['records']:
            for camera in row['cameras'].values():
                for side in ['left','right']:historical.add(camera[side]['sha256'])
    for row in records:
        for mod in ['rgb','thr']:
            for side in ['left','right']:
                if row['cameras'][mod][side]['sha256'] in historical:matches.append([row['stem'],mod,side])
    assert not matches,'New camera image bytes overlap development'
    dump(base/'source-audit.json',{'status':'PASS','created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'frozen_model_sha256':sha(frozen),'sequence':seq,'full_timestamp_streams':metadata,'source_images':80,'depth_images':40,'new_camera_hash_overlap_with_development':matches,'distinct_development_camera_hashes':len(historical),'frozen_inputs':inputs,'recorded_time_diagnostics':[{'stem':r['stem'],'rgb_right_minus_left_ns':r['cameras']['rgb']['right']['recorded_time_ns']-r['cameras']['rgb']['left']['recorded_time_ns'],'thr_right_minus_left_ns':r['cameras']['thr']['right']['recorded_time_ns']-r['cameras']['thr']['left']['recorded_time_ns'],'thr_minus_rgb_left_ns':r['cameras']['thr']['left']['recorded_time_ns']-r['cameras']['rgb']['left']['recorded_time_ns']} for r in records]})
    # Same integrity-manifest schema as historical runners, built before support/scoring.
    dump(base/'input-integrity.json',{'member_sha256':{'ore-ms/'+str(Path(p).relative_to(root)):h for p,h in inputs.items()}})
    print('Prepared',seq,'80images/40depth, no developmentimage overlap')


def evaluate(root,base,output):
    if output.exists():raise FileExistsError(output)
    frozen=base/'frozen-models.json';assert sha(frozen)==FROZEN_SHA256;plan=json.loads(frozen.read_text());dataset=base/'dataset';metric=base/'metric';support=base/'support'
    sr=json.loads((support/'report.json').read_text());mr=json.loads((metric/'report.json').read_text())
    assert sr['protocol']['sequence']==mr['protocol']['sequence']==plan['sequence']
    hashes={str(frozen):sha(frozen),str(metric/'report.json'):sha(metric/'report.json')};rows=[];output.mkdir(parents=True)
    for hist in mr['rows']:
        stem=hist['stem'];mp=metric/f'metric-{stem}.npz';sp=support/f'pairs-{stem}.npz';hashes[str(mp)]=sha(mp);hashes[str(sp)]=sha(sp)
        with np.load(mp) as f:a={k:f[k] for k in f.files}
        with np.load(sp) as f:p={k:f[k] for k in f.files}
        both=a['both_valid_mask'];n=int(both.sum())
        features=extract_features(a['rgb_predicted_xyz'][both],a['thr_predicted_xyz_in_rgb'][both],a['rgb_metric_variance'][both],a['thr_metric_variance'][both],p['rgb_xy'][both])
        selections={};saved={'features':features}
        for name,model in plan['models'].items():
            margin,choice=predict(features,model);selections[name]=choice;saved[name+'_margin']=margin;saved[name+'_choice']=choice
        scalar=plan['scalar_fit']['policy']
        if scalar['kind']=='always_rgb':sel=np.ones(n)
        elif scalar['kind']=='always_thr':sel=np.zeros(n)
        else:
            r=a['rgb_metric_variance'][both]*10**scalar['log10_c'];t=a['thr_metric_variance'][both]
            sel=(r<t).astype(float)+.5*(r==t)
        fixed=plan['scalar_fit']['training_fixed_policy']['kind']
        selections.update({'original':a['learned_propagated'],'camera_scale':a['camera_scale'],'scalar_fit':sel,'always_rgb':np.ones(n),'always_thr':np.zeros(n),'random':np.full(n,.5),'training_fixed':np.full(n,1. if fixed=='always_rgb' else 0.)})
        row={'stem':stem,'availability':{k:hist[k] for k in ['candidates','both_valid','rgb_only_valid','thr_only_valid','neither_valid']},'rgb_choice_fractions':{k:float(v.mean()) if n else None for k,v in selections.items()},'risks':{}}
        for anchor in ['midpoint','rgb','thr']:
            row['risks'][anchor]={}
            for loss,power in [('mean_l2',1),('mse',2)]:
                r,t=a[f'{anchor}_rgb_error_m']**power,a[f'{anchor}_thr_error_m']**power
                risks={k:float(np.mean(s*r+(1-s)*t)) if n else None for k,s in selections.items()};risks['oracle']=float(np.minimum(r,t).mean()) if n else None;row['risks'][anchor][loss]=risks
        np.savez_compressed(output/f'selection-{stem}.npz',**saved);rows.append(row)
    for path,h in hashes.items():assert sha(path)==h
    dump(output/'protocol.json',{'frozen_model_sha256':sha(frozen),'no_test_fit':True,'script_sha256':sha(__file__),'frozen_input_sha256':hashes})
    available=[r for r in rows if r['availability']['both_valid']>0]
    dump(output/'report.json',{'sequence':plan['sequence'],'primary':'combined','rows':rows,'available_frames':len(available),'unavailable_stems':[r['stem'] for r in rows if not r['availability']['both_valid']],'summary':summarize(available) if available else None})
    print('Frozen evaluation complete; no fitting/normalizer updates',plan['sequence'])


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['prepare','evaluate']);p.add_argument('--root',type=Path,required=True);p.add_argument('--base',type=Path,required=True);p.add_argument('--output',type=Path)
    a=p.parse_args()
    if a.stage=='prepare':prepare(a.root.resolve(),a.base.resolve())
    else:evaluate(a.root.resolve(),a.base.resolve(),a.output.resolve())
