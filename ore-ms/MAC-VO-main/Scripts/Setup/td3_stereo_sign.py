"""Audit saved stereo magnitudes and topic order without inferring erased signs."""
import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

from .td3_thermal_provenance import match_stereo, producer_functions, preprocess, compare_image
from .td3_tracking_uncertainty import sha, percentiles
from .td3_night_geometry import verify


def bilinear(image, uv):
    """Finite interior bilinear samples; outside or invalid support yields NaN."""
    image=np.asarray(image,float);uv=np.asarray(uv,float)
    if image.ndim!=2 or uv.ndim!=2 or uv.shape[1]!=2:
        raise ValueError('Expected HxW and Nx2 arrays')
    h,w=image.shape;out=np.full(len(uv),np.nan)
    inbound=np.isfinite(uv).all(axis=1)&(uv[:,0]>=0)&(uv[:,1]>=0)&(uv[:,0]<w-1)&(uv[:,1]<h-1)
    i=np.flatnonzero(inbound);x,y=uv[i].T;ix=x.astype(int);iy=y.astype(int)
    dx=x-ix;dy=y-iy
    values=np.column_stack([image[iy,ix],image[iy,ix+1],image[iy+1,ix],image[iy+1,ix+1]])
    weights=np.column_stack([(1-dx)*(1-dy),dx*(1-dy),(1-dx)*dy,dx*dy])
    out[i]=np.sum(values*weights,axis=1)
    return out


def magnitude_from_depth(depth, fx, baseline):
    depth=np.asarray(depth,float)
    if not np.isfinite(fx) or fx<=0 or not np.isfinite(baseline) or baseline<=0:
        raise ValueError('Expected positive finite execution fx and baseline')
    result=np.full(depth.shape,np.nan);valid=np.isfinite(depth)&(depth>0)
    result[valid]=fx*baseline/depth[valid]
    return result


def projected_disparity(depth,p1,p2):
    """Expected signed left-minus-right displacement for rectified equal-fx P."""
    p1,p2=np.asarray(p1,float),np.asarray(p2,float)
    if p1.shape!=(3,4) or p2.shape!=(3,4) or not np.isclose(p1[0,0],p2[0,0]):
        raise ValueError('Expected rectified equal-fx projections')
    depth=np.asarray(depth,float);out=np.full(depth.shape,np.nan)
    valid=np.isfinite(depth)&(depth>0)
    out[valid]=p1[0,2]-p2[0,2]+(p1[0,3]-p2[0,3])/depth[valid]
    return out


def comparison(observed, model_magnitude, implied_signed, mask):
    valid=mask&np.isfinite(observed)&np.isfinite(model_magnitude)&np.isfinite(implied_signed)
    observed=observed[valid];model=model_magnitude[valid];implied=implied_signed[valid]
    ratio=np.abs(observed)>1
    signs=(np.abs(observed)>1)&(np.abs(implied)>1)
    return dict(count=int(valid.sum()),observed_signed_px=percentiles(observed),
        model_magnitude_px=percentiles(model),absolute_magnitude_error_px=percentiles(np.abs(model-np.abs(observed))),
        ratio_population_count=int(ratio.sum()),model_over_observed_magnitude=percentiles(model[ratio]/np.abs(observed[ratio])),
        positive_observed_fraction=float((observed>0).mean()) if len(observed) else None,
        projection_sign_population_count=int(signs.sum()),
        positive_depth_projection_sign_agreement=float((np.sign(observed[signs])==np.sign(implied[signs])).mean()) if signs.any() else None)


def prepare(extraction,output):
    data=json.loads(extraction.read_text())
    # Staging uses host absolute paths; only this explicit project prefix maps.
    def mapped(v):
        if isinstance(v,str):return v.replace('/root/AirStack/ore-ms/','/workspace/',1)
        if isinstance(v,list):return [mapped(x) for x in v]
        if isinstance(v,dict):return {k:mapped(x) for k,x in v.items()}
        return v
    cases=mapped(data['cases'])
    paths=[extraction,Path(__file__),Path(__file__).parents[1]/'UnitTest/test_td3_stereo_sign.py',
        Path('/workspace/MAC-VO-main/Module/Frontend/Frontend.py'),
        Path('/workspace/MAC-VO-main/Scripts/Setup/run_td3_execution.py'),
        Path('/workspace/MAC-VO-main/Docker/Dockerfile.geometry'),
        Path('/workspace/data/calibration/upstream/get_thermal_data.py'),
        Path('/workspace/data/calibration/upstream/get_thermal_data.yaml')]
    for case in cases:
        paths.extend(Path(case[k]) for k in ['report','config','depth'])
        paths.extend(Path(p) for k in ['images','camera_info'] for p in case[k])
        paths.extend(Path(side[k]) for side in case['raw'].values() for k in ['path','info','camera_info'])
    paths.extend(sorted(Path('/workspace/results/thermal-provenance/run-003').rglob('*')))
    files={str(p.resolve()):sha(p) for p in paths if p.is_file()}
    verify(files);output.mkdir(parents=True,exist_ok=False)
    plan=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),base_commit='75222af3e',cases=cases,files_sha256=files,
        protocol=dict(SIFT_features=2000,mutual_ratio=.7,RANSAC_px=1.5,RANSAC_confidence=.999,seed=0,
            sensitivity_vertical_px=1.5,ratio_min_observed_magnitude_px=1,
            population='Exact first stereo pairs from frozen existing reports; both cameras/modalities, no outcome selection',
            limitation='Saved abs-depth cannot recover model flow sign; no independent calibration or depth truth'))
    (output/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')


def run(output):
    cv2.setNumThreads(1)
    plan=json.loads((output/'plan.json').read_text());verify(plan['files_sha256'])
    funcs=producer_functions('/workspace/data/calibration/upstream/get_thermal_data.py')
    cfg=yaml.safe_load(Path('/workspace/data/calibration/upstream/get_thermal_data.yaml').read_text())
    cal=funcs['read_kalibr_stereo'](cfg)
    records=[];metadata={};(output/'arrays').mkdir()
    for case in plan['cases']:
        tag='-'.join(case[k] for k in ['clip','window','modality'])
        report=json.loads(Path(case['report']).read_text());config=json.loads(Path(case['config']).read_text())
        for i,side in enumerate(['left','right']):
            if not case['images'][i].endswith(report['input_rows'][0][case['modality']+'_'+side]['path']):
                raise ValueError('Saved depth/image input-row mismatch')
        images=[cv2.imread(p,cv2.IMREAD_UNCHANGED) for p in case['images']]
        if any(im is None for im in images):raise ValueError('Missing stereo image')
        infos=[yaml.safe_load(Path(p).read_text()) for p in case['camera_info']]
        p1,p2=[np.array(info['p'],float) for info in infos]
        with np.load(case['depth']) as data:depth=np.asarray(data['depth']).squeeze().copy()
        if depth.shape!=images[0].shape[:2]:raise ValueError('Native depth/image size mismatch')
        geometry,matches=match_stereo(*images,p1,p2)
        keep=matches['inlier'];uv=matches['uv0'][keep];delta=uv-matches['uv1'][keep]
        sampled=bilinear(depth,uv)
        mag=bilinear(magnitude_from_depth(depth,report['K'][0][0],report['execution_baseline_m']),uv)
        implied=bilinear(projected_disparity(depth,p1,p2),uv)
        masks={'all_inliers':np.ones(len(uv),bool),'vertical_le_1.5px':np.abs(delta[:,1])<=1.5}
        row=dict(tag=tag,clip=case['clip'],window=case['window'],modality=case['modality'],
            checkpoint_sha256=report['checkpoint_sha256'],native_depth_shape=list(depth.shape),
            signed_projection_baseline_m=float(-p2[0,3]/p2[0,0]),
            execution_baseline_m=report['execution_baseline_m'],
            enforce_positive_disparity=config['Odometry']['frontend']['args']['enforce_positive_disparity'],
            model_sign='not identifiable from saved absolute depth',geometry=geometry,
            populations={name:comparison(delta[:,0],mag,implied,mask) for name,mask in masks.items()})
        if case['raw']:
            raw=[cv2.imread(case['raw'][s]['path'],cv2.IMREAD_UNCHANGED) for s in ['left','right']]
            if any(im is None or im.dtype!=np.uint16 for im in raw):raise ValueError('Expected raw uint16')
            replay={}
            for label,pair in [('as_labeled',raw),('swapped_raw_diagnostic',raw[::-1])]:
                rect=funcs['stereo_rectify'](*[im.copy() for im in pair],cal)
                replay[label]=[compare_image(provided,preprocess(im,funcs,cfg['process'],cfg['enhance'])) for provided,im in zip(images,rect)]
            row['raw_replay']=replay
            row['raw_timestamp_deltas_ns']={s:case['raw'][s]['delta_ns'] for s in ['left','right']}
            metadata[case['clip']]={s:dict(raw_info=yaml.safe_load(Path(case['raw'][s]['info']).read_text()),
                raw_camera_info=yaml.safe_load(Path(case['raw'][s]['camera_info']).read_text()),processed_camera_info=infos[i]) for i,s in enumerate(['left','right'])}
        np.savez_compressed(output/'arrays'/(tag+'.npz'),**matches,depth_at_inliers=sampled,model_magnitude_at_inliers=mag,
            positive_depth_projection_disparity=implied,observed_inlier_delta=delta,**{'mask_'+k:v for k,v in masks.items()})
        records.append(row);print(tag,geometry['status'],geometry['fundamental_inliers'],flush=True)
    meta_comparison={}
    for side in ['left','right']:
        day,night=metadata['day'][side],metadata['night'][side]
        meta_comparison[side]=dict(raw_info_equal=day['raw_info']==night['raw_info'],
            raw_camera_info_equal=day['raw_camera_info']==night['raw_camera_info'],
            processed_camera_info_equal=day['processed_camera_info']==night['processed_camera_info'],
            raw_frame_labels={'day':day['raw_info'],'night':night['raw_info']},
            projection_max_difference=float(np.max(np.abs(np.array(day['processed_camera_info']['p'])-np.array(night['processed_camera_info']['p'])))))
    analysis=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),base_commit=plan['base_commit'],
        opencv=cv2.__version__,numpy=np.__version__,cases=records,metadata=metadata,metadata_comparison=meta_comparison,
        verified_hashes=verify(plan['files_sha256']))
    (output/'analysis.json').write_text(json.dumps(analysis,indent=2,allow_nan=False)+'\n')
    flat=[]
    for row in records:
        for name,p in row['populations'].items():
            flat.append(dict(case=row['tag'],population=name,count=p['count'],
                observed_disparity_median=p['observed_signed_px']['p10_p50_p90'][1] if p['count'] else None,
                magnitude_error_median=p['absolute_magnitude_error_px']['p10_p50_p90'][1] if p['count'] else None,
                projection_sign_agreement=p['positive_depth_projection_sign_agreement']))
    with (output/'cases.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(flat[0]));writer.writeheader();writer.writerows(flat)
    plot(output,records)


def plot(output,records):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,1,figsize=(11,7),sharex=True)
    x=np.arange(len(records))
    for ax,metric,label in [(axes[0],'observed_signed_px','Observed left-minus-right disparity (px)'),(axes[1],'absolute_magnitude_error_px','Absolute model magnitude error (px)')]:
        for j,pop in enumerate(['all_inliers','vertical_le_1.5px']):
            y=[r['populations'][pop][metric]['p10_p50_p90'][1] if r['populations'][pop]['count'] else np.nan for r in records]
            ax.plot(x+(j-.5)*.12,y,'o',label=pop)
        ax.axhline(0,color='gray',linewidth=.7);ax.set_ylabel(label);ax.grid(alpha=.25)
    axes[0].legend();axes[1].set_xticks(x,[r['tag'] for r in records],rotation=45,ha='right')
    fig.suptitle('Saved depth preserves magnitude; model sign unavailable')
    fig.tight_layout();fig.savefig(output/'stereo-sign.png',dpi=180);fig.savefig(output/'stereo-sign.pdf');plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=['prepare','run'])
    parser.add_argument('--extraction',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.action=='prepare':prepare(args.extraction,args.output)
    else:run(args.output)
