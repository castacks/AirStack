"""Independent image-flow diagnostic on fixed saved thermal predictions."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import cv2
import numpy as np

from .td3_tracking_uncertainty import sha,percentiles,retained_mask
from .td3_night_geometry import verify
from .td3_reference import matrices
from .td3_reprojection import pair_geometry


def track_masks(uv, endpoint, backward, status, reverse_status, width, height):
    finite=np.isfinite(endpoint).all(axis=1)&np.isfinite(backward).all(axis=1)
    fb=np.linalg.norm(backward-uv,axis=1)
    inbound=(endpoint[:,0]>=32)&(endpoint[:,0]<width-32)&(endpoint[:,1]>=32)&(endpoint[:,1]<height-32)
    base=np.asarray(status,bool)&np.asarray(reverse_status,bool)&finite&inbound
    return base&(fb<=1.),base&(fb<=.5),fb,base


def lk_track(first,second,uv):
    uv=np.asarray(uv,np.float32).reshape(-1,2)
    if len(uv)==0:return np.empty((0,2)),np.empty((0,2)),np.empty(0,bool),np.empty(0,bool),np.empty(0),np.empty(0)
    kwargs=dict(winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,30,.01),flags=0,minEigThreshold=1e-4)
    endpoint,status,error=cv2.calcOpticalFlowPyrLK(first,second,uv[:,None],None,**kwargs)
    if endpoint is None:return np.full_like(uv,np.nan),np.full_like(uv,np.nan),np.zeros(len(uv),bool),np.zeros(len(uv),bool),np.full(len(uv),np.nan),np.full(len(uv),np.nan)
    safe=endpoint.copy();safe[~np.isfinite(safe)]=0
    backward,reverse,reverse_error=cv2.calcOpticalFlowPyrLK(second,first,safe,None,**kwargs)
    if backward is None:backward=np.full_like(endpoint,np.nan);reverse=np.zeros_like(status);reverse_error=np.full_like(error,np.nan)
    return endpoint[:,0],backward[:,0],status.ravel().astype(bool),reverse.ravel().astype(bool),error.ravel(),reverse_error.ravel()


def prepare(root,output):
    output.mkdir(parents=True,exist_ok=False)
    replay=root/'results/msa-replay';old=json.loads((replay/'prepared-001/plan.json').read_text())
    long=root/'results/long-salon';longplan=json.loads((long/'prepared-001/plan.json').read_text())
    inputs=[(name,next(c for c in old['cases'] if c['case']==name),replay/'vo-001'/name/'thermal',replay/f'{name}-msa-proxy-reference.npz','T_world_camera_ned') for name in ['night-early','night-middle','night-late','day-late']]
    c=next(c for c in longplan['cases'] if c['case']=='early')
    inputs.append(('day-static',c,long/'vo-001/early/thermal',long/'early-thermal-proxy.npz','reference'))
    files={};cases=[]
    for name,c,directory,proxy,key in inputs:
        report=json.loads((directory/'report.json').read_text());rows=report['input_rows'][:20]
        images=[str(Path(c['dataset'])/r['thermal_left']['path']) for r in rows]
        pairs=[str(directory/'uncertainty'/f'pair-{i:03d}.npz') for i in range(19)]
        cases.append(dict(case=name,dataset=c['dataset'],directory=str(directory),proxy=str(proxy),proxy_key=key,
            rows=rows,images=images,pairs=pairs))
        for p in [directory/'report.json',directory/'effective-config.json',directory/'trajectory.npz',proxy,*map(Path,images),*map(Path,pairs)]:files[str(p)]=sha(p)
    paths=[Path(__file__),Path(__file__).parents[1]/'UnitTest/test_td3_low_motion.py',Path(__file__).parent/'td3_reprojection.py',
        Path(__file__).parent/'td3_tracking_uncertainty.py',Path(__file__).parent/'td3_reference.py',
        replay/'prepared-001/plan.json',long/'prepared-001/plan.json']
    files.update({str(p):sha(p) for p in paths});verify(files)
    plan=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),base_commit='75b8defff',cases=cases,files_sha256=files,
        protocol=dict(window_pairs=19,grid_points=1008,lk_window=[21,21],lk_levels=3,lk_iterations=30,lk_epsilon=.01,
            lk_min_eigenvalue=1e-4,fb_primary_px=1.,fb_sensitivity_px=.5,texture_std_sensitivity=5.,border=32,
            scope='Independent algorithmic image tracking, no ground truth or causal/depth-scale/pose fit'))
    (output/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')


def run(output):
    plan=json.loads((output/'plan.json').read_text());verify(plan['files_sha256']);cv2.setNumThreads(1);cv2.setRNGSeed(0)
    records=[];cases=[]
    metrics=['model_flow_px','lk_flow_px','model_lk_error_px','model_proxy_error_px','lk_proxy_error_px',
        'model_vo_error_px','lk_vo_error_px','proxy_translation_contribution_px','proxy_rotation_flow_px','trace','depth0','source_std','fb_px']
    for case in plan['cases']:
        directory=Path(case['directory']);report=json.loads((directory/'report.json').read_text())
        with np.load(directory/'trajectory.npz') as t:estimated=matrices(t['poses_camera_ned'][:20]);times=t['time_ns'].reshape(-1)[:20]
        with np.load(case['proxy']) as p:camera=p[case['proxy_key']][:20];proxy_times=p['time_ns'].reshape(-1)[:20]
        assert np.array_equal(times,proxy_times) and np.array_equal(times,[r['thermal_left']['time_ns'] for r in case['rows']])
        local=[];dest=output/'points'/case['case'];dest.mkdir(parents=True)
        for i,path in enumerate(case['pairs']):
            images=[cv2.imread(case['images'][j],cv2.IMREAD_GRAYSCALE) for j in [i,i+1]]
            if any(im is None or im.shape!=(512,640) or im.dtype!=np.uint8 for im in images):raise ValueError('Unexpected processed thermal image')
            relative=np.linalg.inv(camera[i+1])@camera[i];vo=np.linalg.inv(estimated[i+1])@estimated[i]
            with np.load(path) as p:
                assert int(p['time0_ns'])==int(times[i]);uv=p['grid_uv0'];model=p['grid_uv1'];depth=p['grid_depth0'];k=p['K']
                end,back,status,reverse,err,rerr=lk_track(*images,uv)
                valid,tight,fb,base=track_masks(uv,end,back,status,reverse,640,512)
                first=images[0].astype(np.float64);mean=cv2.boxFilter(first,-1,(21,21),normalize=True)
                std=np.sqrt(np.maximum(cv2.boxFilter(first*first,-1,(21,21),normalize=True)-mean*mean,0))
                texture=std[uv[:,1].astype(int),uv[:,0].astype(int)]
                geo=pair_geometry(uv,model,depth,p['grid_depth1'],k,relative);vgeo=pair_geometry(uv,model,depth,p['grid_depth1'],k,vo)
                expected=model-geo['forward_error_px'];rotation=model-geo['rotation_only_error_px'];vo_expected=model-vgeo['forward_error_px']
                masks={'all':np.ones(len(uv),bool),'lk_valid':valid,'lk_fb_le_0.5':tight,'lk_texture_std_ge5':valid&(texture>=5)}
                for j in range(3):masks[f'lk_image_third_{j}']=valid&(uv[:,1]>=j*512/3)&(uv[:,1]<(j+1)*512/3)
                arrays=dict(uv0=uv,model_endpoint=model,lk_endpoint=end,lk_backward=back,lk_status=status,lk_reverse_status=reverse,
                    lk_error=err,lk_reverse_error=rerr,fb_px=fb,lk_base=base,source_std=texture,depth0=depth,
                    model_flow_px=np.linalg.norm(model-uv,axis=1),lk_flow_px=np.linalg.norm(end-uv,axis=1),
                    model_lk_error_px=np.linalg.norm(model-end,axis=1),model_proxy_error_px=np.linalg.norm(model-expected,axis=1),
                    lk_proxy_error_px=np.linalg.norm(end-expected,axis=1),model_vo_error_px=np.linalg.norm(model-vo_expected,axis=1),
                    lk_vo_error_px=np.linalg.norm(end-vo_expected,axis=1),proxy_translation_contribution_px=np.linalg.norm(expected-rotation,axis=1),
                    proxy_rotation_flow_px=np.linalg.norm(rotation-uv,axis=1),proxy_endpoint=expected,proxy_rotation_endpoint=rotation,
                    vo_endpoint=vo_expected,trace=p['grid_raw_flow_cov'][:,:2].sum(axis=1),K=k,relative_proxy=relative,relative_vo=vo,
                    time0_ns=times[i],time1_ns=times[i+1])
                keep=retained_mask(p['selected_inbound'],p['outlier_keep_inbound'])
                np.savez_compressed(dest/f'{i:03d}.npz',**arrays,**{'mask_'+key:value for key,value in masks.items()})
                for group,mask in masks.items():
                    row=dict(case=case['case'],pair=i,population=group,count=int(mask.sum()),grid_count=len(uv),retained=int(keep.sum()),
                        image_mae=float(np.mean(np.abs(images[0].astype(float)-images[1]))),**{key:percentiles(arrays[key][mask]) for key in metrics})
                    records.append(row);local.append(row)
        for group in masks:
            rows=[r for r in local if r['population']==group];summary=dict(case=case['case'],population=group,pairs=len(rows),total_points=sum(r['count'] for r in rows),
                pair_count_min=min(r['count'] for r in rows),pair_count_max=max(r['count'] for r in rows))
            for key in metrics:
                values=[r[key]['p10_p50_p90'][1] for r in rows if r[key]['finite_count']]
                summary[key+'_pair_median']=float(np.median(values)) if values else None
            summary['pair_retained_min']=min(r['retained'] for r in rows);summary['pair_retained_max']=max(r['retained'] for r in rows)
            cases.append(summary)
        print(case['case'],'19 pairs',flush=True)
    a=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),base_commit=plan['base_commit'],opencv=cv2.__version__,
        cases=cases,pairs=records,verified_hashes=verify(plan['files_sha256']),source_pair_files=95,
        scope='Image tracking and pose/depth-conditioned consistency only; no independent GT, scale fitting, inference or default changes')
    (output/'analysis.json').write_text(json.dumps(a,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['prepare','run'])
    p.add_argument('--root',type=Path,default=Path('/workspace'));p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.action=='prepare':prepare(args.root,args.output)
    else:run(args.output)
