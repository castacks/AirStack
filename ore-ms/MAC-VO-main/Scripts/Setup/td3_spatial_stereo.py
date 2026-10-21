"""Frozen spatial diagnostics on saved stereo flow; no independent truth claim."""
import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial import cKDTree
import yaml

from .td3_stereo_sign import bilinear, projected_disparity
from .td3_tracking_uncertainty import sha, percentiles
from .td3_night_geometry import verify


def grid(height, width):
    y,x=np.mgrid[32:height-32:16,32:width-32:16]
    return np.column_stack([x.ravel(),y.ravel()]).astype(float)


def patch_samples(image, uv):
    y,x=np.mgrid[-4:5,-4:5]
    offsets=np.column_stack([x.ravel(),y.ravel()])
    return bilinear(image,(uv[:,None,:]+offsets).reshape(-1,2)).reshape(len(uv),81)


def patch_metrics(left, right, uv, flow):
    a=patch_samples(left,uv);b=patch_samples(right,uv+flow)
    sa=a.std(axis=1);sb=b.std(axis=1)
    inbound=np.isfinite(a).all(axis=1)&np.isfinite(b).all(axis=1)
    valid=inbound&(sa>1)&(sb>1)
    ncc=np.full(len(uv),np.nan)
    ac=a[valid]-a[valid].mean(axis=1,keepdims=True)
    bc=b[valid]-b[valid].mean(axis=1,keepdims=True)
    ncc[valid]=(ac*bc).mean(axis=1)/(sa[valid]*sb[valid])
    return dict(source_std=sa,target_std=sb,patch_inbound=inbound,ncc_valid=valid,ncc=ncc)


def measure(left,right,flow,covariance,depth,p1,p2,uv,features):
    h,w=left.shape
    sampled=np.column_stack([bilinear(channel,uv) for channel in flow])
    endpoint=uv+sampled
    distance=cKDTree(features).query(uv)[0] if len(features) else np.full(len(uv),np.inf)
    expected=bilinear(projected_disparity(depth,p1,p2),uv)
    disparity=-sampled[:,0]
    eligible=(np.abs(disparity)>1)&(np.abs(expected)>1)&np.isfinite(disparity)&np.isfinite(expected)
    return dict(uv=uv,flow=sampled,disparity=disparity,vertical_abs=np.abs(sampled[:,1]),
        covariance_trace=bilinear(covariance.sum(axis=0),uv),
        endpoint_inbound=np.isfinite(endpoint).all(axis=1)&(endpoint[:,0]>=0)&(endpoint[:,0]<=w-1)&(endpoint[:,1]>=0)&(endpoint[:,1]<=h-1),
        nearest_inlier_distance=distance,near_inlier=distance<=16,
        projection_eligible=eligible,projection_sign_agree=eligible&(np.sign(disparity)==np.sign(expected)),
        **patch_metrics(left,right,uv,sampled))


def groups(data,height,width):
    n=len(data['uv']);uv=data['uv'];std=data['source_std']
    y=np.floor(uv[:,1]*3/height).astype(int);x=np.floor(uv[:,0]*3/width).astype(int)
    result=dict(all=np.ones(n,bool),near_inlier=data['near_inlier'],no_nearby_inlier=~data['near_inlier'],
        endpoint_inbound=data['endpoint_inbound'],vertical_le_1_5=data['vertical_abs']<=1.5,
        inbound_and_vertical=data['endpoint_inbound']&(data['vertical_abs']<=1.5),ncc_defined=data['ncc_valid'],
        texture_le1=np.isfinite(std)&(std<=1),texture_1_to5=(std>1)&(std<=5),texture_gt5=np.isfinite(std)&(std>5),texture_invalid=~np.isfinite(std))
    result.update({f'row_third_{i}':y==i for i in range(3)})
    result.update({f'tile_{i}_{j}':(y==i)&(x==j) for i in range(3) for j in range(3)})
    for family in [('near_inlier','no_nearby_inlier'),('texture_le1','texture_1_to5','texture_gt5','texture_invalid'),tuple(f'row_third_{i}' for i in range(3)),tuple(f'tile_{i}_{j}' for i in range(3) for j in range(3))]:
        if not np.all(np.sum([result[k] for k in family],axis=0)==1):raise ValueError('Nonpartitioning groups')
    return result


def summarize(data,mask):
    disparity=data['disparity'][mask];vertical=data['vertical_abs'][mask]
    finite=np.isfinite(disparity)&np.isfinite(vertical);n=int(mask.sum());den=int(finite.sum())
    eligible=mask&data['projection_eligible']
    return dict(count=n,finite_count=den,
        positive_fraction=float((disparity[finite]>0).mean()) if den else None,
        positive_gt1_count=int((disparity[finite]>1).sum()),negative_lt_minus1_count=int((disparity[finite]<-1).sum()),
        nearzero_le1_count=int((np.abs(disparity[finite])<=1).sum()),
        endpoint_inbound_count=int(data['endpoint_inbound'][mask].sum()),
        vertical_le1_5_count=int((vertical[finite]<=1.5).sum()),ncc_defined_count=int(data['ncc_valid'][mask].sum()),
        projection_eligible_count=int(eligible.sum()),projection_sign_agreement=float(data['projection_sign_agree'][eligible].mean()) if eligible.any() else None,
        **{key:percentiles(data[key][mask]) for key in ['disparity','vertical_abs','source_std','target_std','ncc','covariance_trace']})


def prepare(source,output):
    inherited=json.loads((source/'plan.json').read_text());files=dict(inherited['files_sha256']);verify(files)
    paths=[source/'plan.json',source/'analysis.json',Path(__file__),
        Path(__file__).parents[1]/'UnitTest/test_td3_spatial_stereo.py',Path('/workspace/MAC-VO-main/Docker/Dockerfile.geometry')]
    paths.extend((source/'arrays').glob('*.npz'))
    files.update({str(p.resolve()):sha(p) for p in paths});verify(files)
    output.mkdir(parents=True,exist_ok=False)
    plan=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),base_commit='618f28af3',source=str(source),
        feature_source=inherited['source'],cases=inherited['cases'],files_sha256=files,
        protocol=dict(grid_border_px=32,grid_stride_px=16,patch_width_px=9,patch_std_min_exclusive=1,
            texture_std_bins=[1,5],near_inlier_radius_px=16,vertical_sensitivity_px=1.5,sign_magnitude_px=1,
            spatial='Equal image row thirds,3x3 tiles,each sampled grid row',
            inference='None; use only existing 12 signed stereo cases',
            scope='Model-conditioned spatial/image consistency, not independent geometric accuracy or confidence calibration'))
    (output/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')


def run(output):
    cv2.setNumThreads(1)
    plan=json.loads((output/'plan.json').read_text());verify(plan['files_sha256'])
    source=Path(plan['source']);features=Path(plan['feature_source'])
    signed=json.loads((source/'analysis.json').read_text())
    if any(r['status']!='PASS' for r in signed['cases']):raise ValueError('Signed depth controls must pass first')
    records=[];dense=[];(output/'points').mkdir()
    for case in plan['cases']:
        tag='-'.join(case[k] for k in ['clip','window','modality'])
        images=[cv2.imread(p,cv2.IMREAD_GRAYSCALE) for p in case['images']]
        if any(im is None for im in images):raise ValueError('Missing original image')
        left,right=images;h,w=left.shape
        infos=[yaml.safe_load(Path(p).read_text()) for p in case['camera_info']]
        p1,p2=[np.asarray(info['p'],float) for info in infos]
        with np.load(source/'arrays'/(tag+'.npz')) as data:
            flow=data['flow'].copy();covariance=data['covariance'].copy();depth=data['depth'].squeeze().copy()
        if flow.shape!=(2,h,w) or covariance.shape!=flow.shape or depth.shape!=(h,w):raise ValueError('Native shapes disagree')
        with np.load(features/'arrays'/(tag+'.npz')) as data:feature_uv=data['uv0'][data['inlier']].copy()
        for population,uv in [('grid',grid(h,w)),('feature',feature_uv)]:
            data=measure(left,right,flow,covariance,depth,p1,p2,uv,feature_uv)
            masks=groups(data,h,w)
            if population=='grid':
                for i,y in enumerate(np.unique(uv[:,1])):masks[f'grid_row_{i:02d}']=uv[:,1]==y
            np.savez_compressed(output/'points'/(tag+'-'+population+'.npz'),**data,**{'mask_'+k:v for k,v in masks.items()})
            for name,mask in masks.items():records.append(dict(case=tag,clip=case['clip'],window=case['window'],modality=case['modality'],population=population,group=name,**summarize(data,mask)))
        for i in range(3):
            yy=np.arange(h);keep=(yy>=i*h/3)&(yy<(i+1)*h/3)
            dense.append(dict(case=tag,row_third=i,count=int(keep.sum()*w),positive_fraction=float((-flow[0,keep]>0).mean()),
                vertical_abs=percentiles(np.abs(flow[1,keep]))))
        print(tag,'grid',len(grid(h,w)),'features',len(feature_uv),flush=True)
    analysis=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),base_commit=plan['base_commit'],opencv=cv2.__version__,numpy=np.__version__,
        records=records,dense_row_thirds=dense,verified_hashes=verify(plan['files_sha256']))
    (output/'analysis.json').write_text(json.dumps(analysis,indent=2,allow_nan=False)+'\n')
    rows=[]
    for r in records:
        flat={k:v for k,v in r.items() if not isinstance(v,dict)}
        for k,v in r.items():
            if isinstance(v,dict):flat[k+'_median']=v['p10_p50_p90'][1] if v['finite_count'] else None
        rows.append(flat)
    with (output/'groups.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    plot(output)


def plot(output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    analysis=json.loads((output/'analysis.json').read_text())
    for modality in ['thermal','rgb']:
        cases=list(dict.fromkeys(r['case'] for r in analysis['records'] if r['modality']==modality))
        fig,axes=plt.subplots(6,4,figsize=(12,14))
        for row,case in enumerate(cases):
            with np.load(output/'points'/(case+'-grid.npz')) as d:
                uv=d['uv'];nx=len(np.unique(uv[:,0]));ny=len(np.unique(uv[:,1]))
                for col,(key,vmin,vmax,cmap,label) in enumerate([('disparity',-30,30,'coolwarm','Model disparity px'),('vertical_abs',0,15,'magma','Model |vertical| px'),('source_std',0,20,'viridis','Source patch std'),('ncc',-1,1,'RdYlGn','Model-target NCC')]):
                    ax=axes[row,col];cm=plt.get_cmap(cmap).copy();cm.set_bad('lightgray')
                    im=ax.imshow(d[key].reshape(ny,nx),origin='upper',vmin=vmin,vmax=vmax,cmap=cm,aspect='auto',extent=[uv[:,0].min(),uv[:,0].max(),uv[:,1].max(),uv[:,1].min()])
                    ax.set_title(label if row==0 else '',fontsize=10)
                    if col==0:ax.set_ylabel(case.replace('-'+modality,''),fontsize=9)
                    fig.colorbar(im,ax=ax,fraction=.04,pad=.02)
        fig.suptitle(modality.upper()+': fixed 16px grid; all points retained; gray NCC is undefined\nDisplay ranges clip colors only; model consistency is not ground truth',fontsize=12)
        fig.tight_layout(rect=[0,0,1,.96]);fig.savefig(output/(modality+'-spatial.png'),dpi=160);fig.savefig(output/(modality+'-spatial.pdf'));plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=['prepare','run'])
    parser.add_argument('--source',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.action=='prepare':prepare(args.source,args.output)
    else:run(args.output)
