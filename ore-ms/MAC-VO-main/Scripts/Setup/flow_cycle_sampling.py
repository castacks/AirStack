"""Research-only pixel-centered bilinear sampler and fixed flow-quality summaries."""
import numpy as np


def bilinear(field, uv):
    field = np.asarray(field); uv = np.asarray(uv, dtype=np.float64); h,w,c = field.shape
    assert uv.ndim == 2 and uv.shape[1] == 2 and c == 2
    inside = np.isfinite(uv).all(1) & (uv[:,0]>=0) & (uv[:,0]<=w-1) & (uv[:,1]>=0) & (uv[:,1]<=h-1)
    indices = np.full((len(uv),4,2),-1,dtype=np.int64); corners = np.full((len(uv),4,2),np.nan,dtype=field.dtype); weights=np.full((len(uv),4),np.nan)
    selected=np.flatnonzero(inside); p=uv[inside]; low=np.floor(p).astype(np.int64); high=np.minimum(low+1,[w-1,h-1]); dx,dy=(p-low).T
    ids=np.stack([low,np.column_stack((high[:,0],low[:,1])),np.column_stack((low[:,0],high[:,1])),high],axis=1)
    indices[selected]=ids;corners[selected]=field[ids[:,:,1],ids[:,:,0]];weights[selected]=np.column_stack(((1-dx)*(1-dy),dx*(1-dy),(1-dx)*dy,dx*dy))
    sampled=np.sum(corners.astype(float)*weights[:,:,None],axis=1);valid=inside & np.isfinite(corners).all((1,2)) & np.isfinite(sampled).all(1);sampled[~valid]=np.nan
    return {'indices':indices,'corners':corners,'weights':weights,'sampled':sampled,'in_bounds':inside,'sample_valid':valid}


def summaries(uv0,uv1,inbound,sampled,sample_valid,k,cov):
    uv0,uv1=np.asarray(uv0,dtype=float),np.asarray(uv1,dtype=float);k=np.asarray(k).reshape(3,3);assert k[0,0]>0 and k[1,1]>0
    closure=uv1+sampled-uv0;valid=np.asarray(inbound,dtype=bool)&sample_valid&np.isfinite(closure).all(1)
    angular=np.linalg.norm(closure/[k[0,0],k[1,1]],axis=1);pixel=np.linalg.norm(closure,axis=1)
    cov=np.asarray(cov);cov_valid=np.isfinite(cov).all(1)&(cov[:,0]>0)&(cov[:,1]>0);trace=cov[:,0]/k[0,0]**2+cov[:,1]/k[1,1]**2
    def quartile(values):return np.quantile(values,[.25,.5,.75]).tolist() if len(values) else [None]*3
    data={'grid_count':len(uv0),'original_inbound_count':int(np.asarray(inbound).sum()),'valid_closure_count':int(valid.sum()),'valid_fraction':float(valid.mean()),'closure_angular_q25_q50_q75':quartile(angular[valid]),'closure_pixel_q25_q50_q75':quartile(pixel[valid]),'covariance_valid_count':int(cov_valid.sum()),'angular_covariance_median':float(np.median(trace[cov_valid])) if cov_valid.any() else None}
    return closure,valid,data


def forbidden_read(path):
    from pathlib import Path
    if not isinstance(path,(str,bytes,Path)):return False
    p=Path(path.decode() if isinstance(path,bytes) else path)
    return 'super_odometry' in p.parts or p.name in {'evaluation.json','pose-proxy.json','headroom.json','trajectory.npz','tensor-map.npz','events.jsonl','tracking-flags.npz','report.json'}


def install_label_guard():
    import sys,os
    denied=[]
    def audit(event,args):
        if event=='open':
            path,mode,flags=args;reading=(isinstance(mode,str) and ('r' in mode or '+' in mode)) or (mode is None and not flags&os.O_WRONLY)
            if reading and forbidden_read(path):denied.append(str(path));raise PermissionError('label/reference read blocked')
    sys.addaudithook(audit);return denied
