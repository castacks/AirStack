"""Offline proxy-conditioned reprojection diagnostics; no fitted correction or GT claim."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import numpy as np
try:
    from .td3_reference import load_reference, camera_reference, matrices, OPTICAL_FROM_NED
    from .td3_feature_consistency import unproject_ned
except ImportError:
    from td3_reference import load_reference, camera_reference, matrices, OPTICAL_FROM_NED
    from td3_feature_consistency import unproject_ned


def project_ned(points, k):
    """Project finite points in front of the camera, return NaN for invalid points."""
    optical = np.asarray(points) @ OPTICAL_FROM_NED.T
    valid = np.isfinite(optical).all(axis=1) & (optical[:,2] > 1e-9)
    uv = np.full((len(points),2),np.nan)
    homogeneous = optical[valid] @ np.asarray(k).T
    uv[valid] = homogeneous[:,:2]/homogeneous[:,2:3]
    return uv


def pair_geometry(uv0, uv1, d0, d1, k, relative):
    """Relative is camera1_from_camera0 in NED; depths are forward-axis meters."""
    k = np.asarray(k).reshape(3,3)
    points0 = unproject_ned(uv0,d0,np.repeat(k[None],len(d0),axis=0))
    points1 = unproject_ned(uv1,d1,np.repeat(k[None],len(d1),axis=0))
    r,t = relative[:3,:3],relative[:3,3]
    transformed = points0@r.T+t
    reverse = (points1-t)@r
    forward_error = np.asarray(uv1)-project_ned(transformed,k)
    reverse_error = np.asarray(uv0)-project_ned(reverse,k)
    rotation_error = np.asarray(uv1)-project_ned(points0@r.T,k)
    rotation,translation = OPTICAL_FROM_NED@r@OPTICAL_FROM_NED.T,OPTICAL_FROM_NED@t
    tx,ty,tz = translation
    skew = np.array([[0,-tz,ty],[tz,0,-tx],[-ty,tx,0]])
    invk = np.linalg.inv(k)
    f = invk.T@skew@rotation@invk
    x0 = np.column_stack([uv0,np.ones(len(d0))]); x1 = np.column_stack([uv1,np.ones(len(d0))])
    line1,line0 = x0@f.T,x1@f
    denominator = np.sum(line1[:,:2]**2+line0[:,:2]**2,axis=1)
    sampson = np.full(len(d0),np.nan)
    valid = np.isfinite(denominator)&(denominator>1e-20)
    sampson[valid] = np.abs(np.sum(x1[valid]*line1[valid],axis=1))/np.sqrt(denominator[valid])
    return {'forward_error_px':forward_error,'reverse_error_px':reverse_error,
            'rotation_only_error_px':rotation_error,'sampson_px':sampson,
            'reference_predicted_depth1_m':transformed[:,0]}


def summary(data,mask):
    def percentile(a):
        a=np.asarray(a)[mask]; a=a[np.isfinite(a)]
        return {'finite_count':len(a),'p10_p50_p90':np.percentile(a,[10,50,90]).tolist() if len(a) else None}
    return {'count':int(mask.sum()),
            **{key:percentile(data[key]) for key in ['depth0_m','disparity0_px','forward_norm_px',
                'reverse_norm_px','sampson_px','vo_forward_norm_px','depth1_over_reference_prediction',
                'reference_disparity_abs_error_px','vo_disparity_abs_error_px']}}


def analyze(directory,reference,dataset,calibration,output):
    report=json.loads((directory/'report.json').read_text())
    source=np.load(directory/'trajectory.npz'); times=source['time_ns'].ravel()
    camera=camera_reference(reference,times,dataset,calibration,report['modality'])
    estimated=matrices(source['poses_camera_ned'])
    graph=np.load(directory/'tensor-map.npz')
    if not np.array_equal(times,graph['frames//time_ns'].ravel()): raise ValueError('Graph timestamp mismatch')
    paths=sorted((directory/'uncertainty').glob('pair-*.npz'))
    if len(paths)!=len(times)-1: raise ValueError('Missing consecutive-pair records')
    populations={'grid':[],'retained':[]}
    for index,path in enumerate(paths):
        with np.load(path) as pair:
            if int(pair['time0_ns'])!=int(times[index]): raise ValueError('Pair timestamp mismatch')
            relative=np.linalg.inv(camera[index+1])@camera[index]
            vo_relative=np.linalg.inv(estimated[index+1])@estimated[index]
            for population in populations:
                prefix='grid' if population=='grid' else 'selected'
                inbound=pair[prefix+'_inbound'].copy()
                mask=inbound.copy()
                if population=='retained':
                    mask[inbound]=pair['outlier_keep_inbound']
                uv0,uv1=pair[prefix+'_uv0'][mask],pair[prefix+'_uv1'][mask]
                d0,d1=pair[prefix+'_depth0'][mask],pair[prefix+'_depth1'][mask]
                geometry=pair_geometry(uv0,uv1,d0,d1,pair['K'],relative)
                vo=pair_geometry(uv0,uv1,d0,d1,pair['K'],vo_relative)
                height,width=pair['height_width']
                region=np.minimum((uv0[:,1]/height*3).astype(int),2)*3+np.minimum((uv0[:,0]/width*3).astype(int),2)
                disparity=pair['K'].reshape(3,3)[0,0]*report['execution_baseline_m']/d0
                if population=='retained':
                    ids=(graph['edge/match2frame1/mapping']==index)&(graph['edge/match2frame2/mapping']==index+1)
                    if not np.array_equal(uv0,graph['match//pixel1_uv'][ids]): raise ValueError('Retained observation association mismatch')
                    saved=graph['match//pixel1_disp'][ids].ravel()
                    if not np.allclose(disparity,saved,rtol=2e-6): raise ValueError('Saved/derived disparity mismatch')
                    disparity=saved
                predicted=geometry['reference_predicted_depth1_m']
                ratio=np.full(len(d1),np.nan)
                np.divide(d1,predicted,out=ratio,where=predicted>1e-9)
                record={'pair':np.full(len(d0),index),'uv0':uv0,'uv1':uv1,
                    'region':region,'depth0_m':d0,'depth1_m':d1,'disparity0_px':disparity,
                    'forward_norm_px':np.linalg.norm(geometry['forward_error_px'],axis=1),
                    'reverse_norm_px':np.linalg.norm(geometry['reverse_error_px'],axis=1),
                    'vo_forward_norm_px':np.linalg.norm(vo['forward_error_px'],axis=1),
                    'depth1_over_reference_prediction':ratio,**geometry}
                factor=pair['K'].reshape(3,3)[0,0]*report['execution_baseline_m']
                disparity1=factor/d1
                for label,geo in [('reference',geometry),('vo',vo)]:
                    predicted=geo['reference_predicted_depth1_m']
                    predicted_disparity=np.full(len(d0),np.nan)
                    np.divide(factor,predicted,out=predicted_disparity,where=predicted>1e-9)
                    record[label+'_disparity_abs_error_px']=np.abs(predicted_disparity-disparity1)
                if population=='retained':
                    uvvar=graph['match//pixel2_uv_cov'][ids,:2]
                    dispvar=graph['match//pixel2_disp_cov'][ids].ravel()
                    # Active disparity graph has zero uv cross-covariance for this frontend.
                    if not np.all(graph['match//pixel2_uv_cov'][ids,2]==0):
                        raise ValueError('Diagonal-energy diagnostic requires zero uv covariance')
                    for label,geo in [('reference',geometry),('vo',vo)]:
                        record[label+'_uv_inverse_variance_energy']=np.sum(geo['forward_error_px']**2/uvvar,axis=1)
                        record[label+'_disparity_inverse_variance_energy']=record[label+'_disparity_abs_error_px']**2/dispvar
                populations[population].append(record)
    results={}
    output.mkdir(parents=True,exist_ok=False)
    for population,records in populations.items():
        data={key:np.concatenate([r[key] for r in records]) for key in records[0]}
        valid=np.isfinite(data['depth0_m'])&np.isfinite(data['depth1_m'])&(data['depth0_m']>0)&(data['depth1_m']>0)
        primary=valid&(data['depth0_m']<=50)&(data['depth1_m']<=50)
        data['valid_positive_depth']=valid;data['primary_mask']=primary
        np.savez_compressed(output/(population+'.npz'),**data)
        results[population]={'all_inbound':summary(data,np.ones(len(valid),dtype=bool)),
            'both_depths_max50m':summary(data,primary),
            'regions':{str(i):summary(data,primary&(data['region']==i)) for i in range(9)},
            'source_depth_bins':{f'{lo}-{hi}':summary(data,primary&(data['depth0_m']>=lo)&(data['depth0_m']<hi)) for lo,hi in [(0,5),(5,10),(10,20),(20,50)]}}
    data=np.load(output/'retained.npz')
    mask=data['primary_mask']
    results['active_objective_diagnostic']={
        'meaning':'image/disparity inverse-variance residual energies before robust Huber; not optimizer loss or causal attribution',
        'graph_type':json.loads((directory/'effective-config.json').read_text())['Odometry']['optimizer']['args']['graph_type']}
    for label in ['reference','vo']:
        uv=data[label+'_uv_inverse_variance_energy'][mask]
        disp=data[label+'_disparity_inverse_variance_energy'][mask]
        finite=np.isfinite(uv)&np.isfinite(disp)
        total=np.sum(uv[finite]+disp[finite])
        results['active_objective_diagnostic'][label]={
            'finite_count':int(finite.sum()),
            'disparity_fraction_of_total_inverse_variance_energy':float(np.sum(disp[finite])/total) if total else None,
            'median_uv_energy':float(np.median(uv[finite])),
            'median_disparity_energy':float(np.median(disp[finite]))}
    return results


def plot(results,source,output,dataset):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import cv2
    names=['early','original','late']
    fig,axes=plt.subplots(2,3,figsize=(13,7),layout='constrained')
    for row,m in enumerate(['rgb','thermal']):
        for col,key in enumerate(['depth0_m','forward_norm_px','sampson_px']):
            ax=axes[row,col]
            for x,name in enumerate(names):
                p=results[name][m]['retained']['both_depths_max50m'][key]['p10_p50_p90']
                ax.errorbar(x,p[1],yerr=[[p[1]-p[0]],[p[2]-p[1]]],fmt='o',capsize=5)
            ax.set_xticks(range(3),names);ax.set_title(m+' '+key);ax.set_ylim(bottom=0)
    fig.suptitle('Retained matches, both depths ≤50m: median and 10–90 percentiles')
    fig.savefig(output/'diagnostics.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(2,3,figsize=(13,7),layout='constrained')
    for row,m in enumerate(['rgb','thermal']):
        for col,name in enumerate(names):
            report=json.loads((source/name/m/'report.json').read_text())
            path=dataset/report['input_rows'][9][m+'_left']['path']
            im=cv2.imread(str(path),cv2.IMREAD_UNCHANGED)
            if im.ndim==3: im=cv2.cvtColor(im,cv2.COLOR_BGR2RGB)
            ax=axes[row,col];ax.imshow(im,cmap='gray',vmin=0,vmax=255)
            data=np.load(output/name/m/'retained.npz');mask=data['primary_mask']&(data['pair']==9)
            sc=ax.scatter(data['uv0'][mask,0],data['uv0'][mask,1],c=data['forward_norm_px'][mask],s=13,vmin=0,vmax=10,cmap='magma')
            ax.set_title(name+' '+m);ax.set_axis_off()
    fig.colorbar(sc,ax=axes,label='Reference-conditioned reprojection residual (px), clipped at10')
    fig.savefig(output/'representative-matches.png',dpi=160);plt.close(fig)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['source','dataset','calibration','output']:p.add_argument('--'+name,type=Path,required=True)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    reference,_=load_reference(args.dataset);results={}
    for name in ['early','original','late']:
        results[name]={}
        for modality in ['rgb','thermal']:
            results[name][modality]=analyze(args.source/name/modality,reference,args.dataset,args.calibration,args.output/name/modality)
    inputs=[args.calibration,args.source/'plan.json']
    for name in results:
        for modality in results[name]:
            directory=args.source/name/modality
            inputs.extend([directory/'report.json',directory/'trajectory.npz',directory/'tensor-map.npz'])
            inputs.extend(sorted((directory/'uncertainty').glob('pair-*.npz')))
    provenance={'created_at_utc':datetime.now(timezone.utc).isoformat(),
        'base_commit':'af553955e844d42abb164ce5218e6be06cf19c8c plus uncommitted changes',
        'source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),Path(__file__).with_name('td3_reference.py'),Path(__file__).with_name('td3_feature_consistency.py')]},
        'input_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}}
    (args.output/'analysis.json').write_text(json.dumps({'scope':'same-clip proxy-conditioned consistency, not GT or causal attribution','provenance':provenance,'results':results},indent=2,allow_nan=False)+'\n')
    plot(results,args.source,args.output,args.dataset)


if __name__=='__main__':main()
