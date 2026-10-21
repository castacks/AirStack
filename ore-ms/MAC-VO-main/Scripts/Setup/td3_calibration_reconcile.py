"""Fixed thermal calibration candidates and held-out image checks; no default replacement."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from .audit_td3_geometry import pose_matrix, static_query, transform_difference
from .td3_tracking_uncertainty import sha, percentiles
from .td3_night_geometry import verify


def raw_rays(uv, rectification, projection):
    uv=np.asarray(uv,float).reshape(-1,2)
    rays=np.column_stack([uv,np.ones(len(uv))])@np.linalg.inv(np.asarray(projection,float)[:,:3]).T
    rays=rays@np.asarray(rectification,float)
    return rays/rays[:,2:3]


def rectified_pixels(rays, rotation, projection):
    q=np.asarray(rays)@np.asarray(rotation).T@np.asarray(projection)[:,:3].T
    out=np.full((len(q),2),np.nan);valid=np.isfinite(q).all(axis=1)&(q[:,2]>1e-12)
    out[valid]=q[valid,:2]/q[valid,2:3]
    return out


def implied_transform(left,right):
    a,b=np.asarray(left['r']),np.asarray(right['r']);p,q=np.asarray(left['p']),np.asarray(right['p'])
    baseline=-q[0,3]/q[0,0]+p[0,3]/p[0,0]
    result=np.eye(4);result[:3,:3]=b.T@a;result[:3,3]=b.T@np.array([-baseline,0,0])
    return result


def essential(transform):
    t=np.asarray(transform)[:3,3];x,y,z=t
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])@np.asarray(transform)[:3,:3]


def sampson_pixels(a,b,transform,k0,k1):
    e=essential(transform);f=np.linalg.inv(k1).T@e@np.linalg.inv(k0)
    p=a@k0.T;q=b@k1.T;fp=p@f.T;ftq=q@f
    denominator=np.sum(fp[:,:2]**2,axis=1)+np.sum(ftq[:,:2]**2,axis=1)
    out=np.full(len(a),np.nan);valid=np.isfinite(denominator)&(denominator>1e-24)
    out[valid]=np.abs(np.sum(q[valid]*fp[valid],axis=1))/np.sqrt(denominator[valid])
    return out


def triangulation(a,b,transform,k0,k1):
    if not len(a):return {k:np.empty((0,3)) if k=='xyz' else np.empty(0,bool) if k in ['finite','positive','eligible'] else np.empty(0) for k in ['xyz','finite','positive','eligible','parallax_deg','reprojection_px']}
    p=np.eye(3,4);q=transform[:3]
    hom=cv2.triangulatePoints(p,q,np.ascontiguousarray(a[:,:2].T),np.ascontiguousarray(b[:,:2].T)).T
    xyz=np.full((len(a),3),np.nan);finite=np.isfinite(hom).all(axis=1)&(np.abs(hom[:,3])>1e-12)
    xyz[finite]=hom[finite,:3]/hom[finite,3:4]
    target=xyz@transform[:3,:3].T+transform[:3,3]
    finite&=np.isfinite(xyz).all(axis=1)&np.isfinite(target).all(axis=1)&(np.abs(xyz[:,2])>1e-12)&(np.abs(target[:,2])>1e-12)
    positive=finite&(xyz[:,2]>0)&(target[:,2]>0)
    first=a/np.linalg.norm(a,axis=1,keepdims=True)
    second=b@transform[:3,:3];second/=np.linalg.norm(second,axis=1,keepdims=True)
    angle=np.degrees(np.arccos(np.clip(np.sum(first*second,axis=1),-1,1)))
    errors=np.full(len(a),np.nan)
    for indices in [np.flatnonzero(finite)]:
        u=xyz[indices]@k0.T;u=u[:,:2]/u[:,2:3]
        v=target[indices]@k1.T;v=v[:,:2]/v[:,2:3]
        obs0=a[indices]@k0.T;obs1=b[indices]@k1.T
        errors[indices]=np.sqrt((np.sum((u-obs0[:,:2])**2,axis=1)+np.sum((v-obs1[:,:2])**2,axis=1))/2)
    return dict(xyz=xyz,finite=finite,positive=positive,eligible=finite&(angle>=.1),parallax_deg=angle,reprojection_px=errors)


def camera(config,which):
    d=config['calib'][which];fx,fy,cx,cy=d['intrinsics']
    return np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]],float),np.array(d['distortion_coeffs'],float)


def rectify(config,transform):
    k0,d0=camera(config,'cam0');k1,d1=camera(config,'cam1')
    result=cv2.stereoRectify(k0,d0,k1,d1,tuple(config['calib']['cam0']['resolution']),
        np.ascontiguousarray(transform[:3,:3]),np.ascontiguousarray(transform[:3,3]).reshape(3,1),flags=cv2.CALIB_ZERO_DISPARITY,alpha=0)
    return result[:4]


def score(a,b,transform,config):
    k0,_=camera(config,'cam0');k1,_=camera(config,'cam1')
    tri=triangulation(a,b,transform,k0,k1);sampson=sampson_pixels(a,b,transform,k0,k1)
    r0,r1,p0,p1=rectify(config,transform)
    delta=rectified_pixels(a,r0,p0)-rectified_pixels(b,r1,p1)
    eligible=tri['eligible'];finite=tri['finite']
    metrics=dict(count=len(a),finite_triangulations=int(finite.sum()),positive_both_count=int(tri['positive'].sum()),
        positive_both_fraction=float(tri['positive'][finite].mean()) if finite.any() else None,
        parallax_eligible_count=int(eligible.sum()),positive_eligible_fraction=float(tri['positive'][eligible].mean()) if eligible.any() else None,
        sampson_px=percentiles(sampson),reprojection_px=percentiles(tri['reprojection_px']),
        parallax_deg=percentiles(tri['parallax_deg']),rectified_vertical_px=percentiles(np.abs(delta[:,1])),
        rectified_signed_disparity_px=percentiles(delta[:,0]))
    return metrics,dict(**tri,sampson_px=sampson,rectified_delta=delta)


def fit_pose(a,b,config):
    if len(a)<8:raise ValueError('At least eight training observations required')
    k0,_=camera(config,'cam0');k1,_=camera(config,'cam1');focal=np.mean([k0[0,0],k0[1,1],k1[0,0],k1[1,1]])
    cv2.setRNGSeed(0)
    e,mask=cv2.findEssentialMat(a[:,:2],b[:,:2],cameraMatrix=np.eye(3),method=cv2.RANSAC,prob=.999,threshold=1.5/focal,maxIters=5000)
    if e is None or e.shape!=(3,3) or mask is None:raise ValueError('Essential fitting failed or returned multiple solutions')
    count,r,t,pose_mask,_=cv2.recoverPose(e,a[:,:2],b[:,:2],np.eye(3),distanceThresh=1e6,mask=mask.copy())
    if count<8:raise ValueError('Essential pose recovery is degenerate')
    result=np.eye(4);result[:3,:3]=r;result[:3,3]=t.ravel()*np.linalg.norm(np.array(config['calib']['cam1']['T_cn_cnm1'])[:3,3])
    return result,dict(essential=e,ransac_mask=mask.ravel().astype(bool),pose_mask=pose_mask.ravel().astype(bool))


def pose_difference(a,b):
    ta,tb=a[:3,3],b[:3,3]
    return dict(rotation_deg=float(np.degrees(Rotation.from_matrix(a[:3,:3]@b[:3,:3].T).magnitude())),
        translation_direction_deg=float(np.degrees(np.arccos(np.clip(ta@tb/(np.linalg.norm(ta)*np.linalg.norm(tb)),-1,1)))))


def support_gate(rows,stability):
    checks=[]
    for r in rows:
        c=r['candidates']['pooled_image_candidate'];baselines=[r['candidates'][n]['sampson_px']['p10_p50_p90'] for n in ['producer_inverse','kalibr_as_stored']]
        med=c['sampson_px']['p10_p50_p90']
        residual_pass=med is not None and all(v is not None for v in baselines) and med[1]<=.75*min(v[1] for v in baselines)
        checks.append(dict(case=r['case'],parallax_count_pass=c['parallax_eligible_count']>=20,
            cheirality_pass=c['positive_eligible_fraction'] is not None and c['positive_eligible_fraction']>=.95,residual_pass=bool(residual_pass)))
    stable=bool(stability is not None and stability['rotation_deg']<=.25 and stability['translation_direction_deg']<=5)
    return dict(passed=bool(len(checks)==10 and stable and all(all(r[k] for k in ['parallax_count_pass','cheirality_pass','residual_pass']) for r in checks)),
        stability_pass=stable,heldout_checks=checks,scope='Image-support gate only; metric scale and body anchoring unvalidated')


def prepare(output):
    cpu=Path('/workspace/results/stereo-sign/run-001');thermal=Path('/workspace/results/thermal-provenance/run-003')
    old=json.loads((cpu/'plan.json').read_text());cases=[]
    for c in old['cases']:
        if c['modality']!='thermal':continue
        tag=c['clip']+'-'+c['window']+('-00' if c['clip']=='night' else '')
        cases.append(dict(case=tag,role='training' if tag in ['day-original','night-early-00'] else 'heldout',
            features=str(cpu/'arrays'/('-'.join(c[k] for k in ['clip','window','modality'])+'.npz')),prefix='',
            images=c['images'],raw=[c['raw'][s]['path'] for s in ['left','right']],camera_info=c['camera_info'],
            grid='/workspace/results/spatial-stereo/run-001/points/'+'-'.join(c[k] for k in ['clip','window','modality'])+'-grid.npz'))
    more=json.loads((thermal/'plan.json').read_text())
    for s in more['samples']:
        if s['position']==0:continue
        tag=f"night-{s['window']}-{s['position']:02d}"
        cases.append(dict(case=tag,role='heldout',features=str(thermal/'arrays'/f"{s['window']}-{s['position']:02d}.npz"),prefix='matches_',
            images=[s['streams'][side]['processed_path'] for side in ['left','right']],raw=[s['streams'][side]['raw_path'] for side in ['left','right']],
            camera_info=[f'/workspace/data/tartandrive3/20251027/night_thermal_so_salon2/sensors/thermal_{side}_processed_camera_info/data.yaml' for side in ['left','right']]))
    assert len(cases)==12 and sum(c['role']=='training' for c in cases)==2
    inherited=json.loads(Path('/workspace/results/spatial-stereo/run-001/plan.json').read_text());files=dict(inherited['files_sha256']);verify(files)
    paths=[Path(__file__),Path(__file__).parents[1]/'UnitTest/test_td3_calibration_reconcile.py',cpu/'plan.json',thermal/'plan.json',
        Path('/workspace/data/calibration/yamaha.yaml'),Path('/workspace/data/calibration/upstream/get_thermal_data.yaml')]
    paths.extend(Path('/workspace/results/calibration-reconcile/staging/upstream').glob('*'))
    for c in cases:
        paths.extend(Path(p) for p in c['images']+c['raw']+c['camera_info']);paths.append(Path(c['features']))
        if 'grid' in c:paths.append(Path(c['grid']))
    files.update({str(p.resolve()):sha(p) for p in paths});verify(files)
    output.mkdir(parents=True,exist_ok=False)
    plan=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),base_commit='523fec36f',cases=cases,files_sha256=files,
        config='/workspace/data/calibration/upstream/get_thermal_data.yaml',body='/workspace/data/calibration/yamaha.yaml',
        protocol=dict(training=['day-original','night-early-00'],essential_threshold_px=1.5,probability=.999,max_iters=5000,seed=0,
            min_parallax_deg=.1,heldout_count=10,gate_min_count=20,gate_cheirality=.95,gate_residual_ratio=.75,
            stability_rotation_deg=.25,stability_translation_deg=5,scope='Fixed stereo calibration candidate; no metric/body truth or adopted defaults'))
    (output/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')


def run(output):
    plan=json.loads((output/'plan.json').read_text());verify(plan['files_sha256']);cv2.setNumThreads(1)
    config=yaml.safe_load(Path(plan['config']).read_text());body=yaml.safe_load(Path(plan['body']).read_text())
    edges=[(r['from_frame'],r['to_frame'],pose_matrix(r['translation']+r['quaternion'])) for r in body['transform_params']]
    body_left=static_query(edges,'vehicle','thermal_left/optical_frame');body_right=static_query(edges,'vehicle','thermal_right/optical_frame')
    stored=np.array(config['calib']['cam1']['T_cn_cnm1']);candidates={'producer_inverse':np.linalg.inv(stored),'kalibr_as_stored':stored}
    for name in list(candidates):
        flip=candidates[name].copy();flip[:3,3]*=-1;candidates[name+'_translation_flipped']=flip
    candidates['body_yaml_relative']=np.linalg.inv(body_right)@body_left
    observations={};(output/'observations').mkdir();(output/'scores').mkdir()
    controls=[]
    for case in plan['cases']:
        infos=[yaml.safe_load(Path(p).read_text()) for p in case['camera_info']]
        with np.load(case['features']) as f:
            prefix=case['prefix'];keep=f[prefix+'inlier'];uv=[f[prefix+k][keep] for k in ['uv0','uv1']]
        a,b=[raw_rays(u,i['r'],i['p']) for u,i in zip(uv,infos)]
        observations[case['case']]=(a,b)
        np.savez_compressed(output/'observations'/(case['case']+'.npz'),rays0=a,rays1=b,uv0=uv[0],uv1=uv[1])
        controls.append(dict(case=case['case'],projection_transform_vs_producer=transform_difference(implied_transform(*infos),candidates['producer_inverse'])))
    train=[c['case'] for c in plan['cases'] if c['role']=='training']
    a=np.concatenate([observations[n][0] for n in train]);b=np.concatenate([observations[n][1] for n in train])
    pooled,fit=fit_pose(a,b,config);np.savez_compressed(output/'pooled-fit.npz',**fit)
    candidates['pooled_image_candidate']=pooled
    singles=[];single_reports=[]
    for name in train:
        try:
            t,arrays=fit_pose(*observations[name],config);singles.append(t)
            single_reports.append(dict(case=name,status='PASS',transform=t.tolist()));np.savez_compressed(output/(name+'-fit.npz'),**arrays)
        except Exception as error:single_reports.append(dict(case=name,status='FAILED',error=str(error)))
    stability=pose_difference(*singles) if len(singles)==2 else None
    records=[];grid_records=[]
    for case in plan['cases']:
        a,b=observations[case['case']];scores={}
        for name,t in candidates.items():
            metrics,arrays=score(a,b,t,config);scores[name]=metrics
            np.savez_compressed(output/'scores'/(case['case']+'-'+name+'.npz'),**arrays)
        records.append(dict(case=case['case'],role=case['role'],candidates=scores))
        if 'grid' in case:
            infos=[yaml.safe_load(Path(p).read_text()) for p in case['camera_info']]
            with np.load(case['grid']) as g:
                x,y=[raw_rays(u,i['r'],i['p']) for u,i in zip([g['uv'],g['uv']+g['flow']],infos)]
                masks={'all':np.ones(len(x),bool),'ncc_defined':g['ncc_valid'],'source_std_gt5':g['source_std']>5}
                for pop,mask in masks.items():
                    grid_records.append(dict(case=case['case'],population=pop,candidates={n:score(x[mask],y[mask],t,config)[0] for n,t in candidates.items()}))
        print(case['case'],case['role'],len(a),flush=True)
    gate=support_gate([r for r in records if r['role']=='heldout'],stability)
    export=dict(status='experimental_image_gate_passed' if gate['passed'] else 'experimental_image_gate_failed',
        direction='T_right_raw_optical_from_left_raw_optical',T_right_from_left=pooled.tolist(),
        baseline_magnitude_source='Norm of supplied stereo translation; metric scale unvalidated',
        body_anchor_status='Original YAML left-body anchor retained; derived right-body transform is algebraic, not independently calibrated',
        T_body_from_left_raw=body_left.tolist(),T_body_from_right_raw=(body_left@np.linalg.inv(pooled)).tolist(),
        gate=gate,source_files_sha256=plan['files_sha256'])
    (output/'candidate-calibration.yaml').write_text(yaml.safe_dump(export,sort_keys=False))
    analysis=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),base_commit=plan['base_commit'],opencv=cv2.__version__,numpy=np.__version__,
        candidates={n:t.tolist() for n,t in candidates.items()},source_controls=controls,
        candidate_vs_source={n:pose_difference(pooled,t) for n,t in candidates.items() if n!='pooled_image_candidate'},
        training_diagnostics=single_reports,training_stability=stability,gate=gate,cases=records,grid_cases=grid_records,
        verified_hashes=verify(plan['files_sha256']))
    (output/'analysis.json').write_text(json.dumps(analysis,indent=2,allow_nan=False)+'\n')
    preview(output,plan,config,pooled)


def preview(output,plan,config,transform):
    from .td3_thermal_provenance import producer_functions,preprocess
    funcs=producer_functions('/workspace/data/calibration/upstream/get_thermal_data.py')
    r0,r1,p0,p1=rectify(config,transform);size=tuple(config['calib']['cam0']['resolution'])
    dest=output/'preview';dest.mkdir()
    (dest/'rectification.yaml').write_text(yaml.safe_dump(dict(status='experimental_only',r_left=r0.tolist(),r_right=r1.tolist(),p_left=p0.tolist(),p_right=p1.tolist())))
    for case in plan['cases']:
        if 'grid' not in case:continue
        for side,p,r,projection,cam in zip(['left','right'],case['raw'],[r0,r1],[p0,p1],['cam0','cam1']):
            raw=cv2.imread(p,cv2.IMREAD_UNCHANGED)
            if raw is None or raw.dtype!=np.uint16:raise ValueError('Expected native raw image')
            k,d=camera(config,cam);mx,my=cv2.initUndistortRectifyMap(k,d,r,projection,size,cv2.CV_32FC1)
            rect=cv2.remap(raw,mx,my,cv2.INTER_LINEAR)
            im=preprocess(rect,funcs,config['process'],config['enhance'])
            if not cv2.imwrite(str(dest/(case['case']+'-'+side+'.png')),im):raise RuntimeError('Preview PNG write failed')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=['prepare','run']);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.action=='prepare':prepare(args.output)
    else:run(args.output)
