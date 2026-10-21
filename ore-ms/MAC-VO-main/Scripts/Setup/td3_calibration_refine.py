"""Constrain nominal camera centers, fit rotation on fixed training pairs, test fresh images."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import yaml

from .td3_calibration_reconcile import camera, essential, raw_rays, score, pose_difference, preview
from .audit_td3_geometry import pose_matrix, static_query
from .td3_thermal_provenance import match_stereo
from .td3_tracking_uncertainty import sha
from .td3_night_geometry import verify


def centered_pose(rotvec,center):
    t=np.eye(4);t[:3,:3]=Rotation.from_rotvec(rotvec).as_matrix();t[:3,3]=-t[:3,:3]@center
    return t


def signed_residual(rotvec,center,a,b,k0,k1):
    f=np.linalg.inv(k1).T@essential(centered_pose(rotvec,center))@np.linalg.inv(k0)
    x=a@k0.T;y=b@k1.T;fx=x@f.T;fty=y@f
    denom=np.sqrt(np.sum(fx[:,:2]**2,axis=1)+np.sum(fty[:,:2]**2,axis=1))
    return np.sum(y*fx,axis=1)/np.maximum(denom,1e-15)


def fit_rotation(a,b,center,config):
    if len(a)<8 or not np.isfinite([a,b]).all() or np.linalg.norm(center)<=0:raise ValueError('Invalid training rays or nominal centers')
    k0,_=camera(config,'cam0');k1,_=camera(config,'cam1');bound=np.deg2rad(5)
    result=least_squares(signed_residual,np.zeros(3),args=(center,a,b,k0,k1),bounds=(-bound,bound),
        loss='soft_l1',f_scale=1.5,max_nfev=2000,xtol=1e-12,ftol=1e-12,gtol=1e-12)
    if not result.success:raise ValueError('Rotation solver did not converge: '+result.message)
    return centered_pose(result.x,center),dict(success=bool(result.success),nfev=result.nfev,cost=float(result.cost),
        optimality=float(result.optimality),rotvec_deg=np.rad2deg(result.x).tolist(),
        bound_saturated=bool(np.any(np.abs(np.rad2deg(result.x))>=4.99)))


def gate(rows,stability,fits):
    checks=[]
    for r in rows:
        c=r['candidates']['body_centers_rotation_fit'];baselines=[r['candidates'][n]['sampson_px']['p10_p50_p90'] for n in ['producer_inverse','kalibr_as_stored']]
        median=c['sampson_px']['p10_p50_p90']
        checks.append(dict(case=r['case'],parallax_count_pass=c['parallax_eligible_count']>=20,
            cheirality_pass=c['positive_eligible_fraction'] is not None and c['positive_eligible_fraction']>=.95,
            residual_pass=bool(median is not None and all(v is not None for v in baselines) and median[1]<=.75*min(v[1] for v in baselines))))
    stable=stability is not None and stability['rotation_deg']<=.25
    interior=bool(len(fits)==3 and all(not r['bound_saturated'] for r in fits))
    return dict(passed=bool(len(checks)==12 and stable and interior and all(all(r[k] for k in ['parallax_count_pass','cheirality_pass','residual_pass']) for r in checks)),
        stability_pass=bool(stable),bounds_pass=interior,fresh_validation_checks=checks,
        scope='Image-support gate with nominal body centers fixed; metric/body/intrinsic truth unvalidated')


def mapped(value):
    if isinstance(value,str):return value.replace('/root/AirStack/ore-ms/','/workspace/',1)
    if isinstance(value,list):return [mapped(v) for v in value]
    if isinstance(value,dict):return {k:mapped(v) for k,v in value.items()}
    return value


def prepare(output):
    source=Path('/workspace/results/calibration-reconcile/run-001');old=json.loads((source/'plan.json').read_text())
    files=dict(old['files_sha256']);verify(files)
    extraction=Path('/workspace/results/calibration-reconcile/inputs/validation/extraction.json')
    fresh=mapped(json.loads(extraction.read_text())['cases'])
    paths=[Path(__file__),Path(__file__).parents[1]/'UnitTest/test_td3_calibration_refine.py',extraction,source/'plan.json',source/'analysis.json']
    paths.extend((source/'observations').glob('*.npz'))
    for c in fresh:paths.extend(Path(p) for p in c['images']+c['camera_info']);paths.append(Path(c['source_report']))
    files.update({str(p.resolve()):sha(p) for p in paths});verify(files)
    output.mkdir(parents=True,exist_ok=False)
    plan=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),base_commit='523fec36f',source=str(source),cases=fresh,files_sha256=files,
        training=old['protocol']['training'],config=old['config'],body=old['body'],
        protocol=dict(centers='Preserve original body YAML camera centers; fit only relative rotation',
            rotvec_bounds_deg=5,initial_rotvec=[0,0,0],loss='soft_l1',f_scale_px=1.5,max_nfev=2000,tolerances=1e-12,
            new_validation='Positions4/8 of all six existing report windows; new feature detections on original processed images',
            matcher='SIFT2000/mutual ratio.7/F-RANSAC1.5px,.999,seed0; retain empty fits',
            gate='Each of12 fresh pairs:>=20 parallax>=.1deg,>=95% front,median Sampson<=.75*better producer/as-stored; single-train rotation<=.25deg; no bound saturation',
            scope='Experimental stereo correction; no fitting to validation or calibration default change'))
    (output/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')


def run(output):
    plan=json.loads((output/'plan.json').read_text());verify(plan['files_sha256']);cv2.setNumThreads(1)
    source=Path(plan['source']);old=json.loads((source/'plan.json').read_text());config=yaml.safe_load(Path(plan['config']).read_text())
    body=yaml.safe_load(Path(plan['body']).read_text());edges=[(r['from_frame'],r['to_frame'],pose_matrix(r['translation']+r['quaternion'])) for r in body['transform_params']]
    left=static_query(edges,'vehicle','thermal_left/optical_frame');right=static_query(edges,'vehicle','thermal_right/optical_frame')
    center=(np.linalg.inv(left)@right)[:3,3]
    training=[]
    for tag in plan['training']:
        with np.load(source/'observations'/(tag+'.npz')) as f:training.append((f['rays0'].copy(),f['rays1'].copy()))
    pooled,diagnostic=fit_rotation(np.concatenate([a for a,b in training]),np.concatenate([b for a,b in training]),center,config)
    singles=[fit_rotation(a,b,center,config) for a,b in training];stability=pose_difference(singles[0][0],singles[1][0])
    fits=[diagnostic]+[r[1] for r in singles]
    candidates={n:np.array(t) for n,t in json.loads((source/'analysis.json').read_text())['candidates'].items()}
    candidates['body_centers_rotation_fit']=pooled
    (output/'observations').mkdir();(output/'scores').mkdir();records=[];source_checks=[]
    for c in plan['cases']:
        infos=[yaml.safe_load(Path(p).read_text()) for p in c['camera_info']];p0,p1=[np.array(i['p']) for i in infos]
        images=[cv2.imread(p,cv2.IMREAD_UNCHANGED) for p in c['images']]
        if any(im is None for im in images):raise ValueError('Missing fresh validation image')
        original_metrics,matches=match_stereo(*images,p0,p1)
        keep=matches['inlier'];uv=[matches[k][keep] for k in ['uv0','uv1']]
        a,b=[raw_rays(u,i['r'],i['p']) for u,i in zip(uv,infos)]
        np.savez_compressed(output/'observations'/(c['case']+'.npz'),**matches,rays0=a,rays1=b)
        scores={}
        for name,t in candidates.items():
            scores[name],arrays=score(a,b,t,config);np.savez_compressed(output/'scores'/(c['case']+'-'+name+'.npz'),**arrays)
        records.append(dict(case=c['case'],status=original_metrics['status'],original_match_diagnostic=original_metrics,candidates=scores))
        for side,i in zip(['cam0','cam1'],infos):
            k,d=camera(config,side)
            source_checks.append(dict(case=c['case'],camera=side,raw_K_max_difference=float(np.max(np.abs(k-np.array(i['k'])))),raw_D_max_difference=float(np.max(np.abs(d-np.array(i['d']))))))
        print(c['case'],original_metrics['status'],len(a),flush=True)
    validation=gate(records,stability,fits)
    derived_right=left@np.linalg.inv(pooled)
    center_error=float(np.linalg.norm(right[:3,3]-derived_right[:3,3]))
    if center_error>1e-12:raise RuntimeError('Nominal body centers changed unexpectedly')
    export=dict(status='experimental_image_gate_passed' if validation['passed'] else 'experimental_image_gate_failed',
        direction='T_right_raw_optical_from_left_raw_optical',T_right_from_left=pooled.tolist(),
        T_body_from_left_raw=left.tolist(),T_body_from_right_raw=derived_right.tolist(),
        nominal_body_camera_centers_preserved=True,body_anchor_status='Source body YAML prior, not independently validated',gate=validation,
        provenance=dict(training=plan['training'],fresh_validation=[c['case'] for c in plan['cases']],source_files_sha256=plan['files_sha256']))
    (output/'candidate-calibration.yaml').write_text(yaml.safe_dump(export,sort_keys=False))
    analysis=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),base_commit=plan['base_commit'],opencv=cv2.__version__,numpy=np.__version__,
        candidate=pooled.tolist(),fixed_center_in_left=center.tolist(),fit_diagnostics=fits,training_stability=stability,
        candidate_vs_source={n:pose_difference(pooled,t) for n,t in candidates.items() if n!='body_centers_rotation_fit'},
        nominal_right_body_center_error_m=center_error,source_intrinsics_checks=source_checks,gate=validation,cases=records,
        verified_hashes=verify(plan['files_sha256']))
    (output/'analysis.json').write_text(json.dumps(analysis,indent=2,allow_nan=False)+'\n')
    preview(output,old,config,pooled)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=['prepare','run']);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.action=='prepare':prepare(args.output)
    else:run(args.output)
