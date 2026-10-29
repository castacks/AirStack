"""Offline nominal-body SLAM-proxy losses for already frozen causal decisions.

Reference-assisted interval transport is an evaluation operation only. Nothing
from this module is a causal predictor, an independent physical label or a fitted
selector. The source advice and expert trajectories are never modified.
"""
import argparse
import hashlib
import json
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from Scripts.Setup.audit_td3_geometry import pose_matrix, static_query
from Scripts.Setup.td3_reference import load_reference, OPTICAL_FROM_NED


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def se3(matrix):
    matrix=np.asarray(matrix,dtype=np.float64)
    if matrix.shape!=(4,4) or not np.isfinite(matrix).all():
        raise ValueError('finite 4x4 SE3 required')
    r=matrix[:3,:3]
    if not np.allclose(matrix[3],[0,0,0,1],atol=1e-10,rtol=0) or not np.allclose(r.T@r,np.eye(3),atol=1e-8,rtol=0) or not np.isclose(np.linalg.det(r),1,atol=1e-8,rtol=0):
        raise ValueError('invalid homogeneous pose or proper rotation')
    return matrix


def body_delta(camera_delta, body_from_camera):
    d,e=se3(camera_delta),se3(body_from_camera)
    return e@d@np.linalg.inv(e)


def transport(delta, common_start, own_start, own_end, common_end):
    d,r0,a,b,r1=map(se3,(delta,common_start,own_start,own_end,common_end))
    return np.linalg.inv(r0)@a@d@np.linalg.inv(b)@r1


def losses(predicted, reference):
    error=np.linalg.inv(se3(reference))@se3(predicted)
    return {'translation_m':float(np.linalg.norm(error[:3,3])),
            'rotation_deg':float(np.degrees(Rotation.from_matrix(error[:3,:3]).magnitude()))}


def at(reference,times):
    if any(type(t) is not int for t in times):
        raise ValueError('exact integer nanosecond query required')
    return reference.at(np.asarray(times,dtype=np.int64))


def rectification_rotation(raw):
    raw=np.asarray(raw,dtype=float)
    if raw.shape!=(3,3) or not np.isfinite(raw).all() or np.linalg.det(raw)<=0 or np.max(np.abs(raw.T@raw-np.eye(3)))>1e-6:
        raise ValueError('rectification matrix exceeds declared serialization-roundoff tolerance')
    u,_,vt=np.linalg.svd(raw)
    return u@vt


def extrinsic(calibration,dataset,camera):
    config=yaml.safe_load(calibration.read_text())
    edges=[(e['from_frame'],e['to_frame'],pose_matrix(e['translation']+e['quaternion'])) for e in config['transform_params']]
    optical='multisense/left_camera_optical_frame' if camera=='rgb' else 'thermal_left/optical_frame'
    name='multisense_left_rect_color_camera_info' if camera=='rgb' else 'thermal_left_processed_camera_info'
    info=yaml.safe_load((dataset/'sensors'/name/'data.yaml').read_text())
    optical_from_ned=np.eye(4);optical_from_ned[:3,:3]=rectification_rotation(info['r']).T@OPTICAL_FROM_NED
    return se3(static_query(edges,'vehicle',optical)@optical_from_ned)


def stats(values):
    a=np.asarray(values,dtype=float)
    return {'count':len(a),'mean':float(a.mean()),'median':float(np.median(a)),
            'rmse':float(np.sqrt(np.mean(a*a)))} if len(a) else {'count':0,'mean':None,'median':None,'rmse':None}


def summarize(rows):
    summary={}
    for window in sorted({r['window'] for r in rows}):
        selected=[r for r in rows if r['window']==window]
        both=[r for r in selected if all(r['experts'][c] is not None for c in ('rgb','thermal'))]
        policies={}
        for policy in ('rgb_only','thermal_only','advisory'):
            values=[]
            for row in selected:
                camera=policy.removesuffix('_only') if policy!='advisory' else row['action'].lower()
                if camera in row['experts'] and row['experts'][camera] is not None:
                    values.append(row['experts'][camera]['common_loss'])
            policies[policy]={'available':len(values),'abstain':len(selected)-len(values),
                **{k:stats([v[k] for v in values]) for k in ('translation_m','rotation_deg')}}
        common={c:{k:stats([r['experts'][c]['common_loss'][k] for r in both]) for k in ('translation_m','rotation_deg')} for c in ('rgb','thermal')}
        wins={'rgb':0,'thermal':0,'tie':0}
        for row in both:
            a,b=[row['experts'][c]['common_loss']['translation_m'] for c in ('rgb','thermal')]
            wins['tie' if abs(a-b)<=1e-12 else 'rgb' if a<b else 'thermal']+=1
        oracle=[min(r['experts'][c]['common_loss']['translation_m'] for c in ('rgb','thermal')) for r in both]
        summary[window]={'decisions':len(selected),'policies':policies,'common_eligible_count':len(both),
            'common_eligible_losses':common,'common_translation_wins':wins,'common_translation_oracle':stats(oracle)}
    return summary


def run(root,output):
    if output.exists():raise FileExistsError(output)
    calibration=root/'data/calibration/yamaha.yaml'
    if sha(calibration)!='031c199619916bf1466a807d9ed91896d58b1428f0bdf74951dabc3c1b59e27d':
        raise ValueError('calibration differs from the historically audited supplied file')
    source=root/'data/tartandrive3/20251027/night_mac_salon_warehousefig8'
    reference,metadata=load_reference(source)
    paths=[calibration,*[(source/'super_odometry/odometry')/n for n in ('info.yaml','interp_data.txt','interp_timestamps.txt')],
        Path(__file__),Path(__file__).with_name('td3_reference.py'),Path(__file__).with_name('audit_td3_geometry.py')]
    inputs=[]
    for window,run_path,dataset in (
        ('early',root/'results/shared-arbiter-events/run-001',root/'results/new-night-validation/prepared-001/datasets/early'),
        ('middle',root/'results/arbiter-support-collapse/run-001',root/'results/long-night-validation/prepared-001/datasets/middle')):
        paths += [run_path/n for n in ('timeline.jsonl','protocol.json','summary.json','validation.json')]
        for c in ('rgb','thermal'):
            paths += [run_path/'experts'/c/'arbiter-events/events.jsonl',run_path.parent/(c+'-preservation.json')]
            name='multisense_left_rect_color_camera_info' if c=='rgb' else 'thermal_left_processed_camera_info'
            paths.append(dataset/'sensors'/name/'data.yaml')
        inputs.append((window,run_path,dataset))
    frozen={str(p.relative_to(root)) if p.is_relative_to(root) else str(p):sha(p) for p in paths}
    output.mkdir(parents=True)
    protocol={'created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'base_commit':'7fde85a6e',
        'input_sha256':frozen,'reference_metadata':metadata,'common_interval':'original RGB interval for each manifest transition',
        'primary_loss':'norm translation(inv(reference_body_delta)*reference_bridged_predicted_body_delta), meters',
        'secondary_loss':'rotation geodesic of same residual, degrees; separate, no mixed-unit score',
        'body_delta':'E*D_camera*inv(E), E=nominal T_body_from_rectified_camera_NED',
        'time_transport':'inv(R(r0))*R(a)*D_body*inv(R(b))*R(r1)',
        'reference_used_for_offline_transport':True,'rectification_rotation_policy':'nearest proper SO3 by SVD; reject source orthogonality defect >1e-6 or det<=0; serialization roundoff only, no fitted calibration',
        'physical_body_transform_verified':False,
        'independent_ground_truth':False,'causal_feature_eligible':False,'gate_fitted':False,
        'scope':'conditional nominal-frame SLAM-proxy evaluation, not physical accuracy or deployable reference-assisted selection'}
    (output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    rows=[]
    for window,path,dataset in inputs:
        audit=json.loads((path/'validation.json').read_text());summary=json.loads((path/'summary.json').read_text())
        if audit['status']!='PASS' or audit['timeline_sha256']!=sha(path/'timeline.jsonl') or summary['timeline_sha256']!=audit['timeline_sha256']:
            raise ValueError('unverified source timeline')
        timeline=[json.loads(s) for s in (path/'timeline.jsonl').read_text().splitlines()]
        manifest=json.loads((path/'protocol.json').read_text())['input_rows']
        events={c:[json.loads(s) for s in (path/'experts'/c/'arbiter-events/events.jsonl').read_text().splitlines()] for c in ('rgb','thermal')}
        es={c:extrinsic(calibration,dataset,c) for c in events}
        for c in events:
            proof=json.loads((path.parent/(c+'-preservation.json')).read_text())
            if proof['status']!='PASS' or proof['events_sha256']!=sha(path/'experts'/c/'arbiter-events/events.jsonl'):
                raise ValueError('unverified expert journal')
        for decision in (r for r in timeline if r['kind']=='decision'):
            pair=decision['pair_ordinal'];common=[manifest[j]['rgb_left']['time_ns'] for j in (pair,pair+1)]
            r0,r1=at(reference,common);ref_delta=np.linalg.inv(r0)@r1
            row={'window':window,'pair_ordinal':pair,'source_decision_sequence':decision['sequence'],
                'action':decision['action'],'common_interval_ns':common,'reference_delta':ref_delta.tolist(),'experts':{}}
            for c in events:
                if not decision['availability'][c]:row['experts'][c]=None;continue
                evidence=decision['evidence'][c];index=evidence['motion_event_sequence']
                if type(index) is not int or index>=decision['observed_event_counts'][c]:raise ValueError('future motion feature')
                event=events[c][index];own=[event['time0_ns'],event['time1_ns']]
                expected=[manifest[j][c+'_left']['time_ns'] for j in (pair,pair+1)]
                if own!=expected or not event['first_write_for_submission'] or event['original_need_interp'] is not False or event['observation_monotonic_ns']>decision['controller_monotonic_ns']:
                    raise ValueError('invalid available motion evidence')
                a,b=at(reference,own);d=body_delta(pose_matrix(event['derived_camera_delta_xyzw']),es[c])
                aligned=transport(d,r0,a,b,r1)
                row['experts'][c]={'sensor_interval_ns':own,'motion_event_sequence':index,
                    'nominal_body_from_camera':es[c].tolist(),'own_body_delta':d.tolist(),'common_body_delta':aligned.tolist(),
                    'own_loss':losses(d,np.linalg.inv(a)@b),'common_loss':losses(aligned,ref_delta)}
            rows.append(row)
    report={'scope':protocol['scope'],'rows':rows,'windows':summarize(rows)}
    (output/'pose-proxy.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    if any(sha(root/rel)!=digest for rel,digest in frozen.items()):raise ValueError('source changed during evaluation')
    print(json.dumps(report['windows'],indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();run(a.root,a.output)
