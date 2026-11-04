"""Independent EXP135 gauge/clock/segment audit; no publisher helper imports."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/selected-motion-publisher/run-001'
PLAN=ROOT/'experiments/selected-motion-publisher-2026-11-04.json'


def read(path):return json.loads(Path(path).read_text())
def sha(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def check_case(contract,trajectory,native,target_from_camera,mode,source_times,source_ids,tolerance):
    count=0
    def check(value,label):
        nonlocal count
        if not value:raise ValueError(label)
        count+=1
    check(all(type(t) is int for t in trajectory['time_ns']),'strict integer output times')
    check(trajectory['time_ns']==source_times,'exact expected source time axis')
    check(trajectory['source_ids']==source_ids,'exact source IDs')
    check(len(trajectory['poses'])==len(trajectory['segments'])==len(source_times),'all source slots retained')
    check(trajectory['reference_used'] is False and trajectory['time_transport_used'] is False,'no reference/time transport')
    check(trajectory['physical_frame_or_clock_verified'] is False,'no physical verification promotion')
    check(trajectory['terminal_success']==native['terminal_success'],'native terminal status preserved')
    for index,(pose,segment) in enumerate(zip(trajectory['poses'],trajectory['segments'])):
        missing=(mode=='thermal_on_rgb_target' or (mode=='rgb_breaks' and index in [7,11]))
        if missing:
            check(pose is None and segment is None,'missing/rejected motion not filled')
            continue
        if mode=='rgb_breaks':
            if index<=6:root,sid=0,0
            elif index<=10:root,sid=7,1
            elif index<=14:root,sid=11,2
            else:root,sid=14,3
        else:root,sid=0,0
        check(type(segment) is int and segment==sid,'independent expected segment')
        # Direct absolute-pose gauge identity, not recursive publisher composition.
        first=np.asarray(native['poses'][root]);current=np.asarray(native['poses'][index])
        expected=target_from_camera@np.linalg.solve(first,current)@np.linalg.inv(target_from_camera)
        check(np.allclose(pose,expected,rtol=0,atol=tolerance),'native-gauge reconstruction')
        matrix=np.asarray(pose)
        check(np.array_equal(matrix[3],[0,0,0,1]),'homogeneous row')
        check(np.linalg.norm(matrix[:3,:3].T@matrix[:3,:3]-np.eye(3))<1e-5,'proper output rotation')
        check(abs(np.linalg.det(matrix[:3,:3])-1)<1e-5,'proper determinant')
    for index,(slot,row) in enumerate(zip(contract['slots'],trajectory['decisions'])):
        check(row['target_interval_ns']==source_times[index:index+2],'decision endpoint identity')
        check(row['action']==slot['action'],'no action fallback')
        accepted=mode!='thermal_on_rgb_target' and not(mode=='rgb_breaks' and index in [6,10])
        check(row['accepted'] is accepted,'independent acceptance population')
        if accepted:
            check(row['source']['received_ns']<=slot['deadline_ns'],'original deadline')
            check(row['source']['id']==slot['selected']['payload']['id'],'source lineage')
            actual=np.asarray(slot['selected']['payload']['delta_camera'])
            expected=np.linalg.solve(np.asarray(native['poses'][index]),np.asarray(native['poses'][index+1]))
            check(np.allclose(actual,expected,rtol=0,atol=1e-12),'exact native source delta')
            converted=target_from_camera@expected@np.linalg.inv(target_from_camera)
            check(np.allclose(row['delta_target'],converted,rtol=0,atol=tolerance),'independent frame conjugation')
        elif mode=='thermal_on_rgb_target':
            check(row['reason']=='native_interval_mismatch','unmatched thermal interval refusal')
        elif index==6:check(row['reason']=='advisory_abstain','fixed missing motion')
        else:check(row['reason']=='late_selected_motion','fixed late result')
    expected_poses=0 if mode=='thermal_on_rgb_target' else 18 if mode=='rgb_breaks' else 20
    expected_segments=0 if mode=='thermal_on_rgb_target' else 4 if mode=='rgb_breaks' else 1
    check(trajectory['pose_frames']==expected_poses,'pose count')
    check(trajectory['segment_count']==expected_segments,'segment count')
    check(trajectory['accepted_motions']==(0 if mode=='thermal_on_rgb_target' else 17 if mode=='rgb_breaks' else 19),'motion count')
    roots=trajectory['segment_roots']
    expected_root_indices=[] if mode=='thermal_on_rgb_target' else [0,7,11,14] if mode=='rgb_breaks' else [0]
    check([r['root_time_ns'] for r in roots]==[source_times[i] for i in expected_root_indices],'explicit segment gauge roots')
    check([r['root_published'] for r in roots]==([True]+[False]*(len(roots)-1) if roots else []),'unpublished restart gauge not backfilled')
    return count


def main(output):
    plan=read(PLAN);lock=read(OUT/'publication-lock.json');checks=0;mutations=[];summary=[]
    assert sha(PLAN)==sha(OUT/'plan.json')==lock['plan_sha256']
    for name,digest in plan['source_sha256'].items():assert sha(ROOT/name)==digest,name
    assert sha(OUT/'source-qualification.json')==lock['source_qualification_sha256']
    qualified=read(OUT/'source-qualification.json')['cases']
    for case in plan['cases']:
        source=read(ROOT/case['manifest']);calibration=read(ROOT/case['calibration'])
        raw={k:np.asarray(v['values'],dtype=float) for k,v in calibration.items()}
        rotation=raw['R_nir2rgb']@raw['R_nir2thr'].T
        target=np.eye(4);target[:3,:3]=rotation
        target[:3,3]=(raw['T_nir2rgb']-rotation@raw['T_nir2thr']).reshape(3)*.001
        qualification=next(r for r in qualified if r['condition']==case['condition'])
        assert np.allclose(target,qualification['provided_rgb_from_thermal'],atol=1e-14,rtol=0)
        assert qualification['physical_calibration_verified'] is False
        assert qualification['cross_stream_clock_mapping_verified'] is False
        rgb=[r['cameras']['rgb']['left']['recorded_time_ns'] for r in source['records']]
        thermal=[r['cameras']['thr']['left']['recorded_time_ns'] for r in source['records']]
        assert sum(a==b for a,b in zip(rgb,thermal))==qualification['exact_equal_left_timestamps']==0
        assert sum(rgb[i:i+2]==thermal[i:i+2] for i in range(19))==qualification['exact_equal_adjacent_intervals']==0
        checks+=6
        for mode in plan['modes']:
            label=case['condition']+'-'+mode
            row=next(r for r in lock['runs'] if r['case']==label)
            contract=read(ROOT/row['contract']);trajectory=read(ROOT/row['trajectory'])
            assert sha(ROOT/row['trajectory'])==row['trajectory_sha256']
            assert sha(ROOT/row['contract'])==trajectory['input_sha256']
            selected='thermal' if mode.startswith('thermal') else 'rgb'
            native=read(ROOT/case['estimates'][selected])
            times=rgb if mode=='thermal_on_rgb_target' or selected=='rgb' else thermal
            frame=target if selected=='thermal' else np.eye(4)
            ids=[r['stem'] for r in source['records']]
            check=lambda t:check_case(contract,t,native,frame,mode,times,ids,plan['matrix_reconstruction_atol'])
            checks+=check(trajectory)
            alterations=[('float_timestamp',lambda x:x['time_ns'].__setitem__(0,float(x['time_ns'][0])))]
            if mode=='rgb_ownclock':
                alterations.append(('wrong_direction',lambda x:x['poses'].__setitem__(5,np.linalg.inv(np.asarray(x['poses'][5])).tolist())))
            if mode=='rgb_breaks':
                alterations += [('filled_missing_pose',lambda x:x['poses'].__setitem__(7,np.eye(4).tolist())),
                                ('resumed_segment',lambda x:x['segments'].__setitem__(8,0))]
            if mode=='thermal_on_rgb_target':
                alterations.append(('silent_thermal_fallback',lambda x:x['poses'].__setitem__(1,np.eye(4).tolist())))
            for label2,modify in alterations:
                changed=deepcopy(trajectory);modify(changed)
                try:check(changed)
                except ValueError:mutations.append(label+'/'+label2)
                else:raise ValueError('auditor failed to reject '+label2)
            summary.append({'case':label,**{k:trajectory[k] for k in ['source_frames','pose_frames','accepted_motions','segment_count']}})
    report={'pass':True,'independent_checks':checks,'semantic_mutations_rejected':len(mutations),
            'mutations':mutations,'runs':summary,'new_native_inference':False,'reference_scoring':False,
            'protected_payload_access':False,'benchmark_execution_ready':False}
    with Path(output).open('x') as stream:json.dump(report,stream,indent=2);stream.write('\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();main(args.output)
