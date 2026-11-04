"""EXP135 source qualification and reference-free replay of saved EXP134 outputs."""
import argparse
import ast
from copy import deepcopy
import hashlib
import json
import os.path
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from selected_motion_trajectory import publish

ROOT = Path(__file__).resolve().parents[1]
PLAN = ROOT/'experiments/selected-motion-publisher-2026-11-04.json'
OUT = ROOT/'results/selected-motion-publisher/run-001'
SOURCE = ROOT/'results/selected-motion-publisher/source-001/MS2_dataset.py'


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def write(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False);stream.write('\n')


def freeze():
    parent = read(ROOT/'experiments/orb-ms2-development-2026-11-04-v4.json')
    cases = []
    files = [Path(__file__), ROOT/'tools/selected_motion_trajectory.py',
             ROOT/'tools/selected_motion_study_audit.py',
             ROOT/'tools/tests/test_selected_motion_trajectory.py', SOURCE,
             ROOT/'experiments/independent-benchmark-2026-11-04.json',
             ROOT/'results/orb-ms2-development/run-004/audit.json']
    for case in parent['cases']:
        estimates = {c: f"results/orb-ms2-development/run-004/{case['condition']}-{m}/estimate.json"
                     for c,m in [('rgb','rgb'),('thermal','thr')]}
        cases.append({**case, 'estimates': estimates})
        files += [ROOT/case['manifest'], ROOT/case['calibration']]
        files += [ROOT/p for p in estimates.values()]
    plan = {'date':'2026-11-04', 'parent_commit':'7071ee4d2e5686d6e27bd618307270a3b30417fe',
            'source_sha256':{str(p.relative_to(ROOT)):sha(p) for p in files},
            'cases':cases, 'ms2_source_revision':'0bb0fc1a544d8ad58c7fda5bc025cb0cbb400284',
            'runtime_image':parent['runtime_image'], 'source_frames_per_case':20,
            'modes':['rgb_ownclock','thermal_ownclock','thermal_on_rgb_target','rgb_breaks'],
            'fixed_break_controls':{'abstain_edge':6,'late_edge':10,'reset_edge':14},
            'matrix_reconstruction_atol':2e-5, 'max_source_gap_ns':150_000_000,
            'reference_scoring':False,'new_native_inference':False,
            'protected_payload_access':False, 'benchmark_execution_ready':False,
            'scope':'exact-native publisher engineering; no model-based time transport'}
    OUT.mkdir(parents=True, exist_ok=False)
    write(PLAN, plan);write(OUT/'plan.json',plan)
    print('FROZEN',len(files),'source/input pins before publication')


def verify():
    p=read(PLAN)
    if sha(PLAN)!=sha(OUT/'plan.json'):raise ValueError('plan changed')
    for path,expected in p['source_sha256'].items():
        if sha(ROOT/path)!=expected:raise ValueError('changed frozen input '+path)
    return p


def source_frame(calibration):
    """Run only the pinned official numeric method, with no pickle or Torch import."""
    values={key:np.asarray(value['values']) for key,value in calibration.items()}
    tree=ast.parse(SOURCE.read_text())
    cls,=[n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='DataLoader_MS2']
    method,=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='set_extrinsics']
    # The source method's np.load is intercepted with already hash-pinned numeric
    # JSON. No .npy object unpickling, image, navigation or pose file is accessed.
    class NumericNumpy:
        concatenate=staticmethod(np.concatenate)
        matmul=staticmethod(np.matmul)
        linalg=np.linalg
        @staticmethod
        def load(path,allow_pickle):
            if path!='/provided/sync_data/exposed/calib.npy' or allow_pickle is not True:
                raise ValueError('unexpected official source calibration load')
            return SimpleNamespace(item=lambda:values)
    namespace={'np':NumericNumpy,'torch':SimpleNamespace(as_tensor=np.asarray), 'osp':os.path}
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(SOURCE),'exec'),namespace)
    upstream=namespace['set_extrinsics'](SimpleNamespace(root='/provided',seq_list=['exposed']))
    target=np.eye(4);target[:3]=upstream['THR2RGB']
    a=np.eye(4);b=np.eye(4)
    for matrix,mod in [(a,'rgb'),(b,'thr')]:
        matrix[:3,:3]=values['R_nir2'+mod]
        matrix[:3,3]=values['T_nir2'+mod].reshape(3)/1000
        if not np.array_equal(values['R_'+mod+'L'],np.eye(3)):
            raise ValueError('nonidentity left rectification needs additional source proof')
    independently=a@np.linalg.inv(b)
    if not np.allclose(target,independently,atol=1e-14,rtol=0):
        raise ValueError('source transform direction mismatch')
    return target


def make_contract(plan,case,mode,common_from_thermal):
    source=read(ROOT/case['manifest'])
    selected='thermal' if mode.startswith('thermal') else 'rgb'
    target='rgb' if mode=='thermal_on_rgb_target' else selected
    native=read(ROOT/case['estimates'][selected])
    sensor=lambda camera:case['recording']+'/'+camera+'_left_recorded_clock'
    times=[row['cameras']['rgb' if target=='rgb' else 'thr']['left']['recorded_time_ns']
           for row in source['records']]
    camera_times=[row['cameras']['rgb' if selected=='rgb' else 'thr']['left']['recorded_time_ns']
                  for row in source['records']]
    controller='exp134/'+case['condition']+'/'+selected+'/receipt_elapsed_clock'
    frame='provided_rgb_rectified_left_camera'
    contracts={camera:{'source_frame':camera+'_rectified_left_camera','target_frame':frame,
                'target_from_camera':(np.eye(4) if camera=='rgb' else common_from_thermal).tolist(),
                'frame_evidence_ref':case['calibration_sha256']+'/official_set_extrinsics'}
               for camera in ['rgb','thermal']}
    initialization=None
    if mode!='thermal_on_rgb_target' and native['poses'][0] is not None:
        now=native['rows'][0]['receipt_elapsed_ns']
        initialization={'source_time_ns':times[0],'deadline_ns':now,'received_ns':now,
                        'controller_clock_id':controller,'target_frame':frame,
                        'evidence_ref':sha(ROOT/case['estimates'][selected])+'/frame0'}
    slots=[]
    for i in range(len(times)-1):
        next_row=native['rows'][i+1]
        deadline=next_row['receipt_elapsed_ns']
        valid=(native['poses'][i] is not None and native['poses'][i+1] is not None
               and native['segments'][i]==native['segments'][i+1])
        delta=np.linalg.solve(np.asarray(native['poses'][i]),np.asarray(native['poses'][i+1])).tolist() if valid else None
        payload={'id':f"{case['condition']}/{mode}/{i}",'camera':selected,
                 'available':valid,'reference_transport_used':False,
                 'native_interval_ns':camera_times[i:i+2], 'sensor_clock_id':sensor(selected),
                 'source_frame':selected+'_rectified_left_camera',
                 'native_segment_id':native['segments'][i+1] or 0,
                 'reset_before':False,'delta_camera':delta}
        slot={'target_interval_ns':times[i:i+2],'deadline_ns':deadline,
              'action':selected.upper(),'selected':{'received_ns':deadline,
              'controller_clock_id':controller,'payload':payload}}
        if mode=='rgb_breaks':
            controls=plan['fixed_break_controls']
            if i==controls['abstain_edge']:slot['action']='ABSTAIN'
            if i==controls['late_edge']:slot['selected']['received_ns']=deadline+1
            if i==controls['reset_edge']:payload['reset_before']=True
        slots.append(slot)
    return {'time_ns':times,'source_ids':[r['stem'] for r in source['records']],
            'target_frame':frame,'sensor_clock_id':sensor(target),
            'sensor_clock_evidence_ref':case['manifest_sha256']+'/same_native_stream_only',
            'controller_clock_id':controller,'max_source_gap_ns':plan['max_source_gap_ns'],
            'initialization':initialization,'frame_contracts':contracts,'slots':slots,
            'terminal_success':native['terminal_success'],'mode':mode,
            'physical_acquisition_clock_mapping_verified':False}


def replay():
    plan=verify();qualifications=[];runs=[]
    for case in plan['cases']:
        source=read(ROOT/case['manifest']);calibration=read(ROOT/case['calibration'])
        target=source_frame(calibration)
        r=[row['cameras']['rgb']['left']['recorded_time_ns'] for row in source['records']]
        t=[row['cameras']['thr']['left']['recorded_time_ns'] for row in source['records']]
        qualifications.append({'condition':case['condition'],'recording':case['recording'],
            'provided_rgb_from_thermal':target.tolist(),'provided_source_method_reproduced':True,
            'source_method_sha256':sha(SOURCE),'calibration_sha256':sha(ROOT/case['calibration']),
            'exact_equal_left_timestamps':sum(a==b for a,b in zip(r,t)),
            'exact_equal_adjacent_intervals':sum(r[i:i+2]==t[i:i+2] for i in range(len(r)-1)),
            'thermal_minus_rgb_ns_range':[min(b-a for a,b in zip(r,t)),max(b-a for a,b in zip(r,t))],
            'physical_calibration_verified':False,'cross_stream_clock_mapping_verified':False})
        for mode in plan['modes']:
            name=case['condition']+'-'+mode
            contract=make_contract(plan,case,mode,target)
            path=OUT/(name+'-contract.json');write(path,contract)
            published=publish(contract)
            published.update(input_sha256=sha(path),publisher_sha256=sha(ROOT/'tools/selected_motion_trajectory.py'))
            result=OUT/(name+'-trajectory.json');write(result,published)
            runs.append({'case':name,'contract':str(path.relative_to(ROOT)),
                         'trajectory':str(result.relative_to(ROOT)), 'trajectory_sha256':sha(result),
                         **{k:published[k] for k in ['source_frames','pose_frames','accepted_motions','segment_count']}})
    write(OUT/'source-qualification.json',{'cases':qualifications,'reference_used':False,
          'claim':'provided numeric frame convention; cross-stream acquisition clock remains unverified'})
    write(OUT/'publication-lock.json',{'plan_sha256':sha(PLAN),'source_qualification_sha256':sha(OUT/'source-qualification.json'),
          'runs':runs,'new_native_inference':False,'reference_scoring':False})
    verify()
    print(json.dumps(runs,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['freeze','replay','verify'])
    args=parser.parse_args();globals()[args.action]()
