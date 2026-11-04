"""EXP138 frozen offline common-time assumption study; no reference reader."""
import argparse
from collections import Counter
from copy import deepcopy
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from common_time_model import identity_assumption, integrate, transport
from selected_motion_study import read, sha, source_frame, write

ROOT = Path(__file__).resolve().parents[1]
PLAN = ROOT/'experiments/common-time-model-2026-11-04.json'
OUT = ROOT/'results/common-time-model/run-001'
MODES = ['fixed_rgb', 'fixed_thermal', 'condition_prior', 'alternating_control']


def freeze():
    parent = read(ROOT/'experiments/droid-longer-prefix-2026-11-04.json')
    cases = []
    files = [Path(__file__), ROOT/'tools/common_time_model.py', ROOT/'tools/common_time_audit.py',
             ROOT/'tools/tests/test_common_time_model.py', ROOT/'tools/selected_motion_study.py',
             ROOT/'tools/selected_motion_trajectory.py',
             ROOT/'MAC-VO-main/Scripts/Setup/causal_interval_model.py',
             ROOT/'results/selected-motion-publisher/source-001/MS2_dataset.py',
             ROOT/'experiments/independent-benchmark-2026-11-04.json',
             ROOT/'experiments/droid-longer-prefix-2026-11-04.json',
             ROOT/'results/droid-longer-prefix/run-001/audit.json']
    for r in parent['runs']:
        if r['modality'] != 'rgb': continue
        estimates = {m: f"results/droid-longer-prefix/run-001/captures/{r['condition']}-{m}/estimate.json"
                     for m in ['rgb', 'thermal']}
        case = {k: r[k] for k in ['condition', 'recording', 'manifest', 'calibration']}
        case['estimates'] = estimates; cases.append(case)
        files += [ROOT/r['manifest'], ROOT/r['calibration']]
        files += [ROOT/p for p in estimates.values()]
    p = {'date': '2026-11-04', 'parent_commit': 'c10eedb744d3cd5e5401f65288b270898face426',
         'runtime_image': parent['runtime_image'], 'cases': cases, 'modes': MODES,
         'condition_prior': {'day': 'rgb', 'night': 'thermal', 'rain': 'rgb'},
         'association': 'same original ordinal adjacent stems only; no time search',
         'target': 'original rgb left timestamps', 'frames': 101,
         'clock_contract': 'explicit provided-ns identity assumption; zero offset, unit rate',
         'horizon': 'one native duration outside either native endpoint',
         'coarse_fine': 'all 99 coarse i->i+2 predictions transported to own i->i+1',
         'audit_matrix_atol': 2e-8, 'no_fitted_parameters': True,
         'reference_scoring': False, 'protected_payload_access': False,
         'new_native_inference': False, 'benchmark_execution_ready': False,
         'source_sha256': {str(f.relative_to(ROOT)): sha(f) for f in files}}
    OUT.mkdir(parents=True, exist_ok=False)
    write(PLAN, p); write(OUT/'plan.json', p)
    print('frozen', len(p['source_sha256']), 'pins')


def verify():
    p = read(PLAN)
    if sha(PLAN) != sha(OUT/'plan.json'): raise ValueError('plan changed')
    for path, expected in p['source_sha256'].items():
        if sha(ROOT/path) != expected: raise ValueError('changed input '+path)
    return p


def clock(case, camera):
    return case['recording']+'/'+camera+'_left_recorded_clock'


def sources(case):
    s = read(ROOT/case['manifest'])
    values = {m: read(ROOT/f) for m, f in case['estimates'].items()}
    ids = [r['stem'] for r in s['records']]
    for m, v in values.items():
        times = [r['cameras']['rgb' if m == 'rgb' else 'thr']['left']['recorded_time_ns']
                 for r in s['records']]
        if v['time_ns'] != times or v['source_ids'] != ids:
            raise ValueError('native source axis differs')
        if len(times) != 101 or v['online_output_latency_available'] is not False:
            raise ValueError('wrong native scope/length')
    return ids, values


def delta(v, i, j):
    if (v['poses'][i] is None or v['poses'][j] is None
            or v['segments'][i] is None or v['segments'][i] != v['segments'][j]):
        return None
    return np.linalg.solve(np.asarray(v['poses'][i]), np.asarray(v['poses'][j]))


def edge(case, ids, values, e, camera, i):
    v = values[camera]; d = delta(v, i, i+1)
    native = v['time_ns'][i:i+2]; target = values['rgb']['time_ns'][i:i+2]
    source_clock, target_clock = clock(case, camera), clock(case, 'rgb')
    c = identity_assumption(source_clock, target_clock) if camera == 'thermal' else None
    if d is None:
        out = {'accepted': False, 'reason': 'missing_native_motion', 'delta_target': None,
               'native_interval_ns': native, 'target_interval_ns': target}
    else:
        out = transport(d.tolist(), native, target, source_clock, target_clock,
                        (np.eye(4) if camera == 'rgb' else e).tolist(), c)
    out.update(camera=camera, source_indices=[i, i+1], source_ids=ids[i:i+2],
               native_segment_ids=v['segments'][i:i+2],
               frame_evidence_ref=sha(ROOT/case['calibration'])+'/official_set_extrinsics',
               availability={'kind': 'offline_native_termination_only',
                             'estimate_sha256': sha(ROOT/case['estimates'][camera]),
                             'native_wall_runtime_ns': v['wall_runtime_ns'],
                             'elapsed_clock_id': 'exp137/'+case['condition']+'/'+camera+'/local_runtime',
                             'shared_controller_deadline_available': False,
                             'per_frame_output_receipt_available': False})
    return out


def replay():
    p = verify(); summary = []
    for case in p['cases']:
        ids, values = sources(case); e = source_frame(read(ROOT/case['calibration']))
        edges = {m: [edge(case, ids, values, e, m, i) for i in range(100)]
                 for m in ['rgb', 'thermal']}
        for mode in p['modes']:
            def choose(i):
                if mode == 'fixed_rgb': return 'rgb'
                if mode == 'fixed_thermal': return 'thermal'
                if mode == 'condition_prior': return p['condition_prior'][case['condition']]
                return 'rgb' if i % 2 == 0 else 'thermal'
            rows = [deepcopy(edges[choose(i)][i]) for i in range(100)]
            result = integrate(rows)
            result.update(time_ns=values['rgb']['time_ns'], source_ids=ids, records=rows,
                          mode=mode, condition=case['condition'],
                          target_frame='provided_rgb_rectified_left_camera')
            write(OUT/(case['condition']+'-'+mode+'.json'), result)
            summary.append({'condition': case['condition'], 'mode': mode,
                            'edges': dict(Counter(r['reason'] for r in rows)),
                            'model_gauge_anchors': len(result['gauge_roots'])})
        diagnostics = []
        for camera, v in values.items():
            transform = np.eye(4) if camera == 'rgb' else e
            for i in range(99):
                coarse = delta(v, i, i+2); fine = delta(v, i, i+1)
                if coarse is None or fine is None:
                    diagnostics.append({'camera': camera, 'source_index': i, 'valid': False})
                    continue
                proj = transport(coarse.tolist(), v['time_ns'][i:i+3:2],
                                 v['time_ns'][i:i+2], clock(case, camera), clock(case, camera),
                                 transform.tolist())
                common_fine = transform@fine@np.linalg.inv(transform)
                error = np.linalg.solve(common_fine, np.asarray(proj['delta_target']))
                diagnostics.append({'camera': camera, 'source_index': i, 'valid': True,
                                    'projection': proj,
                                    'translation_disagreement_m': float(np.linalg.norm(error[:3, 3])),
                                    'rotation_disagreement_deg': float(np.degrees(Rotation.from_matrix(error[:3, :3]).magnitude()))})
        write(OUT/(case['condition']+'-coarse-fine.json'), {
            'scope': 'predicted motion self-consistency only; not truth, accuracy or selection evidence',
            'rows': diagnostics})
    write(OUT/'summary.json', summary)
    print(summary)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('command', choices=['freeze', 'verify', 'replay'])
    command = parser.parse_args().command
    if command == 'verify': print('pins verified', len(verify()['source_sha256']))
    else: globals()[command]()
