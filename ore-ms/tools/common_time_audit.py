"""Independent EXP138 audit: matrix log/exponential, no production helper imports."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.linalg import expm, logm
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results/common-time-model/run-001'
PLAN = ROOT/'experiments/common-time-model-2026-11-04.json'
CHECKS = 0


def read(path): return json.loads(Path(path).read_text())
def sha(path):
    with Path(path).open('rb') as f: return hashlib.file_digest(f, 'sha256').hexdigest()


def check(ok, message):
    global CHECKS
    CHECKS += 1
    if not ok: raise ValueError(message)


def close(a, b):
    return np.allclose(a, b, atol=2e-8, rtol=0)


def normalize(a):
    a = np.asarray(a, dtype=float).copy()
    u, _, vt = np.linalg.svd(a[:3, :3]); a[:3, :3] = u@np.diag([1, 1, np.linalg.det(u@vt)])@vt
    return a


def frame(case):
    c = read(ROOT/case['calibration'])
    def block(m):
        x = np.eye(4); x[:3, :3] = c['R_nir2'+m]['values']
        x[:3, 3] = np.asarray(c['T_nir2'+m]['values']).reshape(3)/1000
        check(np.array_equal(c['R_'+m+'L']['values'], np.eye(3)), 'rectification')
        return x
    return block('rgb')@np.linalg.inv(block('thr'))


def prepare(case):
    s = read(ROOT/case['manifest'])
    ids = [r['stem'] for r in s['records']]
    vs = {m: read(ROOT/path) for m, path in case['estimates'].items()}
    for m, v in vs.items():
        times = [r['cameras']['rgb' if m == 'rgb' else 'thr']['left']['recorded_time_ns'] for r in s['records']]
        check(v['time_ns'] == times and v['source_ids'] == ids and len(ids) == 101, 'source axis')
    return ids, vs, frame(case)


def expected(case, vs, e, m, i, j, target):
    v = vs[m]; native = [v['time_ns'][i], v['time_ns'][j]]
    if v['poses'][i] is None or v['poses'][j] is None or v['segments'][i] != v['segments'][j]:
        return native, None, None
    d = normalize(np.linalg.inv(v['poses'][i])@v['poses'][j])
    e = normalize(np.eye(4) if m == 'rgb' else e)
    common = normalize(e@d@np.linalg.inv(e))
    a, b = native; r0, r1 = target
    outside = max(a-r0, r0-b, a-r1, r1-b, 0)
    if outside > b-a: return native, d, None
    projected = expm(logm(common)*((r1-r0)/(b-a)))
    check(np.max(np.abs(np.imag(projected))) < 1e-10, 'real principal branch')
    return native, d, np.real(projected)


def projection(row, native, d, target, projected, source_clock, target_clock, e):
    check(row['native_interval_ns'] == native and row['target_interval_ns'] == target, 'original intervals')
    check(all(type(t) is int for t in row['native_interval_ns']+row['target_interval_ns']), 'exact integers')
    check(row['source_clock_id'] == source_clock and row['target_clock_id'] == target_clock, 'clock ids')
    check(row['model'] == 'stationary_body_twist_v1', 'model identity')
    for flag in ['admission_ready', 'physical_frame_verified', 'sensor_clock_verified',
                 'controller_deadline_verified', 'benchmark_execution_ready', 'reference_used',
                 'online_output_latency_available']:
        check(row[flag] is False, 'no false admission '+flag)
    c = row['clock_contract']
    if source_clock == target_clock:
        check(c is None, 'same clock no assumption')
    else:
        check(c['kind'] == 'provided_ns_identity_assumption_v1', 'assumption tag')
        check(c['source_clock_id'] == source_clock and c['target_clock_id'] == target_clock, 'assumption domains')
        for k, v in [('offset_ns', 0), ('rate_numerator', 1), ('rate_denominator', 1)]:
            check(type(c[k]) is int and c[k] == v, 'unfitted identity')
        check(c['physically_verified'] is False and bool(c['evidence_ref']), 'no clock proof')
    a, b = native; r0, r1 = target
    check(row['duration_ratio'] == (r1-r0)/(b-a), 'duration ratio')
    check(row['max_outside_native_ns'] == max(a-r0, r0-b, a-r1, r1-b, 0), 'outside')
    check(row['horizon_ns'] == b-a, 'horizon')
    check(close(row['delta_camera'], d) and close(row['target_from_camera'], e), 'geometry provenance')
    check(row['accepted'] is (projected is not None), 'acceptance')
    if projected is None:
        check(row['reason'] == 'horizon_exceeded' and row['delta_target'] is None, 'horizon hole')
    else:
        exact = source_clock == target_clock and native == target
        check(row['transport_kind'] == ('exact_native' if exact else 'conditional_model'), 'transport classification')
        check(row['reason'] == ('exact_native' if exact else 'conditional_model_only'), 'reason')
        check(close(row['delta_target'], projected), 'independent matrix exponential')


def trajectory(out, case, mode, plan, data):
    ids, vs, e = data
    check(out['source_ids'] == ids and out['time_ns'] == vs['rgb']['time_ns'], 'target axis')
    check(out['mode'] == mode and out['condition'] == case['condition'], 'case identity')
    check(out['initialization'] == 'arbitrary_model_gauge_not_measured_at_target_time', 'gauge label')
    check(out['target_frame'] == 'provided_rgb_rectified_left_camera', 'target frame')
    for k in ['benchmark_execution_ready', 'physical_frame_or_clock_verified', 'online_output_latency_available', 'reference_used']:
        check(out[k] is False, 'trajectory flag '+k)
    check(len(out['records']) == 100 and len(out['poses']) == 101 and len(out['segments']) == 101, 'lengths')
    check(close(out['poses'][0], np.eye(4)) and out['segments'][0] == 0, 'model initial gauge')
    current = np.eye(4); active = True; seg = 0
    roots = [{'segment': 0, 'edge': 0, 'published_gauge_anchor': True}]
    for i, row in enumerate(out['records']):
        m = ('rgb' if mode == 'fixed_rgb' else 'thermal' if mode == 'fixed_thermal'
             else plan['condition_prior'][case['condition']] if mode == 'condition_prior'
             else 'rgb' if i % 2 == 0 else 'thermal')
        check(row['camera'] == m and row['source_indices'] == [i, i+1] and row['source_ids'] == ids[i:i+2], 'ordinal lineage')
        target = vs['rgb']['time_ns'][i:i+2]
        native, d, projected = expected(case, vs, e, m, i, i+1, target)
        projection(row, native, d, target, projected, case['recording']+'/'+m+'_left_recorded_clock',
                   case['recording']+'/rgb_left_recorded_clock', np.eye(4) if m == 'rgb' else e)
        check(row['native_segment_ids'] == vs[m]['segments'][i:i+2], 'native segments')
        check(row['frame_evidence_ref'] == sha(ROOT/case['calibration'])+'/official_set_extrinsics', 'frame source')
        av = row['availability']
        check(av == {'kind': 'offline_native_termination_only', 'estimate_sha256': sha(ROOT/case['estimates'][m]),
                     'native_wall_runtime_ns': vs[m]['wall_runtime_ns'],
                     'elapsed_clock_id': 'exp137/'+case['condition']+'/'+m+'/local_runtime',
                     'shared_controller_deadline_available': False, 'per_frame_output_receipt_available': False}, 'offline provenance')
        if projected is None:
            check(out['poses'][i+1] is None and out['segments'][i+1] is None, 'unfilled hole')
            active = False
        else:
            if not active:
                seg += 1; current = np.eye(4); active = True
                roots.append({'segment': seg, 'edge': i, 'published_gauge_anchor': False})
            current = current@projected
            check(close(out['poses'][i+1], current) and out['segments'][i+1] == seg, 'right composition/segments')
    check(out['gauge_roots'] == roots, 'gauge roots')


def run():
    plan = read(PLAN)
    check(sha(PLAN) == sha(OUT/'plan.json'), 'frozen plan')
    for path, digest in plan['source_sha256'].items(): check(sha(ROOT/path) == digest, 'pin '+path)
    summaries = []; mutations = 0
    for case in plan['cases']:
        data = prepare(case); ids, vs, e = data
        for mode in plan['modes']:
            out = read(OUT/(case['condition']+'-'+mode+'.json'))
            trajectory(out, case, mode, plan, data)
            if mode == 'fixed_thermal':
                def alter(path, value):
                    x = deepcopy(out); cursor = x
                    for k in path[:-1]: cursor = cursor[k]
                    cursor[path[-1]] = value
                    return x
                controls = [(['records', 0, 'native_interval_ns', 0], out['time_ns'][0]),
                            (['records', 0, 'native_interval_ns', 0], float(out['records'][0]['native_interval_ns'][0])),
                            (['records', 0, 'clock_contract', 'offset_ns'], 1),
                            (['records', 0, 'model'], 'exact'),
                            (['records', 0, 'sensor_clock_verified'], True),
                            (['records', 0, 'transport_kind'], 'exact_native'),
                            (['records', 0, 'availability', 'per_frame_output_receipt_available'], True),
                            (['records', 0, 'source_indices'], [1, 2]),
                            (['poses', 1, 0, 3], 100.),
                            (['segments', 1], 99)]
                for path, value in controls:
                    try: trajectory(alter(path, value), case, mode, plan, data)
                    except (ValueError, TypeError, KeyError, AssertionError): mutations += 1
                    else: raise ValueError('mutation escaped '+str(path))
        diag = read(OUT/(case['condition']+'-coarse-fine.json'))
        check('not truth' in diag['scope'] and len(diag['rows']) == 198, 'diagnostic scope/length')
        check([(r['camera'], r['source_index']) for r in diag['rows']] ==
              [(m, i) for m in ['rgb', 'thermal'] for i in range(99)], 'complete diagnostic population')
        for row in diag['rows']:
            check(row['valid'] is True, 'finite diagnostic')
            m, i = row['camera'], row['source_index']; v = vs[m]
            native, d, projected = expected(case, vs, e, m, i, i+2, v['time_ns'][i:i+2])
            projection(row['projection'], native, d, v['time_ns'][i:i+2], projected,
                       case['recording']+'/'+m+'_left_recorded_clock', case['recording']+'/'+m+'_left_recorded_clock',
                       np.eye(4) if m == 'rgb' else e)
            fine = np.linalg.inv(v['poses'][i])@v['poses'][i+1]; ef = np.eye(4) if m == 'rgb' else e
            error = np.linalg.inv(ef@fine@np.linalg.inv(ef))@projected
            check(abs(row['translation_disagreement_m']-np.linalg.norm(error[:3, 3])) < 2e-8, 'translation diagnostic')
            check(abs(row['rotation_disagreement_deg']-np.degrees(Rotation.from_matrix(error[:3, :3]).magnitude())) < 2e-6, 'rotation diagnostic')
        for m in ['rgb', 'thermal']:
            rows = [r for r in diag['rows'] if r['camera'] == m]
            summaries.append({'condition': case['condition'], 'camera': m, 'count': len(rows),
                              **{k: dict(zip(['median', 'p95', 'max'], np.percentile([r[k] for r in rows], [50, 95, 100]).tolist()))
                                 for k in ['translation_disagreement_m', 'rotation_disagreement_deg']}})
    report = {'passed': True, 'checks': CHECKS, 'semantic_mutations_rejected': mutations,
              'diagnostic_scope': 'predicted self-consistency; not actual motion error or selection evidence',
              'coarse_fine_summary': summaries, 'reference_scoring': False,
              'benchmark_execution_ready': False}
    with (OUT/'audit.json').open('x') as f: json.dump(report, f, indent=2, allow_nan=False); f.write('\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__': run()
