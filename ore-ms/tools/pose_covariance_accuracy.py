"""Locked, label-free pose combination; reference scoring is a separate step.

Existing MAC/MACT native poses are controls, not an unweighted baseline. Replay
poses come from the separate Gaussian 3D backend. Covariance-intersection means
are approximate in a fixed RGB-anchored common tangent; no global consistency
or calibration claim is inferred from the local construction.
"""
import argparse
from collections import Counter
from datetime import datetime
import hashlib
import json
from pathlib import Path
from zoneinfo import ZoneInfo

import numpy as np

from pose_covariance_replay import (covariance_intersection, exp_se3, json_safe,
    log_se3, pose_xyzw, spd, transport_covariance, validate_transform)
from rotation_signal_audit import feature_vector, fixed_choices

METHODS = ('native_rgb', 'native_thermal', 'native_equal', 'native_angular',
           'replay_rgb', 'replay_thermal', 'replay_equal', 'replay_angular',
           'replay_scalar', 'replay_full')


def sha(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as f:
        json.dump(json_safe(value), f, indent=2, allow_nan=False)
        f.write('\n')


def transforms(calibration, official_loader):
    """Use the same hash-verified official numeric extrinsic as prior scoring."""
    import selected_motion_study
    loader = Path(official_loader)
    if sha(loader) != '299250fcdc33548b6cab3bf585d3f5b205af3635eb038e7e113fee73447de9a1':
        raise ValueError('unrecognized official frame conversion source')
    old = selected_motion_study.SOURCE
    selected_motion_study.SOURCE = loader
    try:
        thermal_from = selected_motion_study.source_frame(calibration)
    finally:
        selected_motion_study.SOURCE = old
    q = np.eye(4)
    q[:3, :3] = [[0, 1, 0], [0, 0, 1], [1, 0, 0]]
    return {'rgb': q, 'thermal': thermal_from@q}


def mapping(t, frame, ratio):
    validate_transform(t)
    validate_transform(frame)
    if not np.isfinite(ratio) or ratio <= 0:
        raise ValueError('positive unfitted duration ratio required')
    return exp_se3(ratio*log_se3(frame@t@np.linalg.inv(frame)))


def project(t, covariance, frame, native, common):
    if (len(native) != 2 or len(common) != 2
            or any(type(x) is not int for x in native+common)
            or native[0] >= native[1] or common[0] >= common[1]):
        raise ValueError('strict positive exact integer source intervals required')
    a, b = native
    r0, r1 = common
    outside = max(a-r0, r0-b, a-r1, r1-b, 0)
    if outside > b-a:
        return {'valid': False, 'reason': 'horizon_exceeded', 'transform': None, 'covariance': None}
    ratio = (r1-r0)/(b-a)
    fn = lambda value: mapping(value, frame, ratio)
    result = {'valid': True, 'reason': 'nominal_stationary_twist', 'transform': fn(t),
              'covariance': None, 'duration_ratio': ratio, 'outside_ns': outside}
    if covariance is not None:
        pushed, j = transport_covariance(fn, t, covariance)
        result['covariance'] = (pushed+pushed.T)/2
        result['transport_jacobian'] = j
    return result


def log_jacobian(t, base, epsilon=1e-6):
    """Derivative of Log(Exp(delta)*T*base^-1), not a left covariance."""
    inverse = np.linalg.inv(base)
    return np.column_stack([
        (log_se3(exp_se3(epsilon*e)@t@inverse)
         - log_se3(exp_se3(-epsilon*e)@t@inverse))/(2*epsilon)
        for e in np.eye(6)])


def block_isotropic(c):
    c = spd(c)
    if c.shape != (6, 6):
        raise ValueError('6x6 covariance required')
    return np.diag([np.trace(c[:3, :3])/3]*3+[np.trace(c[3:, 3:])/3]*3)


def combine(a, b, ca=None, cb=None, mode='equal'):
    if mode not in ('equal', 'scalar', 'full'):
        raise ValueError('unknown combination mode')
    if a is None and b is None:
        return {'transform': None, 'strategy': 'unavailable'}
    if a is None or b is None:
        value = b if a is None else a
        validate_transform(value)
        return {'transform': value, 'strategy': 'sole_camera', 'camera': 'thermal' if a is None else 'rgb'}
    validate_transform(a)
    validate_transform(b)
    mu = log_se3(b@np.linalg.inv(a))
    if np.linalg.norm(mu[3:]) >= np.pi-1e-5:
        raise ValueError('ambiguous rotation-log branch')
    equal = exp_se3(.5*mu)@a
    if mode == 'equal' or ca is None or cb is None:
        return {'transform': equal, 'strategy': 'equal' if mode == 'equal' else 'equal_missing_covariance'}
    ca, cb = spd(ca), spd(cb)
    if mode == 'scalar':
        ca, cb = block_isotropic(ca), block_isotropic(cb)
    jr, jt = log_jacobian(a, a), log_jacobian(b, a)
    ci = covariance_intersection(np.zeros(6), jr@ca@jr.T, mu, jt@cb@jt.T)
    return {'transform': exp_se3(ci['mean'])@a, 'strategy': mode+'_local_ci',
            'rgb_weight': ci['weight'], 'tangent_anchor': 'rgb_motion',
            'covariance_assumption': 'local_gaussian_marginals_unknown_cross_correlation'}


def choice_prediction(experts, choice):
    available = [c for c, e in experts.items() if e['valid']]
    selected = choice if choice in available else available[0] if len(available) == 1 else None
    return {'transform': experts[selected]['transform'] if selected else None,
            'strategy': 'chosen_camera' if selected else 'unavailable', 'camera': selected}


def completed_receipt_valid(health, motion, native):
    """Completed-output accuracy accepts explicitly labelled final writes too."""
    return (health is not None and motion is not None
            and motion['stage'] in ('pair_processing', 'termination')
            and motion['original_need_interp'] is False
            and [motion['time0_ns'], motion['time1_ns']] == native)


def predictions(source_path, calibration_path, capture, replay_paths, official_loader, groups=600):
    source, calibration = read(source_path), read(calibration_path)
    if len(source['records']) < groups or groups < 2:
        raise ValueError('requested exact source prefix absent')
    if [r['stem'] for r in source['records'][:groups]] != [f'{i:06}' for i in range(groups)]:
        raise ValueError('original zero-based source IDs required')
    frames = transforms(calibration, official_loader)
    pins = {str(p): sha(p) for p in [Path(source_path), Path(calibration_path), Path(official_loader)]}
    events, replays = {}, {}
    focal = {}
    for mode, camera in [('rgb', 'rgb'), ('thermal', 'thr')]:
        ep = Path(capture)/mode/'arbiter-events/events.jsonl'
        rp = Path(replay_paths[mode])
        pins[str(ep)], pins[str(rp)] = sha(ep), sha(rp)
        raw = [json.loads(x) for x in ep.read_text().splitlines()]
        events[mode] = {}
        for e in raw:
            if e['kind'] == 'pair_end' or (e['kind'] == 'motion_result_written' and e['first_write_for_submission']):
                events[mode].setdefault((e['kind'], e['time1_ns']), []).append(e)
        rd = read(rp)
        # Rebind the replay to the exact same capture, not just matching row IDs.
        if rd['input_sha256'].get(str(ep)) != sha(ep):
            raise ValueError('replay event provenance differs')
        if rd['group_prefix'] != groups or len(rd['rows']) != groups-1:
            raise ValueError('replay does not have the exact declared prefix')
        replays[mode] = rd['rows']
        k = np.asarray(source['records'][0]['cameras'][camera]['K'])
        focal[mode] = float((k[0, 0]**2+k[1, 1]**2)/2)
    rows = []
    for i in range(groups-1):
        records = source['records'][i:i+2]
        common = [r['cameras']['rgb']['left']['recorded_time_ns'] for r in records]
        natives, replayed, raw = {}, {}, {}
        replay_states = {}
        for mode, camera in [('rgb', 'rgb'), ('thermal', 'thr')]:
            native = [r['cameras'][camera]['left']['recorded_time_ns'] for r in records]
            hh = events[mode].get(('pair_end', native[1]), [])
            mm = events[mode].get(('motion_result_written', native[1]), [])
            h = hh[0] if len(hh) == 1 and [hh[0]['time0_ns'], hh[0]['time1_ns']] == native else None
            m = mm[0] if len(mm) == 1 else None
            raw[mode] = {k: h.get(k) if h else None for k in
                         ('grid_trace_q10_q50_q90_px2', 'inbound_count', 'retained_count', 'occupied_3x3_cells')}
            r = replays[mode][i]
            assert r['from_idx'] == i and r['frame_idx'] == i+1
            assert [r['source_time_ns'], r['target_time_ns']] == native
            replay_states[mode] = r['status']
            valid = completed_receipt_valid(h, m, native)
            if not valid:
                natives[mode] = replayed[mode] = {'valid': False, 'reason': 'missing_unsupported_or_invalid_native',
                                               'transform': None, 'covariance': None}
                continue
            natives[mode] = project(pose_xyzw(m['derived_camera_delta_xyzw']), None, frames[mode], native, common)
            natives[mode]['source_write_stage'] = m['stage']
            natives[mode]['native_event_sequence'] = m['sequence']
            if r['status'] in ('converged', 'iteration_limit', 'line_search_stalled'):
                cov = np.asarray(r['covariance']) if r['covariance'] is not None else None
                replayed[mode] = project(np.asarray(r['transform']), cov, frames[mode], native, common)
            else:
                replayed[mode] = {'valid': False, 'reason': 'invalid_replay', 'transform': None, 'covariance': None}
        x, reason = feature_vector(raw, focal)
        choice = fixed_choices({'features': x, 'raw': raw})['angular_uncertainty']
        methods = {}
        for family, experts in [('native', natives), ('replay', replayed)]:
            for camera in ['rgb', 'thermal']:
                methods[family+'_'+camera] = {'transform': experts[camera]['transform'], 'strategy': camera+'_only'}
            a, b = (experts[c]['transform'] if experts[c]['valid'] else None for c in ['rgb', 'thermal'])
            methods[family+'_equal'] = combine(a, b)
            methods[family+'_angular'] = choice_prediction(experts, choice)
            if family == 'replay':
                for mode in ['scalar', 'full']:
                    methods['replay_'+mode] = combine(a, b, experts['rgb']['covariance'], experts['thermal']['covariance'], mode)
        rows.append({'pair': i, 'source_ids': [r['stem'] for r in records], 'common_interval_ns': common,
                     'native_experts': natives, 'replay_experts': replayed, 'replay_status': replay_states,
                     'angular_choice': choice, 'angular_feature_reason': reason, 'methods': methods})
    assert all(sha(p) == h for p, h in pins.items())
    return {'created_at': datetime.now(ZoneInfo('America/New_York')).isoformat(),
            'recording': source['sequence'], 'groups': groups, 'rows': rows, 'input_sha256': pins,
            'reference_used': False, 'runtime_metrics': False,
            'scope': 'development_nominal_RGB_optical_common_intervals_no_physical_frame_clock_claim',
            'completed_output_policy': 'include supported exact first writes from pair_processing or termination; no online timeliness claim',
            'native_backend': 'existing_covariance_weighted_disp_Huber_not_unweighted',
            'replay_backend': 'separate_Gaussian_3D_registration_not_native_pose_covariance_export',
            'fallback': 'equal available replay means if either covariance unavailable; sole mean if one available',
            'coverage': {name: sum(r['methods'][name]['transform'] is not None for r in rows) for name in METHODS},
            'full_strategies': dict(Counter(r['methods']['replay_full']['strategy'] for r in rows))}


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source', type=Path, required=True)
    ap.add_argument('--calibration', type=Path, required=True)
    ap.add_argument('--capture', type=Path, required=True)
    ap.add_argument('--rgb-replay', type=Path, required=True)
    ap.add_argument('--thermal-replay', type=Path, required=True)
    ap.add_argument('--official-loader', type=Path, required=True)
    ap.add_argument('--groups', type=int, default=600)
    ap.add_argument('--output', type=Path, required=True)
    a = ap.parse_args()
    if a.output.exists():
        raise FileExistsError(a.output)
    result = predictions(a.source, a.calibration, a.capture,
                         {'rgb': a.rgb_replay, 'thermal': a.thermal_replay}, a.official_loader, a.groups)
    write(a.output, result)
    print(json.dumps({'coverage': result['coverage'], 'full_strategies': result['full_strategies']}))
