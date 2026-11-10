"""Fixed-native-mean weighting using a conditional information readout.

No native/replay optimizer or frontend is executed. The readout conditions on
saved landmarks, excludes robust corrections/damping/marginalization, and is
not a calibrated posterior or evidence of native convergence. Recording-excluded
block scaling is a development diagnostic on already exposed recordings.
"""
import argparse
from collections import Counter
from datetime import datetime
import json
from pathlib import Path
import sys
from zoneinfo import ZoneInfo

import numpy as np

from pose_covariance_accuracy import (block_isotropic, combine, project, read, sha,
                                     transforms, write)
from pose_covariance_replay import information_inverse, log_se3, pose_xyzw, skew, spd
from score_pose_covariance_accuracy import error, reference_pose, summarize

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'MAC-VO-main'))
from Scripts.Setup.audit_optimizer_factors import residual_jacobian

METHODS = ('native_rgb', 'native_thermal', 'native_equal', 'native_angular',
           'raw_scalar', 'raw_full', 'cal_scalar', 'cal_full', 'train_best_fixed')
BASELINES = METHODS[:4]
READOUT = ('conditional_fixed_landmark_unrobust_undamped_GN_inverse; '
           'native_convergence_unknown; no_calibrated_posterior_claim')


def adjoint(t):
    r = t[:3, :3]
    return np.block([[r, skew(t[:3, 3])@r], [np.zeros((3, 3)), r]])


def native_readout(points_camera, target_uv, disparity, uv_covariance, disparity_variance, k, baseline):
    """Native disp residual under RIGHT target-camera perturbation."""
    residual, j = residual_jacobian(points_camera, target_uv, disparity, k, baseline)
    c = np.zeros((len(points_camera), 3, 3))
    c[:, 0, 0], c[:, 1, 1] = uv_covariance[:, 0], uv_covariance[:, 1]
    c[:, 0, 1] = c[:, 1, 0] = uv_covariance[:, 2]
    c[:, 2, 2] = np.asarray(disparity_variance).reshape(-1)
    spd(c)
    whitened = np.linalg.solve(np.linalg.cholesky(c), j).reshape(-1, 6)
    if len(points_camera) < 3:
        raise ValueError('insufficient retained factors')
    h = whitened.T@whitened
    inverse = information_inverse(h)
    return inverse, {'factors': len(points_camera), 'information_right': h,
                     'conditional_covariance_right': inverse,
                     'residual_mean_square_px': float(np.mean(residual**2)),
                     'meaning': READOUT}


def information_combination(experts, mode):
    a, b = (experts[c]['transform'] if experts[c]['valid'] else None for c in ['rgb', 'thermal'])
    if a is None or b is None:
        return combine(a, b, mode='equal')
    ca, cb = (experts[c].get('information_inverse') for c in ['rgb', 'thermal'])
    if ca is None or cb is None:
        choice = 'rgb' if ca is not None or cb is None else 'thermal'
        return {'transform': experts[choice]['transform'], 'camera': choice,
                'strategy': 'one_information_available' if ca is not None or cb is not None else 'rgb_no_information'}
    return combine(a, b, ca, cb, mode)


def extract(baseline_path, source_path, calibration_path, capture, loader):
    old, source, calibration = map(read, [baseline_path, source_path, calibration_path])
    frames = transforms(calibration, loader)
    count = old['groups']
    if count != 600 or len(old['rows']) != 599:
        raise ValueError('fixed first600-group cohort required')
    pins = {str(p): sha(p) for p in [Path(baseline_path), Path(source_path), Path(calibration_path), Path(loader)]}
    by_camera = {}
    for camera, sensor in [('rgb', 'rgb'), ('thermal', 'thr')]:
        cp = Path(capture)/camera
        config_path, tensor_path, events_path = cp/'effective-config.json', cp/'tensor-map.npz', cp/'arbiter-events/events.jsonl'
        for p in [config_path, tensor_path, events_path]:
            pins[str(p)] = sha(p)
        cfg = read(config_path)['Odometry']
        if (cfg['optimizer']['type'] != 'TwoFrame_PGO' or cfg['optimizer']['args']['graph_type'] != 'disp'
                or cfg['optimizer']['args']['autodiff'] or cfg['args']['mapping']
                or cfg['postprocess']['type'] != 'MotionInterpolate'):
            raise ValueError('unsupported native graph; no approximate schema substitution')
        events = [json.loads(s) for s in events_path.read_text().splitlines()]
        writes = {}
        for e in events:
            if e['kind'] == 'motion_result_written' and e['first_write_for_submission']:
                key = (e['from_idx'], e['frame_idx'])
                if key in writes:
                    raise ValueError('ambiguous native first write')
                writes[key] = e
        names = ('frames//K', 'frames//baseline', 'points//pos_Tw', 'edge/match2point/mapping',
                 'edge/match2frame1/mapping', 'edge/match2frame2/mapping',
                 'match//pixel2_uv', 'match//pixel2_disp', 'match//pixel2_uv_cov', 'match//pixel2_disp_cov')
        with np.load(tensor_path, allow_pickle=False) as z:
            saved = {k: z[k] for k in names}
        result = []
        for i, old_row in enumerate(old['rows']):
            baseline = old_row['native_experts'][camera]
            if not baseline['valid']:
                result.append({'valid': False, 'transform': None, 'information_inverse': None,
                               'information_status': 'native_unavailable'})
                continue
            e = writes[(i, i+1)]
            native = [source['records'][j]['cameras'][sensor]['left']['recorded_time_ns'] for j in [i, i+1]]
            assert [e['time0_ns'], e['time1_ns']] == native
            assert e['original_need_interp'] is False
            mask = (saved['edge/match2frame1/mapping'] == i)&(saved['edge/match2frame2/mapping'] == i+1)
            ids = saved['edge/match2point/mapping'][mask]
            if ids.dtype.kind not in 'iu' or np.any(ids < 0) or np.any(ids >= len(saved['points//pos_Tw'])):
                raise ValueError('invalid native landmark association')
            native_world = pose_xyzw(e['optimizer_pose_before_write_native_parameterization'])
            points = (saved['points//pos_Tw'][ids].astype(float)-native_world[:3, 3])@native_world[:3, :3]
            row = {'valid': True, 'transform': baseline['transform'], 'information_inverse': None,
                   'source_write_stage': e['stage'], 'information_status': 'not_computed'}
            try:
                right, diagnostic = native_readout(points, saved['match//pixel2_uv'][mask],
                    saved['match//pixel2_disp'][mask], saved['match//pixel2_uv_cov'][mask],
                    saved['match//pixel2_disp_cov'][mask], saved['frames//K'][i+1].astype(float),
                    float(saved['frames//baseline'][i+1]))
                local = pose_xyzw(e['derived_camera_delta_xyzw'])
                a = adjoint(local)
                projected = project(local, a@right@a.T, frames[camera], native, old_row['common_interval_ns'])
                assert projected['valid']
                # Reuse the actual prior baseline bits; no pose is recomputed or optimized.
                np.testing.assert_array_equal(projected['transform'], np.asarray(baseline['transform']))
                row.update(information_inverse=projected['covariance'], information_status='conditional_information_available',
                           diagnostic=diagnostic)
            except ValueError as problem:
                row.update(information_status='invalid_information', reason=str(problem))
            result.append(row)
        by_camera[camera] = result
    rows = []
    for i, old_row in enumerate(old['rows']):
        experts = {camera: by_camera[camera][i] for camera in ['rgb', 'thermal']}
        methods = {method: old_row['methods'][method] for method in BASELINES}
        for mode in ['scalar', 'full']:
            methods['raw_'+mode] = information_combination(experts, mode)
        rows.append({'pair': i, 'source_ids': old_row['source_ids'], 'common_interval_ns': old_row['common_interval_ns'],
                     'experts': experts, 'methods': methods})
    assert all(sha(p) == h for p, h in pins.items())
    return {'recording': old['recording'], 'rows': rows, 'input_sha256': pins, 'reference_used': False,
            'native_means_changed': False, 'optimizer_or_frontend_calls': 0, 'runtime_metrics': False,
            'information_interpretation': READOUT,
            'information_counts': {camera: dict(Counter(r['experts'][camera]['information_status'] for r in rows))
                                   for camera in ['rgb', 'thermal']}}


def scaled_information(c, translation_scale, rotation_scale):
    if (not np.isfinite([translation_scale, rotation_scale]).all()
            or min(translation_scale, rotation_scale) <= 0):
        raise ValueError('positive finite block scales required; no clipping')
    d = np.diag([np.sqrt(translation_scale)]*3+[np.sqrt(rotation_scale)]*3)
    return d@c@d


def fit_scales(training_records, held_out):
    """Equal-record block Mahalanobis MLE; never accepts held-out rows."""
    if held_out in training_records or len(training_records) != 2:
        raise ValueError('exactly two other recordings required')
    per_record, fixed_losses = {}, {}
    for name, rows in training_records.items():
        if len(rows) < 50:
            raise ValueError('less than50joint information-valid training intervals in a recording')
        per_record[name], fixed_losses[name] = {}, {}
        for camera in ['rgb', 'thermal']:
            values = {block: [] for block in ['translation', 'rotation']}
            losses = []
            for row in rows:
                if row['recording'] != name or row['recording'] == held_out:
                    raise ValueError('training-row recording identity violates fold')
                t = np.asarray(row['experts'][camera]['transform'])
                c = np.asarray(row['experts'][camera]['information_inverse'])
                truth = np.asarray(row['reference_delta'])
                e = log_se3(truth@np.linalg.inv(t))
                for block, sl in [('translation', slice(0, 3)), ('rotation', slice(3, 6))]:
                    values[block].append(float(e[sl]@np.linalg.solve(c[sl, sl], e[sl]))/3)
                losses.append(error(t, truth)['rotation_deg'])
            per_record[name][camera] = {block: float(np.mean(v)) for block, v in values.items()}
            fixed_losses[name][camera] = float(np.mean(losses))
    scales = {camera: {block: sum(x[camera][block] for x in per_record.values())/2
                        for block in ['translation', 'rotation']} for camera in ['rgb', 'thermal']}
    for c in scales.values():
        scaled_information(np.eye(6), c['translation'], c['rotation'])
    rotation_means = {camera: sum(x[camera] for x in fixed_losses.values())/2 for camera in ['rgb', 'thermal']}
    best = min(['rgb', 'thermal'], key=lambda c: (rotation_means[c], c != 'rgb'))
    return {'held_out': held_out, 'training_recordings': sorted(training_records), 'scales': scales,
            'training_counts': {k: len(v) for k, v in training_records.items()},
            'per_record_scales': per_record, 'training_rotation_means': rotation_means,
            'training_best_fixed': best, 'fit': 'two_marginal_block_Mahalanobis_scales_per_camera_equal_record_no_search'}


def reference_rows(raw, reference_directory):
    directory = Path(reference_directory)
    pin = read(directory/'reference-lock.json')['sha256']
    poses = {}
    for stem in {s for row in raw['rows'] for s in row['source_ids']}:
        path = directory/(stem+'.txt')
        if sha(path) != pin[stem]:
            raise ValueError('reference changed')
        poses[stem] = reference_pose(path)[0]
    rows = []
    for row in raw['rows']:
        a, b = (poses[s] for s in row['source_ids'])
        if a is not None and b is not None:
            rows.append({**row, 'recording': raw['recording'], 'reference_delta': np.linalg.solve(a, b).tolist()})
    return rows


def apply_model(raw, model):
    result = {k: v for k, v in raw.items() if k != 'rows'}
    result.update(model=model, reference_used_for_application=False, rows=[])
    for row in raw['rows']:
        experts = {k: dict(v) for k, v in row['experts'].items()}
        for camera, expert in experts.items():
            if expert['information_inverse'] is not None:
                s = model['scales'][camera]
                expert['information_inverse'] = scaled_information(np.asarray(expert['information_inverse']), s['translation'], s['rotation'])
        methods = dict(row['methods'])
        for mode in ['scalar', 'full']:
            methods['cal_'+mode] = information_combination(experts, mode)
        best = model['training_best_fixed']
        choice = best if experts[best]['valid'] else next((c for c in ['rgb', 'thermal'] if experts[c]['valid']), None)
        methods['train_best_fixed'] = {'transform': experts[choice]['transform'] if choice else None,
                                       'strategy': 'training_best_fixed', 'camera': choice}
        result['rows'].append({**row, 'methods': methods})
    result['coverage'] = {m: sum(r['methods'][m]['transform'] is not None for r in result['rows']) for m in METHODS}
    result['strategies'] = {m: dict(Counter(r['methods'][m]['strategy'] for r in result['rows']))
                            for m in ['raw_scalar', 'raw_full', 'cal_scalar', 'cal_full']}
    return result


def score_predictions(predictions, references):
    qualified = reference_rows(predictions, references)
    rows = [{'pair': row['pair'], 'source_ids': row['source_ids'], 'reference_delta': row['reference_delta'],
             'errors': {m: error(row['methods'][m]['transform'], row['reference_delta'])
                        if row['methods'][m]['transform'] is not None else None for m in METHODS}}
            for row in qualified]
    common = [row for row in rows if all(row['errors'][m] is not None for m in METHODS)]
    pairwise = {}
    for baseline in METHODS:
        matched = [row for row in rows if row['errors']['cal_full'] is not None and row['errors'][baseline] is not None]
        pairwise[baseline] = {'count': len(matched), 'metrics': summarize(matched, ['cal_full', baseline])}
    return {'recording': predictions['recording'], 'expected_intervals': len(predictions['rows']),
            'reference_qualified': len(rows), 'common_all_count': len(common),
            'common_all_metrics': summarize(common, METHODS), 'own_support': summarize(rows, METHODS),
            'cal_full_pairwise': pairwise, 'coverage': predictions['coverage'], 'strategies': predictions['strategies'],
            'rows': rows, 'native_means_changed': False, 'runtime_metrics': False}


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--baseline', type=Path, required=True)
    ap.add_argument('--source', type=Path, required=True)
    ap.add_argument('--calibration', type=Path, required=True)
    ap.add_argument('--capture', type=Path, required=True)
    ap.add_argument('--loader', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    a = ap.parse_args()
    if a.output.exists():
        raise FileExistsError(a.output)
    value = extract(a.baseline, a.source, a.calibration, a.capture, a.loader)
    write(a.output, value)
    print(json.dumps(value['information_counts']))
