"""Independent native-reference, least-squares, direct-interval and loss audit."""
import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import statistics
from zoneinfo import ZoneInfo

import cv2
import numpy as np

CAMERAS = ('rgb', 'thr')
FAMILIES = ('original', 'scale', 'bias_scale')
WIDTHS = (1., 1.96, 2., 3.)


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def quantile(values, q):
    values = sorted(float(x) for x in values)
    index = (len(values) - 1) * q
    lo, hi = math.floor(index), math.ceil(index)
    return values[lo] + (index - lo) * (values[hi] - values[lo])


def dist(values):
    return {'available_frames': len(values), 'min': min(values) if values else None,
            'median': statistics.median(values) if values else None, 'max': max(values) if values else None}


def audit(root, run):
    protocol = json.loads((run / 'protocol.json').read_text())
    fits = json.loads((run / 'frozen-fits.json').read_text())
    report = json.loads((run / 'report.json').read_text())
    checks, extra, fresh, metric_data = 0, {}, {}, {}

    def close(a, b):
        nonlocal checks
        if isinstance(a, dict):
            assert set(a) == set(b), (set(a), set(b))
            for key in a:
                close(a[key], b[key])
        elif isinstance(a, (list, tuple)):
            assert len(a) == len(b)
            for x, y in zip(a, b):
                close(x, y)
        elif a is None or isinstance(a, str):
            assert a == b, (a, b)
            checks += 1
        else:
            assert np.allclose(a, b, rtol=1e-10, atol=1e-9), (a, b)
            checks += int(np.size(a))

    def load(p):
        extra[str(p.relative_to(root))] = sha(p)
        if p.suffix == '.json':
            return json.loads(p.read_text())
        with np.load(p, allow_pickle=False) as f:
            return {k: f[k] for k in f.files}

    assert sha(root / protocol['declaration']) == protocol['declaration_sha256']
    for rel, h in {**protocol['input_sha256'], **protocol['source_sha256']}.items():
        assert sha(root / rel) == h, rel
        checks += 1
    total = 0
    for name, relative in protocol['recordings'].items():
        base = root / relative
        source = load(base / 'dataset/source-manifest.json')
        cal = {k: np.array(v['values'], float) for k, v in load(base / 'dataset/calibration.json').items()}
        frontend = load(base / 'dataset/frontend-run-001/report.json')
        expected = {p['file']: p['sha256'] for p in frontend['pairs']}
        r = cal['R_nir2thr'].dot(cal['R_nir2rgb'].T)
        t = (cal['T_nir2thr'] - r.dot(cal['T_nir2rgb'])).ravel() / 1000
        fresh[name] = {c: [] for c in CAMERAS}
        metric_data[name] = []
        for i in range(20):
            stem = f'{i:06d}'
            p = load(base / 'support' / f'pairs-{stem}.npz')
            m = load(base / 'metric' / f'metric-{stem}.npz')
            ids = np.flatnonzero(m['both_valid_mask'])
            n = len(ids)
            total += n
            mid = (p['rgb_reference_xyz'][ids] + p['thr_reference_xyz_in_rgb'][ids]) / 2
            bands = np.digitize(np.linalg.norm(mid, axis=1), [5, 10, 20, 40, 80])
            native, predictions, geometry, variance = {}, {}, {}, {}
            for c in CAMERAS:
                rawpath = base / 'dataset/frontend-run-001' / f'{c}-stereo-{stem}-{stem}.npz'
                raw = load(rawpath)
                assert sha(rawpath) == expected[rawpath.name]
                assert raw['source_ids'].tolist() == [stem, stem]
                x, y = p[f'{c}_xy'][ids].T
                d = -raw['flow'][0, 0, y, x].astype(float)
                v = raw['covariance'][0, 0, y, x].astype(float)
                entry = source['records'][i]['references'][c]['depth']
                depthpath = root / entry['path']
                assert sha(depthpath) == entry['sha256']
                extra[entry['path']] = sha(depthpath)
                depth = cv2.imread(str(depthpath), cv2.IMREAD_UNCHANGED)
                assert depth.dtype == np.uint16
                zref = depth[y, x].astype(float) / 256
                k = cal[f'K_{c}L']
                baseline = -cal[f'T_{c}R'][0, 0] / 1000
                fb = k[0, 0] * baseline
                dref = fb / zref
                q = np.stack([(x - k[0, 2]) / k[0, 0], (y - k[1, 2]) / k[1, 1], np.ones(n)], axis=1)
                native[c] = q * (fb / d)[:, None]
                predictions[c] = native[c] if c == 'rgb' else (native[c] - t).dot(r)
                j2 = np.einsum('ij,ij->i', q, q) * fb * fb / d ** 4
                variance[c] = j2 * v
                geometry[c] = np.sum(native[c] ** 2, axis=1) * native[c][:, 2] ** 2 / baseline ** 2
                close(predictions[c], m['rgb_predicted_xyz' if c == 'rgb' else 'thr_predicted_xyz_in_rgb'][ids])
                close(variance[c], m[f'{c}_metric_variance'][ids])
                a = {'disparity': d, 'variance': v, 'delta': d - dref, 'reference_disparity': dref,
                     'metric_variance': j2 * v, 'j2': j2, 'range_band': bands}
                derived = load(root / 'results/ms2-disparity-diagnostics/run-001' / name / f'{c}-{stem}.npz')
                for key in ('disparity', 'variance', 'delta', 'reference_disparity', 'metric_variance', 'range_band'):
                    close(a[key], derived[key])
                cuts = [quantile(v, q) for q in (.25, .5, .75)]
                bins = np.array([sum(value >= cut for cut in cuts) for value in v])
                a['cuts'] = cuts
                a['groups'] = {'all': np.arange(n), **{f'range_{j}': np.flatnonzero(bands == j) for j in range(6)},
                               **{f'variance_{j}': np.flatnonzero(bins == j) for j in range(4)}}
                fresh[name][c].append(a)
            anchors = {'rgb': p['rgb_reference_xyz'][ids], 'thr': p['thr_reference_xyz_in_rgb'][ids], 'midpoint': mid}
            losses = {}
            for anchor, target in anchors.items():
                losses[anchor] = {c: np.linalg.norm(predictions[c] - target, axis=1) for c in CAMERAS}
                for c in CAMERAS:
                    close(losses[anchor][c], m[f'{anchor}_{c}_error_m'])
            angular = {c: geometry[c] * statistics.median(variance[c] / geometry[c]) for c in CAMERAS}
            camera_choice = np.where(angular['rgb'] < angular['thr'], 1., np.where(angular['rgb'] > angular['thr'], 0., .5))
            close(camera_choice, m['camera_scale'])
            availability = {'candidates': len(m['both_valid_mask']), 'both_valid': n,
                'rgb_only_valid': int(np.sum(m['rgb_valid_mask'] & ~m['thr_valid_mask'])),
                'thr_only_valid': int(np.sum(m['thr_valid_mask'] & ~m['rgb_valid_mask'])),
                'neither_valid': int(np.sum(~m['rgb_valid_mask'] & ~m['thr_valid_mask']))}
            metric_data[name].append({'losses': losses, 'camera_scale': camera_choice, 'availability': availability})
    assert total == 106562

    for excluded, fold in fits.items():
        training = [n for n in fresh if n != excluded]
        assert fold['training'] == training and fold['excluded'] == excluded
        for c in CAMERAS:
            arrays = [a for name in training for a in fresh[name][c]]
            assert len(arrays) == 60
            weights = np.concatenate([np.full(len(a['variance']), 1 / (60 * len(a['variance']))) for a in arrays])
            delta = np.concatenate([a['delta'] for a in arrays])
            v = np.concatenate([a['variance'] for a in arrays])
            x = np.sqrt(weights / v)
            beta = float(np.linalg.lstsq(x[:, None], delta * x, rcond=None)[0][0])
            mean = lambda values: math.fsum(float(w) * float(z) for w, z in zip(weights, values))
            scale = mean(delta ** 2 / v)
            centered = mean((delta - beta) ** 2 / v)
            out = fold['cameras'][c]
            assert out['status'] == 'PASS'
            close(out['sufficient_sums'], {'inverse_variance': mean(1 / v), 'residual_over_variance': mean(delta / v), 'residual_squared_over_variance': scale})
            for j, name in enumerate(training):
                for i, a in enumerate(fresh[name][c]):
                    close(out['weights'][j][i], {'n': len(a['variance']), 'per_point_weight': 1 / (60 * len(a['variance']))})
            for family, b, s in [('original', 0., 1.), ('scale', 0., scale), ('bias_scale', beta, centered)]:
                model = out['models'][family]
                close([model['beta'], model['scale_squared']], [b, s])
                components = {'mean_log_2pi_native_variance': mean(np.log(2 * math.pi * v)),
                    'log_scale_squared': math.log(s), 'mean_standardized_residual_squared': mean((delta - b) ** 2 / (s * v))}
                close(model['training_objective_components'], components)
                close(model['training_nll'], sum(components.values()) / 2)

    for name, rec in report['recordings'].items():
        assert rec['status'] == 'PASS'
        evaluated = {f: [] for f in FAMILIES}
        selected_rows = []
        for i in range(20):
            saved = load(run / name / f'predictions-{i:06d}.npz')
            scores = {}
            for f in FAMILIES:
                row = {'cameras': {}}
                scores[f] = {}
                for c in CAMERAS:
                    a = fresh[name][c][i]
                    model = fits[name]['cameras'][c]['models'][f]
                    beta, s2 = model['beta'], model['scale_squared']
                    center = a['disparity'] - beta
                    vv = s2 * a['variance']
                    z = (center - a['reference_disparity']) / np.sqrt(vv)
                    scores[f][c] = a['j2'] * (vv + beta * beta)
                    close(z, saved[f'{f}_{c}_z'])
                    close(scores[f][c], saved[f'{f}_{c}_score'])
                    out = rec['families'][f]['rows'][i]['cameras'][c]
                    close(out['variance_cuts_px2'], a['cuts'])
                    new_groups = {}
                    for group, ids in a['groups'].items():
                        nn = len(ids)
                        if not nn:
                            computed = {'n': 0, 'stats': None}
                        else:
                            zz = z[ids]
                            ave = lambda values: math.fsum(float(x) for x in values) / nn
                            mu = ave(zz)
                            stats = {'mean_z': mu, 'mean_z_squared': ave(zz * zz), 'centered_z_squared': ave((zz - mu) ** 2),
                                'mean_variance_px2': ave(vv[ids]), 'mse_disparity_px2': ave((center[ids] - a['reference_disparity'][ids]) ** 2),
                                'gaussian_nll': ave(.5 * (zz * zz + np.log(vv[ids]) + math.log(2 * math.pi))),
                                'center_nonpositive': sum(center[ids] <= 0) / nn,
                                'original_point_mse_disparity_px2': ave(a['delta'][ids] ** 2)}
                            for q in (.025, .16, .5, .84, .975):
                                stats[f'z_quantile_{q:g}'] = quantile(zz, q)
                            for k in WIDTHS:
                                key = f'{k:g}'
                                lo = center[ids] - k * np.sqrt(vv[ids])
                                hi = center[ids] + k * np.sqrt(vv[ids])
                                target = a['reference_disparity'][ids]
                                coverage = int(np.sum((target >= lo) & (target <= hi))) / nn
                                negative, positive = int(np.sum(target > hi)) / nn, int(np.sum(target < lo)) / nn
                                dev = coverage - math.erf(k / math.sqrt(2))
                                stats.update({f'coverage_{key}': coverage, f'negative_tail_{key}': negative, f'positive_tail_{key}': positive,
                                    f'lower_nonpositive_{key}': int(np.sum(lo <= 0)) / nn, f'upper_nonpositive_{key}': int(np.sum(hi <= 0)) / nn,
                                    f'center_nonpositive_{key}': stats['center_nonpositive'], f'coverage_deviation_{key}': dev,
                                    f'coverage_abs_deviation_{key}': abs(dev), f'tail_imbalance_{key}': positive - negative})
                                close(coverage + negative + positive, 1.)
                            close(stats['mean_z_squared'], mu * mu + stats['centered_z_squared'])
                            computed = {'n': nn, 'stats': stats}
                        close(computed, out['groups'][group])
                        new_groups[group] = computed
                    row['cameras'][c] = {'groups': new_groups}
                evaluated[f].append(row)
            policies = {f: np.where(scores[f]['rgb'] < scores[f]['thr'], 1., np.where(scores[f]['rgb'] > scores[f]['thr'], 0., .5)) for f in FAMILIES}
            n = len(fresh[name]['rgb'][i]['variance'])
            policies.update(rgb=np.ones(n), thr=np.zeros(n), random=np.full(n, .5), camera_scale=metric_data[name][i]['camera_scale'])
            old = rec['selection_rows'][i]
            assert not old['policy_failures']
            close(old['availability'], metric_data[name][i]['availability'])
            close(old['exact_score_ties'], {f: int(np.sum(policies[f] == .5)) for f in FAMILIES})
            for policy, values in policies.items():
                close(values, saved[f'choice_{policy}'])
            row = {'losses': {}}
            for anchor, errors in metric_data[name][i]['losses'].items():
                row['losses'][anchor] = {}
                for loss, power in [('mean_l2', 1), ('mse', 2)]:
                    a, b = errors['rgb'] ** power, errors['thr'] ** power
                    risks = {p: math.fsum(float(w) * float(x) + (1 - float(w)) * float(y) for w, x, y in zip(weights, a, b)) / n for p, weights in policies.items()}
                    row['losses'][anchor][loss] = risks
                    close(risks, old['losses'][anchor][loss])
            selected_rows.append(row)

        for f, rows in evaluated.items():
            for c in CAMERAS:
                for group, summary in rec['families'][f]['summary'][c].items():
                    values = [r['cameras'][c]['groups'][group] for r in rows]
                    available = [v for v in values if v['n']]
                    close(summary['nonempty_frames'], len(available))
                    close(summary['count_min_median_max'], [min(v['n'] for v in values), statistics.median(v['n'] for v in values), max(v['n'] for v in values)])
                    stats = {} if not available else {key: dist([v['stats'][key] for v in available]) for key in available[0]['stats']}
                    close(stats, summary['stats'])
        for c, groups in rec['paired_interval_changes'].items():
            for group, comparisons in groups.items():
                for pair, out in comparisons.items():
                    family, baseline = pair.split('_minus_')
                    values = {'abs_coverage_error_1.96': [], 'gaussian_nll': []}
                    for a, b in zip(evaluated[family], evaluated[baseline]):
                        x, y = a['cameras'][c]['groups'][group], b['cameras'][c]['groups'][group]
                        if x['n']:
                            values['abs_coverage_error_1.96'].append(x['stats']['coverage_abs_deviation_1.96'] - y['stats']['coverage_abs_deviation_1.96'])
                            values['gaussian_nll'].append(x['stats']['gaussian_nll'] - y['stats']['gaussian_nll'])
                    close({k: dist(v) for k, v in values.items()}, out)
        for anchor, losses in rec['selection_summary'].items():
            for loss, policies in losses.items():
                for policy, summary in policies.items():
                    risks = [r['losses'][anchor][loss][policy] for r in selected_rows]
                    close(summary['risk'], dist(risks))
                    for baseline, comparison in summary['comparisons'].items():
                        other = [r['losses'][anchor][loss][baseline] for r in selected_rows]
                        ratios = [a / b for a, b in zip(risks, other) if b > 0]
                        close(comparison, {'ratio': dist(ratios),
                            'ratio_categories': {'finite': len(ratios), 'zero_over_zero': sum(a == b == 0 for a, b in zip(risks, other)),
                                                 'positive_over_zero': sum(a > 0 and b == 0 for a, b in zip(risks, other))},
                            'better': sum(a < b for a, b in zip(risks, other)), 'equal': sum(a == b for a, b in zip(risks, other)),
                            'worse': sum(a > b for a, b in zip(risks, other))})
    for rel, h in {**protocol['input_sha256'], **protocol['source_sha256'], **extra}.items():
        assert sha(root / rel) == h, rel
    result = {'status': 'PASS', 'created_at': datetime.now(ZoneInfo('America/New_York')).isoformat(),
        'script_sha256': sha(__file__), 'scalar_checks': checks, 'both_valid_pairs': total,
        'additional_source_sha256': extra, 'report_sha256': sha(run / 'report.json'), 'fits_sha256': sha(run / 'frozen-fits.json'),
        'scope': 'native PNG/raw frontend reconstruction, weighted least squares, direct intervals, full risks and summaries; no physical truth guarantee'}
    (run / 'validation.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'additional_source_sha256'}, indent=2))


def synthetic():
    from .ms2_bias_scale_calibration import fit, point_score, choice, interval, ratio_record, selection_summary
    from .ms2_disparity_coverage import groups
    fixtures = []
    def frame(delta, variance):
        d, v = np.asarray(delta, float), np.asarray(variance, float)
        return {'delta': d, 'variance': v, 'disparity': np.full(len(d), 10.),
                'reference_disparity': 10. - d, 'metric_variance': v * 3, 'range_band': np.zeros(len(d), int)}
    train = [[frame([1., 3.], [1., 4.]), frame([2.], [2.])],
             [frame([-1.], [3.]), frame([4., 0., 2.], [2., 1., 5.])],
             [frame([3., 1.], [1., 2.]), frame([2.], [3.])]]
    result = fit(train)
    b = result['models']['bias_scale']['beta']
    normal = math.fsum(float(np.mean((a['delta'] - b) / a['variance'])) / 6 for frames in train for a in frames)
    assert abs(normal) < 1e-14
    assert result['models']['bias_scale']['training_nll'] <= result['models']['scale']['training_nll']
    fixtures.append('unequal frame sizes/equal recording weights, independent normal equation and nested likelihood')
    held = frame([1., -2.], [2., 3.])
    before = point_score(held, result['models']['bias_scale']).copy()
    held['delta'] *= 1000; held['reference_disparity'] *= -40
    assert fit(train) == result
    assert np.array_equal(point_score(held, result['models']['bias_scale']), before)
    fixtures.append('excluded-label mutation leaves training fits and deployable scores unchanged')
    assert np.array_equal(choice(np.array([1., 2., 3.]), np.array([2., 2., 1.])), [1., .5, 0.])
    fixtures.append('exact score ties retain equal mixture')
    for bad in [0., -1., math.inf, math.nan]:
        try: choice(np.array([bad]), np.ones(1))
        except ValueError: pass
        else: raise AssertionError(bad)
    fixtures.append('invalid zero/negative/nonfinite scores are explicit failures')
    failed = selection_summary([{'stem': '000000', 'policy_failures': {'scale': 'invalid policy score'}}])
    assert failed == {'status': 'NOT_AGGREGATED_POLICY_FAILURE', 'failed_frames': {'000000': {'scale': 'invalid policy score'}}}
    fixtures.append('policy failure prevents partial risk aggregation and retains frame/reason')
    for a in [frame([], []), frame([1.], [0.]), frame([1.], [math.nan]), frame([0.], [1.])]:
        try: fit([[a]])
        except ValueError: pass
        else: raise AssertionError('invalid fit accepted')
    fixtures.append('empty/zero variance/nonfinite/zero scale fits rejected')
    a = frame([0., 0., 0., 0.], [1., 1., 1., 1.])
    cuts, bins, masks = groups(a['variance'], a['range_band'])
    assert bins.tolist() == [3] * 4 and sum(mask.sum() for k, mask in masks.items() if k.startswith('variance_')) == 4
    assert interval(a, {'beta': 0., 'scale_squared': 1.}, masks['variance_0']) == {'n': 0, 'stats': None}
    fixtures.append('tied quartiles and empty strata preserved')
    a = frame([1.96, -1.96], [1., 1.]); a['disparity'][:] = 1.
    a['reference_disparity'] = a['disparity'] - a['delta']
    out = interval(a, {'beta': 0., 'scale_squared': 1.}, np.ones(2, bool))['stats']
    assert out['coverage_1.96'] == 1. and out['lower_nonpositive_1.96'] == 1.
    fixtures.append('inclusive interval boundaries and nonpositive endpoints retained')
    assert ratio_record(0., 0.)['category'] == 'zero_over_zero'
    assert ratio_record(1., 0.)['category'] == 'positive_over_zero'
    fixtures.append('zero risk denominators categorized without epsilon')
    bias_model = {'beta': 2., 'scale_squared': 3.}
    a = frame([1.], [5.])
    assert point_score(a, bias_model)[0] == 3 * (3 * 5 + 2 ** 2)
    fixtures.append('raw-point first-order risk retains squared bias')
    return {'status': 'PASS', 'fixtures': fixtures, 'script_sha256': sha(__file__)}


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--root', type=Path)
    p.add_argument('--run', type=Path)
    p.add_argument('--synthetic', type=Path)
    args = p.parse_args()
    if args.synthetic:
        result = synthetic()
        args.synthetic.write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2))
    else:
        audit(args.root, args.run)
