"""Execute the declared retrospective disparity calibration; keep 3D points fixed."""
import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import platform
import subprocess
from zoneinfo import ZoneInfo

import numpy as np
import scipy

from .ms2_disparity_diagnostics import PATHS
from .ms2_disparity_coverage import describe, groups, summarize, WIDTHS

CAMERAS = ('rgb', 'thr')
FAMILIES = ('original', 'scale', 'bias_scale')
DECLARATION = 'docs/research/ms2-calibration-protocol-2026-10-28.md'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, obj):
    path.write_text(json.dumps(obj, indent=2, allow_nan=False) + '\n')


def fit(training):
    """Only training arrays enter this function; each recording/frame has equal mass."""
    if not training or any(not frames for frames in training):
        raise ValueError('empty training recording')
    terms = []
    counts = []
    for frames in training:
        recording = []
        for a in frames:
            delta, v = a['delta'], a['variance']
            if not len(v) or not np.isfinite(delta).all() or not np.isfinite(v).all() or np.any(v <= 0):
                raise ValueError('invalid/empty training frame')
            weight = 1 / (len(training) * len(frames))
            terms.append((delta, v, weight))
            recording.append({'n': len(v), 'per_point_weight': weight / len(v)})
        counts.append(recording)
    inv = math.fsum(w * float(np.mean(1 / v)) for d, v, w in terms)
    first = math.fsum(w * float(np.mean(d / v)) for d, v, w in terms)
    second = math.fsum(w * float(np.mean(d * d / v)) for d, v, w in terms)
    beta = first / inv
    centered = math.fsum(w * float(np.mean((d - beta) ** 2 / v)) for d, v, w in terms)
    logvar = math.fsum(w * float(np.mean(np.log(2 * math.pi * v))) for d, v, w in terms)
    if not np.isfinite([beta, centered, second]).all() or min(centered, second) <= 0:
        raise ValueError('nonpositive/nonfinite fitted scale')
    models = {'original': {'beta': 0., 'scale_squared': 1.},
              'scale': {'beta': 0., 'scale_squared': second},
              'bias_scale': {'beta': beta, 'scale_squared': centered}}
    for model in models.values():
        b, s = model['beta'], model['scale_squared']
        residual = math.fsum(w * float(np.mean((d - b) ** 2 / v)) for d, v, w in terms)
        model['training_objective_components'] = {'mean_log_2pi_native_variance': logvar,
            'log_scale_squared': math.log(s), 'mean_standardized_residual_squared': residual / s}
        model['training_nll'] = .5 * (logvar + math.log(s) + residual / s)
    return {'status': 'PASS', 'models': models, 'weights': counts,
            'sufficient_sums': {'inverse_variance': inv, 'residual_over_variance': first,
                                'residual_squared_over_variance': second}}


def point_score(a, model):
    # J² derives from original geometry; no reference/label enters this function.
    v = a['variance']
    return (a['metric_variance'] / v) * (model['scale_squared'] * v + model['beta'] ** 2)


def choice(a, b):
    if not np.isfinite(a).all() or not np.isfinite(b).all() or np.any(a <= 0) or np.any(b <= 0):
        raise ValueError('invalid policy score')
    return np.where(a < b, 1., np.where(a > b, 0., .5))


def interval(a, model, mask):
    center = a['disparity'] - model['beta']
    v = a['variance'] * model['scale_squared']
    result = describe(center, a['reference_disparity'], v, mask)
    if not result['n']:
        return result
    st = result['stats']
    st['center_nonpositive'] = float(np.mean(center[mask] <= 0))
    st['original_point_mse_disparity_px2'] = float(np.mean(a['delta'][mask] ** 2))
    for k in WIDTHS:
        key = f'{k:g}'
        st[f'upper_nonpositive_{key}'] = float(np.mean(center[mask] + k * np.sqrt(v[mask]) <= 0))
        st[f'center_nonpositive_{key}'] = st['center_nonpositive']
        deviation = st[f'coverage_{key}'] - math.erf(k / math.sqrt(2))
        st[f'coverage_deviation_{key}'] = deviation
        st[f'coverage_abs_deviation_{key}'] = abs(deviation)
        st[f'tail_imbalance_{key}'] = st[f'positive_tail_{key}'] - st[f'negative_tail_{key}']
    return result


def distribution(values):
    values = [float(x) for x in values]
    return {'available_frames': len(values), 'min': min(values) if values else None,
            'median': float(np.median(values)) if values else None,
            'max': max(values) if values else None}


def ratio_record(numerator, denominator):
    if denominator > 0:
        return {'category': 'finite', 'value': numerator / denominator}
    return {'category': 'zero_over_zero' if numerator == 0 else 'positive_over_zero', 'value': None}


def selection_summary(rows):
    if any(r['policy_failures'] for r in rows):
        return {'status': 'NOT_AGGREGATED_POLICY_FAILURE',
                'failed_frames': {r['stem']: r['policy_failures'] for r in rows if r['policy_failures']}}
    result = {}
    for anchor in ('midpoint', 'rgb', 'thr'):
        result[anchor] = {}
        for loss in ('mean_l2', 'mse'):
            key = result[anchor][loss] = {}
            policies = rows[0]['losses'][anchor][loss]
            for policy in policies:
                risks = [r['losses'][anchor][loss][policy] for r in rows]
                item = key[policy] = {'risk': distribution(risks), 'comparisons': {}}
                for baseline in policies:
                    other = [r['losses'][anchor][loss][baseline] for r in rows]
                    ratios = [ratio_record(a, b) for a, b in zip(risks, other)]
                    item['comparisons'][baseline] = {'ratio': distribution([r['value'] for r in ratios if r['value'] is not None]),
                        'ratio_categories': {c: sum(r['category'] == c for r in ratios) for c in ('finite', 'zero_over_zero', 'positive_over_zero')},
                        'better': sum(a < b for a, b in zip(risks, other)),
                        'equal': sum(a == b for a, b in zip(risks, other)),
                        'worse': sum(a > b for a, b in zip(risks, other))}
    return result


def run(root, output):
    if output.exists():
        raise FileExistsError(output)
    output.mkdir(parents=True)
    hashes = {}

    def load(path):
        hashes[str(path.relative_to(root))] = sha(path)
        if path.suffix == '.json':
            return json.loads(path.read_text())
        with np.load(path, allow_pickle=False) as f:
            return {k: f[k] for k in f.files}

    derived = root / 'results/ms2-disparity-diagnostics/run-001'
    baseline = root / 'results/ms2-disparity-coverage/run-001'
    parent = load(derived / 'protocol.json')
    assert load(derived / 'validation.json')['status'] == 'PASS'
    assert load(baseline / 'validation.json')['status'] == 'PASS'
    ancestry = dict(parent['input_sha256'])
    ancestry.update(load(baseline / 'validation.json')['additional_raw_reference_sha256'])
    for rel, h in ancestry.items():
        assert sha(root / rel) == h, rel
    original_coverage = load(baseline / 'report.json')
    data, metrics, historical, shapes = {}, {}, {}, {}
    for name, relative in PATHS.items():
        base = root / relative
        data[name] = {c: [] for c in CAMERAS}
        metrics[name] = []
        historical[name] = load(base / 'metric/report.json')
        source = load(base / 'dataset/source-manifest.json')
        assert [r['stem'] for r in source['records']] == [f'{i:06d}' for i in range(20)]
        shapes[name] = {}
        for c in CAMERAS:
            shapes[name][c] = []
            for i in range(20):
                stem = f'{i:06d}'
                a = load(derived / name / f'{c}-{stem}.npz')
                assert len(a['variance']) > 0
                assert all(np.isfinite(v).all() for v in a.values())
                assert np.all(a['variance'] > 0) and np.all(a['disparity'] > 0) and np.all(a['reference_disparity'] > 0)
                raw = load(base / 'dataset/frontend-run-001' / f'{c}-stereo-{stem}-{stem}.npz')
                shapes[name][c].append(list(raw['flow'].shape[-2:]))
                data[name][c].append(a)
        for i in range(20):
            m = load(base / 'metric' / f'metric-{i:06d}.npz')
            assert int(m['both_valid_mask'].sum()) == len(data[name]['rgb'][i]['variance']) == len(data[name]['thr'][i]['variance'])
            metrics[name].append(m)
    assert sum(len(a['variance']) for rec in data.values() for a in rec['rgb']) == 106562
    for c in CAMERAS:
        assert len({tuple(s) for rec in shapes.values() for s in rec[c]}) == 1
    # Freeze full provenance before the first fit and preserve it after evaluation.
    hashes.update(ancestry)
    sources = [Path(__file__), Path(__file__).with_name('ms2_disparity_coverage.py'), Path(__file__).with_name('ms2_disparity_diagnostics.py')]
    protocol = {'created_at': datetime.now(ZoneInfo('America/New_York')).isoformat(),
        'base_commit': subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip(),
        'declaration': DECLARATION, 'declaration_sha256': sha(root / DECLARATION),
        'source_sha256': {str(p.relative_to(root)): sha(p) for p in sources},
        'input_sha256': hashes, 'recordings': PATHS, 'camera_shapes': shapes,
        'ids': [f'{i:06d}' for i in range(20)], 'families': FAMILIES,
        'scope': 'retrospective recording exclusion; original point estimates; no reserved captures',
        'runtime': {'python': platform.python_version(), 'numpy': np.__version__, 'scipy': scipy.__version__}}
    dump(output / 'protocol.json', protocol)
    folds = {}
    for excluded in PATHS:
        training = [n for n in PATHS if n != excluded]
        fold = {'excluded': excluded, 'training': training, 'cameras': {}}
        for c in CAMERAS:
            try:
                fold['cameras'][c] = fit([data[n][c] for n in training])
            except ValueError as e:
                fold['cameras'][c] = {'status': 'FAIL', 'reason': str(e)}
        folds[excluded] = fold
    dump(output / 'frozen-fits.json', folds)
    records = {}
    for name in PATHS:
        dest = output / name
        dest.mkdir()
        fold = folds[name]
        if any(fold['cameras'][c]['status'] != 'PASS' for c in CAMERAS):
            records[name] = {'status': 'FAIL', 'reason': 'invalid training fit; no fallback'}
            continue
        rows_by_family = {f: [] for f in FAMILIES}
        selection = []
        paired = {c: {} for c in CAMERAS}
        for i in range(20):
            stem = f'{i:06d}'
            m = metrics[name][i]
            n = int(m['both_valid_mask'].sum())
            saved = {}
            for family in FAMILIES:
                row = {'stem': stem, 'n': n, 'cameras': {}}
                for c in CAMERAS:
                    a = data[name][c][i]
                    model = fold['cameras'][c]['models'][family]
                    cuts, labels, masks = groups(a['variance'], a['range_band'])
                    row['cameras'][c] = {'variance_cuts_px2': cuts.tolist(),
                        'groups': {g: interval(a, model, mask) for g, mask in masks.items()}}
                    saved[f'{family}_{c}_z'] = (a['delta'] - model['beta']) / np.sqrt(model['scale_squared'] * a['variance'])
                    if family == 'original':
                        old = original_coverage['recordings'][name]['rows'][i]['cameras'][c]
                        for g, value in old['groups'].items():
                            assert row['cameras'][c]['groups'][g]['n'] == value['n']
                            if value['n']:
                                for k, v in value['stats'].items():
                                    assert np.isclose(row['cameras'][c]['groups'][g]['stats'][k], v, rtol=1e-12, atol=1e-12)
                rows_by_family[family].append(row)
            for c in CAMERAS:
                for group in rows_by_family['original'][-1]['cameras'][c]['groups']:
                    p = paired[c].setdefault(group, {})
                    for family in ('scale', 'bias_scale'):
                        for ref in ('original', 'scale'):
                            key = f'{family}_minus_{ref}'
                            out = p.setdefault(key, {'abs_coverage_error_1.96': [], 'gaussian_nll': []})
                            a = rows_by_family[family][-1]['cameras'][c]['groups'][group]
                            b = rows_by_family[ref][-1]['cameras'][c]['groups'][group]
                            if a['n']:
                                out['abs_coverage_error_1.96'].append(a['stats']['coverage_abs_deviation_1.96'] - b['stats']['coverage_abs_deviation_1.96'])
                                out['gaussian_nll'].append(a['stats']['gaussian_nll'] - b['stats']['gaussian_nll'])
            policies, failures = {}, {}
            for family in FAMILIES:
                scores = {c: point_score(data[name][c][i], fold['cameras'][c]['models'][family]) for c in CAMERAS}
                for c in CAMERAS:
                    saved[f'{family}_{c}_score'] = scores[c]
                try:
                    policies[family] = choice(scores['rgb'], scores['thr'])
                except ValueError as e:
                    failures[family] = str(e)
            policies.update(rgb=np.ones(n), thr=np.zeros(n), random=np.full(n, .5), camera_scale=m['camera_scale'])
            assert np.array_equal(policies['original'], m['learned_propagated'])
            for policy, values in policies.items():
                saved[f'choice_{policy}'] = values
            old = historical[name]['rows'][i]
            availability = {k: old[k] for k in ('candidates', 'both_valid', 'rgb_only_valid', 'thr_only_valid', 'neither_valid')}
            assert availability['both_valid'] == n
            row = {'stem': stem, 'availability': availability, 'policy_failures': failures,
                   'exact_score_ties': {f: int(np.sum(policies[f] == .5)) for f in FAMILIES if f in policies}, 'losses': {}}
            for anchor in ('midpoint', 'rgb', 'thr'):
                er, et = m[f'{anchor}_rgb_error_m'], m[f'{anchor}_thr_error_m']
                row['losses'][anchor] = {}
                for loss, power in [('mean_l2', 1), ('mse', 2)]:
                    risks = {p: float(np.mean(w * er ** power + (1 - w) * et ** power)) for p, w in policies.items()}
                    row['losses'][anchor][loss] = risks
                    for p, oldname in [('original', 'learned_propagated'), ('camera_scale', 'camera_scale')]:
                        assert np.isclose(risks[p], old['scores'][anchor][loss][oldname]['uncertainty'], rtol=1e-12, atol=1e-12)
            selection.append(row)
            np.savez_compressed(dest / f'predictions-{stem}.npz', **saved)
        records[name] = {'status': 'PASS' if not any(r['policy_failures'] for r in selection) else 'POLICY_FAILURE',
            'families': {f: {'rows': rows_by_family[f], 'summary': summarize(rows_by_family[f])} for f in FAMILIES},
            'paired_interval_changes': {c: {g: {pair: {k: distribution(v) for k, v in fields.items()} for pair, fields in comparisons.items()} for g, comparisons in paired[c].items()} for c in CAMERAS},
            'selection_rows': selection, 'selection_summary': selection_summary(selection)}
    assert sha(root / DECLARATION) == protocol['declaration_sha256']
    for rel, h in hashes.items():
        assert sha(root / rel) == h, rel
    for rel, h in protocol['source_sha256'].items():
        assert sha(root / rel) == h
    dump(output / 'report.json', {'recordings': records})
    print(json.dumps({'status': {n: r['status'] for n, r in records.items()}, 'input_hashes': len(hashes)}))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    run(args.root, args.output)
