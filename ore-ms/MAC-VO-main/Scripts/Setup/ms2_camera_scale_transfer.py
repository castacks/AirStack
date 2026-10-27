"""Exploratory recording-excluded relative camera-score scale fitting.

These previously inspected recordings are not untouched validation data.
"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
from zoneinfo import ZoneInfo
import numpy as np

PATHS = {
    'rain': 'results/ms2-day-cross-modal-replication/rain-control/metric',
    'day': 'results/ms2-day-cross-modal-replication/day/metric',
    'night': 'results/ms2-night-cross-modal-replication/night/metric',
}
GRID = [{'kind': 'finite', 'log10_c': float(i / 20)} for i in range(-120, 121)] + [
    {'kind': 'always_rgb'}, {'kind': 'always_thr'}]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def choose(rgb, thr, policy):
    if policy['kind'] == 'always_rgb':
        return np.ones(len(rgb))
    if policy['kind'] == 'always_thr':
        return np.zeros(len(rgb))
    a = rgb * 10. ** policy['log10_c']
    return np.where(a < thr, 1., np.where(a > thr, 0., .5))


def load(root):
    data, hashes, reports = {}, {}, {}
    for rec, rel in PATHS.items():
        folder = root / rel
        report_path = folder / 'report.json'
        hashes[str(report_path)] = sha(report_path)
        reports[rec] = json.loads(report_path.read_text())
        assert [r['stem'] for r in reports[rec]['rows']] == [f'{i:06d}' for i in range(20)]
        data[rec] = []
        for row in reports[rec]['rows']:
            path = folder / f"metric-{row['stem']}.npz"
            hashes[str(path)] = sha(path)
            with np.load(path, allow_pickle=False) as f:
                a = {k: f[k] for k in f.files}
            mask = a['both_valid_mask']
            a['u_rgb'] = a['rgb_metric_variance'][mask]
            a['u_thr'] = a['thr_metric_variance'][mask]
            assert len(a['u_rgb']) == row['both_valid'] > 0
            for k in ['u_rgb', 'u_thr']:
                assert np.all(np.isfinite(a[k])) and np.all(a[k] > 0)
            for anchor in ['midpoint', 'rgb', 'thr']:
                for mod in ['rgb', 'thr']:
                    e = a[f'{anchor}_{mod}_error_m']
                    assert np.all(np.isfinite(e)) and np.all(e >= 0)
            data[rec].append(a)
    return data, reports, hashes


def objective(frames, policy):
    values = []
    for a in frames:
        s = choose(a['u_rgb'], a['u_thr'], policy)
        r, t = a['midpoint_rgb_error_m'], a['midpoint_thr_error_m']
        values.append(float(np.mean(s * r + (1 - s) * t) / np.mean((r + t) / 2)))
    return float(np.mean(values))


def tie_key(policy):
    return (0, abs(policy['log10_c']), policy['log10_c']) if policy['kind'] == 'finite' else (1, 0 if policy['kind'] == 'always_rgb' else 1, 0)


def fit(data, train):
    curves = {rec: [objective(data[rec], p) for p in GRID] for rec in train}
    losses = np.mean([curves[rec] for rec in train], axis=0)
    best = min(range(len(GRID)), key=lambda i: (float(losses[i]), tie_key(GRID[i])))
    fixed = min([241, 242], key=lambda i: (float(losses[i]), i))
    return {'train_recordings': train, 'selected_index': best, 'policy': GRID[best],
            'training_fixed_policy': GRID[fixed], 'training_objectives': curves,
            'mean_training_objectives': losses.tolist()}


def evaluate(frames, rows, fitted):
    out = []
    for a, historical in zip(frames, rows, strict=True):
        s = choose(a['u_rgb'], a['u_thr'], fitted['policy'])
        fixed = choose(a['u_rgb'], a['u_thr'], fitted['training_fixed_policy'])
        policies = {'fitted': s, 'original': a['learned_propagated'], 'camera_scale': a['camera_scale'],
                    'always_rgb': np.ones(len(s)), 'always_thr': np.zeros(len(s)),
                    'random': np.full(len(s), .5), 'training_fixed': fixed}
        row = {'stem': historical['stem'], 'availability': {k: historical[k] for k in
               ['candidates', 'both_valid', 'rgb_only_valid', 'thr_only_valid', 'neither_valid']},
               'fitted_rgb_fraction': float(s.mean()), 'risks': {}}
        for anchor in ['midpoint', 'rgb', 'thr']:
            row['risks'][anchor] = {}
            for loss, power in [('mean_l2', 1), ('mse', 2)]:
                r, t = a[f'{anchor}_rgb_error_m'] ** power, a[f'{anchor}_thr_error_m'] ** power
                risks = {name: float(np.mean(w * r + (1 - w) * t)) for name, w in policies.items()}
                risks['oracle'] = float(np.minimum(r, t).mean())
                row['risks'][anchor][loss] = risks
        out.append(row)
    return out


def summary(rows):
    result = {}
    for anchor in ['midpoint', 'rgb', 'thr']:
        result[anchor] = {}
        for loss in ['mean_l2', 'mse']:
            comparisons = {}
            for baseline in ['original', 'camera_scale', 'always_rgb', 'always_thr', 'random', 'training_fixed', 'oracle']:
                ratios = np.array([r['risks'][anchor][loss]['fitted'] / r['risks'][anchor][loss][baseline] for r in rows])
                comparisons[baseline] = {'min': float(ratios.min()), 'median': float(np.median(ratios)),
                                         'max': float(ratios.max()), 'below_one': int((ratios < 1).sum())}
            result[anchor][loss] = comparisons
    return result


def run(root, output):
    if output.exists():
        raise FileExistsError(output)
    data, reports, hashes = load(root)
    protocol = {'created_at': datetime.now(ZoneInfo('America/New_York')).isoformat(),
                'starting_commit': 'fcc0d9d0f', 'script_sha256': sha(__file__), 'numpy_version': np.__version__,
                'scope': 'exploratory recording-excluded transfer on three previously inspected 20-ID windows; not untouched or network-training-held-out validation',
                'grid': GRID, 'training_objective': 'equal recording mean of per-image mean-L2 risk / equal-random mean-L2 risk, midpoint only',
                'tie_break': 'exact objective tie: finite closest log10(c) to zero; smaller log10(c); limits last, RGB before thermal',
                'evaluation': 'unchanged both-valid support; both losses and all three anchors; fit once per fold',
                'no_new_inference_or_prediction_changes': True, 'frozen_input_sha256': hashes}
    folds = {}
    for test in data:
        train = [rec for rec in data if rec != test]
        fitted = fit(data, train)
        rows = evaluate(data[test], reports[test]['rows'], fitted)
        folds[test] = {**fitted, 'test_recording': test, 'rows': rows, 'summary': summary(rows)}
    for p, h in hashes.items():
        assert sha(p) == h
    output.mkdir(parents=True)
    for name, value in [('protocol.json', protocol), ('report.json', {'folds': folds})]:
        (output / name).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    print(json.dumps({rec: {'policy': f['policy'], 'training_fixed': f['training_fixed_policy'],
                           'midpoint': f['summary']['midpoint']} for rec, f in folds.items()}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    run(args.root, args.output)
