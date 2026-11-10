"""Exposed-development opportunity bounds and one frozen, label-free cue rule."""
import argparse
from collections import Counter
from pathlib import Path

import numpy as np

from native_information_weights import reference_rows
from pose_covariance_accuracy import read, sha, write
from pose_covariance_replay import log_se3
from run_native_information_weights import deny_held_labels
from score_pose_covariance_accuracy import error

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results/thermal-opportunities/run-001'
PREVIOUS = ROOT/'results/native-information-weights/run-001'
PLAN = ROOT/'experiments/thermal-opportunities-freeze-2026-11-09.json'
METRICS = ('rotation_deg', 'translation_m')
FEATURES = ('log_factor_ratio', 'log_residual_ratio', 'log_translation_trace_ratio',
            'log_rotation_trace_ratio', 'log_translation_condition_ratio',
            'log_rotation_condition_ratio', 'log1p_rotation_disagreement_rad',
            'log1p_translation_disagreement_m')
TOL = 1e-12


def features(row):
    """No references, error labels, condition identifiers or future frames."""
    a, b = (row['experts'][c] for c in ['rgb', 'thermal'])
    if not (a['valid'] and b['valid']):
        return None
    try:
        values = []
        for e in [a, b]:
            p = np.asarray(e['information_inverse'], dtype=float)
            d = e['diagnostic']
            v = np.array([d['factors'], d['residual_mean_square_px'],
                          np.trace(p[:3, :3]), np.trace(p[3:, 3:]),
                          np.linalg.cond(p[:3, :3]), np.linalg.cond(p[3:, 3:])])
            if not np.isfinite(v).all() or np.any(v <= 0):
                return None
            values.append(v)
        delta = log_se3(np.asarray(b['transform'])@np.linalg.inv(a['transform']))
        x = np.r_[np.log(values[1])-np.log(values[0]),
                  np.log1p(np.linalg.norm(delta[3:])), np.log1p(np.linalg.norm(delta[:3]))]
        return x if np.isfinite(x).all() else None
    except (ValueError, KeyError, TypeError, np.linalg.LinAlgError, IndexError):
        return None


def fit(training, held):
    if len(training) != 2 or held in training:
        raise ValueError('exactly two other recordings required')
    xs, ys, weights, counts, fixed = [], [], [], {}, {}
    for recording, rows in training.items():
        if any(r['recording'] != recording or r['recording'] == held for r in rows):
            raise ValueError('training identity violation')
        valid = [(r, features(r)) for r in rows]
        valid = [(r, x) for r, x in valid if x is not None]
        if len(valid) < 50:
            raise ValueError('insufficient training rows')
        counts[recording] = len(valid)
        losses = {c: [] for c in ['rgb', 'thermal']}
        for r, x in valid:
            for c in losses:
                losses[c].append(error(r['experts'][c]['transform'], r['reference_delta'])['rotation_deg'])
            xs.append(x)
            ys.append(losses['rgb'][-1]-losses['thermal'][-1])
            weights.append(1/(2*len(valid)))
        fixed[recording] = {c: float(np.mean(v)) for c, v in losses.items()}
    x, y, w = np.asarray(xs), np.asarray(ys), np.asarray(weights)
    mean = w@x
    scale = np.sqrt(w@((x-mean)**2))
    scale[scale == 0] = 1
    z = np.column_stack([np.ones(len(x)), (x-mean)/scale])
    penalty = np.diag([0.]+[1.]*len(FEATURES))
    beta = np.linalg.solve(z.T@(w[:, None]*z)+penalty, z.T@(w*y))
    fixed_means = {c: sum(v[c] for v in fixed.values())/2 for c in ['rgb', 'thermal']}
    return {'held_recording': held, 'training_recordings': sorted(training), 'training_counts': counts,
            'features': FEATURES, 'mean': mean.tolist(), 'scale': scale.tolist(), 'beta': beta.tolist(),
            'lambda': 1., 'training_best_fixed': min(fixed_means, key=lambda c: (fixed_means[c], c != 'rgb')),
            'training_fixed_mean_rotation_deg': fixed_means}


def apply(row, model):
    valid = [c for c, e in row['experts'].items() if e['valid']]
    if len(valid) < 2:
        return {'camera': valid[0] if valid else None, 'predicted_gain_deg': None,
                'strategy': 'sole_camera' if valid else 'unavailable'}
    x = features(row)
    if x is None:
        return {'camera': 'rgb', 'predicted_gain_deg': None, 'strategy': 'missing_cue_rgb'}
    gain = float(np.r_[1., (x-np.asarray(model['mean']))/model['scale']]@model['beta'])
    return {'camera': 'thermal' if gain > 0 else 'rgb', 'predicted_gain_deg': gain, 'strategy': 'ridge'}


def runs(rows, predicate):
    result, current = [], []
    for row in rows:
        adjacent = current and row['pair'] == current[-1]['pair']+1 and row['source_ids'][0] == current[-1]['source_ids'][1]
        if not predicate(row) or (current and not adjacent):
            if current:
                result.append(current)
            current = []
        if predicate(row):
            current.append(row)
    if current:
        result.append(current)
    return [{'first_pair': r[0]['pair'], 'last_pair': r[-1]['pair'], 'intervals': len(r),
             'summed_duration_s': sum((x['common_interval_ns'][1]-x['common_interval_ns'][0])/1e9 for x in r)} for r in result]


def diagnose(rows):
    paired = [r for r in rows if all(r['errors'][c] is not None for c in ['rgb', 'thermal'])]
    counts = Counter()
    for r in paired:
        a, b = r['errors']['rgb'], r['errors']['thermal']
        signs = [1 if a[k]-b[k] > TOL else -1 if b[k]-a[k] > TOL else 0 for k in METRICS]
        category = ('both_thermal_better' if signs == [1, 1] else 'both_rgb_better' if signs == [-1, -1]
                    else 'rotation_thermal_translation_rgb' if signs == [1, -1]
                    else 'rotation_rgb_translation_thermal' if signs == [-1, 1] else 'includes_tie')
        counts[category] += 1
    metrics = {}
    for key in METRICS:
        gains = np.array([r['errors']['rgb'][key]-r['errors']['thermal'][key] for r in paired])
        win_runs = runs(paired, lambda r: r['errors']['rgb'][key]-r['errors']['thermal'][key] > TOL)
        metrics[key] = {'thermal_wins': int(np.sum(gains > TOL)), 'rgb_wins': int(np.sum(gains < -TOL)),
                        'ties': int(np.sum(np.abs(gains) <= TOL)), 'thermal_win_runs': win_runs,
                        'oracle_mean_gain': float(np.mean(np.maximum(gains, 0))) if len(gains) else None,
                        'max_run_intervals': max((r['intervals'] for r in win_runs), default=0),
                        'max_run_s': max((r['summed_duration_s'] for r in win_runs), default=0)}
    methods = {m: [] for m in ['rgb', 'thermal', 'rotation_oracle', 'translation_oracle', 'joint_oracle', 'cue', 'train_best_fixed']}
    for r in paired:
        a, b = r['errors']['rgb'], r['errors']['thermal']
        choices = {'rgb': 'rgb', 'thermal': 'thermal', 'cue': r['camera'], 'train_best_fixed': r['fixed_camera'],
                   'rotation_oracle': 'thermal' if b['rotation_deg'] < a['rotation_deg'] else 'rgb',
                   'translation_oracle': 'thermal' if b['translation_m'] < a['translation_m'] else 'rgb',
                   'joint_oracle': 'thermal' if all(b[k] <= a[k] for k in METRICS) else 'rgb'}
        for m, c in choices.items():
            methods[m].append(r['errors'][c])
    averages = {m: {k: float(np.mean([v[k] for v in values])) if values else None for k in METRICS}
                for m, values in methods.items()}
    selected = [r for r in paired if r['camera'] == 'thermal']
    own = [r for r in rows if r['camera'] is not None]
    return {'expected_intervals': len(rows), 'paired_intervals': len(paired),
            'availability': dict(Counter('both' if all(r['errors'][c] is not None for c in ['rgb', 'thermal']) else
                                         'rgb_only' if r['errors']['rgb'] is not None else
                                         'thermal_only' if r['errors']['thermal'] is not None else 'neither' for r in rows)),
            'categories': dict(counts), 'opportunities': metrics, 'paired_mean_errors': averages,
            'cue_thermal_selected_paired': len(selected),
            'cue_thermal_rotation_wins': sum(r['errors']['thermal']['rotation_deg'] < r['errors']['rgb']['rotation_deg'] for r in selected),
            'cue_own_support': len(own),
            'cue_own_mean_errors': {k: float(np.mean([r['errors'][r['camera']][k] for r in own])) for k in METRICS},
            'strategies': dict(Counter(r['strategy'] for r in rows))}


def verify():
    plan = read(PLAN)
    for rel, digest in plan['pins'].items():
        if sha(ROOT/rel) != digest:
            raise ValueError('frozen input/code changed: '+rel)
    return plan


def fold(condition):
    plan = verify()
    raw = {c: read(PREVIOUS/'raw'/f'{c}.json') for c in plan['cases']}
    held = raw[condition]['recording']
    training = {}
    with deny_held_labels(ROOT/plan['cases'][condition]['references']):
        for c in raw:
            if c != condition:
                training[raw[c]['recording']] = reference_rows(raw[c], ROOT/plan['cases'][c]['references'])
        model = fit(training, held)
        predictions = [{'pair': r['pair'], 'source_ids': r['source_ids'], **apply(r, model)} for r in raw[condition]['rows']]
    write(OUT/'predictions'/f'{condition}.json', {'recording': held, 'model': model, 'rows': predictions,
           'held_reference_used': False, 'runtime_metrics': False})
    print({'condition': condition, 'training': model['training_counts'], 'choices': dict(Counter(r['camera'] for r in predictions))})


def lock():
    verify()
    write(OUT/'prediction-lock.json', {'plan_sha256': sha(PLAN), 'predictions': {
        c: sha(OUT/'predictions'/f'{c}.json') for c in read(PLAN)['cases']}, 'scored': False})


def score():
    plan = verify()
    locked = read(OUT/'prediction-lock.json')
    assert locked['plan_sha256'] == sha(PLAN)
    for c, digest in locked['predictions'].items():
        assert sha(OUT/'predictions'/f'{c}.json') == digest
    results = {}
    for c in plan['cases']:
        raw = read(PREVIOUS/'raw'/f'{c}.json')
        pred = read(OUT/'predictions'/f'{c}.json')
        scores = read(PREVIOUS/f'scores-{c}.json')
        labels = {r['pair']: r for r in scores['rows']}
        rows = []
        for r, p in zip(raw['rows'], pred['rows'], strict=True):
            assert (r['pair'], r['source_ids']) == (p['pair'], p['source_ids'])
            s = labels.get(r['pair'])
            errors = {cam: s['errors']['native_'+cam] if s else None for cam in ['rgb', 'thermal']}
            fixed = pred['model']['training_best_fixed']
            if errors[fixed] is None:
                fixed = 'thermal' if errors['thermal'] is not None else 'rgb'
            rows.append({**p, 'common_interval_ns': r['common_interval_ns'], 'errors': errors, 'fixed_camera': fixed})
        result = diagnose(rows)
        write(OUT/f'diagnostic-{c}.json', {'recording': raw['recording'], **result, 'rows': rows})
        results[c] = result
    aggregate = {m: {k: sum(r['paired_mean_errors'][m][k] for r in results.values())/3 for k in METRICS}
                 for m in next(iter(results.values()))['paired_mean_errors']}
    write(OUT/'summary.json', {'development_exposed': True, 'oracle_is_reference_dependent': True,
          'runtime_metrics': False, 'new_frontend_optimizer_calls': 0, 'conditions': results,
          'equal_record_paired_mean_errors': aggregate})
    print(aggregate)


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('action', choices=['fold', 'lock', 'score'])
    ap.add_argument('--condition', choices=['day', 'night', 'rain'])
    a = ap.parse_args()
    if a.action == 'fold':
        fold(a.condition)
    elif a.action == 'lock':
        lock()
    else:
        score()
