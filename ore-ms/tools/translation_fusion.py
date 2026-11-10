"""Frozen recording-excluded translation fusion with native RGB orientation."""
import argparse
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from native_information_weights import reference_rows
from pose_covariance_accuracy import read, sha, write
from run_native_information_weights import deny_held_labels
from score_pose_covariance_accuracy import error, summarize
from thermal_opportunities import FEATURES, features

ROOT = Path(__file__).resolve().parents[1]
PRIOR = ROOT/'results/native-information-weights/run-001'
OUT = ROOT/'results/translation-fusion/run-001'
PLAN = ROOT/'experiments/translation-fusion-freeze-2026-11-09.json'
METHODS = ('rgb', 'thermal', 'thermal_translation', 'equal_translation', 'train_constant',
           'train_best_fixed', 'ridge_select', 'ridge_blend')
PRIMARY = 'ridge_blend'


def blend(a, b, alpha):
    if a is None or b is None:
        return a if b is None else b
    if not np.isfinite(alpha) or not 0 <= alpha <= 1:
        raise ValueError('convex weight required')
    result = np.array(a, dtype=float, copy=True)
    result[:3, 3] = (1-alpha)*np.asarray(a)[:3, 3]+alpha*np.asarray(b)[:3, 3]
    return result.tolist()


def fit(training, held):
    if len(training) != 2 or held in training:
        raise ValueError('exactly two other recordings required')
    xs, ds, es, gains, ws, counts, risks = [], [], [], [], [], {}, {}
    for recording, rows in training.items():
        if any(r['recording'] != recording or r['recording'] == held for r in rows):
            raise ValueError('training identity violation')
        valid = [(r, features(r)) for r in rows]
        valid = [(r, x) for r, x in valid if x is not None]
        if len(valid) < 50:
            raise ValueError('insufficient joint training rows')
        counts[recording] = len(valid)
        losses = {c: [] for c in ['rgb', 'thermal']}
        for row, x in valid:
            a, b = [np.asarray(row['experts'][c]['transform'])[:3, 3] for c in ['rgb', 'thermal']]
            truth = np.asarray(row['reference_delta'])[:3, 3]
            ea, eb = np.linalg.norm(truth-a), np.linalg.norm(truth-b)
            xs.append(x); ds.append(b-a); es.append(truth-a); gains.append(ea-eb)
            ws.append(1/(2*len(valid)))
            losses['rgb'].append(ea); losses['thermal'].append(eb)
        risks[recording] = {c: float(np.mean(v)) for c, v in losses.items()}
    x, d, e, gain, w = map(np.asarray, [xs, ds, es, gains, ws])
    mean = w@x
    scale = np.sqrt(w@((x-mean)**2)); scale[scale == 0] = 1
    z = np.column_stack([np.ones(len(x)), (x-mean)/scale])
    penalty = np.diag([0.]+[1.]*len(FEATURES))
    q = np.sum(d*d, axis=1); target = np.sum(d*e, axis=1)
    # A zero-disagreement cohort has no translation-fusion degree of freedom.
    beta = (np.linalg.solve(z.T@((w*q)[:, None]*z)+penalty, z.T@(w*target))
            if np.sum(w*q) > 0 else np.zeros(z.shape[1]))
    selector = np.linalg.solve(z.T@(w[:, None]*z)+penalty, z.T@(w*gain))
    constant = float(np.clip(np.sum(w*target)/np.sum(w*q), 0, 1)) if np.sum(w*q) > 0 else 0.
    fixed = {c: sum(r[c] for r in risks.values())/2 for c in ['rgb', 'thermal']}
    return {'held_recording': held, 'training_recordings': sorted(training), 'training_counts': counts,
            'features': FEATURES, 'mean': mean.tolist(), 'scale': scale.tolist(), 'blend_beta': beta.tolist(),
            'select_beta': selector.tolist(), 'constant_alpha': constant, 'lambda': 1.,
            'training_best_fixed': min(fixed, key=lambda c: (fixed[c], c != 'rgb')),
            'training_fixed_mean_translation_m': fixed, 'all_training_disagreement_zero': bool(np.sum(w*q) == 0)}


def apply(row, model):
    a, b = [row['experts'][c]['transform'] if row['experts'][c]['valid'] else None for c in ['rgb', 'thermal']]
    x = features(row)
    alpha, predicted_gain = 0., None
    if x is not None:
        z = np.r_[1., (x-np.asarray(model['mean']))/model['scale']]
        alpha = float(np.clip(z@model['blend_beta'], 0, 1))
        predicted_gain = float(z@model['select_beta'])
    weights = {'thermal_translation': 1., 'equal_translation': .5, 'train_constant': model['constant_alpha'],
               'train_best_fixed': float(model['training_best_fixed'] == 'thermal'),
               'ridge_select': float(predicted_gain is not None and predicted_gain > 0), 'ridge_blend': alpha}
    methods = {'rgb': {'transform': a}, 'thermal': {'transform': b}}
    for method, weight in weights.items():
        methods[method] = {'transform': blend(a, b, weight), 'thermal_alpha': weight}
    return {'pair': row['pair'], 'source_ids': row['source_ids'], 'methods': methods,
            'predicted_translation_gain_m': predicted_gain,
            'strategy': 'neither' if a is None and b is None else 'sole_camera' if a is None or b is None
                        else 'missing_cue_rgb' if x is None else 'cue_available',
            'rgb_rotation_preserved': a is not None}


def verify():
    plan = read(PLAN)
    for rel, digest in plan['pins'].items():
        if sha(ROOT/rel) != digest:
            raise ValueError('frozen source/input changed: '+rel)
    return plan


def fold(condition):
    plan = verify()
    raw = {c: read(PRIOR/'raw'/f'{c}.json') for c in plan['cases']}
    training = {}
    with deny_held_labels(ROOT/plan['cases'][condition]['references']):
        for c in raw:
            if c != condition:
                training[raw[c]['recording']] = reference_rows(raw[c], ROOT/plan['cases'][c]['references'])
        model = fit(training, raw[condition]['recording'])
        rows = [apply(row, model) for row in raw[condition]['rows']]
    write(OUT/'predictions'/f'{condition}.json', {'recording': raw[condition]['recording'], 'model': model,
          'rows': rows, 'held_reference_used': False, 'created_at': datetime.now(timezone.utc).isoformat()})
    print({'condition': condition, 'counts': model['training_counts'], 'constant_alpha': model['constant_alpha'],
           'strategies': dict(Counter(r['strategy'] for r in rows))})


def lock():
    verify()
    write(OUT/'prediction-lock.json', {'created_at': datetime.now(timezone.utc).isoformat(), 'plan_sha256': sha(PLAN),
          'predictions': {c: sha(OUT/'predictions'/f'{c}.json') for c in read(PLAN)['cases']}, 'scores_computed': False})


def score():
    plan = verify(); lock = read(OUT/'prediction-lock.json')
    assert lock['plan_sha256'] == sha(PLAN)
    for c, digest in lock['predictions'].items():
        assert sha(OUT/'predictions'/f'{c}.json') == digest
    summaries = {}
    for c in plan['cases']:
        pred = read(OUT/'predictions'/f'{c}.json')
        raw = read(PRIOR/'raw'/f'{c}.json')
        qualified = {r['pair']: r for r in reference_rows(raw, ROOT/plan['cases'][c]['references'])}
        rows, preserved = [], 0
        for r, p in zip(raw['rows'], pred['rows'], strict=True):
            assert (r['pair'], r['source_ids']) == (p['pair'], p['source_ids'])
            for method in METHODS:
                t = p['methods'][method]['transform']
                if r['experts']['rgb']['valid'] and method not in ['rgb', 'thermal']:
                    np.testing.assert_array_equal(np.asarray(t)[:3, :3], np.asarray(r['experts']['rgb']['transform'])[:3, :3])
                    preserved += 1
            if r['pair'] not in qualified:
                continue
            truth = qualified[r['pair']]['reference_delta']
            errors = {m: error(p['methods'][m]['transform'], truth) if p['methods'][m]['transform'] is not None else None for m in METHODS}
            a, b = [r['experts'][cam]['transform'] for cam in ['rgb', 'thermal']]
            oracle_alpha, oracle_error = None, None
            if a is not None and b is not None:
                d = np.asarray(b)[:3, 3]-np.asarray(a)[:3, 3]
                e = np.asarray(truth)[:3, 3]-np.asarray(a)[:3, 3]
                oracle_alpha = float(np.clip(d@e/(d@d), 0, 1)) if d@d > 0 else 0.
                oracle_error = error(blend(a, b, oracle_alpha), truth)
            rows.append({'pair': r['pair'], 'source_ids': r['source_ids'], 'reference_delta': truth,
                         'errors': errors, 'oracle_alpha': oracle_alpha, 'oracle_error': oracle_error})
        common = [r for r in rows if all(r['errors'][m] is not None for m in METHODS)]
        pairwise = {}
        for control in ['rgb', 'train_constant', 'train_best_fixed', 'ridge_select']:
            matched = [r for r in rows if r['errors'][control] is not None and r['errors'][PRIMARY] is not None]
            pairwise[control] = {'count': len(matched), 'metrics': summarize(matched, [PRIMARY, control])}
        result = {'expected_intervals': len(raw['rows']), 'reference_qualified': len(rows),
                  'common_count': len(common), 'common_metrics': summarize(common, METHODS),
                  'own_metrics': summarize(rows, METHODS), 'primary_pairwise': pairwise,
                  'coverage': {m: sum(r['methods'][m]['transform'] is not None for r in pred['rows']) for m in METHODS},
                  'rotation_preservation_checks': preserved, 'strategies': dict(Counter(r['strategy'] for r in pred['rows'])),
                  'oracle_convex_mean': {k: float(np.mean([r['oracle_error'][k] for r in common])) for k in ['rotation_deg', 'translation_m']}}
        summaries[c] = result
        write(OUT/f'scores-{c}.json', {**result, 'rows': rows})
    aggregate = {m: {k: sum(r['common_metrics'][m][k]['mean'] for r in summaries.values())/3 for k in ['rotation_deg', 'translation_m']}
                 for m in METHODS}
    write(OUT/'summary.json', {'primary': PRIMARY, 'development_exposed': True, 'runtime_metrics': False,
          'new_frontend_optimizer_calls': 0, 'conditions': summaries, 'equal_record_common_mean': aggregate})
    print(aggregate)


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('action', choices=['fold', 'lock', 'score'])
    ap.add_argument('--condition', choices=['day', 'night', 'rain'])
    args = ap.parse_args()
    if args.action == 'fold':
        fold(args.condition)
    elif args.action == 'lock':
        lock()
    else:
        score()
