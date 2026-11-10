"""Independent translation models, component geometry and accuracy audit."""
import json
from pathlib import Path

import numpy as np

from audit_thermal_opportunities import cue

ROOT = Path(__file__).resolve().parents[1]
PRIOR = ROOT/'results/native-information-weights/run-001'
OUT = ROOT/'results/translation-fusion/run-001'


def read(p):
    return json.loads(p.read_text())


def metrics(t, truth):
    t, truth = np.asarray(t), np.asarray(truth)
    r = truth[:3, :3].T@t[:3, :3]
    skew = [r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1]]
    return {'rotation_deg': float(np.rad2deg(np.arctan2(np.linalg.norm(skew)/2, (np.trace(r)-1)/2))),
            'translation_m': float(np.linalg.norm(t[:3, 3]-truth[:3, 3]))}


def audit():
    cases = ['day', 'night', 'rain']
    raw = {c: read(PRIOR/'raw'/f'{c}.json') for c in cases}
    labels = {c: {r['pair']: r['reference_delta'] for r in read(PRIOR/f'scores-{c}.json')['rows']} for c in cases}
    geometry, components, oracle_checks = 0, 0, 0
    for held in cases:
        pred = read(OUT/'predictions'/f'{held}.json'); scores = read(OUT/f'scores-{held}.json')
        model = pred['model']
        assert model['held_recording'] == raw[held]['recording']
        assert model['held_recording'] not in model['training_recordings']
        xs, ds, es, ws = [], [], [], []
        for c in cases:
            if c == held:
                continue
            valid = [(r, cue(r)) for r in raw[c]['rows']]
            valid = [(r, x) for r, x in valid if x is not None]
            assert len(valid) == model['training_counts'][raw[c]['recording']]
            for r, x in valid:
                a, b = [np.asarray(r['experts'][cam]['transform'])[:3, 3] for cam in ['rgb', 'thermal']]
                truth = np.asarray(labels[c][r['pair']])[:3, 3]
                xs.append(x); ds.append(b-a); es.append(truth-a); ws.append(1/(2*len(valid)))
        x, d, e, w = map(np.asarray, [xs, ds, es, ws])
        mean = np.average(x, weights=w, axis=0)
        scale = np.sqrt(np.average((x-mean)**2, weights=w, axis=0)); scale[scale == 0] = 1
        np.testing.assert_allclose(model['mean'], mean, rtol=1e-8, atol=1e-10)
        np.testing.assert_allclose(model['scale'], scale, rtol=1e-8, atol=1e-10)
        z = np.column_stack([np.ones(len(x)), (x-mean)/scale])
        design = (z[:, None, :]*d[:, :, None]).reshape(-1, 9)
        penalty = np.diag([0.]+[1.]*8)
        rootw = np.sqrt(np.repeat(w, 3))
        beta = np.linalg.lstsq(np.vstack([rootw[:, None]*design, penalty]),
                               np.r_[rootw*e.reshape(-1), np.zeros(9)], rcond=None)[0]
        target = np.linalg.norm(e, axis=1)-np.linalg.norm(e-d, axis=1)
        selector = np.linalg.lstsq(np.vstack([np.sqrt(w)[:, None]*z, penalty]),
                                   np.r_[np.sqrt(w)*target, np.zeros(9)], rcond=None)[0]
        constant = np.clip(np.sum(w*np.sum(d*e, axis=1))/np.sum(w*np.sum(d*d, axis=1)), 0, 1)
        np.testing.assert_allclose(model['blend_beta'], beta, rtol=1e-7, atol=1e-9)
        np.testing.assert_allclose(model['select_beta'], selector, rtol=1e-7, atol=1e-9)
        np.testing.assert_allclose(model['constant_alpha'], constant, rtol=1e-10, atol=1e-10)
        fixed_risks = {}
        for cam in ['rgb', 'thermal']:
            fixed_risks[cam] = sum(np.mean([metrics(r['experts'][cam]['transform'], labels[c][r['pair']])['translation_m']
                for r in raw[c]['rows'] if all(v['valid'] for v in r['experts'].values())]) for c in cases if c != held)/2
        fixed = min(fixed_risks, key=lambda cam: (fixed_risks[cam], cam != 'rgb'))
        assert fixed == model['training_best_fixed']
        verified = []
        for r, p, s in zip(raw[held]['rows'], pred['rows'], scores['rows'], strict=True):
            assert r['pair'] == p['pair'] == s['pair']
            np.testing.assert_allclose(s['reference_delta'], labels[held][r['pair']], atol=1e-12)
            a, b = [r['experts'][cam]['transform'] if r['experts'][cam]['valid'] else None for cam in ['rgb', 'thermal']]
            feature = cue(r)
            alpha, gain = 0., None
            if feature is not None:
                zz = np.r_[1., (feature-mean)/scale]
                alpha, gain = float(np.clip(zz@beta, 0, 1)), float(zz@selector)
                np.testing.assert_allclose(p['predicted_translation_gain_m'], gain, rtol=1e-7, atol=1e-9)
            weights = {'thermal_translation': 1., 'equal_translation': .5, 'train_constant': constant,
                       'train_best_fixed': float(fixed == 'thermal'), 'ridge_select': float(gain is not None and gain > 0), 'ridge_blend': alpha}
            errors = {}
            for method, value in p['methods'].items():
                if method in ['rgb', 'thermal']:
                    expected = a if method == 'rgb' else b
                elif a is None or b is None:
                    expected = a if b is None else b
                else:
                    weight = weights[method]
                    np.testing.assert_allclose(value['thermal_alpha'], weight, atol=1e-9, rtol=1e-7)
                    expected = np.array(a, copy=True)
                    expected[:3, 3] = np.asarray(a)[:3, 3]+weight*(np.asarray(b)[:3, 3]-np.asarray(a)[:3, 3])
                    np.testing.assert_array_equal(np.asarray(value['transform'])[:3, :3], np.asarray(a)[:3, :3])
                    geometry += 1
                if expected is None:
                    assert value['transform'] is None and s['errors'][method] is None
                    errors[method] = None
                else:
                    np.testing.assert_allclose(value['transform'], expected, rtol=1e-9, atol=1e-10)
                    errors[method] = metrics(expected, s['reference_delta'])
                    for key, v in errors[method].items():
                        np.testing.assert_allclose(s['errors'][method][key], v, rtol=1e-8, atol=1e-9)
                        components += 1
            verified.append(errors)
            if a is not None and b is not None:
                delta = np.asarray(b)[:3, 3]-np.asarray(a)[:3, 3]
                target = np.asarray(s['reference_delta'])[:3, 3]-np.asarray(a)[:3, 3]
                opt = np.clip(delta@target/(delta@delta), 0, 1) if delta@delta > 0 else 0.
                np.testing.assert_allclose(s['oracle_alpha'], opt, atol=1e-12)
                opt_error = np.linalg.norm(target-opt*delta)
                np.testing.assert_allclose(s['oracle_error']['translation_m'], opt_error, atol=1e-10)
                assert opt_error <= min(np.linalg.norm(target), np.linalg.norm(target-delta))+1e-10
                oracle_checks += 1
        for support, recorded in [('own_metrics', scores['own_metrics']), ('common_metrics', scores['common_metrics'])]:
            rows = verified if support == 'own_metrics' else [r for r in verified if all(v is not None for v in r.values())]
            for method in recorded:
                for key in ['rotation_deg', 'translation_m']:
                    values = np.array([r[method][key] for r in rows if r[method] is not None])
                    stats = recorded[method][key]
                    assert stats['count'] == len(values)
                    for k, v in {'mean': np.mean(values), 'median': np.median(values), 'rmse': np.sqrt(np.mean(values**2)), 'p95': np.percentile(values, 95), 'maximum': np.max(values)}.items():
                        np.testing.assert_allclose(stats[k], v, atol=1e-9, rtol=1e-8)
        assert scores['common_count'] == sum(all(v is not None for v in r.values()) for r in verified)
    summary = read(OUT/'summary.json')
    for m, values in summary['equal_record_common_mean'].items():
        for key, value in values.items():
            np.testing.assert_allclose(value, np.mean([read(OUT/f'scores-{c}.json')['common_metrics'][m][key]['mean'] for c in cases]), atol=1e-12)
    return {'status': 'PASS', 'producer_or_scorer_imported': False, 'paired_component_geometry_checks': geometry,
            'error_component_checks': components, 'oracle_optimum_checks': oracle_checks,
            'models_reconstructed_with_augmented_least_squares': 3, 'all_own_common_metric_statistics_verified': True}


if __name__ == '__main__':
    result = audit()
    with (OUT/'audit.json').open('x') as f:
        json.dump(result, f, indent=2)
    print(result)
