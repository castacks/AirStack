"""Reference-dependent error decomposition; no inference, correction or selector."""
import json
from pathlib import Path

import numpy as np

from pose_covariance_accuracy import read, sha, write

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results/fusion-mechanism-review/run-001'


def decompose(a, b, truth):
    a, b, truth = map(lambda x: np.asarray(x, dtype=float), [a, b, truth])
    if a.shape != b.shape or a.shape != truth.shape or a.ndim != 2 or a.shape[1] != 3 or not len(a):
        raise ValueError('matching nonempty Nx3 translations required')
    if not all(np.isfinite(x).all() for x in [a, b, truth]):
        raise ValueError('finite translations required')
    ea, eb, d = a-truth, b-truth, b-a
    mean_a, mean_b, mean_d = ea.mean(0), eb.mean(0), d.mean(0)
    ca, cb, cd = ea-mean_a, eb-mean_b, d-mean_d
    A, B, D = np.mean(np.sum(ea*ea, 1)), np.mean(np.sum(ea*d, 1)), np.mean(np.sum(d*d, 1))
    alpha = float(np.clip(-B/D, 0, 1)) if D > 0 else 0.
    norms = np.linalg.norm(truth, axis=1)
    active = norms > 0
    u = truth[active]/norms[active, None]
    cameras = {}
    for cam, e in [('rgb', ea), ('thermal', eb)]:
        mean = e.mean(0); centered = e-mean
        mse = float(np.mean(np.sum(e*e, 1)))
        radial = np.sum(e[active]*u, 1)
        radial_mse = float(np.mean(radial**2)) if active.any() else None
        perp = e[active]-radial[:, None]*u
        perpendicular_mse = float(np.mean(np.sum(perp*perp, 1))) if active.any() else None
        bias = float(mean@mean); centered_mse = float(np.mean(np.sum(centered*centered, 1)))
        assert np.isclose(mse, bias+centered_mse, rtol=1e-12, atol=1e-14)
        if active.all():
            assert np.isclose(mse, radial_mse+perpendicular_mse, rtol=1e-12, atol=1e-14)
        prediction = a if cam == 'rgb' else b
        cameras[cam] = {'mean_error_m': mean.tolist(), 'mean_norm_error_m': float(np.mean(np.linalg.norm(e, axis=1))),
                        'mse_m2': mse, 'mean_error_squared_m2': bias, 'centered_mse_m2': centered_mse,
                        'bias_share_mse': bias/mse if mse > 0 else None,
                        'motion_axis_mean_error_m': float(np.mean(radial)) if active.any() else None,
                        'motion_axis_mse_m2': radial_mse, 'perpendicular_mse_m2': perpendicular_mse,
                        'motion_axis_share_on_nonzero_motion': radial_mse/(radial_mse+perpendicular_mse) if radial_mse is not None and radial_mse+perpendicular_mse > 0 else None,
                        'median_translation_norm_over_reference': float(np.median(np.linalg.norm(prediction[active], axis=1)/norms[active])) if active.any() else None}
    cross = float(np.mean(np.sum(ea*eb, 1)))
    centered_cross = float(np.mean(np.sum(ca*cb, 1)))
    assert np.isclose(cross, mean_a@mean_b+centered_cross, rtol=1e-12, atol=1e-14)
    bias_B, centered_B = float(mean_a@mean_d), float(np.mean(np.sum(ca*cd, 1)))
    assert np.isclose(B, bias_B+centered_B, rtol=1e-12, atol=1e-14)
    oracle_constant_error = np.linalg.norm(ea+alpha*d, axis=1)
    assert np.isclose(np.mean(oracle_constant_error**2), A+2*alpha*B+alpha**2*D, rtol=1e-12, atol=1e-14)
    q = np.sum(d*d, 1)
    oracle = np.zeros(len(q)); np.divide(-np.sum(ea*d, 1), q, out=oracle, where=q > 0); oracle = np.clip(oracle, 0, 1)
    oracle_error = np.linalg.norm(ea+oracle[:, None]*d, axis=1)
    return {'paired_intervals': len(a), 'nonzero_reference_motion': int(active.sum()), 'cameras': cameras,
            'cross_error_second_moment_m2': cross, 'mean_error_dot_product_m2': float(mean_a@mean_b),
            'centered_cross_error_m2': centered_cross,
            'centered_error_cosine_aggregate': centered_cross/np.sqrt(np.mean(np.sum(ca*ca, 1))*np.mean(np.sum(cb*cb, 1))) if np.any(ca) and np.any(cb) else None,
            'opposed_error_vectors': int(np.sum(np.sum(ea*eb, 1) < 0)),
            'risk_polynomial': {'A_m2': float(A), 'B_m2': float(B), 'D_m2': float(D), 'B_mean_error_term_m2': bias_B, 'B_centered_term_m2': centered_B},
            'same_record_oracle_constant': {'alpha': alpha, 'mean_translation_error_m': float(oracle_constant_error.mean()), 'mse_m2': float(np.mean(oracle_constant_error**2))},
            'per_interval_oracle': {'mean_translation_error_m': float(oracle_error.mean()), 'mse_m2': float(np.mean(oracle_error**2))}}


def run():
    results, pins = {}, {}
    for c in ['day', 'night', 'rain']:
        rp = ROOT/f'results/native-information-weights/run-001/raw/{c}.json'
        sp = ROOT/f'results/translation-fusion/run-001/scores-{c}.json'
        raw, scores = read(rp), read(sp)
        labels = {r['pair']: r for r in scores['rows']}
        rows = [r for r in raw['rows'] if all(e['valid'] for e in r['experts'].values())]
        data = []
        for row in rows:
            truth = labels[row['pair']]['reference_delta']
            data.append([np.asarray(row['experts'][cam]['transform'])[:3, 3] for cam in ['rgb', 'thermal']]+[np.asarray(truth)[:3, 3]])
        a, b, truth = np.asarray(data).transpose(1, 0, 2)
        result = decompose(a, b, truth)
        np.testing.assert_allclose(result['per_interval_oracle']['mean_translation_error_m'], scores['oracle_convex_mean']['translation_m'], atol=1e-12)
        result['pair_ids'] = [r['pair'] for r in rows]
        results[c] = result
        pins.update({str(p.relative_to(ROOT)): sha(p) for p in [rp, sp]})
    write(OUT/'decomposition.json', {'reference_dependent_diagnostic': True, 'new_method': False,
          'development_exposed': True, 'runtime_metrics': False, 'source_sha256': pins, 'conditions': results})
    print(json.dumps({c: {k: v for k, v in r.items() if k not in ['pair_ids']} for c, r in results.items()}))


if __name__ == '__main__':
    run()
