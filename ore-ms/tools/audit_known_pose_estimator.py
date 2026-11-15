"""Independent native-packet and true-pose error/dispersion audit."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.stats import chi2

from audit_paired_frontend import independent_selection, point_covariance, hat, transported
from qualify_native_frontend import now, read, sha


def audit(protocol, out):
    plan, checks = read(protocol), 0
    def check(value, text):
        nonlocal checks
        assert value, text
        checks += 1
    def close(a, b, rtol=3e-5, atol=1e-8):
        nonlocal checks
        np.testing.assert_allclose(a, b, rtol=rtol, atol=atol)
        checks += 1
    for path, digest in plan['pins'].items():
        check(sha(path) == digest, path)
    lock = read(out/'prediction-lock.json')
    for name, digest in lock['files'].items():
        check(sha(out/name) == digest, name)
    pred, scores = read(out/'predictions.json'), read(out/'scores.json')
    rows = pred['rows']
    check(pred['model_state_unchanged'] and pred['true_pose_read_for_scoring'] is False, 'prediction provenance')
    check(pred['loading']['migration'] == 'none', 'strict checkpoint')
    check(len(rows) == lock['rows'] == 780 and lock['fit_attempts'] == 1560, 'complete attempted rows')
    check(len({(r['condition'], r['trial']) for r in rows}) == 780, 'unique pairs')
    check(scores['prediction_lock_sha256'] == sha(out/'prediction-lock.json'), 'prediction binding')
    check(scores['arrays_sha256'] == sha(out/'score-arrays.npz'), 'array binding')
    primitives, fit_count, success_count, invalid_count, nulls = {}, 0, 0, 0, 0
    for scene in plan['scenes']:
        key = scene['id']
        with np.load(scene['path']) as z:
            images = z['images'].reshape(2, 2, 256, 640).astype(np.float32)/255
            truth = z['target_to_source'].copy()
        K = np.asarray(plan['K'])
        fb = np.float32(K[0, 0]*plan['baseline_m'])
        selected = [r for r in rows if r['condition'] == key]
        check(sorted(r['trial'] for r in selected) == [-2, -1]+list(range(128)), 'declared trials')
        baseline = next(r for r in selected if r['trial'] == -2)
        base_dense = None
        for row in selected:
            trial = row['trial']
            check(sha(row['dense_path']) == row['dense_sha256'], 'dense hash')
            with np.load(row['dense_path']) as z:
                flow, covariance = z['flow'], z['covariance']
            check(flow.shape == covariance.shape == (3, 2, 256, 640), 'dense shape')
            check(np.isfinite(flow).all() and np.isfinite(covariance).all() and np.all(covariance > 0), 'dense values')
            if trial < 0:
                noise = np.zeros_like(images)
                check(row['noise_seed'] is None, 'null seed')
            else:
                seed = int.from_bytes(hashlib.sha256(f"{plan['noise_seed']}:{key}:{trial}".encode()).digest()[:8], 'little')
                check(seed == row['noise_seed'], 'noise seed')
                noise = np.random.default_rng(seed).uniform(-plan['half_lsb'], plan['half_lsb'], images.shape).astype(np.float32)
            check(hashlib.sha256(noise.tobytes()).hexdigest() == row['noise_sha256'], 'input draw')
            if trial == -2:
                base_dense = (flow, covariance)
            if trial == -1:
                check(np.array_equal(flow, base_dense[0]) and np.array_equal(covariance, base_dense[1]), 'null dense')
                nulls += 1
            expected_ids = independent_selection(covariance, plan['selection_seed'])
            check(row['arms']['regenerated']['proposed_source_ids'] == expected_ids.tolist(), 'independent native selection')
            check(row['arms']['fixed_ids']['proposed_source_ids'] == baseline['arms']['fixed_ids']['proposed_source_ids'], 'fixed queries')
            depths = [fb/np.abs(flow[i, 0]) for i in [0, 1]]
            for arm, result in row['arms'].items():
                fit_count += 1
                if result['status'] == 'INVALID_PACKET':
                    invalid_count += 1
                    check(not result['successful'], 'invalid eligibility')
                    continue
                check(sha(result['packet_path']) == result['packet_sha256'], 'packet hash')
                source = np.asarray(result['proposed_source_ids'], int)
                target = source.astype(np.float32)+flow[2, :, source[:, 1], source[:, 0]]
                good = (target[:, 0] > 32)&(target[:, 0] < 608)&(target[:, 1] > 32)&(target[:, 1] < 224)
                u0, u1 = source[good], target[good]
                a, b = u0[:, 0], u0[:, 1]
                x, y = u1[:, 0].astype(int), u1[:, 1].astype(int)
                C0 = point_covariance(depths[0], u0, np.full(len(u0), .25, np.float32), np.full(len(u0), .25, np.float32), K)
                C1 = point_covariance(depths[1], u1, np.maximum(covariance[2, 0, b, a], .0625), np.maximum(covariance[2, 1, b, a], .0625), K)
                sane = np.isfinite(C0).all((1, 2))&np.isfinite(C1).all((1, 2))
                d0, d1 = depths[0][b, a].astype(float), depths[1][y, x].astype(float)
                back = lambda uv, d: np.column_stack([(uv[:, 0]-K[0, 2])*d/K[0, 0], (uv[:, 1]-K[1, 2])*d/K[1, 1], d])
                with np.load(result['packet_path']) as p:
                    check(np.array_equal(p['source_ids'], u0[sane]), 'admitted IDs')
                    check(result['points'] == len(p['source_ids']), 'point count')
                    close(p['target_uv'], u1[sane], atol=0)
                    close(p['X0'], back(u0.astype(float), d0)[sane])
                    close(p['X1'], back(u1.astype(float), d1)[sane])
                    close(p['C0'], C0[sane], rtol=4e-4, atol=2e-7)
                    close(p['C1'], C1[sane], rtol=4e-4, atol=2e-7)
                    X0, X1, S0, S1 = [p[name].copy() for name in ['X0', 'X1', 'C0', 'C1']]
                    close(p['length'], np.median(X0[:, 2]))
                T = np.asarray(result['target_to_source'])
                R = T[:3, :3]
                close(R.T@R, np.eye(3), atol=1e-9)
                close(np.linalg.det(R), 1, atol=1e-9)
                q = X1@R.T+T[:3, 3]
                residual = q-X0
                weights = np.linalg.inv(S0+R@S1@R.T)
                H, g = np.zeros((6, 6)), np.zeros(6)
                for point, error, W in zip(q, residual, weights, strict=True):
                    J = np.column_stack([np.eye(3), -hat(point)])
                    H += J.T@W@J
                    g += J.T@W@error
                P = np.linalg.inv(H)
                decrement = float(np.sqrt(max(g@P@g, 0)))
                close(result['score_norm'], decrement, rtol=2e-3, atol=1e-7)
                check(result['successful'] == (result['status'] == 'CONVERGED_FROZEN_WEIGHT_SCORE'), 'eligibility')
                if result['successful']:
                    check(decrement <= plan['solver']['score_tolerance']*1.01+1e-8, 'stationarity')
                    close(result['information_covariance'], P, rtol=2e-5, atol=1e-8)
                    success_count += 1
                    primitives[key, trial, arm] = (T, np.asarray(result['information_covariance']))
                else:
                    check(result['information_covariance'] is None, 'failed readout')
                if trial == -1:
                    for field in ['target_to_source', 'information_covariance', 'status']:
                        check(result.get(field) == baseline['arms'][arm].get(field), 'null fit')
    with np.load(out/'score-arrays.npz') as arrays:
        for scene in plan['scenes']:
            key = scene['id']
            with np.load(scene['path']) as z:
                truth = z['target_to_source']
            selected = [r for r in rows if r['condition'] == key]
            base = next(r for r in selected if r['trial'] == -2)
            trials = [r for r in selected if r['trial'] >= 0]
            paired = [r for r in trials if all(r['arms'][a]['successful'] for a in ['fixed_ids', 'regenerated'])]
            report = scores['scenes'][key]
            check(report['attempted_trials'] == 128 and report['paired_successful_trials'] == len(paired), 'case population')
            base_fit = base['arms']['fixed_ids']
            L = np.linalg.cholesky(base_fit['information_covariance']) if base_fit['successful'] else None
            for arm in ['fixed_ids', 'regenerated']:
                statuses = {status: sum(r['arms'][arm]['status'] == status for r in trials) for status in {r['arms'][arm]['status'] for r in trials}}
                check(report['arms'][arm]['statuses'] == statuses, 'all statuses')
                populations = {'baseline': [base] if base['arms'][arm]['successful'] else [], 'own': [r for r in trials if r['arms'][arm]['successful']], 'matched': paired}
                for label, population in populations.items():
                    summary = report['arms'][arm][label]
                    check(summary['valid_trials'] == len(population), 'denominator')
                    if not population:
                        check(summary['metrics'] is None, 'unavailable metrics')
                        continue
                    values = [transported(primitives[key, r['trial'], arm][0], truth, primitives[key, r['trial'], arm][1]) for r in population]
                    X, PP = np.asarray([x for x, p in values]), np.asarray([p for x, p in values])
                    close(arrays[key+'_'+arm+'_'+label+'_xi'], X, atol=1e-9)
                    close(arrays[key+'_'+arm+'_'+label+'_readout'], PP, rtol=2e-5, atol=1e-8)
                    close(arrays[key+'_'+arm+'_'+label+'_trial_ids'], [r['trial'] for r in population], atol=0)
                    mean, second = X.mean(0), X.T@X/len(X)
                    close(summary['mean_error_vector'], mean)
                    close(summary['error_second_moment'], second, atol=1e-10)
                    if len(X) > 1:
                        C = (X-mean).T@(X-mean)/(len(X)-1)
                        close(summary['centered_covariance_ddof1'], C, atol=1e-10)
                        close(second, (len(X)-1)/len(X)*C+np.outer(mean, mean), rtol=1e-9, atol=1e-10)
                    else:
                        check(summary['centered_covariance_ddof1'] is None, 'one-sample covariance undefined')
                    metrics = {'mean_translation_error_m': np.linalg.norm(X[:, :3], axis=1).mean(), 'translation_rmse_m': np.sqrt(np.sum(X[:, :3]**2, axis=1).mean()), 'mean_rotation_error_deg': np.rad2deg(np.linalg.norm(X[:, 3:], axis=1).mean()), 'rotation_rmse_deg': np.rad2deg(np.sqrt(np.sum(X[:, 3:]**2, axis=1).mean())), 'translation_bias_norm_m': np.linalg.norm(mean[:3]), 'rotation_bias_norm_deg': np.rad2deg(np.linalg.norm(mean[3:])), 'mean_points': np.mean([r['arms'][arm]['points'] for r in population])}
                    for name, value in metrics.items():
                        close(summary[name], value)
                    quadratic = np.array([x@np.linalg.solve(p, x) for x, p in values])
                    close(summary['true_target_95pct_containment'], np.mean(quadratic <= chi2.ppf(.95, 6)))
                    if L is not None:
                        W = np.linalg.solve(L, X.T).T
                        close(summary['baseline_information_whitened_error_second_moment_trace'], np.sum(W**2, axis=1).mean())
                        close(summary['baseline_information_whitened_bias_squared'], W.mean(0)@W.mean(0))
                        if len(X) > 1:
                            CW = (W-W.mean(0)).T@(W-W.mean(0))/(len(W)-1)
                            close(summary['baseline_information_whitened_variance_trace'], np.trace(CW))
                        else:
                            check(summary['baseline_information_whitened_variance_trace'] is None, 'single variance unavailable')
                    else:
                        check(summary['baseline_information_whitened_error_second_moment_trace'] is None and summary['baseline_information_whitened_bias_squared'] is None, 'baseline unavailable')
    result = {'created_utc': now(), 'status': 'PASS', 'checks': checks, 'fit_attempts': fit_count, 'successful_fits': success_count, 'invalid_packets': invalid_count, 'null_dense_repeats': nulls, 'scores_sha256': sha(out/'scores.json'), 'auditor_sha256': sha(__file__), 'scope': 'Independent native-selection/packet/fit, true-pose Log transport and finite-sample error-dispersion audit on six artificial scenes. No physical thermal calibration or full native VO reproduction.'}
    (out/'independent-audit.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--protocol', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    audit(args.protocol, Path(args.output))
