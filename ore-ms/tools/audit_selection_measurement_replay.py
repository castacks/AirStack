"""Independent baseline-packet, fit and crossed-covariance decomposition audit."""
import argparse
import json
from pathlib import Path

import numpy as np

from audit_paired_frontend import read, sha, hat, logarithm, point_covariance
from qualify_native_frontend import now


def audit(protocol, out):
    plan = read(protocol)
    checks = 0

    def check(condition, message):
        nonlocal checks
        assert condition, message
        checks += 1

    def close(left, right, rtol=3e-5, atol=1e-8):
        nonlocal checks
        np.testing.assert_allclose(left, right, rtol=rtol, atol=atol)
        checks += 1

    for path, digest in plan['pins'].items():
        check(sha(path) == digest, path)
    lock = read(out / 'prediction-lock.json')
    for rel, digest in lock['files'].items():
        check(sha(out / rel) == digest, rel)
    crossed = read(out / 'predictions.json')
    original = read(plan['original_predictions'])['rows']
    scores = read(out / 'scores.json')
    old_scores = read(plan['original_scores'])
    check(crossed['new_inference'] is crossed['new_reference_values_read'] is False, 'no inference/references')
    check(lock['attempts'] == len(crossed['rows']) == 390, 'all attempts')
    check(scores['arrays_sha256'] == sha(out / 'decomposition-arrays.npz'), 'arrays binding')
    check(scores['prediction_lock_sha256'] == sha(out / 'prediction-lock.json'), 'lock binding')
    check(len({(r['condition'], r['trial']) for r in crossed['rows']}) == 390, 'unique row IDs')
    primitives = {}
    attempted, successful = 0, 0
    for condition in ['day', 'night', 'rain']:
        base = next(r for r in original if r['condition'] == condition and r['trial'] == -2)
        camera = next(r for r in plan['camera_rows'] if r['condition'] == condition)
        K = np.asarray(camera['K'])
        with np.load(base['dense_path']) as dense:
            flow, covariance = dense['flow'], dense['covariance']
        fb = np.float32(K[0, 0] * camera['baseline_m'])
        depths = [fb / np.abs(flow[i, 0]) for i in [0, 1]]
        for record in [r for r in crossed['rows'] if r['condition'] == condition]:
            attempted += 1
            row = next(r for r in original if r['condition'] == condition and r['trial'] == record['trial'])
            result = record['result']
            check(record['proposed_source_ids'] == row['arms']['regenerated']['proposed_source_ids'], 'exact saved selection')
            if result['status'] == 'INVALID_PACKET':
                check(not result['successful'], 'invalid eligibility')
                continue
            check(sha(result['packet_path']) == result['packet_sha256'], 'packet binding')
            source = np.asarray(record['proposed_source_ids'], dtype=int)
            target = source.astype(np.float32) + flow[2, :, source[:, 1], source[:, 0]]
            good = (target[:, 0] > 32) & (target[:, 0] < 608) & (target[:, 1] > 32) & (target[:, 1] < 224)
            u0, u1 = source[good], target[good]
            a, b = u0[:, 0], u0[:, 1]
            x, y = u1[:, 0].astype(int), u1[:, 1].astype(int)
            C0 = point_covariance(depths[0], u0, np.full(len(u0), .25, np.float32), np.full(len(u0), .25, np.float32), K)
            C1 = point_covariance(depths[1], u1, np.maximum(covariance[2, 0, b, a], .0625), np.maximum(covariance[2, 1, b, a], .0625), K)
            sane = np.isfinite(C0).all((1, 2)) & np.isfinite(C1).all((1, 2))
            check(np.all(sane), 'all independent reconstructed covariance finite')
            back = lambda uv, d: np.column_stack([(uv[:, 0]-K[0, 2])*d/K[0, 0], (uv[:, 1]-K[1, 2])*d/K[1, 1], d])
            with np.load(result['packet_path']) as packet:
                check(np.array_equal(packet['source_ids'], u0), 'accepted IDs')
                check(result['points'] == len(u0), 'point count')
                close(packet['source_uv'], u0, atol=0)
                close(packet['target_uv'], u1, atol=0)
                close(packet['X0'], back(u0.astype(float), depths[0][b, a].astype(float)))
                close(packet['X1'], back(u1.astype(float), depths[1][y, x].astype(float)))
                close(packet['C0'], C0, rtol=4e-4, atol=2e-7)
                close(packet['C1'], C1, rtol=4e-4, atol=2e-7)
                # Audit stationarity on independently checked exact serialized solver inputs.
                X0, X1, S0, S1 = [packet[key].copy() for key in ['X0', 'X1', 'C0', 'C1']]
                close(packet['length'], np.median(X0[:, 2]))
                if record['trial'] < 0:
                    with np.load(base['arms']['fixed_ids']['packet_path']) as reference:
                        check(set(packet.files) == set(reference.files), 'baseline fields')
                        check(all(np.array_equal(packet[k], reference[k]) for k in packet.files), 'bitwise baseline packet')
            T = np.asarray(result['target_to_source'])
            R = T[:3, :3]
            close(R.T @ R, np.eye(3), atol=1e-9)
            close(np.linalg.det(R), 1, atol=1e-9)
            q = X1 @ R.T + T[:3, 3]
            residual = q - X0
            weights = np.linalg.inv(S0 + R @ S1 @ R.T)
            H, g = np.zeros((6, 6)), np.zeros(6)
            for point, error, weight in zip(q, residual, weights, strict=True):
                J = np.column_stack([np.eye(3), -hat(point)])
                H += J.T @ weight @ J
                g += J.T @ weight @ error
            inverse = np.linalg.inv(H)
            decrement = float(np.sqrt(max(g @ inverse @ g, 0)))
            close(result['score_norm'], decrement, rtol=2e-3, atol=1e-7)
            check(result['successful'] == (result['status'] == 'CONVERGED_FROZEN_WEIGHT_SCORE'), 'fit eligibility')
            if result['successful']:
                check(decrement <= plan['solver']['score_tolerance']*1.01 + 1e-8, 'stationarity')
                close(result['information_covariance'], inverse, rtol=2e-5)
                successful += 1
            else:
                check(result['information_covariance'] is None, 'failed readout absent')
            if record['trial'] < 0:
                for key in ['status', 'target_to_source', 'information_covariance']:
                    check(result[key] == base['arms']['fixed_ids'][key], 'exact null fit')
            primitives[condition, record['trial']] = result
    with np.load(out / 'decomposition-arrays.npz') as arrays:
        for condition, summary in scores['cases'].items():
            base = next(r for r in original if r['condition'] == condition and r['trial'] == -2)
            T0 = np.asarray(base['arms']['fixed_ids']['target_to_source'])
            L = np.linalg.cholesky(np.asarray(base['arms']['fixed_ids']['information_covariance']))
            trials = [r for r in original if r['condition'] == condition and r['trial'] >= 0]
            matched = [r for r in trials if primitives.get((condition, r['trial']), {}).get('successful') and all(v['successful'] for v in r['arms'].values())]
            check(summary['attempted_perturbation_trials'] == len(trials) == 128, 'trial attempts')
            check(summary['matched_trials'] == len(matched), 'availability')
            statuses = {}
            for record in crossed['rows']:
                if record['condition'] == condition:
                    status = record['result']['status']
                    statuses[status] = statuses.get(status, 0)+1
            check(summary['crossed_all_row_statuses'] == statuses, 'all statuses')
            if len(matched) < 2:
                continue
            close(arrays[condition+'_trial_ids'], [r['trial'] for r in matched], atol=0)
            logs = lambda transforms: np.asarray([logarithm(np.asarray(t) @ np.linalg.inv(T0)) for t in transforms])
            values = {
                'measurement': logs([r['arms']['fixed_ids']['target_to_source'] for r in matched]),
                'selection': logs([primitives[condition, r['trial']]['target_to_source'] for r in matched]),
                'full': logs([r['arms']['regenerated']['target_to_source'] for r in matched]),
            }
            values['interaction'] = values['full'] - values['measurement'] - values['selection']
            for name, value in values.items():
                close(arrays[condition+'_'+name], value, atol=1e-9)
                close(summary['mean_translation_shift_mm'][name], np.linalg.norm(value[:, :3], axis=1).mean()*1000)
                close(summary['mean_rotation_shift_deg'][name], np.rad2deg(np.linalg.norm(value[:, 3:], axis=1)).mean())
            centered = {name: np.linalg.solve(L, (x-x.mean(0)).T).T for name, x in values.items()}
            components = {name: x.T @ x/(len(x)-1) for name, x in centered.items()}
            for a, b in [('measurement', 'selection'), ('measurement', 'interaction'), ('selection', 'interaction')]:
                components[a+'_cross_'+b] = (centered[a].T @ centered[b] + centered[b].T @ centered[a])/(len(matched)-1)
            for name, matrix in components.items():
                close(arrays[condition+'_cov_'+name], matrix, rtol=1e-9, atol=1e-10)
                close(summary['traces'][name], np.trace(matrix), rtol=1e-9, atol=1e-10)
            total = sum(x for name, x in components.items() if name != 'full')
            close(total, components['full'], rtol=1e-9, atol=1e-10)
            close(summary['selection_only_over_full_trace'], np.trace(components['selection'])/np.trace(components['full']))
            close(summary['relative_covariance_reconstruction_error'], np.linalg.norm(total-components['full'])/np.linalg.norm(components['full']), atol=1e-12)
            counts = [primitives[condition, r['trial']]['points'] for r in matched]
            check(summary['crossed_points_range'] == [min(counts), max(counts)], 'counts range')
            if len(matched) == 128:
                for a, b in [('measurement', 'fixed_ids'), ('full', 'regenerated')]:
                    close(summary['traces'][a], old_scores['recordings'][condition]['arms'][b]['matched']['baseline_information_whitened_variance_trace'], rtol=1e-9)
    result = {'created_utc': now(), 'status': 'PASS', 'checks': checks, 'fit_attempts': attempted, 'successful_fits': successful, 'scores_sha256': sha(out/'scores.json'), 'auditor_sha256': sha(__file__), 'scope': 'Independent baseline projection/native covariance, exact-input stationarity, information inverse and analytic-Log covariance decomposition. Original perturbed arms inherit their retained independent audit.'}
    (out/'independent-audit.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--protocol', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    audit(args.protocol, Path(args.output))
