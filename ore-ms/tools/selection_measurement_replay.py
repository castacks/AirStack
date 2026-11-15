"""Cross saved query selection with baseline dense measurements; no inference."""
import argparse
from pathlib import Path

import numpy as np

from paired_frontend_assessment import native_algorithms, packet, fit
from pose_covariance_replay import log_se3
from qualify_native_frontend import now, read, sha, write, deterministic


def verify(plan):
    for path, digest in plan['pins'].items():
        assert sha(path) == digest, path


def replay(plan, out):
    deterministic()
    original = read(plan['original_predictions'])['rows']
    _, covariance = native_algorithms()
    results = []
    for condition in ['day', 'night', 'rain']:
        baseline = next(r for r in original if r['condition'] == condition and r['trial'] == -2)
        camera = next(r for r in plan['camera_rows'] if r['condition'] == condition)
        with np.load(baseline['dense_path']) as z:
            flow, cov = z['flow'], z['covariance']
        for row in [r for r in original if r['condition'] == condition]:
            ids = np.asarray(row['arms']['regenerated']['proposed_source_ids'], dtype=np.int64)
            record = {'condition': condition, 'trial': row['trial'], 'proposed_source_ids': ids.tolist()}
            try:
                p = packet(flow, cov, camera, ids, covariance)
                path = out / 'packets' / f"{condition}-{row['trial']+2:03}.npz"
                np.savez_compressed(path, **p)
                result = fit(p, plan['solver'])
                result.update(packet_path=str(path), packet_sha256=sha(path), points=len(p['source_ids']))
                if row['trial'] < 0:
                    base_result = baseline['arms']['fixed_ids']
                    with np.load(base_result['packet_path']) as reference:
                        assert set(reference.files) == set(p)
                        assert all(np.array_equal(p[k], reference[k]) for k in p), 'baseline packet'
                    for key in ['status', 'target_to_source', 'information_covariance']:
                        assert result[key] == base_result[key], key
            except (ValueError, np.linalg.LinAlgError) as error:
                result = {'status': 'INVALID_PACKET', 'successful': False, 'reason': str(error)}
            record['result'] = result
            results.append(record)
            with (out / 'journal.jsonl').open('a') as stream:
                import json
                stream.write(json.dumps(record, allow_nan=False) + '\n')
        print(condition, 'attempted', sum(r['condition'] == condition for r in results), flush=True)
    write(out / 'predictions.json', {'created_utc': now(), 'rows': results, 'new_inference': False, 'new_reference_values_read': False})
    write(out / 'prediction-lock.json', {'created_utc': now(), 'files': {str(p.relative_to(out)): sha(p) for p in out.rglob('*') if p.is_file()}, 'attempts': len(results)})


def score(plan, out):
    for rel, digest in read(out / 'prediction-lock.json')['files'].items():
        assert sha(out / rel) == digest, rel
    original = read(plan['original_predictions'])['rows']
    crossed = read(out / 'predictions.json')['rows']
    summaries, arrays = {}, {}
    for condition in ['day', 'night', 'rain']:
        base = next(r for r in original if r['condition'] == condition and r['trial'] == -2)
        T0 = np.asarray(base['arms']['fixed_ids']['target_to_source'])
        inverse = np.linalg.inv(T0)
        L = np.linalg.cholesky(np.asarray(base['arms']['fixed_ids']['information_covariance']))
        trials = [r for r in original if r['condition'] == condition and r['trial'] >= 0]
        available, A, B, Y, counts = [], [], [], [], []
        for row in trials:
            extra = next(r['result'] for r in crossed if r['condition'] == condition and r['trial'] == row['trial'])
            if not (extra['successful'] and all(r['successful'] for r in row['arms'].values())):
                continue
            available.append(row['trial'])
            A.append(log_se3(np.asarray(row['arms']['fixed_ids']['target_to_source']) @ inverse))
            B.append(log_se3(np.asarray(extra['target_to_source']) @ inverse))
            Y.append(log_se3(np.asarray(row['arms']['regenerated']['target_to_source']) @ inverse))
            counts.append(extra['points'])
        status_counts = {}
        for row in crossed:
            if row['condition'] == condition:
                status = row['result']['status']
                status_counts[status] = status_counts.get(status, 0) + 1
        summary = {'attempted_perturbation_trials': len(trials), 'matched_trials': len(available), 'crossed_all_row_statuses': status_counts}
        if len(available) < 2:
            summaries[condition] = summary
            continue
        A, B, Y = np.asarray(A), np.asarray(B), np.asarray(Y)
        I = Y - A - B
        vectors = {'measurement': A, 'selection': B, 'interaction': I, 'full': Y}
        white = {key: np.linalg.solve(L, x.T).T for key, x in vectors.items()}
        centered = {key: x - x.mean(0) for key, x in white.items()}
        matrices = {key: x.T @ x / (len(x) - 1) for key, x in centered.items()}
        for left, right in [('measurement', 'selection'), ('measurement', 'interaction'), ('selection', 'interaction')]:
            x, y = centered[left], centered[right]
            matrices[left + '_cross_' + right] = (x.T @ y + y.T @ x) / (len(x) - 1)
        terms = {key: value for key, value in matrices.items() if key != 'full'}
        np.testing.assert_allclose(sum(terms.values()), matrices['full'], rtol=1e-9, atol=1e-10)
        summary.update(
            traces={key: float(np.trace(value)) for key, value in matrices.items()},
            relative_covariance_reconstruction_error=float(np.linalg.norm(sum(terms.values()) - matrices['full']) / np.linalg.norm(matrices['full'])),
            selection_only_over_full_trace=float(np.trace(matrices['selection']) / np.trace(matrices['full'])),
            mean_translation_shift_mm={key: float(1000 * np.linalg.norm(value[:, :3], axis=1).mean()) for key, value in vectors.items()},
            mean_rotation_shift_deg={key: float(np.rad2deg(np.linalg.norm(value[:, 3:], axis=1)).mean()) for key, value in vectors.items()},
            crossed_points_range=[min(counts), max(counts)],
        )
        arrays[condition + '_trial_ids'] = np.asarray(available)
        arrays.update({condition + '_' + key: value for key, value in vectors.items()})
        arrays.update({condition + '_cov_' + key: value for key, value in matrices.items()})
        summaries[condition] = summary
    np.savez_compressed(out / 'decomposition-arrays.npz', **arrays)
    write(out / 'scores.json', {'created_utc': now(), 'cases': summaries, 'prediction_lock_sha256': sha(out / 'prediction-lock.json'), 'arrays_sha256': sha(out / 'decomposition-arrays.npz'), 'scope': 'Finite crossed query-policy replay, common baseline-relative Log tangent. Includes query identity/count and downstream admission. Covariance terms correlated; selection/full ratio is not an additive explained-variance percentage. No physical calibration, new inference or algorithm claim.'})
    print(summaries, flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('stage', choices=['replay', 'score'])
    parser.add_argument('--protocol', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    plan, out = read(args.protocol), Path(args.output)
    verify(plan)
    if args.stage == 'replay':
        out.mkdir(parents=True, exist_ok=False)
        (out / 'packets').mkdir()
        replay(plan, out)
    else:
        score(plan, out)
    verify(plan)


if __name__ == '__main__':
    main()
