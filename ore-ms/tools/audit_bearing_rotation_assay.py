"""Independent saved-cost, mapping, metric and decision reconstruction."""
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.linalg import fractional_matrix_power

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results/bearing-rotation-assay/run-001'
BASE = ROOT/'results/bearing-rotation-preflight/run-001'


def read(path):
    return json.loads(Path(path).read_text())


def close(a, b):
    np.testing.assert_allclose(a, b, atol=1e-7, rtol=1e-7)


def stats(values):
    a = np.array(values)
    if not len(a):
        return dict(count=0, mean=None, rmse=None, median=None, p95=None, maximum=None)
    return dict(count=len(a), mean=float(sum(a)/len(a)), rmse=float(np.linalg.norm(a)/np.sqrt(len(a))),
                median=float(np.median(a)), p95=float(np.percentile(a, 95)), maximum=float(max(a)))


def angle(a, b):
    r = np.asarray(b).T@a
    sine = np.linalg.norm([r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1]])/2
    return float(np.degrees(np.arctan2(sine, (np.trace(r)-1)/2)))


def normalized(uv, k):
    inverse = np.linalg.inv(k)
    q = np.column_stack([uv, np.ones(len(uv))])@inverse.T
    norm = np.linalg.norm(q, axis=1)
    f = q/norm[:, None]
    j = np.array([(np.eye(3)-np.outer(v, v))@inverse[:, :2]/n for v, n in zip(f, norm)])
    return f, j


def audit():
    freeze = read(ROOT/'experiments/bearing-rotation-execution-freeze-2026-11-12.json')
    for name, digest in freeze['pins'].items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest() == digest, name
    for name, digest in read(OUT/'prediction-lock.json')['pins'].items():
        assert hashlib.sha256((OUT/name).read_bytes()).hexdigest() == digest, name
    predictions = read(OUT/'predictions.json'); scores = read(OUT/'scores.json')
    from pose_covariance_accuracy import transforms
    source = ROOT/'results/pose-covariance/accuracy-001'
    frame = transforms(read(source/'rain/calibration.json'),
        ROOT/'results/pose-covariance/cohort-preflight/snapshot/ore-ms/results/provided-reference-full-record/run-001/source-001/official-loader.py')['thermal']
    optical_frame = frame[:3, :3]@np.array([[0., 0, 1], [1, 0, 0], [0, 1, 0]])
    clocks = read(source/'predictions-v2/rain.json')['rows']
    references = {r['pair']: np.array(r['reference_delta'])[:3, :3] for r in read(
        ROOT/'results/native-information-weights/run-001/scores-rain.json')['rows']}
    native = read(ROOT/'results/thermal-matcher-pilot/run-001/vo-scores.json')['rows_by_arm']['original']
    errors = {k: {} for k in scores['coverage']}
    costs = 0; motions = 0; entries = 0
    assert len(predictions['rows']) == len(scores['rows']) == 19
    for row, scored in zip(predictions['rows'], scores['rows']):
        i = row['pair']; assert i == scored['pair']
        with np.load(BASE/'packets'/f'{i:03}.npz', allow_pickle=False) as z, np.load(OUT/'support'/f'{i:03}.npz', allow_pickle=False) as support:
            selected = z['common']&support['original']&support['replacement']
            np.testing.assert_array_equal(support['common'], selected)
            assert int(selected.sum()) == row['after_filter']
            f0 = normalized(z['source_uv'][selected], z['source_K'].reshape(3, 3))[0]
            selected_covariance = z['covariance'][selected]
            for mean, objectives in row['methods'].items():
                f1, jac = normalized(z[mean+'_uv'][selected], z['target_K'].reshape(3, 3))
                for name, result in objectives.items():
                    key = mean+'/'+name; entry = scored['entries'][key]; entries += 1
                    if result['rotation'] is not None:
                        r, t = np.array(result['rotation']), np.array(result['translation_direction'])
                        close(r.T@r, np.eye(3)); close(np.linalg.det(r), 1); close(np.linalg.norm(t), 1)
                        terms = []
                        for n in range(len(f0)):
                            numerator = t@np.cross(f0[n], r@f1[n])
                            variance = 1.
                            if name != 'NEC':
                                pixel_cov = np.eye(2) if name == 'PNEC_unit_pixel_covariance' else selected_covariance[n]
                                covariance = jac[n]@pixel_cov@jac[n].T
                                a = r.T@np.cross(t, f0[n])
                                variance = a@covariance@a+1e-12
                            terms.append(numerator*numerator/variance)
                        close(sum(terms), result['final_cost']); costs += 1
                    a, b = row['time0_ns'], row['time1_ns']; c, d = clocks[i]['common_interval_ns']
                    eligible = result['status'] == 'CONVERGED' and max(a-c, c-b, a-d, d-b, 0) <= b-a
                    if not eligible:
                        assert entry['mapped_rotation'] is None and entry['error_deg'] is None
                        continue
                    mapped = fractional_matrix_power(optical_frame@r@optical_frame.T, (d-c)/(b-a))
                    assert np.max(np.abs(np.imag(mapped))) < 1e-8
                    mapped = np.real(mapped)
                    close(mapped, entry['mapped_rotation'])
                    e = angle(mapped, references[i]); close(e, entry['error_deg'])
                    errors[key][i] = e; motions += 1
        p = native[i]['transform']; e = None if p is None else angle(np.array(p)[:3, :3], references[i])
        if e is None:
            assert scored['entries']['native']['error_deg'] is None
        else:
            close(e, scored['entries']['native']['error_deg']); errors['native'][i] = e
    assert entries == 114 and predictions['solver_calls'] <= 114
    values = 0
    def compare_statistics(actual, expected):
        nonlocal values
        for name, value in expected.items():
            if value is None:
                assert actual[name] is None
            else:
                close(actual[name], value)
            values += 1
    for key in errors:
        assert scores['coverage'][key] == len(errors[key])
        compare_statistics(scores['own_support_rotation_deg'][key], stats(list(errors[key].values())))
    primary_keys = ['original/NEC', 'original/PNEC_original_pixel_covariance', 'native']
    common = sorted(set.intersection(*(set(errors[k]) for k in primary_keys)))
    assert scores['primary']['matched_pair_ids'] == common
    metrics = {k: stats([errors[k][i] for i in common]) for k in primary_keys}
    for key in primary_keys:
        compare_statistics(scores['primary']['matched_rotation_deg'][key], metrics[key])
    checks = {'nonempty_matched_support': bool(common),
              'no_NEC_coverage_loss': set(errors[primary_keys[0]]) <= set(errors[primary_keys[1]])}
    for metric in ['mean', 'rmse']:
        checks['improves_NEC_'+metric] = bool(common) and metrics[primary_keys[1]][metric] < metrics[primary_keys[0]][metric]
        checks['not_worse_native_'+metric] = bool(common) and metrics[primary_keys[1]][metric] <= metrics['native'][metric]
    assert checks == scores['primary']['checks']
    assert scores['primary']['decision'] == ('RAIN_DEVELOPMENT_GATE_PASS' if all(checks.values()) else 'STOP_THIS_PROTOTYPE_CONFIGURATION')
    for mean in ['original', 'replacement']:
        keys = [k for k in errors if k.startswith(mean+'/')]+['native']
        common = sorted(set.intersection(*(set(errors[k]) for k in keys)))
        group = scores[mean+'_all_method_common']; assert group['pairs'] == common
        for key in keys:
            compare_statistics(group['rotation_deg'][key], stats([errors[key][i] for i in common]))
    result = {'status': 'PASS', 'retained_method_slots': entries, 'saved_objective_costs': costs,
              'independent_motion_mappings_and_errors': motions, 'aggregate_values': values,
              'scope': 'Independent saved cost/mapping/error/statistic/decision audit; common calibration loader reused; no network or optimizer rerun, no physical-reference certification'}
    with (OUT/'audit.json').open('x') as f:
        json.dump(result, f, indent=2)
    print(result)


if __name__ == '__main__':
    audit()
