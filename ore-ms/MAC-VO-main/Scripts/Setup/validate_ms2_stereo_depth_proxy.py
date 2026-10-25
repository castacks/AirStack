"""Independent scalar/rank and complete-array checks for the EXP046 proxy."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def quantile(values, probability):
    ordered = sorted(float(x) for x in values)
    k = (len(ordered) - 1) * probability
    i = int(k)
    return ordered[i] + (ordered[min(i + 1, len(ordered) - 1)] - ordered[i]) * (k - i)


def ranks(values):
    order = sorted(range(len(values)), key=lambda i: float(values[i]))
    result = [0.] * len(values)
    start = 0
    while start < len(order):
        end = start + 1
        while end < len(order) and values[order[end]] == values[order[start]]:
            end += 1
        rank = (start + end - 1) / 2 + 1
        for i in order[start:end]:
            result[i] = rank
        start = end
    return result


def rank_correlation(x, y):
    if len(x) < 2 or min(x) == max(x) or min(y) == max(y):
        return None
    a, b = ranks(x), ranks(y)
    center = (len(a) + 1) / 2
    cross = math.fsum((x - center) * (y - center) for x, y in zip(a, b))
    sa = math.fsum((x - center) ** 2 for x in a)
    sb = math.fsum((x - center) ** 2 for x in b)
    return cross / math.sqrt(sa * sb)


def validate(root, run):
    root, run = Path(root), Path(run)
    report = json.loads((run / 'report.json').read_text())
    manifest = json.loads((root / 'results/ms2-timestamp-semantics/source-protocol-input-manifest.json').read_text())
    protocol = json.loads((root / 'results/ms2-reference-protocol/audit.json').read_text())
    checks = 0
    scalar_points = 0
    arrays_points = 0

    def check(condition):
        nonlocal checks
        assert condition
        checks += 1

    def close(a, b):
        if a is None or b is None:
            check(a is b)
        else:
            check(math.isclose(a, b, rel_tol=1e-10, abs_tol=1e-12))

    check(report['status'] == 'PASS' and report['frozen_inputs_unchanged'])
    check(not report['physical_cross_modal_error_eligible'] and not report['new_model_inference'])
    check(rank_correlation([1, 1, 3, 4], [1, 1, 3, 4]) == 1.)
    check(rank_correlation([1, 1, 3, 4], [4, 4, 2, 1]) == -1.)
    check(rank_correlation([1, 1], [1, 2]) is None)
    seen = set()
    for record in report['records']:
        m, variant, stem = record['modality'], record['variant'], record['stem']
        check((m, variant, stem) not in seen)
        seen.add((m, variant, stem))
        i = int(stem)
        row = manifest['records'][i]
        camera = row['cameras'][m]
        f = np.asarray([camera['K'][0][0], camera['K'][1][1]])
        fB = f[0] * camera['baseline_m']
        check(record['timing'] == protocol['pairs'][i])
        check(not record['timing']['inherited_joint_gate'])
        maps, masks = {}, {}
        for v in ('depth', 'depth_multi', 'depth_filtered'):
            rec = row['references'][m][v]
            check(sha(root / rec['path']) == rec['sha256'])
            raw = cv2.imread(str(root / rec['path']), cv2.IMREAD_UNCHANGED)
            z = raw.astype(float) / 256
            u = np.arange(raw.shape[1])[None, :]
            masks[v] = (z > 0) & (u * z >= fB) & ((u - (raw.shape[1] - 1)) * z < fB)
            maps[v] = raw
        raw = maps[variant]
        mask = masks[variant]
        y, x = np.where(mask)
        common = np.logical_and.reduce(list(masks.values()))[mask]
        check(record['depth_nonzero'] == int(np.count_nonzero(raw)))
        check(record['out_of_bounds_excluded'] == int(np.count_nonzero(raw)) - len(x))
        path = run / record['file']
        check(sha(path) == record['sha256'])
        with np.load(record['source_file'], allow_pickle=False) as original, np.load(path, allow_pickle=False) as data:
            check(np.array_equal(data['x'], x) and np.array_equal(data['y'], y))
            check(np.array_equal(data['common_support'], common))
            check(np.array_equal(data['depth_m'], raw[mask].astype(float) / 256))
            pred = original['flow'][0, :, y, x].astype(float)
            cov = original['covariance'][0, :, y, x].astype(float)
            check(np.array_equal(data['flow'], pred) and np.array_equal(data['covariance'], cov))
            expected = np.column_stack((-256 * fB / raw[mask], np.zeros(len(x))))
            residual = pred - expected
            for key, value in [('target_flow', expected), ('residual_pixel', residual),
                               ('residual_normalized', residual / f),
                               ('covariance_normalized', cov / f ** 2)]:
                check(np.allclose(data[key], value, rtol=1e-13, atol=1e-13))
            arrays_points += len(x)
            for k in np.linspace(0, len(x) - 1, min(16, len(x)), dtype=int):
                # Scalar camera model, source indexing and focal scaling.
                z = int(raw[y[k], x[k]]) / 256.
                target_u = -float(f[0]) * camera['baseline_m'] / z
                du = float(original['flow'][0, 0, y[k], x[k]]) - target_u
                dv = float(original['flow'][0, 1, y[k], x[k]])
                close(float(data['target_flow'][k, 0]), target_u)
                close(float(data['residual_pixel'][k, 0]), du)
                close(float(data['residual_pixel'][k, 1]), dv)
                close(float(data['residual_normalized'][k, 0]), du / float(f[0]))
                scalar_points += 1
            # Recompute every metric via scalar sorting/linear quantiles and ranks.
            for support, keep in [('own_support', np.ones(len(x), dtype=bool)), ('common_support', common)]:
                summary = record[support]
                check(summary['count'] == int(keep.sum()))
                if not keep.any():
                    check(summary['availability'] == 'empty-support')
                    continue
                for units, scale in [('pixel', np.ones(2)), ('normalized_plane', f)]:
                    e = residual[keep] / scale
                    c = cov[keep] / scale ** 2
                    norm = [math.hypot(float(a), float(b)) for a, b in e]
                    trace = [float(a) + float(b) for a, b in c]
                    s = summary[units]
                    for key, values, q in [('l2_median', norm, .5), ('l2_p90', norm, .9),
                                          ('signed_u_median', e[:, 0], .5), ('signed_v_median', e[:, 1], .5),
                                          ('abs_u_median', abs(e[:, 0]), .5), ('abs_v_median', abs(e[:, 1]), .5),
                                          ('abs_u_p90', abs(e[:, 0]), .9), ('abs_v_p90', abs(e[:, 1]), .9),
                                          ('covariance_trace_median', trace, .5)]:
                        close(s[key], quantile(values, q))
                    close(s['uncertainty_error_spearman'], rank_correlation(trace, norm))
    expected = {(m, v, f'{i:06d}') for m in ('rgb', 'thr') for v in ('depth', 'depth_multi', 'depth_filtered') for i in range(20)}
    check(seen == expected)
    for m, variants in report['aggregate'].items():
        for v, supports in variants.items():
            group = [r for r in report['records'] if r['modality'] == m and r['variant'] == v]
            for support, summary in supports.items():
                close(summary['support_count_sum_not_independent_n'], sum(r[support]['count'] for r in group))
                for units in ('pixel', 'normalized_plane'):
                    for key, s in summary[units].items():
                        values = [r[support][units][key] for r in group if r[support].get(units, {}).get(key) is not None]
                        close(s['available_images'], len(values))
                        close(s['median'], quantile(values, .5) if values else None)
                        close(s['min'], min(values) if values else None)
                        close(s['max'], max(values) if values else None)
    frozen = json.loads((run / 'frozen-inputs.json').read_text())
    for path, digest in frozen.items():
        check(sha(path) == digest)
    historical = 0
    for name in ('exp-043', 'exp-044-complete'):
        old = json.loads((Path('/root/ore-ms-recovery') / name / 'manifest.json').read_text())
        for path, digest in old['member_sha256'].items():
            if path.startswith('ore-ms/results/'):
                check(sha(root.parent / path) == digest)
                historical += 1
    return {'status': 'PASS', 'independent_controls': checks, 'case_count': len(seen),
            'full_array_points_checked': arrays_points, 'scalar_points_checked': scalar_points,
            'historical_source_result_hash_checks': historical, 'frozen_inputs_checked': len(frozen)}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        p.error('Refuse to overwrite validation')
    result = validate(a.root.resolve(), a.run.resolve())
    a.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))
