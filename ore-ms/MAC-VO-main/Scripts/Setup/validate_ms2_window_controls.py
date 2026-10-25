"""Independent window controls: unchanged scalar validation, original source-ID lookup."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from Scripts.Setup.validate_ms2_stereo_depth_proxy import ranks, rank_correlation, quantile


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def depth_group(z):
    return sum(z >= edge for edge in [5., 10., 20., 40., 80.])


def validate(root, source, run):
    root, source, run = Path(root), Path(source), Path(run)
    report = json.loads((run / 'report.json').read_text())
    old = json.loads((source / 'report.json').read_text())
    manifest = json.loads(Path(old['source_manifest']).read_text())
    rows_by_stem={r['stem']:r for r in manifest['records']}
    primary = {(r['modality'], r['stem']): r for r in old['records'] if r['variant'] == 'depth'}
    originals = {(r['modality'], r['stem'], r['variant']): r for r in old['records']}
    checks = 0
    strata_checked = 0
    membership_points = 0

    def check(condition):
        nonlocal checks
        assert condition
        checks += 1

    def close(x, y):
        if x is None or y is None:
            check(x is y)
        else:
            check(math.isclose(x, y, rel_tol=1e-10, abs_tol=1e-12))

    check([depth_group(x) for x in [0.1, 4.999, 5., 10., 20., 40., 80., 1000.]] == [0, 0, 1, 2, 3, 4, 5, 5])
    check(ranks([1, 1, 3, 4]) == [1.5, 1.5, 3., 4.])
    check(rank_correlation([1, 1], [2, 3]) is None)
    check(report['status'] == 'PASS' and report['frozen_inputs_unchanged'])
    check(not report['physical_cross_modal_error_eligible'] and not report['new_model_inference'])
    seen = set()
    for record in report['records']:
        m, stem, v, support = [record[k] for k in ('modality', 'stem', 'variant', 'support')]
        key = m, stem, v, support
        check(key not in seen)
        seen.add(key)
        height, width = rows_by_stem[stem]['cameras'][m]['left']['shape'][:2]
        original = originals[m, stem, v]
        check(record['timing'] == original['timing'])
        check(sha(record['source_file']) == original['sha256'])
        with np.load(source / primary[m, stem]['file'], allow_pickle=False) as p:
            pc = p['common_support']
            primary_z = {int(y) * width + int(x): float(z) for x, y, z in zip(p['x'][pc], p['y'][pc], p['depth_m'][pc])}
        with np.load(record['source_file'], allow_pickle=False) as data:
            keep = np.ones(len(data['x']), dtype=bool) if support == 'own_support' else data['common_support']
            x, y = data['x'][keep], data['y'][keep]
            z = data['depth_m'][keep]
            depth = [primary_z[int(b) * width + int(a)] for a, b in zip(x, y)] if support == 'common_support' else z.tolist()
            u = [math.fsum(float(a) for a in row) for row in data['covariance_normalized'][keep]]
            e = [math.hypot(float(row[0]), float(row[1])) for row in data['residual_normalized'][keep]]
        check(record['count'] == len(u))
        close(record['uncertainty_inverse_depth_rho'], rank_correlation(u, [1 / d for d in depth]))
        close(record['error_inverse_depth_rho'], rank_correlation(e, [1 / d for d in depth]))
        dg = [depth_group(float(d)) for d in depth]
        sg = [min(2, int(3 * int(b) / height)) * 3 + min(2, int(3 * int(a) / width)) for a, b in zip(x, y)]
        membership_points += len(u)
        full = rank_correlation(u, e)
        close(full, original[support]['normalized_plane']['uncertainty_error_spearman'])
        for name, groups, count in [('depth', dg, 6), ('spatial', sg, 9), ('joint', [9 * a + b for a, b in zip(dg, sg)], 54)]:
            control = record['controls'][name]
            check(len(control['strata']) == count)
            members = [[] for _ in range(count)]
            for i, g in enumerate(groups):
                members[g].append(i)
            valid = []
            sa = sb = ab = 0.
            eligible = 0
            for g, indices in enumerate(members):
                s = control['strata'][g]
                a = [u[i] for i in indices]
                b = [e[i] for i in indices]
                n = len(indices)
                reason = 'sparse' if n < 30 else 'constant-uncertainty' if min(a) == max(a) else 'constant-error' if min(b) == max(b) else None
                check(s['stratum'] == g and s['count'] == n and s['unavailable_reason'] == reason)
                if reason is None:
                    eligible += 1
                    valid.extend(indices)
                    close(s['rho'], rank_correlation(a, b))
                    ra, rb = ranks(a), ranks(b)
                    mean = (n + 1) / 2
                    # Scalar sums independently implement concatenated normalized-rank moments.
                    sa += math.fsum(((x - mean) / n) ** 2 for x in ra)
                    sb += math.fsum(((x - mean) / n) ** 2 for x in rb)
                    ab += math.fsum((x - mean) * (y - mean) / n ** 2 for x, y in zip(ra, rb))
                else:
                    check(s['rho'] is None)
                strata_checked += 1
            check(control['eligible_strata'] == eligible and control['retained_count'] == len(valid))
            close(control['retained_fraction'], len(valid) / len(u) if len(u) else 0.)
            close(control['full_support_raw_rho'], full)
            close(control['same_retained_pixels_raw_rho'], rank_correlation([u[i] for i in valid], [e[i] for i in valid]))
            close(control['within_bin_rank_coefficient'], ab / math.sqrt(sa * sb) if sa and sb else None)
            check(control['availability'] == ('available' if valid else 'no-eligible-strata'))
    expected = {(m, row['stem'], v, s) for m in ('rgb', 'thr') for row in manifest['records'] for v in ('depth', 'depth_multi', 'depth_filtered') for s in ('own_support', 'common_support')}
    check(seen == expected)
    for m, variants in report['aggregate'].items():
        for v, supports in variants.items():
            for support, summary in supports.items():
                rows = [r for r in report['records'] if r['modality'] == m and r['variant'] == v and r['support'] == support]
                for key in ('uncertainty_inverse_depth_rho', 'error_inverse_depth_rho'):
                    a = [r[key] for r in rows if r[key] is not None]
                    close(summary[key]['median'], quantile(a, .5) if a else None)
                    check(summary[key]['available_images'] == len(a))
                for name in ('depth', 'spatial', 'joint'):
                    stats = summary[name]
                    check(stats['available_images'] == sum(r['controls'][name]['availability'] == 'available' for r in rows))
                    for key in ('full_support_raw_rho', 'same_retained_pixels_raw_rho', 'within_bin_rank_coefficient', 'retained_fraction', 'eligible_strata'):
                        a = [r['controls'][name][key] for r in rows if r['controls'][name][key] is not None]
                        s = stats[key]
                        check(s['available_images'] == len(a))
                        close(s['median'], quantile(a, .5) if a else None)
                        close(s['min'], min(a) if a else None)
                        close(s['max'], max(a) if a else None)
    frozen = json.loads((run / 'frozen-inputs.json').read_text())
    for path, digest in frozen.items():
        check(sha(path) == digest)
    historical = 0
    for name in ('exp-043', 'exp-044-complete', 'exp-046', 'exp-047', 'exp-048'):
        old = json.loads((Path('/root/ore-ms-recovery') / name / 'manifest.json').read_text())
        for path, digest in old['member_sha256'].items():
            if path.startswith('ore-ms/results/'):
                check(sha(root.parent / path) == digest)
                historical += 1
    return {'status': 'PASS', 'independent_controls': checks, 'case_support_count': len(seen),
            'strata_checked': strata_checked, 'membership_points_checked': membership_points,
            'frozen_inputs_checked': len(frozen), 'historical_hash_checks': historical}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        p.error('Refuse to overwrite validation')
    result = validate(a.root.resolve(), a.source.resolve(), a.run.resolve())
    a.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))
