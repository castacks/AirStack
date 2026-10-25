"""Fixed exploratory depth/spatial rank controls on saved EXP046 proxy arrays."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
from zoneinfo import ZoneInfo

import numpy as np
from scipy.stats import rankdata

DEPTH_EDGES = [0., 5., 10., 20., 40., 80.]
MINIMUM = 30


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rho(x, y):
    if len(x) < 2 or np.ptp(x) == 0 or np.ptp(y) == 0:
        return None
    return float(np.corrcoef(rankdata(x), rankdata(y))[0, 1])


def control(u, e, groups, group_count):
    strata = []
    retained = np.zeros(len(u), dtype=bool)
    numerator = ss_u = ss_e = 0.
    for group in range(group_count):
        keep = groups == group
        n = int(keep.sum())
        reason = ('sparse' if n < MINIMUM else
                  'constant-uncertainty' if np.ptp(u[keep]) == 0 else
                  'constant-error' if np.ptp(e[keep]) == 0 else None)
        row = {'stratum': group, 'count': n, 'unavailable_reason': reason, 'rho': None}
        if reason is None:
            a = (rankdata(u[keep]) - (n + 1) / 2) / n
            b = (rankdata(e[keep]) - (n + 1) / 2) / n
            numerator += float(a @ b)
            ss_u += float(a @ a)
            ss_e += float(b @ b)
            row['rho'] = rho(u[keep], e[keep])
            retained |= keep
        strata.append(row)
    n = int(retained.sum())
    return {'strata': strata, 'total_strata': group_count,
            'eligible_strata': sum(s['unavailable_reason'] is None for s in strata),
            'support_count': len(u), 'retained_count': n, 'retained_fraction': n / len(u) if len(u) else 0.,
            'full_support_raw_rho': rho(u, e), 'same_retained_pixels_raw_rho': rho(u[retained], e[retained]),
            'within_bin_rank_coefficient': numerator / np.sqrt(ss_u * ss_e) if ss_u > 0 and ss_e > 0 else None,
            'availability': 'available' if n else 'no-eligible-strata'}


def aggregate(records):
    summary = {}
    for m in ('rgb', 'thr'):
        summary[m] = {}
        for variant in ('depth', 'depth_multi', 'depth_filtered'):
            summary[m][variant] = {}
            for support in ('own_support', 'common_support'):
                rows = [r for r in records if r['modality'] == m and r['variant'] == variant and r['support'] == support]
                assert len(rows) == 20
                result = {}
                for key in ('uncertainty_inverse_depth_rho', 'error_inverse_depth_rho'):
                    a = [r[key] for r in rows if r[key] is not None]
                    result[key] = {'available_images': len(a), 'median': float(np.median(a)) if a else None}
                for name in ('depth', 'spatial', 'joint'):
                    cases = [r['controls'][name] for r in rows]
                    stats = {'total_images': 20, 'available_images': sum(c['availability'] == 'available' for c in cases)}
                    for key in ('full_support_raw_rho', 'same_retained_pixels_raw_rho', 'within_bin_rank_coefficient', 'retained_fraction', 'eligible_strata'):
                        a = [c[key] for c in cases if c[key] is not None]
                        stats[key] = {'available_images': len(a), 'median': float(np.median(a)) if a else None,
                                      'min': min(a) if a else None, 'max': max(a) if a else None}
                    result[name] = stats
                summary[m][variant][support] = result
    return summary


def run(root, source, output):
    root, source, output = Path(root).resolve(), Path(source).resolve(), Path(output)
    output.mkdir(parents=True, exist_ok=False)
    original_path = source / 'report.json'
    original = json.loads(original_path.read_text())
    assert original['status'] == 'PASS' and not original['physical_cross_modal_error_eligible']
    manifest_path = root / 'results/ms2-timestamp-semantics/source-protocol-input-manifest.json'
    manifest = json.loads(manifest_path.read_text())
    frozen = json.loads((source / 'frozen-inputs.json').read_text())
    for path in [original_path, source / 'frozen-inputs.json', source / 'validation.json', Path(__file__)]:
        frozen[str(path.resolve())] = sha(path)
    for rec in original['records']:
        frozen[str(source / rec['file'])] = rec['sha256']
    assert all(sha(p) == h for p, h in frozen.items())
    (output / 'frozen-inputs.json').write_text(json.dumps(frozen, indent=2) + '\n')
    config = {'depth_edges_m_lower_inclusive': DEPTH_EDGES, 'last_depth_upper': 'infinity',
              'spatial_rows': 3, 'spatial_columns': 3, 'minimum_count_per_stratum': MINIMUM,
              'common_support_depth_group_source': 'primary-single-scan at same coordinates for all variants',
              'coefficient': 'Pearson of concatenated within-stratum centered ranks divided by stratum count',
              'eligibility': 'count>=30 and nonconstant U and E; retain exclusions',
              'new_inference': False, 'physical_cross_modal_error_eligible': False,
              'joint_timing_eligible': 0, 'exploratory': True}
    (output / 'protocol.json').write_text(json.dumps(config, indent=2) + '\n')
    primary = {(r['modality'], r['stem']): r for r in original['records'] if r['variant'] == 'depth'}
    result = []
    started = datetime.now(ZoneInfo('America/New_York')).isoformat()
    for rec in original['records']:
        m, stem, variant = rec['modality'], rec['stem'], rec['variant']
        height, width = manifest['records'][int(stem)]['cameras'][m]['left']['shape'][:2]
        with np.load(source / primary[m, stem]['file'], allow_pickle=False) as p:
            p_common = p['common_support']
            p_keys = p['y'][p_common].astype(np.int64) * width + p['x'][p_common]
            p_depth = p['depth_m'][p_common]
        with np.load(source / rec['file'], allow_pickle=False) as data:
            for support in ('own_support', 'common_support'):
                keep = np.ones(len(data['x']), dtype=bool) if support == 'own_support' else data['common_support']
                x, y = data['x'][keep], data['y'][keep]
                depth = data['depth_m'][keep]
                if support == 'common_support':
                    assert np.array_equal(y.astype(np.int64) * width + x, p_keys)
                    grouping_depth = p_depth
                else:
                    grouping_depth = depth
                u = data['covariance_normalized'][keep].sum(axis=1)
                e = np.linalg.norm(data['residual_normalized'][keep], axis=1)
                assert np.isfinite(u).all() and np.isfinite(e).all() and np.isfinite(grouping_depth).all()
                dg = np.searchsorted(DEPTH_EDGES, grouping_depth, side='right') - 1
                sg = (y.astype(np.int64) * 3 // height) * 3 + x.astype(np.int64) * 3 // width
                assert ((0 <= dg) & (dg < 6)).all() and ((0 <= sg) & (sg < 9)).all()
                controls = {name: control(u, e, groups, count) for name, groups, count in
                            [('depth', dg, 6), ('spatial', sg, 9), ('joint', dg * 9 + sg, 54)]}
                result.append({'modality': m, 'variant': variant, 'stem': stem, 'support': support,
                               'source_file': str(source / rec['file']), 'count': len(u),
                               'timing': rec['timing'], 'uncertainty_inverse_depth_rho': rho(u, 1 / grouping_depth),
                               'error_inverse_depth_rho': rho(e, 1 / grouping_depth),
                               'depth_group_reference': 'single-scan' if support == 'common_support' else variant,
                               'controls': controls})
        print(m, variant, stem, flush=True)
    assert len(result) == 240 and all(sha(p) == h for p, h in frozen.items())
    report = {'status': 'PASS', 'started_at': started, 'finished_at': datetime.now(ZoneInfo('America/New_York')).isoformat(),
              'frozen_inputs_unchanged': True, 'frozen_input_count': len(frozen), 'records': result,
              'aggregate': aggregate(result), 'exploratory': True, 'new_model_inference': False,
              'physical_cross_modal_error_eligible': False}
    (output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print('PASS240 case/support combinations and720 fixed controls.')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    run(a.root, a.source, a.output)
