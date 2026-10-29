"""Saved objective/geometry decomposition, followed by a separate outcome join.

Fixed-landmark unrobust leverage and information describe the saved objective;
they are not calibrated confidence, convergence telemetry or physical truth.
"""
import argparse
from datetime import datetime
from zoneinfo import ZoneInfo
import hashlib
import json
from pathlib import Path
import numpy as np
from Scripts.Setup.audit_optimizer_factors import camera_points, residual_jacobian, pair_factors

FACTOR = 'results/optimizer-factor-audit/run-005'
COMPARISON = 'results/factor-proxy-comparison/run-001'
DEPTH_NAMES = ('below_5', '5_to_20', '20_to_80', '80_plus')
DESCRIPTORS = ('median_depth_m', 'uv_variance_trace_median_px2', 'disparity_variance_median_px2',
               'uv_energy_per_factor', 'disparity_energy_per_factor', 'top1pct_energy_share',
               'top1pct_leverage_share', 'near5_leverage_share', 'far80_leverage_share',
               'translation_information_trace_m_inv2', 'rotation_information_trace_rad_inv2')
FEATURES = ('median_reprojection_px', 'median_abs_disparity_px', 'whitened_mean_per_component', 'mixed_units_condition_number')


def read(p): return json.loads(Path(p).read_text())
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p, x): Path(p).write_text(json.dumps(x, indent=2, allow_nan=False)+'\n')


def decompose(depth, pixels, height_width, residual, jacobian, covariance):
    depth, pixels, r, j, c = [np.asarray(a, dtype=float) for a in (depth, pixels, residual, jacobian, covariance)]
    n = len(depth); height, width = np.asarray(height_width).reshape(2)
    if not n or depth.shape != (n,) or pixels.shape != (n, 2) or r.shape != (n, 3) or j.shape != (n, 3, 6) or c.shape != (n, 3, 3):
        raise ValueError('nonempty aligned factor blocks required')
    if not all(np.isfinite(a).all() for a in (depth, pixels, r, j, c)) or np.any(depth <= 0) or not np.isfinite([height, width]).all() or min(height, width) <= 0 or np.any(pixels < 0) or np.any(pixels >= [width, height]):
        raise ValueError('finite positive depths and bounded target pixels required')
    if not np.allclose(c, c.transpose(0, 2, 1), rtol=0, atol=1e-12) or np.any(c[:, :2, 2] != 0) or np.any(c[:, 2, :2] != 0):
        raise ValueError('symmetric block diagonal UV/disparity covariance required')
    try: l = np.linalg.cholesky(c)
    except np.linalg.LinAlgError as error: raise ValueError('positive definite covariance required') from error
    wr = np.linalg.solve(l, r[..., None])[..., 0]; wj = np.linalg.solve(l, j)
    u, singular, _ = np.linalg.svd(wj.reshape(-1, 6), full_matrices=False)
    rank = int(sum(singular > max(3*n, 6)*np.finfo(float).eps*singular[0]))
    leverage = np.sum(u[:, :rank]**2, axis=1).reshape(n, 3).sum(1)
    components = {'uv_energy': np.sum(wr[:, :2]**2, axis=1), 'disparity_energy': wr[:, 2]**2,
                  'leverage': leverage, 'translation_information': np.sum(wj[:, :, :3]**2, axis=(1, 2)),
                  'rotation_information': np.sum(wj[:, :, 3:]**2, axis=(1, 2))}
    totals = {k: float(v.sum()) for k, v in components.items()}
    def group(mask):
        values = {k: float(v[mask].sum()) for k, v in components.items()}
        return {'count': int(mask.sum()), 'sums': values,
                'shares': {k: value/totals[k] if totals[k] else None for k, value in values.items()}}
    depth_ids = np.searchsorted([5, 20, 80], depth, side='right')
    cells = np.floor(pixels/[width, height]*3).astype(int); cell_ids = cells[:, 1]*3+cells[:, 0]
    ranges = {name: group(depth_ids == i) for i, name in enumerate(DEPTH_NAMES)}
    cell_groups = {str(i): group(cell_ids == i) for i in range(9)}
    count = max(1, int(np.ceil(n*.01)))
    energy = components['uv_energy']+components['disparity_energy']
    energy_total = float(energy.sum())
    descriptors = {'median_depth_m': float(np.median(depth)),
        'uv_variance_trace_median_px2': float(np.median(c[:, 0, 0]+c[:, 1, 1])),
        'disparity_variance_median_px2': float(np.median(c[:, 2, 2])),
        'uv_energy_per_factor': totals['uv_energy']/n, 'disparity_energy_per_factor': totals['disparity_energy']/n,
        'top1pct_energy_share': float(np.sort(energy)[-count:].sum()/energy_total) if energy_total else None,
        'top1pct_leverage_share': float(np.sort(leverage)[-count:].sum()/totals['leverage']) if totals['leverage'] else None,
        'near5_leverage_share': ranges['below_5']['shares']['leverage'], 'far80_leverage_share': ranges['80_plus']['shares']['leverage'],
        'translation_information_trace_m_inv2': totals['translation_information'],
        'rotation_information_trace_rad_inv2': totals['rotation_information']}
    return {'factor_count': n, 'numerical_rank': rank, 'depth_quantiles_m': np.quantile(depth, [0, .5, .9, 1]).tolist(),
            'top1pct_factor_budget': count, 'totals': totals, 'depth_bins': ranges, 'target_3x3_cells': cell_groups, 'descriptors': descriptors}


def frozen(root, protocol):
    if protocol['depth_edges_m'] != [0, 5, 20, 80, 'infinity'] or protocol['tail_fraction'] != .01 or protocol['fitted'] or protocol['geometry_uses_labels']:
        raise ValueError('decomposition contract changed')
    for rel, digest in protocol['feature_input_sha256'].items():
        if sha(root/rel) != digest: raise ValueError('frozen geometry source changed: '+rel)


def prepare(root, output):
    output.mkdir(parents=True, exist_ok=False)
    factor = read(root/FACTOR/'audit.json'); previous = read(root/COMPARISON/'protocol.json')
    files = {root/FACTOR/f for f in ('audit.json', 'validation.json')}
    files |= {root/p for p in factor['input_sha256']}
    files |= {root/COMPARISON/f for f in ('predictions.json', 'protocol.json')}
    files |= {root/p for p in previous['input_sha256']}
    files.add(Path(__file__))
    for rel, digest in factor['input_sha256'].items():
        if sha(root/rel) != digest: raise ValueError('historical factor source changed')
    write(output/'protocol.json', {'created_at': datetime.now(ZoneInfo('America/New_York')).isoformat(), 'base_commit': '22465ee54',
        'depth_edges_m': [0, 5, 20, 80, 'infinity'], 'tail_fraction': .01, 'cells': 'all nine row-major target-image 3x3 cells',
        'geometry_uses_labels': False, 'fitted': False, 'features': list(FEATURES), 'outcome_tie_tolerance': 1e-12,
        'outcome_classes': ['better', 'worse', 'tie'], 'examples': 'first chronological interval in each nonempty class, no extremum selection',
        'feature_input_sha256': {str(p.relative_to(root)): sha(p) for p in sorted(files)},
        'outcome_input_sha256': {str((root/COMPARISON/f).relative_to(root)): sha(root/COMPARISON/f) for f in ('analysis.json', 'validation.json')},
        'scope': 'retrospective fixed-landmark unrobust objective/geometry accounting, not causal attribution or calibrated confidence'})


def geometry(root, output):
    if (output/'geometry.json').exists(): raise FileExistsError('geometry already exists')
    protocol = read(output/'protocol.json'); frozen(root, protocol)
    audit = read(root/FACTOR/'audit.json'); proof = read(root/FACTOR/'validation.json')
    if proof['status'] != 'PASS' or proof['audit_sha256'] != sha(root/FACTOR/'audit.json'): raise ValueError('unverified factors')
    factors = {(r['case'], r['camera'], r['pair_ordinal']): r for r in audit['rows']}
    prediction = read(root/COMPARISON/'predictions.json')
    if prediction['labels_loaded'] or prediction['protocol_sha256'] != sha(root/COMPARISON/'protocol.json'): raise ValueError('unverified predictions')
    rows = []
    for decision in prediction['rows']:
        directory = read(root/COMPARISON/'protocol.json')['cases'][decision['window']]
        row = {k: decision[k] for k in ('window', 'pair_ordinal', 'source_decision_sequence', 'deadline_monotonic_ns', 'availability')}; row['experts'] = {}
        for camera in ('rgb', 'thermal'):
            if not decision['availability'][camera]: row['experts'][camera] = None; continue
            f = factors[directory, camera, decision['pair_ordinal']]
            if not f['available_at_original_deadline'] or f['motion_event_sequence'] != decision['motion_event_sequences'][camera] or f['motion_observation_monotonic_ns'] > decision['deadline_monotonic_ns']: raise ValueError('future factor')
            expert = root/'results'/directory/'run-001/experts'/camera; i = decision['pair_ordinal']
            with np.load(expert/'tensor-map.npz', allow_pickle=False) as saved, np.load(expert/'uncertainty'/f'pair-{i:03d}.npz', allow_pickle=False) as raw:
                mask, world = pair_factors(saved, i); p = camera_points(world, f['native_pose_xyzw'])
                uv = saved['match//pixel2_uv'][mask]; k = saved['frames//K'][i+1]
                r, j = residual_jacobian(p, uv, saved['match//pixel2_disp'][mask], k, float(saved['frames//baseline'][i+1]))
                c = np.zeros((len(p), 3, 3)); v = saved['match//pixel2_uv_cov'][mask]
                c[:, 0, 0], c[:, 1, 1] = v[:, 0], v[:, 1]; c[:, 0, 1] = c[:, 1, 0] = v[:, 2]
                c[:, 2, 2] = saved['match//pixel2_disp_cov'][mask].ravel()
                result = decompose(p[:, 0], uv, raw['height_width'], r, j, c)
                np.testing.assert_allclose(result['totals']['uv_energy']+result['totals']['disparity_energy'], f['diagnostics']['whitened_sum_squares'], rtol=1e-12)
                result['motion_event_sequence'] = f['motion_event_sequence']; row['experts'][camera] = result
        rows.append(row)
    frozen(root, protocol)
    write(output/'geometry.json', {'created_at': datetime.now(ZoneInfo('America/New_York')).isoformat(), 'protocol_sha256': sha(output/'protocol.json'), 'labels_loaded': False, 'rows': rows})
    print(json.dumps({'rows': len(rows), 'available_camera_slots': sum(e is not None for r in rows for e in r['experts'].values()), 'labels_loaded': False}))


def describe(values):
    valid = [v for v in values if v is not None]
    return {'count': len(valid), 'missing': len(values)-len(valid), 'median': float(np.median(valid)) if valid else None}


def outcome(root, output):
    if (output/'outcomes.json').exists(): raise FileExistsError('outcomes already exist')
    protocol = read(output/'protocol.json'); frozen(root, protocol)
    for p, digest in protocol['outcome_input_sha256'].items():
        if sha(root/p) != digest: raise ValueError('frozen outcome changed')
    validation = read(root/COMPARISON/'validation.json'); comparison = read(root/COMPARISON/'analysis.json'); g = read(output/'geometry.json')
    if validation['status'] != 'PASS' or validation['analysis_sha256'] != sha(root/COMPARISON/'analysis.json') or g['protocol_sha256'] != sha(output/'protocol.json') or g['labels_loaded']: raise ValueError('unverified geometry or outcomes')
    by_pair = {(r['window'], r['pair_ordinal']): r for r in g['rows']}
    records = []; groups = {}; camera_summaries = {}
    for window in ('early', 'middle', 'day-middle'):
        for camera in ('rgb', 'thermal'):
            available = [r['experts'][camera] for r in g['rows'] if r['window'] == window and r['experts'][camera] is not None]
            camera_summaries[window+'/'+camera] = {'motions': len(available), 'factor_count': sum(e['factor_count'] for e in available),
                'descriptors': {name: describe([e['descriptors'][name] for e in available]) for name in DESCRIPTORS}}
        for feature in FEATURES:
            for loss in ('translation_m', 'rotation_deg'):
                subsets = {k: [] for k in ('better', 'worse', 'tie')}; missing = []; ineligible = []
                for r in comparison['rows']:
                    if r['window'] != window: continue
                    key = r['window'], r['pair_ordinal']; geo = by_pair[key]
                    if geo['source_decision_sequence'] != r['source_decision_sequence'] or geo['availability'] != r['availability']: raise ValueError('geometry/outcome identity mismatch')
                    if not all(r['availability'].values()): ineligible.append(r['pair_ordinal']); continue
                    camera = r['choices'][feature]['action'].lower()
                    if camera not in ('rgb', 'thermal'): missing.append(r['pair_ordinal']); continue
                    other = 'thermal' if camera == 'rgb' else 'rgb'; a, b = r['losses'][camera][loss], r['losses'][other][loss]
                    kind = 'tie' if abs(a-b) <= 1e-12 else 'better' if a < b else 'worse'
                    record = {'window': window, 'pair_ordinal': r['pair_ordinal'], 'feature': feature, 'loss': loss, 'selected_camera': camera,
                              'class': kind, 'selected_loss': a, 'other_loss': b, 'selected_descriptors': geo['experts'][camera]['descriptors'], 'other_descriptors': geo['experts'][other]['descriptors']}
                    records.append(record); subsets[kind].append(record)
                group = {'ineligible_pair_ordinals': ineligible, 'missing_feature_pair_ordinals': missing, 'classes': {}}
                for kind, subset in subsets.items():
                    group['classes'][kind] = {'count': len(subset), 'pair_ordinals': [r['pair_ordinal'] for r in subset],
                        'first_chronological_example': min((r['pair_ordinal'] for r in subset), default=None),
                        'selected': {name: describe([r['selected_descriptors'][name] for r in subset]) for name in DESCRIPTORS},
                        'other': {name: describe([r['other_descriptors'][name] for r in subset]) for name in DESCRIPTORS}}
                groups[window+'/'+feature+'/'+loss] = group
    write(output/'outcomes.json', {'created_at': datetime.now(ZoneInfo('America/New_York')).isoformat(), 'geometry_sha256': sha(output/'geometry.json'),
          'protocol_sha256': sha(output/'protocol.json'), 'camera_summaries': camera_summaries, 'groups': groups, 'rows': records,
          'scope': 'reference-assisted descriptive better/worse decomposition; no fitted separator, causal explanation, or independent physical truth'})
    print(json.dumps({'outcome_records': len(records), 'groups': len(groups)}))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('action', choices=('prepare', 'geometry', 'outcome')); p.add_argument('--root', type=Path, required=True); p.add_argument('--output', type=Path, required=True)
    a = p.parse_args(); {'prepare': prepare, 'geometry': geometry, 'outcome': outcome}[a.action](a.root, a.output)
