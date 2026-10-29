"""Independent raw forward/finite-difference/QR geometry and outcome audit."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

FEATURES = ('median_reprojection_px', 'median_abs_disparity_px', 'whitened_mean_per_component', 'mixed_units_condition_number')
DESCRIPTORS = ('median_depth_m', 'uv_variance_trace_median_px2', 'disparity_variance_median_px2', 'uv_energy_per_factor',
               'disparity_energy_per_factor', 'top1pct_energy_share', 'top1pct_leverage_share', 'near5_leverage_share', 'far80_leverage_share',
               'translation_information_trace_m_inv2', 'rotation_information_trace_rad_inv2')
DEPTH_NAMES = ('below_5', '5_to_20', '20_to_80', '80_plus')


def load(p): return json.loads(Path(p).read_text())
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def compare(a, b):
    if isinstance(b, dict):
        assert isinstance(a, dict) and set(a) == set(b)
        for key in b: compare(a[key], b[key])
    elif isinstance(b, list):
        assert isinstance(a, list) and len(a) == len(b)
        for x, y in zip(a, b): compare(x, y)
    elif b is None or isinstance(b, (str, bool, int)): assert a == b
    else: assert math.isclose(a, b, rel_tol=2e-6, abs_tol=1e-9), (a, b)


def rotation(pose):
    x, y, z, w = np.asarray(pose).reshape(7)[3:]; length = np.sqrt(x*x+y*y+z*z+w*w); x, y, z, w = np.array([x, y, z, w])/length
    return np.array([[1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w], [2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w], [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y]])


def independent(saved, pair, pose, shape):
    indices = np.flatnonzero((saved['edge/match2frame1/mapping'] == pair) & (saved['edge/match2frame2/mapping'] == pair+1))
    ids = saved['edge/match2point/mapping'][indices]; world = saved['points//pos_Tw'][ids].astype(float)
    t = np.asarray(pose).reshape(7); points = (world-t[:3]) @ rotation(t); n = len(points)
    uv = saved['match//pixel2_uv'][indices]; disp = saved['match//pixel2_disp'][indices].ravel()
    k = saved['frames//K'][pair+1].astype(float); baseline = float(saved['frames//baseline'][pair+1])
    def forward(v):
        q = (points-v[:3]) @ Rotation.from_rotvec(v[3:]).as_matrix(); x, y, z = q.T
        return np.column_stack((k[0, 0]*y/x+k[0, 2]-uv[:, 0], k[1, 1]*z/x+k[1, 2]-uv[:, 1], k[0, 0]*baseline/x-disp))
    residual = forward(np.zeros(6)); step = 1e-6
    j = np.stack([(forward(v*step)-forward(-v*step))/(2*step) for v in np.eye(6)], axis=-1)
    v = saved['match//pixel2_uv_cov'][indices].astype(float); dvar = saved['match//pixel2_disp_cov'][indices].ravel().astype(float)
    covariance = np.zeros((n, 3, 3)); covariance[:, 0, 0] = v[:, 0]; covariance[:, 1, 1] = v[:, 1]
    covariance[:, 0, 1] = covariance[:, 1, 0] = v[:, 2]; covariance[:, 2, 2] = dvar
    l = np.linalg.cholesky(covariance); a = np.linalg.solve(l, j).reshape(-1, 6)
    assert np.linalg.matrix_rank(a) == 6
    q, _ = np.linalg.qr(a, mode='reduced'); lev = (q*q).sum(1).reshape(n, 3).sum(1)
    uv_energy = np.einsum('ni,nij,nj->n', residual[:, :2], np.linalg.inv(covariance[:, :2, :2]), residual[:, :2])
    disp_energy = residual[:, 2]**2/dvar; per = a.reshape(n, 3, 6)
    contributions = {'uv_energy': uv_energy, 'disparity_energy': disp_energy, 'leverage': lev,
                     'translation_information': (per[:, :, :3]**2).sum((1, 2)), 'rotation_information': (per[:, :, 3:]**2).sum((1, 2))}
    totals = {name: math.fsum(values) for name, values in contributions.items()}
    def group(take):
        sums = {name: math.fsum(values[take]) for name, values in contributions.items()}
        return {'count': int(sum(take)), 'sums': sums, 'shares': {name: value/totals[name] if totals[name] else None for name, value in sums.items()}}
    depth = points[:, 0]; edges = (0, 5, 20, 80, float('inf'))
    bins = {DEPTH_NAMES[i]: group((depth >= edges[i]) & (depth < edges[i+1])) for i in range(4)}
    height, width = shape; cells = {}
    for i in range(9):
        row, column = divmod(i, 3)
        take = (uv[:, 0] >= column*width/3) & (uv[:, 0] < (column+1)*width/3) & (uv[:, 1] >= row*height/3) & (uv[:, 1] < (row+1)*height/3)
        cells[str(i)] = group(take)
    tail_count = math.ceil(n/100); energy = uv_energy+disp_energy
    descriptors = {'median_depth_m': float(np.median(depth)), 'uv_variance_trace_median_px2': float(np.median(v[:, 0]+v[:, 1])),
        'disparity_variance_median_px2': float(np.median(dvar)), 'uv_energy_per_factor': totals['uv_energy']/n, 'disparity_energy_per_factor': totals['disparity_energy']/n,
        'top1pct_energy_share': math.fsum(sorted(energy)[-tail_count:])/math.fsum(energy) if math.fsum(energy) else None,
        'top1pct_leverage_share': math.fsum(sorted(lev)[-tail_count:])/totals['leverage'],
        'near5_leverage_share': bins['below_5']['shares']['leverage'], 'far80_leverage_share': bins['80_plus']['shares']['leverage'],
        'translation_information_trace_m_inv2': totals['translation_information'], 'rotation_information_trace_rad_inv2': totals['rotation_information']}
    return {'factor_count': n, 'numerical_rank': 6, 'depth_quantiles_m': [float(np.quantile(depth, value)) for value in (0, .5, .9, 1)],
            'top1pct_factor_budget': tail_count, 'totals': totals, 'depth_bins': bins, 'target_3x3_cells': cells, 'descriptors': descriptors}


def describe(values):
    valid = sorted(v for v in values if v is not None); n = len(valid)
    median = valid[n//2] if n % 2 else (valid[n//2-1]+valid[n//2])/2 if n else None
    return {'count': n, 'missing': len(values)-n, 'median': median}


def validate(root, output):
    protocol = load(output/'protocol.json'); geo = load(output/'geometry.json'); outcome = load(output/'outcomes.json')
    assert protocol['depth_edges_m'] == [0, 5, 20, 80, 'infinity'] and protocol['tail_fraction'] == .01 and not protocol['fitted'] and not protocol['geometry_uses_labels']
    assert protocol['features'] == list(FEATURES) and protocol['outcome_tie_tolerance'] == 1e-12
    assert geo['labels_loaded'] is False and geo['protocol_sha256'] == outcome['protocol_sha256'] == sha(output/'protocol.json')
    assert outcome['geometry_sha256'] == sha(output/'geometry.json')
    for section in ('feature_input_sha256', 'outcome_input_sha256'):
        for rel, digest in protocol[section].items(): assert sha(root/rel) == digest
    assert not any(rel.endswith('/analysis.json') or '/proxy-001/' in rel or 'arbiter-pose-proxy/' in rel for rel in protocol['feature_input_sha256'])
    factor = load(root/'results/optimizer-factor-audit/run-005/audit.json'); fproof = load(root/'results/optimizer-factor-audit/run-005/validation.json')
    assert fproof['status'] == 'PASS' and fproof['audit_sha256'] == sha(root/'results/optimizer-factor-audit/run-005/audit.json')
    factors = {(r['case'], r['camera'], r['pair_ordinal']): r for r in factor['rows']}
    pred = load(root/'results/factor-proxy-comparison/run-001/predictions.json'); evaluation = load(root/'results/factor-proxy-comparison/run-001/analysis.json'); vproof = load(root/'results/factor-proxy-comparison/run-001/validation.json')
    assert vproof['status'] == 'PASS' and vproof['analysis_sha256'] == sha(root/'results/factor-proxy-comparison/run-001/analysis.json') and vproof['predictions_sha256'] == sha(root/'results/factor-proxy-comparison/run-001/predictions.json')
    predictions = {(r['window'], r['pair_ordinal']): r for r in pred['rows']}; geos = {}; slots = blocks = 0
    cases = {'early': 'shared-arbiter-events', 'middle': 'arbiter-support-collapse', 'day-middle': 'history-day-replication'}
    for r in geo['rows']:
        key = r['window'], r['pair_ordinal']; assert key not in geos; geos[key] = r; d = predictions[key]
        assert {k: r[k] for k in ('window', 'pair_ordinal', 'source_decision_sequence', 'deadline_monotonic_ns', 'availability')} == {k: d[k] for k in ('window', 'pair_ordinal', 'source_decision_sequence', 'deadline_monotonic_ns', 'availability')}
        for camera in ('rgb', 'thermal'):
            if not d['availability'][camera]: assert r['experts'][camera] is None; continue
            f = factors[cases[r['window']], camera, r['pair_ordinal']]; e = r['experts'][camera]
            assert f['available_at_original_deadline'] and e['motion_event_sequence'] == f['motion_event_sequence'] == d['motion_event_sequences'][camera] and f['motion_observation_monotonic_ns'] <= r['deadline_monotonic_ns']
            expert = root/'results'/cases[r['window']]/'run-001/experts'/camera
            with np.load(expert/'tensor-map.npz', allow_pickle=False) as saved, np.load(expert/'uncertainty'/f'pair-{r["pair_ordinal"]:03d}.npz', allow_pickle=False) as raw:
                expected = independent(saved, r['pair_ordinal'], f['native_pose_xyzw'], raw['height_width']); expected['motion_event_sequence'] = f['motion_event_sequence']
                compare(e, expected); slots += 1; blocks += e['factor_count']
                assert e['factor_count'] == f['retained_factor_count']
                assert math.isclose(e['totals']['leverage'], e['numerical_rank'], rel_tol=1e-12)
                assert math.isclose(e['totals']['uv_energy']+e['totals']['disparity_energy'], f['diagnostics']['whitened_sum_squares'], rel_tol=1e-12)
    assert len(geos) == len(predictions) == 217 and set(geos) == set(predictions) and slots == 331
    for key, summary in outcome['camera_summaries'].items():
        window, camera = key.split('/'); available = [r['experts'][camera] for r in geo['rows'] if r['window'] == window and r['experts'][camera] is not None]
        compare(summary, {'motions': len(available), 'factor_count': sum(e['factor_count'] for e in available), 'descriptors': {name: describe([e['descriptors'][name] for e in available]) for name in DESCRIPTORS}})
    expected_records = []; expected_keys = set()
    for window in cases:
        for feature in FEATURES:
            for loss in ('translation_m', 'rotation_deg'):
                key = window+'/'+feature+'/'+loss; expected_keys.add(key); group = outcome['groups'][key]
                partitions = {name: [] for name in ('better', 'worse', 'tie')}; ineligible = []; missing = []
                for r in evaluation['rows']:
                    if r['window'] != window: continue
                    if not all(r['availability'].values()): ineligible.append(r['pair_ordinal']); continue
                    chosen = r['choices'][feature]['action'].lower()
                    if chosen not in ('rgb', 'thermal'): missing.append(r['pair_ordinal']); continue
                    other = 'thermal' if chosen == 'rgb' else 'rgb'; a, b = r['losses'][chosen][loss], r['losses'][other][loss]
                    kind = 'tie' if abs(a-b) <= 1e-12 else 'better' if a < b else 'worse'; g = geos[window, r['pair_ordinal']]
                    record = {'window': window, 'pair_ordinal': r['pair_ordinal'], 'feature': feature, 'loss': loss, 'selected_camera': chosen,
                              'class': kind, 'selected_loss': a, 'other_loss': b, 'selected_descriptors': g['experts'][chosen]['descriptors'], 'other_descriptors': g['experts'][other]['descriptors']}
                    expected_records.append(record); partitions[kind].append(record)
                assert group['ineligible_pair_ordinals'] == ineligible and group['missing_feature_pair_ordinals'] == missing
                assert set(group['classes']) == set(partitions)
                for kind, values in partitions.items():
                    expected = {'count': len(values), 'pair_ordinals': [r['pair_ordinal'] for r in values], 'first_chronological_example': min((r['pair_ordinal'] for r in values), default=None),
                                'selected': {name: describe([r['selected_descriptors'][name] for r in values]) for name in DESCRIPTORS},
                                'other': {name: describe([r['other_descriptors'][name] for r in values]) for name in DESCRIPTORS}}
                    compare(group['classes'][kind], expected)
    assert set(outcome['groups']) == expected_keys and outcome['rows'] == expected_records and len(expected_records) == 936
    verification = {'status': 'PASS', 'deadlines': len(geos), 'camera_slots': 434, 'available_motions': slots, 'factor_blocks': blocks,
        'outcome_records': len(expected_records), 'outcome_groups': len(expected_keys), 'geometry_sha256': sha(output/'geometry.json'), 'outcomes_sha256': sha(output/'outcomes.json'),
        'protocol_sha256': sha(output/'protocol.json'), 'validator_sha256': sha(Path(__file__)), 'scope': 'independent finite-difference/QR/partition and descriptive-outcome validation; no causal or physical-error identification'}
    (output/'validation.json').write_text(json.dumps(verification, indent=2)+'\n'); print(json.dumps(verification))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('--root', type=Path, required=True); p.add_argument('--output', type=Path, required=True)
    a = p.parse_args(); validate(a.root, a.output)
