"""Fixed source-ID-preserving day windows, reusing existing proxy/rank mathematics."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import cv2
import numpy as np
from Scripts.Setup.ms2_day_replication import score, sha, dump, SEQUENCE
from Scripts.Setup.audit_ms2_pilot import read_calibration
from Scripts.Setup.ms2_depth_spatial_controls import control, aggregate, rho, DEPTH_EDGES


def prepare(root, base):
    root, base = Path(root).resolve(), Path(base).resolve()
    assert not (base / 'source-manifest.json').exists()
    selection_path = base.parent / 'selection.json'
    selection = json.loads(selection_path.read_text())
    start = selection['windows'][base.name]['start']
    assert start in (1936, 3873) and selection['source_count'] == 5810
    old = json.loads((root / 'results/ms2-day-replication/source-manifest.json').read_text())
    manifest = deepcopy(old)
    manifest['scope'] = 'Fixed within-recording later window; original source IDs/times retained; no physical exposure assertion'
    manifest['source_window'] = {'start': start, 'count': 20, 'name': base.name, 'selection_rule': selection['rule']}
    sync = base / 'pilot/sync_data' / SEQUENCE
    calibration = read_calibration(sync / 'calib.npy')
    assert calibration == json.loads((root / 'results/ms2-day-replication/calibration.json').read_text())
    dump(base / 'calibration.json', calibration)
    frozen = {str(selection_path): sha(selection_path), str(Path(__file__).resolve()): sha(__file__)}
    for kind in ('sync_data', 'proj_depth'):
        for rec in json.loads((base / (kind + '-pilot-manifest.json')).read_text()):
            p = base / rec['path'];assert p.stat().st_size == rec['bytes'] and sha(p) == rec['sha256'];frozen[str(p)] = rec['sha256']
    times = {}
    for m in ('rgb', 'thr'):
        for side in ('left', 'right'):
            p = sync / m / ('img_' + side + '_timestamp.txt');a = [int(x) for x in p.read_text().split()]
            assert len(a) == 5810 and all(y > x for x, y in zip(a, a[1:]));times[m + '_' + side] = a
    rows = []
    for index in range(start, start + 20):
        row = deepcopy(old['records'][0]);row['stem'] = f'{index:06d}'
        for m in ('rgb', 'thr'):
            for side in ('left', 'right'):
                p = sync / m / ('img_' + side) / (row['stem'] + '.png');image = cv2.imread(str(p), cv2.IMREAD_UNCHANGED)
                assert image is not None and list(image.shape) == row['cameras'][m][side]['shape'] and str(image.dtype) == row['cameras'][m][side]['dtype']
                row['cameras'][m][side].update(path=str(p.relative_to(root)), sha256=sha(p), recorded_time_ns=times[m + '_' + side][index])
            for variant in ('depth', 'depth_multi', 'depth_filtered'):
                p = base / 'pilot/proj_depth' / SEQUENCE / m / variant / (row['stem'] + '.png')
                row['references'][m][variant].update(path=str(p.relative_to(root)), sha256=sha(p))
            p = base / 'pilot/odom' / SEQUENCE / m / (row['stem'] + '.txt')
            row['references'][m]['pose'].update(path=str(p.relative_to(root)), sha256=sha(p));frozen[str(p)] = sha(p)
        rows.append(row)
    manifest['records'] = rows;dump(base / 'source-manifest.json', manifest)
    frozen[str(base / 'source-manifest.json')] = sha(base / 'source-manifest.json')
    audit = {'sequence': SEQUENCE, 'source_window': manifest['source_window'], 'source_ids': [r['stem'] for r in rows],
             'cameras': {}, 'pairs': [], 'physical_cross_modal_error_eligible': False, 'provided_pair_depth_diagnostic_defined': True, 'frozen_inputs': frozen}
    for m in ('rgb', 'thr'):
        c = rows[0]['cameras'][m];supports = {};poses = [];rigidity = []
        for row in rows:
            a = np.loadtxt(root / row['references'][m]['pose']['path']).reshape(4, 4)
            assert np.isfinite(a).all() and np.array_equal(a[3], [0, 0, 0, 1])
            orth = float(np.linalg.norm(a[:3, :3].T @ a[:3, :3] - np.eye(3)));det = float(np.linalg.det(a[:3, :3]))
            rigidity.append({'orthogonality': orth, 'determinant': det, 'passes_1e_5_screen': orth < 1e-5 and abs(det - 1) < 1e-5});poses.append(a)
        for v in ('depth', 'depth_multi', 'depth_filtered'):
            counts = []
            for row in rows:
                raw = cv2.imread(str(root / row['references'][m][v]['path']), cv2.IMREAD_UNCHANGED)
                assert raw.dtype == np.uint16 and list(raw.shape) == c['left']['shape'][:2]
                z = raw.astype(float) / 256;nz = z > 0;d = np.zeros_like(z);d[nz] = c['K'][0][0] * c['baseline_m'] / z[nz]
                ur = np.arange(z.shape[1])[None, :] - d;keep = nz & (ur >= 0) & (ur < z.shape[1] - 1)
                counts.append({'nonzero_depth': int(nz.sum()), 'right_image_in_bounds': int(keep.sum())})
            supports[v] = counts
        offsets = [times[m + '_right'][i] - times[m + '_left'][i] for i in range(start, start + 20)]
        audit['cameras'][m] = {'depth_support': supports, 'rigidity': rigidity,
                               'source_duration_ns': times[m + '_left'][start + 19] - times[m + '_left'][start],
                               'reference_translation_column_change_m_not_error': float(np.linalg.norm(poses[-1][:3, 3] - poses[0][:3, 3])),
                               'window_abs_stereo_ms_min_median_max': [min(map(abs, offsets)) / 1e6, float(np.median(np.abs(offsets))) / 1e6, max(map(abs, offsets)) / 1e6]}
    for i, row in enumerate(rows):
        index = start + i;t = {m + '_' + s: times[m + '_' + s][index] for m in ('rgb', 'thr') for s in ('left', 'right')}
        a, b, c = t['rgb_right'] - t['rgb_left'], t['thr_right'] - t['thr_left'], t['thr_left'] - t['rgb_left']
        audit['pairs'].append({'stem': row['stem'], 'times_ns': t, 'rgb_stereo_ns': a, 'thermal_stereo_ns': b,
                               'cross_left_ns': c, 'inherited_joint_gate': abs(a) <= 2_000_000 and abs(b) <= 2_000_000 and abs(c) <= 30_000_000})
    nav = []
    for p in sorted((sync / 'gps_imu/data').glob('*.txt')):
        values = np.loadtxt(p).reshape(-1);assert len(values) == 16 and np.isfinite(values).all()
        nav.append({'stem': p.stem, 'filter_state': float(values[13]), 'dynamics_mode': float(values[14]), 'status_flags': float(values[15])})
    audit['source_navigation_status_not_accuracy'] = nav
    assert all(sha(p) == h for p, h in frozen.items());dump(base / 'protocol-audit.json', audit)
    print(base.name, 'PREPARE PASS;original IDs',rows[0]['stem'],rows[-1]['stem'],'joint gate',sum(r['inherited_joint_gate'] for r in audit['pairs']),flush=True)


def controls(root, base, source, output):
    root, base, source, output = map(lambda p: Path(p).resolve(), (root, base, source, output))
    output.mkdir(parents=True, exist_ok=False)
    original = json.loads((source / 'report.json').read_text());manifest = json.loads((base / 'source-manifest.json').read_text())
    rows = {r['stem']: r for r in manifest['records']}
    frozen = json.loads((source / 'frozen-inputs.json').read_text())
    for p in [source / 'report.json', Path(__file__)]:frozen[str(p)] = sha(p)
    for r in original['records']:frozen[str(source / r['file'])] = r['sha256']
    assert all(sha(p) == h for p, h in frozen.items());dump(output / 'frozen-inputs.json', frozen)
    primary = {(r['modality'], r['stem']): r for r in original['records'] if r['variant'] == 'depth'};records = []
    for r in original['records']:
        m, stem = r['modality'], r['stem'];h, w = rows[stem]['cameras'][m]['left']['shape'][:2]
        with np.load(source / primary[m, stem]['file'], allow_pickle=False) as p:
            pc = p['common_support'];keys = p['y'][pc].astype(np.int64) * w + p['x'][pc];primary_z = p['depth_m'][pc]
        with np.load(source / r['file'], allow_pickle=False) as data:
            for support in ('own_support', 'common_support'):
                keep = np.ones(len(data['x']), dtype=bool) if support == 'own_support' else data['common_support']
                x, y = data['x'][keep], data['y'][keep];z = data['depth_m'][keep]
                if support == 'common_support':assert np.array_equal(y.astype(np.int64) * w + x, keys);group_z = primary_z
                else:group_z = z
                u = data['covariance_normalized'][keep].sum(axis=1);e = np.linalg.norm(data['residual_normalized'][keep], axis=1)
                dg = np.searchsorted(DEPTH_EDGES, group_z, side='right') - 1;sg = (y.astype(np.int64) * 3 // h) * 3 + x.astype(np.int64) * 3 // w
                cs = {name: control(u, e, g, n) for name, g, n in [('depth', dg, 6), ('spatial', sg, 9), ('joint', dg * 9 + sg, 54)]}
                records.append({'modality': m, 'variant': r['variant'], 'stem': stem, 'support': support, 'source_file': str(source / r['file']),
                                'count': len(u), 'timing': r['timing'], 'uncertainty_inverse_depth_rho': rho(u, 1 / group_z), 'error_inverse_depth_rho': rho(e, 1 / group_z), 'controls': cs})
    assert len(records) == 240 and all(sha(p) == h for p, h in frozen.items())
    dump(output / 'protocol.json', {'inherited': 'EXP047 definitions unchanged; original stem lookup', 'source_window': manifest['source_window'], 'depth_edges': DEPTH_EDGES})
    dump(output / 'report.json', {'status': 'PASS', 'source_window': manifest['source_window'], 'frozen_inputs_unchanged': True,
                                  'physical_cross_modal_error_eligible': False, 'new_model_inference': False, 'records': records, 'aggregate': aggregate(records)})


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__);p.add_argument('stage', choices=['prepare', 'score', 'controls'])
    p.add_argument('--root', type=Path, required=True);p.add_argument('--base', type=Path, required=True)
    p.add_argument('--source', type=Path);p.add_argument('--output', type=Path);a = p.parse_args()
    if a.stage == 'prepare':prepare(a.root, a.base)
    else:
        if a.source is None or a.output is None:p.error('source/output required')
        (score if a.stage == 'score' else controls)(a.root, a.base, a.source, a.output)
