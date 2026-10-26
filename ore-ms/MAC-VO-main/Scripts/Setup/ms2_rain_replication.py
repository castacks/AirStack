"""Sequence-specific MS2 fixed rain replication; reuses frozen EXP046/047 mathematics.

Historical night runners remain unchanged. Source pairing is not exposure proof.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np

from Scripts.Setup.audit_ms2_pilot import read_calibration
from Scripts.Setup.ms2_stereo_depth_proxy import metrics, aggregate as aggregate_proxy, VARIANTS
from Scripts.Setup.ms2_depth_spatial_controls import control, aggregate as aggregate_controls, rho, DEPTH_EDGES

SEQUENCE = '_2021-08-06-16-19-00'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, data):
    Path(path).write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')


def prepare(root, base):
    root, base = Path(root).resolve(), Path(base).resolve()
    assert not (base / 'source-manifest.json').exists()
    pilot = base / 'pilot'
    sync = pilot / 'sync_data' / SEQUENCE
    calibration = read_calibration(sync / 'calib.npy')
    dump(base / 'calibration.json', calibration)
    template = json.loads((root / 'results/ms2-timestamp-semantics/source-protocol-input-manifest.json').read_text())
    manifest = deepcopy(template)
    manifest['sequence'] = SEQUENCE
    manifest['scope'] = 'Fixed distinct rain first20 IDs; no physical exposure/reference claim'
    manifest['inherited_2ms_stereo_30ms_cross_diagnostic'] = 'Computed in protocol-audit.json; thresholds unchanged'
    times = {}
    frozen = {}
    for kind in ('sync_data', 'proj_depth'):
        for row in json.loads((base / (kind + '-pilot-manifest.json')).read_text()):
            p = base / row['path']
            assert p.stat().st_size == row['bytes'] and sha(p) == row['sha256']
            frozen[str(p)] = row['sha256']
    for m in ('rgb', 'thr'):
        for side in ('left', 'right'):
            p = sync / m / ('img_' + side + '_timestamp.txt')
            a = [int(x) for x in p.read_text().split()]
            assert len(a) >= 20 and all(b > a for a, b in zip(a, a[1:]))
            times[m + '_' + side] = a
    for i, row in enumerate(manifest['records']):
        assert row['stem'] == f'{i:06d}'
        for m in ('rgb', 'thr'):
            K = calibration['K_' + m + 'L']['values']
            R = np.asarray(calibration['R_' + m + 'R']['values'])
            t = np.asarray(calibration['T_' + m + 'R']['values']).reshape(3) / 1000
            assert np.array_equal(R, np.eye(3)) and t[0] < 0 and np.array_equal(t[1:], [0, 0])
            camera = row['cameras'][m]
            camera['K'] = K
            camera['baseline_m'] = float(-t[0])
            for side in ('left', 'right'):
                p = sync / m / ('img_' + side) / (row['stem'] + '.png')
                raw = cv2.imread(str(p), cv2.IMREAD_UNCHANGED)
                assert raw is not None and raw.dtype == (np.uint8 if m == 'rgb' else np.uint16)
                assert list(raw.shape) == ([384, 1224, 3] if m == 'rgb' else [256, 640])
                camera[side] = {'path': str(p.relative_to(root)), 'sha256': sha(p),
                                'shape': list(raw.shape), 'dtype': str(raw.dtype),
                                'recorded_time_ns': times[m + '_' + side][i]}
            for variant in VARIANTS:
                p = pilot / 'proj_depth' / SEQUENCE / m / variant / (row['stem'] + '.png')
                rec = row['references'][m][variant]
                rec['path'], rec['sha256'] = str(p.relative_to(root)), sha(p)
            p = pilot / 'odom' / SEQUENCE / m / (row['stem'] + '.txt')
            row['references'][m]['pose']['path'] = str(p.relative_to(root))
            row['references'][m]['pose']['sha256'] = sha(p)
            frozen[str(p)] = sha(p)
    dump(base / 'source-manifest.json', manifest)
    frozen[str(base / 'source-manifest.json')] = sha(base / 'source-manifest.json')
    audit = {'sequence': SEQUENCE, 'source_ids': [r['stem'] for r in manifest['records']],
             'cameras': {}, 'pairs': [], 'physical_cross_modal_error_eligible': False,
             'provided_pair_depth_diagnostic_defined': True, 'frozen_inputs': frozen}
    for m in ('rgb', 'thr'):
        camera = manifest['records'][0]['cameras'][m]
        poses = []
        rigidity = []
        for row in manifest['records']:
            p = root / row['references'][m]['pose']['path']
            a = np.loadtxt(p).reshape(4, 4)
            assert np.isfinite(a).all() and np.array_equal(a[3], [0, 0, 0, 1])
            orth = float(np.linalg.norm(a[:3, :3].T @ a[:3, :3] - np.eye(3)))
            det = float(np.linalg.det(a[:3, :3]))
            rigidity.append({'orthogonality': orth, 'determinant': det, 'passes_1e_5_screen': orth < 1e-5 and abs(det - 1) < 1e-5})
            poses.append(a)
        support = {}
        for variant in VARIANTS:
            counts = []
            for row in manifest['records']:
                p = root / row['references'][m][variant]['path']
                z = cv2.imread(str(p), cv2.IMREAD_UNCHANGED)
                assert z.dtype == np.uint16 and list(z.shape) == camera['left']['shape'][:2]
                z = z.astype(float) / 256
                valid = z > 0
                d = np.zeros_like(z)
                d[valid] = camera['K'][0][0] * camera['baseline_m'] / z[valid]
                ur = np.arange(z.shape[1])[None, :] - d
                keep = valid & (ur >= 0) & (ur < z.shape[1] - 1)
                counts.append({'nonzero_depth': int(valid.sum()), 'right_image_in_bounds': int(keep.sum())})
            support[variant] = counts
        offsets = [b - a for a, b in zip(times[m + '_left'], times[m + '_right'])]
        audit['cameras'][m] = {'depth_support': support, 'rigidity': rigidity,
                               'initial_identity_max_abs': float(np.max(abs(poses[0] - np.eye(4)))),
                               'endpoint_translation_column_change_m_not_error': float(np.linalg.norm(poses[-1][:3, 3] - poses[0][:3, 3])),
                               'source_duration_ns': times[m + '_left'][19] - times[m + '_left'][0],
                               'full_stereo_within_2ms': sum(abs(d) <= 2_000_000 for d in offsets),
                               'full_stereo_count': len(offsets),
                               'full_abs_stereo_ms_min_median_max': [min(map(abs, offsets)) / 1e6, float(np.median(np.abs(offsets))) / 1e6, max(map(abs, offsets)) / 1e6]}
    for i, row in enumerate(manifest['records']):
        t = {m + '_' + s: times[m + '_' + s][i] for m in ('rgb', 'thr') for s in ('left', 'right')}
        a, b, c = t['rgb_right'] - t['rgb_left'], t['thr_right'] - t['thr_left'], t['thr_left'] - t['rgb_left']
        audit['pairs'].append({'stem': row['stem'], 'times_ns': t, 'rgb_stereo_ns': a, 'thermal_stereo_ns': b,
                               'cross_left_ns': c, 'inherited_joint_gate': abs(a) <= 2_000_000 and abs(b) <= 2_000_000 and abs(c) <= 30_000_000})
    nav = []
    for p in sorted((sync / 'gps_imu/data').glob('*.txt')):
        values = np.loadtxt(p).reshape(-1)
        assert len(values) == 16 and np.isfinite(values).all()
        nav.append({'stem': p.stem, 'filter_state': float(values[13]), 'dynamics_mode': float(values[14]), 'status_flags': float(values[15])})
    audit['source_navigation_status_not_accuracy'] = nav
    assert all(sha(p) == h for p, h in frozen.items())
    dump(base / 'protocol-audit.json', audit)
    print('RAIN PREPARE PASS; physical joint timing eligibility', sum(p['inherited_joint_gate'] for p in audit['pairs']))


def score(root, base, frontend, output):
    root, base, frontend, output = map(lambda p: Path(p).resolve(), (root, base, frontend, output))
    output.mkdir(parents=True, exist_ok=False)
    manifest = json.loads((base / 'source-manifest.json').read_text())
    audit = json.loads((base / 'protocol-audit.json').read_text())
    report = json.loads((frontend / 'report.json').read_text())
    assert report['status'] == 'PASS' and report['arguments']['modalities'] == ['rgb', 'thr']
    frozen = dict(audit['frozen_inputs'])
    frozen.update(report['frozen_inputs'])
    for p in [base / 'protocol-audit.json', frontend / 'report.json', Path(__file__)]:
        frozen[str(p)] = sha(p)
    for module in ['ms2_stereo_depth_proxy.py', 'ms2_depth_spatial_controls.py']:
        p = Path(__file__).with_name(module); frozen[str(p)] = sha(p)
    for m in ('rgb', 'thr'):
        frozen[report['modalities'][m]['checkpoint']] = report['modalities'][m]['checkpoint_sha256']
    pairs = {(p['modality'], p['indices'][0]): p for p in report['pairs'] if p['kind'] == 'stereo'}
    assert len(pairs) == 40
    for p in pairs.values():frozen[str(frontend / p['file'])] = p['sha256']
    assert all(sha(p) == h for p, h in frozen.items())
    dump(output / 'frozen-inputs.json', frozen)
    records = []
    for m in ('rgb', 'thr'):
        for i, row in enumerate(manifest['records']):
            c = row['cameras'][m]; f = np.asarray([c['K'][0][0], c['K'][1][1]])
            p = frontend / pairs[m, i]['file']
            with np.load(p, allow_pickle=False) as data:
                flow = data['flow'][0].astype(float).transpose(1, 2, 0)
                cov = data['covariance'][0].astype(float).transpose(1, 2, 0)
                assert data['source_ids'].tolist() == [row['stem'], row['stem']]
                assert data['input_time_ns'].tolist() == [c['left']['recorded_time_ns'], c['right']['recorded_time_ns']]
            assert np.isfinite(flow).all() and np.isfinite(cov).all() and (cov > 0).all()
            maps, masks = {}, {}
            for v in VARIANTS:
                raw = cv2.imread(str(root / row['references'][m][v]['path']), cv2.IMREAD_UNCHANGED)
                z = raw.astype(float) / 256
                d = np.zeros_like(z); nz = z > 0
                d[nz] = f[0] * c['baseline_m'] / z[nz]
                ur = np.arange(z.shape[1])[None, :] - d
                mask = nz & (ur >= 0) & (ur < z.shape[1] - 1)
                assert int(mask.sum()) == audit['cameras'][m]['depth_support'][v][i]['right_image_in_bounds']
                maps[v], masks[v] = (z, d), mask
            common = np.logical_and.reduce(list(masks.values()))
            for v in VARIANTS:
                z, d = maps[v]; keep = masks[v];y, x = np.nonzero(keep)
                target = np.column_stack((-d[keep], np.zeros(len(x))))
                pred, variance = flow[keep], cov[keep];residual = pred - target;shared = common[keep]
                name = f'{m}-{v}-{row["stem"]}.npz'
                np.savez_compressed(output / name, x=x.astype(np.int32), y=y.astype(np.int32), depth_m=z[keep],
                                    target_flow=target, flow=pred, covariance=variance, residual_pixel=residual,
                                    residual_normalized=residual / f, covariance_normalized=variance / f ** 2, common_support=shared)
                records.append({'modality': m, 'variant': v, 'stem': row['stem'], 'depth_nonzero': int((z > 0).sum()),
                                'out_of_bounds_excluded': int((z > 0).sum() - len(x)), 'source_file': str(p),
                                'file': name, 'sha256': sha(output / name), 'timing': audit['pairs'][i],
                                'own_support': metrics(residual, variance, f), 'common_support': metrics(residual[shared], variance[shared], f)})
            print(m, row['stem'], flush=True)
    assert len(records) == 120 and all(sha(p) == h for p, h in frozen.items())
    dump(output / 'protocol.json', {'inherited': 'EXP045/046 unchanged; sequence paths only differ', 'sequence': SEQUENCE,
                                   'primary': 'depth', 'sensitivities': ['depth_multi', 'depth_filtered'], 'new_model_inference_in_scoring': False})
    dump(output / 'report.json', {'status': 'PASS', 'sequence': SEQUENCE, 'source_manifest': str(base / 'source-manifest.json'),
                                  'physical_cross_modal_error_eligible': False, 'frozen_inputs_unchanged': True,
                                  'frozen_input_count': len(frozen), 'records': records, 'aggregate': aggregate_proxy(records)})


def controls(root, base, source, output):
    root, base, source, output = map(lambda p: Path(p).resolve(), (root, base, source, output))
    output.mkdir(parents=True, exist_ok=False)
    old = json.loads((source / 'report.json').read_text());manifest = json.loads((base / 'source-manifest.json').read_text())
    frozen = json.loads((source / 'frozen-inputs.json').read_text())
    for p in [source / 'report.json', Path(__file__)]:frozen[str(p)] = sha(p)
    for r in old['records']:frozen[str(source / r['file'])] = r['sha256']
    assert all(sha(p) == h for p, h in frozen.items())
    dump(output / 'frozen-inputs.json', frozen)
    primary = {(r['modality'], r['stem']): r for r in old['records'] if r['variant'] == 'depth'}
    records = []
    for r in old['records']:
        m, stem = r['modality'], r['stem'];h, w = manifest['records'][int(stem)]['cameras'][m]['left']['shape'][:2]
        with np.load(source / primary[m, stem]['file'], allow_pickle=False) as p:
            pc = p['common_support']; keys = p['y'][pc].astype(np.int64) * w + p['x'][pc];primary_z = p['depth_m'][pc]
        with np.load(source / r['file'], allow_pickle=False) as data:
            for support in ('own_support', 'common_support'):
                keep = np.ones(len(data['x']), dtype=bool) if support == 'own_support' else data['common_support']
                x, y = data['x'][keep], data['y'][keep];z = data['depth_m'][keep]
                if support == 'common_support':
                    assert np.array_equal(y.astype(np.int64) * w + x, keys);group_z = primary_z
                else:group_z = z
                u = data['covariance_normalized'][keep].sum(axis=1);e = np.linalg.norm(data['residual_normalized'][keep], axis=1)
                dg = np.searchsorted(DEPTH_EDGES, group_z, side='right') - 1
                sg = (y.astype(np.int64) * 3 // h) * 3 + x.astype(np.int64) * 3 // w
                cs = {name: control(u, e, g, n) for name, g, n in [('depth', dg, 6), ('spatial', sg, 9), ('joint', dg * 9 + sg, 54)]}
                records.append({'modality': m, 'variant': r['variant'], 'stem': stem, 'support': support, 'source_file': str(source / r['file']),
                                'count': len(u), 'timing': r['timing'], 'uncertainty_inverse_depth_rho': rho(u, 1 / group_z),
                                'error_inverse_depth_rho': rho(e, 1 / group_z), 'controls': cs})
    assert len(records) == 240 and all(sha(p) == h for p, h in frozen.items())
    dump(output / 'protocol.json', {'inherited': 'EXP047 bins/min30/coefficient unchanged', 'sequence': SEQUENCE, 'depth_edges': DEPTH_EDGES})
    dump(output / 'report.json', {'status': 'PASS', 'sequence': SEQUENCE, 'frozen_inputs_unchanged': True,
                                  'physical_cross_modal_error_eligible': False, 'new_model_inference': False,
                                  'records': records, 'aggregate': aggregate_controls(records)})


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('stage', choices=['prepare', 'score', 'controls'])
    p.add_argument('--root', type=Path, required=True);p.add_argument('--base', type=Path, required=True)
    p.add_argument('--source', type=Path);p.add_argument('--output', type=Path)
    a = p.parse_args()
    if a.stage == 'prepare':prepare(a.root, a.base)
    else:
        if a.source is None or a.output is None:p.error('source/output required')
        (score if a.stage == 'score' else controls)(a.root, a.base, a.source, a.output)
