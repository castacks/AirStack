"""Independent source, input-contract, saved-output and proxy audit for day pilot."""
import argparse
import ast
import hashlib
import json
import math
from pathlib import Path
import struct
import zlib
import cv2
import numpy as np
from DataLoader.Dataset.MS2 import MS2ManifestSequence, THERMAL_RECIPE, THERMAL_PRODUCER_SHA256
from Scripts.Setup.validate_ms2_stereo_depth_proxy import quantile, rank_correlation


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate(root, base, frontend=None, proxy=None):
    root, base = Path(root), Path(base)
    manifest = json.loads((base / 'source-manifest.json').read_text())
    audit = json.loads((base / 'protocol-audit.json').read_text())
    checks = 0
    points = 0
    scalar_points = 0

    def check(condition):
        nonlocal checks
        assert condition
        checks += 1

    def close(x, y):
        if x is None or y is None:check(x is y)
        else:check(math.isclose(x, y, rel_tol=1e-10, abs_tol=1e-12))

    check(manifest['sequence'] == '_2021-08-06-11-23-45')
    check([r['stem'] for r in manifest['records']] == [f'{i:06d}' for i in range(20)])
    for kind in ('sync_data', 'proj_depth'):
        for rec in json.loads((base / (kind + '-pilot-manifest.json')).read_text()):
            p = base / rec['path'];payload = p.read_bytes()
            check(len(payload) == rec['bytes'] and hashlib.sha256(payload).hexdigest() == rec['sha256'])
            if p.suffix == '.png':
                check(payload[:8] == b'\x89PNG\r\n\x1a\n')
                off = 8
                while off < len(payload):
                    n = struct.unpack('>I', payload[off:off + 4])[0];tag = payload[off + 4:off + 8]
                    block = payload[off + 8:off + 8 + n]
                    crc = struct.unpack('>I', payload[off + 8 + n:off + 12 + n])[0]
                    check(zlib.crc32(tag + block) & 0xffffffff == crc);off += 12 + n
                check(off == len(payload))
    inputs = {}
    producer = root / 'results/ms2-mac-adapter/inputs/get_thermal_data.py'
    check(sha(producer) == THERMAL_PRODUCER_SHA256)
    functions = [n for n in ast.parse(producer.read_text()).body if isinstance(n, ast.FunctionDef) and n.name in ('process_image', 'enhance_image')]
    check(len(functions) == 2)
    scope = {'np': np, 'cv2': cv2};exec(compile(ast.Module(body=functions, type_ignores=[]), str(producer), 'exec'), scope)
    for m in ('rgb', 'thr'):
        sequence = MS2ManifestSequence(root, base / 'source-manifest.json', m, thermal_transform=THERMAL_RECIPE if m == 'thr' else None)
        for i, row in enumerate(manifest['records']):
            frame = sequence[i];c = row['cameras'][m]
            check(frame.gt_pose is None and frame.stereo.gt_depth is None)
            check(frame.time_ns == [c['left']['recorded_time_ns']] and frame.stereo.right_time_ns == [c['right']['recorded_time_ns']])
            for side, tensor in [('left', frame.stereo.imageL), ('right', frame.stereo.imageR)]:
                rec = c[side];p = root / rec['path'];check(sha(p) == rec['sha256'])
                raw = cv2.imread(str(p), cv2.IMREAD_UNCHANGED)
                check(str(raw.dtype) == rec['dtype'] and list(raw.shape) == rec['shape'])
                if m == 'thr':
                    image = scope['enhance_image'](scope['process_image'](raw.copy(), 'firestereo_v2'))
                    image = np.repeat(image[..., None], 3, axis=2)
                else:image = raw[..., ::-1].copy()
                expected = (image.astype(np.float32) / np.float32(255)).transpose(2, 0, 1)[None]
                check(np.array_equal(tensor.numpy(), expected))
                inputs[m, i, side] = hashlib.sha256(expected.tobytes()).hexdigest()
        for side in ('left', 'right'):
            p = base / 'pilot/sync_data' / manifest['sequence'] / m / ('img_' + side + '_timestamp.txt')
            a = [int(x) for x in p.read_text().split()]
            check(len(a) == 5810 and all(y > x for x, y in zip(a, a[1:])))
            for i, row in enumerate(manifest['records']):check(a[i] == row['cameras'][m][side]['recorded_time_ns'])
    frontend_count = 0
    source_stereo = {}
    if frontend:
        frontend = Path(frontend);report = json.loads((frontend / 'report.json').read_text());check(report['status'] == 'PASS')
        expected = {(m, kind, i, j) for m in ('rgb', 'thr') for kind, i, j in
                    ([('stereo', i, i) for i in range(20)] + [('temporal', i, i + 1) for i in range(19)])}
        seen = set()
        for p in report['pairs']:
            m, kind = p['modality'], p['kind'];i, j = p['indices'];side = 'right' if kind == 'stereo' else 'left'
            key = (m, kind, i, j);check(key not in seen);seen.add(key)
            if kind == 'stereo':source_stereo[m, i] = p
            c, d = manifest['records'][i]['cameras'][m], manifest['records'][j]['cameras'][m]
            check(p['input_sha256'] == [inputs[m, i, 'left'], inputs[m, j, side]])
            check(sha(frontend / p['file']) == p['sha256'])
            with np.load(frontend / p['file'], allow_pickle=False) as data:
                for field in ('flow', 'covariance'):
                    a = data[field];check(a.shape == (1, 2, *c['left']['shape'][:2]) and a.dtype == np.float32 and np.isfinite(a).all())
                    close(float(a.min()), p[field + '_min']);close(float(a.max()), p[field + '_max'])
                check((data['covariance'] > 0).all())
                check(data['input_time_ns'].tolist() == [c['left']['recorded_time_ns'], d[side]['recorded_time_ns']])
                check(data['camera_times_ns'].tolist() == [c[s]['recorded_time_ns'] for s in ('left', 'right')] + [d[s]['recorded_time_ns'] for s in ('left', 'right')])
                check(data['source_ids'].tolist() == [f'{i:06d}', f'{j:06d}'])
            if 'repeat' in p:check(p['repeat']['flow_bitwise'] and p['repeat']['covariance_bitwise'])
        check(seen == expected);frontend_count = len(seen)
        for c in report['modalities'].values():check(sha(c['checkpoint']) == c['checkpoint_sha256'])
    proxy_count = 0
    if proxy:
        proxy = Path(proxy);report = json.loads((proxy / 'report.json').read_text());check(report['status'] == 'PASS')
        seen = set()
        for rec in report['records']:
            m, v, stem = rec['modality'], rec['variant'], rec['stem'];i = int(stem)
            key = m, v, stem;check(key not in seen);seen.add(key)
            check(Path(rec['source_file']).resolve() == (frontend / source_stereo[m, i]['file']).resolve())
            check(sha(rec['source_file']) == source_stereo[m, i]['sha256'])
            row = manifest['records'][i];c = row['cameras'][m];f = np.array([c['K'][0][0], c['K'][1][1]]);fB = f[0] * c['baseline_m']
            masks, raws = {}, {}
            for variant in ('depth', 'depth_multi', 'depth_filtered'):
                raw = cv2.imread(str(root / row['references'][m][variant]['path']), cv2.IMREAD_UNCHANGED)
                check(raw.dtype == np.uint16 and list(raw.shape) == c['left']['shape'][:2])
                z = raw.astype(float) / 256;u = np.arange(z.shape[1])[None, :]
                masks[variant] = (z > 0) & (u * z >= fB) & ((u - (z.shape[1] - 1)) * z < fB);raws[variant] = raw
            y, x = np.where(masks[v]);common = np.logical_and.reduce(list(masks.values()))[masks[v]]
            with np.load(rec['source_file'], allow_pickle=False) as orig, np.load(proxy / rec['file'], allow_pickle=False) as data:
                check(sha(proxy / rec['file']) == rec['sha256'])
                check(np.array_equal(data['x'], x) and np.array_equal(data['y'], y) and np.array_equal(data['common_support'], common))
                z = raws[v][y, x].astype(float) / 256
                target = np.column_stack((-256 * fB / raws[v][y, x], np.zeros(len(x))))
                pred = orig['flow'][0, :, y, x].astype(float);cov = orig['covariance'][0, :, y, x].astype(float);e = pred - target
                for field, expected in [('depth_m', z), ('flow', pred), ('covariance', cov), ('target_flow', target),
                                         ('residual_pixel', e), ('residual_normalized', e / f), ('covariance_normalized', cov / f ** 2)]:
                    check(np.allclose(data[field], expected, rtol=1e-13, atol=1e-13))
                points += len(x)
                for k in np.linspace(0, len(x) - 1, min(16, len(x)), dtype=int):
                    close(float(data['residual_pixel'][k, 0]), float(orig['flow'][0, 0, y[k], x[k]]) + float(f[0]) * c['baseline_m'] / (int(raws[v][y[k], x[k]]) / 256))
                    scalar_points += 1
                for support, keep in [('own_support', np.ones(len(x), dtype=bool)), ('common_support', common)]:
                    summary = rec[support];check(summary['count'] == int(keep.sum()))
                    if not keep.any():check(summary['availability'] == 'empty-support');continue
                    for units, scale in [('pixel', np.ones(2)), ('normalized_plane', f)]:
                        ee = e[keep] / scale;cc = cov[keep] / scale ** 2
                        norm = [math.hypot(float(a), float(b)) for a, b in ee];trace = [float(a) + float(b) for a, b in cc];s = summary[units]
                        for field, values, q in [('l2_median', norm, .5), ('l2_p90', norm, .9), ('signed_u_median', ee[:, 0], .5),
                                                 ('signed_v_median', ee[:, 1], .5), ('abs_u_median', abs(ee[:, 0]), .5), ('abs_v_median', abs(ee[:, 1]), .5),
                                                 ('abs_u_p90', abs(ee[:, 0]), .9), ('abs_v_p90', abs(ee[:, 1]), .9), ('covariance_trace_median', trace, .5)]:
                            close(s[field], quantile(values, q))
                        close(s['uncertainty_error_spearman'], rank_correlation(trace, norm))
            check(rec['timing'] == audit['pairs'][i])
            check(rec['depth_nonzero'] == int(np.count_nonzero(raws[v])) and rec['out_of_bounds_excluded'] == int(np.count_nonzero(raws[v])) - len(x))
        check(seen == {(m, v, f'{i:06d}') for m in ('rgb', 'thr') for v in ('depth', 'depth_multi', 'depth_filtered') for i in range(20)})
        proxy_count = len(seen)
        for m, variants in report['aggregate'].items():
            for v, supports in variants.items():
                for support, summary in supports.items():
                    group = [r[support] for r in report['records'] if r['modality'] == m and r['variant'] == v]
                    check(summary['support_count_sum_not_independent_n'] == sum(s['count'] for s in group))
                    for units in ('pixel', 'normalized_plane'):
                        for key, stats in summary[units].items():
                            a = [s[units][key] for s in group if s.get(units, {}).get(key) is not None]
                            close(stats['available_images'], len(a));close(stats['median'], quantile(a, .5) if a else None)
                            close(stats['min'], min(a) if a else None);close(stats['max'], max(a) if a else None)
        for p, h in json.loads((proxy / 'frozen-inputs.json').read_text()).items():check(sha(p) == h)
    for p, h in audit['frozen_inputs'].items():check(sha(p) == h)
    historical = 0
    for name in ('exp-043', 'exp-044-complete', 'exp-046', 'exp-047'):
        old = json.loads((Path('/root/ore-ms-recovery') / name / 'manifest.json').read_text())
        for p, h in old['member_sha256'].items():
            if p.startswith('ore-ms/results/'):check(sha(root.parent / p) == h);historical += 1
    return {'status': 'PASS', 'independent_controls': checks, 'model_input_tensors_exact': len(inputs),
            'frontend_pairs_checked': frontend_count, 'proxy_cases_checked': proxy_count,
            'full_array_points_checked': points, 'scalar_points_checked': scalar_points, 'historical_hash_checks': historical}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True);p.add_argument('--base', type=Path, required=True)
    p.add_argument('--frontend', type=Path);p.add_argument('--proxy', type=Path);p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():p.error('Refuse existing validation')
    if a.proxy and not a.frontend:p.error('Frontend required for source-association validation')
    result = validate(a.root.resolve(), a.base.resolve(), a.frontend, a.proxy)
    a.output.write_text(json.dumps(result, indent=2) + '\n');print(json.dumps(result))
