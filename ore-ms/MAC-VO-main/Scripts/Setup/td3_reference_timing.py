"""Fixed-window reference/timestamp diagnostics, without fitted time corrections."""

import argparse
import csv
from datetime import datetime, timezone
from decimal import Decimal
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from .td3_night_geometry import verify
from .td3_reference import PoseReference, camera_reference, matrices, read_ns, load_reference
from .td3_reprojection import pair_geometry
from .td3_tracking_uncertainty import sha


OFFSETS_NS = np.array([-2000, -1000, -500, -250, -100, 0, 100, 250, 500, 1000, 2000], dtype=np.int64) * 1_000_000


def source_timestamps(path):
    """Preserve index holes and nanosecond precision; nonpositive stamps are holes."""
    result = []
    for text in Path(path).read_text().splitlines():
        value = Decimal(text)
        result.append(int((value * 1_000_000_000).to_integral_value())
                      if value.is_finite() and value > 0 else None)
    return result


def distribution(values):
    values = np.asarray(values, dtype=float)
    finite = values[np.isfinite(values)]
    return {"count": int(values.size), "finite_count": int(finite.size),
            "p10_p50_p90": np.percentile(finite, [10, 50, 90]).tolist() if finite.size else None,
            "max": float(finite.max()) if finite.size else None}


def pose_difference(first, second):
    error = np.linalg.inv(first) @ second
    return np.linalg.norm(error[:, :3, 3], axis=1), np.degrees(Rotation.from_matrix(error[:, :3, :3]).magnitude())


def reference_at_offset(reference, times, offset_ns):
    times = np.asarray(times, dtype=np.int64)
    if not isinstance(offset_ns, (int, np.integer)):
        raise ValueError("Offset must be integer nanoseconds")
    shifted = times + offset_ns
    # PoseReference rejects extrapolation; never silently clip a shifted window.
    return reference.at(shifted)


def compare_export(reference, times, data):
    """Compare the binding-grid export with the source-rate series, preserving failures."""
    data = np.asarray(data, float)
    if data.ndim != 2 or len(times) != len(data) or data.shape[1] < 7:
        raise ValueError("Unaligned pose/time rows")
    valid = np.array([t is not None for t in times]) & np.isfinite(data[:, :7]).all(axis=1)
    qnorm = np.linalg.norm(data[:, 3:7], axis=1)
    valid &= np.isfinite(qnorm) & (np.abs(qnorm - 1) <= 1e-3)
    support = np.array([t is not None and reference.time_ns[0] <= t <= reference.time_ns[-1] for t in times])
    keep = valid & support
    selected = np.array([t for t, ok in zip(times, keep) if ok], dtype=np.int64)
    result = {"rows": len(times), "invalid_rows": int((~valid).sum()),
              "outside_source_support": int((valid & ~support).sum()), "compared_rows": int(keep.sum())}
    if len(selected):
        translation, rotation = pose_difference(reference.at(selected), matrices(data[keep]))
        result.update(translation_m=distribution(translation), rotation_deg=distribution(rotation))
    return result


def endpoint_errors(points, relative):
    """Compare the frozen LK endpoints to pose/depth-conditioned image projections."""
    geometry = pair_geometry(points['uv0'], points['lk_endpoint'], points['depth0'],
                             points['depth0'], points['K'], relative)
    forward = np.linalg.norm(geometry['forward_error_px'], axis=1)
    forward[~(np.isfinite(points['depth0']) & (points['depth0'] > 0))] = np.nan
    sampson = geometry['sampson_px'].copy()
    if np.linalg.norm(relative[:3, 3]) <= 1e-6:
        sampson[:] = np.nan
    return forward, sampson


def prepare(root, output):
    source = root / 'results/night-low-motion/run-001/plan.json'
    old = json.loads(source.read_text())
    msa = json.loads((root / 'results/msa-replay/prepared-001/plan.json').read_text())
    long = json.loads((root / 'results/long-salon/prepared-001/plan.json').read_text())
    files = dict(old['files_sha256'])
    paths = [source, root / 'results/msa-replay/prepared-001/plan.json',
             root / 'results/long-salon/prepared-001/plan.json', root / 'data/calibration/yamaha.yaml']
    cases = []
    for case in old['cases']:
        original = (next(c for c in long['cases'] if c['case'] == 'early') if case['case'] == 'day-static'
                    else next(c for c in msa['cases'] if c['case'] == case['case']))
        reference_dataset = Path(original['reference_dataset'])
        point_paths = sorted((root / 'results/night-low-motion/run-001/points' / case['case']).glob('*.npz'))
        if len(point_paths) != 19:
            raise ValueError('Expected all 19 fixed point arrays')
        cases.append({**case, 'reference_dataset': str(reference_dataset), 'point_paths': list(map(str, point_paths))})
        paths += point_paths
        paths += [reference_dataset / 'super_odometry/odometry' / name
                  for name in ['info.yaml', 'interp_data.txt', 'interp_timestamps.txt']]
        paths.append(Path(case['dataset']) / 'sensors/thermal_left_processed_camera_info/data.yaml')
    base = root / 'results/reference-timing'
    paths += [base / 'acquisition-plan.json', base / 'metadata-extraction.json']
    paths += [p for p in (base / 'inputs').rglob('*') if p.is_file()]
    paths += [p for p in (base / 'staging/upstream').rglob('*') if p.is_file()]
    paths += [Path(__file__), Path(__file__).parents[1] / 'UnitTest/test_td3_reference_timing.py']
    paths += [Path(__file__).with_name(name) for name in
              ['td3_reference.py', 'td3_reprojection.py', 'td3_feature_consistency.py',
               'td3_night_geometry.py', 'td3_tracking_uncertainty.py', 'audit_td3_geometry.py']]
    files.update({str(p): sha(p) for p in paths})
    verify(files)
    output.mkdir(parents=True, exist_ok=False)
    plan = {'created_at_utc': datetime.now(timezone.utc).isoformat(), 'base_commit': '46784eb6e',
            'cases': cases, 'files_sha256': files, 'offsets_ns': OFFSETS_NS.tolist(),
            'night_metadata': str(base / 'inputs/night'), 'calibration': str(root / 'data/calibration/yamaha.yaml'),
            'protocol': {'population': 'unchanged EXP-027 LK-valid grid; retain projection-invalid counts',
                         'time_shift': 'reference is queried at image time plus offset; never shift images or fit an offset',
                         'summary': 'unweighted median of 19 pair medians; preserve all offsets',
                         'scope': 'reference extraction consistency/sensitivity and auxiliary context; no independent GT or corrected calibration'}}
    (output / 'plan.json').write_text(json.dumps(plan, indent=2) + '\n')


def auxiliary_context(metadata, times):
    """Describe source-rate streams in their supplied frames, with no cross-frame alignment."""
    result = {}
    streams = ['sensors/novatel_gps_odom', 'sensors/novatel_imu', 'controls/command',
               'controls/teleop_command', 'controls/auto_command']
    for name in streams:
        directory = metadata / name
        info = yaml.safe_load((directory / 'info.yaml').read_text())
        stamps = source_timestamps(directory / 'interp_timestamps.txt')
        data = np.atleast_2d(np.loadtxt(directory / 'interp_data.txt'))
        if len(stamps) != len(data):
            raise ValueError('Auxiliary source time/data length mismatch')
        mask = np.array([t is not None and times[0] <= t <= times[-1] for t in stamps])
        values = data[mask]
        item = {'frames': info, 'source_rows': len(stamps), 'window_rows': int(mask.sum()),
                'columns': data.shape[1], 'finite_rows': int(np.isfinite(values).all(axis=1).sum()),
                'column_distributions': [distribution(values[:, j]) for j in range(data.shape[1])],
                'scope': 'unaligned supplied source stream; neither independent GT nor certified measured vehicle motion'}
        if name.endswith('gps_odom') and data.shape[1] == 13:
            # All source rows must be valid: do not silently repair an auxiliary trajectory.
            good = all(t is not None for t in stamps) and np.isfinite(data[:, :7]).all()
            if good:
                reference = PoseReference(stamps, data)
                if times[0] >= reference.time_ns[0] and times[-1] <= reference.time_ns[-1]:
                    endpoints = reference.at(times[[0, -1]])
                    item['own_frame_net_displacement_m'] = float(np.linalg.norm(endpoints[1, :3, 3] - endpoints[0, :3, 3]))
            item['linear_speed_mps'] = distribution(np.linalg.norm(values[:, 7:10], axis=1))
            item['angular_speed_radps'] = distribution(np.linalg.norm(values[:, 10:13], axis=1))
        elif name.endswith('novatel_imu') and data.shape[1] == 10:
            item['angular_speed_radps'] = distribution(np.linalg.norm(values[:, 4:7], axis=1))
            item['acceleration_norm_mps2'] = distribution(np.linalg.norm(values[:, 7:10], axis=1))
            item['scope'] += '; gravity not removed; no velocity integration or standstill claim'
        elif name.startswith('controls/'):
            item['scope'] += '; command requests are not measured motion'
        result[name] = item
    return result


def run(output):
    plan = json.loads((output / 'plan.json').read_text())
    verify(plan['files_sha256'])
    metadata = Path(plan['night_metadata'])
    source_ref, _ = load_reference(metadata)
    exported = metadata / 'super_odometry/odometry'
    exports = {'binding_grid': compare_export(source_ref, source_timestamps(exported / 'timestamps.txt'), np.loadtxt(exported / 'data.txt')),
               'recorded_tf': compare_export(source_ref, source_timestamps(metadata / 'tf/vehicle_to_sensor_init/timestamps.txt'),
                                             np.loadtxt(metadata / 'tf/vehicle_to_sensor_init/transforms.txt')),
               'source_rows': len(source_ref.time_ns), 'source_gap_seconds': distribution(np.diff(source_ref.time_ns) / 1e9)}
    cases, summaries, pairs = [], [], []
    for case in plan['cases']:
        name = case['case']
        times = np.array([r['thermal_left']['time_ns'] for r in case['rows']], dtype=np.int64)
        reference, _ = load_reference(case['reference_dataset'])
        with np.load(case['proxy']) as saved:
            if not np.array_equal(times, saved['time_ns'].reshape(-1)[:20]):
                raise ValueError('Proxy/image timestamp mismatch')
            original = saved[case['proxy_key']][:20]
        with np.load(Path(case['directory']) / 'trajectory.npz') as trajectory:
            if not np.array_equal(times, trajectory['time_ns'].reshape(-1)[:20]):
                raise ValueError('VO/image timestamp mismatch')
        zero = camera_reference(reference, times, case['dataset'], plan['calibration'], 'thermal')
        delta_t, delta_r = pose_difference(original, zero)
        if delta_t.max() > 1e-8 or delta_r.max() > 1e-5:
            raise ValueError('Zero-offset proxy reconstruction failed')
        record = {'case': name, 'time_ns': times.tolist(), 'duration_seconds': float((times[-1] - times[0]) / 1e9),
                  'acquisition_gap_seconds': distribution(np.diff(times) / 1e9),
                  'zero_proxy_translation_max_m': float(delta_t.max()), 'zero_proxy_rotation_max_deg': float(delta_r.max())}
        if name.startswith('night-'):
            record['source_metadata'] = auxiliary_context(metadata, times)
            joins = []
            for side in ['left', 'right']:
                raw = source_timestamps(metadata / f'sensors/thermal_{side}/timestamps.txt')
                processed = source_timestamps(metadata / f'sensors/thermal_{side}_processed/timestamps.txt')
                for row in case['rows']:
                    image = row['thermal_' + side];index = image['index'];t = image['time_ns']
                    if processed[index] != t:
                        raise ValueError('Processed image acquisition timestamp/index mismatch')
                    matches = [i for i, stamp in enumerate(raw) if stamp == t]
                    if not matches:
                        raise ValueError('No exact raw acquisition timestamp match')
                    joins.append({'side': side, 'source_index': index, 'time_ns': t, 'raw_exact_indices': matches})
            record['raw_processed_timestamp_joins'] = joins
            body = reference.at(times)
            record['slam_body_net_displacement_m'] = float(np.linalg.norm(body[-1, :3, 3] - body[0, :3, 3]))
            ti = reference.time_ns
            mask = (ti >= times[0]) & (ti <= times[-1])
            state = np.loadtxt(Path(case['reference_dataset']) / 'super_odometry/odometry/interp_data.txt')
            record['slam_source_linear_speed_mps'] = distribution(np.linalg.norm(state[mask, 7:10], axis=1))
            if not np.array_equal(reference.time_ns, source_ref.time_ns) or not np.array_equal(reference.poses, source_ref.poses):
                raise ValueError('Restored and freshly CRC-checked SLAM sources differ')
        points = []
        for i, path in enumerate(case['point_paths']):
            with np.load(path) as source:
                point = {key: source[key] for key in source.files}
            if int(point['time0_ns']) != int(times[i]) or int(point['time1_ns']) != int(times[i+1]):
                raise ValueError('Frozen LK pair/image timestamp mismatch')
            points.append(point)
        for offset in plan['offsets_ns']:
            summary = {'case': name, 'offset_ns': offset, 'offset_seconds': offset / 1e9}
            try:
                reference_at_offset(reference, times, offset)
            except ValueError:
                summaries.append({**summary, 'status': 'OUTSIDE_REFERENCE_SUPPORT'})
                continue
            camera = camera_reference(reference, times + offset, case['dataset'], plan['calibration'], 'thermal')
            relative = np.linalg.inv(camera[1:]) @ camera[:-1]
            local = []
            for i, point in enumerate(points):
                forward, sampson = endpoint_errors(point, relative[i])
                mask = point['mask_lk_valid']
                f, s = distribution(forward[mask]), distribution(sampson[mask])
                row = {**summary, 'pair': i, 'status': 'SUPPORTED', 'lk_valid_count': int(mask.sum()),
                       'forward_finite_count': f['finite_count'], 'sampson_finite_count': s['finite_count'],
                       'forward_pair_median_px': f['p10_p50_p90'][1] if f['finite_count'] else None,
                       'sampson_pair_median_px': s['p10_p50_p90'][1] if s['finite_count'] else None,
                       'reference_translation_m': float(np.linalg.norm(relative[i, :3, 3])),
                       'reference_rotation_deg': float(np.degrees(Rotation.from_matrix(relative[i, :3, :3]).magnitude()))}
                local.append(row);pairs.append(row)
                if offset == 0:
                    np.testing.assert_allclose(relative[i], point['relative_proxy'], rtol=1e-12, atol=1e-12)
                    np.testing.assert_allclose(forward[mask], point['lk_proxy_error_px'][mask], rtol=3e-7, atol=1e-6)
            summary.update(status='SUPPORTED', pair_count=len(local),
                           reference_net_displacement_m=float(np.linalg.norm(camera[-1, :3, 3] - camera[0, :3, 3])))
            for key in ['forward_pair_median_px', 'sampson_pair_median_px', 'reference_translation_m', 'reference_rotation_deg']:
                values = [r[key] for r in local if r[key] is not None]
                summary[key] = float(np.median(values)) if values else None
            summaries.append(summary)
        cases.append(record)
        print(name, 'all fixed offsets retained', flush=True)
    result = {'created_at_utc': datetime.now(timezone.utc).isoformat(), 'base_commit': plan['base_commit'],
              'files_verified': verify(plan['files_sha256']), 'exports': exports, 'cases': cases,
              'offsets': summaries, 'scope': plan['protocol']['scope']}
    (output / 'analysis.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    for name, rows in [('offsets', summaries), ('pairs', pairs)]:
        with (output / (name + '.csv')).open('w') as f:
            keys = list(dict.fromkeys(k for row in rows for k in row))
            writer = csv.DictWriter(f, fieldnames=keys);writer.writeheader();writer.writerows(rows)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['prepare', 'run'])
    parser.add_argument('--root', type=Path, default=Path('/workspace'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.action == 'prepare':
        prepare(args.root, args.output)
    else:
        run(args.output)
