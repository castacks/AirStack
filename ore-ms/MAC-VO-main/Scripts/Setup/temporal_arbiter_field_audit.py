"""Extract causally sourced health features; do not train a gate or score poses."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
from zoneinfo import ZoneInfo

import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def features(a, time0, time1, unsupported):
    if int(a['time0_ns']) != time0 or time1 <= time0:
        raise ValueError('pair chronology/identity mismatch')
    inbound = a['selected_inbound']
    keep = a['outlier_keep_inbound']
    assert inbound.dtype == keep.dtype == np.bool_
    assert len(keep) == int(inbound.sum())
    retained = a['selected_uv0'][inbound][keep]
    height, width = a['height_width']
    assert np.isfinite(retained).all()
    if len(retained):
        assert np.all((retained >= 0) & (retained < [width, height]))
        cells = np.floor(retained / [width, height] * 3).astype(int)
        occupied = len(np.unique(cells[:, 1]*3 + cells[:, 0]))
    else:
        occupied = 0
    covariance = a['grid_raw_flow_cov']
    finite = np.isfinite(covariance).all(axis=1)
    positive = finite & (covariance[:, 0] > 0) & (covariance[:, 1] > 0)
    trace = covariance[positive, :2].sum(axis=1)
    return {'time0_ns': time0, 'time1_ns': time1, 'duration_ns': time1-time0,
        'candidate_count': len(inbound), 'inbound_count': int(inbound.sum()),
        'retained_count': int(keep.sum()), 'occupied_3x3_cells': occupied,
        'grid_count': len(covariance), 'grid_covariance_finite_count': int(finite.sum()),
        'grid_positive_uu_vv_count': int(positive.sum()),
        'grid_trace_q10_q50_q90_px2': np.quantile(trace, [.1, .5, .9]).tolist() if len(trace) else None,
        'original_need_interp': bool(unsupported)}


def run(root, output):
    if output.exists():
        raise FileExistsError(output)
    output.mkdir(parents=True)
    hashes = {}

    def load(path):
        hashes[str(path.relative_to(root))] = sha(path)
        if path.suffix == '.json':
            return json.loads(path.read_text())
        with np.load(path, allow_pickle=False) as f:
            return {k: f[k] for k in f.files}

    base = root / 'results/gnss-and-tracking/reporting-001'
    cases, source_rows = {}, {}
    for name, modality in [('short-early-rgb', 'rgb'), ('long-middle-rgb', 'rgb'), ('long-middle-thermal', 'thermal')]:
        directory = base / name / modality
        report = load(directory / 'report.json')
        config = load(directory / 'effective-config.json')
        assert config['Odometry']['keyframe']['type'] == 'AllKeyframe'
        assert config['Odometry']['optimizer']['args']['parallel'] is False
        flags = load(directory / 'tracking-flags.npz')
        trajectory = load(directory / 'trajectory.npz')
        load(directory / 'uncertainty/metadata.json')
        times = [int(r[modality+'_left']['time_ns']) for r in report['input_rows']]
        source_rows[name] = report['input_rows']
        assert np.array_equal(trajectory['time_ns'].reshape(-1), times)
        files = sorted((directory / 'uncertainty').glob('pair-*.npz'))
        assert len(files) == len(times)-1
        assert len(flags['before_refinement']) == len(times)
        records = []
        for i, path in enumerate(files):
            assert path.name == f'pair-{i:03d}.npz'
            records.append(features(load(path), times[i], times[i+1], flags['before_refinement'][i+1]))
        assert sum(r['original_need_interp'] for r in records) == report['tracking_diagnostics']['unsupported_pairs_before_refinement']
        assert sum(r['retained_count'] for r in records) == report['tracking_diagnostics']['retained_observations']
        cases[name] = {'modality': modality, 'camera_frame': report['coordinate_frame'],
            'pairs': len(records), 'original_unsupported': sum(r['original_need_interp'] for r in records),
            'post_refinement_flags': int(flags['after_refinement'].sum()),
            'records': records, 'summary': {key: {'min': min(r[key] for r in records),
                'median': float(np.median([r[key] for r in records])), 'max': max(r[key] for r in records)}
                for key in ('candidate_count', 'inbound_count', 'retained_count', 'occupied_3x3_cells')},
            'not_available': ['online pre-refinement pose increment snapshot', 'pose covariance',
                'per-iteration optimizer loss/convergence trace', 'verified common-body transform', 'independent calibrated pose labels']}
    assert source_rows['long-middle-rgb'] == source_rows['long-middle-thermal']
    rgb = cases['long-middle-rgb']['records']
    thermal = cases['long-middle-thermal']['records']
    alignment = {'paired_manifest_rows': len(rgb), 'exact_same_intervals': sum(
        (a['time0_ns'], a['time1_ns']) == (b['time0_ns'], b['time1_ns']) for a, b in zip(rgb, thermal))}
    for end in ('time0_ns', 'time1_ns'):
        delta = [a[end]-b[end] for a, b in zip(rgb, thermal)]
        alignment[end+'_rgb_minus_thermal_ns'] = {'min': min(delta), 'median': float(np.median(delta)), 'max': max(delta)}
    source_paths = ['Scripts/Setup/run_td3_execution_v2.py', 'Scripts/Setup/td3_uncertainty.py',
                    'Scripts/Setup/td3_tracking_report.py', 'Odometry/MACVO.py', 'Module/MapProcessor.py',
                    'Module/Optimization/TwoFramePGO/Optimizer.py', 'Module/Optimization/Interface.py',
                    'Module/Optimization/TwoFramePGO/Graphs.py']
    source_hashes = {p: sha(root / 'MAC-VO-main' / p) for p in source_paths}
    protocol = {'created_at': datetime.now(ZoneInfo('America/New_York')).isoformat(),
        'script_sha256': sha(__file__), 'input_sha256': hashes, 'source_sha256': source_hashes,
        'field_definitions': {'candidate_count': 'selected keypoints before inbound/outlier filtering, not all dense candidates',
            'pose_availability': 'optimizer result is written into graph by write_map at following pair/termination; existing saved trajectory is post-refinement',
            'missing_diagnostics': 'missing optimizer residual/convergence/pose covariance remains unavailable, not zero'},
        'scope': '217 saved pair records, source/field audit only; no gate fitting, online VO reconstruction or accuracy evaluation',
        'causal_interpretation': 'raw pair arrays captured during pair execution; before-refinement flag assigned per pair in source. Whole-clip aggregates and post-refinement poses are not online features.'}
    for p, h in hashes.items():
        assert sha(root / p) == h
    dump(output / 'protocol.json', protocol)
    dump(output / 'report.json', {'cases': cases, 'alignment': alignment})
    print(json.dumps({'cases': {name: {k: v[k] for k in ('pairs', 'original_unsupported', 'post_refinement_flags', 'summary')} for name, v in cases.items()}, 'alignment': alignment}, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    run(args.root, args.output)
