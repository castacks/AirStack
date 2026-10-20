"""Frozen separate-recording validation of the EXP-012 u-only replay rule."""

import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from Scripts.Setup.td3_channel_mass import apply_weights
from Scripts.Setup.td3_precision_mass import sha, within_tolerance, TOLERANCES
from Scripts.Setup.td3_backend_ablation import pair_input, solve, compose, local_error
from Scripts.Setup.td3_reference import load_reference, camera_reference, matrices, trajectory_metrics

ARMS = ('learned', 'fixed_u_mass', 'fixed_uv_mass')
MODALITIES = ('rgb', 'thermal')
WINDOWS = ('early', 'middle', 'late')


def select_windows(reference, rows):
    """Maximum body displacement in each third; both camera gaps and coverage."""
    count = len(rows)
    if count < 60:
        raise ValueError('At least 60 paired groups are required for three disjoint windows')
    times = {m: np.array([r[m+'_left']['time_ns'] for r in rows], dtype=np.int64) for m in MODALITIES}
    valid = np.ones(count, dtype=bool)
    for ts in times.values():
        valid &= (ts >= reference.time_ns[0]) & (ts <= reference.time_ns[-1])
    positions = np.full((count, 3), np.nan)
    if valid.any():
        positions[valid] = reference.at(times['rgb'][valid])[:, :3, 3]
    bounds = [0, count//3, 2*count//3, count]
    selected = []
    for name, lo, hi in zip(WINDOWS, bounds[:-1], bounds[1:]):
        candidates = []
        for start in range(lo, hi-19):
            if not valid[start:start+20].all():
                continue
            gaps = [np.diff(ts[start:start+20]) for ts in times.values()]
            if any(np.any(g <= 0) or np.any(g > 150_000_000) for g in gaps):
                continue
            distance = float(np.linalg.norm(positions[start+19]-positions[start]))
            candidates.append((distance, start))
        if not candidates:
            raise ValueError('No eligible frozen-protocol window in '+name)
        distance, start = max(candidates, key=lambda q: (q[0], -q[1]))
        selected.append({'name': name, 'manifest_start': start, 'frames': 20,
                         'region': [lo, hi], 'body_reference_displacement_m': distance})
    return selected


def verify(plan):
    for name, digest in plan['files_sha256'].items():
        if sha(Path(name)) != digest:
            raise ValueError('Frozen input/source changed: '+name)


def prepare(args):
    manifest = json.loads(args.manifest.read_text())
    if manifest['policy']['stereo_max_ns'] != 2_000_000 or manifest['policy']['cross_max_ns'] != 30_000_000:
        raise ValueError('Pairing policy differs from preregistration')
    audit = json.loads((args.dataset/'recording_audit.json').read_text())
    source = audit['source']
    if Path(source).name == '1_salon_mac_cov_day' or Path(source).name != args.dataset.name:
        raise ValueError('Recording identity is evaluated or inconsistent')
    old_rows = json.loads(args.evaluated_manifest.read_text())['pairs']
    old_times = {int(row[k]['time_ns']) for row in old_rows
                 for k in ['rgb_left', 'rgb_right', 'thermal_left', 'thermal_right']}
    if any(int(row[k]['time_ns']) in old_times for row in manifest['pairs']
           for k in ['rgb_left', 'rgb_right', 'thermal_left', 'thermal_right']):
        raise ValueError('Acquisitions overlap the evaluated recording')
    protocol = json.loads(args.protocol.read_text())
    for name, expected in protocol['source_sha256'].items():
        if name.endswith('.py') and sha(ROOT.parent/name) != expected:
            raise ValueError('Predeclared implementation changed: '+name)
    preflight = json.loads(args.preflight.read_text())
    if preflight['status'] != 'PASS_WITH_PROVISIONAL_GEOMETRY':
        raise ValueError('Input/calibration/encoding preflight failed')
    reference, _ = load_reference(args.dataset)
    windows = select_windows(reference, manifest['pairs'])
    files = [args.manifest, args.evaluated_manifest, args.protocol, args.preflight, args.calibration,
             args.dataset/'recording_audit.json', *list(args.dataset.glob('tf/**/*.txt')),
             args.dataset/'tf/metadata.yaml',
             *[args.dataset/'super_odometry/odometry'/n for n in
               ['info.yaml', 'interp_data.txt', 'interp_timestamps.txt']],
             *list(args.dataset.glob('sensors/*/data.yaml')),
             *list(args.dataset.glob('sensors/*/timestamps.txt')),
             ROOT.parent/'models/macslam/MACT_Offroad.pth', ROOT.parent/'models/macslam/MACVO_FrontendCov.pth',
             ROOT/'Config/Experiment/MACVO/MACVO_Performant.yaml']
    files += list(ROOT.rglob('*.py'))
    images = {args.dataset/row[k]['path'] for w in windows
              for row in manifest['pairs'][w['manifest_start']:w['manifest_start']+20]
              for k in ['rgb_left', 'rgb_right', 'thermal_left', 'thermal_right']}
    if len(images) != 240:
        raise ValueError('Expected 240 distinct selected camera images')
    files += list(images)
    plan = {'created_at_utc': datetime.now(timezone.utc).isoformat(), 'recording_source': source,
            'dataset': str(args.dataset.resolve()), 'manifest': str(args.manifest.resolve()),
            'calibration': str(args.calibration.resolve()), 'windows': windows,
            'arms': ARMS, 'threads': 1, 'seed': 0, 'tolerances': TOLERANCES,
            'rule': 'Unchanged EXP-012 learned/u-only/both-channel replay, channelwise harmonic means preserving inverse-variance sums before Huber',
            'scope': 'Separate recording; provisional geometry, SLAM proxy, unknown checkpoint training split; frozen independent priors, not new end-to-end weighted VO',
            'files_sha256': {str(p.resolve()): sha(p) for p in sorted(set(files))}}
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'plan.json').write_text(json.dumps(plan, indent=2)+'\n')
    print('Frozen', len(plan['files_sha256']), 'files and windows:', json.dumps(windows), flush=True)


def run_case(directory, output, reference, dataset, calibration, expected_rows, expected_start, checkpoint_digest):
    report = json.loads((directory/'report.json').read_text()) if (directory/'report.json').exists() else {}
    if report.get('status') != 'PASS_EXECUTION_ONLY':
        return {'status': 'BASELINE_FAILED', 'baseline_report': report}
    if (report['input_rows'] != expected_rows or report['manifest_start'] != expected_start
            or report['checkpoint_sha256'] != checkpoint_digest or report['seed'] != 0
            or report['completed_frames'] != 20):
        raise ValueError('Baseline differs from frozen selection/model/settings')
    if report['tracking']['interpolated_frames'] != 0:
        return {'status': 'BASELINE_INTERPOLATED', 'baseline_report': report}
    output.mkdir(parents=True, exist_ok=False)
    with np.load(directory/'tensor-map.npz', allow_pickle=False) as saved, \
            np.load(directory/'trajectory.npz', allow_pickle=False) as original:
        times = original['time_ns'].reshape(-1)
        expected_times = np.array([r[directory.name+'_left']['time_ns'] for r in expected_rows], dtype=np.int64)
        if (not np.array_equal(times, saved['frames//time_ns'].reshape(-1))
                or not np.array_equal(times, expected_times)
                or not np.array_equal(original['poses_camera_ned'], saved['frames//pose'])
                or saved['frames//need_interp'].any()):
            raise ValueError('Baseline pose/time association failed')
        config = json.loads((directory/'effective-config.json').read_text())
        if config['Odometry']['optimizer']['args'] != {
                'device': 'cpu', 'vectorize': True, 'parallel': False, 'graph_type': 'disp', 'autodiff': False}:
            raise ValueError('Unexpected optimizer')
        old = matrices(saved['frames//pose'])
        old_delta = np.linalg.inv(old[:-1]) @ old[1:]
        camera = camera_reference(reference, times, dataset, calibration, directory.name)
        ref_delta = np.linalg.inv(camera[:-1]) @ camera[1:]
        results, signatures = {}, []
        for arm in ARMS:
            motions, pairs, hashes = [], [], []
            for i in range(len(times)-1):
                data = pair_input(saved, i)
                # This helper's legacy constants are validation-only for these arms.
                # They never set weights here: harmonic means come solely from this pair.
                weight = apply_weights(data, arm, [1., 1., 1.])
                pose, metrics = solve(data, 'disp')
                metrics['weight_protocol'] = weight
                motions.append(np.linalg.inv(old[i]) @ matrices(pose[None])[0])
                pairs.append(metrics); hashes.append(weight['nonpixel_sha256'])
            signatures.append(hashes)
            increments = np.stack(motions); trajectory = compose(increments)
            t, r = local_error(increments, ref_delta)
            dt, dr = local_error(increments, old_delta)
            results[arm] = {'trajectory': trajectory_metrics(trajectory, camera),
                'local_translation_rmse_m': float(np.sqrt(np.mean(t*t))),
                'local_rotation_rmse_deg': float(np.sqrt(np.mean(r*r))), 'pairs': pairs,
                'max_local_translation_difference_from_original_m': float(dt.max()),
                'max_local_rotation_difference_from_original_deg': float(dr.max()),
                'matches_baseline_tolerance': within_tolerance(dt, dr)}
            np.savez_compressed(output/(arm+'.npz'), T_composed_camera=trajectory,
                T_camera0_from_camera1=increments, time_ns=times,
                local_reference_translation_error_m=t, local_reference_rotation_error_deg=r)
            print(directory.parent.name, directory.name, arm, 'T/R RMS',
                  results[arm]['trajectory']['anchored_translation_rmse_m'],
                  results[arm]['trajectory']['anchored_rotation_rmse_deg'], flush=True)
        if not all(h == signatures[0] for h in signatures):
            raise ValueError('Nonpixel inputs differ between arms')
        results['validation'] = {'learned_matches_baseline': results['learned']['matches_baseline_tolerance'],
            'identical_nonpixel_inputs': True,
            'mass_max_relative_change': max(p['weight_protocol']['max_precision_relative_change']
                                            for a in ARMS for p in results[a]['pairs'])}
        return {'status': 'REPLAY_PASS' if results['validation']['learned_matches_baseline'] else 'CONTROL_FAILED',
                'baseline_report': report, 'results': results}


def evaluate(args):
    plan = json.loads((args.output/'plan.json').read_text())
    verify(plan)
    replay = args.output/'replay'; replay.mkdir(exist_ok=False)
    baseline = args.output/'baseline'
    source_hashes = {str(p.resolve()): sha(p) for p in baseline.rglob('*') if p.is_file()}
    (replay/'baseline-inputs.json').write_text(json.dumps(source_hashes, indent=2)+'\n')
    torch.set_num_threads(1); torch.manual_seed(0); np.random.seed(0)
    reference, _ = load_reference(Path(plan['dataset']))
    rows = json.loads(Path(plan['manifest']).read_text())['pairs']
    results = {w['name']: {m: run_case(baseline/w['name']/m, replay/w['name']/m, reference,
                                      Path(plan['dataset']), Path(plan['calibration']),
                                      rows[w['manifest_start']:w['manifest_start']+20], w['manifest_start'],
                                      plan['files_sha256'][str((ROOT.parent/'models/macslam'/
                                          ('MACVO_FrontendCov.pth' if m == 'rgb' else 'MACT_Offroad.pth')).resolve())])
                           for m in MODALITIES}
               for w in plan['windows']}
    verify(plan)
    if any(sha(Path(p)) != h for p, h in source_hashes.items()):
        raise ValueError('Baseline inputs changed during replay')
    report = {'created_at_utc': datetime.now(timezone.utc).isoformat(), 'plan_sha256': sha(args.output/'plan.json'),
              'all_cases_pass': all(c['status'] == 'REPLAY_PASS' for v in results.values() for c in v.values()),
              'frozen_files_unchanged': True, 'baseline_files_unchanged': True,
              'torch': torch.__version__, 'pypose': __import__('pypose').__version__, 'numpy': np.__version__,
              'results': results}
    (replay/'analysis.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    rows = []
    for w, modalities in results.items():
        for m, case in modalities.items():
            for a in ARMS:
                entry = {'window': w, 'modality': m, 'arm': a, 'status': case['status']}
                if 'results' in case:
                    r = case['results'][a]
                    entry.update({k: r['trajectory'][k] for k in
                        ['anchored_translation_rmse_m', 'anchored_rotation_rmse_deg']})
                    entry.update({k: r[k] for k in ['local_translation_rmse_m', 'local_rotation_rmse_deg']})
                rows.append(entry)
    fields = ['window', 'modality', 'arm', 'status', 'anchored_translation_rmse_m',
              'anchored_rotation_rmse_deg', 'local_translation_rmse_m', 'local_rotation_rmse_deg']
    with (replay/'summary.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader(); writer.writerows(rows)
    if any(c['status'] == 'CONTROL_FAILED' for v in results.values() for c in v.values()):
        raise RuntimeError('Learned replay failed; investigate before interpretation')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['prepare', 'evaluate'])
    parser.add_argument('--output', type=Path, required=True)
    for name in ['dataset', 'manifest', 'evaluated-manifest', 'protocol', 'preflight', 'calibration']:
        parser.add_argument('--'+name, type=Path)
    args = parser.parse_args()
    if args.action == 'prepare' and any(getattr(args, n) is None for n in
            ['dataset', 'manifest', 'evaluated_manifest', 'protocol', 'preflight', 'calibration']):
        parser.error('prepare requires all dataset/protocol/preflight arguments')
    (prepare if args.action == 'prepare' else evaluate)(args)
