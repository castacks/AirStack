"""Frozen native replay of u-only and v-only precision-mass redistribution."""

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
from Scripts.Setup.td3_backend_ablation import pair_input, solve, compose, local_error
from Scripts.Setup.td3_precision_mass import (
    apply_weights as previous_weights, nonpixel_fingerprint, sha, within_tolerance,
    WINDOWS, MODALITIES, TOLERANCES,
)
from Scripts.Setup.td3_reference import matrices, load_reference, camera_reference, trajectory_metrics

ARMS = ('learned', 'fixed_u_mass', 'fixed_v_mass', 'fixed_uv_mass')
CHANNELS = {'learned': (), 'fixed_u_mass': (0,), 'fixed_v_mass': (1,), 'fixed_uv_mass': (0, 1)}
CONTROLS = ('learned', 'fixed_uv_mass')


def apply_weights(data, arm, constants):
    if arm not in ARMS:
        raise ValueError('Unknown channel arm')
    # Validate all covariance channels using the unchanged EXP-011 helper.
    previous_weights(data, 'learned', constants)
    uv = data.observations.data['pixel2_uv_cov']
    original = uv.clone()
    before = nonpixel_fingerprint(data)
    mass = original[:, :2].double().reciprocal().sum(dim=0)
    if arm in CONTROLS:
        previous_weights(data, arm, constants)
    else:
        channel = CHANNELS[arm][0]
        uv[:, channel] = (len(uv)/mass[channel]).to(uv)
    unchanged = all(torch.equal(uv[:, c], original[:, c]) for c in range(2) if c not in CHANNELS[arm])
    uniform = all(torch.equal(uv[:, c], uv[0, c].expand(len(uv))) for c in CHANNELS[arm])
    applied = uv[:, :2].double().reciprocal().sum(dim=0)
    relative = (applied/mass-1).abs()
    if (not unchanged or not uniform or not torch.equal(uv[:, 2], original[:, 2])
            or nonpixel_fingerprint(data) != before
            or (relative > TOLERANCES['precision_relative']).any()):
        raise ValueError('Channel, nonpixel or precision conservation invariant failed')
    return {'nonpixel_sha256': before, 'uniform_channels': list(CHANNELS[arm]),
            'unselected_channels_bitwise_unchanged': unchanged,
            'selected_channels_uniform': uniform, 'original_precision_u_v': mass.tolist(),
            'applied_precision_u_v': applied.tolist(), 'precision_ratio_u_v': (applied/mass).tolist(),
            'max_precision_relative_change': float(relative.max()),
            'uniform_variances_px2': {str(c): float(uv[0, c]) for c in CHANNELS[arm]}}


def verify(plan, args):
    if plan['arms'] != list(ARMS) or plan['threads'] != args.threads or plan['torch'] != torch.__version__:
        raise ValueError('Frozen execution setting mismatch')
    for name, digest in plan['files_sha256'].items():
        if sha(Path(name)) != digest:
            raise ValueError(f'Frozen file changed: {name}')


def prepare(args):
    old = json.loads((args.previous/'plan.json').read_text())
    # Include the entire predecessor's frozen file set, then pin its outputs.
    hashes = dict(old['files_sha256'])
    for name, digest in hashes.items():
        if sha(Path(name)) != digest:
            raise ValueError(f'EXP-011 frozen file mismatch: {name}')
    old_report = json.loads((args.previous/'analysis.json').read_text())
    if not old_report['controls_pass'] or not old_report['frozen_files_unchanged']:
        raise ValueError('Predecessor control validation failed')
    files = [Path(__file__), args.previous/'plan.json', args.previous/'analysis.json',
             *[args.previous/w/m/(a+'.npz') for w in WINDOWS for m in MODALITIES for a in CONTROLS]]
    hashes.update({str(p.resolve()): sha(p) for p in files})
    # Explicitly pin current CLI sources as well, even if a different source root is provided.
    current = [args.source/w/m/f for w in WINDOWS for m in MODALITIES
               for f in ['tensor-map.npz', 'trajectory.npz', 'effective-config.json']]
    for p in current:
        key = str(p.resolve())
        if key not in old['files_sha256'] or old['files_sha256'][key] != sha(p):
            raise ValueError('Source root differs from EXP-011')
    if old['threads'] != args.threads or old['torch'] != torch.__version__:
        raise ValueError('Runtime differs from predecessor')
    required = [args.calibration, *[args.dataset/'super_odometry/odometry'/f
                for f in ['interp_data.txt', 'interp_timestamps.txt', 'info.yaml']],
                *[args.dataset/'sensors'/n/'data.yaml' for n in
                  ['multisense_left_rect_color_camera_info', 'thermal_left_processed_camera_info']]]
    if any(str(p.resolve()) not in old['files_sha256'] for p in required):
        raise ValueError('Reference/calibration paths differ from EXP-011')
    plan = {'created_at_utc': datetime.now(timezone.utc).isoformat(), 'arms': ARMS,
            'rule': 'Only selected u/v channels made uniform at N/sum(1/learned variance) per pair/channel; every channel precision total preserved before Huber; unselected channels bitwise unchanged',
            'controls': 'learned and fixed_uv_mass must match EXP-011 local motions',
            'scope': 'frozen-prior independent pair replay, same clip, provisional geometry and SLAM proxy; no scale fitting or new inference',
            'constants': old['constants'], 'tolerances': TOLERANCES,
            'torch': torch.__version__, 'threads': args.threads, 'files_sha256': hashes}
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'plan.json').write_text(json.dumps(plan, indent=2)+'\n')
    print('Frozen channel protocol and', len(hashes), 'file hashes', flush=True)


def run_case(args, plan, window, modality, reference):
    source = args.source/window/modality
    output = args.output/window/modality
    output.mkdir(parents=True, exist_ok=False)
    with np.load(source/'tensor-map.npz', allow_pickle=False) as saved, \
            np.load(source/'trajectory.npz', allow_pickle=False) as original:
        times = original['time_ns'].reshape(-1)
        if (not np.array_equal(times, saved['frames//time_ns'].reshape(-1))
                or not np.array_equal(original['poses_camera_ned'], saved['frames//pose'])
                or saved['frames//need_interp'].any()):
            raise ValueError('Source association mismatch or interpolation')
        config = json.loads((source/'effective-config.json').read_text())
        if config['Odometry']['optimizer']['args'] != {
                'device': 'cpu', 'vectorize': True, 'parallel': False, 'graph_type': 'disp', 'autodiff': False}:
            raise ValueError('Unexpected source optimizer')
        old = matrices(saved['frames//pose'])
        old_delta = np.linalg.inv(old[:-1]) @ old[1:]
        camera = camera_reference(reference, times, args.dataset, args.calibration, modality)
        ref_delta = np.linalg.inv(camera[:-1]) @ camera[1:]
        reports, signatures = {}, []
        for arm in ARMS:
            motions, pairs, hashes = [], [], []
            for i in range(len(times)-1):
                data = pair_input(saved, i)
                weights = apply_weights(data, arm, plan['constants'][modality]['variance_u_v_disp_px2'])
                pose, metrics = solve(data, 'disp')
                metrics['weight_protocol'] = weights
                motions.append(np.linalg.inv(old[i]) @ matrices(pose[None])[0])
                pairs.append(metrics); hashes.append(weights['nonpixel_sha256'])
            signatures.append(hashes)
            increments = np.stack(motions)
            trajectory = compose(increments)
            t, r = local_error(increments, ref_delta)
            dt, dr = local_error(increments, old_delta)
            report = {'trajectory': trajectory_metrics(trajectory, camera),
                      'local_translation_rmse_m': float(np.sqrt(np.mean(t*t))),
                      'local_rotation_rmse_deg': float(np.sqrt(np.mean(r*r))), 'pairs': pairs,
                      'max_local_translation_difference_from_original_m': float(dt.max()),
                      'max_local_rotation_difference_from_original_deg': float(dr.max())}
            if arm in CONTROLS:
                with np.load(args.previous/window/modality/(arm+'.npz'), allow_pickle=False) as prior:
                    if not np.array_equal(times, prior['time_ns'].reshape(-1)):
                        raise ValueError('Previous control timestamp mismatch')
                    ct, cr = local_error(increments, prior['T_camera0_from_camera1'])
                report['previous_control'] = {'passed': within_tolerance(ct, cr),
                    'max_local_translation_difference_m': float(ct.max()),
                    'max_local_rotation_difference_deg': float(cr.max())}
            reports[arm] = report
            np.savez_compressed(output/(arm+'.npz'), T_composed_camera=trajectory,
                T_camera0_from_camera1=increments, time_ns=times,
                local_reference_translation_error_m=t, local_reference_rotation_error_deg=r)
            print(window, modality, arm, 'translation/rotation RMS',
                  report['trajectory']['anchored_translation_rmse_m'],
                  report['trajectory']['anchored_rotation_rmse_deg'], flush=True)
        if not all(h == signatures[0] for h in signatures):
            raise ValueError('Arms differ in nonpixel inputs')
        reports['validation'] = {'identical_nonpixel_inputs': True,
            'previous_controls_pass': all(reports[a]['previous_control']['passed'] for a in CONTROLS),
            'learned_matches_original': within_tolerance(
                np.array([reports['learned']['max_local_translation_difference_from_original_m']]),
                np.array([reports['learned']['max_local_rotation_difference_from_original_deg']])),
            'mass_max_relative_change': max(p['weight_protocol']['max_precision_relative_change']
                                            for a in ARMS for p in reports[a]['pairs'])}
        (output/'analysis.json').write_text(json.dumps(reports, indent=2, allow_nan=False)+'\n')
        return reports


def summarize(results, output):
    rows = []
    for w in WINDOWS:
        for m in MODALITIES:
            for a in ARMS:
                report = results[w][m][a]
                rows.append({'window': w, 'modality': m, 'arm': a,
                             **report['trajectory'], 'local_translation_rmse_m': report['local_translation_rmse_m'],
                             'local_rotation_rmse_deg': report['local_rotation_rmse_deg']})
    with (output/'summary.csv').open('x') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys()); writer.writeheader(); writer.writerows(rows)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), layout='constrained')
    for col, m in enumerate(MODALITIES):
        for row, field, units in [(0, 'anchored_translation_rmse_m', 'Translation RMS (m)'),
                                   (1, 'anchored_rotation_rmse_deg', 'Rotation RMS (deg)')]:
            ax = axes[row, col]
            for i, a in enumerate(ARMS):
                ax.bar(np.arange(3)+(i-1.5)*.19, [results[w][m][a]['trajectory'][field] for w in WINDOWS],
                       width=.18, label=a)
            ax.set_xticks(np.arange(3), WINDOWS); ax.set_title(m.upper()); ax.set_ylabel(units)
            ax.grid(axis='y', alpha=.2); ax.set_axisbelow(True)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='outside lower center', ncol=4, frameon=False)
    fig.suptitle('Channel-specific redistribution, per-channel precision preserved\nRecorded-SLAM proxy; fixed independent priors, no fitted scale')
    for ext in ['png', 'pdf']:
        fig.savefig(output/('channel-mass-comparison.'+ext), dpi=170)
    plt.close(fig)


def evaluate(args):
    plan = json.loads((args.output/'plan.json').read_text())
    verify(plan, args)
    torch.set_num_threads(args.threads); torch.manual_seed(0); np.random.seed(0)
    reference, _ = load_reference(args.dataset)
    results = {w: {m: run_case(args, plan, w, m, reference) for m in MODALITIES} for w in WINDOWS}
    verify(plan, args)
    passed = all(results[w][m]['validation']['previous_controls_pass'] and
                 results[w][m]['validation']['learned_matches_original'] for w in WINDOWS for m in MODALITIES)
    report = {'created_at_utc': datetime.now(timezone.utc).isoformat(), 'plan': plan,
              'plan_sha256': sha(args.output/'plan.json'), 'controls_pass': passed,
              'frozen_files_unchanged': True, 'torch': torch.__version__,
              'pypose': __import__('pypose').__version__, 'numpy': np.__version__, 'results': results}
    with (args.output/'analysis.json').open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    if not passed:
        raise RuntimeError('Replay control failed; do not interpret experiment')
    summarize(results, args.output)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['prepare', 'evaluate'])
    for name in ['source', 'previous', 'dataset', 'calibration', 'output']:
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--threads', type=int, default=1)
    args = parser.parse_args()
    if args.threads < 1:
        parser.error('threads must be positive')
    (prepare if args.action == 'prepare' else evaluate)(args)
