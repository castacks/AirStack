"""Native fixed-input replay separating pixel precision distribution and mass."""

import argparse
from dataclasses import replace
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from Module.Map import MatchObs
from Scripts.Setup.td3_backend_ablation import pair_input, fingerprint, solve, compose, local_error
from Scripts.Setup.td3_reference import load_reference, camera_reference, matrices, trajectory_metrics
from Scripts.Setup.td3_weight_ablation import set_weights

ARMS = ('learned', 'fixed_uv', 'fixed_uv_mass')
WINDOWS = ('early', 'original', 'late')
MODALITIES = ('rgb', 'thermal')
TOLERANCES = {'local_translation_m': 1e-3, 'local_rotation_deg': .01,
              'precision_relative': 1e-6}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def nonpixel_fingerprint(data):
    obs = MatchObs.init({k: v for k, v in data.observations.data.items() if k != 'pixel2_uv_cov'})
    return fingerprint(replace(data, observations=obs))


def apply_weights(data, arm, constants):
    if arm not in ARMS:
        raise ValueError('Unknown arm')
    before = nonpixel_fingerprint(data)
    uv = data.observations.data['pixel2_uv_cov']
    if (uv.ndim != 2 or uv.shape[1] != 3 or not len(uv)
            or not torch.isfinite(uv).all() or (uv[:, :2] <= 0).any()
            or (uv[:, 2] != 0).any()):
        raise ValueError('Expected finite positive u/v variance and zero cross-covariance')
    # Reuse the original protocol's positive disparity/constant validation.
    set_weights(data, 'learned', constants)
    original_mass = uv[:, :2].double().reciprocal().sum(dim=0)
    if arm == 'fixed_uv_mass':
        # Harmonic mean per channel: N / sum(1 / learned variance).
        uniform = (len(uv) / original_mass).to(uv)
        uv[:, :2] = uniform
    else:
        set_weights(data, arm, constants)
    if nonpixel_fingerprint(data) != before:
        raise ValueError('Weight arm changed nonpixel inputs, including disparity')
    actual_mass = uv[:, :2].double().reciprocal().sum(dim=0)
    relative = (actual_mass / original_mass - 1).abs()
    if arm in ('learned', 'fixed_uv_mass') and (relative > TOLERANCES['precision_relative']).any():
        raise ValueError('Per-channel pixel precision mass was not preserved')
    return {'nonpixel_sha256': before, 'original_precision_u_v': original_mass.tolist(),
            'applied_precision_u_v': actual_mass.tolist(),
            'precision_ratio_u_v': (actual_mass / original_mass).tolist(),
            'max_precision_relative_change': float(relative.max()),
            'uniform_variance_u_v_px2': uv[0, :2].tolist() if arm != 'learned' else None}


def source_files():
    return [Path(__file__), ROOT/'Scripts/Setup/td3_backend_ablation.py',
            ROOT/'Scripts/Setup/td3_weight_ablation.py', ROOT/'Scripts/Setup/td3_reference.py',
            ROOT/'Scripts/Setup/audit_td3_geometry.py', ROOT/'Module/Map/Template.py',
            ROOT/'Module/Map/Graph.py', ROOT/'Module/Optimization/PyposeOptimizers.py',
            ROOT/'Module/Optimization/TwoFramePGO/Graphs.py',
            ROOT/'Module/Optimization/TwoFramePGO/Optimizer.py']


def input_files(args):
    return [args.previous/'plan.json', args.previous/'analysis.json', args.calibration,
            *[args.source/w/m/f for w in WINDOWS for m in MODALITIES
              for f in ['tensor-map.npz', 'trajectory.npz', 'effective-config.json']],
            *[args.previous/w/m/(arm+'.npz') for w in WINDOWS for m in MODALITIES
              for arm in ['learned', 'fixed_uv']],
            *[args.dataset/'super_odometry/odometry'/f
              for f in ['interp_data.txt', 'interp_timestamps.txt', 'info.yaml']],
            *[args.dataset/'sensors'/name/'data.yaml' for name in
              ['multisense_left_rect_color_camera_info', 'thermal_left_processed_camera_info']]]


def verify_plan(plan, args):
    if plan['arms'] != list(ARMS) or plan['threads'] != args.threads or plan['torch'] != torch.__version__:
        raise ValueError('Frozen execution setting mismatch')
    for name, expected in plan['files_sha256'].items():
        if sha(Path(name)) != expected:
            raise ValueError(f'Frozen file hash mismatch: {name}')


def prepare(args):
    previous = json.loads((args.previous/'plan.json').read_text())
    for name, digest in previous['source_inputs_sha256'].items():
        if sha(args.source/name) != digest:
            raise ValueError('Original EXP-009 source hash mismatch')
    if sha(args.calibration) != previous['calibration_sha256']:
        raise ValueError('Original calibration hash mismatch')
    inputs = input_files(args) + source_files()
    plan = {'created_at_utc': datetime.now(timezone.utc).isoformat(), 'arms': ARMS,
            'rule': 'fixed_uv_mass: per pair/channel uniform variance N/sum(1/learned variance); preserve u and v total precision separately before Huber; disparity unchanged',
            'controls': 'learned and fixed_uv must match EXP-009 saved local motions',
            'scope': 'independent frozen-prior native solves; same clip/provisional geometry/recorded-SLAM proxy, not new end-to-end VO',
            'constants': previous['constants'], 'tolerances': TOLERANCES,
            'torch': torch.__version__, 'threads': args.threads,
            'files_sha256': {str(p.resolve()): sha(p) for p in inputs}}
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'plan.json').write_text(json.dumps(plan, indent=2)+'\n')
    print('Frozen protocol and', len(inputs), 'file hashes before replay', flush=True)


def within_tolerance(t, r):
    return bool(t.max() <= TOLERANCES['local_translation_m'] and
                r.max() <= TOLERANCES['local_rotation_deg'])


def run_case(args, plan, window, modality, reference):
    directory = args.source/window/modality
    output = args.output/window/modality
    output.mkdir(parents=True, exist_ok=False)
    with np.load(directory/'tensor-map.npz', allow_pickle=False) as saved, \
            np.load(directory/'trajectory.npz', allow_pickle=False) as original:
        times = original['time_ns'].reshape(-1)
        if (not np.array_equal(times, saved['frames//time_ns'].reshape(-1))
                or not np.array_equal(original['poses_camera_ned'], saved['frames//pose'])
                or saved['frames//need_interp'].any()):
            raise ValueError('Invalid source association or interpolated source')
        config = json.loads((directory/'effective-config.json').read_text())
        if config['Odometry']['optimizer']['args'] != {
                'device': 'cpu', 'vectorize': True, 'parallel': False,
                'graph_type': 'disp', 'autodiff': False}:
            raise ValueError('Unexpected source optimizer')
        old = matrices(saved['frames//pose'])
        old_delta = np.linalg.inv(old[:-1]) @ old[1:]
        camera = camera_reference(reference, times, args.dataset, args.calibration, modality)
        ref_delta = np.linalg.inv(camera[:-1]) @ camera[1:]
        reports, signatures = {}, []
        for arm in ARMS:
            increments, pairs, hashes = [], [], []
            for i in range(len(times)-1):
                data = pair_input(saved, i)
                weights = apply_weights(data, arm, plan['constants'][modality]['variance_u_v_disp_px2'])
                pose, metrics = solve(data, 'disp')
                metrics['weight_protocol'] = weights
                increments.append(np.linalg.inv(old[i]) @ matrices(pose[None])[0])
                pairs.append(metrics); hashes.append(weights['nonpixel_sha256'])
            signatures.append(hashes)
            increments = np.stack(increments)
            trajectory = compose(increments)
            t, r = local_error(increments, ref_delta)
            original_t, original_r = local_error(increments, old_delta)
            report = {'trajectory': trajectory_metrics(trajectory, camera),
                      'local_translation_rmse_m': float(np.sqrt(np.mean(t*t))),
                      'local_rotation_rmse_deg': float(np.sqrt(np.mean(r*r))), 'pairs': pairs,
                      'max_local_translation_difference_from_original_m': float(original_t.max()),
                      'max_local_rotation_difference_from_original_deg': float(original_r.max())}
            if arm != 'fixed_uv_mass':
                with np.load(args.previous/window/modality/(arm+'.npz'), allow_pickle=False) as previous:
                    if not np.array_equal(times, previous['time_ns'].reshape(-1)):
                        raise ValueError('EXP-009 control time association mismatch')
                    dt, dr = local_error(increments, previous['T_camera0_from_camera1'])
                report['previous_control'] = {
                    'passed': within_tolerance(dt, dr),
                    'max_local_translation_difference_m': float(dt.max()),
                    'max_local_rotation_difference_deg': float(dr.max())}
            reports[arm] = report
            np.savez_compressed(output/(arm+'.npz'), T_composed_camera=trajectory,
                T_camera0_from_camera1=increments, time_ns=times,
                local_reference_translation_error_m=t, local_reference_rotation_error_deg=r)
            print(window, modality, arm, 'translation/rotation RMS',
                  report['trajectory']['anchored_translation_rmse_m'],
                  report['trajectory']['anchored_rotation_rmse_deg'], flush=True)
        if not all(s == signatures[0] for s in signatures):
            raise ValueError('Arms differ in nonpixel inputs')
        reports['validation'] = {'identical_nonpixel_inputs': True,
            'learned_matches_original': within_tolerance(
                np.array([reports['learned']['max_local_translation_difference_from_original_m']]),
                np.array([reports['learned']['max_local_rotation_difference_from_original_deg']])),
            'previous_controls_pass': all(reports[a]['previous_control']['passed'] for a in ['learned', 'fixed_uv']),
            'mass_max_relative_change': max(p['weight_protocol']['max_precision_relative_change']
                                            for p in reports['fixed_uv_mass']['pairs'])}
        (output/'analysis.json').write_text(json.dumps(reports, indent=2, allow_nan=False)+'\n')
        return reports


def plot(results, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), layout='constrained')
    for col, modality in enumerate(MODALITIES):
        for row, metric, units in [(0, 'anchored_translation_rmse_m', 'Translation RMS (m)'),
                                    (1, 'anchored_rotation_rmse_deg', 'Rotation RMS (deg)')]:
            ax = axes[row, col]
            for i, arm in enumerate(ARMS):
                ax.bar(np.arange(3)+(i-1)*.24,
                       [results[w][modality][arm]['trajectory'][metric] for w in WINDOWS],
                       width=.23, label=arm)
            ax.set_xticks(np.arange(3), WINDOWS); ax.set_title(modality.upper())
            ax.set_ylabel(units); ax.grid(axis='y', alpha=.2); ax.set_axisbelow(True)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='outside lower center', ncol=3, frameon=False)
    fig.suptitle('Native fixed-input replay: recorded-SLAM disagreement\nSame clip, provisional geometry; first-pose anchoring, no fitted scale')
    for ext in ['png', 'pdf']:
        fig.savefig(output/('precision-mass-comparison.'+ext), dpi=170)
    plt.close(fig)


def evaluate(args):
    plan = json.loads((args.output/'plan.json').read_text())
    verify_plan(plan, args)
    torch.set_num_threads(args.threads); torch.manual_seed(0); np.random.seed(0)
    reference, _ = load_reference(args.dataset)
    results = {w: {m: run_case(args, plan, w, m, reference) for m in MODALITIES} for w in WINDOWS}
    verify_plan(plan, args)
    passed = all(results[w][m]['validation']['learned_matches_original'] and
                 results[w][m]['validation']['previous_controls_pass'] for w in WINDOWS for m in MODALITIES)
    report = {'created_at_utc': datetime.now(timezone.utc).isoformat(), 'plan': plan,
              'plan_sha256': sha(args.output/'plan.json'), 'controls_pass': passed,
              'frozen_files_unchanged': True, 'torch': torch.__version__,
              'pypose': __import__('pypose').__version__, 'numpy': np.__version__, 'results': results}
    with (args.output/'analysis.json').open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    if not passed:
        raise RuntimeError('Replay control failed; do not interpret experiment')
    plot(results, args.output)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['prepare', 'evaluate'])
    for key in ['source', 'previous', 'dataset', 'calibration', 'output']:
        parser.add_argument('--'+key, type=Path, required=True)
    parser.add_argument('--threads', type=int, default=1)
    args = parser.parse_args()
    if args.threads < 1:
        parser.error('threads must be positive')
    (prepare if args.action == 'prepare' else evaluate)(args)
