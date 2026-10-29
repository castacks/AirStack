"""Offline saved-factor diagnostics; no labels, solves, or estimator changes.

Information conditions on fixed landmarks and stored target covariance. It is
unrobust, undamped, and expressed in a right camera tangent (m, rad).
"""
import argparse
from datetime import datetime
from zoneinfo import ZoneInfo
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

CASES = ('shared-arbiter-events', 'arbiter-support-collapse', 'history-day-replication')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def camera_points(points, pose):
    pose = np.asarray(pose, dtype=float).reshape(-1)
    if pose.shape != (7,) or not np.isfinite(pose).all() or abs(np.linalg.norm(pose[3:])-1) > 1e-5:
        raise ValueError('finite unit xyzw pose required')
    return (np.asarray(points, dtype=float)-pose[:3]) @ Rotation.from_quat(pose[3:]).as_matrix()


def residual_jacobian(points_camera, observed_uv, observed_disp, k, baseline):
    p = np.asarray(points_camera, dtype=float)
    uv = np.asarray(observed_uv, dtype=float)
    disp = np.asarray(observed_disp, dtype=float).reshape(-1)
    k = np.asarray(k, dtype=float)
    n = len(p)
    if p.shape != (n, 3) or uv.shape != (n, 2) or disp.shape != (n,):
        raise ValueError('aligned points and observations required')
    if k.shape != (3, 3) or not np.isfinite(k).all() or k[0, 1] != 0 or not np.array_equal(k[2], [0, 0, 1]) or k[1, 0] != 0 or min(k[0, 0], k[1, 1]) <= 0:
        raise ValueError('canonical zero-skew intrinsics required')
    if not np.isfinite(p).all() or not np.isfinite(uv).all() or not np.isfinite(disp).all() or not np.isfinite(baseline) or np.any(p[:, 0] <= 0):
        raise ValueError('finite observations and positive forward depths required')
    x, y, z = p.T
    fx, fy = k[0, 0], k[1, 1]
    prediction = np.column_stack((fx*y/x+k[0, 2], fy*z/x+k[1, 2], fx*baseline/x))
    residual = prediction-np.column_stack((uv, disp))
    projection = np.zeros((n, 3, 3))
    projection[:, 0, 0] = -fx*y/x**2
    projection[:, 0, 1] = fx/x
    projection[:, 1, 0] = -fy*z/x**2
    projection[:, 1, 2] = fy/x
    projection[:, 2, 0] = -fx*baseline/x**2
    motion = np.zeros((n, 3, 6))
    motion[:, :, :3] = -np.eye(3)
    motion[:, 0, 4] = -z
    motion[:, 0, 5] = y
    motion[:, 1, 3] = z
    motion[:, 1, 5] = -x
    motion[:, 2, 3] = -y
    motion[:, 2, 4] = x
    return residual, projection @ motion


def summarize(residual, jacobian, covariance):
    r, j, c = map(lambda a: np.asarray(a, dtype=float), (residual, jacobian, covariance))
    n = len(r)
    if not n or r.shape != (n, 3) or j.shape != (n, 3, 6) or c.shape != (n, 3, 3):
        raise ValueError('nonempty aligned 3D residual blocks required')
    if not all(np.isfinite(a).all() for a in (r, j, c)) or not np.allclose(c, c.transpose(0, 2, 1), rtol=0, atol=1e-12):
        raise ValueError('finite residuals and symmetric covariance required')
    try:
        l = np.linalg.cholesky(c)
    except np.linalg.LinAlgError as error:
        raise ValueError('positive definite covariance required; no clipping') from error
    wr = np.linalg.solve(l, r[..., None])[..., 0]
    wj = np.linalg.solve(l, j).reshape(-1, 6)
    singular = np.linalg.svd(wj, compute_uv=False)
    threshold = max(wj.shape)*np.finfo(float).eps*singular[0]
    rank = int(np.count_nonzero(singular > threshold))
    return {'factor_count': n, 'residual_dimension': 3*n,
            'median_reprojection_px': float(np.median(np.linalg.norm(r[:, :2], axis=1))),
            'median_abs_disparity_px': float(np.median(np.abs(r[:, 2]))),
            'whitened_sum_squares': float(np.sum(wr*wr)),
            'whitened_mean_per_component': float(np.mean(wr*wr)),
            'information_right_camera_m_rad': (wj.T @ wj).tolist(),
            'whitened_jacobian_singular_values': singular.tolist(),
            'numerical_rank': rank, 'rank_threshold': float(threshold),
            'mixed_units_condition_number': float(singular[0]/singular[-1]) if rank == 6 else None,
            'pose_covariance': None, 'optimizer_convergence': None}


def pair_factors(saved, pair):
    first, second = saved['edge/match2frame1/mapping'], saved['edge/match2frame2/mapping']
    mask = (first == pair) & (second == pair+1)
    ids = saved['edge/match2point/mapping'][mask]
    if ids.dtype.kind not in 'iu' or np.any(ids < 0) or np.any(ids >= len(saved['points//pos_Tw'])):
        raise ValueError('invalid point associations')
    return mask, saved['points//pos_Tw'][ids].astype(float)


def audit(root, output):
    output.mkdir(parents=True, exist_ok=False)
    sources = {}; rows = []
    def freeze(p):
        sources[str(p.relative_to(root))] = sha(p)
        return p
    for directory in CASES:
        run = root/'results'/directory/'run-001'
        proof = json.loads(freeze(run/'validation.json').read_text())
        if proof['status'] != 'PASS' or proof['timeline_sha256'] != sha(freeze(run/'timeline.jsonl')):
            raise ValueError('unverified timeline')
        decisions = {d['pair_ordinal']: d for d in (json.loads(line) for line in (run/'timeline.jsonl').read_text().splitlines()) if d['kind'] == 'decision'}
        for camera in ('rgb', 'thermal'):
            expert = run/'experts'/camera
            config = json.loads(freeze(expert/'effective-config.json').read_text())['Odometry']
            expected_optimizer = {'device': 'cpu', 'vectorize': True, 'parallel': False, 'graph_type': 'disp', 'autodiff': False}
            if config['optimizer']['type'] != 'TwoFrame_PGO' or config['optimizer']['args'] != expected_optimizer or config['args']['mapping'] or config['postprocess']['type'] != 'MotionInterpolate':
                raise ValueError('audit limited to global disp optimizer without mapping')
            journal_path = freeze(expert/'arbiter-events/events.jsonl')
            preservation = json.loads(freeze(run.parent/(camera+'-preservation.json')).read_text())
            if preservation['status'] != 'PASS' or preservation['events_sha256'] != sha(journal_path):
                raise ValueError('unverified journal')
            events = [json.loads(line) for line in journal_path.read_text().splitlines()]
            refinement = min(e['sequence'] for e in events if e['kind'] == 'refinement_begin')
            with np.load(freeze(expert/'tensor-map.npz'), allow_pickle=False) as saved:
                for pair in range(len(saved['frames//time_ns'])-1):
                    times = [int(t) for t in saved['frames//time_ns'][pair:pair+2]]
                    health = [e for e in events if e['kind'] == 'pair_end' and [e['time0_ns'], e['time1_ns']] == times]
                    if len(health) != 1: raise ValueError('unique exact interval health required')
                    h = health[0]
                    mask, points = pair_factors(saved, pair)
                    with np.load(freeze(expert/'uncertainty'/f'pair-{pair:03d}.npz'), allow_pickle=False) as raw:
                        if int(raw['time0_ns']) != times[0]: raise ValueError('raw interval mismatch')
                        inbound, keep = raw['selected_inbound'], raw['outlier_keep_inbound']
                        if inbound.dtype != np.bool_ or keep.dtype != np.bool_: raise ValueError('boolean retention masks required')
                        for endpoint in (0, 1):
                            expected = raw[f'selected_uv{endpoint}'][inbound][keep]
                            if not np.array_equal(expected, saved[f'match//pixel{endpoint+1}_uv'][mask]):
                                raise ValueError('retained factor identity mismatch')
                        if len(points) != h['retained_count']: raise ValueError('retained factor count mismatch')
                    motions = [e for e in events if e['kind'] == 'motion_result_written' and e['first_write_for_submission']
                               and [e['time0_ns'], e['time1_ns']] == times]
                    if len(motions) > 1: raise ValueError('duplicate first motion')
                    row = {'case': directory, 'camera': camera, 'pair_ordinal': pair, 'time_ns': times,
                           'retained_factor_count': len(points), 'original_need_interp': h['original_need_interp'],
                           'available_at_original_deadline': decisions[pair]['availability'][camera],
                           'motion_event_sequence': None, 'diagnostics': None, 'unavailable_reason': None}
                    if not motions:
                        row['unavailable_reason'] = 'no_first_motion_write'
                    elif h['original_need_interp']:
                        raise ValueError('unsupported interval has first motion')
                    else:
                        m = motions[0]
                        if m['sequence'] >= refinement or m['original_need_interp'] or m['from_idx'] != pair or m['frame_idx'] != pair+1:
                            raise ValueError('noncausal or mismatched motion')
                        row['motion_event_sequence'] = m['sequence']
                        row['motion_observation_monotonic_ns'] = m['observation_monotonic_ns']
                        row['native_pose_xyzw'] = m['optimizer_pose_before_write_native_parameterization']
                        p = camera_points(points, row['native_pose_xyzw'])
                        k = saved['frames//K'][pair+1].astype(float)
                        uv = saved['match//pixel2_uv'][mask]
                        disp = saved['match//pixel2_disp'][mask]
                        r, j = residual_jacobian(p, uv, disp, k, float(saved['frames//baseline'][pair+1]))
                        cov = np.zeros((len(points), 3, 3))
                        a = saved['match//pixel2_uv_cov'][mask].astype(float)
                        cov[:, 0, 0], cov[:, 1, 1] = a[:, 0], a[:, 1]
                        cov[:, 0, 1] = cov[:, 1, 0] = a[:, 2]
                        cov[:, 2, 2] = saved['match//pixel2_disp_cov'][mask].reshape(-1)
                        row['diagnostics'] = summarize(r, j, cov)
                    rows.append(row)
    for rel in ('Scripts/Setup/audit_optimizer_factors.py', 'Scripts/Setup/arbiter_event_recorder.py',
                'Module/Optimization/TwoFramePGO/Graphs.py', 'Module/Optimization/TwoFramePGO/Optimizer.py',
                'Module/Optimization/PyposeOptimizers.py', 'Odometry/MACVO.py', 'Module/MapProcessor.py'):
        freeze(root/'MAC-VO-main'/rel)
    for rel, digest in sources.items():
        if sha(root/rel) != digest: raise ValueError('input changed during audit')
    report = {'created_at': datetime.now(ZoneInfo('America/New_York')).isoformat(), 'base_commit': '2c5ad073d',
              'labels_loaded': False, 'fitted': False, 'input_sha256': sources,
              'scope': 'Offline reconstruction at pre-write native pose, fixed saved landmarks; unrobust undamped conditional information; not measured latency, optimizer Hessian, calibrated covariance, or independent error.',
              'rows': rows}
    (output/'audit.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'rows': len(rows), 'diagnostics': sum(r['diagnostics'] is not None for r in rows)}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    audit(args.root, args.output)
