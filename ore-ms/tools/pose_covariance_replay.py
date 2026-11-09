"""Opt-in Gaussian 3D registration replay; no frontend or estimator mutation.

T maps target camera points into source camera coordinates. Tangents are left
perturbations, translation first (metres, radians). The objective includes the
rotation-dependent residual covariance and log determinant. Independent points
and endpoint errors are a modelling approximation, not a calibration claim.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
from zoneinfo import ZoneInfo

import numpy as np
from scipy.spatial.transform import Rotation


def skew(v):
    v = np.asarray(v, dtype=float)
    out = np.zeros(v.shape[:-1] + (3, 3))
    out[..., 0, 1], out[..., 0, 2] = -v[..., 2], v[..., 1]
    out[..., 1, 0], out[..., 1, 2] = v[..., 2], -v[..., 0]
    out[..., 2, 0], out[..., 2, 1] = -v[..., 1], v[..., 0]
    return out


def exp_se3(x):
    x = np.asarray(x, dtype=float)
    if x.shape != (6,) or not np.isfinite(x).all():
        raise ValueError('finite translation-first 6-vector required')
    w = skew(x[3:])
    theta = np.linalg.norm(x[3:])
    a = (1 - np.cos(theta)) / theta**2 if theta > 1e-5 else .5-theta**2/24+theta**4/720
    b = (theta - np.sin(theta)) / theta**3 if theta > 1e-5 else 1/6-theta**2/120+theta**4/5040
    t = np.eye(4)
    t[:3, :3] = Rotation.from_rotvec(x[3:]).as_matrix()
    t[:3, 3] = (np.eye(3)+a*w+b*w@w) @ x[:3]
    return t


def log_se3(t):
    validate_transform(t)
    phi = Rotation.from_matrix(t[:3, :3]).as_rotvec()
    # Recover the left Jacobian by applying exp to unit translation vectors.
    j = np.column_stack([exp_se3(np.r_[np.eye(3)[i], phi])[:3, 3] for i in range(3)])
    return np.r_[np.linalg.solve(j, t[:3, 3]), phi]


def validate_transform(t):
    t = np.asarray(t)
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1], atol=1e-12, rtol=0)
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-8, rtol=0)
            or abs(np.linalg.det(t[:3, :3])-1) > 1e-8):
        raise ValueError('proper finite SE(3) transform required')


def spd(a):
    a = np.asarray(a, dtype=float)
    if (a.ndim < 2 or a.shape[-2] != a.shape[-1] or not np.isfinite(a).all()
            or not np.allclose(a, np.swapaxes(a, -1, -2), rtol=1e-10, atol=1e-14)):
        raise ValueError('finite symmetric covariance required')
    try:
        np.linalg.cholesky(a)
    except np.linalg.LinAlgError as e:
        raise ValueError('strictly positive definite covariance required') from e
    return a


def validate_data(source, target, source_cov, target_cov):
    source, target = np.asarray(source, float), np.asarray(target, float)
    n = len(source)
    if (source.shape != (n, 3) or target.shape != (n, 3) or n < 3
            or not np.isfinite(source).all() or not np.isfinite(target).all()):
        raise ValueError('at least three finite paired 3D points required')
    source_cov, target_cov = spd(source_cov), spd(target_cov)
    if source_cov.shape != (n, 3, 3) or target_cov.shape != (n, 3, 3):
        raise ValueError('one 3x3 covariance at each endpoint per point required')
    return source, target, source_cov, target_cov


def gaussian_model(t, source, target, source_cov, target_cov):
    """NLL, exact left-tangent gradient, Gaussian Fisher and mean-only info.

    C changes with rotation. The Fisher uses expected residual curvature plus
    .5*tr(C^-1 dC_a C^-1 dC_b); it is not an observed Hessian or LM matrix.
    """
    rmat = t[:3, :3]
    q = target @ rmat.T + t[:3, 3]
    residual = q-source
    b = rmat @ target_cov @ rmat.T
    cov = source_cov + b
    inv = np.linalg.solve(cov, np.broadcast_to(np.eye(3), cov.shape))
    z = np.einsum('nij,nj->ni', inv, residual)
    cost = .5 * (np.sum(residual*z) + np.linalg.slogdet(cov)[1].sum())
    j = np.concatenate((np.broadcast_to(np.eye(3), (len(q), 3, 3)), -skew(q)), axis=2)
    dc = np.zeros((len(q), 6, 3, 3))
    for k, e in enumerate(np.eye(3)):
        dc[:, k+3] = skew(e) @ b - b @ skew(e)
    d = inv - z[:, :, None]*z[:, None, :]
    gradient = np.einsum('nia,ni->a', j, z) + .5*np.einsum('nij,naij->a', d, dc)
    mean_info = np.einsum('nia,nij,njb->ab', j, inv, j)
    inv_dc = np.einsum('nij,najk->naik', inv, dc)
    fisher = mean_info + .5*np.einsum('naij,nbji->ab', inv_dc, inv_dc)
    return float(cost), gradient, (fisher+fisher.T)/2, (mean_info+mean_info.T)/2


def information_inverse(h):
    eig = np.linalg.eigvalsh(h)
    tol = np.finfo(float).eps*6*max(abs(eig).max(), 1e-300)
    if eig[0] <= tol:
        raise ValueError('numerically rank-deficient information; no covariance')
    return np.linalg.solve(h, np.eye(6))


def objective_change(t, step, source, target, source_cov, target_cov):
    """Evaluate NLL(T_new)-NLL(T) without subtracting large total losses.

    Delta-inverse identity and log1p of whitened covariance increments retain
    small decreases close to stationarity. The objective itself is unchanged.
    """
    rmat = t[:3, :3]
    q = target@rmat.T+t[:3, 3]
    r = q-source
    b = rmat@target_cov@rmat.T
    c = source_cov+b
    change = exp_se3(step)
    u = change[:3, :3]-np.eye(3)
    dr = q@u.T+change[:3, 3]
    dc = u@b+b@u.T+u@b@u.T
    zold = np.linalg.solve(c, r[..., None])[..., 0]
    znew = np.linalg.solve(c+dc, r[..., None])[..., 0]
    zd = np.linalg.solve(c+dc, dr[..., None])[..., 0]
    quadratic = (2*np.sum(dr*znew)+np.sum(dr*zd)
                 - np.einsum('ni,nij,nj->', znew, dc, zold))
    chol = np.linalg.cholesky(c)
    whitened = np.linalg.solve(chol, dc)
    whitened = np.linalg.solve(chol, whitened.transpose(0, 2, 1)).transpose(0, 2, 1)
    eig = np.linalg.eigvalsh((whitened+whitened.transpose(0, 2, 1))/2)
    if np.any(eig <= -1):
        raise ValueError('covariance-change determinant numerically invalid')
    return float(.5*(quadratic+np.log1p(eig).sum()))


def register(source, target, source_cov, target_cov, initial=None, max_iterations=100,
             score_tolerance=1e-6):
    """Fisher scoring with Armijo backtracking on the exact declared NLL.

    Nonconvergence is a result. No pseudoinverse, covariance clipping, damping
    or silent restart. No reported covariance at a nonstationary result.
    """
    data = validate_data(source, target, source_cov, target_cov)
    t = np.eye(4) if initial is None else np.array(initial, dtype=float, copy=True)
    validate_transform(t)
    if max_iterations < 0 or not np.isfinite(score_tolerance) or score_tolerance <= 0:
        raise ValueError('nonnegative iteration cap and positive score tolerance required')
    history, status, covariance, mean_covariance = [], 'iteration_limit', None, None
    for iteration in range(max_iterations+1):
        cost, g, h, hm = gaussian_model(t, *data)
        try:
            inverse = information_inverse(h)
        except ValueError:
            status = 'rank_deficient'
            break
        decrement = float(np.sqrt(max(g @ inverse @ g, 0)))
        history.append({'iteration': iteration, 'nll': cost, 'score_norm': decrement})
        if decrement <= score_tolerance:
            try:
                mean_covariance = information_inverse(hm)
            except ValueError:
                status = 'mean_information_rank_deficient'
                break
            status, covariance = 'converged', inverse
            break
        if iteration == max_iterations:
            break
        direction = -inverse @ g
        accepted = False
        for line in range(25):
            alpha = .5**line
            trial = exp_se3(alpha*direction) @ t
            delta = objective_change(t, alpha*direction, *data)
            if np.isfinite(delta) and delta <= 1e-4*alpha*(g@direction):
                t, accepted = trial, True
                history[-1]['accepted_step_scale'] = alpha
                history[-1]['stable_nll_change'] = delta
                break
        if not accepted:
            status = 'line_search_stalled'
            break
    return {'status': status, 'transform': t, 'covariance': covariance,
            'mean_only_covariance': mean_covariance, 'history': history,
            'convention': 'left_SE3_translation_first_m_rad',
            'objective': 'independent_gaussian_3d_residual_with_rotation_covariance_and_logdet',
            'covariance_type': 'local_expected_Fisher_inverse_not_empirical_calibration'}


def transport_covariance(mapping, t, covariance, epsilon=1e-6):
    """Push forward a left covariance through an explicit SE(3) mapping."""
    validate_transform(t)
    covariance = spd(covariance)
    if covariance.shape != (6, 6) or epsilon <= 0:
        raise ValueError('6x6 covariance and positive derivative step required')
    out = mapping(t)
    validate_transform(out)
    inverse = np.linalg.inv(out)
    j = np.column_stack([
        (log_se3(mapping(exp_se3(epsilon*e)@t)@inverse)
         - log_se3(mapping(exp_se3(-epsilon*e)@t)@inverse))/(2*epsilon)
        for e in np.eye(6)])
    return j @ covariance @ j.T, j


def covariance_intersection(mean_a, cov_a, mean_b, cov_b):
    """Gaussian CI primitive in ONE caller-declared common linear tangent.

    This function does not pretend independently linearized SE(3) coordinates
    coincide; nonlinear fusion must explicitly supply/transport that tangent.
    """
    a, b = np.asarray(mean_a, float), np.asarray(mean_b, float)
    ca, cb = spd(cov_a), spd(cov_b)
    if (a.shape != (6,) or b.shape != (6,) or ca.shape != (6, 6)
            or cb.shape != (6, 6) or not np.isfinite([a, b]).all()):
        raise ValueError('finite common-tangent six-vectors and 6x6 covariances required')
    ia, ib = np.linalg.solve(ca, np.eye(6)), np.linalg.solve(cb, np.eye(6))
    candidates = []
    for w in np.linspace(0, 1, 21):
        info = w*ia+(1-w)*ib
        cov = np.linalg.solve(info, np.eye(6))
        mean = cov @ (w*ia@a + (1-w)*ib@b)
        candidates.append((np.linalg.slogdet(cov)[1], abs(w-.5), float(w), mean, cov))
    # Midpoint tie-break avoids arbitrary endpoint choices for identical inputs.
    best_logdet = min(c[0] for c in candidates)
    tied = [c for c in candidates if abs(c[0]-best_logdet) <= 1e-12]
    _, _, w, mean, cov = min(tied, key=lambda c: (c[1], c[2]))
    return {'mean': mean, 'covariance': cov, 'weight': w}


def pose_xyzw(vector):
    v = np.asarray(vector, float).reshape(-1)
    if v.shape != (7,) or not np.isfinite(v).all() or abs(np.linalg.norm(v[3:])-1) > 1e-5:
        raise ValueError('finite unit xyzw pose required')
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = Rotation.from_quat(v[3:]).as_matrix(), v[:3]
    return t


def camera_points(uv, depth, k):
    uv, k = np.asarray(uv, float), np.asarray(k, float)
    d = np.asarray(depth, float).reshape(-1)
    if (uv.shape != (len(d), 2) or not np.isfinite(uv).all()
            or not np.isfinite(d).all() or np.any(d <= 0)
            or k.shape != (3, 3) or not np.isfinite(k).all()
            or min(k[0, 0], k[1, 1]) <= 0 or k[0, 1] != 0 or k[1, 0] != 0
            or not np.array_equal(k[2], [0, 0, 1])):
        raise ValueError('finite positive depth and canonical zero-skew intrinsics required')
    return np.column_stack((d, (uv[:, 0]-k[0, 2])*d/k[0, 0],
                            (uv[:, 1]-k[1, 2])*d/k[1, 1]))


def json_safe(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


def replay_capture(capture, output):
    """Label-free local smoke, includes all native adjacent and terminal slots."""
    capture, output = Path(capture), Path(output)
    if output.exists():
        raise FileExistsError(output)
    paths = [capture/'tensor-map.npz', capture/'arbiter-events/events.jsonl']
    pins = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    with np.load(paths[0], allow_pickle=False) as z:
        saved = {k: z[k] for k in z.files}
    events = [json.loads(x) for x in paths[1].read_text().splitlines()]
    writes = {}
    for e in events:
        if e['kind'] == 'motion_result_written' and e['first_write_for_submission']:
            key = (e['from_idx'], e['frame_idx'])
            if key in writes:
                raise ValueError('ambiguous first motion write')
            writes[key] = e
    rows = []
    times = saved['frames//time_ns'].reshape(-1)
    for parent in range(len(times)-1):
        target = parent+1
        mask = ((saved['edge/match2frame1/mapping'] == parent)
                & (saved['edge/match2frame2/mapping'] == target))
        e = writes.get((parent, target))
        row = {'from_idx': parent, 'frame_idx': target, 'points': int(mask.sum()),
               'source_time_ns': int(times[parent]), 'target_time_ns': int(times[target])}
        if e is None:
            row['status'] = 'no_native_first_write'
        else:
            row['native_write_stage'] = e['stage']
            row['native_event_sequence'] = e['sequence']
            if (e['time0_ns'] != int(times[parent]) or e['time1_ns'] != int(times[target])
                    or e['original_need_interp']):
                raise ValueError('motion event not bound to the native supported interval')
            initial = pose_xyzw(e['derived_camera_delta_xyzw'])
            cs, ct = saved['match//obs1_covTc'][mask], saved['match//obs2_covTc'][mask]
            row['endpoint_covariance_max_asymmetry'] = float(max(
                abs(cs-cs.transpose(0, 2, 1)).max(initial=0),
                abs(ct-ct.transpose(0, 2, 1)).max(initial=0)))
            try:
                p = camera_points(saved['match//pixel1_uv'][mask], saved['match//pixel1_d'][mask],
                                  saved['frames//K'][parent])
                q = camera_points(saved['match//pixel2_uv'][mask], saved['match//pixel2_d'][mask],
                                  saved['frames//K'][target])
                result = register(p, q, cs, ct, initial)
                row.update(result)
                row['native_transform'] = initial
                row['initialization_source'] = 'recorded_first_committed_relative_camera_pose_before_refinement'
            except ValueError as error:
                row.update(status='invalid_input', reason=str(error))
        rows.append(row)
    assert all(hashlib.sha256(p.read_bytes()).hexdigest() == pins[str(p)] for p in paths)
    report = {'created_at': datetime.now(ZoneInfo('America/New_York')).isoformat(),
              'input_sha256': pins, 'rows': rows, 'reference_labels_read': False,
              'frontend_inference_calls': 0, 'scope': 'camera_local_development_solver_smoke_no_accuracy_or_fusion_claim',
              'status_counts': {s: sum(r['status'] == s for r in rows) for s in sorted({r['status'] for r in rows})}}
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as f:
        json.dump(json_safe(report), f, indent=2, allow_nan=False)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(replay_capture(args.capture, args.output)['status_counts']))
