"""Common-pose, pre-Huber precision/leverage diagnostic; no VO optimization."""

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


def projection(points, k, baseline):
    x, y, z = np.asarray(points, dtype=float).T
    if not np.isfinite(points).all() or np.any(x <= 0):
        raise ValueError('Finite points in front of the camera are required')
    return np.column_stack((k[0, 0]*y/x+k[0, 2],
                            k[1, 1]*z/x+k[1, 2], k[0, 0]*baseline/x))


def projection_jacobian(points, k, baseline):
    """Right local camera-pose tangent: translation meters, rotation radians."""
    projection(points, k, baseline)
    x, y, z = np.asarray(points, dtype=float).T
    j = np.zeros((len(x), 3, 3))
    j[:, 0, 0], j[:, 0, 1] = -k[0, 0]*y/x**2, k[0, 0]/x
    j[:, 1, 0], j[:, 1, 2] = -k[1, 1]*z/x**2, k[1, 1]/x
    j[:, 2, 0] = -k[0, 0]*baseline/x**2
    skew = np.zeros_like(j)
    skew[:, 0, 1], skew[:, 0, 2] = -z, y
    skew[:, 1, 0], skew[:, 1, 2] = z, -x
    skew[:, 2, 0], skew[:, 2, 1] = -y, x
    return j @ np.concatenate((-np.broadcast_to(np.eye(3), j.shape), skew), axis=2)


def group_diagnostics(jacobian, residual, variances, groups):
    """OLS linearizations before robustification; groups may overlap across axes."""
    j, r, v = map(lambda a: np.asarray(a, dtype=float), (jacobian, residual, variances))
    if (j.ndim != 3 or j.shape[1:] != (3, 6) or r.shape != j.shape[:2]
            or v.shape != r.shape or len(j) == 0
            or not all(np.isfinite(a).all() for a in (j, r, v)) or np.any(v <= 0)):
        raise ValueError('Expected finite Nx3x6 Jacobians, Nx3 residuals and positive variances')
    a = j / np.sqrt(v)[..., None]
    b = r / np.sqrt(v)
    flat = a.reshape(-1, 6)
    full_rank = np.linalg.matrix_rank(flat) == 6
    pinv = np.linalg.pinv(flat)
    step = -pinv @ b.ravel()
    leverage = np.einsum('ij,ji->i', flat, pinv).reshape(-1, 3).sum(axis=1)
    precision = 1/v[:, :2]
    result = {'rank': int(np.linalg.matrix_rank(flat)),
              'leverage_sum': float(leverage.sum()), 'groups': {}}
    for name, mask in groups.items():
        mask = np.asarray(mask, dtype=bool)
        if mask.shape != (len(j),):
            raise ValueError('Group mask length mismatch')
        keep = a[~mask].reshape(-1, 6)
        remaining_rank = int(np.linalg.matrix_rank(keep)) if len(keep) else 0
        change = None
        if full_rank and remaining_rank == 6:
            deleted_step = -np.linalg.lstsq(keep, b[~mask].ravel(), rcond=None)[0]
            delta = deleted_step-step
            change = {'translation_m': float(np.linalg.norm(delta[:3])),
                      'rotation_deg': float(np.rad2deg(np.linalg.norm(delta[3:])))}
        result['groups'][name] = {
            'count': int(mask.sum()), 'count_share': float(mask.mean()),
            'pixel_precision_share_u_v': (precision[mask].sum(axis=0)/precision.sum(axis=0)).tolist(),
            'leverage_share': float(leverage[mask].sum()/leverage.sum()),
            'remaining_rank': remaining_rank, 'linearized_deletion_step_change': change}
    return result


def rotation_xyzw(q):
    if q.shape != (4,) or not np.isfinite(q).all() or not np.isclose(q@q, 1, atol=1e-5):
        raise ValueError('Invalid quaternion')
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def analyze(graph, constants, height, width):
    if height <= 0 or width <= 0:
        raise ValueError('Positive actual image dimensions are required')
    constants = np.asarray(constants, dtype=float)
    if constants.shape != (3,) or not np.isfinite(constants).all() or np.any(constants <= 0):
        raise ValueError('Expected three positive frozen variances')
    if graph['frames//need_interp'].any():
        raise ValueError('Interpolated frames are unsupported')
    poses = graph['frames//pose']
    first = graph['edge/match2frame1/mapping'].reshape(-1)
    second = graph['edge/match2frame2/mapping'].reshape(-1)
    ids = graph['edge/match2point/mapping'].reshape(-1)
    pairs = []
    for i in range(len(poses)-1):
        selected = (first == i) & (second == i+1)
        if not selected.any():
            raise ValueError('Missing consecutive pair')
        pose = poses[i+1].astype(float)
        points = (graph['points//pos_Tw'][ids[selected]]-pose[:3]) @ rotation_xyzw(pose[3:])
        uv = graph['match//pixel2_uv'][selected].astype(float)
        if not np.isfinite(uv).all() or np.any(uv < 0) or np.any(uv >= [width, height]):
            raise ValueError('Observations outside supplied image dimensions')
        cov = graph['match//pixel2_uv_cov'][selected]
        if np.any(cov[:, 2] != 0):
            raise ValueError('Expected zero uv cross-covariance')
        v = np.column_stack((cov[:, :2], graph['match//pixel2_disp_cov'][selected].reshape(-1)))
        k = graph['frames//K'][i+1].astype(float)
        baseline = float(graph['frames//baseline'][i+1].reshape(-1)[0])
        if not np.isfinite(k).all() or not np.isfinite(baseline) or baseline <= 0:
            raise ValueError('Invalid camera geometry')
        target = np.column_stack((uv, graph['match//pixel2_disp'][selected].reshape(-1)))
        residual = projection(points, k, baseline)-target
        jac = projection_jacobian(points, k, baseline)
        groups = {}
        for axis, coordinate, extent in [('row', uv[:, 1], height), ('column', uv[:, 0], width)]:
            bins = np.floor(3*coordinate/extent).astype(int)
            groups.update({f'{axis}_third_{n}': bins == n for n in range(3)})
        edges = [0, 10, 20, 50, 100, np.inf]
        for low, high in zip(edges[:-1], edges[1:]):
            groups[f'forward_depth_m_{low}_{high}'] = (points[:, 0] >= low) & (points[:, 0] < high)
        fixed = v.copy()
        fixed[:, :2] = constants[:2]
        pairs.append({'source_frame': i, 'target_frame': i+1,
                      'target_time_ns': int(graph['frames//time_ns'][i+1].reshape(-1)[0]),
                      'learned': group_diagnostics(jac, residual, v, groups),
                      'fixed_uv': group_diagnostics(jac, residual, fixed, groups)})
    return pairs


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True, help='EXP-006 source root')
    parser.add_argument('--plan', type=Path, required=True, help='Frozen EXP-009 plan.json')
    parser.add_argument('--window', choices=['early', 'original', 'late'], required=True)
    parser.add_argument('--modality', choices=['rgb', 'thermal'], required=True)
    parser.add_argument('--height', type=int, required=True)
    parser.add_argument('--width', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    plan = json.loads(args.plan.read_text())
    relative = Path(args.window)/args.modality/'tensor-map.npz'
    source = args.source/relative
    source_hash = sha(source)
    if plan['source_inputs_sha256'][relative.as_posix()] != source_hash:
        raise ValueError('Source does not match frozen EXP-009 plan')
    with np.load(source, allow_pickle=False) as graph:
        pairs = analyze(graph, plan['constants'][args.modality]['variance_u_v_disp_px2'], args.height, args.width)
    if sha(source) != source_hash:
        raise ValueError('Source changed during analysis')
    result = {'created_at_utc': datetime.now(timezone.utc).isoformat(),
              'scope': 'Common original fitted poses; pre-Huber linearization only. Not native solver influence, a new trajectory, or calibrated confidence.',
              'coordinates': 'Right local camera tangent; translation meters, rotation radians; forward depth is NED x, not Euclidean range.',
              'deletion': 'Difference of full and group-deleted OLS steps at same pose; null if either Jacobian lacks rank six. No damping, robustification or reoptimization.',
              'leverage': 'Trace of per-observation blocks of whitened J @ pinv(J); depends on full-rank convention and excludes robustification.',
              'window': args.window, 'modality': args.modality,
              'height': args.height, 'width': args.width, 'numpy': np.__version__,
              'source_sha256': source_hash, 'plan_sha256': sha(args.plan),
              'script_sha256': sha(Path(__file__)), 'pairs': pairs}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)


if __name__ == '__main__':
    main()
