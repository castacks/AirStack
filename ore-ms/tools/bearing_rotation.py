"""Equation-level NEC/PNEC prototype, not the authors' complete optimizer.

Convention: X_source = R @ X_target + t. Source pixels are conditioned on;
target flow covariance is pushed to unit bearings with a first-order Jacobian.
Unlike the paper this uses rank-two linear propagation, an eight-point start,
and SciPy joint least squares, not unscented propagation and alternating SCF.
"""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

VARIANCE_REGULARIZER = 1e-12
MAX_EVALUATIONS = 200
TOLERANCE = 1e-10
MIN_PARALLAX_RAD = 1e-4


def bearings(uv, intrinsics, covariance=None):
    uv, k = np.asarray(uv, float), np.asarray(intrinsics, float)
    if (uv.ndim != 2 or uv.shape[1] != 2 or k.shape != (3, 3)
            or not np.isfinite(uv).all() or not np.isfinite(k).all()
            or not np.allclose(k[2], [0, 0, 1], atol=1e-12, rtol=0)
            or k[0, 0] <= 0 or k[1, 1] <= 0):
        raise ValueError('finite pinhole coordinates and positive focal lengths required')
    inverse = np.linalg.inv(k)
    q = np.column_stack([uv, np.ones(len(uv))])@inverse.T
    norms = np.linalg.norm(q, axis=1)
    f = q/norms[:, None]
    jacobian = ((np.eye(3)[None]-f[:, :, None]*f[:, None, :])
                @inverse[:, :2])/norms[:, None, None]
    if covariance is None:
        return f, jacobian, None
    c = np.asarray(covariance, float)
    if (c.shape != (len(uv), 2, 2) or not np.isfinite(c).all()
            or not np.allclose(c, c.swapaxes(1, 2), atol=1e-12, rtol=0)
            or np.any(np.linalg.eigvalsh(c) <= 0)):
        raise ValueError('strict SPD pixel covariance required; no flooring')
    pushed = jacobian@c@jacobian.swapaxes(1, 2)
    return f, jacobian, pushed


def residuals(source, target, rotation, translation, covariance=None,
              regularizer=VARIANCE_REGULARIZER):
    f, g, r, t = map(lambda x: np.asarray(x, float),
                     [source, target, rotation, translation])
    if (f.ndim != 2 or f.shape[1] != 3 or g.shape != f.shape
            or r.shape != (3, 3) or t.shape != (3,)
            or not all(np.isfinite(x).all() for x in (f, g, r, t))
            or not np.allclose(np.linalg.norm(f, axis=1), 1)
            or not np.allclose(np.linalg.norm(g, axis=1), 1)
            or not np.allclose(r.T@r, np.eye(3), atol=1e-8)
            or abs(np.linalg.det(r)-1) > 1e-8 or abs(np.linalg.norm(t)-1) > 1e-8):
        raise ValueError('unit rays, proper rotation and unit translation required')
    numerator = np.cross(f, g@r.T)@t
    if covariance is None:
        return numerator
    if regularizer <= 0 or not np.isfinite(regularizer):
        raise ValueError('positive variance regularizer required')
    c = np.asarray(covariance, float)
    if (c.shape != (len(f), 3, 3) or not np.isfinite(c).all()
            or not np.allclose(c, c.swapaxes(1, 2), atol=1e-12, rtol=0)
            or np.any(np.linalg.eigvalsh(c) < -1e-12)):
        raise ValueError('finite PSD bearing covariance required')
    # t dot (f cross R*g) = (R.T*(t cross f)) dot g.
    a = np.cross(t, f)@r
    variance = np.einsum('ni,nij,nj->n', a, c, a)
    if np.any(variance < -1e-12):
        raise ValueError('negative propagated variance')
    return numerator/np.sqrt(variance+regularizer)


def initialize(source, target):
    """Reference-free eight-point initialization and positive-depth disambiguation."""
    f, g = np.asarray(source), np.asarray(target)
    if len(f) < 8:
        raise ValueError('eight correspondences required')
    design = np.einsum('ni,nj->nij', f, g).reshape(-1, 9)
    # With exactly 8 rows, reduced SVD omits the ninth (nullspace) right vector.
    _, singular, vh = np.linalg.svd(design, full_matrices=len(f) < 9)
    if singular[0] == 0 or singular[7]/singular[0] < 1e-8:
        return None
    u, _, vt = np.linalg.svd(vh[-1].reshape(3, 3))
    if np.linalg.det(u) < 0:
        u[:, -1] *= -1
    if np.linalg.det(vt) < 0:
        vt[-1] *= -1
    w = np.array([[0., -1, 0], [1, 0, 0], [0, 0, 1]])
    candidates = []
    for rotation in (u@w@vt, u@w.T@vt):
        for translation in (u[:, 2], -u[:, 2]):
            count = 0
            for a, b in zip(f, g@rotation.T):
                depths = np.linalg.lstsq(np.column_stack([a, -b]), translation, rcond=None)[0]
                count += int(np.all(depths > 0))
            candidates.append((count, rotation, translation))
    best = max(candidates, key=lambda v: v[0])
    return best[1:]


def solve(source, target, covariance=None, max_evaluations=MAX_EVALUATIONS):
    if max_evaluations < 1:
        raise ValueError('positive evaluation budget required')
    start = initialize(source, target)
    if start is None:
        return {'status': 'DEGENERATE_INITIALIZATION', 'rotation': None, 'translation_direction': None}
    r0, t0 = start
    axis = np.eye(3)[np.argmin(np.abs(t0))]
    b0 = np.cross(t0, axis); b0 /= np.linalg.norm(b0)
    basis = np.column_stack([b0, np.cross(t0, b0)])
    def decode(x):
        t = t0+basis@x[3:]
        return Rotation.from_rotvec(x[:3]).as_matrix()@r0, t/np.linalg.norm(t)
    def function(x):
        r, t = decode(x)
        return residuals(source, target, r, t, covariance)
    initial_cost = float(np.dot(function(np.zeros(5)), function(np.zeros(5))))
    fit = least_squares(function, np.zeros(5), jac='3-point', method='trf',
                        ftol=TOLERANCE, xtol=TOLERANCE, gtol=TOLERANCE,
                        max_nfev=max_evaluations, loss='linear')
    rotation, translation = decode(fit.x)
    # atan2 is stable at small angles; no reference pose is used.
    rotated = np.asarray(target)@rotation.T
    parallax = np.arctan2(np.linalg.norm(np.cross(source, rotated), axis=1),
                         np.einsum('ni,ni->n', source, rotated))
    singular = np.linalg.svd(fit.jac, compute_uv=False)
    status = ('CONVERGED' if fit.success else 'BUDGET_EXHAUSTED')
    if np.median(parallax) < MIN_PARALLAX_RAD or singular[-1]/singular[0] < 1e-6:
        status = 'DEGENERATE_GEOMETRY'
    return {'status': status, 'rotation': rotation, 'translation_direction': translation,
            'initial_cost': initial_cost, 'final_cost': float(np.dot(fit.fun, fit.fun)),
            'evaluations': int(fit.nfev), 'termination': str(fit.message),
            'median_parallax_rad': float(np.median(parallax)),
            'jacobian_singular_values': singular}
