"""Bounded raw-uint16 patch alignment; a mechanism prototype, not TP-TIO.

Only translation is fitted. No IMU, pyramid, learned features, flow covariance,
pose solver or calibrated uncertainty is supplied by this module.
"""
import numpy as np
from scipy.ndimage import map_coordinates
from scipy.optimize import least_squares

RADIUS = 7
MAX_CORRECTION_PX = 2.0
MAX_EVALUATIONS = 30
MIN_RAW_STD = 1.0
MIN_SINGULAR_RATIO = 1e-3


def refine(source, target, source_uv, initial_uv):
    """Return every attempt, including explicit failures; never substitute flow.

    Coordinates are x,y at native resolution. Input uint16 arrays are retained
    unchanged. A converged patch fit is not a visibility/correspondence label.
    """
    source, target = np.asarray(source), np.asarray(target)
    uv, initial = np.asarray(source_uv, float), np.asarray(initial_uv, float)
    if (source.dtype != np.uint16 or target.dtype != np.uint16
            or source.ndim != 2 or source.shape != target.shape):
        raise ValueError('equal-sized original uint16 grayscale images required')
    if uv.shape != (2,) or initial.shape != (2,):
        raise ValueError('one x,y source coordinate and initial endpoint required')
    result = {'status': 'INVALID_COORDINATES', 'endpoint_uv': None,
              'correction_px': None, 'initial_cost': None, 'final_cost': None,
              'nfev': 0, 'singular_ratio': None}
    if not np.isfinite(np.r_[uv, initial]).all():
        return result
    height, width = source.shape

    def inside(point, margin):
        return (point >= margin).all() and (point <= [width-1-margin, height-1-margin]).all()

    if not inside(uv, RADIUS) or not inside(initial, RADIUS+MAX_CORRECTION_PX):
        result['status'] = 'BORDER'
        return result
    gx, gy = np.meshgrid(np.arange(-RADIUS, RADIUS+1), np.arange(-RADIUS, RADIUS+1))
    offsets = np.array([gy.ravel(), gx.ravel()])

    def sample(image, point):
        return map_coordinates(image, offsets+point[::-1, None], order=1,
                               mode='constant', cval=np.nan, prefilter=False)

    # Interpolate float64 samples, not quantized uint16 interpolants.
    template = sample(source.astype(np.float64), uv)
    mean, std = template.mean(), template.std()
    if mean <= 0 or std < MIN_RAW_STD:
        result['status'] = 'LOW_TEXTURE'
        return result
    target_float = target.astype(np.float64)

    def residual(delta):
        patch = sample(target_float, initial+delta)
        patch_mean = patch.mean()
        if patch_mean <= 0:
            # A zero target patch is invalid; checked separately before solve.
            raise ValueError('nonpositive target patch mean')
        return (patch/patch_mean-template/mean)*(mean/std)

    try:
        first = residual(np.zeros(2))
        fit = least_squares(residual, np.zeros(2), bounds=(-MAX_CORRECTION_PX, MAX_CORRECTION_PX),
                            max_nfev=MAX_EVALUATIONS, ftol=1e-8, xtol=1e-8, gtol=1e-8)
    except ValueError:
        result['status'] = 'INVALID_TARGET_PATCH'
        return result
    singular = np.linalg.svd(fit.jac, compute_uv=False)
    ratio = float(singular[-1]/singular[0]) if singular[0] > 0 else 0.0
    result.update(initial_cost=float(first@first), final_cost=float(fit.fun@fit.fun),
                  nfev=int(fit.nfev), singular_ratio=ratio)
    if not fit.success:
        result['status'] = 'NOT_CONVERGED'
    elif np.any(fit.active_mask) or np.max(np.abs(fit.x)) >= MAX_CORRECTION_PX-1e-5:
        result['status'] = 'CORRECTION_LIMIT'
    elif ratio < MIN_SINGULAR_RATIO:
        result['status'] = 'APERTURE'
    elif not np.isfinite(fit.fun).all() or result['final_cost'] > result['initial_cost']+1e-10:
        result['status'] = 'INVALID_FIT'
    else:
        result.update(status='OK', endpoint_uv=(initial+fit.x).tolist(), correction_px=fit.x.tolist())
    return result
