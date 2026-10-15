"""Offline evaluation helpers. No runtime geometry or control authority."""
import numpy as np


def disparity_depth(disparity, focal_baseline, upper_disparity=None):
    """Positive left-minus-right pixels to axial meters; invalid stays NaN."""
    d = np.asarray(disparity, dtype=np.float64)
    if not np.isfinite(focal_baseline) or focal_baseline <= 0:
        raise ValueError('positive finite focal-baseline required')
    valid = np.isfinite(d) & (d > 0)
    if upper_disparity is not None:
        if not np.isfinite(upper_disparity) or upper_disparity <= 0:
            raise ValueError('invalid disparity ceiling')
        valid &= d < upper_disparity
    depth = np.full(d.shape, np.nan)
    with np.errstate(over='ignore', divide='ignore'):
        depth[valid] = focal_baseline / d[valid]
    depth[~np.isfinite(depth)] = np.nan
    return depth


def rectangle_mask(shape, rectangle):
    x, y, w, h = rectangle
    if min(x, y) < 0 or min(w, h) <= 0 or x+w > shape[1] or y+h > shape[0]:
        raise ValueError('rectangle outside image')
    mask = np.zeros(shape, dtype=bool)
    mask[y:y+h, x:x+w] = True
    return mask


def error_metrics(truth, prediction, area):
    t, p, a = np.asarray(truth), np.asarray(prediction), np.asarray(area)
    if t.shape != p.shape or t.shape != a.shape or a.dtype != bool:
        raise ValueError('matching arrays and boolean area required')
    teacher = a & np.isfinite(t) & (t > 0)
    supported = teacher & np.isfinite(p) & (p > 0)
    denominator, count = int(teacher.sum()), int(supported.sum())
    result = dict(teacher_pixels=denominator, compared_pixels=count,
                  coverage=count/denominator if denominator else None)
    if not count:
        return result
    error = p[supported].astype(float)-t[supported].astype(float)
    absolute = np.abs(error)
    good = (absolute <= .1) & (absolute/t[supported] <= .2)
    result.update(mae_m=float(absolute.mean()),
                  absolute_error_p50_p95_p99_m=np.quantile(absolute, [.5, .95, .99]).tolist(),
                  signed_bias_m=float(np.median(error)),
                  good_within_10cm_and_20percent_pixels=int(good.sum()),
                  good_fraction_of_all_teacher=float(good.sum()/denominator))
    return result


def comparison_metrics(truth, predictions, area):
    """Own support plus identical all-candidate intersection; neither hides outliers."""
    if not predictions:
        raise ValueError('at least one candidate required')
    common = np.asarray(area).copy()
    for prediction in predictions.values():
        if np.shape(prediction) != np.shape(truth):
            raise ValueError('candidate shape mismatch')
        common &= np.isfinite(prediction) & (prediction > 0)
    return {name: dict(own_support=error_metrics(truth, p, area),
                       all_candidate_common_support=error_metrics(truth, p, common))
            for name, p in predictions.items()}
