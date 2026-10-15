"""Evaluation-only comparison with axial simulator depth; never localization input."""


def assess_axial_depth(disparity, truth, calibration, *, region=None):
    import numpy as np

    d, t = np.asarray(disparity), np.asarray(truth)
    if d.shape != t.shape or d.shape != (calibration.height,calibration.width):
        raise ValueError('assessment image shape mismatch')
    x,y,w,h = calibration.valid_window
    if x < 0 or y < 0 or w <= 0 or h <= 0 or x+w > d.shape[1] or y+h > d.shape[0]:
        raise ValueError('assessment disparity window invalid')
    area = np.zeros(d.shape,dtype=bool)
    area[y:y+h,x:x+w] = True
    if region is not None:
        rx,ry,rw,rh = region
        if rx < 0 or ry < 0 or rw <= 0 or rh <= 0 or rx+rw > d.shape[1] or ry+rh > d.shape[0]:
            raise ValueError('assessment region invalid')
        selected = np.zeros(d.shape,dtype=bool)
        selected[ry:ry+rh,rx:rx+rw] = True
        area &= selected
    valid_truth = area & np.isfinite(t) & (t > 0)
    supported = area & np.isfinite(d) & (d > max(0,calibration.min_disparity)) & (d < calibration.max_disparity)
    common = supported & valid_truth
    result = dict(evidence_scope='EVALUATION_ONLY', geometry_authority=False,
                  region=region, area_pixels=int(area.sum()), teacher_pixels=int(valid_truth.sum()),
                  stereo_pixels=int(supported.sum()), compared_pixels=int(common.sum()),
                  teacher_supported_fraction=float(common.sum()/valid_truth.sum()) if valid_truth.any() else None)
    minimum_range=calibration.fx*calibration.baseline_m/calibration.max_disparity
    result['stereo_minimum_axial_range_m']=minimum_range
    result['all_teacher_depth_quantiles_m']=(np.quantile(t[valid_truth].astype(float),[.05,.5,.95]).tolist()
                                            if valid_truth.any() else None)
    result['teacher_below_stereo_minimum_fraction']=(float(np.mean(t[valid_truth]<minimum_range))
                                                    if valid_truth.any() else None)
    truth_in_range=valid_truth & (t >= minimum_range)
    result['within_range_teacher_supported_fraction']=(float(np.sum(common & truth_in_range)/truth_in_range.sum())
                                                       if truth_in_range.any() else None)
    if not common.any():
        return {**result,'status':'NO_COMPARABLE_PIXELS'}
    estimated = calibration.fx*calibration.baseline_m/d[common].astype(float)
    reference = t[common].astype(float)
    error = estimated-reference
    absolute = np.abs(error)
    relative = absolute/reference
    result.update(status='ASSESSED', signed_bias_m=float(np.median(error)),
                  mae_m=float(absolute.mean()),rmse_m=float(np.sqrt(np.mean(error**2))),
                  absolute_error_quantiles_m=np.quantile(absolute,[.5,.95,.99]).tolist(),
                  relative_error_quantiles=np.quantile(relative,[.5,.95,.99]).tolist(),
                  within_10cm_fraction=float(np.mean(absolute <= .1)),
                  within_5percent_fraction=float(np.mean(relative <= .05)),
                  estimated_depth_quantiles_m=np.quantile(estimated,[.05,.5,.95]).tolist(),
                  teacher_depth_quantiles_m=np.quantile(reference,[.05,.5,.95]).tolist())
    return result
