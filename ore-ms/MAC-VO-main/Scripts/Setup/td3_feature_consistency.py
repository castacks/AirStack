"""Retained-feature consistency against proxy motion; not uncertainty calibration."""

import argparse
import json
from pathlib import Path
import numpy as np
from scipy.stats import rankdata


def unproject_ned(uv, depth, intrinsics):
    uv, depth, intrinsics = np.asarray(uv), np.asarray(depth).reshape(-1), np.asarray(intrinsics)
    return np.stack([depth, (uv[:, 0] - intrinsics[:, 0, 2]) / intrinsics[:, 0, 0] * depth,
                     (uv[:, 1] - intrinsics[:, 1, 2]) / intrinsics[:, 1, 1] * depth], axis=1)


def consistency(points0, points1, covariance0, covariance1, relative):
    rotation, translation = relative[:, :3, :3], relative[:, :3, 3]
    a = points1 - np.einsum('nij,nj->ni', rotation, points0)
    residual = a - translation
    covariance = covariance1 + rotation @ covariance0 @ np.swapaxes(rotation, 1, 2)
    valid = np.isfinite(residual).all(axis=1) & np.isfinite(covariance).all(axis=(1, 2))
    valid &= np.isclose(covariance, covariance.transpose(0, 2, 1), rtol=1e-7, atol=1e-10).all(axis=(1, 2))
    indices = np.flatnonzero(valid)
    valid[indices] &= (np.linalg.eigvalsh(covariance[indices]) > 0).all(axis=1)
    normalized_squared = np.full(len(residual), np.nan)
    factor = np.linalg.cholesky(covariance[valid])
    white = np.linalg.solve(factor, residual[valid, :, None])[:, :, 0]
    normalized_squared[valid] = np.sum(white ** 2, axis=1)
    finite = np.isfinite(a).all(axis=1) & np.isfinite(translation).all(axis=1)
    denominator = np.sum(a[finite] ** 2)
    scale = float(np.sum(a[finite] * translation[finite]) / denominator) if denominator > 0 else None
    scaled = scale * a - translation if scale is not None else np.full_like(residual, np.nan)
    return {"residual": residual, "covariance": covariance, "valid": valid,
            "mahalanobis_squared_approx": normalized_squared,
            "fitted_depth_scale_in_sample": scale, "scaled_residual": scaled}


def rank_association(a, b, control=None):
    a, b = rankdata(a), rankdata(b)
    if control is not None:
        design = np.column_stack([np.ones(len(a)), rankdata(control)])
        a = a - design @ np.linalg.lstsq(design, a, rcond=None)[0]
        b = b - design @ np.linalg.lstsq(design, b, rcond=None)[0]
    if len(a) < 3 or np.std(a) < 1e-10 or np.std(b) < 1e-10:
        return None
    return float(np.corrcoef(a, b)[0, 1])


def fit_trajectory_scale(estimated, reference):
    e = np.linalg.inv(estimated[0]) @ estimated
    r = np.linalg.inv(reference[0]) @ reference
    x, y = e[:, :3, 3], r[:, :3, 3]
    denominator = np.sum(x ** 2)
    scale = float(np.sum(x * y) / denominator) if denominator > 0 else None
    split = len(x) // 2
    train_denominator = np.sum(x[:split] ** 2)
    train_scale = float(np.sum(x[:split] * y[:split]) / train_denominator) if split >= 2 and train_denominator > 0 else None
    return {"scope": "primary all-frame fit is in-sample; sequential split reported separately; no corrections applied",
            "fitted_scale": scale,
            "raw_translation_rmse_m": float(np.sqrt(np.mean(np.sum((x-y)**2, axis=1)))),
            "fitted_translation_rmse_m": float(np.sqrt(np.mean(np.sum((scale*x-y)**2, axis=1)))) if scale is not None else None,
            "sequential_split_diagnostic": {
                "scope": "fit first half, evaluate second half of same selected segment; temporally dependent, not independent validation",
                "fit_frames": split, "evaluation_frames": len(x) - split,
                "first_half_fit_scale": train_scale,
                "second_half_raw_translation_rmse_m": float(np.sqrt(np.mean(np.sum((x[split:]-y[split:])**2, axis=1)))),
                "second_half_scaled_translation_rmse_m": float(np.sqrt(np.mean(np.sum((train_scale*x[split:]-y[split:])**2, axis=1)))) if train_scale is not None else None}}


def depth_scale_fit(points0, points1, relative, mask):
    a = points1 - np.einsum('nij,nj->ni', relative[:, :3, :3], points0)
    t = relative[:, :3, 3]
    denominator = np.sum(a[mask] ** 2)
    scale = float(np.sum(a[mask] * t[mask]) / denominator) if denominator > 0 else None
    return {"unconstrained_scale": scale, "physically_positive": scale is not None and scale > 0,
            "fit_status": "in-sample least squares; near-zero/negative fits are not physical calibration",
            "fitted_residual_rms_m": float(np.sqrt(np.mean(np.sum((scale*a[mask]-t[mask])**2, axis=1)))) if scale is not None else None}


def analyze(directory, output, max_depths=(10,20,50,100)):
    directory, output = Path(directory), Path(output)
    with np.load(directory / "tensor-map.npz", allow_pickle=False) as graph, np.load(
            directory / "proxy-reference.npz", allow_pickle=False) as proxy:
        count = len(graph['match//pixel1_uv'])
        first = graph['edge/match2frame1/mapping']; second = graph['edge/match2frame2/mapping']
        times = graph['frames//time_ns'].reshape(-1)
        world = proxy['T_world_camera_ned']
        if len(first) != count or len(second) != count or not np.array_equal(times, proxy['time_ns'].reshape(-1)):
            raise ValueError("Graph observation/reference association mismatch")
        if np.any(first < 0) or np.any(second >= len(world)) or np.any(second <= first):
            raise ValueError("Invalid or reversed observation endpoints")
        k = graph['frames//K']
        depth0, depth1 = graph['match//pixel1_d'].ravel(), graph['match//pixel2_d'].ravel()
        points0 = unproject_ned(graph['match//pixel1_uv'], depth0, k[first])
        points1 = unproject_ned(graph['match//pixel2_uv'], depth1, k[second])
        relative = np.linalg.inv(world[second]) @ world[first]
        result = consistency(points0, points1, graph['match//obs1_covTc'], graph['match//obs2_covTc'], relative)
        error = np.linalg.norm(result['residual'], axis=1)
        trace = np.trace(result['covariance'], axis1=1, axis2=2)
        depth = .5 * (depth0 + depth1)
        valid = result['valid'] & np.isfinite(depth) & (depth0 > 0) & (depth1 > 0)
        if not valid.any():
            raise ValueError("No finite positive-depth observations with positive-definite covariance")
        def summary(mask):
            if not mask.any(): return {"count": 0}
            return {"count": int(mask.sum()), "median_consistency_residual_m": float(np.median(error[mask])),
                    "median_predicted_trace_m2": float(np.median(trace[mask])),
                    "median_depth_m": float(np.median(depth[mask])),
                    "trace_vs_residual_spearman": rank_association(trace[mask], error[mask]),
                    "trace_vs_residual_depth_partial_rank": rank_association(trace[mask], error[mask], depth[mask])}
        groups = []
        for f0, f1 in np.unique(np.stack([first, second], axis=1), axis=0):
            mask = valid & (first == f0) & (second == f1)
            groups.append({"frame0": int(f0), "frame1": int(f1), **summary(mask)})
        bins = []
        for lo, hi in zip([0,5,10,20,50], [5,10,20,50,np.inf]):
            bins.append({"depth_min_m": lo, "depth_max_m": hi if np.isfinite(hi) else None,
                         **summary(valid & (depth >= lo) & (depth < hi))})
        sensitivity = []
        for limit in max_depths:
            mask = valid & (depth0 <= limit) & (depth1 <= limit)
            association = []
            for f0, f1 in np.unique(np.stack([first, second], axis=1), axis=0):
                select = mask & (first == f0) & (second == f1)
                if select.sum() >= 3:
                    association.append(summary(select))
            correlations = [v['trace_vs_residual_spearman'] for v in association if v['trace_vs_residual_spearman'] is not None]
            partials = [v['trace_vs_residual_depth_partial_rank'] for v in association if v['trace_vs_residual_depth_partial_rank'] is not None]
            sensitivity.append({"both_endpoint_depths_max_m": limit, **summary(mask),
                                "raw_residual_rms_m": float(np.sqrt(np.mean(error[mask]**2))) if mask.any() else None,
                                "median_within_pair_spearman": float(np.median(correlations)) if correlations else None,
                                "median_within_pair_depth_partial_rank": float(np.median(partials)) if partials else None,
                                "depth_scale_fit": depth_scale_fit(points0, points1, relative, mask)})
        scaled_norm = np.linalg.norm(result['scaled_residual'], axis=1)
        report = {"scope": "retained static-scene feature consistency against recorded SLAM proxy; not GT or calibration",
                  "residual_definition": "p1 - (R_proxy_1_from_0 p0 + t_proxy_1_from_0), camera1 NED meters",
                  "covariance_approximation": "C1 + R C0 R.T; ignores cross-correlations, reference/calibration/time uncertainty",
                  "selection": "upstream covariance-aware keypoints and retained outlier-filtered matches only",
                  "observations": count, "valid_observations": int(valid.sum()), "summary": summary(valid),
                  "range_sensitivity_scope": "exploratory after observing extreme depth tails; originals and primary all-retained result unchanged",
                  "endpoint_depth_percentiles_m": np.percentile(np.maximum(depth0[valid],depth1[valid]),[50,90,95,99,100]).tolist(),
                  "depth_range_sensitivity": sensitivity,
                  "median_approx_mahalanobis_squared": float(np.median(result['mahalanobis_squared_approx'][valid])),
                  "feature_depth_scale_sensitivity": {
                      "scope": "one unweighted in-sample scalar on both endpoint depths; no covariance rescaling or calibration claim",
                      "scale": result['fitted_depth_scale_in_sample'],
                      "physically_positive": result['fitted_depth_scale_in_sample'] is not None and result['fitted_depth_scale_in_sample'] > 0,
                      "fit_status": ("near-zero/negative fit; not usable calibration" if result['fitted_depth_scale_in_sample'] is None or result['fitted_depth_scale_in_sample'] <= 1e-3 else "exploratory in-sample fit, not calibration"),
                      "raw_residual_rms_m": float(np.sqrt(np.mean(error[valid]**2))),
                      "fitted_residual_rms_m": float(np.sqrt(np.mean(scaled_norm[valid]**2)))},
                  "frame_pairs": groups, "depth_bins": bins,
                  "trajectory_scale_sensitivity": fit_trajectory_scale(
                      matrices_from_graph(graph['frames//pose']), world)}
        effective = json.loads((directory / 'effective-config.json').read_text())
        floor = effective['Odometry']['cov']['obs']['args']['min_depth_cov']
        report['covariance_construction'] = {
            'meaning': 'propagated observation covariance including pixel priors, local depth variance and floors; not raw learned flow uncertainty',
            'configured_min_depth_variance_m2': floor,
            'frame0_at_depth_variance_floor_fraction': float(np.isclose(graph['match//obs1_covTc'][:,0,0],floor,rtol=1e-6).mean()),
            'frame1_at_depth_variance_floor_fraction': float(np.isclose(graph['match//obs2_covTc'][:,0,0],floor,rtol=1e-6).mean())}
        output.mkdir(parents=True, exist_ok=False)
        np.savez_compressed(output / 'observations.npz', frame0=first, frame1=second, time0_ns=times[first],
                            time1_ns=times[second], depth_mean_m=depth, residual=result['residual'],
                            covariance_approx=result['covariance'], predicted_trace_m2=trace,
                            mahalanobis_squared_approx=result['mahalanobis_squared_approx'], valid=valid,
                            scaled_residual_diagnostic=result['scaled_residual'])
        (output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        return report


def matrices_from_graph(poses):
    try:
        from .td3_reference import matrices
    except ImportError:
        from td3_reference import matrices
    return matrices(poses)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trajectory-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-depths-m', type=float, nargs='+', default=[10,20,50,100])
    args = parser.parse_args()
    if any(not np.isfinite(x) or x <= 0 for x in args.max_depths_m):
        parser.error('Depth sensitivity limits must be finite and positive')
    for modality in ['rgb', 'thermal']:
        report = analyze(args.trajectory_root / modality, args.output / modality, args.max_depths_m)
        print(json.dumps({'modality': modality, **{k:report[k] for k in [
            'observations','valid_observations','summary','feature_depth_scale_sensitivity','trajectory_scale_sensitivity']}},indent=2))
