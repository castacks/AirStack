"""Reference-free nominal ray uncertainty and independent temporal image consistency."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import cv2
import numpy as np
from .td3_low_motion import lk_track, track_masks
from .td3_tracking_uncertainty import sha, percentiles, rank_association
from .td3_night_geometry import verify


def rays(uv, k):
    r = np.column_stack((uv, np.ones(len(uv)))) @ np.linalg.inv(np.asarray(k).reshape(3, 3)).T
    return r / np.linalg.norm(r, axis=1)[:, None]


def ray_jacobian(uv, k):
    inverse = np.linalg.inv(np.asarray(k).reshape(3, 3))
    r = np.column_stack((uv, np.ones(len(uv)))) @ inverse.T
    length = np.linalg.norm(r, axis=1)
    n = r / length[:, None]
    return ((np.eye(3)[None] - n[:, :, None]*n[:, None, :]) @ inverse[:, :2]) / length[:, None, None]


def angular_variance(uv, covariance, k):
    """Return rad² trace, PSD validity; no variance flooring or cross-term deletion."""
    c = np.asarray(covariance, float)
    valid = np.isfinite(c).all(axis=1) & (c[:, 0] >= 0) & (c[:, 1] >= 0) & (c[:, 2]**2 <= c[:, 0]*c[:, 1])
    j = ray_jacobian(uv, k)
    value = (j[:, :, 0]**2).sum(1)*c[:, 0] + (j[:, :, 1]**2).sum(1)*c[:, 1] + 2*(j[:, :, 0]*j[:, :, 1]).sum(1)*c[:, 2]
    valid &= np.isfinite(value) & (value >= 0)
    return np.where(valid, value, np.nan), valid


def angle(first, second, k):
    a, b = rays(first, k), rays(second, k)
    return np.arctan2(np.linalg.norm(np.cross(a, b), axis=1), (a*b).sum(1))


def prepare(root, output):
    output.mkdir(parents=True, exist_ok=False)
    source = root/'results/stereo-image-consistency/run-002/plan.json'
    cases, files = [], {str(source): sha(source)}
    for inherited in json.loads(source.read_text())['cases']:
        report = Path(inherited['report']); directory = report.parent
        rows = json.loads(report.read_text())['input_rows'][:20]
        prepared = root/f"results/{inherited['group']}/prepared-001/plan.json"
        original = next(c for c in json.loads(prepared.read_text())['cases'] if c['case'] == inherited['window'])
        images = [str(Path(original['dataset'])/r[inherited['modality']+'_left']['path']) for r in rows]
        pairs = [str(directory/'uncertainty'/f'pair-{i:03d}.npz') for i in range(19)]
        assert len(rows) == 20
        cases.append(dict(case=inherited['case'], modality=inherited['modality'], rows=rows, images=images, pairs=pairs))
        for p in [report, prepared, *map(Path, images), *map(Path, pairs)]: files[str(p)] = sha(p)
    for name in ['td3_temporal_angular.py', 'td3_low_motion.py', 'td3_tracking_uncertainty.py', 'td3_night_geometry.py']:
        p = Path(__file__).with_name(name); files[str(p)] = sha(p)
    p = Path(__file__).parents[1]/'UnitTest/test_td3_temporal_angular.py'; files[str(p)] = sha(p)
    plan = dict(created_at_utc=datetime.now(timezone.utc).isoformat(), base_commit='480f2d9bf04b7c3cb8750f0a563f618ea7364ada',
                cases=cases, files_sha256=files, protocol=dict(pairs_per_case=19, lk='EXP-027 unchanged',
                populations=['all', 'lk_valid', 'lk_tight', 'lk_texture'], covariance='raw temporal uu,vv,uv before floors',
                angular='target-ray Jacobian covariance trace rad²; exact model-LK angle rad',
                statistics='pair quantiles/spatial rho; window medians and chronological rho of pair medians',
                scope='Nominal camera model; independent algorithm disagreement, not physical error or calibration'))
    verify(files); (output/'plan.json').write_text(json.dumps(plan, indent=2)+'\n')


def median_stat(stat):
    return stat['p10_p50_p90'][1] if stat['finite_count'] else None


def run(output):
    plan = json.loads((output/'plan.json').read_text()); verify(plan['files_sha256'])
    cv2.setNumThreads(1); cv2.setRNGSeed(0)
    records, summaries = [], []
    metrics = ['variance_rad2', 'variance_px2', 'disagreement_rad', 'disagreement_px', 'model_motion_rad', 'lk_motion_rad', 'source_std']
    for case in plan['cases']:
        dest = output/'points'/case['case']; dest.mkdir(parents=True, exist_ok=False)
        local = []
        for i, path in enumerate(case['pairs']):
            images = [cv2.imread(case['images'][j], cv2.IMREAD_GRAYSCALE) for j in (i, i+1)]
            with np.load(path) as p:
                height, width = map(int, p['height_width']); k = p['K'].reshape(3, 3)
                assert all(im is not None and im.shape == (height, width) for im in images)
                assert int(p['time0_ns']) == int(case['rows'][i][case['modality']+'_left']['time_ns'])
                uv, model, covariance = p['grid_uv0'], p['grid_uv1'], p['grid_raw_flow_cov']
            end, back, status, reverse, err, rerr = lk_track(*images, uv)
            valid, tight, fb, base = track_masks(uv, end, back, status, reverse, width, height)
            first = images[0].astype(float); mean = cv2.boxFilter(first, -1, (21, 21))
            std = np.sqrt(np.maximum(cv2.boxFilter(first*first, -1, (21, 21))-mean*mean, 0))
            texture = std[uv[:, 1].astype(int), uv[:, 0].astype(int)]
            variance, psd = angular_variance(model, covariance, k)
            arrays = dict(uv0=uv, model_endpoint=model, lk_endpoint=end, lk_backward=back, covariance=covariance,
                K=k, covariance_valid=psd, lk_status=status, lk_reverse_status=reverse, fb_px=fb,
                variance_rad2=variance, variance_px2=np.where(psd, covariance[:, :2].sum(1), np.nan),
                disagreement_rad=angle(model, end, k), disagreement_px=np.linalg.norm(model-end, axis=1),
                model_motion_rad=angle(uv, model, k), lk_motion_rad=angle(uv, end, k), source_std=texture)
            masks = dict(all=np.ones(len(uv), bool), lk_valid=valid, lk_tight=tight, lk_texture=valid & (texture >= 5))
            np.savez_compressed(dest/f'{i:03d}.npz', **arrays, **{'mask_'+key: value for key, value in masks.items()})
            for population, mask in masks.items():
                row = dict(case=case['case'], modality=case['modality'], pair=i, population=population,
                    count=int(mask.sum()), grid_count=len(uv), covariance_invalid=int((mask & ~psd).sum()),
                    sigma_above_0_1rad=int((mask & (variance > .01)).sum()),
                    angular_rho=rank_association(variance[mask], arrays['disagreement_rad'][mask]),
                    pixel_rho=rank_association(arrays['variance_px2'][mask], arrays['disagreement_px'][mask]),
                    **{key: percentiles(arrays[key][mask]) for key in metrics})
                local.append(row); records.append(row)
        for population in masks:
            rows = [r for r in local if r['population'] == population]
            medians = {key: [median_stat(r[key]) for r in rows] for key in metrics}
            numeric = lambda values: np.array([np.nan if v is None else v for v in values])
            summary = dict(case=case['case'], modality=case['modality'], population=population, pairs=len(rows),
                support_min=min(r['count']/r['grid_count'] for r in rows), support_max=max(r['count']/r['grid_count'] for r in rows),
                covariance_invalid=sum(r['covariance_invalid'] for r in rows), sigma_above_0_1rad=sum(r['sigma_above_0_1rad'] for r in rows),
                spatial_angular_rho=median_stat(percentiles(numeric([r['angular_rho'] for r in rows]))),
                spatial_pixel_rho=median_stat(percentiles(numeric([r['pixel_rho'] for r in rows]))),
                temporal_angular_rho=rank_association(numeric(medians['variance_rad2']), numeric(medians['disagreement_rad'])),
                **{key: median_stat(percentiles(numeric(values))) for key, values in medians.items()})
            summaries.append(summary)
        print(case['case'], '19 pairs', flush=True)
    analysis = dict(created_at_utc=datetime.now(timezone.utc).isoformat(), opencv=cv2.__version__, cases=summaries,
                    pairs=records, verified_hashes=verify(plan['files_sha256']))
    (output/'analysis.json').write_text(json.dumps(analysis, indent=2, allow_nan=False)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['prepare', 'run']); parser.add_argument('--root', type=Path, default=Path('/workspace'))
    parser.add_argument('--output', type=Path, required=True); args = parser.parse_args()
    if args.action == 'prepare': prepare(args.root, args.output)
    else: run(args.output)
