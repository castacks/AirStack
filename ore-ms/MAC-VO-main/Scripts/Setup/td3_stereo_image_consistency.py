"""Frozen-grid stereo correspondence consistency without a pose reference."""

import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path

import cv2
import numpy as np

from .td3_night_geometry import verify
from .td3_tracking_uncertainty import sha, percentiles, rank_association


PROTOCOL = dict(grid_px=16, border_px=32, patch_radius=5, horizontal_radius=128,
                vertical_radius=2, runner_up_horizontal_exclusion=3,
                cost_arithmetic='float32 mean-centered source patch and target search stripe',
                primary=dict(texture=5., score=.8, gap=.05),
                strict=dict(texture=10., score=.9, gap=.1), reciprocal_error_px=1.)


def grid(width, height):
    return np.array([(x, y) for y in range(32, height-32, 16)
                     for x in range(32, width-32, 16)], dtype=np.int32).reshape(-1, 2)


def ncc_peak(source, target, uv, horizontal_radius=128, vertical_radius=2):
    """Exhaustive signed search; the model supplies neither guesses nor masks."""
    x, y = map(int, uv)
    radius = 5
    height, width = source.shape
    if target.shape != source.shape or source.dtype != np.uint8 or target.dtype != np.uint8:
        raise ValueError('Equal native uint8 grayscale image shapes required')
    if horizontal_radius < 4 or vertical_radius < 1:
        raise ValueError('Search must support interior peaks and a runner-up')
    if not (radius <= x < width-radius and radius <= y < height-radius):
        raise ValueError('Source patch outside image')
    patch = source[y-radius:y+radius+1, x-radius:x+radius+1]
    source_std = float(patch.std())
    empty = dict(endpoint=[np.nan, np.nan], score=np.nan, runner_up=np.nan,
                 gap=np.nan, source_std=source_std, target_std=np.nan,
                 boundary=True, bounds=[0, 0, 0, 0])
    if source_std == 0:
        return empty
    xmin, xmax = max(radius, x-horizontal_radius), min(width-radius-1, x+horizontal_radius)
    ymin, ymax = max(radius, y-vertical_radius), min(height-radius-1, y+vertical_radius)
    stripe = target[ymin-radius:ymax+radius+1, xmin-radius:xmax+radius+1]
    # NCC is invariant to separate additive intensity offsets. Centering reduces
    # cancellation in OpenCV's float32 cross-product on high-DC thermal patches.
    centered_patch = patch.astype(np.float32)
    centered_patch -= centered_patch.mean()
    centered_stripe = stripe.astype(np.float32)
    centered_stripe -= centered_stripe.mean()
    scores = cv2.matchTemplate(centered_stripe, centered_patch, cv2.TM_CCOEFF_NORMED)
    if not np.isfinite(scores).all():
        return empty
    iy, ix = np.unravel_index(np.argmax(scores), scores.shape)
    peak = float(scores[iy, ix])
    alternatives = scores[:, np.abs(np.arange(scores.shape[1])-ix) > 3]
    runner = float(alternatives.max()) if alternatives.size else np.nan
    u, v = int(xmin+ix), int(ymin+iy)
    target_std = float(target[v-radius:v+radius+1, u-radius:u+radius+1].std())
    return dict(endpoint=[u, v], score=peak, runner_up=runner, gap=peak-runner,
                source_std=source_std, target_std=target_std,
                boundary=bool(u in (xmin, xmax) or v in (ymin, ymax)),
                bounds=[xmin, xmax, ymin, ymax])


def match_grid(first, second, uv, horizontal_radius=128, vertical_radius=2):
    fields = ['score', 'runner_up', 'gap', 'source_std', 'target_std', 'boundary', 'bounds', 'endpoint']
    forward, reverse = [], []
    for point in uv:
        f = ncc_peak(first, second, point, horizontal_radius, vertical_radius)
        if np.isfinite(f['endpoint']).all():
            r = ncc_peak(second, first, f['endpoint'], horizontal_radius, vertical_radius)
        else:
            r = dict(endpoint=[np.nan, np.nan], score=np.nan, runner_up=np.nan,
                     gap=np.nan, source_std=np.nan, target_std=np.nan,
                     boundary=True, bounds=[0, 0, 0, 0])
        forward.append(f);reverse.append(r)
    arrays = {'uv0': np.asarray(uv)}
    for label, values in [('forward', forward), ('reverse', reverse)]:
        for field in fields:
            arrays[label+'_'+field] = np.asarray([v[field] for v in values])
    arrays['reciprocal_error_px'] = np.linalg.norm(arrays['reverse_endpoint']-uv, axis=1)
    arrays['observed_disparity_px'] = uv[:, 0]-arrays['forward_endpoint'][:, 0]
    arrays['observed_vertical_flow_px'] = arrays['forward_endpoint'][:, 1]-uv[:, 1]
    return arrays


def acceptance(arrays, texture=5., score=.8, gap=.05):
    n = len(arrays['uv0'])
    keep = np.ones(n, dtype=bool)
    for label in ['forward', 'reverse']:
        keep &= np.isfinite(arrays[label+'_endpoint']).all(axis=1)
        keep &= arrays[label+'_source_std'] >= texture
        keep &= arrays[label+'_target_std'] >= texture
        keep &= arrays[label+'_score'] >= score
        keep &= arrays[label+'_gap'] >= gap
        keep &= ~arrays[label+'_boundary'].astype(bool)
    return keep & (arrays['reciprocal_error_px'] <= 1.)


def summarize(arrays, mask):
    disparity = arrays['observed_disparity_px']
    predicted = arrays['model_disparity_px']
    eligible = mask & (np.abs(disparity) > 1) & (np.abs(predicted) > 1)
    result = dict(count=int(mask.sum()), sign_eligible_count=int(eligible.sum()),
                  sign_agreement=float((np.sign(disparity[eligible]) == np.sign(predicted[eligible])).mean()) if eligible.any() else None)
    for key in ['observed_disparity_px', 'model_disparity_px', 'horizontal_error_px',
                'vertical_error_px', 'endpoint_error_px', 'raw_stereo_variance_u',
                'raw_stereo_variance_trace', 'forward_source_std', 'forward_score']:
        result[key] = percentiles(arrays[key][mask])
    result['variance_u_horizontal_error_rho'] = rank_association(arrays['raw_stereo_variance_u'][mask], arrays['horizontal_error_px'][mask])
    result['variance_trace_endpoint_error_rho'] = rank_association(arrays['raw_stereo_variance_trace'][mask], arrays['endpoint_error_px'][mask])
    return result


def prepare(root, output):
    paths = [Path(__file__), Path(__file__).parents[1]/'UnitTest/test_td3_stereo_image_consistency.py',
             Path(__file__).with_name('td3_tracking_uncertainty.py'), Path(__file__).with_name('td3_night_geometry.py')]
    cases = []
    for group, expected in [('msa-replay', 6), ('salon-so-day', 6), ('long-salon', 6)]:
        source = root/'results'/group
        source_plan = source/'prepared-001/plan.json'
        old = json.loads(source_plan.read_text());paths.append(source_plan)
        captures = sorted((source/'vo-001').glob('*/*/first-signed-stereo.npz'))
        if len(captures) != expected:
            raise ValueError('Missing inherited stereo captures: '+group)
        for capture in captures:
            window, modality = capture.parts[-3:-1]
            original = next(c for c in old['cases'] if c['case'] == window)
            dataset = Path(original['dataset'])
            report_path = capture.with_name('report.json')
            report = json.loads(report_path.read_text());row = report['input_rows'][0]
            if row != original['rows'][0]:
                raise ValueError('First captured image row differs from preparation')
            images = [dataset/row[modality+'_'+side]['path'] for side in ['left', 'right']]
            if abs(row[modality+'_left']['time_ns']-row[modality+'_right']['time_ns']) > 2_000_000:
                raise ValueError('Original stereo acquisition bound violated')
            cases.append(dict(case=group+'-'+window+'-'+modality, group=group, window=window,
                              modality=modality, images=list(map(str, images)), capture=str(capture),
                              report=str(report_path), first_depth=str(capture.with_name('first-depth.npz')),
                              input_row=row))
            paths += [*images, capture, report_path, capture.with_name('first-depth.npz')]
    files = {str(p): sha(p) for p in paths};verify(files)
    output.mkdir(parents=True, exist_ok=False)
    plan = dict(created_at_utc=datetime.now(timezone.utc).isoformat(), base_commit='1f1a5ba05',
                cases=cases, files_sha256=files, protocol=PROTOCOL,
                scope='Algorithmic stereo-image consistency; no pose reference, metric truth, calibration fit or new inference')
    (output/'plan.json').write_text(json.dumps(plan, indent=2)+'\n')


def run(output):
    plan = json.loads((output/'plan.json').read_text());verify(plan['files_sha256'])
    cv2.setNumThreads(1);cv2.setRNGSeed(0)
    records = [];(output/'arrays').mkdir(exist_ok=False)
    for case in plan['cases']:
        images = [cv2.imread(p, cv2.IMREAD_UNCHANGED) for p in case['images']]
        if any(im is None or im.dtype != np.uint8 for im in images):
            raise ValueError('Missing/non-uint8 inherited images')
        gray = [cv2.cvtColor(im, cv2.COLOR_BGR2GRAY) if im.ndim == 3 else im for im in images]
        if gray[0].shape != gray[1].shape:
            raise ValueError('Stereo shapes differ')
        height, width = gray[0].shape;uv = grid(width, height);x, y = uv.T
        with np.load(case['capture']) as capture, np.load(case['first_depth']) as saved:
            flow = capture['flow'];cov = capture['covariance'];depth = capture['depth']
            if flow.shape != (2, height, width) or cov.shape != flow.shape or not np.isfinite(flow).all() or not (np.isfinite(cov) & (cov > 0)).all():
                raise ValueError('Invalid saved stereo prediction')
            if not np.array_equal(depth, saved['depth']):
                raise ValueError('Captured/saved first depth differs')
            report = json.loads(Path(case['report']).read_text())
            factor = report['execution_baseline_m']*report['K'][0][0]
            expected = factor/np.abs(flow[0])
            np.testing.assert_allclose(expected, depth.reshape(height, width), rtol=1e-6, atol=0)
            data = match_grid(*gray, uv)
            sampled = flow[:, y, x].T.copy();variance = cov[:, y, x].T.copy()
            data.update(model_flow=sampled, model_disparity_px=-sampled[:, 0],
                        raw_stereo_variance_u=variance[:, 0], raw_stereo_variance_trace=variance.sum(axis=1),
                        saved_model_depth_m=depth.reshape(height, width)[y, x])
        observed = data['forward_endpoint']-uv
        data['horizontal_error_px'] = np.abs(sampled[:, 0]-observed[:, 0])
        data['vertical_error_px'] = np.abs(sampled[:, 1]-observed[:, 1])
        data['endpoint_error_px'] = np.linalg.norm(sampled-observed, axis=1)
        masks = dict(all_grid=np.ones(len(uv), bool), primary=acceptance(data),
                     strict=acceptance(data, **PROTOCOL['strict']))
        for i in range(3):
            masks['primary_image_third_'+str(i)] = masks['primary'] & (uv[:, 1] >= i*height/3) & (uv[:, 1] < (i+1)*height/3)
        np.savez_compressed(output/'arrays'/(case['case']+'.npz'), **data,
                            **{'mask_'+key: value for key, value in masks.items()})
        populations = {name: summarize(data, mask) for name, mask in masks.items()}
        reasons = dict(source_texture_below5=int((data['forward_source_std'] < 5).sum()),
                       forward_score_below08=int((data['forward_score'] < .8).sum()),
                       forward_gap_below005=int((data['forward_gap'] < .05).sum()),
                       forward_boundary=int(data['forward_boundary'].sum()),
                       reverse_boundary=int(data['reverse_boundary'].sum()),
                       reciprocal_error_above1=int((data['reciprocal_error_px'] > 1).sum()),
                       unavailable_forward=int((~np.isfinite(data['forward_score'])).sum()),
                       unavailable_reverse=int((~np.isfinite(data['reverse_score'])).sum()))
        records.append(dict(case=case['case'], group=case['group'], modality=case['modality'],
                            height=height, width=width, total_grid=len(uv), depth_reproduction='PASS',
                            populations=populations, overlapping_failure_counts=reasons))
        print(case['case'], populations['primary']['count'], '/', len(uv), 'primary', flush=True)
    result = dict(created_at_utc=datetime.now(timezone.utc).isoformat(), base_commit=plan['base_commit'],
                  opencv=cv2.__version__, numpy=np.__version__, cases=records,
                  frozen_hashes_verified=verify(plan['files_sha256']), scope=plan['scope'])
    (output/'analysis.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    rows = []
    for case in records:
        for name, summary in case['populations'].items():
            row = dict(case=case['case'], modality=case['modality'], population=name, total_grid=case['total_grid'])
            for key, value in summary.items():
                if isinstance(value, dict):
                    row[key+'_finite_count'] = value['finite_count']
                    row[key+'_median'] = value['p10_p50_p90'][1] if value['finite_count'] else None
                else:
                    row[key] = value
            rows.append(row)
    with (output/'summary.csv').open('w') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['prepare', 'run'])
    parser.add_argument('--root', type=Path, default=Path('/workspace'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.action == 'prepare':
        prepare(args.root, args.output)
    else:
        run(args.output)
