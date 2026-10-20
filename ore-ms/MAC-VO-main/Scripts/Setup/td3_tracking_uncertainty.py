"""Unchanged-night-baseline raw uncertainty and retention characterization."""

import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np
from scipy.stats import spearmanr

ROOT = Path(__file__).resolve().parents[2]
WINDOWS = ('early', 'middle', 'late')
MODALITIES = ('rgb', 'thermal')
FLOOR = .0625
QUALITY_CAP = 100.
MIN_MATCHES = 10


def sha(path):
    import hashlib
    return hashlib.sha256(path.read_bytes()).hexdigest()


def percentiles(values):
    values = np.asarray(values, dtype=float).reshape(-1)
    finite = values[np.isfinite(values)]
    return {'count': len(values), 'finite_count': len(finite),
            'p10_p50_p90': np.percentile(finite, [10, 50, 90]).tolist() if len(finite) else None}


def auc(scores, labels):
    """Contemporaneous descriptive AUROC, explicit ties; no threshold fitting."""
    scores, labels = np.asarray(scores, float), np.asarray(labels, bool)
    valid = np.isfinite(scores)
    positive, negative = scores[valid & labels], scores[valid & ~labels]
    if not len(positive) or not len(negative):
        return None
    delta = positive[:, None]-negative[None, :]
    return float(((delta > 0)+.5*(delta == 0)).mean())


def rank_association(first, second):
    a, b = np.asarray(first, float), np.asarray(second, float)
    valid = np.isfinite(a) & np.isfinite(b)
    if valid.sum() < 3 or np.ptp(a[valid]) == 0 or np.ptp(b[valid]) == 0:
        return None
    return float(spearmanr(a[valid], b[valid]).statistic)


def retained_mask(inbound, keep):
    inbound, keep = np.asarray(inbound, bool), np.asarray(keep, bool)
    if inbound.ndim != 1 or keep.shape != (int(inbound.sum()),):
        raise ValueError('Selected/inbound/outlier association mismatch')
    result = np.zeros(len(inbound), bool)
    result[inbound] = keep
    return result


def postprocessed_flags(loss):
    """Existing MotionInterpolate clears the first/last two motion flags."""
    flags = np.asarray(loss, bool).copy()
    flags[:2] = False
    flags[-2:] = False
    return flags


def population(uv, cov, depth0, depth1, inbound, height):
    uv, cov = np.asarray(uv), np.asarray(cov)
    depth0, depth1, inbound = np.asarray(depth0), np.asarray(depth1), np.asarray(inbound, bool)
    n = len(uv)
    if (uv.shape != (n, 2) or cov.shape != (n, 3) or depth0.shape != (n,)
            or depth1.shape != (n,) or inbound.shape != (n,) or height <= 0):
        raise ValueError('Population shape mismatch')
    if not np.isfinite(uv).all() or np.any(uv[:, 1] < 0) or np.any(uv[:, 1] >= height):
        raise ValueError('Invalid source image coordinates')
    trace, quality = cov[:, :2].sum(axis=1), cov[:, :2].sum(axis=1)-2*cov[:, 2]

    def summarize(mask):
        finite_q = np.isfinite(quality[mask])
        finite_var = np.isfinite(cov[mask, :2])
        finite_depth = np.isfinite(depth0[mask])
        return {'count': int(mask.sum()), 'inbound_count': int(inbound[mask].sum()),
                'raw_trace_px2': percentiles(trace[mask]), 'raw_quality_px2': percentiles(quality[mask]),
                'raw_uu_px2': percentiles(cov[mask, 0]), 'raw_vv_px2': percentiles(cov[mask, 1]),
                'grid_quality_below_native_cap_fraction': float((quality[mask][finite_q] < QUALITY_CAP).mean()) if finite_q.any() else None,
                'variance_components_below_floor_fraction': float((cov[mask, :2][finite_var] < FLOOR).mean()) if finite_var.any() else None,
                'depth0_m': percentiles(depth0[mask]), 'depth1_m': percentiles(depth1[mask]),
                'depth0_positive_finite_count': int((finite_depth & (depth0[mask] > 0)).sum())}
    result = summarize(np.ones(n, bool))
    result['image_thirds'] = {str(i): summarize((uv[:, 1] >= i*height/3) & (uv[:, 1] < (i+1)*height/3)) for i in range(3)}
    good_depth = np.isfinite(depth0) & (depth0 > 0)
    edges = [0, 10, 20, 50, 100, np.inf]
    depth_groups = {f'{lo}_{hi}': good_depth & (depth0 >= lo) & (depth0 < hi) for lo, hi in zip(edges[:-1], edges[1:])}
    depth_groups['invalid'] = ~good_depth
    result['source_forward_depth_bins_m'] = {k: summarize(mask) for k, mask in depth_groups.items()}
    if sum(v['count'] for v in result['image_thirds'].values()) != n or sum(v['count'] for v in result['source_forward_depth_bins_m'].values()) != n:
        raise ValueError('Population partition failed')
    return result


def summarize_pair(saved):
    height = int(saved['height_width'][0])
    inbound = saved['selected_inbound']
    retained = retained_mask(inbound, saved['outlier_keep_inbound'])
    result = {'time0_ns': int(saved['time0_ns']), 'counts': {
        'grid': len(saved['grid_inbound']), 'selected': len(inbound),
        'inbound': int(inbound.sum()), 'retained': int(retained.sum())},
        'tracking_loss': bool(retained.sum() < MIN_MATCHES)}
    for name, prefix, mask in [('grid', 'grid', slice(None)), ('selected', 'selected', slice(None)),
                                ('retained', 'selected', retained)]:
        result[name] = population(saved[prefix+'_uv0'][mask], saved[prefix+'_raw_flow_cov'][mask],
            saved[prefix+'_depth0'][mask], saved[prefix+'_depth1'][mask], saved[prefix+'_inbound'][mask], height)
    if not (result['counts']['selected'] >= result['counts']['inbound'] >= result['counts']['retained']):
        raise ValueError('Nonmonotonic retention counts')
    return result


def compare_npz(old, new):
    with np.load(old, allow_pickle=False) as a, np.load(new, allow_pickle=False) as b:
        return set(a.files) == set(b.files) and all(np.array_equal(a[k], b[k], equal_nan=True) for k in a.files)


def verify(plan):
    for name, digest in plan['files_sha256'].items():
        if sha(Path(name)) != digest:
            raise ValueError('Frozen source/baseline/input changed: '+name)


def prepare(args):
    old = json.loads((args.source/'plan.json').read_text())
    verify(old)
    files = [args.source/'plan.json', args.source/'execution-plan.json',
             args.source/'baseline-status.json', *[p for p in (args.source/'baseline').rglob('*') if p.is_file()],
             Path(__file__), ROOT/'Scripts/UnitTest/test_td3_tracking_uncertainty.py']
    hashes = dict(old['files_sha256'])
    hashes.update({str(p.resolve()): sha(p) for p in files})
    execution = json.loads((args.source/'execution-plan.json').read_text())
    plan = {'created_at_utc': datetime.now(timezone.utc).isoformat(), 'source': str(args.source.resolve()),
        'source_plan': old, 'execution': execution,
        'protocol': 'Only enable unchanged --record-uncertainty on fixed six night cases. Compare all available arrays bitwise and preserve RGB failures. Fixed-grid raw trace before selection/floors; no new weights, masks or thresholds.',
        'analysis_settings': {'component_floor_px2': FLOOR, 'native_quality_cap_px2': QUALITY_CAP,
                              'min_retained_tracking': MIN_MATCHES, 'depth_bins_m': [0, 10, 20, 50, 100, 'inf'],
                              'depth_bin_coordinate': 'source forward depth0, not Euclidean range',
                              'associations': 'within modality, contemporaneous raw-grid median trace vs retained count/loss; no fitted thresholds, p-values or prediction lead time'},
        'files_sha256': hashes}
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'plan.json').write_text(json.dumps(plan, indent=2)+'\n')
    print('Frozen recorder/analysis protocol and', len(hashes), 'files', flush=True)


def analyze_case(original, recorded, rows, modality):
    old = json.loads((original/'report.json').read_text())
    new = json.loads((recorded/'report.json').read_text())
    stable_keys = set(old) | set(new)
    stable_keys.discard('started_at_utc')
    if any(old.get(k) != new.get(k) for k in stable_keys):
        raise ValueError('Recording changed baseline status/settings/tracking report')
    if (original/'effective-config.json').read_bytes() != (recorded/'effective-config.json').read_bytes():
        raise ValueError('Effective configuration changed')
    controls = {'report_unchanged_except_start_time': True,
                'first_depth_bitwise_equal': compare_npz(original/'first-depth.npz', recorded/'first-depth.npz')}
    for filename in ['tensor-map.npz', 'trajectory.npz']:
        if (original/filename).exists() != (recorded/filename).exists():
            raise ValueError('Recording changed baseline artifact availability')
        controls[filename+'_bitwise_equal'] = compare_npz(original/filename, recorded/filename) if (original/filename).exists() else None
    if any(v is False for v in controls.values()):
        raise ValueError('Observer-on/off control failed')
    paths = sorted((recorded/'uncertainty').glob('pair-*.npz'))
    if len(paths) != 19:
        raise ValueError('Expected 19 diagnostic pairs, including failed cases')
    graph = np.load(original/'tensor-map.npz', allow_pickle=False) if (original/'tensor-map.npz').exists() else None
    pairs = []
    try:
        for index, path in enumerate(paths):
            with np.load(path, allow_pickle=False) as saved:
                pair = summarize_pair(saved)
            if pair['time0_ns'] != int(rows[index][modality+'_left']['time_ns']):
                raise ValueError('Source acquisition timestamp mismatch')
            pair.update(pair_index=index, time1_ns=int(rows[index+1][modality+'_left']['time_ns']))
            if graph is not None:
                retained = int(((graph['edge/match2frame1/mapping'] == index) &
                                (graph['edge/match2frame2/mapping'] == index+1)).sum())
                if retained != pair['counts']['retained']:
                    raise ValueError('Diagnostic retention differs from unchanged graph')
            pairs.append(pair)
        expected_flags = postprocessed_flags([p['tracking_loss'] for p in pairs])
        if graph is not None and not np.array_equal(graph['frames//need_interp'][1:].reshape(-1), expected_flags):
            raise ValueError('Postprocessed graph flags differ from native boundary policy')
    finally:
        if graph is not None:
            graph.close()
    if sum(p['counts']['retained'] for p in pairs) != old.get('tracking', {}).get('retained_matches', 0):
        raise ValueError('Diagnostic retained total differs from baseline')
    if 'tracking' in old and int(expected_flags.sum()) != old['tracking']['interpolated_frames']:
        raise ValueError('Diagnostic tracking-loss count differs from baseline interpolation')
    return {'baseline_status': old['status'], 'baseline_error': old.get('error'),
            'controls': controls, 'raw_low_retention_pairs': sum(p['tracking_loss'] for p in pairs),
            'postprocessed_interpolation_flags': int(expected_flags.sum()),
            'flag_scope': 'Native MotionInterpolate clears first/last two motion flags; false endpoint flags do not prove tracking.',
            'pairs': pairs}


def scalar(record, population_name='grid'):
    p = record[population_name]['raw_trace_px2']['p10_p50_p90']
    return p[1] if p is not None else float('nan')


def evaluate(args):
    plan = json.loads((args.output/'plan.json').read_text())
    verify(plan)
    manifest = json.loads(Path(plan['source_plan']['manifest']).read_text())['pairs']
    original = Path(plan['source'])/'baseline'
    results = {}
    for window in plan['source_plan']['windows']:
        name, start = window['name'], window['manifest_start']
        results[name] = {m: analyze_case(original/name/m, args.output/'recorded'/name/m,
                                       manifest[start:start+20], m) for m in MODALITIES}
    verify(plan)
    associations = {}
    for m in MODALITIES:
        pairs = [p for w in WINDOWS for p in results[w][m]['pairs']]
        scores = [scalar(p) for p in pairs]
        associations[m] = {'pairs': len(pairs), 'loss_pairs': sum(p['tracking_loss'] for p in pairs),
            'grid_trace_vs_retained_spearman': rank_association(scores, [p['counts']['retained'] for p in pairs]),
            'grid_trace_contemporaneous_loss_auc': auc(scores, [p['tracking_loss'] for p in pairs]),
            'scope': 'Dependent descriptive pairs; covariance already affects selection. No independent geometric-error calibration or early-warning claim.'}
    report = {'created_at_utc': datetime.now(timezone.utc).isoformat(), 'controls_pass': True,
              'frozen_files_unchanged': True, 'plan_sha256': sha(args.output/'plan.json'),
              'associations': associations, 'results': results}
    with (args.output/'analysis.json').open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    rows = []
    for w in WINDOWS:
        for m in MODALITIES:
            for p in results[w][m]['pairs']:
                def median(pop, field):
                    v = p[pop][field]['p10_p50_p90']
                    return v[1] if v is not None else None
                rows.append({'window': w, 'modality': m, 'pair': p['pair_index'], 'time0_ns': p['time0_ns'],
                    'time1_ns': p['time1_ns'], **p['counts'], 'tracking_loss': p['tracking_loss'],
                    'grid_median_trace_px2': median('grid', 'raw_trace_px2'),
                    'selected_median_trace_px2': median('selected', 'raw_trace_px2'),
                    'retained_median_trace_px2': median('retained', 'raw_trace_px2'),
                    'grid_quality_below_native_cap_fraction': p['grid']['grid_quality_below_native_cap_fraction'],
                    'grid_variance_components_below_floor_fraction': p['grid']['variance_components_below_floor_fraction'],
                    'grid_depth0_median_m': median('grid', 'depth0_m')})
    with (args.output/'pairs.csv').open('x') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys()); writer.writeheader(); writer.writerows(rows)
    plot(results, args.output)


def plot(results, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 3, figsize=(13, 7), layout='constrained')
    for col, w in enumerate(WINDOWS):
        for m, color in [('rgb', '#2463a7'), ('thermal', '#d47a23')]:
            pairs = results[w][m]['pairs']; x = np.arange(1, len(pairs)+1)
            axes[0, col].plot(x, [scalar(p) for p in pairs], marker='.', color=color, label=m.upper())
            counts = np.array([p['counts']['retained'] for p in pairs])
            axes[1, col].plot(x, counts, marker='.', color=color, label=m.upper())
            loss = counts < MIN_MATCHES
            axes[1, col].scatter(x[loss], counts[loss], color='crimson', marker='x', zorder=4)
        axes[0, col].set_yscale('log'); axes[0, col].set_title(w)
        axes[1, col].axhline(MIN_MATCHES, color='gray', linestyle='--', linewidth=1)
        axes[1, col].set_ylim(-5, 210); axes[1, col].set_xlabel('Consecutive pair within frozen window')
        for row in range(2):
            axes[row, col].grid(alpha=.2); axes[row, col].set_axisbelow(True)
    axes[0, 0].set_ylabel('Median raw fixed-grid trace (px²)')
    axes[1, 0].set_ylabel('Retained observations')
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='outside lower center', ncol=2, frameon=False)
    fig.suptitle('Unchanged night baseline: raw uncertainty and retention\nCamera-local pixel units; selection uses covariance; descriptive, not calibrated cross-modal confidence')
    for ext in ['png', 'pdf']:
        fig.savefig(output/('uncertainty-retention.'+ext), dpi=175)
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['prepare', 'evaluate'])
    parser.add_argument('--source', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.action == 'prepare' and args.source is None:
        parser.error('prepare requires --source pointing to EXP-013 output')
    (prepare if args.action == 'prepare' else evaluate)(args)
