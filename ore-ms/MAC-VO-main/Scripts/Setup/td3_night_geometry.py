"""Frozen night fixed-grid uncertainty versus SLAM-proxy geometric residuals."""
import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from .td3_reference import load_reference, camera_reference
from .td3_reprojection import pair_geometry
from .td3_tracking_uncertainty import sha, percentiles, rank_association, retained_mask

ROOT = Path('/workspace')


def measure(pair, prefix, relative):
    uv0, uv1 = pair[prefix+'_uv0'], pair[prefix+'_uv1']
    d0, d1 = pair[prefix+'_depth0'], pair[prefix+'_depth1']
    geometry = pair_geometry(uv0, uv1, d0, d1, pair['K'], relative)
    forward = np.linalg.norm(geometry['forward_error_px'], axis=1)
    forward[~(np.isfinite(d0) & (d0 > 0))] = np.nan
    sampson = geometry['sampson_px'].copy()
    if np.linalg.norm(relative[:3, 3]) <= 1e-6:
        sampson[:] = np.nan
    return dict(uv0=uv0, uv1=uv1, depth0=d0, depth1=d1,
                trace=pair[prefix+'_raw_flow_cov'][:, :2].sum(axis=1),
                inbound=pair[prefix+'_inbound'], forward_px=forward, sampson_px=sampson)


def summarize(data, mask):
    result = {'count': int(mask.sum())}
    for key in ['trace', 'forward_px', 'sampson_px']:
        result[key] = percentiles(data[key][mask])
    for key in ['forward_px', 'sampson_px']:
        valid = mask & np.isfinite(data['trace']) & np.isfinite(data[key])
        result[key+'_association_n'] = int(valid.sum())
        result[key+'_rho'] = rank_association(data['trace'][valid], data[key][valid])
    return result


def masks(data, height):
    d0, d1 = data['depth0'], data['depth1']
    positive = np.isfinite(d0) & (d0 > 0)
    result = {'all': np.ones(len(d0), bool), 'inbound': data['inbound'].astype(bool),
              'inbound_both_depths_max50m': data['inbound'] & positive & (d0 <= 50)
              & np.isfinite(d1) & (d1 > 0) & (d1 <= 50)}
    for i in range(3):
        result['image_third_'+str(i)] = (data['uv0'][:, 1] >= i*height/3) & (data['uv0'][:, 1] < (i+1)*height/3)
    for lo, hi in [(0,10),(10,20),(20,50),(50,np.inf)]:
        result[f'depth_{lo}_{hi}'] = positive & (d0 >= lo) & (d0 < hi)
    result['depth_invalid'] = ~positive
    assert sum(result[f'image_third_{i}'].sum() for i in range(3)) == len(d0)
    assert sum(v.sum() for k,v in result.items() if k.startswith('depth_')) == len(d0)
    return result


def verify(files):
    bad = [p for p,h in files.items() if not Path(p).is_file() or sha(Path(p)) != h]
    if bad:
        raise ValueError('Frozen inputs changed: '+str(bad[:3]))
    return len(files)


def prepare(source, output):
    inherited = json.loads((source/'plan.json').read_text())
    files = dict(inherited['files_sha256'])
    paths = [source/'plan.json', *sorted((source/'recorded').glob('*/*/uncertainty/pair-*.npz')),
             Path(__file__), Path(__file__).parents[1]/'UnitTest/test_td3_night_geometry.py']
    files.update({str(p.resolve()): sha(p) for p in paths})
    verify(files)
    output.mkdir(parents=True, exist_ok=False)
    plan = {'created_at_utc': datetime.now(timezone.utc).isoformat(), 'source': str(source),
            'source_plan': inherited['source_plan'], 'files_sha256': files,
            'protocol': {'population': 'all fixed grid; selected retained separately',
                'forward': 'positive finite source depth, finite projection in front; no target-depth requirement',
                'sampson': 'finite coordinates, no depth requirement; undefined if translation <=1e-6m',
                'sensitivity': ['inbound','inbound_both_depths_max50m'],
                'statistics': 'within-pair Spearman; unweighted window medians; no fitting, p values or pooled inference',
                'translation_bins_m': [0,.01,.1,'infinity'], 'rotation_bins_deg': [0,.1,1,'infinity'],
                'scope': 'recorded SLAM proxy, provisional calibration; no cross-camera ranking or GT claim'}}
    (output/'plan.json').write_text(json.dumps(plan, indent=2)+'\n')


def run(output):
    plan = json.loads((output/'plan.json').read_text()); verify(plan['files_sha256'])
    src = Path(plan['source']); old = plan['source_plan']
    reference, _ = load_reference(Path(old['dataset']))
    rows = json.loads(Path(old['manifest']).read_text())['pairs']
    records = []; cases = []
    for window in old['windows']:
        for modality in ['rgb', 'thermal']:
            name = window['name']; start = window['manifest_start']
            times = np.array([r[modality+'_left']['time_ns'] for r in rows[start:start+20]], dtype=np.int64)
            cameras = camera_reference(reference, times, Path(old['dataset']), Path(old['calibration']), modality)
            case_records = []
            destination = output/'points'/name/modality; destination.mkdir(parents=True)
            paths = sorted((src/'recorded'/name/modality/'uncertainty').glob('pair-*.npz'))
            if len(paths) != 19: raise ValueError('Expected 19 diagnostic pairs')
            for i,path in enumerate(paths):
                relative = np.linalg.inv(cameras[i+1]) @ cameras[i]
                translation = float(np.linalg.norm(relative[:3,3]))
                rotation = float(np.degrees(Rotation.from_matrix(relative[:3,:3]).magnitude()))
                with np.load(path) as pair:
                    if int(pair['time0_ns']) != int(times[i]): raise ValueError('Timestamp mismatch')
                    for population,prefix in [('grid','grid'),('retained','selected')]:
                        data = measure(pair, prefix, relative)
                        if population == 'retained':
                            keep = retained_mask(pair['selected_inbound'], pair['outlier_keep_inbound'])
                            data = {k:v[keep] for k,v in data.items()}
                        groups = masks(data, int(pair['height_width'][0]))
                        np.savez_compressed(destination/f'{population}-{i:03d}.npz', **data,
                            **{'mask_'+k:v for k,v in groups.items()}, relative=relative,
                            time0_ns=times[i], time1_ns=times[i+1])
                        for group,mask in groups.items():
                            record = dict(window=name, modality=modality, pair=i, population=population,
                                group=group, translation_m=translation, rotation_deg=rotation,
                                translation_bin=int(np.searchsorted([.01,.1],translation,side='right')),
                                rotation_bin=int(np.searchsorted([.1,1],rotation,side='right')),
                                **summarize(data, mask))
                            records.append(record); case_records.append(record)
            for population in ['grid','retained']:
                for group in groups:
                    subset = [r for r in case_records if r['population']==population and r['group']==group]
                    result = dict(window=name, modality=modality, population=population, group=group,
                                  pairs=len(subset), total_points=sum(r['count'] for r in subset))
                    for metric in ['trace','forward_px','sampson_px']:
                        values = [r[metric]['p10_p50_p90'][1] for r in subset if r[metric]['finite_count']]
                        result[metric+'_pair_median'] = float(np.median(values)) if values else None
                        result[metric+'_finite_points'] = sum(r[metric]['finite_count'] for r in subset)
                    for metric in ['forward_px','sampson_px']:
                        values = [r[metric+'_rho'] for r in subset if r[metric+'_rho'] is not None]
                        result[metric+'_rho_median'] = float(np.median(values)) if values else None
                        result[metric+'_rho_defined_pairs'] = len(values)
                    cases.append(result)
    motion = []
    for modality in ['rgb','thermal']:
        for kind in ['translation','rotation']:
            for bin_id in range(3):
                subset = [r for r in records if r['modality']==modality and r['population']=='grid' and r['group']=='all' and r[kind+'_bin']==bin_id]
                motion.append(dict(modality=modality, kind=kind, bin=bin_id, pairs=len(subset),
                    **{metric:percentiles([r[metric]['p10_p50_p90'][1] for r in subset if r[metric]['finite_count']]) for metric in ['trace','forward_px','sampson_px']}))
    analysis = {'created_at_utc':datetime.now(timezone.utc).isoformat(), 'base_commit':'3ac07f047',
                'cases':cases, 'pairs':records, 'motion_strata':motion,
                'verified_hashes':verify(plan['files_sha256']), 'pair_files':114,
                'epipolar_degenerate_pairs':sum(r['translation_m']<=1e-6 for r in records if r['population']=='grid' and r['group']=='all')}
    (output/'analysis.json').write_text(json.dumps(analysis, indent=2, allow_nan=False)+'\n')
    with (output/'cases.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(cases[0])); writer.writeheader(); writer.writerows(cases)
    plot(output,cases)


def plot(output,cases):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(12,3.5))
    for ax,metric,title in zip(axes,['trace','forward_px','sampson_px'],['Raw trace (px²)','Forward reprojection (px)','Sampson residual (px)']):
        for modality in ['rgb','thermal']:
            values=[next(c for c in cases if c['window']==w and c['modality']==modality and c['population']=='grid' and c['group']=='all')[metric+'_pair_median'] for w in ['early','middle','late']]
            ax.plot(['early','middle','late'],values,'o-',label=modality)
        ax.set_yscale('log'); ax.set_title(title); ax.grid(alpha=.25)
    axes[0].legend(); fig.suptitle('All fixed-grid points; median of pair medians; recorded SLAM proxy')
    fig.tight_layout(); fig.savefig(output/'night-geometry.png',dpi=180); fig.savefig(output/'night-geometry.pdf'); plt.close(fig)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['prepare','run']); parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--source',type=Path)
    args=parser.parse_args()
    if args.action=='prepare': prepare(args.source,args.output)
    else: run(args.output)
