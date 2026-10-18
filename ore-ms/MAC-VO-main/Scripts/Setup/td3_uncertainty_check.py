"""Predeclare disjoint windows, then evaluate raw variance and frozen translation scales."""
import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
import numpy as np
try:
    from .td3_reference import load_reference, camera_reference, matrices, trajectory_metrics, select_window
    from .td3_feature_consistency import fit_trajectory_scale, unproject_ned, rank_association
except ImportError:
    from td3_reference import load_reference, camera_reference, matrices, trajectory_metrics, select_window
    from td3_feature_consistency import fit_trajectory_scale, unproject_ned, rank_association


def prepare(args):
    reference, _ = load_reference(args.dataset)
    rows = json.loads(args.manifest.read_text())['pairs']
    # Regions fixed before inference: first/last quarters; exclude original348:368.
    regions = [(0, len(rows)//4), (3*len(rows)//4, len(rows))]
    windows = []
    for name, (lo, hi) in zip(['early','late'], regions):
        window = select_window(reference, rows[lo:hi], 20, 150_000_000)
        window['manifest_start'] += lo
        window.update(name=name, region=[lo,hi])
        windows.append(window)
    scales = {}
    for modality in ['rgb','thermal']:
        source = np.load(args.training / modality / 'trajectory.npz')
        camera = camera_reference(reference,source['time_ns'].reshape(-1),args.dataset,args.calibration,modality)
        scales[modality] = fit_trajectory_scale(matrices(source['poses_camera_ned']),camera)['sequential_split_diagnostic']['first_half_fit_scale']
    plan = {'created_at_utc':datetime.now(timezone.utc).isoformat(),
            'scope':'separate windows of same clip; recorded SLAM proxy, not independent GT',
            'frozen_scale_training':'EXP-004 original moving window first10 frames only',
            'frozen_scales':scales, 'windows':windows,
            'association_max_endpoint_depth_m':50,
            'manifest_sha256':hashlib.sha256(args.manifest.read_bytes()).hexdigest(),
            'calibration_sha256':hashlib.sha256(args.calibration.read_bytes()).hexdigest()}
    args.output.mkdir(parents=True,exist_ok=True)
    with (args.output/'plan.json').open('x') as f: json.dump(plan,f,indent=2)
    print(json.dumps(plan,indent=2))


def raw_association(directory, camera, times):
    groups = {'grid':[], 'selected':[], 'retained':[]}
    floors = {str(i):{'flow_before':[], 'depth_before':[], 'depth_after':[]} for i in range(2)}
    kept_counts = []
    paths = sorted((directory/'uncertainty').glob('pair-*.npz'))
    if len(paths) != len(times)-1: raise ValueError('Expected one diagnostic per consecutive pair')
    for index,path in enumerate(paths):
        with np.load(path) as data:
            if int(data['time0_ns']) != int(times[index]): raise ValueError('Pair timestamp mismatch')
            relative = np.linalg.inv(camera[index+1]) @ camera[index]
            for population in groups:
                prefix = 'grid' if population=='grid' else 'selected'
                mask = data[prefix+'_inbound'].copy()
                if population=='retained':
                    retained = np.zeros(len(mask),dtype=bool)
                    retained[mask] = data['outlier_keep_inbound']
                    mask &= retained
                d0,d1 = data[prefix+'_depth0'],data[prefix+'_depth1']
                mask &= np.isfinite(d0)&np.isfinite(d1)&(d0>0)&(d1>0)&(d0<=50)&(d1<=50)
                k = np.repeat(data['K'].reshape(1,3,3),len(d0),axis=0)
                p0 = unproject_ned(data[prefix+'_uv0'],d0,k)
                p1 = unproject_ned(data[prefix+'_uv1'],d1,k)
                residual = np.linalg.norm(p1-(p0@relative[:3,:3].T+relative[:3,3]),axis=1)
                trace = data[prefix+'_raw_flow_cov'][:,:2].sum(axis=1)
                mask &= np.isfinite(trace)&np.isfinite(residual)
                groups[population].append(np.column_stack([trace[mask],residual[mask],.5*(d0[mask]+d1[mask])]))
            kept_counts.append([len(data['selected_inbound']),int(data['selected_inbound'].sum()),int(data['outlier_keep_inbound'].sum())])
            for i in range(2):
                for key,src in [('flow_before','flow_cov_before'),('depth_before','depth_variance_before'),('depth_after','depth_variance_after')]:
                    floors[str(i)][key].append(data[f'endpoint{i}_{src}'])
    summaries = {}
    for pop,pairs in groups.items():
        allrows = np.concatenate(pairs)
        if not len(allrows):
            summaries[pop] = {'count':0}
            continue
        within = [rank_association(a[:,0],a[:,1],a[:,2]) for a in pairs]
        within = [x for x in within if x is not None]
        summaries[pop] = {'count':len(allrows), 'median_raw_trace_px2':float(np.median(allrows[:,0])),
            'median_residual_m':float(np.median(allrows[:,1])),
            'pooled_spearman':rank_association(allrows[:,0],allrows[:,1]),
            'pooled_depth_partial_rank':rank_association(allrows[:,0],allrows[:,1],allrows[:,2]),
            'median_within_pair_depth_partial_rank':float(np.median(within)) if within else None}
    construction = {}
    for i,values in floors.items():
        flow, before, after = [np.concatenate(values[k]) for k in ['flow_before','depth_before','depth_after']]
        construction[i] = {'observations_before_outlier':len(before),
            'flow_diagonals_below_floor_fraction':float((flow[:,:2]<.25**2).mean()),
            'depth_variance_below_floor_fraction':float((before<.05).mean()),
            'median_depth_variance_before_m2':float(np.median(before)),
            'floor_application_verified':bool(np.allclose(after,np.maximum(before,.05)))}
    return {'depth_cap_m':50,'populations':summaries,'covariance_construction':construction,
            'selection_counts_candidate_inbound_retained':np.sum(kept_counts,axis=0).tolist()}


def evaluate(args):
    plan = json.loads((args.output/'plan.json').read_text())
    for path,key in [(args.manifest,'manifest_sha256'),(args.calibration,'calibration_sha256')]:
        if hashlib.sha256(path.read_bytes()).hexdigest() != plan[key]:
            raise ValueError('Plan input hash mismatch')
    reference,_ = load_reference(args.dataset)
    results = {}
    for name in ['original']+[w['name'] for w in plan['windows']]:
        results[name] = {}
        for modality in ['rgb','thermal']:
            directory = args.output/name/modality
            source = np.load(directory/'trajectory.npz')
            times = source['time_ns'].reshape(-1)
            camera = camera_reference(reference,times,args.dataset,args.calibration,modality)
            estimated = matrices(source['poses_camera_ned'])
            anchored = np.linalg.inv(estimated[0])@estimated
            anchored[:,:3,3] *= plan['frozen_scales'][modality]
            results[name][modality] = {'raw':trajectory_metrics(estimated,camera),
                'frozen_translation_scale':plan['frozen_scales'][modality],
                'scaled_sensitivity':trajectory_metrics(anchored,camera),
                'raw_uncertainty':raw_association(directory,camera,times)}
            if name=='original':
                old = np.load(args.training/modality/'tensor-map.npz')
                new = np.load(directory/'tensor-map.npz')
                equal = all(np.array_equal(old[k],new[k],equal_nan=True) for k in old.files)
                results[name][modality]['observer_on_off_graph_bitwise_equal'] = equal
                results[name][modality]['observer_on_off_max_pose_difference'] = float(np.max(np.abs(source['poses_camera_ned']-np.load(args.training/modality/'trajectory.npz')['poses_camera_ned'])))
    sources = [Path(__file__), Path(__file__).with_name('td3_uncertainty.py'),
               Path(__file__).with_name('run_td3_execution.py'),
               Path(__file__).parents[2]/'Module/Covariance/Project2to3.py']
    provenance = {'evaluated_at_utc':datetime.now(timezone.utc).isoformat(),
                  'source_sha256':{str(p.relative_to(Path(__file__).parents[2])):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
                  'training_trajectory_sha256':{m:hashlib.sha256((args.training/m/'trajectory.npz').read_bytes()).hexdigest() for m in ['rgb','thermal']}}
    with (args.output/'analysis.json').open('x') as f:
        json.dump({'plan':plan,'provenance':provenance,'results':results},f,indent=2,allow_nan=False)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1,2,figsize=(11,4),layout='constrained')
    names = [w['name'] for w in plan['windows']]
    for ax,modality in zip(axes,['rgb','thermal']):
        x = np.arange(len(names))
        raw = [results[n][modality]['raw']['anchored_translation_rmse_m'] for n in names]
        scaled = [results[n][modality]['scaled_sensitivity']['anchored_translation_rmse_m'] for n in names]
        ax.bar(x-.18,raw,.36,label='Original trajectory')
        ax.bar(x+.18,scaled,.36,label='Frozen scale sensitivity')
        ax.set_xticks(x,names); ax.set_ylabel('Translation disagreement RMS (m)')
        ax.set_title(modality.upper()); ax.legend(fontsize=8)
    fig.suptitle('Separate windows, same clip and recorded SLAM proxy')
    fig.savefig(args.output/'scale-check.png',dpi=160)
    plt.close(fig)
    print(json.dumps(results,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=['prepare','evaluate'])
    for key in ['dataset','manifest','calibration','training','output']: p.add_argument('--'+key,type=Path,required=True)
    args=p.parse_args()
    (prepare if args.action=='prepare' else evaluate)(args)
