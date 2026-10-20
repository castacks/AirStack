"""Within-window observational rank diagnostics, never a fitted VO/fusion rule."""
import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import rankdata
from .td3_tracking_uncertainty import sha, rank_association
from .td3_night_geometry import verify

CONTEXT = ('translation_m', 'rotation_deg', 'flow_px', 'inverse_depth_m')
TARGETS = ('forward_px', 'sampson_px')


def median_finite(values):
    values = np.asarray(values, float)
    valid = values[np.isfinite(values)]
    return float(np.median(valid)) if len(valid) else None


def partial_rank(x, y, context):
    """Pearson of rank residuals, with intercept; rank nuisance once on valid rows."""
    x, y, context = np.asarray(x, float), np.asarray(y, float), np.asarray(context, float)
    if x.ndim != 1 or y.shape != x.shape or context.ndim != 2 or len(context) != len(x):
        raise ValueError('Expected aligned x/y vectors and context matrix')
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(context).all(axis=1)
    indices = np.flatnonzero(valid)
    n = len(indices)
    residuals = np.full((len(x), 2), np.nan)
    result = dict(n=n, design_columns=context.shape[1]+1, design_rank=0,
                  residual_dimension=0, rho=None, status='insufficient_samples')
    if n == 0:
        return result, residuals, valid
    nuisance = np.column_stack([np.ones(n), *[rankdata(c) for c in context[valid].T]])
    ranked = np.column_stack([rankdata(x[valid]), rankdata(y[valid])])
    coefficients, _, rank, _ = np.linalg.lstsq(nuisance, ranked, rcond=None)
    remaining = ranked-nuisance@coefficients
    residuals[valid] = remaining
    result.update(design_rank=int(rank), residual_dimension=n-int(rank))
    if n < int(rank)+3:
        return result, residuals, valid
    if np.any(np.linalg.norm(remaining, axis=0) <= 1e-10*max(1.,np.linalg.norm(ranked))):
        result['status'] = 'zero_rank_residual'
        return result, residuals, valid
    result.update(rho=float(np.corrcoef(remaining.T)[0,1]), status='defined')
    return result, residuals, valid


def temporal_checks(x, y, pair_index):
    x, y, pair_index = np.asarray(x,float), np.asarray(y,float), np.asarray(pair_index,int)
    if x.shape != y.shape or x.shape != pair_index.shape or np.any(np.diff(pair_index)<=0):
        raise ValueError('Chronologically aligned distinct pairs required')
    adjacent = np.diff(pair_index)==1
    dx, dy = np.diff(x), np.diff(y)
    good = adjacent & np.isfinite(dx) & np.isfinite(dy)
    first = pair_index < 9
    return dict(difference_n=int(good.sum()), difference_rho=rank_association(dx[good],dy[good]),
                first9_n=int((first & np.isfinite(x) & np.isfinite(y)).sum()),
                first9_rho=rank_association(x[first],y[first]),
                last10_n=int((~first & np.isfinite(x) & np.isfinite(y)).sum()),
                last10_rho=rank_association(x[~first],y[~first]))


def pair_summary(data, mask):
    d = {k:data[k][mask] for k in ['trace','uv0','uv1','depth0',*TARGETS]}
    depth = d['depth0']; inverse = np.full(len(depth),np.nan)
    positive = np.isfinite(depth) & (depth>0)
    np.divide(1., depth, out=inverse, where=positive)
    flow = np.linalg.norm(d['uv1']-d['uv0'],axis=1)
    result = dict(points=int(mask.sum()))
    for key, values in dict(trace=d['trace'], flow_px=flow, inverse_depth_m=inverse,
                            **{k:d[k] for k in TARGETS}).items():
        result[key] = median_finite(values)
        result[key+'_finite_n'] = int(np.isfinite(values).sum())
    return result


def prepare(source, output, literature):
    old = json.loads((source/'plan.json').read_text())
    files = dict(old['files_sha256'])
    inputs = sorted((source/'points').glob('*/*/grid-*.npz'))
    if len(inputs)!=114: raise ValueError('Expected 114 fixed-grid input records')
    paths = [source/'plan.json',source/'analysis.json',*inputs,Path(__file__),
             Path(__file__).parents[1]/'UnitTest/test_td3_temporal_context.py', literature]
    files.update({str(p.resolve()):sha(p) for p in paths}); verify(files)
    output.mkdir(parents=True,exist_ok=False)
    plan = dict(created_at_utc=datetime.now(timezone.utc).isoformat(),source=str(source),
        literature=str(literature), files_sha256=files, base_commit='3ac07f047',
        protocol=dict(populations=['all','inbound'],frames_per_case=20,pairs_per_case=19,
            windows=['early','middle','late'],modalities=['rgb','thermal'],targets=TARGETS,
            context=CONTEXT, time_adjustment='secondary ranked chronological pair index',
            aggregation='per-pair finite medians, inverse depth only finite positive source depth',
            statistic='Spearman and partial rank via OLS residualization; no significance/causality/prediction claims',
            degeneracy='undefined if n<rank(design)+3 or residual rank norm <=1e-10*max(1,norm(ranked_xy))',
            temporal='adjacent first differences; fixed chronological first9/last10 raw rank correlation',
            calibration='unchanged; SLAM reference and predicted depth/flow may share model/context errors'))
    (output/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')


def write_csv(path, records):
    with path.open('w') as f:
        writer=csv.DictWriter(f, fieldnames=list(records[0])); writer.writeheader(); writer.writerows(records)


def run(output):
    plan=json.loads((output/'plan.json').read_text()); verify(plan['files_sha256'])
    source=Path(plan['source']); pairs=[]; cases=[]; arrays=output/'arrays'; arrays.mkdir()
    for window in plan['protocol']['windows']:
        for modality in plan['protocol']['modalities']:
            local=[]
            for i in range(19):
                with np.load(source/'points'/window/modality/f'grid-{i:03d}.npz') as data:
                    relative=data['relative']
                    for group in ['all','inbound']:
                        row=dict(window=window,modality=modality,pair=i,population=group,
                                 time0_ns=int(data['time0_ns']),time1_ns=int(data['time1_ns']),
                                 translation_m=float(np.linalg.norm(relative[:3,3])),
                                 rotation_deg=float(np.degrees(Rotation.from_matrix(relative[:3,:3]).magnitude())),
                                 **pair_summary(data,data['mask_'+group]))
                        pairs.append(row); local.append(row)
            for group in ['all','inbound']:
                rows=[r for r in local if r['population']==group]
                context=np.array([[r[k] for k in CONTEXT] for r in rows],float)
                x=np.array([r['trace'] for r in rows],float); index=np.array([r['pair'] for r in rows])
                flow=np.array([r['flow_px'] for r in rows],float)
                saved={k:np.array([r[k] for r in rows]) for k in ['pair','time0_ns','time1_ns','trace',*CONTEXT,*TARGETS]}
                for target in TARGETS:
                    y=np.array([r[target] for r in rows],float)
                    adjusted,residuals,valid=partial_rank(x,y,context)
                    timed,timed_residuals,timed_valid=partial_rank(x,y,np.column_stack([context,index]))
                    result=dict(window=window,modality=modality,population=group,target=target,pairs=19,
                                raw_n=int((np.isfinite(x)&np.isfinite(y)).sum()),
                                raw_rho=rank_association(x,y),flow_rho=rank_association(flow,y),
                                raw_common_mask_rho=rank_association(x[valid],y[valid]),
                                **{'adjusted_'+k:v for k,v in adjusted.items()},
                                **{'time_adjusted_'+k:v for k,v in timed.items()},
                                **temporal_checks(x,y,index))
                    cases.append(result)
                    saved.update({target+'_rank_residuals':residuals,target+'_valid_rows':valid,
                                  target+'_time_rank_residuals':timed_residuals,target+'_time_valid_rows':timed_valid})
                np.savez_compressed(arrays/f'{window}-{modality}-{group}.npz',**saved)
    write_csv(output/'pairs.csv',pairs); write_csv(output/'cases.csv',cases)
    analysis=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),base_commit=plan['base_commit'],
        cases=cases,pairs=pairs,verified_hashes=verify(plan['files_sha256']),source_pair_files=114,
        scope='within-window observational evidence; context regression is not a fitted sensor correction or predictor')
    (output/'analysis.json').write_text(json.dumps(analysis,indent=2,allow_nan=False)+'\n')
    plot(output,pairs,cases)


def plot(output, pairs, cases):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes=plt.subplots(2,3,figsize=(13,6),sharex=True,sharey=True)
    for row,modality in enumerate(['rgb','thermal']):
        for col,window in enumerate(['early','middle','late']):
            rows=[r for r in pairs if r['modality']==modality and r['window']==window and r['population']=='all']
            ax=axes[row,col]
            for key,label in [('trace','uncertainty'),('forward_px','reprojection'),('sampson_px','epipolar'),('flow_px','flow magnitude')]:
                values=np.array([r[key] for r in rows],float); valid=np.isfinite(values)
                ranked=np.full(19,np.nan); ranked[valid]=rankdata(values[valid])/valid.sum()
                ax.plot(np.arange(19),ranked,label=label,alpha=.8,linewidth=1.3)
            ax.axvline(8.5,color='black',linestyle=':',alpha=.4); ax.set_title(f'{modality}: {window}')
            ax.grid(alpha=.2); ax.set_xlabel('pair index'); ax.set_ylabel('within-window rank / n')
    axes[0,0].legend(fontsize=8);fig.suptitle('All fixed-grid pair medians: simultaneous observations, no forecasting')
    fig.tight_layout();fig.savefig(output/'temporal-context.png',dpi=180);fig.savefig(output/'temporal-context.pdf');plt.close(fig)
    primary=[c for c in cases if c['population']=='all']
    labels=[c['modality']+'/'+c['window']+'/'+('reproj' if c['target']=='forward_px' else 'epi') for c in primary]
    fig,ax=plt.subplots(figsize=(12,4))
    for offset,key,label in [(-.24,'raw_rho','raw'),(-.08,'adjusted_rho','context adjusted'),(.08,'time_adjusted_rho','context + time'),(.24,'difference_rho','first differences')]:
        ax.bar(np.arange(len(primary))+offset,[np.nan if c[key] is None else c[key] for c in primary],width=.16,label=label)
    ax.set_xticks(np.arange(len(primary)),labels,rotation=45,ha='right');ax.set_ylim(-1,1);ax.axhline(0,color='black',linewidth=.7)
    ax.set_ylabel('rank correlation');ax.legend(fontsize=8,ncol=4);ax.set_title('Within-window associations, 19 pairs each; recorded SLAM proxy')
    fig.tight_layout();fig.savefig(output/'associations.png',dpi=180);fig.savefig(output/'associations.pdf');plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=['prepare','run'])
    parser.add_argument('--source',type=Path);parser.add_argument('--literature',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.action=='prepare': prepare(args.source,args.output,args.literature)
    else:run(args.output)
