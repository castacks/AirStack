"""Fixed range/grid/tail diagnostics on previously inspected candidate pairs."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
from zoneinfo import ZoneInfo
import numpy as np

PATHS={'rain':'results/ms2-day-cross-modal-replication/rain-control',
       'day':'results/ms2-day-cross-modal-replication/day',
       'night':'results/ms2-night-cross-modal-replication/night'}
EDGES=[0,5,10,20,40,80]
SHAPE=(384,1224)


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def masks(xyz,xy):
    distance=np.linalg.norm(xyz,axis=1)
    assert np.all(np.isfinite(distance)) and np.all(distance>0)
    band=np.searchsorted(EDGES,distance,side='right')-1
    h,w=SHAPE
    assert np.all(xy>=0) and np.all(xy[:,0]<w) and np.all(xy[:,1]<h)
    cell=(xy[:,1]*3//h)*3+xy[:,0]*3//w
    groups={'all':np.ones(len(distance),bool)}
    groups.update({f'range_{i}':band==i for i in range(6)})
    groups.update({f'cell_{j}':cell==j for j in range(9)})
    groups.update({f'joint_{i}_{j}':(band==i)&(cell==j) for i in range(6) for j in range(9)})
    return distance,groups


def tail_weights(r,t,fraction):
    n=len(r);keep=n-int(np.ceil(n*fraction));score=np.maximum(r,t)
    cutoff=np.sort(score)[keep-1]
    w=(score<cutoff).astype(float)
    w[score==cutoff]=(keep-w.sum())/np.count_nonzero(score==cutoff)
    assert np.isclose(w.sum(),keep)
    return w


def describe(r,t,distance,mask):
    n=int(mask.sum());total=len(r)
    if not n:return {'n':0,'fraction':0.,'stats':None}
    a,b,d=r[mask],t[mask],distance[mask]
    stats={'rgb_win_fraction':float((a<b).mean()),'tie_fraction':float((a==b).mean()),
           'median_reference_range_m':float(np.median(d))}
    for mod,e in [('rgb',a),('thr',b)]:
        stats[mod]={'mean_l2':float(e.mean()),'mse':float((e**2).mean()),'median_l2':float(np.median(e)),
                    'mean_relative_l2':float((e/d).mean()),'median_relative_l2':float(np.median(e/d))}
    stats['rgb_over_thr_mean_l2']=float(a.mean()/b.mean())
    stats['rgb_over_thr_mse']=float((a**2).mean()/(b**2).mean())
    stats['gap_contribution_mean_l2']=float((a-b).sum()/total)
    stats['gap_contribution_mse']=float((a**2-b**2).sum()/total)
    return {'n':n,'fraction':n/total,'stats':stats}


def run(root,output):
    if output.exists():raise FileExistsError(output)
    hashes={};results={}
    for rec,rel in PATHS.items():
        folder=root/rel;rows=[]
        for part in ['metric','support']:
            path=folder/part/'report.json';hashes[str(path)]=sha(path)
        for i in range(20):
            stem=f'{i:06d}'
            paths=[folder/'metric'/f'metric-{stem}.npz',folder/'support'/f'pairs-{stem}.npz']
            arrays=[]
            for path in paths:
                hashes[str(path)]=sha(path)
                with np.load(path,allow_pickle=False) as f:arrays.append({k:f[k] for k in f.files})
            a,p=arrays;valid=a['both_valid_mask']
            midpoint=(p['rgb_reference_xyz'][valid]+p['thr_reference_xyz_in_rgb'][valid])/2
            distance,groups=masks(midpoint,p['rgb_xy'][valid])
            weights={str(f):tail_weights(a['midpoint_rgb_error_m'],a['midpoint_thr_error_m'],f) for f in [.01,.05]}
            row={'stem':stem,'n':int(valid.sum()),'anchors':{}}
            for anchor in ['midpoint','rgb','thr']:
                r,t=a[f'{anchor}_rgb_error_m'],a[f'{anchor}_thr_error_m']
                summary={name:describe(r,t,distance,mask) for name,mask in groups.items()}
                tails={}
                for fraction,w in weights.items():
                    record={'kept_mass':float(w.sum()),'removed_fraction':float(1-w.mean()),'camera':{}}
                    for mod,e in [('rgb',r),('thr',t)]:
                        record['camera'][mod]={}
                        for loss,power in [('mean_l2',1),('mse',2)]:
                            loss_values=e**power
                            record['camera'][mod][loss]={'retained_risk':float(np.dot(w,loss_values)/w.sum()),
                                                         'removed_loss_fraction':float(np.dot(1-w,loss_values)/loss_values.sum())}
                    for loss in ['mean_l2','mse']:
                        record[f'rgb_over_thr_{loss}']=record['camera']['rgb'][loss]['retained_risk']/record['camera']['thr'][loss]['retained_risk']
                    tails[fraction]=record
                row['anchors'][anchor]={'groups':summary,'tails':tails}
            rows.append(row)
        results[rec]={'rows':rows,'summary':{}}
        for anchor in ['midpoint','rgb','thr']:
            group_summary={}
            for group in rows[0]['anchors'][anchor]['groups']:
                selected=[row['anchors'][anchor]['groups'][group] for row in rows]
                available=[x for x in selected if x['n']]
                stats={'nonempty_frames':len(available),'counts_min_median_max':[min(x['n'] for x in selected),float(np.median([x['n'] for x in selected])),max(x['n'] for x in selected)]}
                stats['median_fraction']=float(np.median([x['fraction'] for x in selected]))
                stats['median_stats']=None
                if available:
                    stats['median_stats']={k:float(np.median([x['stats'][k] for x in available])) for k in available[0]['stats'] if k not in ['rgb','thr']}
                    for mod in ['rgb','thr']:
                        stats['median_stats'][mod]={k:float(np.median([x['stats'][mod][k] for x in available])) for k in available[0]['stats'][mod]}
                    for loss in ['mean_l2','mse']:
                        stats[f'rgb_lower_{loss}_frames']=sum(x['stats'][f'rgb_over_thr_{loss}']<1 for x in available)
                group_summary[group]=stats
            results[rec]['summary'][anchor]={'groups':group_summary,'tails':{}}
            for fraction in ['0.01','0.05']:
                selected=[row['anchors'][anchor]['tails'][fraction] for row in rows]
                stats={}
                for loss in ['mean_l2','mse']:
                    ratios=[x[f'rgb_over_thr_{loss}'] for x in selected]
                    stats[loss]={'median_rgb_over_thr':float(np.median(ratios)),'rgb_lower_frames':sum(x<1 for x in ratios),
                                 'rgb_removed_loss_median':float(np.median([x['camera']['rgb'][loss]['removed_loss_fraction'] for x in selected])),
                                 'thr_removed_loss_median':float(np.median([x['camera']['thr'][loss]['removed_loss_fraction'] for x in selected]))}
                results[rec]['summary'][anchor]['tails'][fraction]=stats
    for path,h in hashes.items():assert sha(path)==h
    protocol={'created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'starting_commit':'47bf0ed8b','script_sha256':sha(__file__),
              'range_lower_edges_m':EDGES,'range_definition':'Euclidean midpoint reference range from RGB origin, fixed across anchors',
              'rgb_native_shape_hw':SHAPE,'shape_source':'EXP044 RGB output table 384x1224, fixed original overlays 1864x424=1224+640 by384+40',
              'groups':'all, 6 ranges, 9 RGB grid cells, 54 joint cells; empty explicit; no minimum-count exclusion',
              'tail_rule':'remove ceil(n*f) largest midpoint max(E_RGB,E_thermal); fractional cutoff ties, fixed across anchors; diagnostic only',
              'tail_removed_fractions':[.01,.05],'no_fit_inference_or_default_change':True,'frozen_input_sha256':hashes}
    output.mkdir(parents=True)
    (output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    (output/'report.json').write_text(json.dumps({'recordings':results},indent=2,allow_nan=False)+'\n')
    print('Completed three recordings / 60 frames / all anchors; inputs:',len(hashes))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();run(a.root,a.output)
