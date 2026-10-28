"""Reference-assisted spatial residual accounting, never a deployment policy."""
import argparse,json,hashlib
from datetime import datetime
from zoneinfo import ZoneInfo
from pathlib import Path
import numpy as np
from .ms2_error_structure import masks
PATHS={'original_rain':'results/ms2-day-cross-modal-replication/rain-control','original_day':'results/ms2-day-cross-modal-replication/day','original_night':'results/ms2-night-cross-modal-replication/night','exp063_rain':'results/ms2-unexamined-recording',**{f'exp064_{c}':f'results/ms2-frozen-suite/{c}' for c in ['day','night','rainy']}}
ANCHORS=['midpoint','rgb','thr']
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def describe(e):
    if not len(e):return {'n':0,'stats':None}
    mu=e.mean(axis=0);axes=(e**2).mean(axis=0);spread=((e-mu)**2).mean(axis=0)
    mse=float(axes.sum());offset=float(mu@mu);center=float(spread.sum())
    assert np.isclose(mse,offset+center,rtol=1e-10,atol=1e-8)
    return {'n':len(e),'stats':{'mean_xyz_m':mu.tolist(),'axis_mse_m2':axes.tolist(),'axis_centered_m2':spread.tolist(),'mse_m2':mse,'mean_l2_m':float(np.linalg.norm(e,axis=1).mean()),'mean_vector_squared_m2':offset,'centered_m2':center,'global_mean_fraction':offset/mse if mse else None}}
def summarize(rows,anchor):
    result={}
    for cam in ['rgb','thr']:
        frames=[r['anchors'][anchor][cam] for r in rows]
        result[cam]={k:float(np.median([f[k] for f in frames])) for k in ['global_mean_fraction','joint_mean_fraction','range_mean_fraction','cell_mean_fraction','mse_m2','centered_m2']}
    pairs=[(r['anchors'][anchor]['rgb'],r['anchors'][anchor]['thr']) for r in rows]
    for k in ['mse_m2','centered_m2']:
        result[f'median_rgb_over_thr_{k}']=float(np.median([a[k]/b[k] for a,b in pairs]))
        result[f'rgb_lower_{k}_frames']=sum(a[k]<b[k] for a,b in pairs)
        result[f'tie_{k}_frames']=sum(a[k]==b[k] for a,b in pairs)
    result['strict_order_reversal_frames']=sum((a['mse_m2']-b['mse_m2'])*(a['centered_m2']-b['centered_m2'])<0 for a,b in pairs)
    return result
def run(root,output):
    if output.exists():raise FileExistsError(output)
    hashes={};recordings={}
    for rec,rel in PATHS.items():
        rows=[]
        for i in range(20):
            arrays=[];stem=f'{i:06d}'
            for path in [root/rel/'metric'/f'metric-{stem}.npz',root/rel/'support'/f'pairs-{stem}.npz']:
                hashes[str(path.relative_to(root))]=sha(path)
                with np.load(path,allow_pickle=False) as f:arrays.append({k:f[k] for k in f.files})
            a,p=arrays;valid=a['both_valid_mask'];refs={'rgb':p['rgb_reference_xyz'][valid],'thr':p['thr_reference_xyz_in_rgb'][valid]};refs['midpoint']=(refs['rgb']+refs['thr'])/2
            _,groups=masks(refs['midpoint'],p['rgb_xy'][valid]);row={'stem':stem,'n':int(valid.sum()),'anchors':{}}
            for anchor in ANCHORS:
                row['anchors'][anchor]={}
                for cam,key in [('rgb','rgb_predicted_xyz'),('thr','thr_predicted_xyz_in_rgb')]:
                    e=a[key][valid].astype(np.float64)-refs[anchor]
                    assert e.shape==(row['n'],3) and np.all(np.isfinite(e))
                    stats={name:describe(e[mask]) for name,mask in groups.items()};total=dict(stats['all']['stats'])
                    for part,names in [('range',[f'range_{j}' for j in range(6)]),('cell',[f'cell_{j}' for j in range(9)]),('joint',[f'joint_{j}_{k}' for j in range(6) for k in range(9)])]:
                        assert sum(stats[g]['n'] for g in names)==row['n']
                        within=sum(stats[g]['n']*stats[g]['stats']['centered_m2'] for g in names if stats[g]['n'])/row['n']
                        means=sum(stats[g]['n']*stats[g]['stats']['mean_vector_squared_m2'] for g in names if stats[g]['n'])/row['n']
                        assert np.isclose(total['mse_m2'],within+means,rtol=1e-10,atol=1e-8)
                        assert means+1e-8>=total['mean_vector_squared_m2']
                        total.update({f'{part}_within_m2':within,f'{part}_means_m2':means,f'{part}_between_m2':means-total['mean_vector_squared_m2'],f'{part}_mean_fraction':means/total['mse_m2']})
                    total['groups']=stats;row['anchors'][anchor][cam]=total
            rows.append(row)
        recordings[rec]={'path':rel,'rows':rows,'summary':{a:summarize(rows,a) for a in ANCHORS}}
    for p,h in hashes.items():assert sha(root/p)==h
    output.mkdir(parents=True)
    protocol={'created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'starting_commit':'e0de6922c','script_sha256':sha(__file__),'group_helper_sha256':sha(Path(__file__).with_name('ms2_error_structure.py')),'input_sha256':hashes,'recordings':PATHS,'anchors':ANCHORS,'residual':'predicted minus reference XYZ in RGB frame; both-valid; float64','groups':'all + 6 midpoint ranges [0,5,10,20,40,80,infinity) + 9 RGB 3x3 cells + 54 joint cells; empties explicit','interpretation':'Spatial empirical mean, not physical/stochastic bias. Population spread, reference-assisted centering. No fit/filter/inference/policy change.'}
    (output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n');(output/'report.json').write_text(json.dumps({'recordings':recordings},indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v['summary']['midpoint'] for k,v in recordings.items()},indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.root,a.output)
