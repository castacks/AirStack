"""Own-reference disparity and finite inverse-depth error; descriptive reuse only."""
import argparse,hashlib,json
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo
import numpy as np
from scipy.stats import spearmanr
PATHS={'exp063_rain':'results/ms2-unexamined-recording',**{f'exp064_{c}':f'results/ms2-frozen-suite/{c}' for c in ['day','night','rainy']}}
EDGES=[0,.1,.25,.5,1,2]
RANGES=[0,5,10,20,40,80]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def dump(p,v):p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def correlation(a,b):
    if len(a)<2 or np.ptp(a)==0 or np.ptp(b)==0:return None
    return float(spearmanr(a,b).statistic)
def describe(a,mask):
    n=int(mask.sum())
    if not n:return {'n':0,'stats':None,'bins':None}
    a={k:v[mask] for k,v in a.items()};d=a['delta'];z=a['depth_delta'];ratio=a['ratio'];v=a['variance']
    stats={'mean_signed_disp_px':float(d.mean()),'mean_abs_disp_px':float(np.abs(d).mean()),'mse_disp_px2':float(np.mean(d*d)),
        'mean_signed_depth_m':float(z.mean()),'mse_depth_m2':float(np.mean(z*z)),'mse_linear_depth_m2':float(np.mean(a['linear_depth_delta']**2)),
        'mse_point_m2':float(np.mean(a['point_error_sq'])),'mean_standardized_disp_sq':float(np.mean(d*d/v)),
        'median_pred_over_ref_disparity':float(np.median(ratio)),
        'spearman_variance_disp_sq':correlation(v,d*d),'spearman_variance_depth_sq':correlation(v,z*z),
        'spearman_propagated_point_sq':correlation(a['metric_variance'],a['point_error_sq'])}
    bins={};labels=np.searchsorted(EDGES,ratio,side='right')-1
    for i in range(6):
        keep=labels==i;record={'n':int(keep.sum()),'point_fraction':float(keep.mean())}
        for key,error in [('disparity_sq',d*d),('depth_sq',z*z)]:
            record[key+'_fraction']=float(error[keep].sum()/error.sum()) if error.sum()>0 else None
        bins[str(i)]=record
    assert sum(b['n'] for b in bins.values())==n
    return {'n':n,'stats':stats,'bins':bins}
def summarize(rows):
    result={}
    for cam in ['rgb','thr']:
        result[cam]={}
        for group in rows[0]['cameras'][cam]['groups']:
            items=[r['cameras'][cam]['groups'][group] for r in rows];available=[r for r in items if r['n']]
            summary={'nonempty_frames':len(available),'median_count':float(np.median([r['n'] for r in items])),'stats':{},'bins':{}}
            if available:
                for key in available[0]['stats']:
                    values=[r['stats'][key] for r in available if r['stats'][key] is not None]
                    summary['stats'][key]={'available_frames':len(values),'median':float(np.median(values)) if values else None}
                for i in range(6):
                    summary['bins'][str(i)]={}
                    for key in available[0]['bins'][str(i)]:
                        values=[r['bins'][str(i)][key] for r in available if r['bins'][str(i)][key] is not None]
                        summary['bins'][str(i)][key]={'available_frames':len(values),'median':float(np.median(values)) if values else None}
            result[cam][group]=summary
    return result

def run(root,output):
    if output.exists():raise FileExistsError(output)
    output.mkdir(parents=True);hashes={};records={};inventory={}
    def load(path):
        hashes[str(path.relative_to(root))]=sha(path)
        if path.suffix=='.json':return json.loads(path.read_text())
        with np.load(path,allow_pickle=False) as f:return {k:f[k] for k in f.files}
    for name,relative in PATHS.items():
        base=root/relative;dataset=base/'dataset';raw=dataset/'frontend-run-001'
        inventory[name]={cam:len(list(raw.glob(f'{cam}-stereo-*.npz'))) for cam in ['rgb','thr']}
        assert inventory[name]=={'rgb':20,'thr':20}
        cal={k:np.asarray(v['values'],float) for k,v in load(dataset/'calibration.json').items()}
        frontend=load(raw/'report.json');assert frontend['status']=='PASS'
        expected={item['file']:item['sha256'] for item in frontend['pairs']}
        r=cal['R_nir2thr']@cal['R_nir2rgb'].T;t=(cal['T_nir2thr']-r@cal['T_nir2rgb']).reshape(3)/1000
        rows=[];out=output/name;out.mkdir()
        for i in range(20):
            stem=f'{i:06d}';p=load(base/'support'/f'pairs-{stem}.npz');m=load(base/'metric'/f'metric-{stem}.npz');valid=m['both_valid_mask']
            midpoint=(p['rgb_reference_xyz'][valid]+p['thr_reference_xyz_in_rgb'][valid])*.5
            band=np.searchsorted(RANGES,np.linalg.norm(midpoint,axis=1),side='right')-1
            groups={'all':np.ones(valid.sum(),bool),**{f'range_{j}':band==j for j in range(6)}}
            row={'stem':stem,'n':int(valid.sum()),'cameras':{}}
            for cam in ['rgb','thr']:
                path=raw/f'{cam}-stereo-{stem}-{stem}.npz';a=load(path);assert sha(path)==expected[path.name]
                assert a['source_ids'].tolist()==[stem,stem]
                xy=p[f'{cam}_xy'][valid];x,y=xy.T;k=cal[f'K_{cam}L'];fb=float(k[0,0]*(-cal[f'T_{cam}R'][0,0]/1000))
                d=-a['flow'][0,0,y,x].astype(float);variance=a['covariance'][0,0,y,x].astype(float)
                ref=p['rgb_reference_xyz'][valid] if cam=='rgb' else p['thr_reference_xyz_in_rgb'][valid]@r.T+t
                zref=ref[:,2];dref=fb/zref;z=fb/d;delta=d-dref
                ray=np.column_stack(((x-k[0,2])/k[0,0],(y-k[1,2])/k[1,1],np.ones(len(x))))
                assert np.allclose(ref,ray*zref[:,None],rtol=1e-10,atol=1e-9)
                pred=ray*z[:,None];shared=pred if cam=='rgb' else pred@r-t@r
                assert np.allclose(shared,m['rgb_predicted_xyz' if cam=='rgb' else 'thr_predicted_xyz_in_rgb'][valid],rtol=1e-10,atol=1e-9)
                metric_variance=np.sum(ray**2,axis=1)*(fb/d**2)**2*variance
                assert np.allclose(metric_variance,m[f'{cam}_metric_variance'][valid],rtol=1e-10,atol=1e-9)
                depth_delta=z-zref;exact=-fb*delta/(d*dref);linear=-fb*delta/dref**2
                assert np.allclose(depth_delta,exact,rtol=1e-10,atol=1e-9)
                assert np.allclose(exact,linear*(dref/d),rtol=1e-10,atol=1e-9)
                assert np.all(d>0) and np.all(dref>0) and np.all(variance>0)
                data={'disparity':d,'reference_disparity':dref,'delta':delta,'variance':variance,'reference_depth':zref,'predicted_depth':z,'depth_delta':depth_delta,'linear_depth_delta':linear,'ratio':d/dref,'metric_variance':metric_variance,'point_error_sq':np.sum((pred-ref)**2,axis=1),'range_band':band}
                assert all(np.isfinite(v).all() for v in data.values())
                np.savez_compressed(out/f'{cam}-{stem}.npz',**data)
                row['cameras'][cam]={'groups':{group:describe(data,mask) for group,mask in groups.items()}}
            rows.append(row)
        records[name]={'path':relative,'sequence':load(dataset/'source-manifest.json')['sequence'],'rows':rows,'summary':summarize(rows)}
    for path,digest in hashes.items():assert sha(root/path)==digest
    protocol={'created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'starting_commit':'db453e02d','script_sha256':sha(__file__),'input_sha256':hashes,'recordings':PATHS,'inventory':inventory,'excluded_local_frontends':{'original_day':'only1pair per camera','original_rain':'no complete local fields','original_night':'no complete local fields'},'ratio_bin_lower_edges':EDGES,'range_lower_edges_m':RANGES,'reference':'own supplied endpoint; inverse transform for thermal; implied disparity fx*B/Z','selection':'all existing both-valid pairs; no clipping/fitting/inference','statistics':'Spearman with average ties, null if constant or n<2; equal-frame medians, no capture pooling; raw pixel variance never compared across cameras','scope':'finite-error algebra and ranking, not physical truth/probability calibration or original cross-camera midpoint policy'}
    dump(output/'protocol.json',protocol);dump(output/'report.json',{'recordings':records})
    print(json.dumps({name:{cam:rec['summary'][cam]['all'] for cam in ['rgb','thr']} for name,rec in records.items()},indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.root,a.output)
