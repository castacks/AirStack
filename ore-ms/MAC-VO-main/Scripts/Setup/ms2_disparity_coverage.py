"""Empirical supplied-reference interval coverage; no recalibration or inference."""
import argparse,hashlib,json,math
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo
import numpy as np
PATHS={'exp063_rain':'results/ms2-unexamined-recording',**{f'exp064_{c}':f'results/ms2-frozen-suite/{c}' for c in ['day','night','rainy']}}
WIDTHS=[1.,1.96,2.,3.]
QUANTILES=[.025,.16,.5,.84,.975]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def dump(p,v):p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def groups(variance,band):
    cuts=np.quantile(variance,[.25,.5,.75],method='linear');labels=np.searchsorted(cuts,variance,side='right')
    masks={'all':np.ones(len(variance),bool),**{f'range_{j}':band==j for j in range(6)},**{f'variance_{j}':labels==j for j in range(4)}}
    assert sum(int(masks[f'range_{j}'].sum()) for j in range(6))==len(variance)
    assert sum(int(masks[f'variance_{j}'].sum()) for j in range(4))==len(variance)
    return cuts,labels,masks

def describe(d,ref,variance,mask):
    n=int(mask.sum())
    if not n:return {'n':0,'stats':None}
    d,ref,variance=d[mask],ref[mask],variance[mask];delta=d-ref;z=delta/np.sqrt(variance)
    mean=float(z.mean());spread=float(((z-mean)**2).mean());second=float((z*z).mean())
    assert np.isclose(second,mean*mean+spread,rtol=1e-12,atol=1e-12)
    stats={'mean_z':mean,'mean_z_squared':second,'centered_z_squared':spread,'mean_variance_px2':float(variance.mean()),'mse_disparity_px2':float((delta*delta).mean()),'gaussian_nll':float((.5*(z*z+np.log(2*math.pi*variance))).mean())}
    for q,value in zip(QUANTILES,np.quantile(z,QUANTILES,method='linear')):stats[f'z_quantile_{q:g}']=float(value)
    for k in WIDTHS:
        key=f'{k:g}';coverage=float((np.abs(z)<=k).mean());negative=float((z<-k).mean());positive=float((z>k).mean())
        assert np.isclose(coverage+negative+positive,1.,rtol=0,atol=1e-14)
        stats.update({f'coverage_{key}':coverage,f'negative_tail_{key}':negative,f'positive_tail_{key}':positive,f'lower_nonpositive_{key}':float((d-k*np.sqrt(variance)<=0).mean())})
    assert all(np.isfinite(value) for value in stats.values())
    return {'n':n,'stats':stats}

def summarize(rows):
    result={}
    for cam in ['rgb','thr']:
        result[cam]={}
        for group in rows[0]['cameras'][cam]['groups']:
            frames=[r['cameras'][cam]['groups'][group] for r in rows];available=[r for r in frames if r['n']]
            summary={'nonempty_frames':len(available),'count_min_median_max':[min(r['n'] for r in frames),float(np.median([r['n'] for r in frames])),max(r['n'] for r in frames)],'stats':{}}
            if available:
                for key in available[0]['stats']:
                    values=[r['stats'][key] for r in available];summary['stats'][key]={'available_frames':len(values),'median':float(np.median(values)),'min':min(values),'max':max(values)}
            result[cam][group]=summary
    return result

def run(root,derived,output):
    if output.exists():raise FileExistsError(output)
    output.mkdir(parents=True);hashes={};records={}
    def load(path):
        hashes[str(path.relative_to(root))]=sha(path)
        if path.suffix=='.json':return json.loads(path.read_text())
        with np.load(path,allow_pickle=False) as f:return {k:f[k] for k in f.files}
    parent=load(derived/'protocol.json');validation=load(derived/'validation.json');assert parent['recordings']==PATHS and validation['status']=='PASS' and validation['both_valid_pairs']==106562
    for rel,digest in parent['input_sha256'].items():assert sha(root/rel)==digest
    for name,relative in PATHS.items():
        rows=[];(output/name).mkdir()
        for i in range(20):
            stem=f'{i:06d}';row={'stem':stem,'cameras':{}};n=None
            for cam in ['rgb','thr']:
                a=load(derived/name/f'{cam}-{stem}.npz');d,ref,v=a['disparity'],a['reference_disparity'],a['variance']
                assert np.isfinite(d).all() and np.isfinite(ref).all() and np.isfinite(v).all() and np.all(d>0) and np.all(ref>0) and np.all(v>0)
                if n is None:n=len(d)
                else:assert len(d)==n
                cuts,labels,masks=groups(v,a['range_band']);z=(d-ref)/np.sqrt(v)
                row['cameras'][cam]={'variance_cuts_px2':cuts.tolist(),'groups':{group:describe(d,ref,v,mask) for group,mask in masks.items()}}
                np.savez_compressed(output/name/f'{cam}-{stem}.npz',standardized_residual=z,variance_bin=labels,range_band=a['range_band'])
            row['n']=n;rows.append(row)
        records[name]={'path':relative,'rows':rows,'summary':summarize(rows)}
    for rel,digest in hashes.items():assert sha(root/rel)==digest
    for rel,digest in parent['input_sha256'].items():assert sha(root/rel)==digest
    protocol={'created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'starting_commit':'8bc11c5e2','script_sha256':sha(__file__),'input_sha256':hashes,'recordings':PATHS,'widths_standard_deviations':WIDTHS,'hypothetical_zero_mean_gaussian_coverage':{f'{k:g}':math.erf(k/math.sqrt(2)) for k in WIDTHS},'z_quantiles':QUANTILES,'intervals':'predicted disparity +/- k*sqrt(horizontal variance); inclusive bounds, no truncation','groups':'all + six fixed midpoint reference ranges + four per-frame variance quantile bins; linear .25/.5/.75 cuts, right insertion keeps exact ties together, empties explicit','aggregation':'equal-frame medians/min/max and availability per recording; no pooled camera or capture risk','interpretation':'empirical coverage against supplied native depth-implied disparity, not physical accuracy or demonstrated Gaussian calibration; no fitting,clipping,inference,default change'}
    dump(output/'protocol.json',protocol);dump(output/'report.json',{'recordings':records})
    print(json.dumps({name:{cam:rec['summary'][cam]['all'] for cam in ['rgb','thr']} for name,rec in records.items()},indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--derived',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.root,a.derived,a.output)
