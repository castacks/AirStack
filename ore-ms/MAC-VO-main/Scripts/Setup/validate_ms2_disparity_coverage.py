"""Independent raw-field/native-reference coverage and sorted-quantile audit."""
import argparse,hashlib,json,math,statistics
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import cv2
import numpy as np
from scipy.stats import norm

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def quantile(values,p):
    values=sorted(float(x) for x in values);index=(len(values)-1)*p;lo=math.floor(index);hi=math.ceil(index);weight=index-lo
    return values[lo]*(1-weight)+values[hi]*weight

def audit(root,run):
    protocol=json.loads((run/'protocol.json').read_text());report=json.loads((run/'report.json').read_text());checks=0;extra={};total=0
    def close(a,b):
        nonlocal checks
        assert np.allclose(a,b,rtol=1e-10,atol=1e-9),(a,b);checks+=int(np.size(a))
    def load(p):
        extra[str(p.relative_to(root))]=sha(p)
        if p.suffix=='.json':return json.loads(p.read_text())
        with np.load(p,allow_pickle=False) as f:return {k:f[k] for k in f.files}
    for rel,h in protocol['input_sha256'].items():assert sha(root/rel)==h;checks+=1
    for key,value in protocol['hypothetical_zero_mean_gaussian_coverage'].items():close(2*norm.cdf(float(key))-1,value)
    for name,rec in report['recordings'].items():
        base=root/rec['path'];dataset=base/'dataset';cal={k:np.asarray(v['values']) for k,v in load(dataset/'calibration.json').items()};source=load(dataset/'source-manifest.json');frontend=load(dataset/'frontend-run-001/report.json');expected={r['file']:r['sha256'] for r in frontend['pairs']};fresh=[]
        for i,row in enumerate(rec['rows']):
            stem=f'{i:06d}';assert row['stem']==stem;m=load(base/'metric'/f'metric-{stem}.npz');p=load(base/'support'/f'pairs-{stem}.npz');ids=np.flatnonzero(m['both_valid_mask']);n=len(ids);assert row['n']==n;total+=n
            midpoint=(p['rgb_reference_xyz'][ids]+p['thr_reference_xyz_in_rgb'][ids])/2;dist=np.sqrt(np.einsum('ij,ij->i',midpoint,midpoint));bands=np.digitize(dist,[5,10,20,40,80]);new={'cameras':{}}
            for cam in ['rgb','thr']:
                path=dataset/'frontend-run-001'/f'{cam}-stereo-{stem}-{stem}.npz';a=load(path);assert sha(path)==expected[path.name];x,y=p[f'{cam}_xy'][ids].T;d=np.negative(a['flow'][0,0,y,x],dtype=np.float64);v=a['covariance'][0,0,y,x].astype(float)
                entry=source['records'][i]['references'][cam]['depth'];path=root/entry['path'];h=sha(path);assert h==entry['sha256'];extra[entry['path']]=h
                raw=cv2.imread(str(path),cv2.IMREAD_UNCHANGED);assert raw.dtype==np.uint16;zref=raw[y,x].astype(float)/256;fx=cal[f'K_{cam}L'][0,0];baseline=-cal[f'T_{cam}R'][0,0]*.001;ref=fx*baseline/zref
                std=np.sqrt(v);z=(d-ref)/std;cuts=[quantile(v,q) for q in [.25,.5,.75]];labels=np.array([sum(value>=cut for cut in cuts) for value in v]);groups={'all':np.arange(n),**{f'range_{j}':np.flatnonzero(bands==j) for j in range(6)},**{f'variance_{j}':np.flatnonzero(labels==j) for j in range(4)}}
                assert sum(len(groups[f'range_{j}']) for j in range(6))==sum(len(groups[f'variance_{j}']) for j in range(4))==n
                close(cuts,row['cameras'][cam]['variance_cuts_px2'])
                with np.load(run/name/f'{cam}-{stem}.npz') as saved:
                    close(z,saved['standardized_residual']);assert np.array_equal(labels,saved['variance_bin']) and np.array_equal(bands,saved['range_band']);checks+=2*n
                result={}
                for group,sel in groups.items():
                    out=row['cameras'][cam]['groups'][group];assert out['n']==len(sel);checks+=1
                    if not len(sel):assert out['stats'] is None;result[group]={'n':0,'stats':None};continue
                    nn=len(sel);zz=z[sel];vv=v[sel];dd=d[sel];rr=ref[sel];ss=std[sel]
                    mean=lambda values:math.fsum(float(x) for x in values)/nn
                    mu=mean(zz);second=mean(zz*zz);center=mean((zz-mu)**2)
                    stats={'mean_z':mu,'mean_z_squared':second,'centered_z_squared':center,'mean_variance_px2':mean(vv),'mse_disparity_px2':mean((dd-rr)**2),'gaussian_nll':mean(.5*(zz*zz+np.log(vv)+math.log(2*math.pi)))}
                    for q in [.025,.16,.5,.84,.975]:stats[f'z_quantile_{q:g}']=quantile(zz,q)
                    for k in [1.,1.96,2.,3.]:
                        key=f'{k:g}';lower=dd-k*ss;upper=dd+k*ss
                        stats.update({f'coverage_{key}':int(((rr>=lower)&(rr<=upper)).sum())/nn,f'negative_tail_{key}':int((rr>upper).sum())/nn,f'positive_tail_{key}':int((rr<lower).sum())/nn,f'lower_nonpositive_{key}':int((lower<=0).sum())/nn})
                        close(sum(stats[f'{s}_{key}'] for s in ['coverage','negative_tail','positive_tail']),1.)
                    close(mu*mu+center,second)
                    assert set(stats)==set(out['stats'])
                    for key,value in stats.items():close(value,out['stats'][key])
                    result[group]={'n':nn,'stats':stats}
                new['cameras'][cam]={'groups':result}
            fresh.append(new)
        for cam,groups in rec['summary'].items():
            for group,out in groups.items():
                all_items=[row['cameras'][cam]['groups'][group] for row in fresh];items=[r for r in all_items if r['n']];assert len(items)==out['nonempty_frames']
                close([min(r['n'] for r in all_items),statistics.median(r['n'] for r in all_items),max(r['n'] for r in all_items)],out['count_min_median_max'])
                if not items:assert out['stats']=={};continue
                for key,summary in out['stats'].items():
                    values=[r['stats'][key] for r in items];assert summary['available_frames']==len(values);close([statistics.median(values),min(values),max(values)],[summary['median'],summary['min'],summary['max']])
    for rel,h in protocol['input_sha256'].items():assert sha(root/rel)==h
    for rel,h in extra.items():assert sha(root/rel)==h
    result={'status':'PASS','created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'starting_commit':'8bc11c5e2','script_sha256':sha(__file__),'scalar_checks':checks,'recordings':4,'frames':80,'both_valid_pairs':total,'input_hashes':len(protocol['input_sha256']),'additional_raw_reference_sha256':extra,'report_sha256':sha(run/'report.json'),'scope':'raw frontend/native PNG references, direct interval inclusion, independent sorted quantiles/fsum,groups and aggregates; no physical accuracy/calibration guarantee'}
    (run/'validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='additional_raw_reference_sha256'},indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--run',type=Path,required=True);a=p.parse_args();audit(a.root,a.run)
