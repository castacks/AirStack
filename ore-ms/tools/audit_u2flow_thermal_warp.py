"""Independent saved-image/known-warp metric audit; no Torch/producer imports."""
import argparse,json,hashlib
from pathlib import Path
import cv2
import numpy as np
from scipy.stats import rankdata


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def corr(x,y):
    if np.ptp(x)==0 or np.ptp(y)==0:return None
    a=rankdata(x);b=rankdata(y);a-=a.mean();b-=b.mean();return float(a@b/np.sqrt((a@a)*(b@b)))
def main(protocol,folder):
    plan=json.loads(Path(protocol).read_text());out=Path(folder);scores=json.loads((out/'scores.json').read_text());lock=json.loads((out/'prediction-lock.json').read_text());assert sha(out/'prediction-lock.json')==scores['prediction_lock_sha256']
    report={'status':'PASS','frames':0,'case_metrics':0,'pair_metrics':0,'checks':1,'max_metric_difference':0.,'scope':'Independent raw recipe, fixed masks, known displacements, EPE/rank/AUSE/disagreement reconstruction; no inference rerun/natural-label qualification.'};lookup={(r['condition'],r['stem']):r for r in plan['inputs']}
    def near(x,y):
        if x is None or y is None:assert x is None and y is None
        else:
            delta=abs(float(x)-float(y));report['max_metric_difference']=max(report['max_metric_difference'],delta);assert delta<1e-9,(x,y)
        report['checks']+=1
    for row in scores['rows']:
        path=out/row['path'];assert sha(path)==row['sha256'];meta=lookup[row['condition'],row['stem']];rawpath=Path(plan['project_root'])/meta['path'];assert sha(rawpath)==meta['sha256']
        raw=cv2.imread(str(rawpath),cv2.IMREAD_UNCHANGED);assert raw.dtype==np.uint16
        flat=raw.reshape(-1);loindex=round(len(flat)*.01);hiindex=round(len(flat)*.99)-1;lo,hi=np.partition(flat,[loindex,hiindex])[[loindex,hiindex]]
        u8=((np.minimum(np.maximum(raw,lo),hi)-lo)/(hi-lo)*255).astype(np.uint8)
        u8=cv2.bilateralFilter(cv2.createCLAHE(clipLimit=2.,tileGridSize=(8,8)).apply(u8),5,20,15);u8=cv2.resize(u8,(320,128),interpolation=cv2.INTER_AREA)
        with np.load(path) as z:stored=z['source_uint8'];flows=z['flow'];alpha=z['alpha']
        assert np.array_equal(u8,stored);assert np.isfinite(flows).all() and np.isfinite(alpha).all();assert flows.shape==(4,2,128,320) and alpha.shape==(4,1,128,320)
        report['frames']+=1;report['checks']+=5
        for i,name in enumerate(plan['cases']):
            dx,dy=(0,0) if i%2==0 else (4,-3);v,u=np.indices((128,320));mask=(u+dx>=0)&(u+dx<320)&(v+dy>=0)&(v+dy<128)
            fx,fy=flows[i].astype(float);errors=np.sqrt((fx[mask]-dx)**2+(fy[mask]-dy)**2);unc=alpha[i,0][mask].astype(float);m=row['cases'][name]
            expected={'pixels':len(errors),'finite_fraction':1.,'epe_mean':errors.mean(),'epe_rmse':np.sqrt((errors**2).mean()),'success1px':np.count_nonzero(errors<=1)/len(errors),'success3px':np.count_nonzero(errors<=3)/len(errors),'alpha_epe_spearman':corr(unc,errors)}
            order=np.argsort(unc,kind='stable');best=np.sort(errors);ret=np.linspace(1,.05,20);gaps=[]
            for fraction in ret:
                n=max(1,int(np.floor(fraction*len(errors))));gaps.append((errors[order[:n]].mean()-best[:n].mean())/errors.mean())
            # Explicit trapezoid sum instead of producer cumulative prefixes/trapz.
            expected['normalized_ause_0to95']=float(sum((gaps[j]+gaps[j+1])*.5*(ret[j]-ret[j+1]) for j in range(19))) if errors.mean()>0 else None
            for k,x in expected.items():near(x,m[k])
            report['case_metrics']+=1
        for i,name in [(0,'identity'),(1,'shift')]:
            mask=np.ones((128,320),bool) if i==0 else (np.indices((128,320))[1]<316)&(np.indices((128,320))[0]>=3);dx,dy=(0,0) if i==0 else (4,-3)
            e=np.sqrt((flows[i+2,0].astype(float)-dx)**2+(flows[i+2,1].astype(float)-dy)**2)[mask]
            d=np.sqrt(np.sum((flows[i+2].astype(float)-flows[i].astype(float))**2,axis=0))[mask];bad=e>1;wrong=bad&(d<.1);m=row['pairs'][name]
            expected={'disagreement_epe_spearman':corr(d,e),'disagreement_mean_px':d.mean(),'error_gt1_count':int(bad.sum()),'consistent_wrong_count':int(wrong.sum()),'consistent_wrong_fraction_all':wrong.sum()/len(e),'consistent_wrong_fraction_bad':wrong.sum()/bad.sum() if bad.sum() else None,'pixels':len(e)}
            for k,x in expected.items():near(x,m[k])
            report['pair_metrics']+=1
    assert report['frames']==60 and report['case_metrics']==240 and report['pair_metrics']==120
    for c,s in scores['recordings'].items():
      rows=[r for r in scores['rows'] if r['condition']==c]
      for section in ['cases','pairs']:
       for name,metrics in s[section].items():
        for k,v in metrics.items():
         values=[r[section][name][k] for r in rows if r[section][name][k] is not None];assert len(values)==v['valid_frames'];near(np.mean(values) if values else None,v['mean'])
    (out/'independent-audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--protocol',required=True);p.add_argument('--output',required=True);a=p.parse_args();main(a.protocol,a.output)
