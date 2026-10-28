"""Independent raw-frontend and reciprocal-moment audit; no runner imports."""
import argparse,hashlib,json,statistics
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import numpy as np

def audit(root,run):
    protocol=json.loads((run/'protocol.json').read_text());report=json.loads((run/'report.json').read_text());checks=0;raw_hashes={};total=0
    def close(a,b):
        nonlocal checks
        assert np.allclose(a,b,rtol=1e-8,atol=1e-9),(a,b)
        checks+=int(np.size(a))
    def decision(a,b):return np.array([1. if x<y else 0. if x>y else .5 for x,y in zip(a,b)])
    for rel,h in protocol['input_sha256'].items():assert hashlib.sha256((root/rel).read_bytes()).hexdigest()==h;checks+=1
    for name,rec in report['recordings'].items():
        base=root/rec['path'];dataset=base/'dataset';cal_path=dataset/'calibration.json';raw_hashes[str(cal_path.relative_to(root))]=hashlib.sha256(cal_path.read_bytes()).hexdigest()
        c={k:np.asarray(v['values']) for k,v in json.loads(cal_path.read_text()).items()};frontend=json.loads((dataset/'frontend-run-001/report.json').read_text());expected={p['file']:p['sha256'] for p in frontend['pairs']}
        rows=[]
        for i,row in enumerate(rec['rows']):
            stem=f'{i:06d}';assert row['stem']==stem
            with np.load(base/'metric'/f'metric-{stem}.npz') as f:m={k:f[k] for k in f.files}
            with np.load(base/'support'/f'pairs-{stem}.npz') as f:p={k:f[k] for k in f.files}
            ids=np.flatnonzero(m['both_valid_mask']);n=len(ids);assert row['n']==n;total+=n;data={};percam={}
            for cam in ['rgb','thr']:
                path=dataset/'frontend-run-001'/f'{cam}-stereo-{stem}-{stem}.npz';digest=hashlib.sha256(path.read_bytes()).hexdigest();assert digest==expected[path.name];raw_hashes[str(path.relative_to(root))]=digest
                xy=p[f'{cam}_xy'][ids];u,v=xy.T;k=c[f'K_{cam}L'];fb=-k[0,0]*c[f'T_{cam}R'][0,0]/1000
                with np.load(path) as f:mu=-f['flow'][0,0,v,u].astype(float);variance=f['covariance'][0,0,v,u].astype(float)
                q2=1+((u-k[0,2])/k[0,0])**2+((v-k[1,2])/k[1,1])**2
                g=q2*(fb/mu)**2;cv=variance/(mu*mu);original=g*cv
                close(original,m[f'{cam}_metric_variance'][ids])
                # Independent lognormal reciprocal moments with unit-mean normalization.
                s2=np.log1p(cv.astype(np.longdouble));normrisk=np.expm1(3*s2)-2*np.expm1(s2)
                ln=np.log(g.astype(np.longdouble))+np.log(normrisk)
                alpha=(mu.astype(np.longdouble)**2)/variance.astype(np.longdouble);finite=alpha>2
                ga=np.full(n,np.inf,dtype=np.longdouble)
                a=alpha[finite];m1=a/(a-1);m2=(a/(a-1))*(a/(a-2))
                ga[finite]=np.log(g[finite].astype(np.longdouble))+np.log(m2-2*m1+1)
                percam[cam]={'c':cv,'gamma_finite':finite,'log_first_order':np.log(original),'log_lognormal':ln,'log_gamma':ga}
                data.update({cam+'_'+key:value for key,value in percam[cam].items()})
                fields=row['assumptions'][cam];close(statistics.median(cv),fields['median_c']);assert int((~finite).sum())==fields['gamma_infinite_count'];close((~finite).sum()/n,fields['gamma_infinite_fraction'])
            a,b=percam['rgb'],percam['thr'];choices={'lognormal':decision(a['log_lognormal'],b['log_lognormal']),'gamma':decision(a['log_gamma'],b['log_gamma']),
                'first_order':decision(m['rgb_metric_variance'][ids],m['thr_metric_variance'][ids]),'camera_scale':m['camera_scale'],'geometry_angular':m['geometry_angular'],'geometry_pixel':m['geometry_pixel'],'unpropagated_angular':m['learned_unpropagated_angular'],'always_rgb':np.ones(n),'always_thr':np.zeros(n),'random':np.full(n,.5)}
            for key in ['lognormal','gamma','first_order']:data['choose_'+key]=choices[key]
            with np.load(run/name/f'risk-{stem}.npz') as saved:
                assert set(saved.files)==set(data)
                for key,value in data.items():
                    if key.endswith('log_gamma'):
                        assert np.array_equal(np.isinf(value),np.isinf(saved[key]));mask=np.isfinite(value);close(value[mask],saved[key][mask])
                    elif key.startswith('choose_') or key.endswith('gamma_finite'):assert np.array_equal(value,saved[key]);checks+=len(value)
                    else:close(value,saved[key])
            both=int((~a['gamma_finite']&~b['gamma_finite']).sum());assert both==row['gamma_both_infinite_count'];close(both/n,row['gamma_both_infinite_fraction'])
            for k1,k2 in [('lognormal','first_order'),('gamma','first_order'),('lognormal','gamma')]:close(np.count_nonzero(choices[k1]!=choices[k2])/n,row['disagreement'][k1+'_vs_'+k2])
            for key,value in choices.items():close(np.sum(value)/n,row['rgb_choice_fraction'][key])
            independent={}
            for anchor in ['midpoint','rgb','thr']:
                independent[anchor]={}
                for loss,power in [('mse',2),('mean_l2',1)]:
                    r=np.power(m[f'{anchor}_rgb_error_m'],power);t=np.power(m[f'{anchor}_thr_error_m'],power)
                    independent[anchor][loss]={}
                    for key,ch in choices.items():
                        selected=np.where(ch==1,r,np.where(ch==0,t,(r+t)/2));value=float(np.sum(selected)/n)
                        close(value,row['risks'][anchor][loss][key]);independent[anchor][loss][key]=value
            rows.append(independent)
        for anchor,losses in rec['summary'].items():
            for loss,methods in losses.items():
                for method,controls in methods.items():
                    for control,out in controls.items():
                        ratios=[row[anchor][loss][method]/row[anchor][loss][control] for row in rows]
                        close([statistics.median(ratios),min(ratios),max(ratios)],[out['median'],out['min'],out['max']])
                        assert [sum(x<1 for x in ratios),sum(x==1 for x in ratios),sum(x>1 for x in ratios)]==[out['better'],out['equal'],out['worse']]
        for cam in ['rgb','thr']:
            s=rec['assumption_summary'][cam];close(statistics.median(r['assumptions'][cam]['median_c'] for r in rec['rows']),s['median_c']);assert sum(r['assumptions'][cam]['gamma_infinite_count'] for r in rec['rows'])==s['gamma_infinite_points'];close(statistics.median(r['assumptions'][cam]['gamma_infinite_fraction'] for r in rec['rows']),s['median_gamma_infinite_fraction'])
        assert sum(r['gamma_both_infinite_count'] for r in rec['rows'])==rec['assumption_summary']['both_gamma_infinite_points']
        close(statistics.median(r['gamma_both_infinite_fraction'] for r in rec['rows']),rec['assumption_summary']['median_both_gamma_infinite_fraction'])
        for key,value in rec['median_disagreement'].items():close(statistics.median(r['disagreement'][key] for r in rec['rows']),value)
    for rel,h in protocol['input_sha256'].items():assert hashlib.sha256((root/rel).read_bytes()).hexdigest()==h
    result={'status':'PASS','created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'starting_commit':'52b53e3a0','script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'scalar_checks':checks,'recordings':4,'frames':80,'both_valid_pairs':total,'input_hashes':len(protocol['input_sha256']),'raw_frontend_and_calibration_sha256':raw_hashes,'report_sha256':hashlib.sha256((run/'report.json').read_bytes()).hexdigest(),'scope':'independent raw reconstruction, reciprocal moments, choices, assumed infinities, risks and summaries; no distribution calibration or physical accuracy claim'}
    (run/'validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='raw_frontend_and_calibration_sha256'},indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--run',type=Path,required=True);a=p.parse_args();audit(a.root,a.run)
