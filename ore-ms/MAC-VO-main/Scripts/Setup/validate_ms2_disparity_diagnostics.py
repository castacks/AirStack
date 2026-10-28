"""Independent audit using native PNG depth and explicit average-rank correlation."""
import argparse,json,hashlib,statistics
from datetime import datetime
from zoneinfo import ZoneInfo
from pathlib import Path
import cv2
import numpy as np
from scipy.stats import rankdata

def audit(root,run):
    protocol=json.loads((run/'protocol.json').read_text());report=json.loads((run/'report.json').read_text());checks=0;reference_hashes={};total=0;audited={}
    def close(a,b):
        nonlocal checks
        if a is None or b is None:assert a is None and b is None;checks+=1;return
        assert np.allclose(a,b,rtol=1e-9,atol=1e-8),(a,b)
        checks+=int(np.size(a))
    def rho(x,y):
        if len(x)<2 or min(x)==max(x) or min(y)==max(y):return None
        x=rankdata(x,method='average');y=rankdata(y,method='average');x-=x.mean();y-=y.mean()
        return float(np.dot(x,y)/np.sqrt(np.dot(x,x)*np.dot(y,y)))
    def group_stats(a,ids):
        if not len(ids):return {'n':0,'stats':None,'bins':None}
        a={k:v[ids] for k,v in a.items()};d=a['delta'];z=a['depth_delta'];v=a['variance'];n=len(ids)
        mean=lambda x:float(np.sum(x)/n)
        stats={'mean_signed_disp_px':mean(d),'mean_abs_disp_px':mean(abs(d)),'mse_disp_px2':mean(d**2),
            'mean_signed_depth_m':mean(z),'mse_depth_m2':mean(z**2),'mse_linear_depth_m2':mean(a['linear_depth_delta']**2),
            'mse_point_m2':mean(a['point_error_sq']),'mean_standardized_disp_sq':mean(d**2/v),
            'median_pred_over_ref_disparity':statistics.median(a['ratio']),
            'spearman_variance_disp_sq':rho(v,d*d),'spearman_variance_depth_sq':rho(v,z*z),
            'spearman_propagated_point_sq':rho(a['metric_variance'],a['point_error_sq'])}
        bins={};labels=np.digitize(a['ratio'],[.1,.25,.5,1,2])
        for i in range(6):
            keep=labels==i;bins[str(i)]={'n':int(keep.sum()),'point_fraction':float(keep.sum()/n),
                'disparity_sq_fraction':float(np.sum(d[keep]**2)/np.sum(d**2)) if np.sum(d**2)>0 else None,
                'depth_sq_fraction':float(np.sum(z[keep]**2)/np.sum(z**2)) if np.sum(z**2)>0 else None}
        return {'n':n,'stats':stats,'bins':bins}
    for path,h in protocol['input_sha256'].items():assert hashlib.sha256((root/path).read_bytes()).hexdigest()==h;checks+=1
    for name,rec in report['recordings'].items():
        base=root/rec['path'];dataset=base/'dataset';source=json.loads((dataset/'source-manifest.json').read_text());c={k:np.array(v['values']) for k,v in json.loads((dataset/'calibration.json').read_text()).items()}
        rotation=c['R_nir2thr'].dot(c['R_nir2rgb'].T);translation=(c['T_nir2thr']-rotation.dot(c['T_nir2rgb'])).ravel()/1000
        rows=[]
        assert len(rec['rows'])==20
        for i,row in enumerate(rec['rows']):
            stem=f'{i:06d}';assert row['stem']==stem
            with np.load(base/'metric'/f'metric-{stem}.npz') as f:m={k:f[k] for k in f.files}
            with np.load(base/'support'/f'pairs-{stem}.npz') as f:p={k:f[k] for k in f.files}
            valid=np.flatnonzero(m['both_valid_mask']);n=len(valid);assert row['n']==n;total+=n
            midpoint=(p['rgb_reference_xyz'][valid]+p['thr_reference_xyz_in_rgb'][valid])/2
            bands=np.digitize(np.sqrt(np.einsum('ij,ij->i',midpoint,midpoint)),[5,10,20,40,80]);groups={'all':np.arange(n),**{f'range_{j}':np.flatnonzero(bands==j) for j in range(6)}}
            fresh={'cameras':{}}
            for cam in ['rgb','thr']:
                k=c[f'K_{cam}L'];fb=float(-k[0,0]*c[f'T_{cam}R'][0,0]*.001);xy=p[f'{cam}_xy'][valid];u,v=xy.T
                entry=source['records'][i]['references'][cam]['depth'];depth_path=root/entry['path'];digest=hashlib.sha256(depth_path.read_bytes()).hexdigest();assert digest==entry['sha256'];reference_hashes[entry['path']]=digest
                raw_depth=cv2.imread(str(depth_path),cv2.IMREAD_UNCHANGED);assert raw_depth.dtype==np.uint16
                zref=raw_depth[v,u].astype(float)/256.;assert np.all(zref>0)
                with np.load(dataset/'frontend-run-001'/f'{cam}-stereo-{stem}-{stem}.npz') as f:
                    d=np.negative(f['flow'][0,0,v,u],dtype=np.float64);var=f['covariance'][0,0,v,u].astype(float)
                dref=fb/zref;pred_z=fb/d;delta=d-dref;ray=np.stack([(u-k[0,2])/k[0,0],(v-k[1,2])/k[1,1],np.ones(n)],axis=1)
                pred=ray*pred_z[:,None];ref=ray*zref[:,None]
                native_support=p['rgb_reference_xyz'][valid] if cam=='rgb' else (p['thr_reference_xyz_in_rgb'][valid]+translation@rotation)@np.linalg.inv(rotation)
                close(ref,native_support)
                pred_shared=pred if cam=='rgb' else (pred-translation)@rotation
                close(pred_shared,m['rgb_predicted_xyz' if cam=='rgb' else 'thr_predicted_xyz_in_rgb'][valid])
                metric_var=np.einsum('ij,ij->i',ray,ray)*(fb*fb)*var/(d**4)
                close(metric_var,m[f'{cam}_metric_variance'][valid])
                values={'disparity':d,'reference_disparity':dref,'delta':delta,'variance':var,'reference_depth':zref,'predicted_depth':pred_z,
                    'depth_delta':pred_z-zref,'linear_depth_delta':(-zref/dref)*delta,'ratio':d/dref,'metric_variance':metric_var,
                    'point_error_sq':np.sum((pred-ref)**2,axis=1),'range_band':bands}
                close(values['depth_delta'],(-fb/d)*(delta/dref));close(values['depth_delta'],values['linear_depth_delta']/values['ratio'])
                with np.load(run/name/f'{cam}-{stem}.npz') as saved:
                    assert set(saved.files)==set(values)
                    for key,expected in values.items():close(expected,saved[key])
                result={group:group_stats(values,ids) for group,ids in groups.items()};fresh['cameras'][cam]={'groups':result}
                for group,expected in result.items():
                    out=row['cameras'][cam]['groups'][group];assert out['n']==expected['n'];checks+=1
                    if not expected['n']:assert out['stats'] is None and out['bins'] is None;continue
                    for key,value in expected['stats'].items():close(value,out['stats'][key])
                    for b,fields in expected['bins'].items():
                        for key,value in fields.items():close(value,out['bins'][b][key])
            rows.append(fresh)
        audited[name]=rows
        for cam,groups_summary in rec['summary'].items():
            for group,summary in groups_summary.items():
                groups=[r['cameras'][cam]['groups'][group] for r in rows];available=[g for g in groups if g['n']]
                assert summary['nonempty_frames']==len(available);close(statistics.median(g['n'] for g in groups),summary['median_count'])
                if not available:assert not summary['stats'] and not summary['bins'];continue
                for key,out in summary['stats'].items():
                    values=[g['stats'][key] for g in available if g['stats'][key] is not None]
                    assert out['available_frames']==len(values);close(statistics.median(values) if values else None,out['median'])
                for b,fields in summary['bins'].items():
                    for key,out in fields.items():
                        values=[g['bins'][b][key] for g in available if g['bins'][b][key] is not None]
                        assert out['available_frames']==len(values);close(statistics.median(values) if values else None,out['median'])
    for path,h in protocol['input_sha256'].items():assert hashlib.sha256((root/path).read_bytes()).hexdigest()==h
    result={'status':'PASS','created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'starting_commit':'db453e02d','script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'scalar_checks':checks,'recordings':4,'frames':80,'both_valid_pairs':total,'input_hashes':len(protocol['input_sha256']),'native_depth_png_hashes':reference_hashes,'report_sha256':hashlib.sha256((run/'report.json').read_bytes()).hexdigest(),'scope':'native PNG references and raw frontend reconstruction, scalar algebra, independent average-rank correlations, groups, bins and summaries; not physical accuracy'}
    (run/'validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='native_depth_png_hashes'},indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--run',type=Path,required=True);a=p.parse_args();audit(a.root,a.run)
