"""Independent raw-array, second-moment and aggregation audit (no runner imports)."""
import argparse,json,hashlib,statistics
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import numpy as np

def audit(root,run):
    protocol=json.loads((run/'protocol.json').read_text());report=json.loads((run/'report.json').read_text());checks=0
    def close(a,b):
        nonlocal checks
        assert np.allclose(a,b,rtol=1e-10,atol=1e-8),(a,b)
        checks+=int(np.size(a))
    for path,digest in protocol['input_sha256'].items():
        assert hashlib.sha256((root/path).read_bytes()).hexdigest()==digest;checks+=1
    total=0
    for name,rec in report['recordings'].items():
        assert len(rec['rows'])==20
        for index,row in enumerate(rec['rows']):
            assert row['stem']==f'{index:06d}'
            with np.load(root/rec['path']/'metric'/f'metric-{index:06d}.npz') as a,np.load(root/rec['path']/'support'/f'pairs-{index:06d}.npz') as p:
                ids=np.flatnonzero(a['both_valid_mask']);assert len(ids)==row['n'];total+=len(ids)
                r=p['rgb_reference_xyz'][ids];t=p['thr_reference_xyz_in_rgb'][ids];mid=(r+t)*.5
                distances=np.sqrt(np.sum(mid*mid,axis=1));xy=p['rgb_xy'][ids]
                bands=np.digitize(distances,[5,10,20,40,80]);cells=(xy[:,1]//128)*3+(xy[:,0]//408)
                groups={'all':np.arange(len(ids))}
                groups.update({f'range_{j}':np.flatnonzero(bands==j) for j in range(6)})
                groups.update({f'cell_{j}':np.flatnonzero(cells==j) for j in range(9)})
                groups.update({f'joint_{j}_{k}':np.flatnonzero((bands==j)&(cells==k)) for j in range(6) for k in range(9)})
                for anchor,ref in [('midpoint',mid),('rgb',r),('thr',t)]:
                    for cam,key in [('rgb','rgb_predicted_xyz'),('thr','thr_predicted_xyz_in_rgb')]:
                        e=np.subtract(a[key][ids],ref,dtype=np.float64);out=row['anchors'][anchor][cam]
                        for group,selected in groups.items():
                            record=out['groups'][group];assert record['n']==len(selected);checks+=1
                            if not len(selected):assert record['stats'] is None;continue
                            values=e[selected];n=len(values);mu=np.sum(values,axis=0)/n
                            second=np.einsum('ij,ij->j',values,values)/n
                            centered=second-mu*mu;s=record['stats'];mse=float(second.sum());offset=float(mu@mu)
                            close(mu,s['mean_xyz_m']);close(second,s['axis_mse_m2']);close(centered,s['axis_centered_m2'])
                            close([mse,offset,float(centered.sum()),offset/mse],[s['mse_m2'],s['mean_vector_squared_m2'],s['centered_m2'],s['global_mean_fraction']])
                            close(np.sqrt(np.sum(values*values,axis=1)).sum()/n,s['mean_l2_m'])
                        for partition,count in [('range',6),('cell',9),('joint',54)]:
                            labels=bands if partition=='range' else cells if partition=='cell' else bands*9+cells
                            grand=np.sum(e,axis=0)/len(e);within=0.;between=0.;means=0.
                            for j in range(count):
                                v=e[labels==j]
                                if not len(v):continue
                                mu=v.sum(axis=0)/len(v)
                                within+=np.sum((v-mu)**2)/len(e)
                                between+=len(v)*np.sum((mu-grand)**2)/len(e)
                                means+=len(v)*np.sum(mu**2)/len(e)
                            close([within,between,means,means/out['mse_m2']],[out[f'{partition}_within_m2'],out[f'{partition}_between_m2'],out[f'{partition}_means_m2'],out[f'{partition}_mean_fraction']])
                            close(within+between+np.sum(grand**2),out['mse_m2'])
        for anchor,summary in rec['summary'].items():
            for cam in ['rgb','thr']:
                for key,value in summary[cam].items():close(statistics.median(row['anchors'][anchor][cam][key] for row in rec['rows']),value)
            pairs=[(row['anchors'][anchor]['rgb'],row['anchors'][anchor]['thr']) for row in rec['rows']]
            for key in ['mse_m2','centered_m2']:
                close(statistics.median(a[key]/b[key] for a,b in pairs),summary[f'median_rgb_over_thr_{key}'])
                assert sum(a[key]<b[key] for a,b in pairs)==summary[f'rgb_lower_{key}_frames']
                assert sum(a[key]==b[key] for a,b in pairs)==summary[f'tie_{key}_frames']
            assert sum(np.sign(a['mse_m2']-b['mse_m2'])==-np.sign(a['centered_m2']-b['centered_m2']) and a['mse_m2']!=b['mse_m2'] and a['centered_m2']!=b['centered_m2'] for a,b in pairs)==summary['strict_order_reversal_frames']
    output={'status':'PASS','created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'starting_commit':'e0de6922c','auditor_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'scalar_checks':checks,'recordings':len(report['recordings']),'frame_pairs':140,'both_valid_pairs':total,'input_hashes':len(protocol['input_sha256']),'report_sha256':hashlib.sha256((run/'report.json').read_bytes()).hexdigest()}
    (run/'validation.json').write_text(json.dumps(output,indent=2)+'\n');print(json.dumps(output,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--run',type=Path,required=True);a=p.parse_args();audit(a.root,a.run)
