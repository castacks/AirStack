"""Independent geometric/range/grid/tail audit of the saved-array diagnostic."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np


def validate(root,source,output):
    if output.exists():raise FileExistsError(output)
    p=json.loads((source/'protocol.json').read_text());report=json.loads((source/'report.json').read_text())
    counts={'hashes':0,'reconstructed_error_values':0,'group_fields':0,'tail_fields':0,'summary_fields':0,'historical_risks':0,'partition_gap_checks':0}
    def check(a,b,category):
        assert np.isclose(a,b,rtol=1e-10,atol=1e-10),(category,a,b)
        counts[category]+=1
    assert p['range_lower_edges_m']==[0,5,10,20,40,80] and p['rgb_native_shape_hw']==[384,1224]
    assert len(p['frozen_input_sha256'])==126
    for path,h in p['frozen_input_sha256'].items():
        assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==h;counts['hashes']+=1
    paths={'rain':'ms2-day-cross-modal-replication/rain-control','day':'ms2-day-cross-modal-replication/day','night':'ms2-night-cross-modal-replication/night'}
    for rec,sub in paths.items():
        folder=root/'results'/sub
        old=json.loads((folder/'metric/report.json').read_text())['rows']
        rows=report['recordings'][rec]['rows'];assert len(rows)==20
        for i,row in enumerate(rows):
            assert row['stem']==old[i]['stem']==f'{i:06d}'
            with np.load(folder/'metric'/f'metric-{i:06d}.npz') as f:a={k:f[k] for k in f.files}
            with np.load(folder/'support'/f'pairs-{i:06d}.npz') as f:b={k:f[k] for k in f.files}
            valid=a['both_valid_mask'];n=int(valid.sum());assert row['n']==n
            rp=b['rgb_reference_xyz'][valid];tp=b['thr_reference_xyz_in_rgb'][valid];mid=(rp+tp)/2
            d=np.array([math.sqrt(float(x*x+y*y+z*z)) for x,y,z in mid]);assert (d>0).all()
            # Scalar predicates independently assign lower-inclusive/upper-exclusive ranges/grid.
            bands=np.array([next(j for j,(lo,hi) in enumerate(zip([0,5,10,20,40,80],[5,10,20,40,80,math.inf])) if lo<=v<hi) for v in d])
            xy=b['rgb_xy'][valid]
            cells=np.array([int(y//128)*3+int(x//408) for x,y in xy])
            assert (cells>=0).all() and (cells<9).all()
            groups={'all':np.ones(n,bool)}
            for j in range(6):groups[f'range_{j}']=bands==j
            for j in range(9):groups[f'cell_{j}']=cells==j
            for j in range(6):
                for k in range(9):groups[f'joint_{j}_{k}']=(bands==j)&(cells==k)
            for prefix in ['range_','cell_','joint_']:
                assert np.array_equal(sum(v.astype(int) for k,v in groups.items() if k.startswith(prefix)),np.ones(n,int));counts['partition_gap_checks']+=1
            # Recompute 3D errors from saved predicted/reference points, not saved errors alone.
            for anchor,target in [('midpoint',mid),('rgb',rp),('thr',tp)]:
                for mod,key in [('rgb','rgb_predicted_xyz'),('thr','thr_predicted_xyz_in_rgb')]:
                    e=np.sqrt(np.sum((a[key][valid]-target)**2,axis=1))
                    assert np.allclose(e,a[f'{anchor}_{mod}_error_m'],rtol=1e-12,atol=1e-12)
                    counts['reconstructed_error_values']+=n
                r,t=a[f'{anchor}_rgb_error_m'],a[f'{anchor}_thr_error_m']
                for name,mask in groups.items():
                    v=row['anchors'][anchor]['groups'][name];k=int(mask.sum());assert v['n']==k
                    check(v['fraction'],k/n,'group_fields')
                    if not k:assert v['stats'] is None;continue
                    stats=v['stats'];rr,tt,dd=r[mask],t[mask],d[mask]
                    check(stats['median_reference_range_m'],np.median(dd),'group_fields')
                    check(stats['rgb_win_fraction'],np.count_nonzero(rr<tt)/k,'group_fields')
                    check(stats['tie_fraction'],np.count_nonzero(rr==tt)/k,'group_fields')
                    for mod,e in [('rgb',rr),('thr',tt)]:
                        expected={'mean_l2':sum(e)/k,'mse':np.dot(e,e)/k,'median_l2':np.median(e),
                                  'mean_relative_l2':sum(e/dd)/k,'median_relative_l2':np.median(e/dd)}
                        for field,value in expected.items():check(stats[mod][field],value,'group_fields')
                    for loss,power in [('mean_l2',1),('mse',2)]:
                        check(stats[f'rgb_over_thr_{loss}'],sum(rr**power)/sum(tt**power),'group_fields')
                        check(stats[f'gap_contribution_{loss}'],sum(rr**power-tt**power)/n,'group_fields')
                for loss,power in [('mean_l2',1),('mse',2)]:
                    for prefix in ['range_','cell_','joint_']:
                        gap=sum(v['stats'][f'gap_contribution_{loss}'] for name,v in row['anchors'][anchor]['groups'].items() if name.startswith(prefix) and v['stats'] is not None)
                        check(gap,float(np.mean(r**power)-np.mean(t**power)),'partition_gap_checks')
                    for mod,e in [('rgb',r),('thr',t)]:
                        check(np.mean(e**power),old[i]['scores'][anchor][loss]['learned_propagated'][mod],'historical_risks')
                ranking=np.maximum(a['midpoint_rgb_error_m'],a['midpoint_thr_error_m'])
                for fraction in [.01,.05]:
                    keep=n-math.ceil(n*fraction)
                    # Ascending error groups fill retained mass, sharing the boundary equally.
                    order=np.argsort(ranking,kind='stable');cut=ranking[order[keep-1]]
                    low=np.flatnonzero(ranking<cut);equal=np.flatnonzero(ranking==cut)
                    w=np.zeros(n);w[low]=1;w[equal]=(keep-len(low))/len(equal)
                    tail=row['anchors'][anchor]['tails'][str(fraction)]
                    check(tail['kept_mass'],keep,'tail_fields');check(tail['removed_fraction'],1-keep/n,'tail_fields')
                    for mod,e in [('rgb',r),('thr',t)]:
                        for loss,power in [('mean_l2',1),('mse',2)]:
                            z=e**power
                            retained=sum(w*z)/keep;removed=sum((1-w)*z)/sum(z)
                            check(tail['camera'][mod][loss]['retained_risk'],retained,'tail_fields')
                            check(tail['camera'][mod][loss]['removed_loss_fraction'],removed,'tail_fields')
                    for loss in ['mean_l2','mse']:
                        check(tail[f'rgb_over_thr_{loss}'],tail['camera']['rgb'][loss]['retained_risk']/tail['camera']['thr'][loss]['retained_risk'],'tail_fields')
        for anchor,summary in report['recordings'][rec]['summary'].items():
            for name,s in summary['groups'].items():
                g=[row['anchors'][anchor]['groups'][name] for row in rows];good=[v for v in g if v['n']]
                assert s['nonempty_frames']==len(good)
                assert s['counts_min_median_max']==[min(v['n'] for v in g),float(np.median([v['n'] for v in g])),max(v['n'] for v in g)]
                check(s['median_fraction'],np.median([v['fraction'] for v in g]),'summary_fields')
                if not good:assert s['median_stats'] is None;continue
                for field,val in s['median_stats'].items():
                    if isinstance(val,dict):
                        for k,x in val.items():check(x,np.median([v['stats'][field][k] for v in good]),'summary_fields')
                    else:check(val,np.median([v['stats'][field] for v in good]),'summary_fields')
                for loss in ['mean_l2','mse']:assert s[f'rgb_lower_{loss}_frames']==sum(v['stats'][f'rgb_over_thr_{loss}']<1 for v in good)
            for fraction,tail in summary['tails'].items():
                frames=[row['anchors'][anchor]['tails'][fraction] for row in rows]
                for loss,stats in tail.items():
                    check(stats['median_rgb_over_thr'],np.median([x[f'rgb_over_thr_{loss}'] for x in frames]),'summary_fields')
                    assert stats['rgb_lower_frames']==sum(x[f'rgb_over_thr_{loss}']<1 for x in frames)
                    for mod in ['rgb','thr']:check(stats[f'{mod}_removed_loss_median'],np.median([x['camera'][mod][loss]['removed_loss_fraction'] for x in frames]),'summary_fields')
    result={'status':'PASS','checks':counts,'validator_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'report_sha256':hashlib.sha256((source/'report.json').read_bytes()).hexdigest(),
            'scope':'saved-array geometry, groups, loss/tail accounting and aggregation; original physical correspondence/calibration not certified'}
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();validate(a.root,a.source,a.output)
