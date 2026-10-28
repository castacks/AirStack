"""Parameter-free sensitivity to hypothetical positive disparity distributions."""
import argparse,hashlib,json
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo
import numpy as np
PATHS={'exp063_rain':'results/ms2-unexamined-recording',**{f'exp064_{c}':f'results/ms2-frozen-suite/{c}' for c in ['day','night','rainy']}}
PRIMARY=['lognormal','gamma','first_order']
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def dump(p,v):p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def choose(a,b):return np.where(a<b,1.,np.where(a>b,0.,.5))
def scores(mu,variance,first):
    assert np.all(mu>0) and np.all(variance>0) and np.all(first>0)
    c=variance/mu**2;logc=np.log(c);logu=np.log(first)
    lognormal=logu+np.logaddexp.reduce(np.stack([np.zeros_like(c),np.log(3)+logc,2*logc]))
    finite=c<.5;gamma=np.full(len(c),np.inf)
    gamma[finite]=logu[finite]+np.log1p(2*c[finite])-np.log1p(-c[finite])-np.log1p(-2*c[finite])
    assert np.isfinite(c).all() and np.isfinite(lognormal).all() and np.isfinite(gamma[finite]).all()
    return {'c':c,'log_first_order':logu,'log_lognormal':lognormal,'log_gamma':gamma,'gamma_finite':finite}
def summary(rows):
    result={}
    for anchor in ['midpoint','rgb','thr']:
        result[anchor]={}
        for loss in ['mse','mean_l2']:
            result[anchor][loss]={}
            for method in PRIMARY:
                result[anchor][loss][method]={}
                for control in rows[0]['risks'][anchor][loss]:
                    if control==method:continue
                    ratios=[r['risks'][anchor][loss][method]/r['risks'][anchor][loss][control] for r in rows]
                    result[anchor][loss][method][control]={'median':float(np.median(ratios)),'min':min(ratios),'max':max(ratios),'better':sum(x<1 for x in ratios),'equal':sum(x==1 for x in ratios),'worse':sum(x>1 for x in ratios)}
    return result
def run(root,derived,output):
    if output.exists():raise FileExistsError(output)
    output.mkdir(parents=True);hashes={};records={}
    def load(p):
        hashes[str(p.relative_to(root))]=sha(p)
        if p.suffix=='.json':return json.loads(p.read_text())
        with np.load(p,allow_pickle=False) as f:return {k:f[k] for k in f.files}
    parent=load(derived/'protocol.json');assert parent['recordings']==PATHS
    validation=load(derived/'validation.json');assert validation['status']=='PASS' and validation['both_valid_pairs']==106562
    for relative,digest in parent['input_sha256'].items():assert sha(root/relative)==digest
    for name,relative in PATHS.items():
        base=root/relative;rows=[];(output/name).mkdir()
        for i in range(20):
            stem=f'{i:06d}';m=load(base/'metric'/f'metric-{stem}.npz');valid=m['both_valid_mask'];n=int(valid.sum());fields={};predictions={}
            for cam in ['rgb','thr']:
                a=load(derived/name/f'{cam}-{stem}.npz');assert len(a['disparity'])==n
                first=m[f'{cam}_metric_variance'][valid]
                assert np.array_equal(first,a['metric_variance']) or np.allclose(first,a['metric_variance'],rtol=1e-12,atol=1e-10)
                predictions[cam]=scores(a['disparity'],a['variance'],first)
                fields.update({f'{cam}_{k}':v for k,v in predictions[cam].items()})
            r,t=predictions['rgb'],predictions['thr']
            choices={'lognormal':choose(r['log_lognormal'],t['log_lognormal']),'gamma':choose(r['log_gamma'],t['log_gamma']),
                'first_order':choose(m['rgb_metric_variance'][valid],m['thr_metric_variance'][valid]),
                **{key:m[old] for key,old in [('camera_scale','camera_scale'),('geometry_angular','geometry_angular'),('geometry_pixel','geometry_pixel'),('unpropagated_angular','learned_unpropagated_angular')]},
                'always_rgb':np.ones(n),'always_thr':np.zeros(n),'random':np.full(n,.5)}
            assert np.array_equal(choices['first_order'],m['learned_propagated'])
            fields.update({f'choose_{k}':v for k,v in choices.items() if k in PRIMARY})
            row={'stem':stem,'n':n,'assumptions':{},'disagreement':{},'risks':{}}
            for cam,p in predictions.items():row['assumptions'][cam]={'median_c':float(np.median(p['c'])),'gamma_infinite_count':int((~p['gamma_finite']).sum()),'gamma_infinite_fraction':float((~p['gamma_finite']).mean())}
            row['gamma_both_infinite_count']=int((~r['gamma_finite']&~t['gamma_finite']).sum())
            row['gamma_both_infinite_fraction']=row['gamma_both_infinite_count']/n
            for a,b in [('lognormal','first_order'),('gamma','first_order'),('lognormal','gamma')]:row['disagreement'][a+'_vs_'+b]=float(np.mean(choices[a]!=choices[b]))
            row['rgb_choice_fraction']={key:float(v.mean()) for key,v in choices.items()}
            for anchor in ['midpoint','rgb','thr']:
                row['risks'][anchor]={}
                for loss,power in [('mse',2),('mean_l2',1)]:
                    er=m[f'{anchor}_rgb_error_m']**power;et=m[f'{anchor}_thr_error_m']**power
                    row['risks'][anchor][loss]={key:float(np.mean(v*er+(1-v)*et)) for key,v in choices.items()}
            np.savez_compressed(output/name/f'risk-{stem}.npz',**fields);rows.append(row)
        records[name]={'path':relative,'rows':rows,'summary':summary(rows),'assumption_summary':{}}
        for cam in ['rgb','thr']:
            records[name]['assumption_summary'][cam]={'median_c':float(np.median([r['assumptions'][cam]['median_c'] for r in rows])),'gamma_infinite_points':sum(r['assumptions'][cam]['gamma_infinite_count'] for r in rows),'median_gamma_infinite_fraction':float(np.median([r['assumptions'][cam]['gamma_infinite_fraction'] for r in rows]))}
        records[name]['assumption_summary']['both_gamma_infinite_points']=sum(r['gamma_both_infinite_count'] for r in rows)
        records[name]['assumption_summary']['median_both_gamma_infinite_fraction']=float(np.median([r['gamma_both_infinite_fraction'] for r in rows]))
        records[name]['median_disagreement']={key:float(np.median([r['disagreement'][key] for r in rows])) for key in rows[0]['disagreement']}
    for path,digest in hashes.items():assert sha(root/path)==digest
    protocol={'created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'starting_commit':'52b53e3a0','script_sha256':sha(__file__),'input_sha256':hashes,'recordings':PATHS,'primary':'midpoint squared 3D error; meanL2 and endpoint sensitivities','assumption':'hypothetical true positive disparity mean=prediction,var=horizontal output,not validated','c':'variance / predicted disparity²','lognormal':'original_metric_variance*(1+3*c+c²)','gamma':'original_metric_variance*(1+2*c)/((1-c)*(1-2*c)) if c<.5; mathematical infinity otherwise','decision':'lower model risk; exact ties=.5; both gamma infinite=.5 mixture explicitly counted; no epsilon/floor','unchanged_point_estimates':True,'fit_inference_new_data':False,'interpretation':'exploratory assumed-distribution sensitivity, not calibrated risk or prospective evidence','external_distribution_conventions':['https://docs.scipy.org/doc/scipy-1.15.3/reference/generated/scipy.stats.lognorm.html','https://docs.scipy.org/doc/scipy-1.15.3/reference/generated/scipy.stats.gamma.html']}
    dump(output/'protocol.json',protocol);dump(output/'report.json',{'recordings':records})
    print(json.dumps({name:{'assumptions':v['assumption_summary'],'disagreement':v['median_disagreement'],'midpoint_mse':v['summary']['midpoint']['mse']} for name,v in records.items()},indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--derived',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.root,a.derived,a.output)
