"""Independent reconstruction, augmented least-squares fit and policy-risk audit."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def validate(root,source,output):
    if output.exists():raise FileExistsError(output)
    protocol=json.loads((source/'protocol.json').read_text());report=json.loads((source/'report.json').read_text())
    counts={'hashes':0,'feature_values':0,'reconstructed_errors':0,'fit_fields':0,'margin_values':0,'decision_values':0,'risk_fields':0,'historical_fields':0,'summary_fields':0}
    def close(a,b,category):
        aa,bb=np.asarray(a),np.asarray(b)
        assert aa.shape==bb.shape and np.allclose(aa,bb,rtol=1e-10,atol=1e-10),(category,a,b)
        counts[category]+=int(aa.size)
    paths={'rain':'ms2-day-cross-modal-replication/rain-control','day':'ms2-day-cross-modal-replication/day','night':'ms2-night-cross-modal-replication/night'}
    families={'constant':[],'uncertainty':[0,1],'geometry':[2,3,4,5],'combined':list(range(6))}
    assert protocol['families']==families and protocol['ridge']==.01
    assert protocol['declaration_sha256']=='ca8e50bd2ad525010391397871c4e102ec6a28927efa511e2fa091064b995a97'
    assert len(protocol['frozen_input_sha256'])==128
    for path,h in protocol['frozen_input_sha256'].items():
        assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==h;counts['hashes']+=1
    prior=json.loads((root/'results/ms2-camera-scale-transfer/run-001/report.json').read_text())['folds']
    data={};historical={}
    for rec,rel in paths.items():
        folder=root/'results'/rel
        historical[rec]=json.loads((folder/'metric/report.json').read_text())['rows']
        data[rec]=[]
        for i in range(20):
            with np.load(folder/'metric'/f'metric-{i:06d}.npz') as f:a={k:f[k] for k in f.files}
            with np.load(folder/'support'/f'pairs-{i:06d}.npz') as f:p={k:f[k] for k in f.files}
            mask=a['both_valid_mask'];n=int(mask.sum());xy=p['rgb_xy'][mask]
            rgb,thr=a['rgb_predicted_xyz'][mask],a['thr_predicted_xyz_in_rgb'][mask]
            a['features']=np.array([np.log(a['rgb_metric_variance'][mask]),np.log(a['thr_metric_variance'][mask]),
                                    .5*np.log(np.einsum('ij,ij->i',rgb,rgb)),.5*np.log(np.einsum('ij,ij->i',thr,thr)),
                                    xy[:,0]/1224.,xy[:,1]/384.]).T
            for anchor,target in [('midpoint',(p['rgb_reference_xyz'][mask]+p['thr_reference_xyz_in_rgb'][mask])/2),('rgb',p['rgb_reference_xyz'][mask]),('thr',p['thr_reference_xyz_in_rgb'][mask])]:
                for mod,pred in [('rgb',rgb),('thr',thr)]:
                    close(np.sqrt(np.einsum('ij,ij->i',pred-target,pred-target)),a[f'{anchor}_{mod}_error_m'],'reconstructed_errors')
            a['n']=n;data[rec].append(a)
    assert set(report['folds'])==set(paths)
    for test,fold in report['folds'].items():
        train=[rec for rec in paths if rec!=test];assert fold['train_recordings']==train and fold['test_recording']==test
        assert prior[test]['train_recordings']==train and fold['scalar_policy']==prior[test]['policy']
        train_rows=[a for rec in train for a in data[rec]]
        base=np.concatenate([np.full(a['n'],1/(40*a['n'])) for a in train_rows])
        raw=np.vstack([a['features'] for a in train_rows])
        labels=np.concatenate([np.sign(a['midpoint_thr_error_m']-a['midpoint_rgb_error_m']) for a in train_rows])
        cost=np.concatenate([np.full(a['n'],1/(40*a['n']))*abs(a['midpoint_thr_error_m']-a['midpoint_rgb_error_m']) / ((a['midpoint_thr_error_m'].sum()+a['midpoint_rgb_error_m'].sum())/(2*a['n'])) for a in train_rows])
        fixed={}
        for mod in ['rgb','thr']:
            fixed['always_'+mod]=float(sum(float(a[f'midpoint_{mod}_error_m'].sum()/a['n'])/float((a['midpoint_rgb_error_m'].sum()+a['midpoint_thr_error_m'].sum())/(2*a['n'])) for a in train_rows)/40)
        for key,value in fixed.items():close(value,fold['fixed_objectives'][key],'fit_fields')
        selected=min(fixed,key=lambda k:(fixed[k],0 if k=='always_rgb' else 1));assert fold['training_fixed']==selected
        alternatives={}
        for family,cols in families.items():
            m=fold['models'][family];assert m['feature_indices']==cols and m['ridge']==.01 and m['training_pairs']==len(raw)
            values=raw[:,cols]
            mu=np.average(values,axis=0,weights=base)
            sd=np.sqrt(np.average((values-mu)**2,axis=0,weights=base));sd[sd<1e-12]=1
            close(mu,m['mean'],'fit_fields');close(sd,m['scale'],'fit_fields')
            x=np.ones((len(raw),len(cols)+1));x[:,1:]=(values-mu)/sd
            regularizer=np.diag([0.]+[.1]*len(cols))
            # Alternate solver, augmented weighted LS rather than solving normal equations.
            aug=np.vstack([np.sqrt(cost)[:,None]*x,regularizer]);target=np.r_[np.sqrt(cost)*labels,np.zeros(len(cols)+1)]
            theta=np.linalg.lstsq(aug,target,rcond=None)[0]
            close(theta,m['theta'],'fit_fields');alternatives[family]=theta
            residual=x@np.asarray(m['theta'])-labels
            close(np.dot(cost,residual**2)+.01*np.dot(np.asarray(m['theta'])[1:],np.asarray(m['theta'])[1:]),m['training_objective'],'fit_fields')
            close(base.sum(),m['base_weight_sum'],'fit_fields');close(cost.sum(),m['cost_weight_sum'],'fit_fields')
            normal=x.T@(cost[:,None]*x)+np.diag([0.]+[.01]*len(cols));rhs=x.T@(cost*labels)
            close(np.max(np.abs(normal@np.asarray(m['theta'])-rhs)),m['normal_equation_max_abs_residual'],'fit_fields')
        assert len(fold['rows'])==20
        for i,(a,row,hist,previous) in enumerate(zip(data[test],fold['rows'],historical[test],prior[test]['rows'],strict=True)):
            assert row['stem']==hist['stem']==previous['stem']==f'{i:06d}'
            assert row['availability']=={k:hist[k] for k in row['availability']}
            with np.load(source/test/f'selection-{i:06d}.npz') as f:saved={k:f[k] for k in f.files}
            close(a['features'],saved['features'],'feature_values')
            choices={}
            for family,cols in families.items():
                m=fold['models'][family];x=np.ones((a['n'],len(cols)+1));x[:,1:]=(a['features'][:,cols]-np.asarray(m['mean']))/np.asarray(m['scale'])
                margin=x@np.asarray(m['theta']);close(margin,saved[family+'_margin'],'margin_values')
                s=np.full(a['n'],.5);s[margin>0]=1;s[margin<0]=0
                assert np.array_equal(s,saved[family+'_choice']);counts['decision_values']+=a['n'];choices[family]=s
                alternate_margin=x@alternatives[family]
                alternate_choice=(alternate_margin>0).astype(float)+.5*(alternate_margin==0)
                assert np.array_equal(alternate_choice,s);counts['decision_values']+=a['n']
            mask=a['both_valid_mask'];p=fold['scalar_policy']
            if p['kind']=='always_rgb':scalar=np.ones(a['n'])
            elif p['kind']=='always_thr':scalar=np.zeros(a['n'])
            else:
                r=a['rgb_metric_variance'][mask]*10**p['log10_c'];t=a['thr_metric_variance'][mask]
                scalar=(r<t).astype(float)+.5*(r==t)
            choices.update({'original':a['learned_propagated'],'camera_scale':a['camera_scale'],'scalar_fit':scalar,
                            'always_rgb':np.ones(a['n']),'always_thr':np.zeros(a['n']),'random':np.full(a['n'],.5),
                            'training_fixed':np.full(a['n'],1. if selected=='always_rgb' else 0.)})
            assert np.array_equal(choices['constant'],choices['training_fixed'])
            for name,s in choices.items():close(s.mean(),row['rgb_choice_fractions'][name],'risk_fields')
            for anchor in ['midpoint','rgb','thr']:
                for loss,power in [('mean_l2',1),('mse',2)]:
                    r,t=a[f'{anchor}_rgb_error_m']**power,a[f'{anchor}_thr_error_m']**power
                    risks=row['risks'][anchor][loss]
                    for name,s in choices.items():
                        actual=(r[s==1].sum()+t[s==0].sum()+((r[s==.5]+t[s==.5])/2).sum())/a['n']
                        close(actual,risks[name],'risk_fields')
                    oracle=sum(min(x,y) for x,y in zip(r,t,strict=True))/a['n'];close(oracle,risks['oracle'],'risk_fields')
                    assert min(risks.values())>=oracle-1e-9
                    for new,old in [('original','original'),('camera_scale','camera_scale'),('scalar_fit','fitted'),('always_rgb','always_rgb'),('always_thr','always_thr'),('random','random'),('training_fixed','training_fixed'),('oracle','oracle')]:
                        close(risks[new],previous['risks'][anchor][loss][old],'historical_fields')
        for anchor,losses in fold['summary'].items():
            for loss,models in losses.items():
                for name,comparisons in models.items():
                    for baseline,stats in comparisons.items():
                        ratios=sorted(row['risks'][anchor][loss][name]/row['risks'][anchor][loss][baseline] for row in fold['rows'])
                        for key,value in [('min',ratios[0]),('median',(ratios[9]+ratios[10])/2),('max',ratios[-1]),('below_one',sum(v<1 for v in ratios))]:close(value,stats[key],'summary_fields')
    result={'status':'PASS','checks':counts,'validator_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'protocol_sha256':hashlib.sha256((source/'protocol.json').read_bytes()).hexdigest(),'report_sha256':hashlib.sha256((source/'report.json').read_bytes()).hexdigest(),
            'scope':'saved predictions/features, training-only fits, decisions and conditional reference risks; not physical calibration or untouched validation'}
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();validate(a.root,a.source,a.output)
