"""Independent frozen-model feature/margin/decision/risk audit; no fitting imports."""
import argparse,hashlib,json,math
from pathlib import Path
import numpy as np


def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def validate(root,base,output):
    if output.exists():raise FileExistsError(output)
    frozen=base/'frozen-models.json';assert digest(frozen)=='51b2b9a15d1951c2902e6ef8a0a5e45dcae3d1713d428e36d9545b530e0ae377'
    plan=json.loads(frozen.read_text());evaluation=base/'frozen-evaluation';r=json.loads((evaluation/'report.json').read_text());protocol=json.loads((evaluation/'protocol.json').read_text());raw=json.loads((base/'metric/report.json').read_text())
    assert protocol['frozen_model_sha256']==digest(frozen) and protocol['no_test_fit'] is True
    assert r['sequence']==plan['sequence']==raw['protocol']['sequence'] and r['primary']=='combined'
    families={'constant':[],'uncertainty':[0,1],'geometry':[2,3,4,5],'combined':list(range(6))}
    checks={'hashes':0,'feature_values':0,'margin_values':0,'choice_values':0,'risk_fields':0,'summary_fields':0,'historical_fields':0,'availability_fields':0}
    minmargin=math.inf
    def close(a,b,category):
        aa,bb=np.asarray(a),np.asarray(b);assert aa.shape==bb.shape and np.allclose(aa,bb,rtol=1e-10,atol=1e-10),(category,a,b)
        checks[category]+=int(aa.size)
    for source in [plan['frozen_development_inputs'],protocol['frozen_input_sha256']]:
        for path,h in source.items():assert digest(path)==h;checks['hashes']+=1
    for name,h in plan['source_sha256'].items():assert digest(root/'MAC-VO-main/Scripts/Setup'/(name+'.py'))==h
    assert len(r['rows'])==20
    for i,(row,old) in enumerate(zip(r['rows'],raw['rows'],strict=True)):
        stem=f'{i:06d}';assert row['stem']==old['stem']==plan['ids'][i]==stem
        assert row['availability']=={k:old[k] for k in row['availability']};checks['availability_fields']+=len(row['availability'])
        with np.load(base/'metric'/f'metric-{stem}.npz') as f:a={k:f[k] for k in f.files}
        with np.load(base/'support'/f'pairs-{stem}.npz') as f:p={k:f[k] for k in f.files}
        with np.load(evaluation/f'selection-{stem}.npz') as f:saved={k:f[k] for k in f.files}
        mask=a['both_valid_mask'];n=int(mask.sum());xy=p['rgb_xy'][mask];rgb=a['rgb_predicted_xyz'][mask];thr=a['thr_predicted_xyz_in_rgb'][mask]
        features=np.column_stack([np.log(a['rgb_metric_variance'][mask]),np.log(a['thr_metric_variance'][mask]),
                                 .5*np.log(np.einsum('ij,ij->i',rgb,rgb)),.5*np.log(np.einsum('ij,ij->i',thr,thr)),xy[:,0]/1224.,xy[:,1]/384.])
        close(features,saved['features'],'feature_values');choices={}
        for family,indices in families.items():
            model=plan['models'][family];assert model['feature_indices']==indices and model['ridge']==.01
            margin=np.full(n,model['theta'][0])
            for j,col in enumerate(indices):margin+=model['theta'][j+1]*(features[:,col]-model['mean'][j])/model['scale'][j]
            close(margin,saved[family+'_margin'],'margin_values')
            choice=(margin>0).astype(float)+.5*(margin==0);assert np.array_equal(choice,saved[family+'_choice']);checks['choice_values']+=n;choices[family]=choice
            if n:minmargin=min(minmargin,float(np.abs(margin).min()))
        scalar=plan['scalar_fit']['policy']
        if scalar['kind']=='always_rgb':s=np.ones(n)
        elif scalar['kind']=='always_thr':s=np.zeros(n)
        else:
            q=a['rgb_metric_variance'][mask]*10**scalar['log10_c'];t=a['thr_metric_variance'][mask];s=(q<t).astype(float)+.5*(q==t)
        fixed=plan['scalar_fit']['training_fixed_policy']['kind']
        choices.update({'original':a['learned_propagated'],'camera_scale':a['camera_scale'],'scalar_fit':s,'always_rgb':np.ones(n),'always_thr':np.zeros(n),'random':np.full(n,.5),'training_fixed':np.full(n,1. if fixed=='always_rgb' else 0.)})
        assert np.array_equal(choices['constant'],choices['training_fixed'])
        if not n:
            assert all(v is None for v in row['rgb_choice_fractions'].values());assert all(v is None for losses in row['risks'].values() for risks in losses.values() for v in risks.values());continue
        for name,s in choices.items():close(s.mean(),row['rgb_choice_fractions'][name],'risk_fields')
        for anchor in ['midpoint','rgb','thr']:
            for loss,power in [('mean_l2',1),('mse',2)]:
                rr,tt=a[f'{anchor}_rgb_error_m']**power,a[f'{anchor}_thr_error_m']**power;risks=row['risks'][anchor][loss]
                oracle=math.fsum(min(x,y) for x,y in zip(rr,tt,strict=True))/n;close(oracle,risks['oracle'],'risk_fields')
                for name,s in choices.items():
                    value=math.fsum(float(c)*float(x)+(1-float(c))*float(y) for c,x,y in zip(s,rr,tt,strict=True))/n
                    close(value,risks[name],'risk_fields');assert value>=oracle-1e-9
                for name,original in [('original','learned_propagated'),('camera_scale','camera_scale')]:close(risks[name],old['scores'][anchor][loss][original]['uncertainty'],'historical_fields')
                for name,oldkey in [('always_rgb','rgb'),('always_thr','thr'),('random','random'),('oracle','oracle')]:close(risks[name],old['scores'][anchor][loss]['learned_propagated'][oldkey],'historical_fields')
    good=[row for row in r['rows'] if row['availability']['both_valid']>0]
    assert r['available_frames']==len(good) and r['unavailable_stems']==[row['stem'] for row in r['rows'] if not row['availability']['both_valid']]
    if good:
        for anchor,losses in r['summary'].items():
            for loss,models in losses.items():
                for name,comparisons in models.items():
                    for comparator,stats in comparisons.items():
                        ratios=sorted(row['risks'][anchor][loss][name]/row['risks'][anchor][loss][comparator] for row in good);n=len(ratios)
                        median=ratios[n//2] if n%2 else (ratios[n//2-1]+ratios[n//2])/2
                        for key,value in [('min',ratios[0]),('median',median),('max',ratios[-1]),('below_one',sum(x<1 for x in ratios))]:close(value,stats[key],'summary_fields')
    else:assert r['summary'] is None
    result={'status':'PASS','checks':checks,'available_frames':len(good),'minimum_absolute_margin':minmargin if good else None,
            'validator_sha256':digest(__file__),'frozen_model_sha256':digest(frozen),'evaluation_report_sha256':digest(evaluation/'report.json'),
            'scope':'fixed model/feature/selection risks and unchanged development inputs; no new fit or physical-label certification'}
    output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--base',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();validate(a.root,a.base,a.output)
