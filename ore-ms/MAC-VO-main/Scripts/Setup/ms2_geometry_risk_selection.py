"""Exploratory cost-sensitive linear selectors; margins are not calibrated risks."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
from zoneinfo import ZoneInfo
import numpy as np

PATHS={'rain':'results/ms2-day-cross-modal-replication/rain-control',
       'day':'results/ms2-day-cross-modal-replication/day',
       'night':'results/ms2-night-cross-modal-replication/night'}
FEATURES=['log_u_rgb','log_u_thr','log_predicted_range_rgb','log_predicted_range_thr','rgb_x_over_1224','rgb_y_over_384']
FAMILIES={'constant':[],'uncertainty':[0,1],'geometry':[2,3,4,5],'combined':list(range(6))}
RIDGE=.01


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def extract_features(rgb_xyz,thr_xyz,rgb_u,thr_u,rgb_xy):
    """Only model predictions and native candidate coordinates; no labels."""
    rr=np.linalg.norm(rgb_xyz,axis=1);rt=np.linalg.norm(thr_xyz,axis=1)
    assert np.all(rr>0) and np.all(rt>0) and np.all(rgb_u>0) and np.all(thr_u>0)
    x=np.column_stack([np.log(rgb_u),np.log(thr_u),np.log(rr),np.log(rt),rgb_xy[:,0]/1224,rgb_xy[:,1]/384])
    assert np.isfinite(x).all()
    return x


def load(root,recovery):
    expected={}
    for snapshot in ['2026-10-26-exp-057','2026-10-27-exp-058','2026-10-27-exp-060']:
        m=json.loads((recovery/snapshot/'manifest.json').read_text());expected.update(m['member_sha256'])
    data={};hashes={};reports={}
    def verify(p):
        h=sha(p);assert h==expected['ore-ms/'+str(p.relative_to(root))],p
        hashes[str(p)]=h
    for rec,rel in PATHS.items():
        folder=root/rel
        for part in ['metric','support']:verify(folder/part/'report.json')
        reports[rec]=json.loads((folder/'metric/report.json').read_text())
        assert [r['stem'] for r in reports[rec]['rows']]==[f'{i:06d}' for i in range(20)]
        data[rec]=[]
        for i,row in enumerate(reports[rec]['rows']):
            ps=[folder/'metric'/f'metric-{i:06d}.npz',folder/'support'/f'pairs-{i:06d}.npz'];arrays=[]
            for p in ps:
                verify(p)
                with np.load(p,allow_pickle=False) as f:arrays.append({k:f[k] for k in f.files})
            a,p=arrays;mask=a['both_valid_mask'];n=int(mask.sum());assert n==row['both_valid']>0
            a['x']=extract_features(a['rgb_predicted_xyz'][mask],a['thr_predicted_xyz_in_rgb'][mask],a['rgb_metric_variance'][mask],a['thr_metric_variance'][mask],p['rgb_xy'][mask])
            a['n']=n;a['stem']=row['stem']
            data[rec].append(a)
    base=root/'results/ms2-camera-scale-transfer/run-001'
    for name in ['protocol.json','report.json']:verify(base/name)
    scalar=json.loads((base/'report.json').read_text())['folds']
    return data,reports,hashes,scalar


def fit(data,train,indices):
    arrays=[];base=[];target=[];cost=[]
    for rec in train:
        for a in data[rec]:
            arrays.append(a['x'][:,indices]);b=np.full(a['n'],1/(len(train)*len(data[rec])*a['n']))
            r,t=a['midpoint_rgb_error_m'],a['midpoint_thr_error_m'];random=float(np.mean((r+t)/2));assert random>0
            base.append(b);target.append(np.sign(t-r));cost.append(b*np.abs(t-r)/random)
    raw=np.concatenate(arrays);b=np.concatenate(base);y=np.concatenate(target);w=np.concatenate(cost)
    mean=np.sum(b[:,None]*raw,axis=0)/b.sum()
    scale=np.sqrt(np.sum(b[:,None]*(raw-mean)**2,axis=0)/b.sum());scale=np.where(scale<1e-12,1.,scale)
    x=np.column_stack([np.ones(len(raw)),(raw-mean)/scale]);penalty=np.diag([0.]+[RIDGE]*len(indices))
    gram=x.T@(w[:,None]*x)+penalty;rhs=x.T@(w*y)
    theta=np.zeros(len(indices)+1) if not np.any(w) else np.linalg.solve(gram,rhs)
    objective=float(np.dot(w,(x@theta-y)**2)+RIDGE*np.dot(theta[1:],theta[1:]))
    return {'feature_indices':indices,'features':[FEATURES[i] for i in indices],'mean':mean.tolist(),'scale':scale.tolist(),'theta':theta.tolist(),
            'ridge':RIDGE,'training_objective':objective,'training_pairs':len(raw),'base_weight_sum':float(b.sum()),'cost_weight_sum':float(w.sum()),
            'normal_equation_max_abs_residual':float(np.max(np.abs(gram@theta-rhs)))}


def predict(features,model):
    raw=features[:,model['feature_indices']];x=np.column_stack([np.ones(len(features)),(raw-np.array(model['mean']))/np.array(model['scale'])])
    margin=x@np.array(model['theta']);choice=np.where(margin>0,1.,np.where(margin<0,0.,.5))
    return margin,choice


def scalar_choice(a,policy):
    if policy['kind']=='always_rgb':return np.ones(a['n'])
    if policy['kind']=='always_thr':return np.zeros(a['n'])
    mask=a['both_valid_mask'];r=a['rgb_metric_variance'][mask]*10.**policy['log10_c'];t=a['thr_metric_variance'][mask]
    return np.where(r<t,1.,np.where(r>t,0.,.5))


def summarize(rows):
    result={}
    for anchor in ['midpoint','rgb','thr']:
        result[anchor]={}
        for loss in ['mean_l2','mse']:
            result[anchor][loss]={}
            names=rows[0]['risks'][anchor][loss]
            for family in FAMILIES:
                comparisons={}
                for comparator in names:
                    if comparator==family:continue
                    ratios=np.array([r['risks'][anchor][loss][family]/r['risks'][anchor][loss][comparator] for r in rows])
                    comparisons[comparator]={'min':float(ratios.min()),'median':float(np.median(ratios)),'max':float(ratios.max()),'below_one':int((ratios<1).sum())}
                result[anchor][loss][family]=comparisons
    return result


def run(root,recovery,output):
    if output.exists():raise FileExistsError(output)
    declaration=root/'results/ms2-geometry-risk-selection/design-declaration.md'
    assert sha(declaration)=='ca8e50bd2ad525010391397871c4e102ec6a28927efa511e2fa091064b995a97'
    data,reports,hashes,scalar=load(root,recovery)
    protocol={'created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'starting_commit':'b67bb783b','script_sha256':sha(__file__),
              'declaration_sha256':sha(declaration),'numpy_version':np.__version__,'features':FEATURES,'families':FAMILIES,'ridge':RIDGE,
              'training':'equal-recording/equal-frame baseweights; target sign(E_T-E_R); costweight b*abs(E_T-E_R)/frame_random_meanL2; midpoint only',
              'standardization':'training-only baseweighted mean/STD; STD<1e-12 replaced by1; intercept unpenalized',
              'scope':'exploratory previously inspected windows; recording-excluded, not untouched or network-training-held-out',
              'inference':'positive margin RGB, negative thermal, exact0 equal mixture; margins not probabilities/calibrated risks',
              'support':'unchanged reference-selected both-valid candidates; no availability/correspondence deployment policy','frozen_input_sha256':hashes}
    output.mkdir(parents=True);folds={}
    (output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    for test in data:
        train=[rec for rec in data if rec!=test]
        assert scalar[test]['train_recordings']==train
        models={name:fit(data,train,indices) for name,indices in FAMILIES.items()}
        fixed_objectives={}
        for name,choice in [('always_rgb',1),('always_thr',0)]:
            fixed_objectives[name]=float(np.mean([np.mean([np.mean(a['midpoint_rgb_error_m'] if choice else a['midpoint_thr_error_m'])/np.mean((a['midpoint_rgb_error_m']+a['midpoint_thr_error_m'])/2) for a in data[rec]]) for rec in train]))
        fixed=min(fixed_objectives,key=lambda k:(fixed_objectives[k],0 if k=='always_rgb' else 1))
        folder=output/test;folder.mkdir();rows=[]
        for a,hist in zip(data[test],reports[test]['rows'],strict=True):
            selections={};saved={'features':a['x']}
            for family,model in models.items():
                margin,choice=predict(a['x'],model);selections[family]=choice;saved[family+'_margin']=margin;saved[family+'_choice']=choice
            selections.update({'original':a['learned_propagated'],'camera_scale':a['camera_scale'],'scalar_fit':scalar_choice(a,scalar[test]['policy']),
                               'always_rgb':np.ones(a['n']),'always_thr':np.zeros(a['n']),'random':np.full(a['n'],.5),'training_fixed':np.full(a['n'],1. if fixed=='always_rgb' else 0.)})
            row={'stem':a['stem'],'availability':{k:hist[k] for k in ['candidates','both_valid','rgb_only_valid','thr_only_valid','neither_valid']},
                 'rgb_choice_fractions':{k:float(s.mean()) for k,s in selections.items()},'risks':{}}
            for anchor in ['midpoint','rgb','thr']:
                row['risks'][anchor]={}
                for loss,power in [('mean_l2',1),('mse',2)]:
                    r,t=a[f'{anchor}_rgb_error_m']**power,a[f'{anchor}_thr_error_m']**power
                    risks={k:float(np.mean(s*r+(1-s)*t)) for k,s in selections.items()};risks['oracle']=float(np.minimum(r,t).mean())
                    row['risks'][anchor][loss]=risks
            np.savez_compressed(folder/f"selection-{a['stem']}.npz",**saved);rows.append(row)
        folds[test]={'train_recordings':train,'test_recording':test,'models':models,'fixed_objectives':fixed_objectives,'training_fixed':fixed,
                     'scalar_policy':scalar[test]['policy'],'rows':rows,'summary':summarize(rows)}
    for path,h in hashes.items():assert sha(path)==h
    (output/'report.json').write_text(json.dumps({'folds':folds},indent=2,allow_nan=False)+'\n')
    print(json.dumps({rec:{'training_fixed':f['training_fixed'],'combined_midpoint':f['summary']['midpoint']} for rec,f in folds.items()},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--recovery',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();run(a.root,a.recovery,a.output)
