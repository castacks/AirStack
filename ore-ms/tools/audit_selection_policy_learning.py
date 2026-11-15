"""Independent loss/gradient/Adam, subset/fit and scoring reconstruction."""
import argparse
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np

from audit_paired_frontend import hat,logarithm
from qualify_native_frontend import now,read,sha


def seed(*values):return int.from_bytes(hashlib.sha256(':'.join(map(str,values)).encode()).digest()[:8],'little')

def weights(theta,F):
    z=F@theta;z=np.exp(z-z.max());return z/z.sum()*len(F)

def draw_and_score(theta,F,k,random_seed):
    rng=np.random.default_rng(random_seed);remaining=list(range(len(F)));sequence=[];score=np.zeros(8)
    for _ in range(k):
        values=F[remaining];z=values@theta;prob=np.exp(z-z.max());prob/=prob.sum();position=rng.choice(len(remaining),p=prob);chosen=remaining[position]
        score+=F[chosen]-prob@values;sequence.append(chosen);remaining.pop(position)
    return np.array(sequence),score


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--protocol',required=True);args=parser.parse_args();plan=read(args.protocol);base=read(plan['corpus_contract']);trained=Path(plan['training_output']);predicted=Path(plan['evaluation_output']);scored=Path(plan['scoring_output']);checks=0;fit_count=0;successful=0;replayed_failures=0
    def check(value,message):
        nonlocal checks
        assert value,message;checks+=1
    def close(a,b,rtol=1e-8,atol=1e-10):
        nonlocal checks
        np.testing.assert_allclose(a,b,rtol=rtol,atol=atol);checks+=1
    for p,h in plan['pins'].items():check(sha(p)==h,p)
    for root,lock in [(trained,'model-lock.json'),(predicted,'prediction-lock.json')]:
        for rel,h in read(root/lock)['files'].items():check(sha(root/rel)==h,rel)
    cache=Path(plan['native_cache']);derived=Path(plan['derived_cache']);rows={r['id']:r for r in read(cache/'predictions.json')['rows']};renders={r['id']:r for r in read(plan['render_inventory'])['records']};label_sources={r['id']:r for r in read(derived/'inventory.json')['labels']};packet_cache={};truth_cache={};reward_cache={}
    def packet(key):
        if key not in packet_cache:
            with np.load(rows[key]['packet_path']) as z:packet_cache[key]={k:z[k].copy() for k in z.files}
        return packet_cache[key]
    def truth(key):
        if key not in truth_cache:
            with np.load(renders[key]['path']) as z:truth_cache[key]=z['target_to_source'].copy()
        return truth_cache[key]
    def reward(key):
        if key not in reward_cache:
            with np.load(label_sources[key]['path']) as z:reward_cache[key]=z['reward'].copy()
        return reward_cache[key]
    def uniform(key,n,index=0):return np.sort(np.random.default_rng(seed(base['seed'],key,'uniform',index)).choice(n,min(32,n),replace=False))
    def audit_fit(key,ids,result,utility=None,replay_theta=None,replay_F=None):
        nonlocal fit_count,successful,replayed_failures
        fit_count+=1;p=packet(key);ids=np.sort(ids)
        P={name:(value[ids] if value.ndim>0 and len(value)==len(p['X0']) else value) for name,value in p.items() if name!='features'}
        if utility is not None:P['C0']=P['C0']/utility[:,None,None];P['C1']=P['C1']/utility[:,None,None]
        check(result['successful']==(result['status']=='CONVERGED_FROZEN_WEIGHT_SCORE'),'fit eligibility')
        if not result['successful']:
            check(result.get('information_covariance') is None,'failed readout unavailable')
            # Independently reconstruct failed inputs, then reproduce the frozen algorithm failure.
            from paired_frontend_assessment import fit
            if replay_theta is not None:
                from selection_policy_contract import residual_utility_weights
                exact=residual_utility_weights(replay_theta,replay_F)
                P['C0']=p['C0'][ids]/exact[:,None,None];P['C1']=p['C1'][ids]/exact[:,None,None]
            try:replay=fit(P,base['solver'])
            except (ValueError,np.linalg.LinAlgError):replay={'status':'NUMERICAL_FAILURE','successful':False}
            check(replay['status']==result['status'] and not replay['successful'],'failed algorithm replay');replayed_failures+=1
            return 1.
        T=np.array(result['target_to_source']);R=T[:3,:3];close(R.T@R,np.eye(3),atol=1e-9);close(np.linalg.det(R),1,atol=1e-9)
        q=P['X1']@R.T+T[:3,3];e=q-P['X0'];W=np.linalg.inv(P['C0']+R@P['C1']@R.T);J=np.concatenate([np.broadcast_to(np.eye(3),(len(ids),3,3)),-np.stack([hat(v) for v in q])],2)
        H=np.einsum('nji,njk,nkl->il',J,W,J);g=np.einsum('nji,njk,nk->i',J,W,e);cov=np.linalg.inv(H);score=np.sqrt(max(g@cov@g,0))
        check(score<=base['solver']['score_tolerance']*1.01+1e-8,'independent stationarity')
        close(result['score_norm'],score,rtol=2e-3,atol=1e-7);close(result['information_covariance'],cov,rtol=2e-5,atol=1e-8);successful+=1
        error=logarithm(T@np.linalg.inv(truth(key)));risk=np.sum((error[:3]/.01)**2)+np.sum((error[3:]/np.deg2rad(.1))**2)
        return float(-np.expm1(-risk))
    models={};training_events=0
    for fold in base['folds']:
        group=fold['excluded_group'];stats_path=derived/(group+'-statistics.json');stats=read(stats_path);mean=np.array(stats['mean']);scale=np.array(stats['scale']);ordered=sorted(fold['training_scene_ids']);schedule=np.asarray(ordered)[np.random.default_rng(seed(base['seed'],group,'scene-schedule')).permutation(len(ordered))].tolist()[:128]
        for arm in ['pose_weight_only32','match_reward_policy32','pose_reward_policy32']:
            folder=trained/group/arm;model=read(folder/'model.json');journal=[json.loads(v) for v in (folder/'journal.jsonl').read_text().splitlines()]
            check(len(journal)==64 and model['updates']==64,'fixed updates');check(model['training_scene_ids']==fold['training_scene_ids'] and model['used_scene_ids']==schedule,'fold membership/schedule');check(model['normalization_sha256']==sha(stats_path),'normalization binding')
            theta=np.zeros(8);moment=np.zeros(8);variance=np.zeros(8);local_fits=0
            for update,record in enumerate(journal):
                keys=schedule[update*2:update*2+2];check(record['update']==update and record['scene_ids']==keys,'update schedule');gradient=np.zeros(8);batch_losses=[]
                for key in keys:
                    check(key in fold['training_scene_ids'] and key not in fold['evaluation_scene_ids'],'no evaluation train scene')
                    events=[e for e in record['events'] if e['scene_id']==key];training_events+=len(events)
                    if rows[key]['status']!='AVAILABLE':check(len(events)==1 and events[0]['loss']==1,'unavailable retained');batch_losses.append(1.);continue
                    p=packet(key);F=(p['features']-mean)/scale;n=len(F)
                    if arm=='pose_weight_only32':
                        check(len(events)==17,'weight finite-difference events');ids=uniform(key,n);losses={}
                        for event in events:
                            label=event['label'];expected=theta.copy()
                            if label!='base':side,j=label.split(':');expected[int(j)]+=.001*(1 if side=='plus' else -1)
                            close(event['theta'],expected);w=weights(expected,F[ids]);loss=audit_fit(key,ids,event['fit'],w,expected,F[ids]);local_fits+=1;close(event['loss'],loss)
                            losses[label]=event['loss']
                        check(set(losses)=={'base'}|{f'{s}:{j}' for s in ['plus','minus'] for j in range(8)},'all perturbations')
                        gradient+=np.array([(losses[f'plus:{j}']-losses[f'minus:{j}'])/.002 for j in range(8)])/2;batch_losses.append(losses['base'])
                    else:
                        check(len(events)==4,'four policy samples');losses=[];scores=[]
                        for draw,event in enumerate(events):
                            expected_seed=seed(base['seed'],group,update,key,draw);sequence,score=draw_and_score(theta,F,min(32,n),expected_seed)
                            check(event['seed']==expected_seed and event['sequence']==sequence.tolist(),'sampler draw');ids=np.sort(sequence)
                            if arm=='match_reward_policy32':loss=float(1-reward(key)[ids].mean());check(event['fit'] is None,'match reward has no pose oracle')
                            else:loss=audit_fit(key,ids,event['fit']);local_fits+=1
                            close(event['loss'],loss);losses.append(event['loss']);scores.append(score)
                        values=np.array(losses);baseline=(values.sum()-values)/3;gradient+=((values-baseline)[:,None]*np.array(scores)).mean(0)/2;batch_losses.append(values.mean())
                close(record['mean_loss'],np.mean(batch_losses));close(record['gradient'],gradient)
                clipped=gradient*min(1.,1/(np.linalg.norm(gradient)+1e-6));moment=.9*moment+.1*clipped;variance=.999*variance+.001*clipped**2
                expected=theta-.01*(moment/(1-.9**(update+1)))/(np.sqrt(variance/(1-.999**(update+1)))+1e-8)
                close(record['theta'],expected);theta=np.array(record['theta'])
            close(model['theta'],theta);check(model['fit_attempts']==local_fits,'training fit count');models[group,arm]=theta
            print('audited training',group,arm,local_fits,flush=True)
    pred=read(predicted/'predictions.json');check(pred['evaluation_pose_values_read'] is False,'prediction target isolation');check(len(pred['rows'])==240,'evaluation rows');primitives={};evaluation_fits=0
    for fold in base['folds']:
        group=fold['excluded_group'];stats=read(derived/(group+'-statistics.json'));selected=[r for r in pred['rows'] if r['fold']==group];check(len(selected)==80 and {r['scene_id'] for r in selected}==set(fold['evaluation_scene_ids']),'evaluation scope')
        for row in selected:
            key=row['scene_id']
            if rows[key]['status']!='AVAILABLE':check(row['status']=='UNAVAILABLE_POOL' and not row['arms'],'unavailable pool retained');continue
            p=packet(key);F=(p['features']-np.array(stats['mean']))/np.array(stats['scale']);n=len(F);k=min(32,n)
            expected={'native_context':np.arange(n)};expected.update({f'uniform32_seed{i}':uniform(key,n,i) for i in range(4)})
            for arm in ['match_reward_policy32','pose_reward_policy32']:
                expected[arm]=np.argsort(-(F@models[group,arm]),kind='stable')[:k]
                for i,master in enumerate(base['adapter']['stochastic_context_seeds']):expected[arm+f'_stochastic{i}']=draw_and_score(models[group,arm],F,k,seed(master,key))[0]
            expected['pose_weight_only32']=uniform(key,n)
            check(set(row['arms'])==set(expected)|{'information32'},'all17arms')
            # Verify every greedy choice against independently formed information blocks.
            q=p['X1'];J=np.concatenate([np.broadcast_to(np.eye(3),(n,3,3)),-np.stack([hat(v) for v in q])],2);D=np.diag([float(p['length'])]*3+[1.]*3);A=J@D;blocks=np.einsum('nji,njk,nkl->nil',A,np.linalg.inv(p['C0']+p['C1']),A);ridge=1e-6*np.trace(blocks.sum(0))/6;current=np.eye(6)*ridge;available=set(range(n));chosen=[]
            # Source stores sorted final set, so independently reconstruct greedy order.
            for _ in range(k):
                ids=np.array(sorted(available));_,logs=np.linalg.slogdet(current[None]+blocks[ids]);best=int(ids[np.argmax(logs)]);chosen.append(best);available.remove(best);current+=blocks[best]
            expected['information32']=np.array(chosen)
            for arm,result in row['arms'].items():
                ids=np.sort(expected[arm]);check(result['selected_rows']==ids.tolist() and result['points']==len(ids),'evaluation subset')
                w=None
                if arm=='pose_weight_only32':w=weights(models[group,arm],F[ids]);close(result['utility_weights'],w)
                else:check(result['utility_weights'] is None,'no hidden utility weights')
                audit_fit(key,ids,result,np.array(result['utility_weights']) if w is not None else None);evaluation_fits+=1
                if result['successful']:primitives[group,key,arm]=logarithm(np.array(result['target_to_source'])@np.linalg.inv(truth(key)))
        print('audited evaluation',group,flush=True)
    scores=read(scored/'scores.json');check(scores['prediction_lock_sha256']==sha(predicted/'prediction-lock.json'),'scoring lock');check(scores['error_arrays_sha256']==sha(scored/'error-arrays.npz'),'scoring arrays')
    with np.load(scored/'error-arrays.npz') as arrays:
        for fold in base['folds']:
            group=fold['excluded_group'];selected=[r for r in pred['rows'] if r['fold']==group];report=scores['folds'][group];valid={name:{r['scene_id'] for r in selected if (group,r['scene_id'],name) in primitives} for name in report['arms']};common=set.intersection(*(valid[name] for name in ['uniform32_seed0','information32','pose_weight_only32','match_reward_policy32','pose_reward_policy32']))
            def summary(name,ids,label,target):
                keys=[r['scene_id'] for r in selected if r['scene_id'] in ids];check(target['valid_scenes']==len(keys),'metric population')
                if not keys:check(target['metrics'] is None,'undefined empty metric');return
                X=np.array([primitives[group,key,name] for key in keys]);prefix=group+'__'+name+'__'+label;close(arrays[prefix+'_error'],X,rtol=3e-5,atol=1e-9);check(arrays[prefix+'_scene_ids'].tolist()==keys,'metric IDs')
                metrics={'mean_translation_error_m':np.linalg.norm(X[:,:3],axis=1).mean(),'translation_rmse_m':np.sqrt(np.sum(X[:,:3]**2,1).mean()),'mean_rotation_error_deg':np.rad2deg(np.linalg.norm(X[:,3:],axis=1).mean()),'rotation_rmse_deg':np.rad2deg(np.sqrt(np.sum(X[:,3:]**2,1).mean())),'mean_selected_points':np.mean([r['arms'][name]['points'] for r in selected if r['scene_id'] in ids])}
                for key,value in metrics.items():close(target[key],value)
            for arm,value in report['arms'].items():
                check(value['attempted_scenes']==80 and value['successful_scene_ids']==sorted(valid[arm]),'availability');statuses={}
                for row in selected:status=row['arms'].get(arm,{}).get('status','UNAVAILABLE_POOL');statuses[status]=statuses.get(status,0)+1
                check(value['statuses']==statuses,'statuses');summary(arm,valid[arm],'own',value['own']);summary(arm,valid[arm]&common,'common',value['common_matched_budget'])
            passes=[]
            for control,value in report['comparisons'].items():
                ids=valid['pose_reward_policy32']&valid[control];summary('pose_reward_policy32',ids,'paired_'+control,value['pose_policy']);summary(control,ids,'paired_pose',value['control']);a,b=value['pose_policy'],value['control'];criteria={'all_control_successes_preserved':valid['pose_reward_policy32'].issuperset(valid[control]),'matched_population_available':bool(ids)}
                if ids:criteria.update(positive_control_translation_error=b['mean_translation_error_m']>0 and b['translation_rmse_m']>0,translation_mean_5pct=a['mean_translation_error_m']<=.95*b['mean_translation_error_m'],translation_rmse_5pct=a['translation_rmse_m']<=.95*b['translation_rmse_m'],rotation_mean_nonregression=a['mean_rotation_error_deg']<=b['mean_rotation_error_deg'],rotation_rmse_nonregression=a['rotation_rmse_deg']<=b['rotation_rmse_deg'])
                check(value['criteria']==criteria and value['passes']==all(criteria.values()),'exact frozen criteria');passes.append(all(criteria.values()))
            check(report['passes_learning_target_gate']==all(passes),'fold decision')
    check(scores['passes_all_learning_target_gates']==all(f['passes_learning_target_gate'] for f in scores['folds'].values()),'all-fold decision')
    result={'created_utc':now(),'status':'PASS','checks':checks,'training_events_verified':training_events,'training_updates_verified':576,'fit_receipts_verified':fit_count,'converged_fit_receipts':successful,'failed_fit_receipts_replayed':replayed_failures,'evaluation_fit_attempts':evaluation_fits,'auditor_sha256':sha(__file__),'scores_sha256':sha(scored/'scores.json'),'scope':'Independent training loss/score/finite-difference/NumPyAdam, evaluation subsets/fit stationarity and analyticLog/scoring audit. No retuning or physical-transfer/novelty claim.'}
    (scored/'independent-audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)


if __name__=='__main__':main()
