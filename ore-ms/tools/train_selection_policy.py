"""Fixed-budget matched policy/utility baseline experiment on a locked native cache."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import torch

from paired_frontend_assessment import fit
from feature_selection_comparison import select as established_selection
from pose_covariance_replay import log_se3
from selection_policy_contract import standardize,sample_subset,top_subset,ordered_log_probability_and_score,residual_utility_weights,bounded_pose_loss
from qualify_native_frontend import now,read,sha,write


def mixed_seed(*values):
    return int.from_bytes(hashlib.sha256(':'.join(map(str,values)).encode()).digest()[:8],'little')


def load_packet(row):
    with np.load(row['packet_path']) as z:return {k:z[k].copy() for k in z.files}


def subset(packet,indices,weights=None):
    indices=np.sort(indices);n=len(packet['X0'])
    selected={k:(v[indices] if v.ndim>0 and len(v)==n else v) for k,v in packet.items() if k!='features'}
    if weights is not None:
        selected['C0']=selected['C0']/weights[:,None,None];selected['C1']=selected['C1']/weights[:,None,None]
    return selected


def uniform_ids(plan,key,n,seed_index=0):
    seed=mixed_seed(plan['seed'],key,'uniform',seed_index)
    return np.sort(np.random.default_rng(seed).choice(n,min(32,n),replace=False))


def solve(packet,indices,settings,weights=None):
    try:return fit(subset(packet,indices,weights),settings)
    except (ValueError,np.linalg.LinAlgError) as error:return {'successful':False,'status':'NUMERICAL_FAILURE','reason':str(error)}


def training(plan,out):
    base=read(plan['corpus_contract']);cache=Path(plan['native_cache']);derived=Path(plan['derived_cache']);rows={r['id']:r for r in read(cache/'predictions.json')['rows']};renders={r['id']:r for r in read(plan['render_inventory'])['records']};labels={r['id']:r for r in read(derived/'inventory.json')['labels']}
    for fold in base['folds']:
        group=fold['excluded_group'];statistics=read(derived/(group+'-statistics.json'));mean,scale=np.array(statistics['mean']),np.array(statistics['scale'])
        rng=np.random.default_rng(mixed_seed(base['seed'],group,'scene-schedule'));ordered=sorted(fold['training_scene_ids']);schedule=[]
        while len(schedule)<128:schedule.extend(np.asarray(ordered)[rng.permutation(len(ordered))].tolist())
        for arm in ['pose_weight_only32','match_reward_policy32','pose_reward_policy32']:
            theta=torch.zeros(8,dtype=torch.float64,requires_grad=True);optimizer=torch.optim.Adam([theta],lr=.01,betas=(.9,.999),eps=1e-8)
            folder=out/group/arm;folder.mkdir(parents=True);history=[];fit_attempts=0
            for update in range(64):
                value=theta.detach().numpy().copy();gradient=np.zeros(8);batch_losses=[];events=[]
                for key in schedule[2*update:2*update+2]:
                    row=rows[key]
                    if row['status']!='AVAILABLE':batch_losses.append(1.);events.append({'scene_id':key,'status':'UNAVAILABLE_POOL','loss':1.});continue
                    packet=load_packet(row);F=standardize(packet['features'],mean,scale);n=len(F)
                    # Only training scene references are opened. Evaluation predictions read no labels.
                    if arm!='match_reward_policy32':
                        with np.load(renders[key]['path']) as z:truth=z['target_to_source'].copy()
                    if arm=='pose_weight_only32':
                        ids=uniform_ids(base,key,n)
                        def objective(t,label):
                            nonlocal fit_attempts
                            weights=residual_utility_weights(t,F[ids]);result=solve(packet,ids,base['solver'],weights);fit_attempts+=1
                            loss=bounded_pose_loss(log_se3(np.array(result['target_to_source'])@np.linalg.inv(truth))) if result['successful'] else 1.
                            events.append({'scene_id':key,'label':label,'theta':t.tolist(),'loss':loss,'fit':result});return loss
                        loss=objective(value,'base');g=[]
                        for j in range(8):
                            delta=np.eye(8)[j]*1e-3;g.append((objective(value+delta,'plus:'+str(j))-objective(value-delta,'minus:'+str(j)))/.002)
                        gradient+=np.array(g)/2;batch_losses.append(loss)
                    else:
                        losses=[];scores=[]
                        with np.load(labels[key]['path']) as z:reward=z['reward'].copy()
                        for draw in range(4):
                            seed=mixed_seed(base['seed'],group,update,key,draw);sequence=sample_subset(value,F,min(32,n),seed);ids=np.sort(sequence);_,score=ordered_log_probability_and_score(value,F,sequence)
                            if arm=='match_reward_policy32':loss=float(1-reward[ids].mean());result=None
                            else:
                                result=solve(packet,ids,base['solver']);fit_attempts+=1;loss=bounded_pose_loss(log_se3(np.array(result['target_to_source'])@np.linalg.inv(truth))) if result['successful'] else 1.
                            losses.append(loss);scores.append(score);events.append({'scene_id':key,'seed':seed,'sequence':sequence.tolist(),'loss':loss,'fit':result})
                        losses=np.array(losses);baseline=(losses.sum()-losses)/3
                        gradient+=np.mean((losses-baseline)[:,None]*np.array(scores),axis=0)/2;batch_losses.append(float(losses.mean()))
                optimizer.zero_grad();theta.grad=torch.tensor(gradient,dtype=torch.float64);torch.nn.utils.clip_grad_norm_([theta],1.);optimizer.step()
                record={'update':update,'scene_ids':schedule[2*update:2*update+2],'mean_loss':float(np.mean(batch_losses)),'theta':theta.detach().tolist(),'gradient':gradient.tolist(),'events':events};history.append({k:v for k,v in record.items() if k!='events'})
                with (folder/'journal.jsonl').open('a') as stream:stream.write(json.dumps(record,allow_nan=False)+'\n')
            write(folder/'model.json',{'created_utc':now(),'fold':group,'arm':arm,'theta':theta.detach().tolist(),'training_scene_ids':fold['training_scene_ids'],'used_scene_ids':schedule[:128],'normalization_sha256':sha(derived/(group+'-statistics.json')),'updates':64,'fit_attempts':fit_attempts,'history':history})
            print('trained',group,arm,fit_attempts,flush=True)
    write(out/'model-lock.json',{'created_utc':now(),'files':{str(p.relative_to(out)):sha(p) for p in out.rglob('*') if p.is_file()},'scope':'All3folds/3trained controls complete before evaluation pose-error scores.'})


def predictions(plan,out):
    base=read(plan['corpus_contract']);cache=Path(plan['native_cache']);derived=Path(plan['derived_cache']);trained=Path(plan['training_output']);rows={r['id']:r for r in read(cache/'predictions.json')['rows']}
    for rel,h in read(trained/'model-lock.json')['files'].items():assert sha(trained/rel)==h,rel
    results=[]
    for fold in base['folds']:
        group=fold['excluded_group'];stats=read(derived/(group+'-statistics.json'));thetas={arm:np.array(read(trained/group/arm/'model.json')['theta']) for arm in ['pose_weight_only32','match_reward_policy32','pose_reward_policy32']}
        for key in fold['evaluation_scene_ids']:
            row=rows[key];arms={}
            if row['status']!='AVAILABLE':
                results.append({'fold':group,'scene_id':key,'status':'UNAVAILABLE_POOL','arms':{}});continue
            packet=load_packet(row);F=standardize(packet['features'],np.array(stats['mean']),np.array(stats['scale']));n=len(F);k=min(32,n)
            selections=[('native_context',np.arange(n),None)]
            selections += [(f'uniform32_seed{i}',uniform_ids(base,key,n,i),None) for i in range(4)]
            info={'ids':np.arange(n),'uv':packet['source_uv'],'X0':packet['X0'],'X1':packet['X1'],'S':packet['C0']+packet['C1'],'length':packet['length']}
            selections.append(('information32',established_selection(info,'information',k),None))
            ids=uniform_ids(base,key,n);selections.append(('pose_weight_only32',ids,residual_utility_weights(thetas['pose_weight_only32'],F[ids])))
            for arm in ['match_reward_policy32','pose_reward_policy32']:
                selections.append((arm,top_subset(thetas[arm],F,k),None))
                for i,seed in enumerate(base['adapter']['stochastic_context_seeds']):
                    selections.append((arm+f'_stochastic{i}',sample_subset(thetas[arm],F,k,mixed_seed(seed,key)),None))
            for name,ids,weights in selections:
                result=solve(packet,ids,base['solver'],weights);arms[name]={**result,'selected_rows':np.sort(ids).tolist(),'points':len(ids),'utility_weights':None if weights is None else weights.tolist()}
            result={'fold':group,'scene_id':key,'status':'AVAILABLE_POOL','arms':arms};results.append(result)
            with (out/'journal.jsonl').open('a') as stream:stream.write(json.dumps(result,allow_nan=False)+'\n')
        print('predicted',group,flush=True)
    write(out/'predictions.json',{'created_utc':now(),'rows':results,'evaluation_pose_values_read':False})
    write(out/'prediction-lock.json',{'created_utc':now(),'files':{str(p.relative_to(out)):sha(p) for p in out.rglob('*') if p.is_file()}})


def main():
    parser=argparse.ArgumentParser();parser.add_argument('stage',choices=['train','predict']);parser.add_argument('--protocol',required=True);parser.add_argument('--output',required=True);args=parser.parse_args();plan,out=read(args.protocol),Path(args.output)
    for path,h in plan['pins'].items():assert sha(path)==h,path
    torch.set_num_threads(1);torch.use_deterministic_algorithms(True);out.mkdir(parents=True,exist_ok=False)
    {'train':training,'predict':predictions}[args.stage](plan,out)
    for path,h in plan['pins'].items():assert sha(path)==h,path


if __name__=='__main__':main()
