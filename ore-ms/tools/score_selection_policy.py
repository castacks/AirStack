"""Frozen per-fold pose-error, availability and matched-control advancement scoring."""
import argparse
from pathlib import Path

import numpy as np

from pose_covariance_replay import log_se3
from qualify_native_frontend import now,read,sha,write

PRIMARY=['uniform32_seed0','information32','pose_weight_only32','match_reward_policy32','pose_reward_policy32']


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--protocol',required=True);parser.add_argument('--predictions',required=True);parser.add_argument('--output',required=True);args=parser.parse_args()
    plan=read(args.protocol);folder=Path(args.predictions);out=Path(args.output)
    for path,h in plan['pins'].items():assert sha(path)==h,path
    for rel,h in read(folder/'prediction-lock.json')['files'].items():assert sha(folder/rel)==h,rel
    rows=read(folder/'predictions.json')['rows'];renders={r['id']:r for r in read(plan['render_inventory'])['records']};base=read(plan['corpus_contract']);out.mkdir(parents=True,exist_ok=False)
    report={};arrays={}
    for fold in base['folds']:
        group=fold['excluded_group'];selected=[r for r in rows if r['fold']==group];assert {r['scene_id'] for r in selected}==set(fold['evaluation_scene_ids']) and len(selected)==80
        truth={}
        for row in selected:
            with np.load(renders[row['scene_id']]['path']) as z:truth[row['scene_id']]=z['target_to_source'].copy()
        names=sorted({name for row in selected for name in row['arms']})
        valid={name:{r['scene_id'] for r in selected if r['arms'].get(name,{}).get('successful',False)} for name in names}
        common=set.intersection(*(valid.get(name,set()) for name in PRIMARY))
        def summarize(name,ids,label):
            rr=[r for r in selected if r['scene_id'] in ids];X=np.array([log_se3(np.array(r['arms'][name]['target_to_source'])@np.linalg.inv(truth[r['scene_id']])) for r in rr])
            if not len(X):return {'valid_scenes':0,'metrics':None}
            prefix=group+'__'+name+'__'+label;arrays[prefix+'_error']=X;arrays[prefix+'_scene_ids']=np.array([r['scene_id'] for r in rr])
            return {'valid_scenes':len(X),'mean_translation_error_m':float(np.linalg.norm(X[:,:3],axis=1).mean()),'translation_rmse_m':float(np.sqrt(np.sum(X[:,:3]**2,axis=1).mean())),'mean_rotation_error_deg':float(np.rad2deg(np.linalg.norm(X[:,3:],axis=1).mean())),'rotation_rmse_deg':float(np.rad2deg(np.sqrt(np.sum(X[:,3:]**2,axis=1).mean()))),'mean_selected_points':float(np.mean([r['arms'][name]['points'] for r in rr]))}
        arms={}
        for name in names:
            statuses={}
            for r in selected:
                status=r['arms'].get(name,{}).get('status','UNAVAILABLE_POOL');statuses[status]=statuses.get(status,0)+1
            arms[name]={'attempted_scenes':80,'successful_scene_ids':sorted(valid[name]),'statuses':statuses,'own':summarize(name,valid[name],'own'),'common_matched_budget':summarize(name,valid[name]&common,'common')}
        comparisons={}
        for control in ['match_reward_policy32','pose_weight_only32']:
            pose=valid.get('pose_reward_policy32',set());control_ids=valid.get(control,set());ids=pose&control_ids
            a=summarize('pose_reward_policy32',ids,'paired_'+control);b=summarize(control,ids,'paired_pose');criteria={'all_control_successes_preserved':pose.issuperset(control_ids),'matched_population_available':len(ids)>0}
            if ids:
                criteria.update(positive_control_translation_error=b['mean_translation_error_m']>0 and b['translation_rmse_m']>0,translation_mean_5pct=a['mean_translation_error_m']<=.95*b['mean_translation_error_m'],translation_rmse_5pct=a['translation_rmse_m']<=.95*b['translation_rmse_m'],rotation_mean_nonregression=a['mean_rotation_error_deg']<=b['mean_rotation_error_deg'],rotation_rmse_nonregression=a['rotation_rmse_deg']<=b['rotation_rmse_deg'])
            comparisons[control]={'pose_policy':a,'control':b,'criteria':criteria,'passes':all(criteria.values())}
        report[group]={'arms':arms,'comparisons':comparisons,'passes_learning_target_gate':all(c['passes'] for c in comparisons.values()),'unavailable_pool_scene_ids':[r['scene_id'] for r in selected if r['status']!='AVAILABLE_POOL']}
    np.savez_compressed(out/'error-arrays.npz',**arrays)
    write(out/'scores.json',{'created_utc':now(),'folds':report,'passes_all_learning_target_gates':all(v['passes_learning_target_gate'] for v in report.values()),'prediction_lock_sha256':sha(folder/'prediction-lock.json'),'error_arrays_sha256':sha(out/'error-arrays.npz'),'scope':'Developmental synthetic metric pose errors. Pairwise common-success primary comparisons, no lost control-success scenes; top-k primary, stochastic contexts not best-seed selections. Not native-system promotion, real thermal transfer, physical calibration or algorithm novelty.'})
    for path,h in plan['pins'].items():assert sha(path)==h,path
    print('Scored',len(rows),'evaluation scenes',flush=True)


if __name__=='__main__':main()
