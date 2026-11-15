"""Fixed natural thermal pairs: nominal supplied-geometry disagreement, not true flow."""
import argparse, hashlib, json, sys
from pathlib import Path
from datetime import datetime, timezone
import numpy as np
from u2flow_thermal_warp import recipe


def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p): return json.loads(Path(p).read_text())
def write(p, data):
    with Path(p).open('x') as f: json.dump(data, f, indent=2, allow_nan=False); f.write('\n')
def stamp(): return datetime.now(timezone.utc).isoformat()
def verify(plan):
    for p, h in plan['pins'].items(): assert sha(p)==h, p

def capture(plan, out):
    import cv2, torch
    from yacs.config import CfgNode
    source=Path(plan['source_root']); sys.path[:0]=[str(source),str(source/'core')]
    from core.Networks.raft_2f_u import RAFT
    assert torch.__version__=='2.7.1+cpu' and cv2.__version__=='4.10.0' and np.__version__=='1.26.4'
    torch.set_num_threads(1); torch.set_num_interop_threads(1); torch.manual_seed(plan['seed']); torch.use_deterministic_algorithms(True)
    model=RAFT(CfgNode({'iters':12,'mixed_precision':False})).cpu().eval()
    state=torch.load(plan['checkpoint'],map_location='cpu',weights_only=True)
    assert len(state)==106 and all(k.startswith('module.') for k in state)
    model.load_state_dict({k[7:]:v for k,v in state.items()},strict=True)
    statehash=lambda:{k:hashlib.sha256(v.detach().numpy().tobytes()).hexdigest() for k,v in model.state_dict().items()}
    before=statehash(); hooks=[]
    def hook(module,args,value):
        a=value[-1].detach(); assert torch.isfinite(a).all(); hooks.append([float(a.min()),float(a.max())])
    handle=model.update_block.register_forward_hook(hook)
    prepare=recipe(plan['preprocess_source']); images={}
    for row in plan['inputs']:
        raw=cv2.imread(row['raw_path'],cv2.IMREAD_UNCHANGED); assert raw.shape==(256,640) and raw.dtype==np.uint16
        old=raw.copy(); u8=cv2.resize(prepare(raw),(320,128),interpolation=cv2.INTER_AREA); assert np.array_equal(raw,old)
        images[row['condition'],row['stem']]=u8
    folder=out/'capture'; folder.mkdir(); rows=[]
    with torch.inference_mode():
      for pair in plan['pairs']:
        a=images[pair['condition'],pair['source']]; b=images[pair['condition'],pair['target']]
        first=a.astype(np.float32)/255; second=b.astype(np.float32)/255
        flows=[]; alphas=[]
        for target in [second,second**np.float32(1.6)]:
            x=torch.from_numpy(np.repeat(np.stack([first,target])[:,None],3,axis=1))[None]
            y=model(x,only_fw=True); assert len(y['flows_f12'])==len(y['uncertainty_f12'])==12
            assert all(torch.isfinite(v).all() for values in y.values() for v in values)
            flows.append(y['flows_f12'][-1][0].numpy()); alphas.append(y['uncertainty_f12'][-1][0].numpy())
        p=folder/(pair['condition']+'-'+pair['source']+'.npz')
        np.savez_compressed(p,source_uint8=a,target_uint8=b,flow=np.stack(flows),alpha=np.stack(alphas))
        row={**pair,'path':str(p.relative_to(out)),'sha256':sha(p)}; rows.append(row)
        with (out/'journal.jsonl').open('a') as f: f.write(json.dumps(row)+'\n')
        print(pair['condition'],pair['source'],'clean/gamma locked',flush=True)
    handle.remove(); assert before==statehash() and len(hooks)==1368
    write(out/'prediction-lock.json',{'created_utc':stamp(),'rows':rows,'forward_calls':114,'upstream_finite_uncertainty_calls':len(hooks),'model_state_unchanged':True,'pose_values_parsed':False,'geometry_endpoints_constructed':False})


def geometry(d0,d1,K,T):
    """All original positive-depth points, independent of model predictions."""
    v,u=np.nonzero(d0); uv=np.stack([u,v],axis=1).astype(float); z=d0[v,u].astype(float)/256
    rays=np.column_stack([uv,np.ones(len(uv))])@np.linalg.inv(K).T
    X=rays*z[:,None]; Y=X@T[:3,:3].T+T[:3,3]
    positive=Y[:,2]>0
    end=np.zeros_like(uv); end[positive]=(Y[positive]@K.T)[:,:2]/Y[positive,2,None]
    inbound=positive&(end[:,0]>=0)&(end[:,0]<=639)&(end[:,1]>=0)&(end[:,1]<=255)
    rounded=np.floor(end+.5).astype(int); indexes=np.flatnonzero(inbound)
    cells=rounded[indexes,1]*640+rounded[indexes,0]
    order=np.lexsort((indexes,Y[indexes,2],cells)); ordered=indexes[order]; ordered_cells=cells[order]
    keep=np.r_[True,ordered_cells[1:]!=ordered_cells[:-1]] if len(ordered) else np.zeros(0,bool)
    tier1=np.zeros(len(uv),bool); tier1[ordered[keep]]=True
    zt=np.zeros(len(uv)); zt[tier1]=d1[rounded[tier1,1],rounded[tier1,0]].astype(float)/256
    tier2=tier1&(zt>0)
    depth_ok=np.abs(Y[:,2]-zt)<=.1+.02*zt
    back=np.full(len(uv),np.inf); idx=np.flatnonzero(tier2)
    target_rays=np.column_stack([rounded[idx],np.ones(len(idx))])@np.linalg.inv(K).T
    Z=target_rays*zt[idx,None]; inv=np.linalg.inv(T); Z=Z@inv[:3,:3].T+inv[:3,3]
    good=Z[:,2]>0; ids=idx[good]
    back[ids]=np.linalg.norm((Z[good]@K.T)[:,:2]/Z[good,2,None]-uv[ids],axis=1)
    tier3=tier2&depth_ok&(back<=1)
    coords=(uv+.5)/2-.5; sampling=(coords[:,0]>=0)&(coords[:,0]<=319)&(coords[:,1]>=0)&(coords[:,1]<=127)
    counts={'source_depth':len(uv),'behind_camera':int((~positive).sum()),'outside_target':int((positive&~inbound).sum()),'target_collisions':int(inbound.sum()-tier1.sum()),'tier1':int(tier1.sum()),'missing_target_depth':int((tier1&~tier2).sum()),'tier2':int(tier2.sum()),'depth_disagreement':int((tier2&~depth_ok).sum()),'backward_reprojection_rejection_after_depth':int((tier2&depth_ok&~tier3).sum()),'tier3':int(tier3.sum())}
    return {'source_uv':uv,'target_uv':end,'projected_z':Y[:,2],'reference_flow':(end-uv)/2,'tier_masks':np.stack([tier1,tier2,tier3]),'sampling_mask':sampling,'counts':counts}


def sample(field, coords):
    """Nx2 coordinates inside the dense field, floating-point bilinear sampling."""
    x,y=coords.T; u=np.floor(x).astype(int); v=np.floor(y).astype(int); u1=np.minimum(u+1,field.shape[-1]-1); v1=np.minimum(v+1,field.shape[-2]-1)
    a=x-u; b=y-v
    return (field[:,v,u]*(1-a)*(1-b)+field[:,v,u1]*a*(1-b)+field[:,v1,u]*(1-a)*b+field[:,v1,u1]*a*b).T

def corr(a,b):
    from scipy.stats import spearmanr
    return float(spearmanr(a,b).statistic) if len(a)>1 and np.ptp(a)>0 and np.ptp(b)>0 else None

def error_metrics(error,alpha):
    if not len(error): return {k:None for k in ['mean_px','rmse_px','agreement1px','agreement3px','alpha_disagreement_spearman']}
    return {'mean_px':float(error.mean()),'rmse_px':float(np.sqrt(np.mean(error**2))),'agreement1px':float(np.mean(error<=1)),'agreement3px':float(np.mean(error<=3)),'alpha_disagreement_spearman':corr(alpha,error) if alpha is not None else None}

def tier_metrics(flow,alpha,reference):
    errors=np.linalg.norm(flow-reference[None],axis=-1); consistency=np.linalg.norm(flow[1]-flow[0],axis=-1)
    bad=errors[1]>1; wrong=bad&(consistency<.1)
    return {'clean':error_metrics(errors[0],alpha[0]),'gamma':error_metrics(errors[1],alpha[1]),'zero_flow':error_metrics(np.linalg.norm(reference,axis=1),None),
        'predictive_consistency':{'mean_px':float(consistency.mean()) if len(consistency) else None,'vs_clean_geometry_disagreement_spearman':corr(consistency,errors[0]),'vs_gamma_geometry_disagreement_spearman':corr(consistency,errors[1]),'gamma_disagreement_gt1_count':int(bad.sum()),'consistent_disagreement_gt1_count':int(wrong.sum()),'fraction_of_gt1_disagreements_consistent':float(wrong.sum()/bad.sum()) if bad.any() else None}}

def aggregate(rows):
    out={}
    for condition in ['day','night','rain']:
      selected=[r for r in rows if r['condition']==condition]; assert len(selected)==19
      tiers={}
      for tier in ['tier1','tier2','tier3']:
        values=[r['tiers'][tier] for r in selected]; methods={}
        for method in ['clean','gamma','zero_flow','predictive_consistency']:
          methods[method]={}
          for key in values[0][method]:
            valid=[v[method][key] for v in values if v[method][key] is not None]
            methods[method][key]={'valid_pairs':len(valid),'mean':float(np.mean(valid)) if valid else None}
        tiers[tier]={'pairs':19,'supported_pairs':sum(v['scored_points']>0 for v in values),'geometry_points':sum(v['geometry_points'] for v in values),'scored_points':sum(v['scored_points'] for v in values),'sampling_border_excluded':sum(v['sampling_border_excluded'] for v in values),'metrics':methods}
      out[condition]=tiers
    return out


def score(plan,out):
    import cv2
    lock=read(out/'prediction-lock.json'); assert lock['pose_values_parsed'] is False
    inputs={(r['condition'],r['stem']):r for r in plan['inputs']}; geometry_dir=out/'geometry'; geometry_dir.mkdir(); rows=[]
    for prediction in lock['rows']:
      c=prediction['condition']; a=inputs[c,prediction['source']]; b=inputs[c,prediction['target']]
      P0=np.loadtxt(a['rgb_pose_path']).reshape(4,4); P1=np.loadtxt(b['rgb_pose_path']).reshape(4,4)
      for P in [P0,P1]:
        assert np.isfinite(P).all() and np.max(np.abs(P[3]-[0,0,0,1]))<1e-12
        assert np.linalg.norm(P[:3,:3].T@P[:3,:3]-np.eye(3))<1e-5 and abs(np.linalg.det(P[:3,:3])-1)<1e-5
      E=np.array(plan['thermal_from_rgb'][c]); T=E@np.linalg.solve(P1,P0)@np.linalg.inv(E)
      d0=cv2.imread(a['depth_path'],cv2.IMREAD_UNCHANGED); d1=cv2.imread(b['depth_path'],cv2.IMREAD_UNCHANGED)
      geo=geometry(d0,d1,np.array(a['K']),T); uv=geo['source_uv']; coords=(uv+.5)/2-.5; sampling=geo['sampling_mask']
      pp=out/prediction['path']; assert sha(pp)==prediction['sha256']
      with np.load(pp) as z: flow=z['flow']; alpha=z['alpha']
      sampled=np.zeros((2,len(uv),2)); uncertainty=np.zeros((2,len(uv)))
      for arm in range(2):
        sampled[arm,sampling]=sample(flow[arm],coords[sampling]); uncertainty[arm,sampling]=sample(alpha[arm],coords[sampling])[:,0]
      tiers={}
      for i,tier in enumerate(['tier1','tier2','tier3']):
        geometry_mask=geo['tier_masks'][i]; mask=geometry_mask&sampling
        tiers[tier]={'geometry_points':int(geometry_mask.sum()),'scored_points':int(mask.sum()),'sampling_border_excluded':int((geometry_mask&~sampling).sum()),**tier_metrics(sampled[:,mask],uncertainty[:,mask],geo['reference_flow'][mask])}
      path=geometry_dir/(c+'-'+a['stem']+'.npz')
      np.savez_compressed(path,**{k:v for k,v in geo.items() if k!='counts'},target_from_source=T,sampled_flow=sampled,sampled_alpha=uncertainty)
      rows.append({'condition':c,'source':a['stem'],'target':b['stem'],'counts':geo['counts'],'tiers':tiers,'geometry_path':str(path.relative_to(out)),'geometry_sha256':sha(path)})
    write(out/'scores.json',{'created_utc':stamp(),'rows':rows,'recordings':aggregate(rows),'prediction_lock_sha256':sha(out/'prediction-lock.json'),'decision':'COMPLETE_DESCRIPTIVE_PROVIDED_GEOMETRY_DIAGNOSTIC_NO_METHOD_PROMOTION','scope':'Previously exposed development prefixes; nominal static geometry, not measured natural flow, physical calibration or VO accuracy.'})
    print('SCORED',len(rows),'pairs',flush=True)


def main():
    p=argparse.ArgumentParser(); p.add_argument('stage',choices=['capture','score']); p.add_argument('--protocol',required=True); p.add_argument('--output',required=True); args=p.parse_args()
    plan=read(args.protocol); verify(plan); out=Path(args.output)
    if args.stage=='capture': out.mkdir(parents=True,exist_ok=False); capture(plan,out)
    else: score(plan,out)
    verify(plan)
if __name__=='__main__': main()
