"""Source-pinned U2Flow thermal known-warp target assay; no fitting."""
import argparse,ast,hashlib,json,sys
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import numpy as np


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def write(p,d):
    with Path(p).open('x') as f:json.dump(d,f,indent=2,allow_nan=False);f.write('\n')
def recipe(path):
    import cv2
    tree=ast.parse(Path(path).read_text());node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='thermal_producer_replay');ns={'np':np,'cv2':cv2};exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),ns);return ns['thermal_producer_replay']

def capture(plan,out):
    import torch,cv2
    source=Path(plan['source_root']);sys.path.insert(0,str(source));sys.path.insert(0,str(source/'core'))
    from core.Networks.raft_2f_u import RAFT
    from yacs.config import CfgNode
    torch.set_num_threads(1);torch.set_num_interop_threads(1);torch.manual_seed(plan['seed']);torch.use_deterministic_algorithms(True)
    model=RAFT(CfgNode({'iters':12,'mixed_precision':False})).cpu().eval()
    state=torch.load(plan['checkpoint'],map_location='cpu',weights_only=True);assert all(k.startswith('module.') for k in state)
    model.load_state_dict({k[7:]:v for k,v in state.items()},strict=True)
    statehash=lambda:{k:hashlib.sha256(v.detach().numpy().tobytes()).hexdigest() for k,v in model.state_dict().items()}
    before=statehash();hookcalls=[]
    def hook(module,args,value):
        x=value[-1].detach();assert torch.isfinite(x).all();hookcalls.append([float(x.min()),float(x.max())])
    handle=model.update_block.register_forward_hook(hook);prepare=recipe(plan['preprocess_source']);root=Path(plan['project_root'])
    folder=out/'capture';folder.mkdir();predictions=[]
    with torch.inference_mode():
      for row in plan['inputs']:
        raw=cv2.imread(str(root/row['path']),cv2.IMREAD_UNCHANGED);assert raw.dtype==np.uint16 and raw.shape==(256,640)
        original=raw.copy();u8=prepare(raw);assert np.array_equal(raw,original)
        u8=cv2.resize(u8,(320,128),interpolation=cv2.INTER_AREA);image=u8.astype(np.float32)/255
        shifted=np.zeros_like(image);shifted[:-3,4:]=image[3:,:-4]
        targets=[image,shifted,image**np.float32(1.6),shifted**np.float32(1.6)]
        flows=[];alphas=[]
        for target in targets:
            pair=np.repeat(np.stack([image,target])[:,None],3,axis=1);x=torch.from_numpy(pair)[None]
            y=model(x,only_fw=True);assert len(y['flows_f12'])==len(y['uncertainty_f12'])==12
            assert all(torch.isfinite(v).all() for values in y.values() for v in values)
            flows.append(y['flows_f12'][-1][0].numpy());alphas.append(y['uncertainty_f12'][-1][0].numpy())
        path=folder/(row['condition']+'-'+row['stem']+'.npz');np.savez_compressed(path,source_uint8=u8,flow=np.stack(flows),alpha=np.stack(alphas))
        entry={'condition':row['condition'],'stem':row['stem'],'path':str(path.relative_to(out)),'sha256':sha(path),'raw_sha256':row['sha256'],'processed_sha256':hashlib.sha256(u8.tobytes()).hexdigest()};predictions.append(entry)
        with (out/'journal.jsonl').open('a') as f:f.write(json.dumps(entry)+'\n')
        print(row['condition'],row['stem'],'4cases locked',flush=True)
    handle.remove();assert before==statehash();assert len(hookcalls)==2880
    write(out/'prediction-lock.json',{'created':datetime.now(ZoneInfo('America/New_York')).isoformat(),'rows':predictions,'upstream_finite_uncertainty_calls':len(hookcalls),'model_state_unchanged':True,'scope':'All240predictions locked before scoring. Synthetic labels fixed in protocol, no real pose/flow references.'})

def metrics(flow,alpha,mask,dx,dy):
    from scipy.stats import spearmanr
    e=np.hypot(flow[0].astype(float)-dx,flow[1].astype(float)-dy)[mask];u=alpha[0][mask].astype(float)
    corr=float(spearmanr(u,e).statistic) if np.ptp(u)>0 and np.ptp(e)>0 else None
    order=np.argsort(u,kind='stable');oracle=np.sort(e);ce=np.cumsum(e[order]);co=np.cumsum(oracle);ret=np.linspace(1,.05,20);n=np.maximum(1,np.floor(ret*len(e)).astype(int));gap=ce[n-1]/n-co[n-1]/n
    ause=float(np.trapz(gap/e.mean(),x=1-ret)) if e.mean()>0 else None
    return {'pixels':len(e),'finite_fraction':float(np.isfinite(flow).all(axis=0)[mask].mean()),'epe_mean':float(e.mean()),'epe_rmse':float(np.sqrt(np.mean(e*e))),'success1px':float(np.mean(e<=1)),'success3px':float(np.mean(e<=3)),'alpha_epe_spearman':corr,'normalized_ause_0to95':ause}

def score(plan,out):
    from scipy.stats import spearmanr
    lock=read(out/'prediction-lock.json');rows=[];h,w=128,320;yy,xx=np.mgrid[:h,:w];masks=[np.ones((h,w),bool),(xx+4<w)&(yy-3>=0)];truth=[(0,0),(4,-3)]
    for row in lock['rows']:
      p=out/row['path'];assert sha(p)==row['sha256']
      with np.load(p) as z:flow=z['flow'];alpha=z['alpha']
      cases={}
      for i,name in enumerate(plan['cases']):cases[name]=metrics(flow[i],alpha[i],masks[i%2],*truth[i%2])
      pairs={}
      for i,name in [(0,'identity'),(1,'shift')]:
        mask=masks[i];dx,dy=truth[i];actual=np.hypot(flow[i+2,0].astype(float)-dx,flow[i+2,1].astype(float)-dy)[mask];disagreement=np.linalg.norm(flow[i+2].astype(float)-flow[i].astype(float),axis=0)[mask];bad=actual>1;wrong=bad&(disagreement<.1)
        corr=float(spearmanr(disagreement,actual).statistic) if np.ptp(disagreement)>0 and np.ptp(actual)>0 else None
        pairs[name]={'disagreement_epe_spearman':corr,'disagreement_mean_px':float(disagreement.mean()),'error_gt1_count':int(bad.sum()),'consistent_wrong_count':int(wrong.sum()),'consistent_wrong_fraction_all':float(wrong.mean()),'consistent_wrong_fraction_bad':float(wrong.sum()/bad.sum()) if bad.any() else None,'pixels':len(actual)}
      rows.append({**row,'cases':cases,'pairs':pairs})
    aggregate={}
    for c in ['day','night','rain']:
      selected=[r for r in rows if r['condition']==c];assert len(selected)==20
      def mean(items,k):
        x=[r[k] for r in items if r[k] is not None];return {'valid_frames':len(x),'mean':float(np.mean(x)) if x else None}
      aggregate[c]={'cases':{name:{k:mean([r['cases'][name] for r in selected],k) for k in selected[0]['cases'][name]} for name in plan['cases']},'pairs':{name:{k:mean([r['pairs'][name] for r in selected],k) for k in selected[0]['pairs'][name]} for name in ['identity','shift']}}
    write(out/'scores.json',{'rows':rows,'recordings':aggregate,'decision':'DESCRIPTIVE_TARGET_ASSAY_COMPLETE_NO_METHOD_PROMOTION','prediction_lock_sha256':sha(out/'prediction-lock.json'),'scope':'Equal-frame means of per-image metrics, not natural thermal flow, physical calibration, VO accuracy or fresh-test evidence.'});print(json.dumps({'status':'COMPLETE','frames':len(rows),'predictions':len(rows)*4}))

def main():
    a=argparse.ArgumentParser();a.add_argument('stage',choices=['capture','score']);a.add_argument('--protocol',required=True);a.add_argument('--output',required=True);v=a.parse_args();p=read(v.protocol);out=Path(v.output)
    for n,h in p['source_sha256'].items():assert sha(n)==h,n
    assert sha(p['checkpoint'])==p['checkpoint_sha256']
    for row in p['inputs']:assert sha(Path(p['project_root'])/row['path'])==row['sha256']
    if v.stage=='capture':out.mkdir(parents=True,exist_ok=False);capture(p,out)
    else:score(p,out)
    for row in p['inputs']:assert sha(Path(p['project_root'])/row['path'])==row['sha256']
if __name__=='__main__':main()
