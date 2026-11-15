"""Released-checkpoint CPU/FP32 interface qualification; synthetic images only."""
import argparse,ast,hashlib,json,sys,platform
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import numpy as np
import torch
from scipy.ndimage import gaussian_filter
from yacs.config import CfgNode


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main(protocol,output):
    plan=json.loads(Path(protocol).read_text());out=Path(output);assert not out.exists()
    assert sha(__file__)==plan['runner_sha256']
    for n,h in plan['sources'].items():assert sha(n)==h,n
    assert sha(plan['checkpoint'])==plan['checkpoint_sha256']
    source=Path(plan['source_root']);sys.path.insert(0,str(source));sys.path.insert(0,str(source/'core'))
    from core.Networks.raft_2f_u import RAFT
    from core.utils.utils import InputPadder
    torch.set_num_threads(1);torch.set_num_interop_threads(1);torch.manual_seed(plan['seed']);torch.use_deterministic_algorithms(True)
    cfg=CfgNode({'iters':12,'mixed_precision':False});model=RAFT(cfg).cpu().eval()
    state=torch.load(plan['checkpoint'],map_location='cpu',weights_only=True)
    container_type=type(state).__name__
    if isinstance(state,dict) and 'state_dict' in state:state=state['state_dict']
    stripped=bool(state) and all(k.startswith('module.') for k in state)
    if stripped:state={k.removeprefix('module.'):v for k,v in state.items()}
    result=model.load_state_dict(state,strict=True)
    assert not result.missing_keys and not result.unexpected_keys
    before={k:hashlib.sha256(v.detach().numpy().tobytes()).hexdigest() for k,v in model.state_dict().items()}
    raw_uncertainty=[]
    def hook(module,args,value):
        u=value[-1].detach();assert torch.isfinite(u).all();raw_uncertainty.append([float(u.min()),float(u.max())])
    handle=model.update_block.register_forward_hook(hook)
    rng=np.random.default_rng(plan['seed']);h,w=125,157
    gray=gaussian_filter(rng.normal(size=(h,w)),1.0);gray=(gray-gray.min())/(gray.max()-gray.min());gray=np.round(gray*255).astype(np.uint8)
    shifted=np.zeros_like(gray);dx,dy=4,-3;shifted[:h+dy,dx:]=gray[-dy:,:w-dx]
    yy,xx=np.mgrid[:h,:w];valid=(xx+dx>=0)&(xx+dx<w)&(yy+dy>=0)&(yy+dy<h)
    assert np.array_equal(shifted[(yy+dy)[valid],(xx+dx)[valid]],gray[valid])
    cases=[('identity',gray,np.ones((h,w),bool),np.zeros((2,h,w))),('shift4minus3',shifted,valid,np.broadcast_to(np.array([dx,dy])[:,None,None],(2,h,w)))]
    arrays={};rows=[]
    with torch.inference_mode():
      for name,target,mask,truth in cases:
        im=np.stack([gray,target]);im=np.repeat(im[:,None],3,axis=1).astype(np.float32)/255
        imgs=torch.from_numpy(im)[None];pad=InputPadder(imgs.shape);padded=pad.pad(imgs)
        assert tuple(padded.shape)==(1,2,3,128,160)
        assert torch.equal(pad.unpad(padded),imgs)
        a=model(padded,only_fw=True);b=model(padded,only_fw=True)
        assert len(a['flows_f12'])==len(a['uncertainty_f12'])==12
        assert all(torch.equal(x,y) for key in a for x,y in zip(a[key],b[key]))
        assert all(torch.isfinite(x).all() for key in a for x in a[key])
        flow=pad.unpad(a['flows_f12'][-1])[0].numpy();alpha=pad.unpad(a['uncertainty_f12'][-1])[0].numpy()
        assert flow.shape==(2,h,w) and alpha.shape==(1,h,w)
        error=np.linalg.norm(flow-truth,axis=0);variance_proxy=np.exp(alpha.astype(np.float64));assert np.isfinite(variance_proxy).all()
        arrays[name+'_flow']=flow;arrays[name+'_alpha']=alpha;arrays[name+'_input_uint8']=np.stack([gray,target]);arrays[name+'_mask']=mask;arrays[name+'_truth']=truth
        rows.append({'case':name,'valid_pixels':int(mask.sum()),'epe_mean':float(error[mask].mean()),'epe_rmse':float(np.sqrt(np.mean(error[mask]**2))),'median_flow_xy':[float(np.median(flow[i][mask])) for i in range(2)],'alpha_min':float(alpha.min()),'alpha_max':float(alpha.max()),'repeat_bitwise_equal':True})
      # Execute the exact author resize_flow function AST, avoiding unrelated
      # cv2/file-reader imports. Constant fields give independently known units.
      path=source/'core/utils/flow_utils.py';tree=ast.parse(path.read_text());node=next(x for x in tree.body if isinstance(x,ast.FunctionDef) and x.name=='resize_flow')
      namespace={'torch':torch};exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),namespace)
      constant=torch.empty((1,2,32,40));constant[:,0]=2;constant[:,1]=-3
      resized=namespace['resize_flow'](constant,(64,120));assert torch.allclose(resized[:,0],torch.tensor(6.)) and torch.allclose(resized[:,1],torch.tensor(-6.))
    handle.remove()
    after={k:hashlib.sha256(v.detach().numpy().tobytes()).hexdigest() for k,v in model.state_dict().items()};assert before==after
    for n,hsh in plan['sources'].items():assert sha(n)==hsh,n
    assert sha(plan['checkpoint'])==plan['checkpoint_sha256']
    out.mkdir(parents=True);np.savez_compressed(out/'arrays.npz',**arrays)
    import torchvision,scipy,matplotlib
    report={'status':'PASS_RELEASED_CHECKPOINT_CPU_FP32_INTERFACE_ONLY','run_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'checkpoint_sha256':plan['checkpoint_sha256'],'source_commit':plan['source_commit'],'checkpoint_container':container_type,'module_prefix_removed':stripped,'state_tensors':len(state),'parameters':sum(p.numel() for p in model.parameters()),'state_unchanged':True,'cases':rows,'raw_uncertainty_hook_calls':len(raw_uncertainty),'raw_uncertainty_all_finite':True,'coordinate_controls':{'known_shift_pixel_identity':True,'pad_unpad_exact':True,'constant_flow_resize_xy': [6.,-6.]},'runtime':{'python':sys.version,'torch':torch.__version__,'torchvision':torchvision.__version__,'numpy':np.__version__,'scipy':scipy.__version__,'matplotlib':matplotlib.__version__,'platform':platform.platform(),'threads':1,'device':'cpu','mixed_precision':False},'protocol_sha256':sha(protocol),'arrays_sha256':sha(out/'arrays.npz'),'scope':'No thermal data, training, author benchmark reproduction or learned-accuracy gate. Author forward only, CPUFP32 port. Scalar alpha retained unmodified; exp(alpha) is not a certified2D covariance.'}
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--protocol',required=True);p.add_argument('--output',required=True);a=p.parse_args();main(a.protocol,a.output)
