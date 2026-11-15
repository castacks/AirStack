"""Isolated native thermal FlowFormerCov qualification; no estimator/MC outcomes."""
import argparse,ast,json,hashlib,sys,types,time
from pathlib import Path
from datetime import datetime,timezone
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def write(p,d):
    with Path(p).open('x') as f:json.dump(d,f,indent=2,allow_nan=False);f.write('\n')
def now():return datetime.now(timezone.utc).isoformat()
def recipe():
    import cv2
    p=ROOT/'MAC-VO-main/DataLoader/Dataset/MS2.py';node=next(x for x in ast.parse(p.read_text()).body if isinstance(x,ast.FunctionDef) and x.name=='thermal_producer_replay');ns={'np':np,'cv2':cv2};exec(compile(ast.Module(body=[node],type_ignores=[]),str(p),'exec'),ns);return ns['thermal_producer_replay']
def load_model(checkpoint):
    import torch
    # Namespace-only ancestors avoid importing unrelated dataset/optimizer plugins.
    # All model/config/checkpoint implementation bodies are imported unmodified.
    repo=ROOT/'MAC-VO-main'
    for name,path in [('Module',repo/'Module'),('Module.Network',repo/'Module/Network')]:
        if name not in sys.modules:
            module=types.ModuleType(name);module.__path__=[str(path)];sys.modules[name]=module
    from Module.Network.FlowFormer.configs.submission import get_cfg
    from Module.Network.FlowFormerCov import build_flowformer
    cfg=get_cfg();cfg.latentcostformer.pretrain=False;cfg.latentcostformer.decoder_depth=12
    model=build_flowformer(cfg,torch.float32,torch.float32).eval().cuda()
    weights=torch.load(checkpoint,map_location='cpu',weights_only=True);report=model.load_ddp_state_dict(weights)
    return model,report

def deterministic():
    import torch
    torch.set_num_threads(4);torch.manual_seed(20261114);torch.cuda.manual_seed_all(20261114)
    torch.use_deterministic_algorithms(True);torch.backends.cuda.matmul.allow_tf32=False;torch.backends.cudnn.allow_tf32=False;torch.backends.cudnn.benchmark=False;torch.backends.cudnn.deterministic=True

def main():
    import torch,cv2,timm,pypose,scipy
    ap=argparse.ArgumentParser();ap.add_argument('--protocol',required=True);ap.add_argument('--output',required=True);a=ap.parse_args();plan=read(a.protocol);out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
    for p,h in plan['pins'].items():assert sha(p)==h,p
    deterministic();assert torch.cuda.is_available();model,loading=load_model(plan['checkpoint'])
    assert loading['migration']=='none'
    prepare=recipe();case=plan['rows'][:2];frames=[]
    for row in case:
      frame=[]
      for side in ['left','right']:
        raw=cv2.imread(row[side]['path'],cv2.IMREAD_UNCHANGED);assert raw.dtype==np.uint16 and raw.shape==(256,640);frame.append(prepare(raw))
      frames.append(frame)
    # Native call sequence: source stereo, target stereo, temporal left pair.
    source=np.stack([frames[0][0],frames[1][0],frames[0][0]]);target=np.stack([frames[0][1],frames[1][1],frames[1][0]])
    first=torch.from_numpy(np.repeat(source[:,None],3,axis=1).astype(np.float32)/255).cuda();second=torch.from_numpy(np.repeat(target[:,None],3,axis=1).astype(np.float32)/255).cuda()
    states={k:hashlib.sha256(v.cpu().numpy().tobytes()).hexdigest() for k,v in model.state_dict().items()};outputs=[];times=[]
    with torch.inference_mode():
      for repeat in range(2):
        torch.cuda.synchronize();start=time.perf_counter();flow,cov=model.inference(first,second);torch.cuda.synchronize();times.append(time.perf_counter()-start)
        assert flow.shape==cov.shape==(3,2,256,640);assert torch.isfinite(flow).all() and torch.isfinite(cov).all() and (cov>0).all()
        outputs.append((flow.cpu().numpy(),cov.cpu().numpy()))
    assert all(np.array_equal(x,y) for x,y in zip(outputs[0],outputs[1]))
    assert states=={k:hashlib.sha256(v.cpu().numpy().tobytes()).hexdigest() for k,v in model.state_dict().items()}
    np.savez_compressed(out/'outputs.npz',source_uint8=source,target_uint8=target,flow=outputs[0][0],covariance=outputs[0][1])
    report={'created_utc':now(),'status':'PASS_NATIVE_FRONTEND_INTERFACE','strict_load':loading,'parameters':sum(p.numel() for p in model.parameters()),'state_tensors':len(states),'checkpoint_sha256':sha(plan['checkpoint']),'output_sha256':sha(out/'outputs.npz'),'batch_pair_order':['source_stereo','target_stereo','source_left_to_target_left'],'shape':[3,2,256,640],'finite_positive_covariance':True,'identical_input_bitwise_repeat':True,'model_state_unchanged':True,'seconds':times,'gpu':torch.cuda.get_device_name(0),'peak_memory_bytes':torch.cuda.max_memory_allocated(),'versions':{'python':sys.version.split()[0],'torch':torch.__version__,'torchvision':__import__('torchvision').__version__,'numpy':np.__version__,'opencv':cv2.__version__,'scipy':scipy.__version__,'timm':timm.__version__,'pypose':pypose.__version__},'scope':'Native thermal architecture/checkpoint and interface only; newer isolated runtime, not historical bitwise replay or complete MACI2 reproduction. No MC, selector or pose/reference outcomes.'}
    write(out/'qualification.json',report)
    for p,h in plan['pins'].items():assert sha(p)==h,p
    print(json.dumps(report))
if __name__=='__main__':main()
