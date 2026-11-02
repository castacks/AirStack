import os
os.environ['CUBLAS_WORKSPACE_CONFIG']=':4096:8'
from pathlib import Path
import datetime,hashlib,json,math
import numpy as np
import torch
from Train.MatchingNet import retraining_runner as base
from Train.MatchingNet import objective_retraining as policy
from Train.MatchingNet.retraining_manifest import inspect_manifest,load_pair,sha256
ROOT=Path('/root/AirStack/ore-ms');OUT=ROOT/'results/objective-comparison-preflight/run-001';OUT.mkdir(parents=True,exist_ok=False);p=json.loads((ROOT/'docs/research/objective-comparison-execution-2026-11-02.json').read_text());manifest=inspect_manifest(ROOT,ROOT/'experiments/covariance-coverage-broad-2026-11-01.json')
for rel,d in p['pins'].items():assert sha256(ROOT/rel)==d,rel
torch.manual_seed(0);torch.cuda.manual_seed_all(0);torch.set_num_threads(1);torch.use_deterministic_algorithms(True);torch.backends.cudnn.benchmark=False
model,_=base.build_model(ROOT,manifest,'cuda');names=base.configure(model);assert len(names)==24;assert sum(t.numel() for t in model.parameters() if t.requires_grad)==3075202
sample=next(s for s in manifest['samples'] if s['partition']=='train');pair={k:v.to('cuda') for k,v in load_pair(ROOT,sample).items()};assert pair['image1'].shape==(1,3,480,640)
def digest():
 h=hashlib.sha256()
 for name,t in model.state_dict().items():
  h.update(name.encode());h.update(str(t.dtype).encode());h.update(str(tuple(t.shape)).encode());h.update(t.detach().cpu().contiguous().numpy().tobytes())
 return h.hexdigest()
before=digest();rows={};shared_means=None
for mode in ('recurrent_diagonal_nll',*policy.OBJECTIVES):
 model.zero_grad(set_to_none=True);base.configure(model)
 means,raw=model(pair['image1'],pair['image2']);hashes=[hashlib.sha256(t.detach().cpu().contiguous().numpy().tobytes()).hexdigest() for t in means]
 if shared_means is None:shared_means=hashes
 assert hashes==shared_means
 class Outputs:
  def __call__(self,*args):return means,raw
 if mode=='recurrent_diagonal_nll':result=base.objective(Outputs(),pair,manifest['optimization'])
 else:
  with policy.objective_policy(mode):result=base.objective(Outputs(),pair,manifest['optimization'])
 result.loss.backward();gradients={}
 for name,t in model.named_parameters():
  if t.requires_grad:
   assert t.grad is not None and torch.isfinite(t.grad).all();g=t.grad.detach().cpu().contiguous();gradients[name]={'sha256':hashlib.sha256(g.numpy().tobytes()).hexdigest(),'squared_norm':float(g.double().square().sum())}
  else:assert t.grad is None
 assert len(gradients)==24 and digest()==before
 rows[mode]={'objective_score':result.loss.item(),'valid_components':result.valid_components,'gradient_norm_before_clip':math.sqrt(sum(g['squared_norm'] for g in gradients.values())),'gradient_sha256':gradients,'mean_sha256':hashes,'state_unchanged':True};print(mode,result.loss.item(),rows[mode]['gradient_norm_before_clip'],flush=True)
 del result,means,raw
model.zero_grad(set_to_none=True);assert digest()==before
result={'status':'PASS_REAL_NETWORK_OBJECTIVE_PREFLIGHT','created_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'sample_sha256':sample['file']['sha256'],'recording_id':sample['recording_id'],'state_before':before,'state_after':digest(),'arms':rows,'trainable_tensors':24,'trainable_scalars':3075202,'all_gradients_finite':True,'shared_mean_fields':len(shared_means),'optimizer_steps':0,'candidate_exported':False,'protocol_sha256':sha256(ROOT/'docs/research/objective-comparison-execution-2026-11-02.json')}
with (OUT/'real-preflight.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
