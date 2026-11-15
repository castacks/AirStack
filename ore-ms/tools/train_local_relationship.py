"""Bounded local thermal task gate through the actual finite motion solver."""
import argparse,hashlib,json,math,time
from pathlib import Path
import numpy as np,torch
import nominal_joint_motion as native
R=Path(__file__).resolve().parents[1];REC=R/'results/thermal-incremental-comparison/recovery';CACHE=R/'results/local-relationship-contract/cache-001';OUT=R/'results/local-relationship-learning/run-001';MODES=['constant','affine','local','context','all','rgb_only']
torch.set_num_threads(1);torch.set_num_interop_threads(1);torch.set_default_dtype(torch.float64);torch.use_deterministic_algorithms(True)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def write(p,v):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def skew(v):
 x,y,z=v.unbind();o=x*0;return torch.stack([o,-z,y,z,o,-x,-y,x,o]).reshape(3,3)
def tensors(z):return {k:torch.tensor(v) for k,v in z.items()}
def packet(path):
 with np.load(path) as z:return tensors({k:z[k] for k in z.files})
def solve(p,weights=None):
 n=len(p['points']);weights=torch.ones(n) if weights is None else weights;L=torch.linalg.cholesky(p['covariance']);rot=torch.eye(3);t=torch.zeros(3);costs=[]
 for _ in range(12):
  q=p['points']@rot.T+t;c=torch.einsum('nij,nj->ni',p['camera_R'],q)+p['camera_t'];x,y,z=c.unbind(1)
  if bool(torch.any(z.detach()<=0)):raise ValueError('invalid_target_depth')
  fx,fy,cx,cy,b=p['Kb'].unbind(1);o=z*0
  uv=torch.stack([fx*x/z+cx,fy*y/z+cy,fx*b/z],1);D=torch.stack([fx/z,o,-fx*x/z**2,o,fy/z,-fy*y/z**2,o,o,-fx*b/z**2],1).reshape(n,3,3)
  xx,yy,zz=q.unbind(1);zero=xx*0;W=torch.stack([zero,-zz,yy,zz,zero,-xx,-yy,xx,zero],1).reshape(n,3,3);J=D@p['camera_R']@torch.cat([torch.eye(3).expand(n,3,3),-W],2)
  rr=torch.linalg.solve_triangular(L,(uv-p['observations']).unsqueeze(-1),upper=False).squeeze(-1);AA=torch.linalg.solve_triangular(L,J,upper=False);w=torch.sqrt(weights)
  r=(w[:,None]*rr).reshape(-1);A=(w[:,None,None]*AA).reshape(-1,6);costs.append(float((r.detach()@r.detach())))
  sv=torch.linalg.svdvals(A.detach());
  if float(sv[-1]/sv[0])<1e-10:raise ValueError('degenerate_geometry')
  H=A.T@A;g=A.T@r;delta=torch.linalg.solve(H+1e-8*torch.diag(torch.diagonal(H)),-g);W=skew(delta[3:]);theta=torch.linalg.vector_norm(delta[3:]);dR=torch.eye(3)+torch.sinc(theta/torch.pi)*W+.5*torch.sinc(theta/(2*torch.pi))**2*(W@W);rot=dR@rot;t=dR@t+delta[:3]
 T=torch.cat([torch.cat([rot,t[:,None]],1),torch.tensor([[0.,0,0,1]])],0)
 # Check the final objective with the independently restored original projection.
 with torch.no_grad():
  uv,_=native.project(p['points'].numpy(),p['camera_R'].numpy(),p['camera_t'].numpy(),p['Kb'].numpy(),T.detach().numpy());res=np.linalg.solve(np.linalg.cholesky(p['covariance'].numpy()),(uv-p['observations'].numpy())[:,:,None])[:,:,0];final=float(np.sum(weights.detach().numpy()[:,None]*res*res))
 if final>costs[0]+1e-9*max(1,costs[0]):raise ValueError('objective_increased')
 if not bool(torch.isfinite(T.detach()).all()):raise ValueError('nonfinite_pose')
 # Exactly invert rigid transform to original target-to-source convention.
 result=torch.cat([torch.cat([rot.T,(-rot.T@t)[:,None]],1),torch.tensor([[0.,0,0,1]])],0)
 return result

def data():
 lock=read(CACHE/'lock.json');features={}
 for x in lock['rows']:
  p=CACHE/x['feature_path'];assert sha(p)==x['sha256']
  with np.load(p) as z:features[x['condition'],x['pair'],x['camera']]=torch.tensor(z['features'])
 dataset={}
 for c in ['day','night','rain']:
  for row in read(REC/f'binding-{c}.json')['rows']:
   ps={};fs={}
   for cam in ['rgb','thermal']:
    e=row['cameras'][cam]
    if not e['valid']:continue
    p=REC/e['packet'];assert sha(p)==e['sha256'];ps[cam]=packet(p);fs[cam]=features[c,row['pair'],cam]
   dataset[c,row['pair']]={'packets':ps,'features':fs}
 return dataset
class Gate(torch.nn.Module):
 def __init__(self,mode):
  super().__init__();self.mode=mode;torch.manual_seed(0)
  if mode=='constant':self.bias=torch.nn.Parameter(torch.tensor(math.log(.1/.9)))
  elif mode=='affine':self.linear=torch.nn.Linear(5,1);torch.nn.init.zeros_(self.linear.weight);torch.nn.init.constant_(self.linear.bias,math.log(.1/.9))
  else:
   self.first=torch.nn.Linear(24,16);torch.nn.init.xavier_uniform_(self.first.weight);torch.nn.init.zeros_(self.first.bias);self.last=torch.nn.Linear(16,1);torch.nn.init.zeros_(self.last.weight);torch.nn.init.constant_(self.last.bias,math.log(.1/.9))
 def forward(self,F,mean,scale):
  x=(F-mean)/scale
  if self.mode=='constant':return torch.sigmoid(self.bias).expand(len(x))
  if self.mode=='affine':return torch.sigmoid(self.linear(x[:,[4,6,7,2,3]])).flatten()
  if self.mode=='local':x=torch.cat([x[:,:16],x[:,16:]*0],1)
  if self.mode=='context':x=torch.cat([x[:,:16]*0,x[:,16:]],1)
  return torch.sigmoid(self.last(torch.tanh(self.first(x)))).flatten()
def inference(entry,model,mean,scale):
 ps=entry['packets'];cam='rgb' if model.mode=='rgb_only' else 'thermal'
 if cam not in ps:return solve(ps['rgb'])
 weights=model(entry['features'][cam],mean,scale)
 if model.mode=='rgb_only':return solve(ps['rgb'],weights)
 p={k:torch.cat([ps['rgb'][k],ps['thermal'][k]]) for k in ps['rgb']};return solve(p,torch.cat([torch.ones(len(ps['rgb']['points'])),weights]))
def qualify():
 OUT.mkdir(parents=True,exist_ok=False);ds=data();proof=[]
 for c in ['day','night','rain']:
  e=ds[c,0];p,q=e['packets']['rgb'],e['packets']['thermal']
  for gate in [0.,.1,1.]:
   pp={k:torch.cat([p[k],q[k]]) for k in p};actual=solve(p) if gate==0 else solve(pp,torch.cat([torch.ones(len(p['points'])),torch.full((len(q['points']),),gate)]));pn={k:v.numpy() for k,v in p.items()};qn={k:v.numpy().copy() for k,v in q.items()};qn['covariance']/=gate if gate>0 else 1
   expected=np.asarray(native.solve([pn] if gate==0 else [pn,qn],'cov',native.CONTRACT)['transform']);err=float(np.max(np.abs(actual.detach().numpy()-expected)));assert err<1e-8,(c,gate,err);proof.append({'condition':c,'gate':gate,'parity_max_error':err})
  F=e['features']['thermal'];mean=F.mean(0);scale=F.std(0,unbiased=False);scale[scale==0]=1
  for nonzero in [False,True]:
   m=Gate('all')
   if nonzero:
    with torch.no_grad():m.last.weight.copy_(.03*torch.sin(torch.arange(16))[None])
   parameters=list(m.parameters());rng=np.random.default_rng(0);direction=[torch.tensor(rng.normal(size=tuple(x.shape))) for x in parameters];norm=torch.sqrt(sum((v*v).sum() for v in direction));direction=[v/norm for v in direction]
   def objective():
    T=inference(e,m,mean,scale);return (T[:3,3]*torch.tensor([.3,-.2,.5])).sum()+.1*T[0,1]+.2*T[1,2]
   value=objective();grads=torch.autograd.grad(value,parameters);analytic=float(sum((g*v).sum() for g,v in zip(grads,direction)));orig=[v.detach().clone() for v in parameters];loss=[]
   for sign in [1,-1]:
    with torch.no_grad():
     for v,o,d in zip(parameters,orig,direction):v.copy_(o+sign*1e-5*d)
    loss.append(float(objective().detach()))
   with torch.no_grad():
    for v,o in zip(parameters,orig):v.copy_(o)
   fd=(loss[0]-loss[1])/2e-5;assert abs(fd-analytic)<=1e-6+1e-3*abs(fd),(c,nonzero,fd,analytic);proof.append({'condition':c,'nonzero':nonzero,'gradient':analytic,'finite_difference':fd})
 write(OUT/'qualification.json',{'checks':proof,'source_sha256':sha(__file__),'cache_sha256':sha(CACHE/'lock.json'),'passed':True});print('QUALIFIED',len(proof),flush=True)
def train(held,mode):
 qual=read(OUT/'qualification.json');assert qual['passed'] and qual['source_sha256']==sha(__file__);ds=data();training=[c for c in ['day','night','rain'] if c!=held];cam='rgb' if mode=='rgb_only' else 'thermal';F=torch.cat([e['features'][cam] for (c,i),e in ds.items() if c in training and cam in e['features']]);mean=F.mean(0);scale=F.std(0,unbiased=False);scale[scale==0]=1
 # Only the training recordings' reference values are opened by this process.
 labels={c:{r['pair']:torch.tensor(r['reference_delta']) for r in read(REC/f'scores-{c}.json')['rows']} for c in training};model=Gate(mode);opt=torch.optim.Adam(model.parameters(),lr=.001);rng=np.random.default_rng(0);orders={c:rng.permutation(99).tolist() for c in training};history=[];started=time.perf_counter()
 for update in range(40):
  opt.zero_grad();losses=[];pairs=[]
  for c in training:
   for j in range(3):
    i=orders[c][(3*update+j)%99];T=inference(ds[c,i],model,mean,scale);target=labels[c][i];loss=((T[:3,3]-target[:3,3])/.1).square().sum()+((T[:3,:3]-target[:3,:3])/(math.sqrt(2)*.01)).square().sum();losses.append(loss);pairs.append([c,i])
  loss=torch.stack(losses).mean();assert bool(torch.isfinite(loss));loss.backward();assert all(bool(torch.isfinite(x.grad).all()) for x in model.parameters());opt.step();history.append({'update':update+1,'loss':float(loss.detach()),'pairs':pairs})
  if (update+1)%10==0:print(held,mode,update+1,float(loss.detach()),flush=True)
 dest=OUT/held/mode;dest.mkdir(parents=True,exist_ok=False);write(dest/'model.json',{'mode':mode,'held':held,'training':training,'mean':mean.tolist(),'scale':scale.tolist(),'state':{k:v.detach().tolist() for k,v in model.state_dict().items()},'history':history,'training_seconds':time.perf_counter()-started,'source_sha256':sha(__file__)})
 predictions=[];started=time.perf_counter()
 with torch.no_grad():
  for i in range(99):
   T=inference(ds[held,i],model,mean,scale);e=ds[held,i];camera='rgb' if mode=='rgb_only' else 'thermal';w=model(e['features'][camera],mean,scale) if camera in e['features'] else torch.empty(0);predictions.append({'pair':i,'transform':T.tolist(),'gate_mean':float(w.mean()) if len(w) else None,'gate_min':float(w.min()) if len(w) else None,'gate_max':float(w.max()) if len(w) else None,'fallback':'thermal' not in e['packets'] and mode!='rgb_only'})
 write(dest/'predictions.json',{'held':held,'mode':mode,'rows':predictions,'evaluation_seconds':time.perf_counter()-started});print('COMPLETE',held,mode,flush=True)
if __name__=='__main__':
 import sys
 if sys.argv[1]=='qualify':qualify()
 else:train(sys.argv[2],sys.argv[3])
