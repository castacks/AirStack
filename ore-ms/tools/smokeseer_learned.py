"""Existing learned frontend adapter; reuse frozen SIFT geometry/scoring code."""
import argparse,gc,hashlib,json,sys,time
from pathlib import Path
from datetime import datetime,timezone
import cv2,numpy as np,torch
from qualify_native_frontend import load_model,deterministic
import smokeseer_sift as backend

ROOT=Path(__file__).resolve().parents[1]; OUT=ROOT/'results/smokeseer-learned'
PREF=ROOT/'results/smokeseer-preflight'; OLD=ROOT/'results/smokeseer-sift'
AXIS=np.rint(np.linspace(8,791,64)).astype(np.int64)
XX,YY=np.meshgrid(AXIS,AXIS)
GRID=np.column_stack([XX.ravel(),YY.ravel()]).astype(np.float64)
WEIGHTS={'rgb':ROOT/'models/macslam/MACVO_FrontendCov.pth','thermal':ROOT/'models/macslam/MACT_Offroad.pth'}

def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(p.read_text())
def write(p,d):
 with p.open('x') as f:json.dump(d,f,indent=2,allow_nan=False);f.write('\n')
def setup():
 deterministic();torch.set_num_threads(1);cv2.setNumThreads(1)
def tensors(row,pins):
 arrays=[np.load(ROOT/pins[row[k]]['path'],allow_pickle=False) for k in ['source','target']]
 return [torch.from_numpy(x.copy()).float()[None,None].repeat(1,3,1,1).cuda()/255 for x in arrays]
def infer(model,a,b):
 with torch.inference_mode():flow,cov=model.inference(torch.cat([a,b]),torch.cat([b,a]))
 assert flow.shape==cov.shape==(2,2,800,800)
 assert torch.isfinite(flow).all() and torch.isfinite(cov).all() and (cov>0).all()
 return flow.cpu().numpy(),cov.cpu().numpy()
def sample(field,points):
 x,y=points.T; x0=np.floor(x).astype(int);y0=np.floor(y).astype(int)
 assert (x0>=0).all() and (y0>=0).all() and (x0<=799).all() and (y0<=799).all()
 x1=np.minimum(x0+1,799);y1=np.minimum(y0+1,799);dx=x-x0;dy=y-y0
 return ((1-dx)*(1-dy))[:,None]*field[:,y0,x0].T+(dx*(1-dy))[:,None]*field[:,y0,x1].T+((1-dx)*dy)[:,None]*field[:,y1,x0].T+(dx*dy)[:,None]*field[:,y1,x1].T

def correspondences(flow,cov):
 forward=flow[0,:,GRID[:,1].astype(int),GRID[:,0].astype(int)]
 # NumPy advanced indexing yields(N,2), not(2,N).
 assert forward.shape==(4096,2)
 target=GRID+forward
 inside=np.isfinite(target).all(1)&(target>=0).all(1)&(target<=799).all(1)
 ids=np.flatnonzero(inside);backward=sample(flow[1],target[ids]);cycle=np.linalg.norm(forward[ids]+backward,axis=1)
 accepted=ids[cycle<=1.]
 evidence={'grid':GRID,'forward':forward,'target':target,'inside':inside,'inside_ids':ids,
           'backward_at_target':backward,'cycle':cycle,'accepted_ids':accepted,
           'source_covariance':cov[0,:,GRID[:,1].astype(int),GRID[:,0].astype(int)]}
 return GRID[accepted],target[accepted],evidence

def verify():
 plan=read(OUT/'execution-freeze.json')
 for p,h in plan['pins'].items():assert sha(ROOT/p)==h,p
 backend.verify_input_freeze()
 return read(OUT/'gray-pins.json')

def qualify():
 setup();backend.verify_execution()
 pins=read(OLD/'gray-pins.json');pairs=read(PREF/'declared-pairs.json');checks=0
 for member,pin in pins.items():
  p=ROOT/pin['path'];assert sha(p)==pin['sha256'];a=np.load(p,allow_pickle=False);assert a.dtype==np.uint8 and a.shape==(800,800);checks+=2
 # Exact known translation validates flow sign, bilinear interpolation and cycle.
 flow=np.zeros((2,2,800,800),np.float32);flow[0,0]=2.25;flow[0,1]=-.5;flow[1,0]=-2.25;flow[1,1]=.5
 a,b,e=correspondences(flow,np.ones_like(flow));assert len(a)==4096;np.testing.assert_allclose(b-a,np.tile([2.25,-.5],(4096,1)));assert not np.any(e['cycle']);checks+=3
 flow[1,0]=0
 a,b,_=correspondences(flow,np.ones_like(flow));assert len(a)==0;checks+=1
 files={Path(__file__).resolve(),ROOT/'tools/qualify_native_frontend.py',ROOT/'tools/smokeseer_sift.py',ROOT/'docs/research/smokeseer-learned-protocol-2026-11-15.md',*WEIGHTS.values()}
 for folder in ['FlowFormer','FlowFormerCov']:files.update((ROOT/'MAC-VO-main/Module/Network'/folder).rglob('*.py'))
 files.update(ROOT/p for p in read(PREF/'freeze.json')['pins'])
 files.update([PREF/'freeze.json',OLD/'gray-pins.json',OUT/'qualification-freeze.json',OUT/'qualification.json'])
 write(OUT/'batch-qualification-freeze.json',{'pins':{str(p.relative_to(ROOT)):sha(p) for p in sorted(files)},'scope':'Batch2full-size interface only, no poses'})
 reports=[]
 for label in ['rgb','thermal']:
  model,loading=load_model(WEIGHTS[label])
  assert loading['migration']==('none' if label=='thermal' else 'flowformer-pre-cleanup-rgb')
  states={k:hashlib.sha256(v.detach().cpu().numpy().tobytes()).hexdigest() for k,v in model.state_dict().items()}
  for arm in (['rgb_smoke','rgb_clean'] if label=='rgb' else ['thermal']):
   row=next(x for x in pairs if x['arm']==arm);a,b=tensors(row,pins);torch.cuda.reset_peak_memory_stats();outputs=[];seconds=[]
   for repeat in range(2):
    torch.cuda.synchronize();start=time.perf_counter();result=infer(model,a,b);torch.cuda.synchronize();seconds.append(time.perf_counter()-start);outputs.append(result)
   assert all(np.array_equal(x,y) for x,y in zip(outputs[0],outputs[1]));checks+=1
   p=OUT/(arm+'-batch-qualification.npz');np.savez_compressed(p,flow=outputs[0][0],covariance=outputs[0][1]);reports.append({'arm':arm,'seconds':seconds,'peak_memory_bytes':torch.cuda.max_memory_allocated(),'sha256':sha(p),'loading':loading})
   del a,b,outputs,result;gc.collect();torch.cuda.empty_cache()
  assert states=={k:hashlib.sha256(v.detach().cpu().numpy().tobytes()).hexdigest() for k,v in model.state_dict().items()};checks+=1
  del model;gc.collect();torch.cuda.empty_cache()
 freeze=read(OUT/'batch-qualification-freeze.json')
 for p,h in freeze['pins'].items():assert sha(ROOT/p)==h,p
 write(OUT/'batch-qualification.json',{'status':'pass','checks':checks,'reports':reports,'pose_predictions':0,'opencv':cv2.__version__,'torch':torch.__version__,'gpu':torch.cuda.get_device_name(0),'completed_utc':datetime.now(timezone.utc).isoformat()})
 write(OUT/'gray-pins.json',pins)
 final=dict(freeze['pins']);final.update({str(p.relative_to(ROOT)):sha(p) for p in [OUT/'batch-qualification.json',OUT/'batch-qualification-freeze.json',OUT/'gray-pins.json']})
 write(OUT/'execution-freeze.json',{'pins':final,'runtime_image':read(PREF/'freeze.json')['runtime_image'],'query_grid_sha256':hashlib.sha256(GRID.tobytes()).hexdigest(),'created_utc':datetime.now(timezone.utc).isoformat(),'scope':'Full population monocular learned frontend; no training, no pose reference during inference'})
 print(json.dumps(read(OUT/'batch-qualification.json'),indent=2))

def run():
 setup();pins=verify();rows=read(PREF/'declared-pairs.json');assert len(rows)==3465
 for pin in pins.values():assert sha(ROOT/pin['path'])==pin['sha256']
 packets=OUT/'packets';packets.mkdir(exist_ok=False);started=time.monotonic();calls=0;canonical={}
 with (OUT/'journal.jsonl').open('x') as journal:
  for label in ['rgb','thermal']:
   model,loading=load_model(WEIGHTS[label]);states={k:hashlib.sha256(v.detach().cpu().numpy().tobytes()).hexdigest() for k,v in model.state_dict().items()}
   for slot,row in enumerate(rows):
    if ('thermal' if row['arm']=='thermal' else 'rgb')!=label:continue
    begin=time.monotonic();a,b=tensors(row,pins);flow,cov=infer(model,a,b);calls+=1
    p0,p1,evidence=correspondences(flow,cov);result=backend.pose(p0,p1,np.asarray(row['K']))
    packet=packets/f'{slot:04}.npz';np.savez_compressed(packet,**evidence)
    result.update(slot=slot,scene=row['scene'],arm=row['arm'],index=row['index'],source=row['source'],target=row['target'],
                  points0=p0.tolist(),points1=p1.tolist(),query_budget=4096,inside=int(evidence['inside'].sum()),cycle_accepted=len(p0),
                  packet=str(packet.relative_to(ROOT)),packet_sha256=sha(packet),elapsed_seconds=time.monotonic()-begin)
    assert slot not in canonical;canonical[slot]=result;journal.write(json.dumps(result,allow_nan=False)+'\n');journal.flush()
    del a,b,flow,cov,evidence
    if calls%50==0:print('Completed',calls,'/3465 elapsed',round(time.monotonic()-started,1),flush=True)
   assert states=={k:hashlib.sha256(v.detach().cpu().numpy().tobytes()).hexdigest() for k,v in model.state_dict().items()}
   del model;gc.collect();torch.cuda.empty_cache()
 assert len(canonical)==3465 and calls==3465
 with (OUT/'predictions.jsonl').open('x') as f:
  for i in range(len(rows)):f.write(json.dumps(canonical[i],allow_nan=False)+'\n')
 verify()
 write(OUT/'prediction-lock.json',{'rows':3465,'inference_batches':calls,'directional_pairs':calls*2,'elapsed_seconds':time.monotonic()-started,
       'predictions_sha256':sha(OUT/'predictions.jsonl'),'execution_freeze_sha256':sha(OUT/'execution-freeze.json'),
       'journal_sha256':sha(OUT/'journal.jsonl'),'completed_utc':datetime.now(timezone.utc).isoformat(),'scoring_started':False,'model_states_unchanged':True})

def score():
 # The frozen scorer accepts identical record fields. Its path is the sole adapter;
 # its verification/geometry/threshold/aggregation implementation is unchanged.
 backend.OUT=OUT
 backend.score()

if __name__=='__main__':
 parser=argparse.ArgumentParser();parser.add_argument('stage',choices=['qualify','run','score']);stage=parser.parse_args().stage
 {'qualify':qualify,'run':run,'score':score}[stage]()
