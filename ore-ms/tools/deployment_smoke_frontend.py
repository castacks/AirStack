"""Reference-free real-smoke adapter; reuses frozen lighting matcher primitives.

ZIP/PNG intake and MEI ray lifting differ from the frozen ROSbag/pinhole lighting
study. This module adds that input/projection contract, without changing its
frozen runner, pretrained models, matching primitives or pose solver.
"""
import argparse,concurrent.futures,hashlib,json,time,zipfile,threading
from pathlib import Path
import cv2,numpy as np,yaml
import deployment_lighting as common
from smokeseer_preflight import png_check

ROOT=common.ROOT;DATA=ROOT/'data/deployment-characterization';OUT=ROOT/'results/deployment-smoke-frontend';INTAKE=ROOT/'results/deployment-characterization'
read=common.read;write=common.write;sha=common.sha

def camera(arm):
 c=yaml.safe_load((ROOT/'results/deployment-data-search/smoke_intrinsics.yaml').read_text())[arm+'_camera'];p=c['projection_parameters'];d=c['distortion_parameters']
 k=np.array([[p.get('fx',p.get('gamma1')),0,p.get('cx',p.get('u0'))],[0,p.get('fy',p.get('gamma2')),p.get('cy',p.get('v0'))],[0,0,1.]])
 return k,np.array([d[x] for x in ['k1','k2','p1','p2']]),c.get('mirror_parameters',{}).get('xi',0),np.array([[400.,0,c['image_width']/2],[0,400.,c['image_height']/2],[0,0,1.]])
def lift(points,arm):
 if not len(points):return np.empty((0,2))
 k,d,xi,v=camera(arm);xy=cv2.undistortPoints(np.asarray(points,float)[:,None,:],k,d,criteria=(cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,100,1e-12)).reshape(-1,2)
 if xi:
  r2=np.sum(xy*xy,axis=1);disc=1+(1-xi*xi)*r2;assert (disc>0).all();lam=(xi+np.sqrt(disc))/(1+r2);rays=np.column_stack([lam[:,None]*xy,lam-xi]);assert (rays[:,2]>0).all();xy=rays[:,:2]/rays[:,2:]
 return xy*400+v[:2,2]
def project(rays,arm):
 k,d,xi,v=camera(arm);denom=rays[:,2]+xi*np.linalg.norm(rays,axis=1);xy=rays[:,:2]/denom[:,None];p,_=cv2.projectPoints(np.column_stack([xy,np.ones(len(xy))]),np.zeros(3),np.zeros(3),k,d);return p.reshape(-1,2)

def prepare():
 OUT.mkdir(exist_ok=False);cv2.setNumThreads(1);receipt=read(INTAKE/'smoke-download-receipt.json')[0];archive=DATA/'subt-smoke.zip';assert sha(archive)==receipt['sha256']
 with zipfile.ZipFile(archive) as z:
  names=sorted(n for n in z.namelist() if n.endswith('.png'));times={}
  for arm,folder in [('rgb','cam_0'),('thermal','thermal')]:
   v=[int(x) for x in z.read('SmokeRoom_Folder/'+folder+'/timestamps.txt').decode().splitlines()];assert np.all(np.diff(v)>0);times[arm]=v
 local=threading.local()
 def decode(name):
  if not hasattr(local,'archive'):local.archive=zipfile.ZipFile(archive)
  b=local.archive.read(name)
  png_check(b);a=cv2.imdecode(np.frombuffer(b,np.uint8),cv2.IMREAD_UNCHANGED);assert a is not None;arm='rgb' if '/cam_0/' in name else 'thermal';i=int(Path(name).stem);t=times[arm][i]
  if arm=='rgb':assert a.dtype==np.uint8 and a.shape==(480,640,3);gray=cv2.cvtColor(a,cv2.COLOR_BGR2GRAY)
  else:
   assert a.dtype==np.uint16 and a.shape==(512,640);lo,hi=int(a.min()),int(a.max());gray=np.zeros(a.shape,np.uint8) if hi==lo else np.rint((a.astype(float)-lo)*255/(hi-lo)).clip(0,255).astype(np.uint8)
  path=DATA/'smoke-gray'/arm/f'{i:05}.npy';path.parent.mkdir(parents=True,exist_ok=True);np.save(path,gray,allow_pickle=False);q=np.percentile(gray,[5,95]);return {'arm':arm,'index':i,'time_ns':t,'member':name,'member_sha256':hashlib.sha256(b).hexdigest(),'path':str(path.relative_to(ROOT)),'sha256':sha(path),'shape':list(a.shape),'dtype':str(a.dtype),'metrics':{'mean':float(gray.mean()),'p05_p95_range':float(q[1]-q[0]),'std':float(gray.std()),'raw_min':int(a.min()),'raw_max':int(a.max())}}
 frames=[]
 with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
  for row in pool.map(decode,names):
   frames.append(row)
   if len(frames)%3000==0:print('Decoded',len(frames),'/',len(names),flush=True)
 assert len(frames)==22800;write(OUT/'images.json',frames);finish_prepare()

def finish_prepare():
 frames=read(OUT/'images.json');receipt=read(INTAKE/'smoke-download-receipt.json')[0];times={arm:[x['time_ns'] for x in sorted((f for f in frames if f['arm']==arm),key=lambda x:x['index'])] for arm in ['rgb','thermal']}
 lookup={(x['arm'],x['index']):x for x in frames};refs=[];anchors=list(range(times['rgb'][0],times['rgb'][-1]-500_000_000+1,500_000_000))
 for i,a in enumerate(anchors):
  for arm in ['rgb','thermal']:
   ids=[common.nearest(times[arm],t) for t in [a,a+500_000_000]];rows=[lookup[(arm,j)] for j in ids];errors=[x['time_ns']-t for x,t in zip(rows,[a,a+500_000_000])];refs.append({'arm':arm,'index':i,'anchor_ns':a,'source':rows[0]['path'],'target':rows[1]['path'],'times_ns':[x['time_ns'] for x in rows],'anchor_errors_ns':errors,'qualified':max(abs(x) for x in errors)<=25_000_000 and ids[1]>ids[0],'source_metrics':rows[0]['metrics'],'target_metrics':rows[1]['metrics']})
 report=[]
 for arm,h in [('rgb',480),('thermal',512)]:
  p=common.grid(h,640);v=lift(p,arm);k=camera(arm)[3];rays=np.column_stack([(v-k[:2,2])/400,np.ones(len(v))]);error=np.linalg.norm(project(rays,arm)-p,axis=1);assert error.max()<.001;report.append({'arm':arm,'query_points':len(p),'max_roundtrip_error_px':float(error.max())})
 write(OUT/'projection-qualification.json',report);write(OUT/'intervals.json',refs);write(OUT/'preflight.json',{'decoded_images':len(frames),'anchors':len(anchors),'sensor_intervals':len(refs),'qualified':sum(x['qualified'] for x in refs),'archive_sha256':receipt['sha256'],'zero_image_motion_predictions':True})
 files={Path(__file__).resolve(),ROOT/'tools/deployment_lighting.py',ROOT/'tools/smokeseer_sift.py',ROOT/'tools/smokeseer_preflight.py',ROOT/'tools/qualify_native_frontend.py',ROOT/'docs/research/deployment-smoke-protocol-2026-11-16.md',ROOT/'results/deployment-data-search/smoke_intrinsics.yaml',OUT/'images.json',OUT/'intervals.json',OUT/'projection-qualification.json',OUT/'preflight.json',INTAKE/'smoke-download-receipt.json',ROOT/'models/macslam/MACVO_FrontendCov.pth',ROOT/'models/macslam/MACT_Offroad.pth'}
 for folder in ['FlowFormer','FlowFormerCov']:files.update((ROOT/'MAC-VO-main/Module/Network'/folder).rglob('*.py'))
 pins={str(p.relative_to(ROOT)):sha(p) for p in files};pins.update({x['path']:x['sha256'] for x in frames});write(OUT/'input-freeze.json',{'pins':pins,'runtime_image':'sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09'});print('Smoke preflight complete',len(refs),'intervals',flush=True)

def qualify():
 import torch
 from qualify_native_frontend import load_model,deterministic
 deterministic();torch.set_num_threads(1);cv2.setNumThreads(1);plan=read(OUT/'input-freeze.json');common.verify(plan['pins']);reports=[]
 for arm in ['rgb','thermal']:
  h=480 if arm=='rgb' else 512;grid=common.grid(h,640);assert np.linalg.norm(project(np.column_stack([(lift(grid,arm)-camera(arm)[3][:2,2])/400,np.ones(len(grid))]),arm)-grid,axis=1).max()<.001
  model,loading=load_model(ROOT/('models/macslam/MACVO_FrontendCov.pth' if arm=='rgb' else 'models/macslam/MACT_Offroad.pth'));states={k:hashlib.sha256(v.detach().cpu().numpy().tobytes()).hexdigest() for k,v in model.state_dict().items()};r=next(x for x in read(OUT/'intervals.json') if x['arm']==arm and x['qualified']);a,b=[np.load(ROOT/r[k],allow_pickle=False) for k in ['source','target']];f0=common.infer(model,a,b);f1=common.infer(model,a,b);assert np.array_equal(f0,f1);assert states=={k:hashlib.sha256(v.detach().cpu().numpy().tobytes()).hexdigest() for k,v in model.state_dict().items()};reports.append({'arm':arm,'shape':list(f0.shape),'bitwise_repeat':True,'loading':loading});del model,f0,f1;torch.cuda.empty_cache()
 write(OUT/'qualification.json',{'reports':reports,'pose_accuracy_claim':False,'torch':torch.__version__,'opencv':cv2.__version__,'numpy':np.__version__});pins=dict(plan['pins']);pins[str((OUT/'qualification.json').relative_to(ROOT))]=sha(OUT/'qualification.json');common.verify(pins);write(OUT/'execution-freeze.json',{'pins':pins,'runtime_image':plan['runtime_image']});print('Smoke runtime frozen',flush=True)

def run():
 import torch
 from qualify_native_frontend import load_model,deterministic
 deterministic();torch.set_num_threads(1);cv2.setNumThreads(1);common.verify(read(OUT/'execution-freeze.json')['pins']);rows=read(OUT/'intervals.json');started=time.monotonic();n=0
 with (OUT/'predictions.jsonl').open('x') as output:
  for frontend in ['sift','learned']:
   for arm in ['rgb','thermal']:
    cache={};model=None
    if frontend=='learned':model,loading=load_model(ROOT/('models/macslam/MACVO_FrontendCov.pth' if arm=='rgb' else 'models/macslam/MACT_Offroad.pth'));states={k:hashlib.sha256(v.detach().cpu().numpy().tobytes()).hexdigest() for k,v in model.state_dict().items()}
    for row in [x for x in rows if x['arm']==arm]:
     start=time.perf_counter();record={k:row[k] for k in ['arm','index','qualified']};record['frontend']=frontend
     if not row['qualified']:record.update(status='input_unsupported',matches=0)
     else:
      a,b=[np.load(ROOT/row[k],allow_pickle=False) for k in ['source','target']]
      if frontend=='learned':p,q,info=common.correspondences(common.infer(model,a,b));record.update(info)
      else:
       pairs=[]
       for k,img in zip(['source','target'],[a,b]):
        if row[k] not in cache:
         kp,desc=common.geometry.sift().detectAndCompute(img,None);cache[row[k]]=(np.array([x.pt for x in kp]).reshape(-1,2),desc)
        pairs.append(cache[row[k]])
       (kp0,d0),(kp1,d1)=pairs;matches=common.geometry.mutual_matches(d0,d1);p=np.array([kp0[i] for i,j in matches]).reshape(-1,2);q=np.array([kp1[j] for i,j in matches]).reshape(-1,2);record.update(source_keypoints=len(kp0),target_keypoints=len(kp1))
      p,q=lift(p,arm),lift(q,arm);record.update(common.geometry.pose(p,q,camera(arm)[3]));record.update(points0=p.tolist(),points1=q.tolist())
     record['seconds']=time.perf_counter()-start;output.write(json.dumps(record,allow_nan=False)+'\n');output.flush();n+=1
     if n%200==0:print('Smoke predictions',n,'elapsed',round(time.monotonic()-started,1),flush=True)
    if model is not None:
     assert states=={k:hashlib.sha256(v.detach().cpu().numpy().tobytes()).hexdigest() for k,v in model.state_dict().items()};del model;torch.cuda.empty_cache()
 common.verify(read(OUT/'execution-freeze.json')['pins']);write(OUT/'prediction-lock.json',{'rows':n,'seconds':time.monotonic()-started,'predictions_sha256':sha(OUT/'predictions.jsonl'),'scoring_started':False,'model_states_unchanged':True});print('Smoke predictions locked',n,flush=True)

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('stage',choices=['prepare','finish','qualify','run']);a=p.parse_args();{'prepare':prepare,'finish':finish_prepare,'qualify':qualify,'run':run}[a.stage]()
