"""NGCLAB lighting characterization; shared frozen matcher/geometry primitives.

A new adapter is required for ROSbag images, distorted camera models and measured
camera references. Existing SmokeSeer runners are frozen synthetic populations;
their matching/pose primitives are imported rather than copied or modified.
"""
import argparse,hashlib,json,time,sys
from pathlib import Path
import cv2,numpy as np
from scipy.spatial.transform import Rotation,Slerp
import smokeseer_sift as geometry

ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/deployment-characterization'
DATA=ROOT/'data/deployment-characterization';CAL=OUT/'calibration'
CONDITIONS=['bright','switching','dim'];STEP=500_000_000

def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def write(p,v):
 with Path(p).open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')
def stamp(m):return int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec)
def calibrations():
 result={}
 for arm,name in [('rgb','Strandard.txt'),('thermal','LWIR.txt')]:
  v=np.fromstring((CAL/name).read_text().splitlines()[0],sep=' ');assert len(v)==8
  fx,fy,cx,cy=v[:4];result[arm]={'K':[[fx,0,cx],[0,fy,cy],[0,0,1]],'D':v[4:].tolist()}
 f=cv2.FileStorage(str(CAL/'extris_LWIR_RGB.yml'),cv2.FILE_STORAGE_READ)
 r=f.getNode('R1to2').mat();t=f.getNode('t1to2').mat().reshape(3);f.release()
 u,s,vt=np.linalg.svd(r);q=u@vt;assert np.linalg.det(r)>0 and np.linalg.det(q)>0
 correction=float(np.linalg.norm(q-r));assert correction<.01
 transform=np.eye(4);transform[:3,:3]=q;transform[:3,3]=t
 return result,transform,{'original_R':r.tolist(),'singular_values':s.tolist(),'original_determinant':float(np.linalg.det(r)),'orthogonality_frobenius':float(np.linalg.norm(r.T@r-np.eye(3))),'closest_so3_correction_frobenius':correction,'thermal_to_rgb':transform.tolist(),'normalization':'SVD closest proper SO3, supplied translation unchanged; no reference/image fitting'}
def nearest(times,t):
 i=int(np.searchsorted(times,t));return min([j for j in [i-1,i] if 0<=j<len(times)],key=lambda j:(abs(int(times[j])-int(t)),j))
def interpolate(poses,t):
 times=np.array([x['time_ns'] for x in poses],np.int64);j=int(np.searchsorted(times,t))
 if j==0 or j==len(times):return None
 if times[j]-times[j-1]>50_000_000:return None
 a=(t-int(times[j-1]))/int(times[j]-times[j-1]);pair=poses[j-1:j+1]
 r=Slerp([0.,1.],Rotation.from_quat([x['q'] for x in pair]))([a]).as_matrix()[0]
 result=np.eye(4);result[:3,:3]=r;result[:3,3]=(1-a)*np.array(pair[0]['p'])+a*np.array(pair[1]['p']);return result

def prepare():
 from rosbags.rosbag1 import Reader
 from rosbags.typesys import Stores,get_typestore,get_types_from_msg
 store=get_typestore(Stores.ROS1_NOETIC);cv2.setNumThreads(1)
 cams,thermal_to_rgb,cal_report=calibrations();write(OUT/'calibration-qualification.json',cal_report)
 frames=[];references=[];report={};inputpins={};downloads=read(OUT/'download-receipts.json')
 for condition in CONDITIONS:
  receipt=next(x for x in downloads if x['label']==condition);bag=ROOT/receipt['path'];assert sha(bag)==receipt['sha256']
  index={'rgb':[],'thermal':[]};poses=[];flags=[];encodings={};started=time.monotonic()
  with Reader(bag) as reader:
   connections=[c for c in reader.connections if c.topic in ['/camera/image_raw','/optris/thermal_image','/vrpn_client_node/RigidBody/pose','/optris/flag_state']]
   for c in connections:
    if c.msgtype not in store.types:store.register(get_types_from_msg(c.msgdef.data,c.msgtype))
   for conn,record_t,data in reader.messages(connections=connections):
    m=store.deserialize_ros1(data,conn.msgtype);t=stamp(m)
    if conn.topic.endswith('/pose'):
     v=m.pose;poses.append({'time_ns':t,'p':[v.position.x,v.position.y,v.position.z],'q':[v.orientation.x,v.orientation.y,v.orientation.z,v.orientation.w]});continue
    if conn.topic.endswith('flag_state'):
     flags.append({'time_ns':t,'flag_state':int(m.flag_state)});continue
    arm='rgb' if conn.topic=='/camera/image_raw' else 'thermal';ordinal=len(index[arm]);assert m.height==480 and m.width==640
    expected='bgr8' if arm=='rgb' else 'mono16';assert m.encoding==expected and len(m.data)==m.step*m.height
    if arm=='rgb':
     assert m.step==640*3;raw=np.frombuffer(m.data,dtype=np.uint8).reshape(480,640,3);gray=cv2.cvtColor(raw,cv2.COLOR_BGR2GRAY)
    else:
     assert m.step==640*2;raw=np.frombuffer(m.data,dtype='>u2' if m.is_bigendian else '<u2').reshape(480,640).astype(np.uint16)
     lo,hi=int(raw.min()),int(raw.max());gray=np.zeros(raw.shape,np.uint8) if hi==lo else np.rint((raw.astype(np.float64)-lo)*255/(hi-lo)).clip(0,255).astype(np.uint8)
    p=DATA/'gray'/condition/arm/f'{ordinal:05}.npy';p.parent.mkdir(parents=True,exist_ok=True);np.save(p,gray,allow_pickle=False)
    q=np.percentile(gray,[1,5,50,95,99]);g=cv2.Sobel(gray,cv2.CV_32F,1,0)
    gy=cv2.Sobel(gray,cv2.CV_32F,0,1)
    row={'condition':condition,'arm':arm,'index':ordinal,'time_ns':t,'record_time_ns':int(record_t),'path':str(p.relative_to(ROOT)),'sha256':sha(p),'raw_pixel_sha256':hashlib.sha256(raw.tobytes()).hexdigest(),'metrics':{'mean':float(gray.mean()),'std':float(gray.std()),'p05_p95_range':float(q[3]-q[1]),'fraction_le10':float((gray<=10).mean()),'fraction_ge245':float((gray>=245).mean()),'mean_gradient':float(np.hypot(g,gy).mean()),'raw_p01_p99_range':float(np.diff(np.percentile(raw,[1,99]))[0]),'raw_min':int(raw.min()),'raw_max':int(raw.max())}}
    index[arm].append(row);inputpins[row['path']]=row['sha256'];encodings[arm]=expected
   for arm in index:
    assert len(index[arm])==sum(c.msgcount for c in connections if c.topic==('/camera/image_raw' if arm=='rgb' else '/optris/thermal_image'))
    assert np.all(np.diff([x['time_ns'] for x in index[arm]])>0)
   assert np.all(np.diff([x['time_ns'] for x in poses])>0)
   assert np.isfinite(np.array([x['p']+x['q'] for x in poses])).all()
   assert np.max(np.abs(np.linalg.norm([x['q'] for x in poses],axis=1)-1))<1e-4
  times={arm:np.array([x['time_ns'] for x in rows],np.int64) for arm,rows in index.items()}
  first=int(times['rgb'][0]);end=int(times['rgb'][-1]);anchors=list(range(first,end-STEP+1,STEP));declared=[]
  for i,a in enumerate(anchors):
   for arm in ['rgb','thermal']:
    ids=[nearest(times[arm],target) for target in [a,a+STEP]];rows=[index[arm][j] for j in ids];errors=[int(row['time_ns'])-target for row,target in zip(rows,[a,a+STEP])]
    reason=[]
    if max(abs(x) for x in errors)>20_000_000:reason.append('anchor_distance')
    if ids[1]<=ids[0]:reason.append('reused_or_reversed_frame')
    transforms=[interpolate(poses,row['time_ns']) for row in rows]
    if any(x is None for x in transforms):reason.append('reference_bracket')
    ref={'condition':condition,'arm':arm,'index':i,'anchor_time_ns':a,'source':rows[0]['path'],'target':rows[1]['path'],'image_indices':ids,'times_ns':[x['time_ns'] for x in rows],'anchor_errors_ns':errors,'K':cams[arm]['K'],'D':cams[arm]['D'],'qualified':not reason,'unsupported_reasons':reason,'source_metrics':rows[0]['metrics'],'target_metrics':rows[1]['metrics']}
    if not reason:
     if arm=='thermal':transforms=[x@thermal_to_rgb for x in transforms]
     rel=np.linalg.solve(transforms[1],transforms[0]);ref['source_to_next_optical']=rel.tolist();ref['translation_m']=float(np.linalg.norm(rel[:3,3]));ref['translation_eligible']=ref['translation_m']>.001
     ref['reference_rotation_deg']=float(np.degrees(Rotation.from_matrix(rel[:3,:3]).magnitude()))
    declared.append(ref)
  references+=declared;frames.extend(index['rgb']+index['thermal']);write(OUT/(condition+'-mocap.json'),poses);write(OUT/(condition+'-flags.json'),flags)
  report[condition]={'frames':{k:len(v) for k,v in index.items()},'poses':len(poses),'flags':flags,'anchors':len(anchors),'qualified_arm_intervals':sum(x['qualified'] for x in declared),'declared_arm_intervals':len(declared),'unsupported':[{k:x[k] for k in ['arm','index','unsupported_reasons']} for x in declared if not x['qualified']],'seconds':time.monotonic()-started,'image_encodings':encodings}
  print('Prepared',condition,json.dumps({k:v for k,v in report[condition].items() if k!='flags'}),flush=True)
 write(OUT/'images.json',frames);write(OUT/'references.json',references);write(OUT/'preflight.json',report)
 files=[Path(__file__),ROOT/'tools/smokeseer_sift.py',ROOT/'docs/research/deployment-lighting-protocol-2026-11-16.md',OUT/'images.json',OUT/'references.json',OUT/'preflight.json',OUT/'calibration-qualification.json',OUT/'download-receipts.json',*CAL.iterdir()]
 pins={str(p.relative_to(ROOT)):sha(p) for p in files};pins.update(inputpins)
 write(OUT/'input-freeze.json',{'pins':pins,'opencv':cv2.__version__,'numpy':np.__version__,'scope':'Full image/reference preflight; no predictions','intervals':len(references)})

def grid(h,w):
 xx,yy=np.meshgrid(np.rint(np.linspace(8,w-9,64)).astype(int),np.rint(np.linspace(8,h-9,64)).astype(int));return np.column_stack([xx.ravel(),yy.ravel()]).astype(float)
def sample(field,points):
 h,w=field.shape[1:];x,y=points.T;x0=np.floor(x).astype(int);y0=np.floor(y).astype(int);assert (x0>=0).all() and (y0>=0).all() and (x0<w).all() and (y0<h).all()
 x1=np.minimum(x0+1,w-1);y1=np.minimum(y0+1,h-1);dx=x-x0;dy=y-y0
 return ((1-dx)*(1-dy))[:,None]*field[:,y0,x0].T+(dx*(1-dy))[:,None]*field[:,y0,x1].T+((1-dx)*dy)[:,None]*field[:,y1,x0].T+(dx*dy)[:,None]*field[:,y1,x1].T

def correspondences(flow):
 h,w=flow.shape[2:];p=grid(h,w);f=flow[0,:,p[:,1].astype(int),p[:,0].astype(int)];q=p+f
 good=np.isfinite(q).all(1)&(q[:,0]>=0)&(q[:,0]<=w-1)&(q[:,1]>=0)&(q[:,1]<=h-1);ids=np.flatnonzero(good);cycle=np.linalg.norm(f[ids]+sample(flow[1],q[ids]),axis=1);selected=ids[cycle<=1]
 return p[selected],q[selected],{'grid_inside':len(ids),'cycle_accepted':len(selected)}
def infer(model,a,b):
 import torch
 tensors=[torch.from_numpy(x.copy()).float()[None,None].repeat(1,3,1,1).cuda()/255 for x in [a,b]]
 with torch.inference_mode():flow,cov=model.inference(torch.cat(tensors),torch.cat(tensors[::-1]))
 assert tuple(flow.shape)==tuple(cov.shape)==(2,2,*a.shape) and torch.isfinite(flow).all() and torch.isfinite(cov).all() and (cov>0).all()
 return flow.cpu().numpy()
def verify(pins):
 for path,h in pins.items():assert sha(ROOT/path)==h,path

def qualify():
 import torch
 from qualify_native_frontend import load_model,deterministic
 deterministic();torch.set_num_threads(1);cv2.setNumThreads(1);pre=read(OUT/'input-freeze.json');verify(pre['pins'])
 # Border interpolation and known translation qualify generalized native-size sampling.
 f=np.zeros((2,2,480,640),np.float32);f[0,0]=2.25;f[0,1]=-.5;f[1,0]=-2.25;f[1,1]=.5;p,q,e=correspondences(f);assert len(p)==4096;np.testing.assert_allclose(q-p,np.tile([2.25,-.5],(4096,1)))
 np.testing.assert_allclose(sample(np.ones((2,480,640)),np.array([[639.,479.]])),[[1.,1.]])
 # Known pinhole distortion is inverted without changing camera coordinates.
 cams,T,c=calibrations();np.testing.assert_allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-12)
 rng=np.random.default_rng(42);x=rng.uniform([-.5,-.5,3],[.5,.5,6],(100,3));k=np.array(cams['rgb']['K']);d=np.array(cams['rgb']['D']);uv,_=cv2.projectPoints(x,np.zeros(3),np.zeros(3),k,d);r=cv2.undistortPoints(uv,k,d,P=k).reshape(-1,2);p=x@k.T;np.testing.assert_allclose(r,p[:,:2]/p[:,2:],atol=1e-5)
 reports=[];refs=read(OUT/'references.json');weights={arm:ROOT/('models/macslam/MACVO_FrontendCov.pth' if arm=='rgb' else 'models/macslam/MACT_Offroad.pth') for arm in ['rgb','thermal']}
 for arm in ['rgb','thermal']:
  model,loading=load_model(weights[arm]);states={k:hashlib.sha256(v.detach().cpu().numpy().tobytes()).hexdigest() for k,v in model.state_dict().items()}
  row=next(x for x in refs if x['arm']==arm and x['qualified']);a,b=[np.load(ROOT/row[key],allow_pickle=False) for key in ['source','target']]
  outputs=[];seconds=[]
  for repeat in range(2):
   torch.cuda.synchronize();started=time.perf_counter();flow=infer(model,a,b);torch.cuda.synchronize();seconds.append(time.perf_counter()-started);outputs.append(flow)
  assert np.array_equal(outputs[0],outputs[1]);p,q,info=correspondences(outputs[0]);assert p.shape==q.shape and p.shape[1]==2
  assert states=={k:hashlib.sha256(v.detach().cpu().numpy().tobytes()).hexdigest() for k,v in model.state_dict().items()}
  reports.append({'arm':arm,'loading':loading,'seconds':seconds,'flow_sha256':hashlib.sha256(flow.tobytes()).hexdigest(),'shape':list(flow.shape),**info})
  del model,flow,outputs;torch.cuda.empty_cache()
 files={Path(__file__).resolve(),ROOT/'tools/qualify_native_frontend.py',*weights.values()}
 for folder in ['FlowFormer','FlowFormerCov']:files.update((ROOT/'MAC-VO-main/Module/Network'/folder).rglob('*.py'))
 write(OUT/'qualification.json',{'reports':reports,'opencv':cv2.__version__,'numpy':np.__version__,'torch':torch.__version__,'gpu':torch.cuda.get_device_name(0),'native_size_repeat_bitwise':True,'synthetic_controls':'native flow/cycle, border interpolation, distortion inverse, supplied transform SO3','pose_predictions':0})
 files.update([OUT/'qualification.json',OUT/'common-reference-indices.json']);pins=dict(pre['pins']);pins.update({str(p.relative_to(ROOT)):sha(p) for p in files});verify(pins)
 write(OUT/'execution-freeze.json',{'pins':pins,'runtime_image':'sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09','scope':'NGCLAB all-three-lighting declared half-second intervals; no scoring during inference'})
 print('Qualified and frozen',json.dumps(reports),flush=True)

def run():
 import torch
 from qualify_native_frontend import load_model,deterministic
 deterministic();torch.set_num_threads(1);cv2.setNumThreads(1);freeze=read(OUT/'execution-freeze.json');verify(freeze['pins'])
 # Scoring-only reference arrays never passed to a matcher or solver.
 refs=read(OUT/'references.json');inputs=[dict({k:r[k] for k in ['condition','arm','index','source','target','K','D']}, qualified=all(reason=='reference_bracket' for reason in r['unsupported_reasons'])) for r in refs]
 started=time.monotonic();n=0
 with (OUT/'predictions.jsonl').open('x') as output:
  for frontend in ['sift','learned']:
   for arm in ['rgb','thermal']:
    model=None;feature_cache={}
    if frontend=='learned':
     model,loading=load_model(ROOT/('models/macslam/MACVO_FrontendCov.pth' if arm=='rgb' else 'models/macslam/MACT_Offroad.pth'));state={k:hashlib.sha256(v.detach().cpu().numpy().tobytes()).hexdigest() for k,v in model.state_dict().items()}
    for row in [x for x in inputs if x['arm']==arm]:
     start=time.perf_counter();record={k:row[k] for k in ['condition','arm','index']};record['frontend']=frontend
     if not row['qualified']:record['status']='input_unsupported'
     else:
      a,b=[np.load(ROOT/row[key],allow_pickle=False) for key in ['source','target']]
      if frontend=='learned':p,q,info=correspondences(infer(model,a,b));record.update(info)
      else:
       features=[]
       for key,array in zip(['source','target'],[a,b]):
        path=row[key]
        if path not in feature_cache:
         kp,desc=geometry.sift().detectAndCompute(array,None);feature_cache[path]=(np.array([x.pt for x in kp],float).reshape(-1,2),desc)
        features.append(feature_cache[path])
       (kp0,d0),(kp1,d1)=features;matches=geometry.mutual_matches(d0,d1);p=np.array([kp0[i] for i,j in matches]).reshape(-1,2);q=np.array([kp1[j] for i,j in matches]).reshape(-1,2);record.update(source_keypoints=len(kp0),target_keypoints=len(kp1))
      k=np.array(row['K']);d=np.array(row['D'])
      if len(p):p=cv2.undistortPoints(p[:,None,:],k,d,P=k).reshape(-1,2);q=cv2.undistortPoints(q[:,None,:],k,d,P=k).reshape(-1,2)
      record.update(geometry.pose(p,q,k));record.update(points0=p.tolist(),points1=q.tolist())
     if model is not None:torch.cuda.synchronize()
     record['elapsed_seconds']=time.perf_counter()-start;output.write(json.dumps(record,allow_nan=False)+'\n');output.flush();n+=1
     if n%100==0:print('Predicted',n,'elapsed',round(time.monotonic()-started,1),flush=True)
    if model is not None:
     assert state=={k:hashlib.sha256(v.detach().cpu().numpy().tobytes()).hexdigest() for k,v in model.state_dict().items()};del model;torch.cuda.empty_cache()
 verify(freeze['pins']);write(OUT/'prediction-lock.json',{'rows':n,'seconds':time.monotonic()-started,'predictions_sha256':sha(OUT/'predictions.jsonl'),'execution_freeze_sha256':sha(OUT/'execution-freeze.json'),'scoring_started':False,'model_states_unchanged':True});print('Predictions locked',n,flush=True)

def score():
 verify(read(OUT/'execution-freeze.json')['pins']);lock=read(OUT/'prediction-lock.json');assert sha(OUT/'predictions.jsonl')==lock['predictions_sha256'];refs={(x['condition'],x['arm'],x['index']):x for x in read(OUT/'references.json')};scores=[]
 for line in (OUT/'predictions.jsonl').read_text().splitlines():
  row=json.loads(line);r=refs[(row['condition'],row['arm'],row['index'])];s={k:row[k] for k in ['condition','arm','index','frontend','status','elapsed_seconds']};s.update(rotation_error_deg=None,translation_error_deg=None,joint_success=False,rotation_success=False,reference_supported=r['qualified'],identity_success=False)
  if r['qualified']:
   truth=np.array(r['source_to_next_optical']);s['identity_success']=r['reference_rotation_deg']<=5;s['reference_rotation_deg']=r['reference_rotation_deg'];s['translation_eligible']=r['translation_eligible']
   if row['status']=='ok':
    s['rotation_error_deg']=float(np.degrees(Rotation.from_matrix(np.array(row['R'])@truth[:3,:3].T).magnitude()));s['rotation_success']=s['rotation_error_deg']<=5
    if r['translation_eligible']:
     t=np.array(row['t']);t/=np.linalg.norm(t);v=truth[:3,3]/np.linalg.norm(truth[:3,3]);s['translation_error_deg']=float(np.degrees(np.arccos(np.clip(t@v,-1,1))));s['joint_success']=bool(s['rotation_success'] and s['translation_error_deg']<=15)
  scores.append(s)
 assert len(scores)==lock['rows']==len(refs)*2
 def aggregate(group):
  supported=[x for x in group if x['status']=='ok'];n=len(group);r=[x['rotation_error_deg'] for x in supported if x['rotation_error_deg'] is not None];t=[x['translation_error_deg'] for x in supported if x['translation_error_deg'] is not None]
  failures=[not x['joint_success'] for x in group];longest=current=0;recovery=[]
  previous_index=None
  for record,fail in zip(group,failures):
   if previous_index is not None and record['index']!=previous_index+1:current=0
   previous_index=record['index']
   if fail:current+=1;longest=max(longest,current)
   elif current:recovery.append(current*.5);current=0
  return {'intervals':n,'reference_supported':sum(x['reference_supported'] for x in group),'translation_eligible':sum(x.get('translation_eligible',False) for x in group),'solver_supported':len(supported),'rotation_success':sum(x['rotation_success'] for x in group),'joint_success':sum(x['joint_success'] for x in group),'joint_rate':sum(x['joint_success'] for x in group)/n,'identity_rotation_rate':sum(x['identity_success'] for x in group)/n,'support_rate':len(supported)/n,'conditional_rotation_median_deg':float(np.median(r)) if r else None,'conditional_rotation_p90_deg':float(np.percentile(r,90)) if r else None,'conditional_translation_median_deg':float(np.median(t)) if t else None,'conditional_translation_p90_deg':float(np.percentile(t,90)) if t else None,'longest_joint_failure_run_intervals':longest,'completed_failure_runs_seconds':recovery,'right_censored_failure_run_intervals':current,'median_elapsed_seconds':float(np.median([x['elapsed_seconds'] for x in group]))}
 summary={}
 for condition in CONDITIONS:
  summary[condition]={}
  for frontend in ['sift','learned']:
   summary[condition][frontend]={arm:aggregate(sorted([x for x in scores if x['condition']==condition and x['frontend']==frontend and x['arm']==arm],key=lambda x:x['index'])) for arm in ['rgb','thermal']}
 common={}
 for condition in CONDITIONS:
  eligible={i for c,a,i in refs if c==condition and all(refs[(condition,arm,i)]['qualified'] and refs[(condition,arm,i)].get('translation_eligible',False) for arm in ['rgb','thermal'])}
  common[condition]={frontend:{arm:aggregate(sorted([x for x in scores if x['condition']==condition and x['frontend']==frontend and x['arm']==arm and x['index'] in eligible],key=lambda x:x['index'])) for arm in ['rgb','thermal']} for frontend in ['sift','learned']}
 write(OUT/'common-reference-summary.json',common)
 write(OUT/'scores.json',scores);write(OUT/'summary.json',summary);print(json.dumps({'all_declared':summary,'common_reference':common},indent=2))

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('stage',choices=['prepare','qualify','run','score']);a=p.parse_args();{'prepare':prepare,'qualify':qualify,'run':run,'score':score}[a.stage]()
