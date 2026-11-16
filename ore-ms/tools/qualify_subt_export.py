"""Whole-export qualification without matching; generic SubT sequence config.

Frozen smoke/lighting runners remain unchanged. This adapter adds full export
intake, nominal frame/time/reference/gyro gates for another platform, rather than
copying an inference runner with changed paths. Reuses frozen PNG and grid helpers.
"""
import argparse,csv,io,json,hashlib,zipfile,threading,concurrent.futures
from pathlib import Path
import cv2,numpy as np,yaml
from scipy.spatial.transform import Rotation,Slerp
from smokeseer_preflight import png_check
from deployment_lighting import grid,nearest,sha,write,read
ROOT=Path(__file__).resolve().parents[1]

def run(config):
 cfg=read(config);out=ROOT/cfg['output'];data=ROOT/cfg['gray_output'];out.mkdir(exist_ok=True);cv2.setNumThreads(1)
 pins={p:sha(ROOT/p) for p in [cfg['archive'],cfg['reference_archive'],cfg['intrinsics'],cfg['extrinsics'],cfg['protocol'],str(Path(__file__).relative_to(ROOT)),'tools/deployment_lighting.py','tools/smokeseer_preflight.py',str(Path(config).resolve().relative_to(ROOT))]}
 write(out/'input-freeze.json',{'pins':pins,'new_inference_calls':0,'scope':'Nominal export/reference qualification; no raw image-header binding'})
 archive=ROOT/cfg['archive'];intr=yaml.safe_load((ROOT/cfg['intrinsics']).read_text());extr=yaml.safe_load((ROOT/cfg['extrinsics']).read_text());cal={};projection=[]
 for arm in ['rgb','thermal']:
  c=intr[arm+'_camera'];p=c['projection_parameters'];d=np.array([c['distortion_parameters'][x] for x in ['k1','k2','p1','p2']]);k=np.array([[p.get('fx',p.get('gamma1')),0,p.get('cx',p.get('u0'))],[0,p.get('fy',p.get('gamma2')),p.get('cy',p.get('v0'))],[0,0,1.]])
  xi=c.get('mirror_parameters',{}).get('xi',0);assert c['model_type']==('MEI' if arm=='rgb' else 'PINHOLE')
  points=grid(c['image_height'],c['image_width']);xy=cv2.undistortPoints(points[:,None,:].astype(float),k,d,criteria=(cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,100,1e-12)).reshape(-1,2);r2=(xy*xy).sum(1);disc=1+(1-xi*xi)*r2;assert (disc>0).all();lam=(xi+np.sqrt(disc))/(1+r2);rays=np.column_stack([lam[:,None]*xy,lam-xi]);assert (rays[:,2]>0).all();uv=rays[:,:2]/(rays[:,2]+xi*np.linalg.norm(rays,axis=1))[:,None];projected=cv2.projectPoints(np.column_stack([uv,np.ones(len(uv))]),np.zeros(3),np.zeros(3),k,d)[0].reshape(-1,2);error=np.linalg.norm(projected-points,axis=1);assert error.max()<.001
  projection.append({'arm':arm,'points':len(points),'max_roundtrip_error_px':float(error.max())});a=np.array(extr[arm+'_camera_to_imu']['data']).reshape(4,4);u,_,vt=np.linalg.svd(a[:3,:3]);r=u@vt;correction=np.linalg.norm(r-a[:3,:3]);assert correction<1e-5 and np.linalg.det(r)>0;t=np.eye(4);t[:3,:3]=r;t[:3,3]=a[:3,3];cal[arm]={'source_matrix':a.tolist(),'effective_camera_to_imu':t.tolist(),'roundoff_correction':float(correction),'source_last_row':a[3].tolist()}
 write(out/'calibration.json',cal);write(out/'projection.json',projection)
 prefix=cfg['prefix'];times={};members=[]
 with zipfile.ZipFile(archive) as z:
  for arm,folder in [('rgb','cam_0'),('thermal','thermal')]:
   stamps=[int(x) for x in z.read(prefix+'/'+folder+'/timestamps.txt').decode().splitlines()];assert np.all(np.diff(stamps)>0);times[arm]=stamps;names=[n for n in z.namelist() if n.startswith(prefix+'/'+folder+'/') and n.endswith('.png')];assert sorted(int(Path(n).stem) for n in names)==list(range(len(stamps)));members.extend((arm,n) for n in sorted(names,key=lambda n:int(Path(n).stem)))
  imu_member=next(n for n in z.namelist() if n.endswith('/imu_data.csv'));imu_bytes=z.read(imu_member)
 local=threading.local()
 def decode(job):
  arm,name=job
  if not hasattr(local,'z'):local.z=zipfile.ZipFile(archive)
  b=local.z.read(name);png_check(b);a=cv2.imdecode(np.frombuffer(b,np.uint8),cv2.IMREAD_UNCHANGED);assert a is not None;c=intr[arm+'_camera'];expected=(c['image_height'],c['image_width']);assert a.shape==(expected+(3,) if arm=='rgb' else expected);assert a.dtype==(np.uint8 if arm=='rgb' else np.uint16);lo,hi=int(a.min()),int(a.max());g=cv2.cvtColor(a,cv2.COLOR_BGR2GRAY) if arm=='rgb' else np.zeros(a.shape,np.uint8) if lo==hi else np.rint((a.astype(float)-lo)*255/(hi-lo)).clip(0,255).astype(np.uint8);i=int(Path(name).stem);p=data/arm/f'{i:05}.npy';p.parent.mkdir(parents=True,exist_ok=True);np.save(p,g,allow_pickle=False);q=np.percentile(g,[5,95]);return {'arm':arm,'index':i,'time_ns':times[arm][i],'member':name,'member_sha256':hashlib.sha256(b).hexdigest(),'path':str(p.relative_to(ROOT)),'sha256':sha(p),'shape':list(a.shape),'dtype':str(a.dtype),'metrics':{'mean':float(g.mean()),'std':float(g.std()),'p05_p95_range':float(q[1]-q[0]),'raw_min':lo,'raw_max':hi}}
 frames=[]
 with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
  for f in pool.map(decode,members):
   frames.append(f)
   if len(frames)%3000==0:print('Decoded',len(frames),'/',len(members),flush=True)
 write(out/'images.json',frames)
 def parse(b):return [{k.strip():v.strip() for k,v in row.items()} for row in csv.DictReader(io.StringIO(b.decode()))]
 imu=parse(imu_bytes)
 with zipfile.ZipFile(ROOT/cfg['reference_archive']) as z:
  ref=parse(z.read(next(n for n in z.namelist() if n.endswith('odometry_data.csv'))));other=parse(z.read(next(n for n in z.namelist() if n.endswith('imu_data.csv'))))
 def imu_map(rows):
  t=np.array([int(r['timestamp']) for r in rows],np.int64);assert np.all(np.diff(t)>0);v=np.array([[float(r[k]) for k in ['q_x','q_y','q_z','q_w','ang_vel_x','ang_vel_y','ang_vel_z','lin_acc_x','lin_acc_y','lin_acc_z']] for r in rows]);assert np.isfinite(v).all() and np.max(abs(np.linalg.norm(v[:,:4],axis=1)-1))<1e-3;return t,v
 it,iv=imu_map(imu);ot,ov=imu_map(other);ii={int(t):j for j,t in enumerate(it)};oi={int(t):j for j,t in enumerate(ot)};shared=sorted(set(ii)&set(oi));different=[t for t in shared if not np.array_equal(iv[ii[t]],ov[oi[t]])];binding={'export_rows':len(it),'reference_rows':len(ot),'shared':len(shared),'different_numeric_rows':different,'export_only_times':sorted(set(ii)-set(oi)),'reference_only_times':sorted(set(oi)-set(ii))};write(out/'imu-binding.json',binding)
 rt=np.array([int(r['timestamp']) for r in ref],np.int64);pos=np.array([[float(r[k]) for k in ['x','y','z']] for r in ref]);q=np.array([[float(r[k]) for k in ['q_x','q_y','q_z','q_w']] for r in ref]);assert np.all(np.diff(rt)>0) and np.isfinite(pos).all() and np.isfinite(q).all() and np.max(abs(np.linalg.norm(q,axis=1)-1))<1e-3;origin=int(rt[0]);slerp=Slerp((rt-origin)*1e-9,Rotation.from_quat(q));np.savez_compressed(out/'reference.npz',time_ns=rt,position=pos,quaternion_xyzw=q);np.savez_compressed(out/'export-imu.npz',time_ns=it,values=iv)
 def pose(t):
  j=int(np.searchsorted(rt,t))
  if t<int(rt[0]) or j==len(rt):return None
  if rt[j]==t:r=Rotation.from_quat(q[j]).as_matrix();p=pos[j]
  else:
   if rt[j]-rt[j-1]>50_000_000:return None
   a=(t-int(rt[j-1]))/int(rt[j]-rt[j-1]);p=(1-a)*pos[j-1]+a*pos[j];r=slerp([(t-origin)*1e-9]).as_matrix()[0]
  v=np.eye(4);v[:3,:3]=r;v[:3,3]=p;return v
 lookup={(f['arm'],f['index']):f for f in frames};anchors=list(range(times['rgb'][0],times['rgb'][-1]-500_000_000+1,500_000_000));intervals=[]
 for i,a in enumerate(anchors):
  for arm in ['rgb','thermal']:
   ids=[nearest(times[arm],t) for t in [a,a+500_000_000]];fs=[lookup[arm,j] for j in ids];ts=[f['time_ns'] for f in fs];errors=[t-v for t,v in zip(ts,[a,a+500_000_000])];ps=[pose(t) for t in ts];v={'arm':arm,'index':i,'anchor_ns':a,'times_ns':ts,'source':fs[0]['path'],'target':fs[1]['path'],'anchor_errors_ns':errors,'qualified':max(abs(e) for e in errors)<=25_000_000 and ids[1]>ids[0],'reference_supported':all(p is not None for p in ps)}
   if v['reference_supported']:
    t=np.array(cal[arm]['effective_camera_to_imu']);p0,p1=[p@t for p in ps];rel=np.linalg.solve(p1,p0);v.update(target_from_source=rel.tolist(),translation_m=float(np.linalg.norm(rel[:3,3])),reference_rotation_deg=float(np.degrees(Rotation.from_matrix(rel[:3,:3]).magnitude())))
   intervals.append(v)
 write(out/'intervals.json',intervals);by={(r['arm'],r['index']):r for r in intervals};common=[i for i in range(len(anchors)) if all(by[a,i]['reference_supported'] for a in ['rgb','thermal'])];moving=[i for i in common if all(by[a,i]['reference_rotation_deg']>=2 for a in ['rgb','thermal'])];translation=[i for i in common if all(by[a,i]['translation_m']>=.01 for a in ['rgb','thermal'])];write(out/'population.json',{'common_reference':common,'paired_rotation_ge2deg':moving,'translation_ge1cm_both':translation})
 gyro=[]
 for i,t0 in enumerate(range(origin,int(rt[-1])-500_000_000+1,500_000_000)):
  t1=t0+500_000_000;j0=int(np.searchsorted(it,t0,side='right')-1);j1=int(np.searchsorted(it,t1,side='left'));v={'index':i,'t0_ns':t0,'t1_ns':t1}
  if j0<0 or j1>=len(it):v['supported']=False;gyro.append(v);continue
  gaps=np.diff(it[j0:j1+1]);v['max_imu_gap_ns']=int(gaps.max());v['supported']=bool(gaps.max()<=20_000_000)
  if not v['supported']:gyro.append(v);continue
  boundaries=[t0]+[int(t) for t in it[j0+1:j1] if t0<int(t)<t1]+[t1];r=np.eye(3)
  for a,b in zip(boundaries,boundaries[1:]):
   j=int(np.searchsorted(it,a,side='right')-1);r=r@Rotation.from_rotvec(iv[j,4:7]*((b-a)*1e-9)).as_matrix()
  rr=slerp((np.array([t0-origin,t1-origin]))*1e-9).as_matrix();truth=rr[0].T@rr[1];v.update(reference_rotation_deg=float(np.degrees(Rotation.from_matrix(truth).magnitude())),gyro_error_deg=float(np.degrees(Rotation.from_matrix(r.T@truth).magnitude())),reference_rotvec=Rotation.from_matrix(truth).as_rotvec().tolist());gyro.append(v)
 write(out/'gyro-comparison.json',gyro);gm=[g for g in gyro if g['supported'] and g['reference_rotation_deg']>=2];err=[g['gyro_error_deg'] for g in gm];med=float(np.median(err)) if err else None;p95=float(np.percentile(err,95)) if err else None;consistent=len(gm)>=20 and med<=.2 and p95<=.5
 result={'decoded_images':len(frames),'images_by_arm':{a:len(times[a]) for a in times},'anchors':len(anchors),'input_qualified':sum(r['qualified'] for r in intervals),'unsupported_inputs':[{k:r[k] for k in ['arm','index','anchor_errors_ns']} for r in intervals if not r['qualified']],'reference_poses':len(rt),'reference_max_gap_ns':int(np.diff(rt).max()),'common_reference_anchors':len(common),'paired_moving_anchors':len(moving),'translation_anchors':len(translation),'gyro_intervals':len(gyro),'gyro_unsupported':sum(not g['supported'] for g in gyro),'gyro_moving_intervals':len(gm),'gyro_moving_median_deg':med,'gyro_moving_p95_deg':p95,'observed_axes_time_consistency_pass':bool(consistent),'reference_rotvec_rms':np.sqrt(np.mean(np.array([g['reference_rotvec'] for g in gm])**2,axis=0)).tolist() if gm else None,'export_imu_equal_all_shared':not different,'new_inference_calls':0,'physical_image_header_binding':False,'independent_reference_truth':False}
 write(out/'qualification.json',result)
 for p,h in pins.items():assert sha(ROOT/p)==h,p
 write(out/'completion.json',{'pins':{str(p.relative_to(ROOT)):sha(p) for p in out.iterdir() if p.is_file() and p.name!='completion.json' and p.suffix not in ['.log']},'input_freeze_sha256':sha(out/'input-freeze.json'),'new_inference_calls':0});print(json.dumps(result,indent=2),flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('config',type=Path);args=p.parse_args();run(args.config)
