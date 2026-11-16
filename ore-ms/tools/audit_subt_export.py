"""Independent full export, pose interpolation and gyro quaternion audit.
Does not import the qualification runner or its numerical helper functions.
"""
from pathlib import Path
import json,hashlib,zipfile,csv,io,concurrent.futures,threading
import numpy as np,cv2,yaml
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];checks=0

def check(v,label):
 global checks
 assert v,label
 checks+=1

def read(p):return json.loads(p.read_text())
def digest(p):
 with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def mul(a,b):return np.r_[a[3]*b[:3]+b[3]*a[:3]+np.cross(a[:3],b[:3]),a[3]*b[3]-a[:3]@b[:3]]
def inv(a):return a*np.array([-1,-1,-1,1])
def slerp(a,b,w):
 a=a/np.linalg.norm(a);b=b/np.linalg.norm(b);dot=a@b
 if dot<0:b=-b;dot=-dot
 if dot>.9995:q=(1-w)*a+w*b;return q/np.linalg.norm(q)
 theta=np.arccos(np.clip(dot,-1,1));return (np.sin((1-w)*theta)*a+np.sin(w*theta)*b)/np.sin(theta)
def angle(q):return float(np.degrees(2*np.arctan2(np.linalg.norm(q[:3]),abs(q[3]))))
def parse(b):return [{k.strip():v.strip() for k,v in r.items()} for r in csv.DictReader(io.StringIO(b.decode()))]
def audit_without_reference(out,report,images,intervals,ts,cal,extr,raw_imu):
 """Audit missing labels explicitly; never derive reference from export IMU."""
 check(report['reference_available'] is False,'reference unavailable')
 check(report['reference_poses']==0 and report['reference_max_gap_ns'] is None,'no reference count/gaps')
 check(report['export_imu_equal_all_shared'] is None,'no unperformed IMU comparison claim')
 check(report['observed_axes_time_consistency_pass'] is False,'no reference consistency certification')
 check(report['gyro_moving_median_deg'] is None and report['gyro_moving_p95_deg'] is None,'no gyro/reference errors')
 check(report['gyro_intervals']==report['gyro_moving_intervals']==0 and read(out/'gyro-comparison.json')==[],'no gyro reference comparisons')
 ref=np.load(out/'reference.npz');check(ref['time_ns'].shape==(0,) and ref['position'].shape==(0,3) and ref['quaternion_xyzw'].shape==(0,4),'empty reference arrays')
 stamps=np.array([int(r['timestamp']) for r in raw_imu],np.int64);values=np.array([[float(r[k]) for k in ['q_x','q_y','q_z','q_w','ang_vel_x','ang_vel_y','ang_vel_z','lin_acc_x','lin_acc_y','lin_acc_z']] for r in raw_imu])
 check(len(stamps)>=2 and np.all(np.diff(stamps)>0) and np.isfinite(values).all(),'valid export IMU')
 check(np.max(abs(np.linalg.norm(values[:,:4],axis=1)-1))<.001,'export quaternion norms')
 saved=np.load(out/'export-imu.npz');check(np.array_equal(saved['time_ns'],stamps) and np.array_equal(saved['values'],values),'export IMU archive binding')
 binding=read(out/'imu-binding.json');check(binding=={'export_rows':len(stamps),'reference_rows':0,'shared':0,'different_numeric_rows':[],'export_only_times':stamps.tolist(),'reference_only_times':[]},'missing reference-side IMU membership')
 for arm in ts:
  source=np.array(extr[arm+'_camera_to_imu']['data']).reshape(4,4);eff=np.array(cal[arm]['effective_camera_to_imu'])
  check(np.array_equal(source,np.array(cal[arm]['source_matrix'])),'source extrinsics')
  check(np.linalg.norm(eff[:3,:3]-source[:3,:3])<1e-5 and np.linalg.det(eff[:3,:3])>0 and np.allclose(eff[:3,:3].T@eff[:3,:3],np.eye(3),atol=1e-12),'effective SO3')
  check(np.array_equal(eff[:3,3],source[:3,3]) and np.array_equal(eff[3],[0,0,0,1]),'camera origin preserved')
 anchors=list(range(ts['rgb'][0],ts['rgb'][-1]-500_000_000+1,500_000_000));lookup={(r['arm'],r['index']):r for r in images};keys=set()
 for r in intervals:
  key=(r['arm'],r['index']);check(key not in keys,'unique interval');keys.add(key);arm=r['arm'];a=anchors[r['index']];ids=[]
  for target in [a,a+500_000_000]:
   j=int(np.searchsorted(ts[arm],target));ids.append(min([v for v in [j-1,j] if 0<=v<len(ts[arm])],key=lambda v:(abs(ts[arm][v]-target),v)))
  times=[ts[arm][j] for j in ids];errors=[v-t for v,t in zip(times,[a,a+500_000_000])]
  check(r['anchor_ns']==a and r['times_ns']==times and r['anchor_errors_ns']==errors and r['source']==lookup[arm,ids[0]]['path'] and r['target']==lookup[arm,ids[1]]['path'],'input endpoints')
  check(r['qualified']==(max(abs(e) for e in errors)<=25_000_000 and ids[1]>ids[0]),'input gate')
  check(r['reference_supported'] is False and not any(k in r for k in ['target_from_source','translation_m','reference_rotation_deg']),'no invented pose labels')
 check(keys=={(a,i) for a in ts for i in range(len(anchors))},'all sensor intervals')
 check(len(anchors)==report['anchors'] and sum(r['qualified'] for r in intervals)==report['input_qualified'],'whole-span summary')
 check(report['common_reference_anchors']==report['paired_moving_anchors']==report['translation_anchors']==0,'empty accuracy cohorts')
 check(read(out/'population.json')=={'common_reference':[],'paired_rotation_ge2deg':[],'translation_ge1cm_both':[]},'no reference cohorts')
 result={'status':'pass','checks':checks,'images_redecoded':len(images),'intervals':len(intervals),'reference_available':False,'new_inference_calls':0,'auditor_sha256':digest(Path(__file__)),'scope':'Independent full image/gray/timestamp/IMU/calibration/input binding and missing-reference audit; no pose accuracy certification'}
 (out/'audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
def main(config):
 cfg=read(config);out=ROOT/cfg['output'];freeze=read(out/'input-freeze.json');report=read(out/'qualification.json')
 for p,h in freeze['pins'].items():check(digest(ROOT/p)==h,'frozen '+p)
 completion=read(out/'completion.json')
 for p,h in completion['pins'].items():check(digest(ROOT/p)==h,'completion '+p)
 images=read(out/'images.json');intervals=read(out/'intervals.json');cal=read(out/'calibration.json');intr=yaml.safe_load((ROOT/cfg['intrinsics']).read_text());extr=yaml.safe_load((ROOT/cfg['extrinsics']).read_text());ts={};prefix=cfg['prefix'];archive=ROOT/cfg['archive']
 with zipfile.ZipFile(archive) as z:
  check(len(z.namelist())==len(set(z.namelist())),'unique ZIP members')
  for a,folder in [('rgb','cam_0'),('thermal','thermal')]:ts[a]=[int(x) for x in z.read(prefix+'/'+folder+'/timestamps.txt').splitlines()];check(all(x<y for x,y in zip(ts[a],ts[a][1:])),'strict stamps')
  raw_imu=parse(z.read(next(n for n in z.namelist() if n.endswith('/imu_data.csv'))))
 local=threading.local()
 def image_check(r):
  if not hasattr(local,'z'):local.z=zipfile.ZipFile(archive)
  b=local.z.read(r['member']);a=cv2.imdecode(np.frombuffer(b,np.uint8),cv2.IMREAD_UNCHANGED);g=np.load(ROOT/r['path'],allow_pickle=False);lo,hi=int(a.min()),int(a.max());expected=cv2.cvtColor(a,cv2.COLOR_BGR2GRAY) if r['arm']=='rgb' else np.zeros_like(a,dtype=np.uint8) if lo==hi else np.round((a.astype(np.float64)-lo)*(255./(hi-lo))).clip(0,255).astype(np.uint8)
  # Multiplication order can differ only at half-integer floating boundaries;
  # use exact same mathematical division order for byte-level export binding.
  if r['arm']=='thermal' and hi!=lo:expected=np.round((a.astype(np.float64)-lo)*255/(hi-lo)).clip(0,255).astype(np.uint8)
  assert hashlib.sha256(b).hexdigest()==r['member_sha256'] and digest(ROOT/r['path'])==r['sha256'] and np.array_equal(g,expected)
  assert ts[r['arm']][r['index']]==r['time_ns'] and list(a.shape)==r['shape'] and str(a.dtype)==r['dtype']
  assert lo==r['metrics']['raw_min'] and hi==r['metrics']['raw_max'] and abs(float(g.mean())-r['metrics']['mean'])<1e-10
  c=intr[r['arm']+'_camera'];assert a.shape[:2]==(c['image_height'],c['image_width']);return 5
 with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
  for n in pool.map(image_check,images):
   global checks
   checks+=n
 check(len(images)==sum(len(t) for t in ts.values())==report['decoded_images'],'complete decodes')
 if not cfg.get('reference_archive'):
  audit_without_reference(out,report,images,intervals,ts,cal,extr,raw_imu)
  return
 with zipfile.ZipFile(ROOT/cfg['reference_archive']) as z:
  refs=parse(z.read(next(n for n in z.namelist() if n.endswith('odometry_data.csv'))));other=parse(z.read(next(n for n in z.namelist() if n.endswith('imu_data.csv'))))
 def values(rows):return np.array([[float(r[k]) for k in ['q_x','q_y','q_z','q_w','ang_vel_x','ang_vel_y','ang_vel_z','lin_acc_x','lin_acc_y','lin_acc_z']] for r in rows])
 im={int(r['timestamp']):r for r in raw_imu};om={int(r['timestamp']):r for r in other};shared=sorted(im.keys()&om.keys());different=[t for t in shared if not np.array_equal(values([im[t]]),values([om[t]]))];binding={'export_rows':len(raw_imu),'reference_rows':len(other),'shared':len(shared),'different_numeric_rows':different,'export_only_times':sorted(im.keys()-om.keys()),'reference_only_times':sorted(om.keys()-im.keys())};check(binding==read(out/'imu-binding.json'),'exact IMU export/reference membership')
 rt=np.array([int(r['timestamp']) for r in refs],np.int64);q=np.array([[float(r[k]) for k in ['q_x','q_y','q_z','q_w']] for r in refs]);pos=np.array([[float(r[k]) for k in ['x','y','z']] for r in refs]);it=np.array([int(r['timestamp']) for r in raw_imu],np.int64);iv=values(raw_imu)
 check(np.all(np.diff(rt)>0) and np.all(np.diff(it)>0),'time monotonicity');check(np.isfinite(iv).all() and np.isfinite(q).all() and np.isfinite(pos).all(),'finite data');check(np.max(abs(np.linalg.norm(q,axis=1)-1))<.001,'reference quaternion norm')
 def interp(t):
  j=int(np.searchsorted(rt,t))
  if j==len(rt) or t<rt[0]:return None
  if rt[j]==t:qq=q[j]/np.linalg.norm(q[j]);pp=pos[j]
  else:
   if rt[j]-rt[j-1]>50_000_000:return None
   w=(t-int(rt[j-1]))/int(rt[j]-rt[j-1]);qq=slerp(q[j-1],q[j],w);pp=(1-w)*pos[j-1]+w*pos[j]
  p=np.eye(4);p[:3,:3]=Rotation.from_quat(qq).as_matrix();p[:3,3]=pp;return p,qq
 for arm in ['rgb','thermal']:
  source=np.array(extr[arm+'_camera_to_imu']['data']).reshape(4,4);eff=np.array(cal[arm]['effective_camera_to_imu']);check(np.array_equal(source,np.array(cal[arm]['source_matrix'])),'source calib');check(np.linalg.norm(eff[:3,:3]-source[:3,:3])<1e-5 and np.linalg.det(eff[:3,:3])>0 and np.allclose(eff[:3,:3].T@eff[:3,:3],np.eye(3),atol=1e-12),'calib SO3');check(np.array_equal(eff[:3,3],source[:3,3]) and np.array_equal(eff[3],[0,0,0,1]),'calib origin')
 anchors=list(range(ts['rgb'][0],ts['rgb'][-1]-500_000_000+1,500_000_000));lookup={(r['arm'],r['index']):r for r in images}
 for r in intervals:
  a=anchors[r['index']];arm=r['arm'];ids=[]
  for t in [a,a+500_000_000]:
   j=int(np.searchsorted(ts[arm],t));ids.append(min([v for v in [j-1,j] if 0<=v<len(ts[arm])],key=lambda v:(abs(ts[arm][v]-t),v)))
  times=[ts[arm][j] for j in ids];errors=[t-e for t,e in zip(times,[a,a+500_000_000])];check(r['times_ns']==times and r['anchor_errors_ns']==errors and r['source']==lookup[arm,ids[0]]['path'] and r['target']==lookup[arm,ids[1]]['path'],'pair binding');check(r['qualified']==(max(abs(e) for e in errors)<=25_000_000 and ids[1]>ids[0]),'input support');p=[interp(t) for t in times];valid=all(x is not None for x in p);check(r['reference_supported']==valid,'reference support')
  if valid:
   t=np.array(cal[arm]['effective_camera_to_imu']);p0,p1=[x[0]@t for x in p];relative=np.eye(4);relative[:3,:3]=p1[:3,:3].T@p0[:3,:3];relative[:3,3]=p1[:3,:3].T@(p0[:3,3]-p1[:3,3]);check(np.allclose(relative,r['target_from_source'],atol=1e-9),'camera relative pose');check(abs(np.linalg.norm(relative[:3,3])-r['translation_m'])<1e-9 and abs(angle(Rotation.from_matrix(relative[:3,:3]).as_quat())-r['reference_rotation_deg'])<1e-6,'reference strata labels')
 check(len(intervals)==2*len(anchors)==2*report['anchors'],'whole span')
 by={(r['arm'],r['index']):r for r in intervals};common=[i for i in range(len(anchors)) if all(by[a,i]['reference_supported'] for a in ts)];moving=[i for i in common if all(by[a,i]['reference_rotation_deg']>=2 for a in ts)];trans=[i for i in common if all(by[a,i]['translation_m']>=.01 for a in ts)];check(read(out/'population.json')=={'common_reference':common,'paired_rotation_ge2deg':moving,'translation_ge1cm_both':trans},'all cohorts')
 errors=[];gyro=read(out/'gyro-comparison.json')
 for g in gyro:
  t0,t1=g['t0_ns'],g['t1_ns'];left=int(np.searchsorted(it,t0,side='right')-1);right=int(np.searchsorted(it,t1));supported=left>=0 and right<len(it) and np.diff(it[left:right+1]).max()<=20_000_000;check(g['supported']==supported,'gyro support')
  if not supported:continue
  current=np.array([0.,0.,0.,1.]);cursor=t0;j=left
  while cursor<t1:
   end=min(t1,int(it[j+1]));v=iv[j,4:7]*((end-cursor)*1e-9);m=np.linalg.norm(v);dq=np.r_[v*(np.sin(m/2)/m if m else .5),np.cos(m/2)];current=mul(current,dq);cursor=end;j+=1
  a,b=interp(t0),interp(t1);check(a is not None and b is not None,'gyro reference gap support');truth=mul(inv(a[1]),b[1]);e=angle(mul(inv(current),truth));check(abs(e-g['gyro_error_deg'])<1e-6 and abs(angle(truth)-g['reference_rotation_deg'])<1e-6,'independent quaternion integration')
  if angle(truth)>=2:errors.append(e)
 med=float(np.median(errors));p95=float(np.percentile(errors,95));check(abs(med-report['gyro_moving_median_deg'])<1e-6 and abs(p95-report['gyro_moving_p95_deg'])<1e-6,'gyro summary');check(report['observed_axes_time_consistency_pass']==(len(errors)>=20 and med<=.2 and p95<=.5),'fixed consistency gate')
 result={'status':'pass','checks':checks,'images_redecoded':len(images),'intervals':len(intervals),'gyro_intervals':len(gyro),'new_inference_calls':0,'auditor_sha256':digest(Path(__file__)),'scope':'Independent byte/gray/stamp/IMU/frame/pose/cohort and quaternion gyro recomputation; no raw header or independent physical truth certification'};(out/'audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':
 import sys
 main(Path(sys.argv[1]))
