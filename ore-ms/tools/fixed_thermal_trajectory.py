"""Compose locked nominal same-ID increments; no fitted trajectory alignment."""
from pathlib import Path
import hashlib,json,math
import numpy as np
from scipy.spatial.transform import Rotation
from selected_motion_trajectory import rigid
ROOT=Path(__file__).resolve().parents[1];PREV=ROOT/'results/local-relationship-learning/run-001';REC=ROOT/'results/thermal-incremental-comparison/recovery';OUT=ROOT/'results/fixed-thermal-trajectory/run-001'
def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,x):p.write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
def compose(values):
 out=[np.eye(4)];Rs=np.eye(3);ts=np.zeros(3);correction=0
 for value in values:
  if value is None:raise ValueError('MISSING_INCREMENT_NO_FILL')
  T,change=rigid(value);correction=max(correction,change);P=out[-1]@T
  # Independent component composition instead of homogeneous multiplication.
  oldR=Rs.copy();ts=ts+oldR@T[:3,3];Rs=oldR@T[:3,:3];assert np.allclose(P[:3,:3],Rs,rtol=0,atol=1e-12) and np.allclose(P[:3,3],ts,rtol=0,atol=1e-12)
  out.append(P)
 return np.stack(out),correction

def capture():
 OUT.mkdir(parents=True,exist_ok=False);lock=read(PREV/'global-prediction-lock.json');assert all(sha(PREV/k)==v for k,v in lock['files'].items());rows=[]
 # Noncommuting fixture establishes composition order and lever-arm behavior.
 A=np.eye(4);A[:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix();A[:3,3]=[1,0,0];B=np.eye(4);B[:3,3]=[1,0,0];path,_=compose([A,B]);assert np.allclose(path[-1,:3,3],[1,1,0],atol=1e-12);assert not np.allclose(A@B,B@A)
 for c in ['day','night','rain']:
  bind=read(REC/f'binding-{c}.json');scores=read(REC/f'scores-{c}.json')['rows'];prior=read(REC/f'predictions-{c}.json')['rows'];assert [r['pair'] for r in scores]==list(range(99));assert all(r['source_ids']==[f'{i:06}',f'{i+1:06}'] for i,r in enumerate(bind['rows']))
  poses={'reference':[r['reference_delta'] for r in scores],'fixed_point1':[r['transform'] for r in read(PREV/f'fixed-point1-{c}.json')['rows']],'learned_all':[r['transform'] for r in read(PREV/c/'all/predictions.json')['rows']]}
  for mode in ['rgb_cov','joint_cov','native_rgb']:poses[mode]=[r['methods'][mode]['transform'] for r in prior]
  for mode,values in poses.items():
   path,corr=compose(values);p=OUT/f'{c}-{mode}.npz';np.savez_compressed(p,poses=path);rows.append({'condition':c,'method':mode,'poses':len(path),'path':p.name,'sha256':sha(p),'maximum_increment_roundoff_correction':corr})
 write(OUT/'trajectory-lock.json',{'rows':rows,'source_sha256':sha(__file__),'parent_prediction_lock_sha256':sha(PREV/'global-prediction-lock.json'),'alignment':'Identity common initial gauge only; no fitted SE3/Sim3/scale/time transform.','source_axis':'Original nominal same-ID100groups,99increments','new_training_or_inference':False});print('LOCKED',len(rows),'trajectories',flush=True)
def score():
 lock=read(OUT/'trajectory-lock.json');summary={};audit=[]
 for c in ['day','night','rain']:
  with np.load(OUT/f'{c}-reference.npz') as z:ref=z['poses']
  bind=read(REC/f'binding-{c}.json')['rows'];duration=(bind[-1]['actual_camera_times_ns']['rgb_left'][1]-bind[0]['actual_camera_times_ns']['rgb_left'][0])/1e9;length=float(np.sum(np.linalg.norm(np.diff(ref[:,:3,3],axis=0),axis=1)));v={}
  for row in [r for r in lock['rows'] if r['condition']==c and r['method']!='reference']:
   assert sha(OUT/row['path'])==row['sha256']
   with np.load(OUT/row['path']) as z:P=z['poses']
   d=P[:,:3,3]-ref[:,:3,3];err=np.linalg.norm(d,axis=1);rot=Rotation.from_matrix(np.transpose(ref[:,:3,:3],(0,2,1))@P[:,:3,:3]).magnitude()*180/np.pi;rmse=float(np.sqrt(np.mean(err*err)));scalar=math.sqrt(sum(sum(float(P[i,j,3]-ref[i,j,3])**2 for j in range(3)) for i in range(100))/100);assert abs(scalar-rmse)<1e-12
   v[row['method']]={'pose_count':100,'position_rmse_m':rmse,'position_mean_m':float(err.mean()),'position_p95_m':float(np.percentile(err,95)),'final_position_error_m':float(err[-1]),'orientation_rmse_deg':float(np.sqrt(np.mean(rot*rot))),'final_orientation_error_deg':float(rot[-1])}
  summary[c]={'duration_rgb_seconds':duration,'provided_reference_path_length_m':length,'methods':v}
 write(OUT/'scores.json',{'trajectory_lock_sha256':sha(OUT/'trajectory-lock.json'),'summary':summary,'scope':'Short previously exposed nominal trajectory, provided-reference increments, not independentGT/fullnativeVO/longrun generalization.'});print(json.dumps(summary,indent=2))
if __name__=='__main__':
 import sys
 {'capture':capture,'score':score}[sys.argv[1]]()
