"""Frozen fixed-precision RGB/thermal validation on saved600group captures."""
from pathlib import Path
import hashlib,json,time
import numpy as np
from scipy.spatial.transform import Rotation
import nominal_joint_motion as joint
from selected_motion_trajectory import rigid
ROOT=Path(__file__).resolve().parents[1];REC=ROOT/'results/longer-fixed-thermal/recovery';OUT=ROOT/'results/longer-fixed-thermal/run-001';METHODS=['rgb','thermal','fixed_point1','full_joint']
def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,d):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(d,indent=2,allow_nan=False)+'\n')
def capture():
 OUT.mkdir(parents=True,exist_ok=False);receipt=[];started=time.perf_counter()
 for c in ['day','night','rain']:
  source=read(REC/'inputs'/c/'source-manifest.json');cal=read(REC/'inputs'/c/'calibration.json');E=joint.geometry(cal);native={};events={}
  for cam,sensor in [('rgb','rgb'),('thermal','thr')]:
   cap=REC/'captures'/c/cam
   with np.load(cap/'tensor-map.npz') as z:native[cam]={k:z[k] for k in z.files}
   graph=native[cam];assert len(graph['frames//time_ns'])>=600
   assert graph['frames//time_ns'].reshape(-1)[:600].tolist()==[r['cameras'][sensor]['left']['recorded_time_ns'] for r in source['records'][:600]]
   for i,r in enumerate(source['records'][:600]):assert np.array_equal(graph['frames//K'][i],np.asarray(r['cameras'][sensor]['K'],dtype=graph['frames//K'].dtype))
   events[cam]={}
   for line in (cap/'arbiter-events/events.jsonl').read_text().splitlines():
    e=json.loads(line)
    if e['kind']=='motion_result_written' and e['first_write_for_submission']:
     key=(e['from_idx'],e['frame_idx']);assert key not in events[cam];events[cam][key]=e
  rows=[]
  for i in range(599):
   row={'pair':i,'source_ids':[f'{i:06}',f'{i+1:06}'],'binding':{},'methods':{}};packets={}
   for cam,sensor in [('rgb','rgb'),('thermal','thr')]:
    try:
     p,meta=joint.bind_camera(native[cam],events[cam].get((i,i+1)),i,source['records'],sensor,E[cam]);row['binding'][cam]={'valid':True,**meta};packets[cam]=p
    except ValueError as e:row['binding'][cam]={'valid':False,'reason':str(e)}
   for method in METHODS:
    cameras=[method] if method in ['rgb','thermal'] else [cam for cam in ['rgb','thermal'] if cam in packets]
    if not cameras or any(cam not in packets for cam in cameras):row['methods'][method]={'status':'UNAVAILABLE_INPUT','transform':None};continue
    pp=[]
    for cam in cameras:
     p={k:v.copy() for k,v in packets[cam].items()}
     if method=='fixed_point1' and cam=='thermal':p['covariance']/=.1
     pp.append(p)
    try:result=joint.solve(pp,'cov',joint.CONTRACT);row['methods'][method]={**result,'cameras':cameras}
    except (ValueError,np.linalg.LinAlgError) as e:row['methods'][method]={'status':'FIT_FAILED','reason':str(e),'transform':None,'cameras':cameras}
   rows.append(row)
  p=OUT/f'predictions-{c}.json';write(p,{'sequence':source['sequence'],'rows':rows,'references_read':False});receipt.append({'condition':c,'sequence':source['sequence'],'prediction_sha256':sha(p),'source_manifest_sha256':sha(REC/'inputs'/c/'source-manifest.json'),'captured_groups':600,'source_groups':len(source['records'])});print('CAPTURED',c,{m:sum(r['methods'][m]['transform'] is not None for r in rows) for m in METHODS},flush=True)
 write(OUT/'prediction-lock.json',{'rows':receipt,'source_sha256':sha(__file__),'geometry_sha256':sha(joint.__file__),'capture_seconds':time.perf_counter()-started,'gate':.1,'references_read':False})
def path_stats(rows,refs,method,start,end):
 selected=rows[start:end]
 missing=[r['pair'] for r in selected if r['methods'][method]['transform'] is None]
 if missing:return {'status':'INCOMPLETE_NO_GAP_FILL','missing_pairs':missing}
 P=np.eye(4);positions=[P.copy()];R0=refs[start];reference=[np.eye(4)];correction=0;Rs=np.eye(3);ts=np.zeros(3)
 for row in selected:
  D,change=rigid(row['methods'][method]['transform']);correction=max(correction,change);P=P@D;old=Rs.copy();ts=ts+old@D[:3,3];Rs=old@D[:3,:3];assert np.allclose(ts,P[:3,3],rtol=0,atol=1e-10);positions.append(P.copy());reference.append(np.linalg.solve(R0,refs[row['pair']+1]))
 P=np.array(positions);reference=np.array(reference);e=np.linalg.norm(P[:,:3,3]-reference[:,:3,3],axis=1);ang=np.rad2deg(Rotation.from_matrix(np.transpose(reference[:,:3,:3],(0,2,1))@P[:,:3,:3]).magnitude());scalar=sum(sum(float(P[i,j,3]-reference[i,j,3])**2 for j in range(3)) for i in range(len(P)))/len(P);assert abs(scalar-np.mean(e*e))<1e-8
 name=f'path-{method}-{start:03}-{end:03}.npz';return {'status':'COMPLETE','pose_count':len(P),'position_rmse_m':float(np.sqrt(np.mean(e*e))),'position_mean_m':float(np.mean(e)),'final_position_error_m':float(e[-1]),'orientation_rmse_deg':float(np.sqrt(np.mean(ang*ang))),'final_orientation_error_deg':float(ang[-1]),'max_roundoff_correction':correction,'positions':P,'reference':reference,'filename':name}
def score():
 lock=read(OUT/'prediction-lock.json');summaries={}
 for entry in lock['rows']:
  c=entry['condition'];p=OUT/f'predictions-{c}.json';assert sha(p)==entry['prediction_sha256'];rows=read(p)['rows'];refs=[]
  for i in range(600):
   T,_=rigid(np.loadtxt(REC/'references'/c/f'{i:06}.txt').reshape(4,4));refs.append(T)
  result={'sequence':entry['sequence'],'expected_intervals':599,'methods':{}}
  for method in METHODS:
   errors=[]
   for row in rows:
    value=row['methods'][method]['transform']
    if value is None:continue
    i=row['pair'];reference=np.linalg.solve(refs[i],refs[i+1]);T=np.array(value);delta=np.linalg.solve(reference,T);errors.append({'pair':i,'translation_m':float(np.linalg.norm(delta[:3,3])),'rotation_deg':float(np.rad2deg(Rotation.from_matrix(delta[:3,:3]).magnitude()))})
   def stats(start,end):
    value=path_stats(rows,refs,method,start,end)
    if value['status']=='COMPLETE':
     folder=OUT/'paths'/c;folder.mkdir(parents=True,exist_ok=True);np.savez_compressed(folder/value['filename'],positions=value.pop('positions'),reference=value.pop('reference'))
    return value
   result['methods'][method]={'available':len(errors),'missing':[r['pair'] for r in rows if r['methods'][method]['transform'] is None],'increment_errors':errors,'full_path':stats(0,599),'blocks':[{'start':start,'end':start+99,**stats(start,start+99)} for start in range(0,600,100)]}
  common=[row for row in rows if all(row['methods'][m]['transform'] is not None for m in ['rgb','fixed_point1'])]
  result['rgb_fixed_common_intervals']=len(common);result['duration_rgb_seconds']=(read(REC/'inputs'/c/'source-manifest.json')['records'][599]['cameras']['rgb']['left']['recorded_time_ns']-read(REC/'inputs'/c/'source-manifest.json')['records'][0]['cameras']['rgb']['left']['recorded_time_ns'])/1e9
  summaries[c]=result;print('SCORED',c,flush=True)
 write(OUT/'scores.json',{'prediction_lock_sha256':sha(OUT/'prediction-lock.json'),'summary':summaries,'scope':'Already exposed600groups,nominalsameIDprovidedreference,fullpathsrefusedonmissingincrements,noalignmentfit.'})
if __name__=='__main__':
 import sys
 {'capture':capture,'score':score}[sys.argv[1]]()
