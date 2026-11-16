"""Independent quaternion/rigid-algebra audit of SmokeRoom supplied-reference scoring."""
import json,hashlib,math,csv,io,zipfile
from pathlib import Path
import numpy as np,yaml
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/smoke-reference-contract';PARENT=ROOT/'results/deployment-smoke-frontend';checks=0

def read(p):return json.loads(Path(p).read_text())
def check(v,label):
 global checks
 assert v,label
 checks+=1

def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def multiply(a,b):
 av,bv=np.asarray(a[:3]),np.asarray(b[:3]);return np.r_[a[3]*bv+b[3]*av+np.cross(av,bv),a[3]*b[3]-av@bv]
def conjugate(q):return np.r_[-np.array(q[:3]),q[3]]
def angle(q):q=np.array(q)/np.linalg.norm(q);return math.degrees(2*math.atan2(np.linalg.norm(q[:3]),abs(q[3])))
def matrix(q):
 x,y,z,w=np.array(q)/np.linalg.norm(q);return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def at(t,times,qs,positions):
 j=int(np.searchsorted(times,t))
 if j==len(times) or t<times[0]:return None
 if times[j]==t:return qs[j],positions[j]
 if j==0 or times[j]-times[j-1]>50_000_000:return None
 a=(t-int(times[j-1]))/int(times[j]-times[j-1]);q=qs[j-1]/np.linalg.norm(qs[j-1]);v=qs[j]/np.linalg.norm(qs[j]);dot=float(q@v)
 if dot<0:v=-v;dot=-dot
 theta=math.acos(np.clip(dot,-1,1));out=(1-a)*q+a*v if theta<1e-10 else (math.sin((1-a)*theta)*q+math.sin(a*theta)*v)/math.sin(theta);out/=np.linalg.norm(out);return out,(1-a)*positions[j-1]+a*positions[j]

def main():
 plan=read(OUT/'pose-freeze.json')
 for name,h in plan['pins'].items():check(sha(ROOT/name)==h,'frozen '+name)
 comp=read(OUT/'pose-completion.json')
 for name,key in [('pose-scores.json','scores_sha256'),('pose-summary.json','summary_sha256')]:check(sha(OUT/name)==comp[key],'completion '+name)
 d=np.load(OUT/'reference.npz');times=d['time_ns'];qs=d['quaternion_xyzw'];pos=d['position'];gyro=np.load(OUT/'raw-gyro.npz');gt=gyro['time_ns'];omega=gyro['omega']
 with zipfile.ZipFile(ROOT/'results/deployment-data-search/subt-smoke-gt.zip') as z:
  rows=[{k.strip():v.strip() for k,v in r.items()} for r in csv.DictReader(io.TextIOWrapper(z.open(next(n for n in z.namelist() if n.endswith('odometry_data.csv')))))];check(np.array_equal(times,np.array([int(r['timestamp']) for r in rows])),'reference timestamps reparse');check(np.array_equal(qs,np.array([[float(r['q_'+k]) for k in ['x','y','z','w']] for r in rows])),'reference quaternion reparse');check(np.array_equal(pos,np.array([[float(r[k]) for k in ['x','y','z']] for r in rows])),'reference position reparse')
  imu=[{k.strip():v.strip() for k,v in r.items()} for r in csv.DictReader(io.TextIOWrapper(z.open(next(n for n in z.namelist() if n.endswith('imu_data.csv')))))];it=np.array([int(r['timestamp']) for r in imu]);ig=np.array([[float(r['ang_vel_'+k]) for k in ['x','y','z']] for r in imu]);ids=np.searchsorted(it,gt);check(np.array_equal(it[ids],gt) and np.array_equal(ig[ids],omega),'all raw gyros match reference-side CSV');check(np.sum(~np.isin(it,gt))==2 and np.all(np.isin(times,it)),'two CSV-only IMU samples explicit')
 bindings=read(OUT/'image-bindings.json');expected={r['path']:r for r in read(PARENT/'images.json')};check(len(bindings)==len(expected)==22800,'whole image binding population')
 for r in bindings:
  e=expected[r['cache_path']];check(r['reproduced_npy_sha256']==e['sha256'] and r['arm']==e['arm'] and r['time_ns']==e['time_ns'],'raw/export model-input binding')
 gyro_rows=read(OUT/'gyro-reference-comparison.json');observed=[]
 for r in gyro_rows:
  if r.get('status','').endswith('unsupported'):check(r['max_imu_gap_ns']>20_000_000 or r['status']=='reference_gap_unsupported','integration gaps retained');continue
  t0,t1=r['t0_ns'],r['t1_ns'];a,b=at(t0,times,qs,pos),at(t1,times,qs,pos);check(a is not None and b is not None,'gyro reference brackets');relative=multiply(conjugate(a[0]),b[0]);left=int(np.searchsorted(gt,t0,side='right'))-1;right=int(np.searchsorted(gt,t1));knots=np.r_[t0,gt[left+1:right],t1];q=np.array([0.,0.,0.,1.])
  for k in range(len(knots)-1):
   j=int(np.searchsorted(gt,knots[k],side='right'))-1;v=omega[j]*((int(knots[k+1])-int(knots[k]))*1e-9);mag=np.linalg.norm(v);step=np.r_[v*(math.sin(mag/2)/mag) if mag else np.zeros(3),math.cos(mag/2)];q=multiply(q,step);q/=np.linalg.norm(q)
  error=angle(multiply(conjugate(q),relative));check(abs(error-r['gyro_error_deg'])<1e-6,'independent gyro integral error');check(abs(angle(relative)-r['reference_angle_deg'])<1e-6,'independent reference motion angle')
  if angle(relative)>=2:observed.append(error)
 raw=read(OUT/'raw-binding-qualification.json');check(len(observed)==raw['moving_intervals'] and len(observed)>=20,'moving gyro population');check(abs(np.median(observed)-raw['moving_error_median_deg'])<1e-6 and abs(np.percentile(observed,95)-raw['moving_error_p95_deg'])<1e-6,'gyro gate statistics');check(np.median(observed)<=.5 and np.percentile(observed,95)<=2,'original no-fit gyro gate')
 source=yaml.safe_load((ROOT/'results/deployment-data-search/smoke_extrinsics.yaml').read_text());transforms={}
 for arm in ['rgb','thermal']:
  o=np.array(source[arm+'_camera_to_imu']['data']).reshape(4,4);u,s,v=np.linalg.svd(o[:3,:3]);rotation=u@v;t=np.eye(4);t[:3,:3]=rotation;t[:3,3]=o[:3,3];check(np.linalg.norm(rotation-o[:3,:3])<1e-5 and np.linalg.det(rotation)>0,'roundoff-only transform construction');transforms[arm]=t
 refs={(r['arm'],r['index']):r for r in read(OUT/'camera-reference.json')};pop=read(OUT/'pose-population.json');common=set(pop['common_reference_indices']);eligible=set(pop['translation_ge1cm_both_indices']);scores={(r['frontend'],r['arm'],r['index']):r for r in read(OUT/'pose-scores.json')};seen=set()
 for key,r in refs.items():
  if not r['reference_supported']:continue
  a,b=[at(t,times,qs,pos) for t in r['times_ns']];check(a is not None and b is not None,'camera reference interpolation');t=transforms[r['arm']];ra,rb=matrix(a[0]),matrix(b[0]);ca,cb=ra@t[:3,:3],rb@t[:3,:3];pa=a[1]+ra@t[:3,3];pb=b[1]+rb@t[:3,3];rel=np.eye(4);rel[:3,:3]=cb.T@ca;rel[:3,3]=cb.T@(pa-pb);check(np.allclose(rel,r['target_from_source'],atol=1e-8),'independent camera relative transform');check(abs(r['translation_m']-np.linalg.norm(rel[:3,3]))<1e-10,'reference translation magnitude')
 expected_common={i for a,i in refs if all(refs[(arm,i)]['reference_supported'] for arm in ['rgb','thermal'])};expected_t={i for i in expected_common if all(refs[(arm,i)]['translation_m']>=.01 for arm in ['rgb','thermal'])};check(common==expected_common and eligible==expected_t,'preflight cohort definition')
 for line in (PARENT/'predictions.jsonl').read_text().splitlines():
  p=json.loads(line);key=(p['frontend'],p['arm'],p['index']);check(key not in seen,'unique prediction');seen.add(key);s=scores[key];r=refs[key[1:]];check(s['common_reference']==(p['index'] in common) and s['translation_qualified']==(p['index'] in eligible),'score eligibility binding');success1=success5=joint=False
  if s['common_reference'] and p['status']=='ok':
   truth=np.array(r['target_from_source']);err=math.degrees(math.acos(np.clip((np.trace(np.array(p['R'])@truth[:3,:3].T)-1)/2,-1,1)));check(abs(err-s['rotation_error_deg'])<1e-5,'independent fullSO3 score');success1=err<=1;success5=err<=5
   if s['translation_qualified']:
    a=np.array(p['t']);b=truth[:3,3];errt=math.degrees(math.atan2(np.linalg.norm(np.cross(a,b)),float(a@b)));check(abs(errt-s['translation_error_deg'])<1e-6,'independent directed translation score');joint=success5 and errt<=15
  else:check(s['rotation_error_deg'] is None,'failed/unscorable estimate stays missing')
  check(s['rotation_success_1deg']==success1 and s['rotation_success_5deg']==success5 and s['joint_success_5deg_15deg']==joint,'score threshold correctness')
 check(len(seen)==len(scores)==3352,'full population')
 summary=read(OUT/'pose-summary.json')
 for f,arms in summary.items():
  for arm,strata in arms.items():
   for name,v in strata.items():
    rows=[r for (ff,a,i),r in scores.items() if ff==f and a==arm and r['common_reference']];rows=[r for r in rows if name=='all' or (name=='reference_rotation_ge2deg' and r['reference_rotation_deg']>=2) or (name=='reference_rotation_lt2deg' and r['reference_rotation_deg']<2) or (name=='translation_ge1cm_both' and r['translation_qualified']) or (name.startswith('third') and r['index']*3//838==int(name[-1])-1)];check(len(rows)==v['intervals'],'stratum denominator')
    for field in ['rotation_success_1deg','rotation_success_5deg','identity_rotation_success_1deg','translation_qualified','joint_success_5deg_15deg']:check(sum(r[field] for r in rows)==v[field],'stratum count '+field)
    for prefix,key in [('rotation','rotation_error_deg'),('translation','translation_error_deg')]:
     values=[r[key] for r in rows if r[key] is not None]
     if values:check(abs(np.median(values)-v['conditional_'+prefix+'_median_deg'])<1e-12 and abs(np.percentile(values,90)-v['conditional_'+prefix+'_p90_deg'])<1e-12,'conditional distribution')
 paired={}
 for f in ['sift','learned']:
  paired[f]={}
  for name,ids in [('all',sorted(common)),('translation_ge1cm_both',sorted(eligible)),*[(f'third{j+1}',sorted(i for i in common if i*3//838==j)) for j in range(3)]]:
   paired[f][name]={}
   for metric in ['rotation_success_1deg','rotation_success_5deg','joint_success_5deg_15deg']:
    states=[(scores[(f,'rgb',i)][metric],scores[(f,'thermal',i)][metric]) for i in ids];c={'intervals':len(states),'both':sum(a and b for a,b in states),'rgb_only':sum(a and not b for a,b in states),'thermal_only':sum(b and not a for a,b in states),'neither':sum(not a and not b for a,b in states)};check(sum(c[k] for k in ['both','rgb_only','thermal_only','neither'])==len(ids),'paired partition');paired[f][name][metric]=c
 with (OUT/'pose-paired.json').open('x') as f:json.dump(paired,f,indent=2);f.write('\n')
 result={'status':'pass','checks':checks,'score_rows':len(scores),'common_reference_anchors':len(common),'translation_qualified_anchors':len(eligible),'independent_quaternion_gyro_and_rigid_algebra':True,'raw_images_independently_redecoded_by_auditor':False,'physical_reference_origin_metrology':False,'new_inference_calls':0,'source_sha256':sha(__file__)};(OUT/'pose-audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
