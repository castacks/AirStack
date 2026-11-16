"""Independent reference/score/pose arithmetic audit; no matcher implementation import."""
import hashlib,json,math
from pathlib import Path
import cv2,numpy as np

ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/deployment-characterization'
checks=0

def check(value,label):
 global checks
 assert value,label
 checks+=1

def load(name):return json.loads((OUT/name).read_text())
def digest(path):
 with Path(path).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def quaternion_matrix(q):
 x,y,z,w=np.asarray(q)/np.linalg.norm(q)
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def pose_at(rows,t):
 times=[r['time_ns'] for r in rows];j=int(np.searchsorted(times,t));check(0<j<len(times),'bracket exists');a=(t-times[j-1])/(times[j]-times[j-1]);q0=np.array(rows[j-1]['q']);q0/=np.linalg.norm(q0);q1=np.array(rows[j]['q']);q1/=np.linalg.norm(q1)
 dot=float(q0@q1)
 if dot<0:q1=-q1;dot=-dot
 if dot>.9995:q=(1-a)*q0+a*q1
 else:
  theta=math.acos(np.clip(dot,-1,1));q=(math.sin((1-a)*theta)*q0+math.sin(a*theta)*q1)/math.sin(theta)
 result=np.eye(4);result[:3,:3]=quaternion_matrix(q);result[:3,3]=(1-a)*np.array(rows[j-1]['p'])+a*np.array(rows[j]['p']);return result

def main():
 cv2.setNumThreads(1)
 freeze=load('execution-freeze.json')
 for name,h in freeze['pins'].items():check(digest(ROOT/name)==h,'frozen member '+name)
 lock=load('prediction-lock.json');check(digest(OUT/'predictions.jsonl')==lock['predictions_sha256'],'prediction lock');check(digest(OUT/'execution-freeze.json')==lock['execution_freeze_sha256'],'freeze lock')
 refs={(x['condition'],x['arm'],x['index']):x for x in load('references.json')};images={x['path']:x for x in load('images.json')};common=load('common-reference-indices.json');summary=load('common-reference-summary.json');scores={(x['frontend'],x['condition'],x['arm'],x['index']):x for x in load('scores.json')};T=np.array(load('calibration-qualification.json')['thermal_to_rgb']);poses={c:load(c+'-mocap.json') for c in ['bright','switching','dim']}
 check(len(refs)==842,'842 sensor intervals');check(len(scores)==1684,'1684 score rows');check(len(images)==13517,'all decoded images')
 for key,r in refs.items():
  check(r['times_ns']==[images[r[k]]['time_ns'] for k in ['source','target']],'image timestamps bind')
  check(r['qualified']==(len(r['unsupported_reasons'])==0),'input support binding')
  if r['qualified']:
   p0,p1=[pose_at(poses[r['condition']],t) for t in r['times_ns']]
   if r['arm']=='thermal':p0=p0@T;p1=p1@T
   # Inverse rigid transform, independent of solve used by the runner.
   rel=np.eye(4);rel[:3,:3]=p1[:3,:3].T@p0[:3,:3];rel[:3,3]=p1[:3,:3].T@(p0[:3,3]-p1[:3,3]);check(np.allclose(rel,r['source_to_next_optical'],atol=1e-7),'independent reference reconstruction')
 for c,ids in common.items():
  expected=sorted({i for condition,arm,i in refs if condition==c and all(refs[(c,a,i)]['qualified'] and refs[(c,a,i)].get('translation_eligible',False) for a in ['rgb','thermal'])});check(ids==expected,'predeclared common indices')
 counters={};seen=set()
 with (OUT/'predictions.jsonl').open() as f:
  for line in f:
   p=json.loads(line);key=(p['frontend'],p['condition'],p['arm'],p['index']);check(key not in seen,'no duplicate prediction');seen.add(key);s=scores[key];r=refs[key[1:]]
   rotation_success=joint=False
   if p['status']=='ok':
    R=np.array(p['R']);t=np.array(p['t']);check(np.linalg.norm(R.T@R-np.eye(3))<1e-6 and abs(np.linalg.det(R)-1)<1e-6,'predicted proper rotation');check(abs(np.linalg.norm(t)-1)<1e-6,'unit translation')
    a=np.array(p['points0'],np.float64);b=np.array(p['points1'],np.float64);check(len(a)==len(b)==p['matches'],'match count binding')
    mask=np.array(p['essential_mask'],np.uint8);count,recovered_R,recovered_t,recovered_mask=cv2.recoverPose(np.array(p['E']),a,b,np.array(r['K']),mask=mask.copy());check(count==p['cheirality_inliers'],'independent cheirality count');check(np.allclose(recovered_R,R,atol=1e-10) and np.allclose(recovered_t.reshape(3),t,atol=1e-10),'recoverPose packet binding')
    if r['qualified']:
     truth=np.array(r['source_to_next_optical']);angle=math.degrees(math.acos(np.clip((np.trace(R@truth[:3,:3].T)-1)/2,-1,1)));check(abs(angle-s['rotation_error_deg'])<1e-5,'independent rotation score');rotation_success=angle<=5
     if r['translation_eligible']:
      v=truth[:3,3];translation=math.degrees(math.acos(np.clip(float(t@v)/np.linalg.norm(v),-1,1)));check(abs(translation-s['translation_error_deg'])<1e-6,'independent directed translation score');joint=rotation_success and translation<=15
   if not r['qualified'] or p['status']!='ok':check(s['rotation_error_deg'] is None and not s['joint_success'],'unscorable/failed estimates cannot become successes')
   check(s['rotation_success']==rotation_success and s['joint_success']==joint,'success definitions')
   if p['index'] in common[p['condition']]:
    g=key[:3];counts=counters.setdefault(g,{'intervals':0,'joint_success':0,'solver_supported':0});counts['intervals']+=1;counts['joint_success']+=joint;counts['solver_supported']+=p['status']=='ok'
 for (frontend,c,arm),counts in counters.items():
  target=summary[c][frontend][arm]
  for k,v in counts.items():check(target[k]==v,'common-reference denominator '+k)
  check(abs(target['joint_rate']-counts['joint_success']/counts['intervals'])<1e-12,'joint rate')
 check(len(seen)==len(scores)==lock['rows'],'all locked predictions covered')
 check(all(len(v)>0 for v in common.values()),'all conditions present')
 relationships={}
 for frontend in ['sift','learned']:
  relationships[frontend]={}
  scoremap={(x['condition'],x['arm'],x['index']):x for x in load('scores.json') if x['frontend']==frontend}
  for population in ['pooled','bright','switching','dim']:
   cases=[]
   for c,ids in common.items():
    if population!='pooled' and c!=population:continue
    for i in ids:
     rgb=refs[(c,'rgb',i)];a=rgb['source_metrics'];b=rgb['target_metrics'];rs=scoremap[(c,'rgb',i)]['joint_success'];ts=scoremap[(c,'thermal',i)]['joint_success']
     cases.append({'condition':c,'index':i,'rgb_success':rs,'thermal_success':ts,'rgb_contrast':min(a['p05_p95_range'],b['p05_p95_range']),'rgb_brightness':min(a['mean'],b['mean']),'rgb_brightness_change':abs(a['mean']-b['mean'])})
   groups={}
   plan=load('relationship-analysis-plan.json')
   for metric,bounds in plan['bins'].items():
    rows=[]
    for binid in range(len(bounds)+1):
     selected=[x for x in cases if int(np.searchsorted(bounds,x[metric],side='right'))==binid];n=len(selected)
     counts={'both':sum(x['rgb_success'] and x['thermal_success'] for x in selected),'rgb_only':sum(x['rgb_success'] and not x['thermal_success'] for x in selected),'thermal_only':sum(x['thermal_success'] and not x['rgb_success'] for x in selected),'neither':sum(not x['rgb_success'] and not x['thermal_success'] for x in selected)}
     check(sum(counts.values())==n,'observable paired denominator')
     rows.append({'bin':binid,'lower':bounds[binid-1] if binid else None,'upper':bounds[binid] if binid<len(bounds) else None,'intervals':n,**counts,'rgb_rate':(counts['both']+counts['rgb_only'])/n if n else None,'thermal_rate':(counts['both']+counts['thermal_only'])/n if n else None})
    check(sum(x['intervals'] for x in rows)==len(cases),'all observable bins retained');groups[metric]=rows
   relationships[frontend][population]=groups
 with (OUT/'relationships.json').open('x') as f:json.dump(relationships,f,indent=2);f.write('\n')
 result={'status':'pass','checks':checks,'predictions':len(seen),'decoded_images':len(images),'reference_arm_intervals':len(refs),'independent_reference_reconstruction':True,'recoverPose_packets_rechecked':True,'scope':'Provenance, reference interpolation/composition, pose packet and score arithmetic; not independent physical metrology or matcher reproduction','audit_source_sha256':digest(__file__)}
 with (OUT/'audit.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
 print(json.dumps(result,indent=2))
if __name__=='__main__':main()
