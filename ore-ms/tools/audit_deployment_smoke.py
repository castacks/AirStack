"""Independent real-smoke manifest and pose-packet arithmetic audit; no runner import."""
import hashlib,json
from pathlib import Path
import cv2,numpy as np
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/deployment-smoke-frontend';checks=0

def check(v,label):
 global checks
 assert v,label
 checks+=1

def read(name):return json.loads((OUT/name).read_text())
def sha(path):
 with Path(path).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def main():
 cv2.setNumThreads(1);images=read('images.json');check(len(images)==22800,'all decoded smoke images');plan=read('execution-freeze.json')
 for path,digest in plan['pins'].items():check(sha(ROOT/path)==digest,'freeze '+path)
 lock=read('prediction-lock.json');check(sha(OUT/'predictions.jsonl')==lock['predictions_sha256'],'locked predictions')
 images=read('images.json');frames={(x['arm'],x['index']):x for x in images};by_path={x['path']:x for x in images};intervals=read('intervals.json');refs={(x['arm'],x['index']):x for x in intervals};check(len(images)==len(frames)==22800,'all 22800 images');check(len(intervals)==1676,'1676 sensor intervals');seen=set();counts={}
 for key,r in refs.items():
  a,b=[by_path[r[k]] for k in ['source','target']];errors=[a['time_ns']-r['anchor_ns'],b['time_ns']-(r['anchor_ns']+500_000_000)]
  check(errors==r['anchor_errors_ns'],'anchor errors bind');check(r['qualified']==(max(abs(x) for x in errors)<=25_000_000 and b['index']>a['index']),'availability gate')
 for line in (OUT/'predictions.jsonl').read_text().splitlines():
  p=json.loads(line);key=(p['frontend'],p['arm'],p['index']);check(key not in seen,'unique prediction');seen.add(key);r=refs[key[1:]];check(p['qualified']==r['qualified'],'whole-span qualification binding')
  if p['qualified']:
   a=np.array(p['points0'],float).reshape(-1,2);b=np.array(p['points1'],float).reshape(-1,2);check(len(a)==len(b)==p['matches'],'accepted matches bind')
   if p['frontend']=='learned':check(0<=p['cycle_accepted']<=p['grid_inside']<=4096 and p['cycle_accepted']==len(a),'query/cycle budget')
   if p['status']=='ok':
    h=480 if p['arm']=='rgb' else 512;k=np.array([[400.,0,320],[0,400.,h/2],[0,0,1.]]);count,R,t,mask=cv2.recoverPose(np.array(p['E']),a,b,k,mask=np.array(p['essential_mask'],np.uint8));check(count==p['cheirality_inliers'] and count>=8,'shared solver support');check(np.allclose(R,p['R'],atol=1e-10) and np.allclose(t.reshape(-1),p['t'],atol=1e-10),'geometry packet binding')
  else:check(p['status']=='input_unsupported' and p['matches']==0,'missing input remains missing')
  c=counts.setdefault((p['frontend'],p['arm']),{'intervals':0,'supported':0,'at_least_8_matches':0});c['intervals']+=1;c['supported']+=p['status']=='ok';c['at_least_8_matches']+=p['matches']>=8
 check(len(seen)==lock['rows']==len(intervals)*2,'all prediction rows')
 availability=json.loads((ROOT/'results/deployment-characterization/availability-summary.json').read_text())['real_smoke']
 for (f,arm),c in counts.items():
  for k,v in c.items():check(availability[f][arm][k]==v,'independent smoke aggregate '+k)
 result={'status':'pass','checks':checks,'predictions':len(seen),'images':len(images),'independent_geometry_packet_checks':True,'pose_accuracy_claim':False,'source_sha256':sha(__file__)}
 with (OUT/'audit.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
 print(json.dumps(result,indent=2))
if __name__=='__main__':main()
