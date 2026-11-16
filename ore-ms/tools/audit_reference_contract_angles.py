"""Independent raw-mocap quaternion/trace audit of scalar-angle characterization."""
import hashlib,json,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/reference-contract';SOURCE=ROOT/'results/deployment-characterization';checks=0

def read(p):return json.loads(Path(p).read_text())
def check(v,label):
 global checks
 assert v,label
 checks+=1

def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def q_at(rows,t):
 times=[r['time_ns'] for r in rows];j=int(np.searchsorted(times,t));check(0<j<len(rows) and times[j]-times[j-1]<=50_000_000,'raw bracket');a=(t-times[j-1])/(times[j]-times[j-1]);q=np.array(rows[j-1]['q']);q/=np.linalg.norm(q);v=np.array(rows[j]['q']);v/=np.linalg.norm(v);dot=float(q@v)
 if dot<0:v=-v;dot=-dot
 dot=float(np.clip(dot,-1,1))
 if dot>1-1e-12:out=(1-a)*q+a*v
 else:
  theta=math.acos(dot);out=(math.sin((1-a)*theta)*q+math.sin(a*theta)*v)/math.sin(theta)
 return out/np.linalg.norm(out)
def main():
 plan=read(OUT/'angle-freeze.json')
 for name,h in plan['pins'].items():check(sha(ROOT/name)==h,'frozen '+name)
 comp=read(OUT/'angle-completion.json');check(sha(OUT/'angle-scores.json')==comp['scores_sha256'],'scores lock');check(sha(OUT/'angle-summary.json')==comp['summary_sha256'],'summary lock');check(sha(OUT/'angle-freeze.json')==comp['freeze_sha256'],'freeze lock')
 scores=read(OUT/'angle-scores.json');summary=read(OUT/'angle-summary.json');lookup={(s['condition'],s['frontend'],s['arm'],s['index']):s for s in scores};check(len(lookup)==1684,'all cached rows');refs={(r['condition'],r['arm'],r['index']):r for r in read(SOURCE/'references.json')};poses={c:read(SOURCE/(c+'-mocap.json')) for c in ['bright','switching','dim']};common=read(SOURCE/'common-reference-indices.json');seen=set()
 for line in (SOURCE/'predictions.jsonl').read_text().splitlines():
  p=json.loads(line);key=(p['condition'],p['frontend'],p['arm'],p['index']);check(key not in seen,'unique');seen.add(key);s=lookup[key];r=refs[(p['condition'],p['arm'],p['index'])];eligible=p['index'] in common[p['condition']];check(s['common_reference']==eligible,'reference eligibility')
  if eligible:
   q0,q1=[q_at(poses[p['condition']],t) for t in r['times_ns']];theta=math.degrees(2*math.acos(np.clip(abs(float(q0@q1)),-1,1)));check(abs(theta-s['reference_angle_deg'])<1e-5,'independent quaternion body angle');check(s['identity_success_1deg']==(theta<=1),'identity baseline')
   if p['status']=='ok':
    angle=math.degrees(math.acos(np.clip((np.trace(p['R'])-1)/2,-1,1)));err=abs(angle-theta);check(abs(angle-s['estimated_angle_deg'])<1e-5 and abs(err-s['angle_error_deg'])<1e-5,'trace angular amount and error');check(s['success_1deg']==(err<=1),'one-degree success')
   else:check(s['angle_error_deg'] is None and not s['success_1deg'],'rejected pose stays failure')
  else:check(s['reference_angle_deg'] is None and s['angle_error_deg'] is None and not s['success_1deg'],'unscorable remains separate')
 check(len(seen)==len(scores),'complete population')
 for c,fronts in summary.items():
  for f,arms in fronts.items():
   for arm,strata in arms.items():
    base=[r for r in scores if r['condition']==c and r['frontend']==f and r['arm']==arm and r['common_reference']]
    for stratum,want in strata.items():
     rows=[r for r in base if stratum=='all' or (r['reference_angle_deg']>=2 if stratum=='rotation_ge2deg' else r['reference_angle_deg']<2)];err=[r['angle_error_deg'] for r in rows if r['angle_error_deg'] is not None];check(want['intervals']==len(rows) and want['supported']==len(err),'summary denominator');success=sum(r['success_1deg'] for r in rows);check(want['success_1deg']==success and (not rows or abs(want['success_rate']-success/len(rows))<1e-12),'unconditional accuracy rate');check(want['identity_success_1deg']==sum(r['identity_success_1deg'] for r in rows),'identity count');check(not err or abs(want['conditional_median_error_deg']-np.median(err))<1e-12,'median');check(not err or abs(want['conditional_p90_error_deg']-np.percentile(err,90))<1e-12,'p90')
 paired={}
 for c in ['bright','switching','dim']:
  paired[c]={}
  for f in ['sift','learned']:
   pairs=[(lookup[(c,f,'rgb',i)],lookup[(c,f,'thermal',i)]) for i in common[c]]
   def aggregate(v):
    states=[(a['success_1deg'],b['success_1deg']) for a,b in v];counts={'intervals':len(v),'both':sum(a and b for a,b in states),'rgb_only':sum(a and not b for a,b in states),'thermal_only':sum(b and not a for a,b in states),'neither':sum(not a and not b for a,b in states)};check(sum(counts[k] for k in ['both','rgb_only','thermal_only','neither'])==len(v),'paired angular amount partition');return counts
   paired[c][f]={'all':aggregate(pairs),'both_reference_ge2deg':aggregate([(a,b) for a,b in pairs if min(a['reference_angle_deg'],b['reference_angle_deg'])>=2])}
 (OUT/'angle-paired.json').write_text(json.dumps(paired,indent=2)+'\n');result={'status':'pass','checks':checks,'score_rows':len(scores),'new_inference_calls':0,'independent_raw_quaternion_reference':True,'scope':'Angular-motion amount only; axis/orientation/translation and timing metrology remain unqualified','source_sha256':sha(__file__)};(OUT/'angle-audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
