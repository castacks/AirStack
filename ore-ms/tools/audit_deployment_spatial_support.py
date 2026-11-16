"""Independent polynomial projection, histogram/window and summary audit."""
import hashlib,json,math,itertools
from pathlib import Path
import cv2,numpy as np,yaml
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/deployment-spatial-support';checks=0

def check(v,label):
 global checks
 assert v,label
 checks+=1

def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def project(points,c,arm,ref):
 p=np.asarray(points,float).reshape(-1,2)
 if not len(p):return p
 if c=='real_smoke':
  cal=yaml.safe_load((ROOT/'results/deployment-data-search/smoke_intrinsics.yaml').read_text())[arm+'_camera'];params=cal['projection_parameters'];xi=cal.get('mirror_parameters',{}).get('xi',0);h=512 if arm=='thermal' else 480;x=(p[:,0]-320)/400;y=(p[:,1]-h/2)/400;den=1+xi*np.sqrt(x*x+y*y+1);x=x/den;y=y/den
  fx=params.get('fx',params.get('gamma1'));fy=params.get('fy',params.get('gamma2'));cx=params.get('cx',params.get('u0'));cy=params.get('cy',params.get('v0'));dist=cal['distortion_parameters'];k1,k2,p1,p2=[dist[k] for k in ['k1','k2','p1','p2']]
 else:
  k=np.array(ref['K']);fx,fy,cx,cy=k[0,0],k[1,1],k[0,2],k[1,2];x=(p[:,0]-cx)/fx;y=(p[:,1]-cy)/fy;k1,k2,p1,p2=ref['D']
 r2=x*x+y*y;rad=1+k1*r2+k2*r2*r2;u=(x*rad+2*p1*x*y+p2*(r2+2*x*x))*fx+cx;v=(y*rad+p1*(r2+2*y*y)+2*p2*x*y)*fy+cy;h=512 if c=='real_smoke' and arm=='thermal' else 480
 a=np.round(np.column_stack([u,v]),4);check((a>=-.05).all() and (a[:,0]<=639.05).all() and (a[:,1]<=h-1+.05).all(),'independent native bounds');a[:,0]=np.clip(a[:,0],0,639);a[:,1]=np.clip(a[:,1],0,h-1);return a

def spatial(points,h,w):
 n=len(points);counts=np.histogram2d(points[:,1],points[:,0],bins=[np.linspace(0,h,9),np.linspace(0,w,9)])[0].astype(int).reshape(-1) if n else np.zeros(64,int);pos=counts[counts>0]
 entropy=float(np.sum(pos*(np.log(n)-np.log(pos)))/(n*math.log(64))) if n else 0.
 hull=0.
 if n>=3:
  poly=cv2.convexHull(points.astype(np.float32)).reshape(-1,2).astype(float);x,y=poly.T;hull=float(abs(x@np.roll(y,1)-y@np.roll(x,1))/2/(h*w))
 center=float(sum(.25*w<=x<.75*w and .25*h<=y<.75*h for x,y in points)/n) if n else None
 return {'points':n,'counts':counts.tolist(),'occupied_cells':int(sum(counts>=3)),'hull_fraction':hull,'entropy':entropy,'top4_fraction':sum(sorted(counts)[-4:])/n if n else None,'central_fraction':center}

def main():
 cv2.setNumThreads(1);freeze=read(OUT/'freeze.json')
 for name,d in freeze['pins'].items():check(sha(ROOT/name)==d,'frozen '+name)
 check(read(OUT/'window-fixture-qualification.json')['status']=='pass','actual-runner synthetic window/mask fixtures')
 fixture=read(OUT/'window-fixture-qualification.json');check(fixture['runner_sha256']==sha(ROOT/'tools/deployment_spatial_support.py'),'fixture runner version')
 metrics=read(OUT/'pair-metrics.json');windows=read(OUT/'window-metrics.json');summary=read(OUT/'summary.json');completion=read(OUT/'completion.json')
 for name,key in [('pair-metrics.json','pair_metrics_sha256'),('window-metrics.json','window_metrics_sha256'),('summary.json','summary_sha256')]:check(sha(OUT/name)==completion[key],'completed output hash')
 by_key={(r['condition'],r['frontend'],r['arm'],r['index']):r for r in metrics};check(len(by_key)==len(metrics)==5036,'pair population');natives=(json.loads(x) for x in (OUT/'native-cache.jsonl').open());raws=[];refs={}
 for folder,name in [(ROOT/'results/deployment-characterization','references.json'),(ROOT/'results/deployment-smoke-frontend','intervals.json')]:
  p=read(folder/'execution-freeze.json')
  for consumed,h in freeze['pins'].items():
   if consumed in p['pins']:check(h==p['pins'][consumed],'consumed dependency matches parent freeze')
  refs.update({(r.get('condition','real_smoke'),r['arm'],r['index']):r for r in read(folder/name)})
  raws.append(folder/'predictions.jsonl')
 def raw_iter():
  for path in raws:
   with path.open() as f:
    for line in f:yield json.loads(line)
 seen=set();groups={}
 for raw,native in itertools.zip_longest(raw_iter(),natives):
  check(raw is not None and native is not None,'native/raw population alignment');c=raw.get('condition','real_smoke');key=(c,raw['frontend'],raw['arm'],raw['index']);check(key not in seen,'unique raw key');seen.add(key);metric=by_key[key];r=refs[(c,raw['arm'],raw['index'])]
  check(all(native[k]==metric[k]==v for k,v in zip(['condition','frontend','arm','index'],key)),'metadata alignment');check(metric['source']==native['source']==r['source'] and metric['target']==native['target']==r['target'],'image binding')
  p,q=[project(raw.get(k,[]),c,raw['arm'],r) for k in ['points0','points1']];h=512 if c=='real_smoke' and raw['arm']=='thermal' else 480
  check(np.allclose(p,np.array(native['points0']).reshape(-1,2),atol=1e-8) and np.allclose(q,np.array(native['points1']).reshape(-1,2),atol=1e-8),'independent projection to native pixels')
  mask=np.array(raw.get('pose_mask',[0]*len(p))).reshape(-1)!=0;check(mask.astype(int).tolist()==native['pose_mask'],'mask binding')
  for mode,sel in [('accepted',np.ones(len(p),bool)),('cheirality',mask)]:
   sources=[]
   for side,points in [('source',p[sel]),('target',q[sel])]:
    own=spatial(points,h,640);target=metric[mode][side];sources.append(own)
    for k,value in own.items():
     if isinstance(value,list):check(value==target[k],'independent cell histogram')
     elif value is None:check(target[k] is None,'undefined empty metric')
     else:check(abs(value-target[k])<2e-6,'independent spatial '+k)
   expected=min(s['occupied_cells'] for s in sources);check(metric[mode]['min_cells']==expected and metric[mode]['broad_support']==(expected>=16),'pair breadth')
   disp=float(np.median(np.sqrt(np.sum((q[sel]-p[sel])**2,axis=1)))) if sel.any() else None;check(disp==metric[mode]['median_displacement_px'],'native displacement')
  groups.setdefault(key[:3],[]).append(metric)
 check(len(seen)==5036,'all consumed predictions');wlookup={(x['condition'],x['frontend'],x['arm'],x['mode'],x['index']):x for x in windows};check(len(wlookup)==len(windows)==10008,'all two-mode windows');expected_w=0
 for (c,f,a),rows in groups.items():
  rows.sort(key=lambda x:x['index']);n=len(rows)
  for mode in ['accepted','cheirality']:
   wins=[]
   for i in range(n-2):
    r=rows[i:i+3];linked=r[0]['index']+1==r[1]['index'] and r[1]['index']+1==r[2]['index'] and r[0]['target']==r[1]['source'] and r[1]['target']==r[2]['source'];mn=min(x[mode]['min_cells'] for x in r);sets=[{j for j,v in enumerate(x[mode][side]['counts']) if v>=3} for x in r for side in ['source','target']];shared=len(set.intersection(*sets));target=wlookup[(c,f,a,mode,r[0]['index'])];check(target['linked']==linked and target['min_cells']==mn and target['shared_cells']==shared,'independent window support');check(target['sustained_broad_support']==(linked and mn>=16),'window threshold');wins.append(target);expected_w+=1
   s=summary[c][f][a][mode];check(s['intervals']==n and s['windows']==len(wins),'summary denominators');check(s['broad_pairs']==sum(r[mode]['min_cells']>=16 for r in rows),'broad pair count');check(s['sustained_broad_windows']==sum(w['sustained_broad_support'] for w in wins),'sustained count');check(s['median_min_cells']==np.median([r[mode]['min_cells'] for r in rows]),'coverage median');check(s['median_shared_window_cells']==np.median([w['shared_cells'] for w in wins]),'shared cell median')
 check(expected_w==len(windows),'no omitted windows');write_strata(metrics,windows,groups)
 result={'status':'pass','checks':checks,'pair_rows':len(metrics),'window_rows':len(windows),'new_inference_calls':0,'independent_native_polynomial_projection':True,'source_sha256':sha(__file__),'scope':'All cached spatial/mask/window/count arithmetic; no physical match/pose truth or world-track identity'};(OUT/'audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))

def write_strata(metrics,windows,groups):
 paired={};strata={};thirds={};lookup={(r['condition'],r['frontend'],r['arm'],r['index']):r for r in metrics}
 for c,f,a in groups:
  if a!='rgb':continue
  rgb=sorted(groups[(c,f,a)],key=lambda x:x['index']);n=len(rgb);paired.setdefault(c,{}).setdefault(f,{});strata.setdefault(c,{}).setdefault(f,{})
  for mode in ['accepted','cheirality']:
   cases=[]
   for r in rgb:
    t=lookup[(c,f,'thermal',r['index'])];d=r['accepted']['median_displacement_px'];motion='missing' if d is None else '<1' if d<1 else '1-5' if d<5 else '>=5';contrast='<40' if r['rgb_contrast']<40 else '40-120' if r['rgb_contrast']<120 else '>=120';cases.append({'rgb':r[mode]['broad_support'],'thermal':t[mode]['broad_support'],'contrast':contrast,'motion':motion,'index':r['index']})
   def counts(v):
    result={'pairs':len(v),'both':sum(x['rgb'] and x['thermal'] for x in v),'rgb_only':sum(x['rgb'] and not x['thermal'] for x in v),'thermal_only':sum(x['thermal'] and not x['rgb'] for x in v),'neither':sum(not x['rgb'] and not x['thermal'] for x in v)};check(sum(result[k] for k in ['both','rgb_only','thermal_only','neither'])==len(v),'paired partition');return result
   paired[c][f][mode]=counts(cases);strata[c][f][mode]={metric:{label:counts([x for x in cases if x[metric]==label]) for label in labels} for metric,labels in [('contrast',['<40','40-120','>=120']),('motion',['missing','<1','1-5','>=5'])]}
   if c=='real_smoke':thirds.setdefault(f,{})[mode]={str(j+1):counts([x for x in cases if x['index']*3//n==j]) for j in range(3)}
 for name,v in [('paired-spatial.json',paired),('stratified-spatial.json',strata),('smoke-thirds-spatial.json',thirds)]:
  with (OUT/name).open('x') as f:json.dump(v,f,indent=2);f.write('\n')

if __name__=='__main__':main()
