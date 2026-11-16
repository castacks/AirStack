"""Cached spatial-support analysis; no inference or modification of frozen studies.

Existing runners store virtual coordinates and no spatial-window summaries. This
adapter consumes their immutable caches and reuses their camera projection only.
"""
import argparse,hashlib,json,math
from pathlib import Path
import cv2,numpy as np
import deployment_smoke_frontend as smoke
import deployment_lighting as light

ROOT=light.ROOT;OUT=ROOT/'results/deployment-spatial-support'
SOURCES=[(ROOT/'results/deployment-characterization','references.json'),(ROOT/'results/deployment-smoke-frontend','intervals.json')]

def read(p):return json.loads(Path(p).read_text())
def write(p,v):
 with Path(p).open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def native(points,condition,arm,ref):
 points=np.asarray(points,float).reshape(-1,2)
 if not len(points):return points
 if condition=='real_smoke':
  v=smoke.camera(arm)[3];rays=np.column_stack([(points-v[:2,2])/400,np.ones(len(points))]);return smoke.project(rays,arm)
 k=np.array(ref['K']);d=np.array(ref['D']);xy=(points-k[:2,2])/np.array([k[0,0],k[1,1]]);rays=np.column_stack([xy,np.ones(len(xy))]);p,_=cv2.projectPoints(rays,np.zeros(3),np.zeros(3),k,d);return p.reshape(-1,2)
def canonical(points,h,w):
 points=np.asarray(points,float).reshape(-1,2);assert np.isfinite(points).all()
 if not len(points):return points,0
 assert (points>=-.05).all() and (points[:,0]<=w-1+.05).all() and (points[:,1]<=h-1+.05).all(), 'native boundary'
 rounded=np.round(points,4);clipped=np.column_stack([np.clip(rounded[:,0],0,w-1),np.clip(rounded[:,1],0,h-1)]);return clipped,int(np.any(clipped!=rounded,axis=1).sum())
def spatial(points,h,w):
 points,clipped=canonical(points,h,w);n=len(points)
 ids=(np.minimum((points[:,1]/h*8).astype(int),7)*8+np.minimum((points[:,0]/w*8).astype(int),7)) if n else np.array([],int)
 counts=np.bincount(ids,minlength=64);positive=counts[counts>0];probs=positive/n if n else np.array([])
 entropy=float(-np.sum(probs*np.log(probs))/math.log(64)) if n else 0.
 hull=float(cv2.contourArea(cv2.convexHull(points.astype(np.float32)))/(h*w)) if n>=3 else 0.
 return {'points':n,'counts':counts.tolist(),'occupied_cells':int((counts>=3).sum()),'hull_fraction':hull,'entropy':entropy,'top4_fraction':float(np.sort(counts)[-4:].sum()/n) if n else None,'central_fraction':float(((points[:,0]>=.25*w)&(points[:,0]<.75*w)&(points[:,1]>=.25*h)&(points[:,1]<.75*h)).mean()) if n else None,'clipped_roundoff_points':clipped}
def qualify():
 OUT.mkdir(exist_ok=False);cv2.setNumThreads(1);checks=0;reports=[]
 p=np.array([[(x+.5)*80,(y+.5)*60] for y in range(8) for x in range(8) for _ in range(3)])
 a=spatial(p,480,640);assert a['occupied_cells']==64 and a['entropy']==1 and a['top4_fraction']==.0625;checks+=3
 a=spatial(np.tile([320.,240.],(192,1)),480,640);assert a['occupied_cells']==1 and a['top4_fraction']==1 and a['central_fraction']==1;checks+=3
 a=spatial([],480,640);assert a['occupied_cells']==0 and a['entropy']==0 and a['top4_fraction'] is None;checks+=3
 a=spatial([[0,0],[639,479],[80,60]],480,640);assert a['counts'][0]==a['counts'][9]==a['counts'][63]==1;checks+=1
 try:canonical([[-1,0]],480,640);raise RuntimeError('boundary guard did not reject')
 except AssertionError:checks+=1
 pins={str(Path(__file__).resolve().relative_to(ROOT)):sha(__file__),'docs/research/deployment-spatial-support-protocol-2026-11-16.md':sha(ROOT/'docs/research/deployment-spatial-support-protocol-2026-11-16.md')}
 for path in [ROOT/'tools/deployment_smoke_frontend.py',ROOT/'tools/deployment_lighting.py',ROOT/'tools/smokeseer_sift.py',ROOT/'tools/smokeseer_preflight.py',ROOT/'results/deployment-data-search/smoke_intrinsics.yaml']:
  pins[str(path.relative_to(ROOT))]=sha(path)
 total=0;link_breaks=0
 with (OUT/'native-cache.jsonl').open('x') as cache:
  for folder,refname in SOURCES:
   lock=read(folder/'prediction-lock.json');assert sha(folder/'predictions.jsonl')==lock['predictions_sha256'];refs=read(folder/refname);bound={(r.get('condition','real_smoke'),r['arm'],r['index']):r for r in refs};parent=read(folder/'execution-freeze.json');seen=set()
   for path in [folder/'predictions.jsonl',folder/'prediction-lock.json',folder/'execution-freeze.json',folder/refname]:
    key=str(path.relative_to(ROOT));pins[key]=sha(path)
    if key in parent['pins']:assert pins[key]==parent['pins'][key]
   for condition,arm in sorted({(key[0],key[1]) for key in bound}):
    ref=next(v for key,v in bound.items() if key[:2]==(condition,arm));h=512 if condition=='real_smoke' and arm=='thermal' else 480;p=light.grid(h,640)
    if condition=='real_smoke':virtual=smoke.lift(p,arm)
    else:virtual=cv2.undistortPoints(p[:,None,:],np.array(ref['K']),np.array(ref['D']),P=np.array(ref['K'])).reshape(-1,2)
    error=float(np.linalg.norm(native(virtual,condition,arm,ref)-p,axis=1).max());assert error<=.05;reports.append({'condition':condition,'arm':arm,'grid_roundtrip_max_px':error});checks+=1
   previous={}
   with (folder/'predictions.jsonl').open() as stream:
    for line in stream:
     row=json.loads(line);condition=row.get('condition','real_smoke');key=(condition,row['frontend'],row['arm'],row['index']);assert key not in seen;seen.add(key);ref=bound[(condition,row['arm'],row['index'])];h=512 if condition=='real_smoke' and row['arm']=='thermal' else 480
     p,q=[native(row.get(k,[]),condition,row['arm'],ref) for k in ['points0','points1']];p,_=canonical(p,h,640);q,_=canonical(q,h,640);assert len(p)==len(q)==row.get('matches',0)
     mask=np.array(row.get('pose_mask',[0]*len(p))).reshape(-1)!=0;assert len(mask)==len(p)
     if 'cheirality_inliers' in row:assert mask.sum()==row['cheirality_inliers']
     group=key[:3];prev=previous.get(group);linked=(prev is None or (prev['index']+1==row['index'] and prev['target']==ref['source']));link_breaks+=int(not linked);previous[group]=dict(index=row['index'],target=ref['target'])
     r={'condition':condition,'frontend':row['frontend'],'arm':row['arm'],'index':row['index'],'h':h,'w':640,'source':ref['source'],'target':ref['target'],'linked_to_previous':linked,'points0':p.tolist(),'points1':q.tolist(),'pose_mask':mask.astype(int).tolist(),'solver_supported':row['status']=='ok','rgb_contrast':None}
     rgb=bound[(condition,'rgb',row['index'])];r['rgb_contrast']=min(rgb['source_metrics']['p05_p95_range'],rgb['target_metrics']['p05_p95_range']);cache.write(json.dumps(r,allow_nan=False)+'\n');total+=1
   assert len(seen)==lock['rows']==len(refs)*2
 assert total==5036
 write(OUT/'qualification.json',{'checks':checks,'predictions':total,'link_breaks':link_breaks,'projection_controls':reports,'new_inference_calls':0,'opencv':cv2.__version__,'numpy':np.__version__})
 for p in [OUT/'native-cache.jsonl',OUT/'qualification.json']:pins[str(p.relative_to(ROOT))]=sha(p)
 write(OUT/'freeze.json',{'pins':pins,'scope':'Exposed cached spatial/temporal descriptive study; no inference/physical accuracy/identity-linked tracks'});print('Qualified cached population',total,'link breaks',link_breaks,flush=True)
def run():
 freeze=read(OUT/'freeze.json')
 for name,digest in freeze['pins'].items():assert sha(ROOT/name)==digest,name
 rows=[];cv2.setNumThreads(1)
 with (OUT/'native-cache.jsonl').open() as stream:
  for line in stream:
   r=json.loads(line);p=np.array(r['points0']).reshape(-1,2);q=np.array(r['points1']).reshape(-1,2);mask=np.array(r['pose_mask'],bool);v={k:r[k] for k in ['condition','frontend','arm','index','source','target','linked_to_previous','solver_supported','rgb_contrast']}
   for mode,sel in [('accepted',np.ones(len(p),bool)),('cheirality',mask)]:
    a,b=spatial(p[sel],r['h'],r['w']),spatial(q[sel],r['h'],r['w']);v[mode]={'source':a,'target':b,'min_cells':min(a['occupied_cells'],b['occupied_cells']),'median_displacement_px':float(np.median(np.linalg.norm(q[sel]-p[sel],axis=1))) if sel.any() else None,'broad_support':min(a['occupied_cells'],b['occupied_cells'])>=16}
   rows.append(v)
 write(OUT/'pair-metrics.json',rows);groups={}
 for r in rows:groups.setdefault((r['condition'],r['frontend'],r['arm']),[]).append(r)
 windows=[];summary={}
 for (c,f,a),group in sorted(groups.items()):
  group.sort(key=lambda x:x['index']);n=len(group);summary.setdefault(c,{}).setdefault(f,{})[a]={};result=summary[c][f][a]
  for mode in ['accepted','cheirality']:
   win=[]
   for i in range(n-2):
    v=group[i:i+3];linked=all(v[j]['index']+1==v[j+1]['index'] and v[j]['target']==v[j+1]['source'] for j in [0,1]);maps=[np.array(r[mode][side]['counts'])>=3 for r in v for side in ['source','target']];cells=min(r[mode]['min_cells'] for r in v);shared=int(np.logical_and.reduce(maps).sum());w={'condition':c,'frontend':f,'arm':a,'mode':mode,'index':v[0]['index'],'linked':linked,'min_cells':cells,'shared_cells':shared,'sustained_broad_support':bool(linked and cells>=16)};win.append(w);windows.append(w)
   source=[r[mode]['source'] for r in group];target=[r[mode]['target'] for r in group]
   result[mode]={'intervals':n,'median_min_cells':float(np.median([r[mode]['min_cells'] for r in group])),'broad_pairs':sum(r[mode]['broad_support'] for r in group),'windows':len(win),'sustained_broad_windows':sum(r['sustained_broad_support'] for r in win),'median_window_min_cells':float(np.median([r['min_cells'] for r in win])),'median_shared_window_cells':float(np.median([r['shared_cells'] for r in win])),'median_source_hull_fraction':float(np.median([r['hull_fraction'] for r in source])),'median_source_entropy':float(np.median([r['entropy'] for r in source])),'median_source_top4_fraction_nonempty':float(np.median([r['top4_fraction'] for r in source if r['top4_fraction'] is not None])) if any(r['top4_fraction'] is not None for r in source) else None}
 write(OUT/'window-metrics.json',windows);write(OUT/'summary.json',summary)
 write(OUT/'completion.json',{'pair_rows':len(rows),'window_rows':len(windows),'new_inference_calls':0,'pair_metrics_sha256':sha(OUT/'pair-metrics.json'),'window_metrics_sha256':sha(OUT/'window-metrics.json'),'summary_sha256':sha(OUT/'summary.json'),'freeze_sha256':sha(OUT/'freeze.json')});print(json.dumps(summary,indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('stage',choices=['qualify','run']);a=p.parse_args();{'qualify':qualify,'run':run}[a.stage]()
