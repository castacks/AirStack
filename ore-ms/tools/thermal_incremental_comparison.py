"""Fixed exploratory image-feature ablation on saved nominal RGB/thermal motions."""
import hashlib,json,tarfile,time
from pathlib import Path
import cv2,numpy as np
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/thermal-incremental-comparison';REC=OUT/'recovery';RUN=OUT/'run-001';TMP=Path('/tmp/thermal-intake')
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def write(p,d):p.write_text(json.dumps(d,indent=2,allow_nan=False)+'\n')
def recover():
 receipts=[]
 with tarfile.open(TMP/'nominal.tar.gz') as t:
  for c in ['day','night','rain']:
   x=next(x for x in t.getmembers() if '/unweighted-longer/run-001/inputs/'+c+'/source-manifest.json' in x.name);(REC/(c+'-source.json')).write_bytes(t.extractfile(x).read())
 for c in ['day','night','rain']:
  m=read(TMP/(c+'-raw-manifest.json'));source=read(REC/(c+'-source.json'));binding=read(REC/('binding-'+c+'.json'));assert source['sequence']==binding['sequence']
  wanted={}
  for row in source['records'][:100]:
   for cam in ['rgb','thr']:
    e=row['cameras'][cam]['left'];key=f"/{source['sequence']}/{cam}/img_left/{row['stem']}.png";wanted[key]=(e,row['stem'],cam)
  found=set()
  for p in sorted((TMP/'raw-parts').glob(c+'-*.tar')):
   meta=next(x for x in m['parts'] if x['file']==p.name);assert sha(p)==meta['sha256']
   with tarfile.open(p) as t:
    for x in t:
     suffix=next((k for k in wanted if x.name.endswith(k)),None)
     if suffix is None:continue
     e,stem,cam=wanted[suffix];b=t.extractfile(x).read();h=hashlib.sha256(b).hexdigest();assert h==e['sha256']==m['member_sha256'][x.name]
     dest=REC/'images'/c/cam/(stem+'.png');dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(b);found.add(suffix)
  assert len(found)==200,(c,len(found))
  for row in binding['rows']:
   for cam,bkey in [('rgb','rgb_left'),('thr','thermal_left')]:
    times=[source['records'][int(s)]['cameras'][cam]['left']['recorded_time_ns'] for s in row['source_ids']];assert times==row['actual_camera_times_ns'][bkey]
  receipts.append({'condition':c,'sequence':source['sequence'],'verified_images':len(found),'source_sha256':sha(REC/(c+'-source.json'))})
 write(REC/'image-binding.json',receipts);print('BIND',receipts,flush=True)
def image(c,cam,stem):
 p=REC/'images'/c/cam/(stem+'.png');a=cv2.imread(str(p),cv2.IMREAD_UNCHANGED);assert a is not None
 if a.ndim==3:a=cv2.cvtColor(a,cv2.COLOR_BGR2GRAY)
 x=a.astype(float);lo,hi=np.percentile(x,[5,95]);z=(x-lo)/(hi-lo) if hi>lo else np.zeros_like(x);gx=(z[1:-1,2:]-z[1:-1,:-2])/2;gy=(z[2:,1:-1]-z[:-2,1:-1])/2
 ev=np.linalg.eigvalsh([[np.mean(gx*gx),np.mean(gx*gy)],[np.mean(gx*gy),np.mean(gy*gy)]]);minor=ev[0]/ev.sum() if ev.sum()>0 else 0
 return np.uint8(np.clip(z,0,1)*255),[float(np.sqrt(ev.sum())),float(minor)]
def temporal(a,b):
 pts=cv2.goodFeaturesToTrack(a,200,.01,5)
 if pts is None:return [0.,0.]
 q,s,_=cv2.calcOpticalFlowPyrLK(a,b,pts,None,winSize=(21,21),maxLevel=3);back,sb,_=cv2.calcOpticalFlowPyrLK(b,a,q,None,winSize=(21,21),maxLevel=3)
 h,w=a.shape;v=(s[:,0]>0)&(sb[:,0]>0)&(np.linalg.norm(back[:,0]-pts[:,0],axis=1)<=1)&(q[:,0,0]>=0)&(q[:,0,0]<w)&(q[:,0,1]>=0)&(q[:,0,1]<h)
 xy=pts[v,0];cells={(min(7,int(x/w*8)),min(3,int(y/h*4))) for x,y in xy};return [float(v.mean()),len(cells)/32]
def features():
 RUN.mkdir(parents=True,exist_ok=False);cv2.setNumThreads(1);cv2.setRNGSeed(0);allrows=[];start=time.perf_counter()
 for c in ['day','night','rain']:
  binding=read(REC/('binding-'+c+'.json'));pred=read(REC/('predictions-'+c+'.json'))['rows'];imgs={cam:[image(c,cam,f'{i:06}') for i in range(100)] for cam in ['rgb','thr']}
  for row,pr in zip(binding['rows'],pred):
   assert row['pair']==pr['pair'];i=row['pair'];methods=pr['methods'];ar=methods['native_rgb'].get('transform');at=methods['native_thermal'].get('transform');entry={'condition':c,'pair':i,'source_ids':row['source_ids'],'rgb':ar,'thermal':at,'summary':None,'image':None}
   if ar is not None and at is not None and all(row['cameras'][cam]['valid'] for cam in ['rgb','thermal']):
    stats=[]
    for cam in ['rgb','thermal']:
     packet=REC/row['cameras'][cam]['packet'];assert sha(packet)==row['cameras'][cam]['sha256']
     with np.load(packet) as z:
      depth=np.linalg.norm(z['points'],axis=1);stats.append(np.array([len(depth),np.trace(z['covariance'],axis1=1,axis2=2).mean(),np.median(depth),max(float(np.subtract(*np.percentile(depth,[75,25]))),1e-12)]))
    a,b=np.asarray(ar),np.asarray(at);theta=np.arccos(np.clip((np.trace(a[:3,:3].T@b[:3,:3])-1)/2,-1,1));entry['summary']=(np.r_[np.log(stats[1]/stats[0]),np.linalg.norm(a[:3,3]-b[:3,3]),theta,np.linalg.norm(a[:3,3]),np.linalg.norm(b[:3,3])]).tolist()
    entry['image']=sum((imgs[cam][i+1][1]+temporal(imgs[cam][i][0],imgs[cam][i+1][0]) for cam in ['rgb','thr']),[])
   allrows.append(entry)
  print('FEATURES',c,flush=True)
 write(RUN/'features.json',{'rows':allrows,'feature_seconds':time.perf_counter()-start,'source_sha256':sha(__file__),'references_read':False,'binding_sha256':sha(REC/'image-binding.json')})
def fit():
 rows=read(RUN/'features.json')['rows'];labels={c:{r['pair']:np.asarray(r['reference_delta']) for r in read(REC/('scores-'+c+'.json'))['rows']} for c in ['day','night','rain']};pred=[];models=[];audit=[]
 for held in ['day','night','rain']:
  train=[r for r in rows if r['condition']!=held and r['summary'] is not None];counts={c:sum(r['condition']==c for r in train) for c in ['day','night','rain'] if c!=held};w=np.array([1/(2*counts[r['condition']]) for r in train]);delta=np.array([np.asarray(r['thermal'])[:3,3]-np.asarray(r['rgb'])[:3,3] for r in train]);err=np.array([np.asarray(r['rgb'])[:3,3]-labels[r['condition']][r['pair']][:3,3] for r in train]);constant=float(np.clip(-np.sum(w[:,None]*delta*err)/np.sum(w[:,None]*delta*delta),0,1));mods={}
  for mode in ['summary','augmented']:
   x=np.array([r['summary']+(r['image'] if mode=='augmented' else []) for r in train]);mean=w@x;scale=np.sqrt(w@((x-mean)**2));scale[scale==0]=1;z=np.c_[np.ones(len(x)),(x-mean)/scale];d2=np.sum(delta*delta,axis=1);ed=np.sum(err*delta,axis=1);pen=np.diag([0]+[1]*x.shape[1]);beta=np.linalg.solve(z.T@((w*d2)[:,None]*z)+pen,-z.T@(w*ed))
   # Independent augmented least-squares fit, including all vector components.
   design=(z[:,None,:]*delta[:,:,None]).reshape(-1,z.shape[1]);target=(-err).reshape(-1);ww=np.repeat(np.sqrt(w),3);reg=np.eye(z.shape[1])[1:];other=np.linalg.lstsq(np.r_[ww[:,None]*design,reg],np.r_[ww*target,np.zeros(len(reg))],rcond=None)[0];assert np.allclose(beta,other,rtol=1e-8,atol=1e-10)
   mods[mode]={'mean':mean.tolist(),'scale':scale.tolist(),'beta':beta.tolist()};audit.append({'held':held,'mode':mode,'max_fit_difference':float(np.max(np.abs(beta-other)))})
  models.append({'held':held,'training_counts':counts,'constant':constant,'models':mods})
  for r in [r for r in rows if r['condition']==held]:
   if r['summary'] is None:continue
   weights={'rgb':0.,'thermal_translation':1.,'equal':.5,'constant':constant}
   for mode,m in mods.items():
    x=np.array(r['summary']+(r['image'] if mode=='augmented' else []));weights[mode]=float(np.clip(np.r_[1,(x-m['mean'])/m['scale']]@m['beta'],0,1))
   a,b=np.asarray(r['rgb']),np.asarray(r['thermal']);pred.append({'condition':held,'pair':r['pair'],'weights':weights,'translations':{name:((1-v)*a[:3,3]+v*b[:3,3]).tolist() for name,v in weights.items()},'rgb_rotation':a[:3,:3].tolist()})
 # Lock every fold before evaluation metrics.
 write(RUN/'prediction-lock.json',{'features_sha256':sha(RUN/'features.json'),'models':models,'rows':pred,'fit_checks':audit})
def score():
 lock=read(RUN/'prediction-lock.json');summary={};details=[]
 for c in ['day','night','rain']:
  refs={r['pair']:r for r in read(REC/('scores-'+c+'.json'))['rows']};rr=[r for r in lock['rows'] if r['condition']==c];summary[c]={'paired':len(rr),'expected':99,'methods':{}}
  for mode in ['rgb','thermal_translation','equal','constant','summary','augmented']:
   errors=[]
   for row in rr:
    ref=np.asarray(refs[row['pair']]['reference_delta']);t=np.array(row['translations'][mode]);error=float(np.linalg.norm(t-ref[:3,3]));scalar=sum((float(t[j])-float(ref[j,3]))**2 for j in range(3))**.5;assert abs(error-scalar)<1e-12
    if mode=='rgb':assert abs(error-refs[row['pair']]['errors']['native_rgb']['translation_m'])<1e-7
    errors.append(error)
   x=np.array(errors);summary[c]['methods'][mode]={'mean_m':float(x.mean()),'rmse_m':float(np.sqrt(np.mean(x*x))),'p95_m':float(np.percentile(x,95))}
  summary[c]['advances']=all(summary[c]['methods']['augmented'][metric]<summary[c]['methods'][base][metric] for metric in ['mean_m','rmse_m'] for base in ['rgb','summary'])
 write(RUN/'scores.json',{'prediction_lock_sha256':sha(RUN/'prediction-lock.json'),'summary':summary,'scope':'Same-ID nominal provided-reference development; native RGB rotation fixed; no new native VO/trajectory/runtime savings claim.'});print(json.dumps(summary,indent=2))
if __name__=='__main__':
 import sys
 {'recover':recover,'features':features,'fit':fit,'score':score}[sys.argv[1]]()
