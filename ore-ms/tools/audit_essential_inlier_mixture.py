"""Independent native projection, mask contingency and spatial reconstruction."""
import json,hashlib,math
from pathlib import Path
import numpy as np,yaml
from scipy.spatial import ConvexHull
ROOT=Path(__file__).resolve().parents[1]
def read(p):return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def quant(v,q):
    v=sorted(v);x=(len(v)-1)*q;l=int(x);h=math.ceil(x);return v[l]+(v[h]-v[l])*(x-l)
def summary(v):
    good=[x for x in v if x is not None];return {'n':len(good),'missing':len(v)-len(good),'quartiles':[quant(good,q) for q in [.25,.5,.75]] if good else None}
def project(points,c):
    xy=(points-np.array([c['image_width']/2,c['image_height']/2]))/400;xi=c.get('mirror_parameters',{}).get('xi',0);xy=xy/(1+xi*np.sqrt(1+np.sum(xy*xy,axis=1)))[:,None];x,y=xy.T;rr=x*x+y*y;d=c['distortion_parameters'];k1,k2,p1,p2=[d[k] for k in ['k1','k2','p1','p2']];rad=1+k1*rr+k2*rr*rr;xx=x*rad+2*p1*x*y+p2*(rr+2*x*x);yy=y*rad+p1*(rr+2*y*y)+2*p2*x*y;p=c['projection_parameters'];out=np.column_stack([p.get('fx',p.get('gamma1'))*xx+p.get('cx',p.get('u0')),p.get('fy',p.get('gamma2'))*yy+p.get('cy',p.get('v0'))]);h,w=c['image_height'],c['image_width'];assert (out>=-.05).all() and (out[:,0]<=w-1+.05).all() and (out[:,1]<=h-1+.05).all();out=np.round(out,4);out[:,0]=out[:,0].clip(0,w-1);out[:,1]=out[:,1].clip(0,h-1);return out

def spatial_check(points,h,w):
    n=len(points);cells=np.zeros(64,int)
    for x,y in points:cells[min(int(y/h*8),7)*8+min(int(x/w*8),7)]+=1
    positive=cells[cells>0];prob=positive/n if n else [];entropy=-sum(float(p)*math.log(float(p)) for p in prob)/math.log(64);hull=0.
    if n>=3:
        p=np.unique(points.astype(np.float32),axis=0)
        if len(p)>=3 and np.linalg.matrix_rank(p.astype(float)-p.mean(axis=0))>=2:hull=float(ConvexHull(p).volume/(h*w))
    return {'points':n,'counts':cells.tolist(),'occupied_cells':int(sum(cells>=3)),'hull_fraction':hull,'entropy':entropy,'top4_fraction':float(sum(sorted(cells)[-4:])/n) if n else None,'central_fraction':float(sum(.25*w<=x<.75*w and .25*h<=y<.75*h for x,y in points)/n) if n else None,'clipped_roundoff_points':0}

def main(config):
    cfg=read(config);out=ROOT/cfg['output'];count=0
    def check(v,label):
        nonlocal count
        assert v,label
        count+=1
    def equal(a,b,label):
        if isinstance(b,dict):
            check(set(a)==set(b),label+' fields')
            for k,v in b.items():equal(a[k],v,label+'.'+k)
        elif isinstance(b,list):
            check(len(a)==len(b),label+' length')
            for x,y in zip(a,b):equal(x,y,label)
        elif isinstance(b,(float,np.floating)):check(a is not None and abs(a-float(b))<1e-6,label)
        else:check(a==b,label)
    freeze=read(out/'input-freeze.json');check(freeze['low_motion_ray_deg']==.1,'fixed per-ray assay')
    for p,h in freeze['pins'].items():check(sha(ROOT/p)==h,'frozen '+p)
    lock=read(out/'feature-lock.json');check(lock['reference_values_parsed'] is False and sha(out/'features.json')==lock['features_sha256'],'feature lock');done=read(out/'completion.json');check(done['new_inference_calls']==done['new_mask_or_pose_calls']==0,'no policy/fit')
    for file,key in [('summary','summary_sha256'),('sample-features','sample_features_sha256'),('samples','samples_sha256')]:check(sha(out/(file+'.json'))==done[key],'completion '+file)
    rows=read(out/'features.json');ss=read(out/'summary.json');sample=read(out/'samples.json');trace=read(out/'sample-features.json');oldcases=read(ROOT/cfg['strata']/'cases.json');prior=read(ROOT/cfg['strata']/'samples.json');equal(sample,prior,'unchanged visual sample');modes=['accepted','essential','nonessential','cheirality','essential_not_cheirality']
    for name,c in cfg['records'].items():
        raw=[json.loads(l) for l in (ROOT/c['matching']/'predictions.jsonl').read_text().splitlines()];pop=read(ROOT/c['qualification']/'population.json');ids=set(pop['paired_rotation_ge2deg']);intr=yaml.safe_load((ROOT/read(ROOT/c['comparison_config'])['intrinsics']).read_text());lookup={(r['frontend'],r['arm'],r['index']):r for r in rows[name]};check(len(rows[name])==len(lookup)==4*len(ids),'full population');location={(r['frontend'],r['arm'],r['index']):r for r in read(ROOT/cfg['location']/'rows.json')[name]}
        for p in raw:
            if p['index'] not in ids:continue
            key=p['frontend'],p['arm'],p['index'];r=lookup[key];check(all(r[k]==p[k] for k in ['frontend','arm','index','status']),'source key');cam=intr[p['arm']+'_camera'];h,w=cam['image_height'],cam['image_width'];center=np.array([w/2,h/2]);pp=[np.asarray(p.get(k,[]),float).reshape(-1,2) for k in ['points0','points1']];rays=[]
            for points in pp:
                v=np.column_stack([(points-center)/400,np.ones(len(points))]);v/=np.linalg.norm(v,axis=1)[:,None];rays.append(v)
            a,b=rays;angle=np.degrees(np.arctan2(np.linalg.norm(np.cross(a,b),axis=1),np.sum(a*b,axis=1)));low=angle<=.1;n=len(a);check(n==len(b)==p.get('matches',0),'count binding');em=np.asarray(p.get('essential_mask',[0]*n)).reshape(-1)!=0;pm=np.asarray(p.get('pose_mask',[0]*n)).reshape(-1)!=0;check(len(em)==len(pm)==n and not np.any(pm&~em) and int(pm.sum())==p.get('cheirality_inliers',0),'nested original masks');masks=[np.ones(n,bool),em,~em,pm,em&~pm];native=[project(v,cam) for v in pp]
            for mode,mask in zip(modes,masks):
                d=r['masks'][mode];v=angle[mask];expected={'count':int(mask.sum()),'low_count':int((mask&low).sum()),'low_fraction':float((mask&low).sum()/mask.sum()) if mask.any() else None,'angle_quartiles':[quant(v,q) for q in [.25,.5,.75]] if len(v) else None};equal({k:d[k] for k in expected},expected,'motion mask '+mode)
                for side,points in zip(['source','target'],native):equal(d[side],spatial_check(points[mask],h,w),'independent spatial '+side)
                if mode in ['accepted','essential','cheirality']:
                    old=location[key]['subsets'][mode];check(d['count']==old['points'],'old mask count parity');v=d['angle_quartiles'];check(v is None if old['ray_change']['median'] is None else abs(v[1]-old['ray_change']['median'])<1e-8,'old ray median parity')
            nl=int(low.sum());nh=n-nl;el=int((low&em).sum());eh=int((~low&em).sum());equal(r['retention'],{'accepted_low':nl,'accepted_high':nh,'essential_low':el,'essential_high':eh,'low_retention':el/nl if nl else None,'high_retention':eh/nh if nh else None},'retention contingency')
        jobs=[]
        for f in ['sift','learned']:
            for a in ['rgb','thermal']:jobs.append(([r for r in rows[name] if r['frontend']==f and r['arm']==a],ss[name][f][a]))
        for cell in ['both_correct','rgb_only_correct','thermal_only_correct','neither_correct']:
            keep={r['index'] for r in oldcases[name] if r['endpoint']=='R5' and r['cell']==cell}
            for a in ['rgb','thermal']:jobs.append(([r for r in rows[name] if r['frontend']=='learned' and r['arm']==a and r['index'] in keep],ss[name]['learned_R5_strata'][cell][a]))
        for rr,d in jobs:
            check(d['anchors']==len(rr) and d['outputs']==sum(r['status']=='ok' for r in rr),'group denominator')
            for field in ['low_retention','high_retention']:equal(d['retention'][field],summary([r['retention'][field] for r in rr]),'aggregate retention')
            for mode in modes:
                for field in ['count','low_count','low_fraction']:equal(d['masks'][mode][field],summary([r['masks'][mode][field] for r in rr]),'aggregate mask '+field)
                for side in ['source','target']:
                    for field in ['occupied_cells','hull_fraction','entropy','top4_fraction']:equal(d['masks'][mode][side][field],summary([r['masks'][mode][side][field] for r in rr]),'aggregate spatial '+field)
                equal(d['masks'][mode]['angle_median'],summary([r['masks'][mode]['angle_quartiles'][1] if r['masks'][mode]['angle_quartiles'] else None for r in rr]),'aggregate angle')
        keep={s['index'] for s in prior[name]};equal(trace[name],[r for r in rows[name] if r['frontend']=='learned' and r['index'] in keep],'full prior visual trace')
    result={'status':'pass','checks':count,'rows':sum(len(r) for r in rows.values()),'auditor_sha256':sha(__file__),'scope':'Independent native distortion/rays/masks/low-motion retention, spatial grids/hulls/entropy and full cohorts/strata/sample aggregates; no causal or selector validation'};(out/'audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return result

def overlays(config):
    cfg=read(config);out=ROOT/cfg['output'];receipt=read(out/'render-receipts.json');samples=read(out/'samples.json');checks=0;expected=[]
    assert read(out/'audit.json')['status']=='pass'
    for name,c in cfg['records'].items():
        inputs={(r['arm'],r['index']):r for r in read(ROOT/c['matching']/'matcher-inputs.json')};images={r['path']:r for r in read(ROOT/c['qualification']/'images.json')}
        for s in samples[name]:
            for arm in ['rgb','thermal']:
                r=inputs[arm,s['index']];expected.append({'recording':name,'cell':s['cell'],'index':s['index'],'arm':arm,'path':r['source'],'time_ns':r['times_ns'][0],'sha256':images[r['source']]['sha256']})
    assert receipt==expected;checks+=1
    for r in receipt:assert sha(ROOT/r['path'])==r['sha256'];checks+=1
    for name,h in read(out/'render-lock.json')['figure_sha256'].items():assert sha(out/'overlays'/name)==h;checks+=1
    assert read(out/'render-lock.json')['receipts_sha256']==sha(out/'render-receipts.json');checks+=1;result={'status':'pass','checks':checks,'source_images':len(receipt),'scope':'Independent locked sample/source/time/file bindings and figure hashes; overlay primitives inherit audited frozen point/mask inputs'};(out/'render-audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':
    import sys
    overlays(Path(sys.argv[1])) if len(sys.argv)>2 and sys.argv[2]=='overlays' else main(Path(sys.argv[1]))
