"""Frozen post-match image-mask ablation, unchanged essential/pose helper."""
import argparse,json,hashlib,time
from pathlib import Path
import numpy as np,cv2
from scipy.spatial.transform import Rotation
from subt_export_comparison import Comparison
from essential_inlier_mixture import native,canonical
from smokeseer_sift import pose
from camera_priority_controls import choices
from deployment_lighting import read,write,sha
ROOT=Path(__file__).resolve().parents[1]
VARIANTS=['original','zero','bottom','combined']
POSE_FIELDS=['status','matches','E','essential_mask','cheirality_inliers','R','t','pose_mask','error']
def index_sha(indices):return hashlib.sha256(np.asarray(indices,dtype='<i8').tobytes()).hexdigest()
def image_masks(frames):
    h,w=frames[0].shape;zero=np.logical_or.reduce([f>0 for f in frames]);bottom=np.broadcast_to(np.arange(h)[:,None]<2*h/3,(h,w)).copy();return zero,bottom

def kept_indices(row,camera,mask):
    n=row.get('matches',0);keep=np.ones(n,bool);h,w=mask.shape
    for key in ['points0','points1']:
        points,_=canonical(native(np.asarray(row.get(key,[])).reshape(-1,2),camera),h,w);assert len(points)==n;pixel=np.floor(points+.5).astype(int);pixel[:,0]=pixel[:,0].clip(0,w-1);pixel[:,1]=pixel[:,1].clip(0,h-1);keep&=mask[pixel[:,1],pixel[:,0]]
    return np.flatnonzero(keep)
def exact_pose(old,new):
    expected={k:old[k] for k in POSE_FIELDS if k in old};assert set(new)==set(expected),(set(new),set(expected))
    for k,v in new.items():assert v==expected[k],k
    return len(expected)

def fit(row,k,indices):
    if not row['qualified']:return {'status':'input_unsupported','matches':0},False,0.
    a,b=[np.asarray(row.get(key,[]),float).reshape(-1,2)[indices] for key in ['points0','points1']];t=time.perf_counter();result=pose(a,b,k);return result,len(a)>=8,time.perf_counter()-t

def scored(row,refs,common,trans):
    i=row['index'];ref=refs[row['arm'],i];s={k:row[k] for k in ['recording','variant','frontend','arm','index','status']};s.update(R_error=None,t_error=None,R1=False,R5=False,joint=False)
    if i in common and row['status']=='ok':
        truth=np.asarray(ref['target_from_source']);error=float(np.degrees(Rotation.from_matrix(np.asarray(row['R'])@truth[:3,:3].T).magnitude()));s.update(R_error=error,R1=error<=1,R5=error<=5)
        if i in trans:
            a=np.asarray(row['t']);b=truth[:3,3];et=float(np.degrees(np.arccos(np.clip(a@b/(np.linalg.norm(a)*np.linalg.norm(b)),-1,1))));s.update(t_error=et,joint=bool(error<=5 and et<=15))
    return s

class Study:
    def __init__(self,config):self.config=Path(config).resolve();self.cfg=read(self.config);self.out=ROOT/self.cfg['output']
    def load(self,c):
        p=ROOT/c['matching'];return [json.loads(l) for l in (p/'predictions.jsonl').read_text().splitlines()],Comparison(ROOT/c['comparison_config'])
    def verify(self):
        for p,h in read(self.out/'input-freeze.json')['pins'].items():assert sha(ROOT/p)==h,p
    def masks(self):
        self.out.mkdir(exist_ok=False,parents=True);folder=self.out/'masks';folder.mkdir();records={};pins={}
        for name,c in self.cfg['records'].items():
            q=ROOT/c['qualification'];images=read(q/'images.json');records[name]={};assert read(q/'audit.json')['status']=='pass'
            for arm in ['rgb','thermal']:
                frames=sorted((r for r in images if r['arm']==arm),key=lambda r:r['index']);positions=np.rint(np.linspace(0,len(frames)-1,5)).astype(int);selected=[frames[j] for j in positions];arrays=[]
                for r in selected:
                    assert sha(ROOT/r['path'])==r['sha256'];a=np.load(ROOT/r['path'],allow_pickle=False);assert a.dtype==np.uint8 and a.ndim==2;arrays.append(a);pins[r['path']]=r['sha256']
                zero,bottom=image_masks(arrays)
                if arm=='thermal':zero=np.ones_like(zero)
                masks={'original':np.ones_like(zero),'zero':zero,'bottom':bottom,'combined':zero&bottom};info={}
                for variant,mask in masks.items():
                    path=folder/(name+'-'+arm+'-'+variant+'.npy');np.save(path,mask,allow_pickle=False);info[variant]={'path':str(path.relative_to(ROOT)),'sha256':sha(path),'kept_pixels':int(mask.sum()),'pixels':int(mask.size)}
                records[name][arm]={'samples':selected,'masks':info,'shape':list(zero.shape)}
        write(self.out/'mask-qualification.json',{'records':records,'sample_image_pins':pins,'recipe':'Five ordinal samples union nonzero RGB; bottom y<2h/3; endpoint-nearest pixel; coarse candidate masks, not physical/rig segmentation'});write(self.out/'mask-lock.json',{'qualification_sha256':sha(self.out/'mask-qualification.json'),'reference_errors_used':False});print('Own image masks locked',len(pins))
    def freeze(self):
        assert sha(self.out/'mask-qualification.json')==read(self.out/'mask-lock.json')['qualification_sha256'];qual=read(self.out/'mask-qualification.json');files=[self.config,Path(__file__),ROOT/'tools/smokeseer_sift.py',ROOT/'tools/subt_export_comparison.py',ROOT/'tools/essential_inlier_mixture.py',ROOT/'tools/deployment_spatial_support.py',ROOT/'tools/deployment_smoke_frontend.py',ROOT/'tools/bearing_rotation.py',ROOT/'tools/motion_compression_location.py',ROOT/'tools/camera_priority_controls.py',ROOT/'tools/smoke_rgb_first_fallback.py',ROOT/'tools/deployment_lighting.py',ROOT/'MAC-VO-main/Scripts/Setup/compare_factor_proxy.py',ROOT/'tools/audit_essential_inlier_mixture.py',ROOT/'tools/audit_pre_essential_exclusion.py',ROOT/'tools/test_pre_essential_exclusion.py',ROOT/self.cfg['protocol'],self.out/'mask-qualification.json',self.out/'mask-lock.json'];plan=[];ns={}
        for name,c in self.cfg['records'].items():
            p,q=[ROOT/c[k] for k in ['matching','qualification']];assert read(p/'audit.json')['status']==read(q/'audit.json')['status']=='pass';assert sha(p/'predictions.jsonl')==read(p/'prediction-lock.json')['predictions_sha256'];raw,model=self.load(c);n=read(p/'input-freeze.json')['anchors'];assert len(raw)==len({(r['frontend'],r['arm'],r['index']) for r in raw})==4*n;ns[name]=n
            masks={a:{v:np.load(ROOT/qual['records'][name][a]['masks'][v]['path'],allow_pickle=False) for v in VARIANTS} for a in ['rgb','thermal']}
            for r in raw:
                camera=model.camera(r['arm']);assert r['qualified'] or r['status']=='input_unsupported'
                for variant in VARIANTS:
                    indices=kept_indices(r,camera,masks[r['arm']][variant]);plan.append({'recording':name,'frontend':r['frontend'],'arm':r['arm'],'index':r['index'],'variant':variant,'input_qualified':r['qualified'],'input_matches':r.get('matches',0),'kept':len(indices),'kept_sha256':index_sha(indices),'fit_eligible':r['qualified'] and len(indices)>=8})
            files.extend(p/k for k in ['predictions.jsonl','prediction-lock.json','audit.json','matcher-inputs.json','input-freeze.json','scores.json','summary.json','completion.json']);files.extend(q/k for k in ['images.json','intervals.json','population.json','audit.json']);files.append(ROOT/c['comparison_config']);files.append(ROOT/read(ROOT/c['comparison_config'])['intrinsics'])
        for name,arms in qual['records'].items():
            for arm,d in arms.items():
                for v,m in d['masks'].items():assert sha(ROOT/m['path'])==m['sha256'];files.append(ROOT/m['path'])
        write(self.out/'preflight.json',plan);files.append(self.out/'preflight.json');pins={str(p.relative_to(ROOT)):sha(p) for p in files};pins.update(qual['sample_image_pins']);write(self.out/'input-freeze.json',{'pins':pins,'anchors':ns,'expected_rows':len(plan),'runtime_image':self.cfg['runtime_image']});print('All variants preflighted',len(plan))
    def baseline(self):
        self.verify();assert np.__version__=='2.2.6' and cv2.__version__=='5.0.0';cv2.setNumThreads(1);checks=0;total=0;calls=0
        with (self.out/'baseline.jsonl').open('x') as f:
            for name,c in self.cfg['records'].items():
                raw,model=self.load(c)
                for p in raw:
                    indices=np.arange(p.get('matches',0));result,call,seconds=fit(p,model.camera(p['arm'])[3],indices);checks+=exact_pose(p,result);row={k:p[k] for k in ['frontend','arm','index','qualified']};row.update(result,recording=name,variant='original',input_matches=p.get('matches',0),kept_sha256=index_sha(indices),solver_called=call,reused=False,reuse_variant=None,geometry_seconds=seconds);f.write(json.dumps(row,allow_nan=False)+'\n');total+=1;calls+=call
                    if total%500==0:print('Unmasked parity rows',total,flush=True)
        self.verify();write(self.out/'baseline-parity.json',{'status':'pass','rows':total,'pose_field_checks':checks,'solver_calls':calls,'baseline_sha256':sha(self.out/'baseline.jsonl'),'numpy':np.__version__,'opencv':cv2.__version__,'runtime_image':self.cfg['runtime_image']});print('Exact unmasked pose parity passed',total,checks)
    def run(self):
        self.verify();parity=read(self.out/'baseline-parity.json');assert parity['status']=='pass' and sha(self.out/'baseline.jsonl')==parity['baseline_sha256'];assert np.__version__=='2.2.6' and cv2.__version__=='5.0.0';cv2.setNumThreads(1);baseline=[json.loads(l) for l in (self.out/'baseline.jsonl').read_text().splitlines()];original={(r['recording'],r['frontend'],r['arm'],r['index']):r for r in baseline};plan={(r['recording'],r['variant'],r['frontend'],r['arm'],r['index']):r for r in read(self.out/'preflight.json')};qual=read(self.out/'mask-qualification.json');total=0;calls=0;reused=0
        with (self.out/'predictions.jsonl').open('x') as f:
            for row in baseline:f.write(json.dumps(row,allow_nan=False)+'\n');total+=1
            for name,c in self.cfg['records'].items():
                raw,model=self.load(c);masks={a:{v:np.load(ROOT/qual['records'][name][a]['masks'][v]['path'],allow_pickle=False) for v in VARIANTS} for a in ['rgb','thermal']}
                for p in raw:
                    key=name,p['frontend'],p['arm'],p['index'];old=original[key];cache={old['kept_sha256']:('original',{k:old[k] for k in POSE_FIELDS if k in old})}
                    for variant in VARIANTS[1:]:
                        indices=kept_indices(p,model.camera(p['arm']),masks[p['arm']][variant]);h=index_sha(indices);assert h==plan[name,variant,p['frontend'],p['arm'],p['index']]['kept_sha256']
                        if h in cache:previous,result=cache[h];call=False;seconds=0.;reuse=True;reused+=1
                        else:result,call,seconds=fit(p,model.camera(p['arm'])[3],indices);previous=None;reuse=False;cache[h]=(variant,result)
                        row={k:p[k] for k in ['frontend','arm','index','qualified']};row.update(result,recording=name,variant=variant,input_matches=p.get('matches',0),kept_sha256=h,solver_called=call,reused=reuse,reuse_variant=previous,geometry_seconds=seconds);f.write(json.dumps(row,allow_nan=False)+'\n');total+=1;calls+=call
                    if total%500==0:print('Full geometry rows',total,flush=True)
        assert total==read(self.out/'input-freeze.json')['expected_rows'];self.verify();write(self.out/'prediction-lock.json',{'rows':total,'predictions_sha256':sha(self.out/'predictions.jsonl'),'baseline_parity_sha256':sha(self.out/'baseline-parity.json'),'additional_solver_calls':calls,'reused_rows':reused,'reference_values_parsed':False,'new_learned_inference_calls':0});print('Geometry predictions locked',total)
    def select(self):
        self.verify();assert sha(self.out/'predictions.jsonl')==read(self.out/'prediction-lock.json')['predictions_sha256'];raw=[json.loads(l) for l in (self.out/'predictions.jsonl').read_text().splitlines()];result=[]
        for name,c in self.cfg['records'].items():
            n=read(self.out/'input-freeze.json')['anchors'][name];inputs=read(ROOT/c['matching']/'matcher-inputs.json')
            for variant in VARIANTS:
                picked=choices([r for r in raw if r['recording']==name and r['variant']==variant],inputs,n);result.extend(dict(r,recording=name,variant=variant) for r in picked)
        write(self.out/'choices.json',result);write(self.out/'choice-lock.json',{'rows':len(result),'choices_sha256':sha(self.out/'choices.json'),'reference_errors_used':False});print('Choices locked',len(result))
    def score(self):
        self.verify();assert sha(self.out/'predictions.jsonl')==read(self.out/'prediction-lock.json')['predictions_sha256'] and sha(self.out/'choices.json')==read(self.out/'choice-lock.json')['choices_sha256'];raw=[json.loads(l) for l in (self.out/'predictions.jsonl').read_text().splitlines()];selected=read(self.out/'choices.json');all_scores=[];summary={};transitions={}
        for name,c in self.cfg['records'].items():
            q=ROOT/c['qualification'];refs={(r['arm'],r['index']):r for r in read(q/'intervals.json')};pop=read(q/'population.json');common=set(pop['common_reference']);trans=set(pop['translation_ge1cm_both']);n=read(self.out/'input-freeze.json')['anchors'][name];scores=[scored(r,refs,common,trans) for r in raw if r['recording']==name];all_scores.extend(scores);smap={(r['variant'],r['frontend'],r['arm'],r['index']):r for r in scores};dmap={(r['variant'],r['frontend'],r['index']):r for r in selected if r['recording']==name};groups={'all_declared':set(range(n)),'common_reference':common,'paired_rotation_ge2deg':set(pop['paired_rotation_ge2deg']),'translation_ge1cm_both':trans,**{'third'+str(j+1):{i for i in common if i*3//n==j} for j in range(3)}};summary[name]={};transitions[name]={}
            for variant in VARIANTS:
                summary[name][variant]={};transitions[name][variant]={}
                for front in ['sift','learned']:
                    summary[name][variant][front]={};transitions[name][variant][front]={}
                    for group,ids in groups.items():
                        counts={};delta={}
                        for policy in ['rgb','thermal','rgb_first','thermal_first']:
                            def pick(v,i):
                                a=dmap[v,front,i]['policies'][policy]['sensor'];return smap[v,front,a,i] if a else None
                            pairs=[(pick('original',i),pick(variant,i),i) for i in sorted(ids)];r={'anchors':len(ids),'outputs':sum(b is not None for _,b,_ in pairs),'rotation_eligible':len(ids&common),'translation_eligible':len(ids&trans)};t={'new_outputs':sum(a is None and b is not None for a,b,_ in pairs),'lost_outputs':sum(a is not None and b is None for a,b,_ in pairs)}
                            for endpoint,eligible in [('R1',common),('R5',common),('joint',trans)]:
                                pp=[(bool(a and a[endpoint]),bool(b and b[endpoint])) for a,b,i in pairs if i in eligible];r[endpoint]=sum(b for a,b in pp);t[endpoint]={'eligible':len(pp),'gained':sum(not a and b for a,b in pp),'lost':sum(a and not b for a,b in pp),'retained_correct':sum(a and b for a,b in pp)}
                            counts[policy]=r;delta[policy]=t
                        summary[name][variant][front][group]=counts;transitions[name][variant][front][group]=delta
            old=read(ROOT/c['matching']/'summary.json')
            for front in ['sift','learned']:
                for group in groups:
                    for policy in ['rgb','thermal','rgb_first']:
                        for key in ['anchors','outputs','rotation_eligible','translation_eligible','R1','R5','joint']:assert summary[name]['original'][front][group][policy][key]==old[front][group][policy][key]
        write(self.out/'scores.json',all_scores);write(self.out/'summary.json',summary);write(self.out/'transitions.json',transitions);self.verify();write(self.out/'completion.json',{'scores_sha256':sha(self.out/'scores.json'),'summary_sha256':sha(self.out/'summary.json'),'transitions_sha256':sha(self.out/'transitions.json'),'scope':'Nominal supplied-reference exposed engineering ablation; no physical truth, end-to-end latency or deployment adoption'});print('Scores and transitions complete',len(all_scores))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['masks','freeze','baseline','run','select','score']);p.add_argument('config',type=Path);a=p.parse_args();getattr(Study(a.config),a.stage)()
