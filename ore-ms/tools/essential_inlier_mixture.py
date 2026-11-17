"""Fixed correspondence-mask motion/spatial diagnosis, no refit or selector."""
import argparse,json
from pathlib import Path
import numpy as np,cv2
from bearing_rotation import bearings
from subt_export_comparison import Comparison
from motion_compression_location import angles
from deployment_spatial_support import spatial,canonical
from deployment_lighting import read,write,sha
ROOT=Path(__file__).resolve().parents[1]
MODES=['accepted','essential','nonessential','cheirality','essential_not_cheirality']
CUT=.1

def native(points,camera):
    points=np.asarray(points,float).reshape(-1,2)
    if not len(points):return points
    k,d,xi,v=camera;rays=np.column_stack([(points-v[:2,2])/400,np.ones(len(points))]);den=rays[:,2]+xi*np.linalg.norm(rays,axis=1);xy=rays[:,:2]/den[:,None];p,_=cv2.projectPoints(np.column_stack([xy,np.ones(len(xy))]),np.zeros(3),np.zeros(3),k,d);return p.reshape(-1,2)
def divide(a,b):return a/b if b else None

def feature(p,camera,h,w):
    n=p.get('matches',0);points=[np.asarray(p.get(k,[]),float).reshape(-1,2) for k in ['points0','points1']];assert all(len(v)==n for v in points);a,b=[bearings(v,camera[3])[0] for v in points];motion=angles(a,b);low=motion<=CUT;em=np.asarray(p.get('essential_mask',[0]*n)).reshape(-1)!=0;pm=np.asarray(p.get('pose_mask',[0]*n)).reshape(-1)!=0;assert len(em)==len(pm)==n and not np.any(pm&~em);assert pm.sum()==p.get('cheirality_inliers',0);masks=dict(zip(MODES,[np.ones(n,bool),em,~em,pm,em&~pm]));pp=[canonical(native(v,camera),h,w)[0] for v in points];result={k:p[k] for k in ['frontend','arm','index','status']};result['masks']={}
    for mode,mask in masks.items():
        values=motion[mask];result['masks'][mode]={'count':int(mask.sum()),'low_count':int((mask&low).sum()),'low_fraction':divide(int((mask&low).sum()),int(mask.sum())),'angle_quartiles':np.percentile(values,[25,50,75]).tolist() if len(values) else None,'source':spatial(pp[0][mask],h,w),'target':spatial(pp[1][mask],h,w)}
    nl=int(low.sum());ne=int((em&low).sum());nh=n-nl;nhe=int((em&~low).sum());result['retention']={'accepted_low':nl,'accepted_high':nh,'essential_low':ne,'essential_high':nhe,'low_retention':divide(ne,nl),'high_retention':divide(nhe,nh)};return result

def aggregate(rows):
    def summarize(v):
        good=[x for x in v if x is not None];return {'n':len(good),'missing':len(v)-len(good),'quartiles':np.percentile(good,[25,50,75]).tolist() if good else None}
    d={'anchors':len(rows),'outputs':sum(r['status']=='ok' for r in rows),'masks':{},'retention':{k:summarize([r['retention'][k] for r in rows]) for k in ['low_retention','high_retention']}}
    for mode in MODES:
        d['masks'][mode]={k:summarize([r['masks'][mode][k] for r in rows]) for k in ['count','low_count','low_fraction']}
        for side in ['source','target']:
            d['masks'][mode][side]={k:summarize([r['masks'][mode][side][k] for r in rows]) for k in ['occupied_cells','hull_fraction','entropy','top4_fraction']}
        d['masks'][mode]['angle_median']=summarize([r['masks'][mode]['angle_quartiles'][1] if r['masks'][mode]['angle_quartiles'] else None for r in rows])
    return d

class Study:
    def __init__(self,config):self.config=Path(config).resolve();self.cfg=read(self.config);self.out=ROOT/self.cfg['output']
    def verify(self):
        for p,h in read(self.out/'input-freeze.json')['pins'].items():assert sha(ROOT/p)==h,p
    def load(self,c):
        model=Comparison(ROOT/c['comparison_config']);cams={a:model.camera(a) for a in ['rgb','thermal']};raw=[json.loads(l) for l in (ROOT/c['matching']/'predictions.jsonl').read_text().splitlines()];pop=read(ROOT/c['qualification']/'population.json');return model,cams,raw,pop
    def freeze(self):
        files=[self.config,Path(__file__),ROOT/'tools/deployment_spatial_support.py',ROOT/'tools/deployment_smoke_frontend.py',ROOT/'tools/bearing_rotation.py',ROOT/'tools/motion_compression_location.py',ROOT/'tools/subt_export_comparison.py',ROOT/'tools/deployment_lighting.py',ROOT/'tools/audit_essential_inlier_mixture.py',ROOT/'tools/test_essential_inlier_mixture.py',ROOT/self.cfg['protocol']];reports=[];ns={}
        for name,c in self.cfg['records'].items():
            p,q=[ROOT/c[k] for k in ['matching','qualification']];assert read(p/'audit.json')['status']==read(q/'audit.json')['status']=='pass';assert sha(p/'predictions.jsonl')==read(p/'prediction-lock.json')['predictions_sha256'];model,cams,raw,pop=self.load(c);n=read(p/'input-freeze.json')['anchors'];assert len(raw)==len({(r['frontend'],r['arm'],r['index']) for r in raw})==4*n;ns[name]=len(pop['paired_rotation_ge2deg']);ids=set(pop['paired_rotation_ge2deg'])
            for arm,cam in cams.items():
                h,w=model.cameras[arm+'_camera']['image_height'],model.cameras[arm+'_camera']['image_width'];grid=np.array([[0,0],[w-1,0],[0,h-1],[w-1,h-1],[w/2,h/2]],float);error=float(np.linalg.norm(native(model.lift(grid,arm),cam)-grid,axis=1).max());assert error<=.05;reports.append({'recording':name,'arm':arm,'roundtrip_max_px':error})
            for r in raw:
                if r['index'] in ids:
                    cam=model.cameras[r['arm']+'_camera'];feature(r,cams[r['arm']],cam['image_height'],cam['image_width'])
            files.extend(p/k for k in ['predictions.jsonl','prediction-lock.json','audit.json','matcher-inputs.json']);files.extend(q/k for k in ['population.json','images.json','audit.json']);files.append(ROOT/c['comparison_config']);files.append(ROOT/read(ROOT/c['comparison_config'])['intrinsics'])
        prior=ROOT/self.cfg['strata'];assert read(prior/'audit.json')['status']=='pass' and sha(prior/'samples.json')==read(prior/'sample-lock.json')['samples_sha256'];files.extend(prior/k for k in ['cases.json','samples.json','sample-lock.json','completion.json','audit.json']);loc=ROOT/self.cfg['location'];assert read(loc/'audit.json')['status']=='pass';files.extend(loc/k for k in ['rows.json','completion.json','audit.json']);self.out.mkdir(parents=True,exist_ok=False);write(self.out/'input-freeze.json',{'pins':{str(p.relative_to(ROOT)):sha(p) for p in files},'moving_anchors':ns,'low_motion_ray_deg':CUT,'projection_controls':reports})
    def extract(self):
        self.verify();allrows={}
        for name,c in self.cfg['records'].items():
            model,cams,raw,pop=self.load(c);ids=set(pop['paired_rotation_ge2deg']);rows=[]
            for r in raw:
                if r['index'] in ids:
                    cam=model.cameras[r['arm']+'_camera'];rows.append(feature(r,cams[r['arm']],cam['image_height'],cam['image_width']))
            allrows[name]=rows
        write(self.out/'features.json',allrows);write(self.out/'feature-lock.json',{'features_sha256':sha(self.out/'features.json'),'reference_values_parsed':False});print('Feature rows locked',sum(len(v) for v in allrows.values()))
    def join(self):
        self.verify();assert sha(self.out/'features.json')==read(self.out/'feature-lock.json')['features_sha256'];rows=read(self.out/'features.json');cases=read(ROOT/self.cfg['strata']/'cases.json');samples=read(ROOT/self.cfg['strata']/'samples.json');location=read(ROOT/self.cfg['location']/'rows.json');summary={};traces={}
        for name,rr in rows.items():
            summary[name]={f:{a:aggregate([r for r in rr if r['frontend']==f and r['arm']==a]) for a in ['rgb','thermal']} for f in ['sift','learned']};summary[name]['learned_R5_strata']={}
            for cell in ['both_correct','rgb_only_correct','thermal_only_correct','neither_correct']:
                ids={r['index'] for r in cases[name] if r['endpoint']=='R5' and r['cell']==cell};summary[name]['learned_R5_strata'][cell]={a:aggregate([r for r in rr if r['frontend']=='learned' and r['arm']==a and r['index'] in ids]) for a in ['rgb','thermal']}
            ids={r['index'] for r in samples[name]};traces[name]=[r for r in rr if r['frontend']=='learned' and r['index'] in ids]
            old={(r['frontend'],r['arm'],r['index']):r for r in location[name]}
            for r in rr:
                o=old[r['frontend'],r['arm'],r['index']]
                for mode,new in [('accepted','accepted'),('essential','essential'),('cheirality','cheirality')]:
                    stat=r['masks'][new];assert stat['count']==o['subsets'][mode]['points'];v=stat['angle_quartiles'];assert v is None if o['subsets'][mode]['ray_change']['median'] is None else abs(v[1]-o['subsets'][mode]['ray_change']['median'])<1e-9
        write(self.out/'summary.json',summary);write(self.out/'sample-features.json',traces);write(self.out/'samples.json',samples);write(self.out/'completion.json',{'summary_sha256':sha(self.out/'summary.json'),'sample_features_sha256':sha(self.out/'sample-features.json'),'samples_sha256':sha(self.out/'samples.json'),'new_inference_calls':0,'new_mask_or_pose_calls':0});print('Prior sample sensor rows',sum(len(v) for v in traces.values()))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['freeze','extract','join']);p.add_argument('config',type=Path);a=p.parse_args();getattr(Study(a.config),a.stage)()
