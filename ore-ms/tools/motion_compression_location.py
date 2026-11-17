"""Inspect fixed matched rays/E branches; no new matching or pose estimation."""
import argparse,json
from pathlib import Path
import cv2,numpy as np
from bearing_rotation import bearings
from reference_contract_angles import degrees
from subt_export_comparison import Comparison
from deployment_lighting import read,write,sha
ROOT=Path(__file__).resolve().parents[1]
MODES=['accepted','essential','cheirality']
METRICS=['ray_change','estimated_warp','estimated_epipolar','reference_warp','reference_epipolar']
def skew(t):
    x,y,z=t;return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
def angles(a,b):return np.degrees(np.arccos(np.clip(np.sum(a*b,axis=1),-1,1)))
def plane(a,b,e):
    normal=a@e.T;norm=np.linalg.norm(normal,axis=1);valid=norm>1e-12;return np.degrees(np.arcsin(np.clip(abs(np.sum(b[valid]*normal[valid],axis=1))/norm[valid],0,1)))
def values(v):return {'n':len(v),'median':float(np.median(v)) if len(v) else None,'p90':float(np.percentile(v,90)) if len(v) else None}
def pair(p,k,truth=None,translation_eligible=False):
    p0,p1=[np.asarray(p.get(key,[]),float).reshape(-1,2) for key in ['points0','points1']];assert len(p0)==len(p1)==p.get('matches',0);a=bearings(p0,k)[0];b=bearings(p1,k)[0];n=len(a);masks={'accepted':np.ones(n,bool),'essential':np.array(p.get('essential_mask',[0]*n)).reshape(-1)!=0,'cheirality':np.array(p.get('pose_mask',[0]*n)).reshape(-1)!=0};assert all(len(m)==n for m in masks.values());assert int(masks['cheirality'].sum())==p.get('cheirality_inliers',0)
    e=np.asarray(p.get('E',[]));valid_e=e.shape==(3,3) and np.isfinite(e).all();supported=p['status']=='ok';rot=np.asarray(p['R']) if supported else None;r={'frontend':p['frontend'],'arm':p['arm'],'index':p['index'],'status':p['status'],'E_valid':bool(valid_e),'candidates':[],'subsets':{}}
    if valid_e:
        r1,r2,_=cv2.decomposeEssentialMat(e);r['candidates']=[{'R':rr.tolist(),'angle_deg':degrees(rr)} for rr in [r1,r2]]
        if supported:assert min(degrees(rot@rr.T) for rr in [r1,r2])<1e-4
    for mode,mask in masks.items():
        aa,bb=a[mask],b[mask];d={'points':len(aa),'ray_change':values(angles(aa,bb)),'estimated_warp':values(angles(aa@rot.T,bb)) if supported else values([]),'estimated_epipolar':values(plane(aa,bb,e)) if valid_e else values([])}
        if truth is not None:
            d.update(reference_warp=values(angles(aa@truth[:3,:3].T,bb)),reference_epipolar=values(plane(aa,bb,skew(truth[:3,3])@truth[:3,:3])) if translation_eligible else values([]))
        r['subsets'][mode]=d
    if truth is not None:
        for c in r['candidates']:c['error_deg']=degrees(np.asarray(c['R'])@truth[:3,:3].T)
        r['reference_angle_deg']=degrees(truth[:3,:3]);r['output_error_deg']=degrees(rot@truth[:3,:3].T) if supported else None;r['translation_eligible']=translation_eligible
    return r

def aggregate(rows):
    d={'anchors':len(rows),'outputs':sum(r['status']=='ok' for r in rows),'E_available':sum(r['E_valid'] for r in rows),'subsets':{}}
    for t in [1,5]:
        d['R'+str(t)]={'original_correct':sum(r['output_error_deg'] is not None and r['output_error_deg']<=t for r in rows),'either_candidate_correct':sum(any(c['error_deg']<=t for c in r['candidates']) for r in rows),'wrong_output_alternative_correct':sum(r['output_error_deg'] is not None and r['output_error_deg']>t and any(c['error_deg']<=t for c in r['candidates']) for r in rows)}
    for mode in MODES:
        d['subsets'][mode]={}
        for field in METRICS:
            v=[r['subsets'][mode][field]['median'] for r in rows if r['subsets'][mode][field]['median'] is not None];d['subsets'][mode][field]={'n':len(v),'missing':len(rows)-len(v),'quartiles':np.percentile(v,[25,50,75]).tolist() if v else None}
    return d

class Study:
    def __init__(self,config):self.config=Path(config).resolve();self.cfg=read(self.config);self.out=ROOT/self.cfg['output']
    def verify(self):
        for p,h in read(self.out/'input-freeze.json')['pins'].items():assert sha(ROOT/p)==h,p
    def load(self,c):
        parent=ROOT/c['matching'];q=ROOT/c['qualification'];model=Comparison(ROOT/c['comparison_config']);pop=read(q/'population.json');raw=[json.loads(l) for l in (parent/'predictions.jsonl').read_text().splitlines()];return raw,{a:model.camera(a)[3] for a in ['rgb','thermal']},pop
    def freeze(self):
        files=[self.config,Path(__file__),ROOT/'tools/bearing_rotation.py',ROOT/'tools/reference_contract_angles.py',ROOT/'tools/subt_export_comparison.py',ROOT/'tools/deployment_lighting.py',ROOT/'tools/audit_motion_compression_location.py',ROOT/'tools/test_motion_compression_location.py',ROOT/self.cfg['protocol']];counts={}
        for name,c in self.cfg['records'].items():
            p,q=[ROOT/c[k] for k in ['matching','qualification']];assert read(p/'audit.json')['status']==read(q/'audit.json')['status']=='pass';assert sha(p/'predictions.jsonl')==read(p/'prediction-lock.json')['predictions_sha256'];raw,k,pop=self.load(c);n=read(p/'input-freeze.json')['anchors'];assert len(raw)==len({(r['frontend'],r['arm'],r['index']) for r in raw})==4*n;ids=set(pop['paired_rotation_ge2deg']);refs={(r['arm'],r['index']):r for r in read(q/'intervals.json')}
            for r in raw:
                if r['index'] in ids:
                    ref=refs[r['arm'],r['index']];assert ref['reference_supported'] and np.asarray(ref['target_from_source']).shape==(4,4);pair(r,k[r['arm']])
            counts[name]=len(ids);files.extend(p/f for f in ['predictions.jsonl','prediction-lock.json','scores.json','completion.json','audit.json','matcher-inputs.json']);files.extend(q/f for f in ['population.json','intervals.json','audit.json','calibration.json']);files.append(ROOT/c['comparison_config']);cfg=read(ROOT/c['comparison_config']);files.append(ROOT/cfg['intrinsics'])
        prior=ROOT/self.cfg['strata'];assert read(prior/'audit.json')['status']=='pass';files.extend(prior/f for f in ['cases.json','completion.json','audit.json']);self.out.mkdir(exist_ok=False,parents=True);write(self.out/'input-freeze.json',{'pins':{str(p.relative_to(ROOT)):sha(p) for p in files},'moving_anchors':counts,'numpy':np.__version__,'opencv':cv2.__version__})
    def extract(self):
        self.verify();allrows={}
        for name,c in self.cfg['records'].items():
            raw,k,pop=self.load(c);ids=set(pop['paired_rotation_ge2deg']);allrows[name]=[pair(r,k[r['arm']]) for r in raw if r['index'] in ids]
        write(self.out/'features.json',allrows);write(self.out/'feature-lock.json',{'features_sha256':sha(self.out/'features.json'),'reference_values_parsed':False});print('Reference-free rows locked',sum(len(v) for v in allrows.values()))
    def score(self):
        self.verify();assert sha(self.out/'features.json')==read(self.out/'feature-lock.json')['features_sha256'];features=read(self.out/'features.json');cases=read(ROOT/self.cfg['strata']/'cases.json');allrows={};summary={}
        for name,c in self.cfg['records'].items():
            raw,k,pop=self.load(c);ids=set(pop['paired_rotation_ge2deg']);trans=set(pop['translation_ge1cm_both']);refs={(r['arm'],r['index']):r for r in read(ROOT/c['qualification']/'intervals.json')};scores={(r['frontend'],r['arm'],r['index']):r for r in read(ROOT/c['matching']/'scores.json')};rows=[pair(r,k[r['arm']],np.asarray(refs[r['arm'],r['index']]['target_from_source']),r['index'] in trans) for r in raw if r['index'] in ids]
            for r,f in zip(rows,features[name]):
                for key in ['frontend','arm','index','status','E_valid']:assert r[key]==f[key]
                for mode in MODES:
                    for key in ['points','ray_change','estimated_warp','estimated_epipolar']:assert r['subsets'][mode][key]==f['subsets'][mode][key]
                for a,b in zip(r['candidates'],f['candidates']):assert a['R']==b['R'] and a['angle_deg']==b['angle_deg']
                old=scores[r['frontend'],r['arm'],r['index']]['R_error'];assert r['output_error_deg'] is None if old is None else abs(r['output_error_deg']-old)<1e-6
            allrows[name]=rows;summary[name]={f:{a:aggregate([r for r in rows if r['frontend']==f and r['arm']==a]) for a in ['rgb','thermal']} for f in ['sift','learned']};summary[name]['learned_R5_strata']={}
            for cell in ['both_correct','rgb_only_correct','thermal_only_correct','neither_correct']:
                keep={r['index'] for r in cases[name] if r['endpoint']=='R5' and r['cell']==cell};summary[name]['learned_R5_strata'][cell]={a:aggregate([r for r in rows if r['frontend']=='learned' and r['arm']==a and r['index'] in keep]) for a in ['rgb','thermal']}
        write(self.out/'rows.json',allrows);write(self.out/'summary.json',summary);self.verify();write(self.out/'completion.json',{'rows_sha256':sha(self.out/'rows.json'),'summary_sha256':sha(self.out/'summary.json'),'new_inference_calls':0,'new_E_or_pose_fit_calls':0});print('Nominal-reference rows complete',sum(len(v) for v in allrows.values()))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['freeze','extract','score']);p.add_argument('config',type=Path);a=p.parse_args();getattr(Study(a.config),a.stage)()
