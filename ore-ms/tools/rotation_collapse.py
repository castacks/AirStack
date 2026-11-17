"""Cached supplied-reference rotation-amount diagnostic; no routing or inference."""
import argparse,json
from pathlib import Path
import numpy as np
from reference_contract_angles import degrees
from deployment_lighting import read,write,sha
ROOT=Path(__file__).resolve().parents[1]
CUT=.1

def angle_row(p):
    angle=None
    if p['status']=='ok':
        r=np.asarray(p['R']);assert r.shape==(3,3) and np.isfinite(r).all();assert np.max(np.abs(r.T@r-np.eye(3)))<=1e-5 and abs(np.linalg.det(r)-1)<=1e-5;angle=degrees(r)
    return {k:p[k] for k in ['frontend','arm','index','status']}|{'estimated_deg':angle,'near_identity':angle is not None and angle<=CUT}

def join_row(a,s):
    for key in ['frontend','arm','index','status']:assert a[key]==s[key]
    e=a['estimated_deg'];truth=s['reference_rotation_deg'];error=s['R_error'];r=dict(a,reference_deg=truth,error_deg=error,R1=s['R1'],R5=s['R5'],motion_ratio=e/truth if e is not None and truth is not None and truth>=2 else None,error_minus_identity_deg=error-truth if error is not None else None)
    if error is not None:assert e is not None and truth is not None and abs(e-truth)<=error+1e-5 and error<=e+truth+1e-5
    return r

def stats(rows,eligible):
    compared=[r for r in rows if r['error_deg'] is not None];out=[r for r in rows if r['estimated_deg'] is not None];d={'anchors':len(rows),'outputs':len(out),'reference_eligible':sum(r['index'] in eligible for r in rows),'compared_outputs':len(compared),'near_identity_outputs':sum(r['near_identity'] for r in out)}
    for key,t in [('R1',1),('R5',5)]:
        d[key+'_correct']=sum(r[key] for r in compared);d[key+'_wrong_supported']=sum(not r[key] for r in compared);d[key+'_wrong_near_identity']=sum(not r[key] and r['near_identity'] for r in compared);d['identity_'+key+'_correct']=sum(r['reference_deg'] is not None and r['reference_deg']<=t for r in rows if r['index'] in eligible)
    for field in ['estimated_deg','reference_deg','error_deg','motion_ratio','error_minus_identity_deg']:
        values=[r[field] for r in (compared if field=='reference_deg' else rows) if r[field] is not None];d[field]={'n':len(values),'quartiles':np.percentile(values,[25,50,75]).tolist() if values else None}
    return d

class Study:
    def __init__(self,config):self.config=Path(config).resolve();self.cfg=read(self.config);self.out=ROOT/self.cfg['output']
    def verify(self):
        for p,h in read(self.out/'input-freeze.json')['pins'].items():assert sha(ROOT/p)==h,p
    def freeze(self):
        files=[self.config,Path(__file__),ROOT/'tools/reference_contract_angles.py',ROOT/'tools/deployment_lighting.py',ROOT/'tools/audit_rotation_collapse.py',ROOT/'tools/test_rotation_collapse.py',ROOT/self.cfg['protocol']];ns={}
        for name,c in self.cfg['records'].items():
            p,q=[ROOT/c[k] for k in ['matching','qualification']];assert read(p/'audit.json')['status']==read(q/'audit.json')['status']=='pass';assert sha(p/'predictions.jsonl')==read(p/'prediction-lock.json')['predictions_sha256'] and sha(p/'scores.json')==read(p/'completion.json')['scores_sha256'];n=read(p/'input-freeze.json')['anchors'];ns[name]=n;seen=set()
            for line in (p/'predictions.jsonl').read_text().splitlines():
                r=json.loads(line);key=(r['frontend'],r['arm'],r['index']);assert key not in seen;seen.add(key);angle_row(r)
            assert seen=={(f,a,i) for f in ['sift','learned'] for a in ['rgb','thermal'] for i in range(n)}
            files.extend(p/f for f in ['predictions.jsonl','prediction-lock.json','scores.json','completion.json','audit.json']);files.extend(q/f for f in ['population.json','audit.json'])
        strata=ROOT/self.cfg['strata'];assert read(strata/'audit.json')['status']=='pass' and sha(strata/'cases.json')==read(strata/'completion.json')['cases_sha256'];files.extend(strata/f for f in ['cases.json','completion.json','audit.json']);self.out.mkdir(parents=True,exist_ok=False);write(self.out/'input-freeze.json',{'pins':{str(p.relative_to(ROOT)):sha(p) for p in files},'anchors':ns,'near_identity_deg':CUT,'scope':'Known supplied-reference development data; no selection/physical truth'})
    def extract(self):
        self.verify();result={}
        for name,c in self.cfg['records'].items():result[name]=[angle_row(json.loads(l)) for l in (ROOT/c['matching']/'predictions.jsonl').read_text().splitlines()]
        write(self.out/'angles.json',result);write(self.out/'angle-lock.json',{'angles_sha256':sha(self.out/'angles.json'),'reference_values_parsed':False});print('Angles locked',sum(len(v) for v in result.values()))
    def score(self):
        self.verify();assert sha(self.out/'angles.json')==read(self.out/'angle-lock.json')['angles_sha256'];angles=read(self.out/'angles.json');cases=read(ROOT/self.cfg['strata']/'cases.json');allrows={};summary={}
        for name,c in self.cfg['records'].items():
            p,q=[ROOT/c[k] for k in ['matching','qualification']];scores={(s['frontend'],s['arm'],s['index']):s for s in read(p/'scores.json')};rows=[join_row(r,scores[r['frontend'],r['arm'],r['index']]) for r in angles[name]];allrows[name]=rows;pop=read(q/'population.json');common=set(pop['common_reference']);n=read(self.out/'input-freeze.json')['anchors'][name];groups={'all_declared':set(range(n)),'common_reference':common,'paired_rotation_ge2deg':set(pop['paired_rotation_ge2deg'])};summary[name]={}
            for f in ['sift','learned']:
                summary[name][f]={a:{g:stats([r for r in rows if r['frontend']==f and r['arm']==a and r['index'] in ids],common) for g,ids in groups.items()} for a in ['rgb','thermal']}
            summary[name]['learned_R5_strata']={}
            for cell in ['both_correct','rgb_only_correct','thermal_only_correct','neither_correct']:
                ids={r['index'] for r in cases[name] if r['endpoint']=='R5' and r['cell']==cell};summary[name]['learned_R5_strata'][cell]={a:stats([r for r in rows if r['frontend']=='learned' and r['arm']==a and r['index'] in ids],common) for a in ['rgb','thermal']}
        write(self.out/'rows.json',allrows);write(self.out/'summary.json',summary);self.verify();write(self.out/'completion.json',{'rows_sha256':sha(self.out/'rows.json'),'summary_sha256':sha(self.out/'summary.json'),'new_inference_calls':0,'selector_changed':False});print('Angle/reference rows complete',sum(len(v) for v in allrows.values()))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['freeze','extract','score']);p.add_argument('config',type=Path);a=p.parse_args();getattr(Study(a.config),a.stage)()
