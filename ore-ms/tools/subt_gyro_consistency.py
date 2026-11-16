"""Nominal raw-gyro angular agreement of locked, camera-tagged SubT proposals.

Distinct from pose-reference scoring: no inference, bias/clock fitting or metric
translation. Left-held body gyro yields source-from-target; proposals use its inverse.
"""
import argparse,json,math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from deployment_lighting import read,write,sha
ROOT=Path(__file__).resolve().parents[1]

def integrate_gyro(times,gyro,t0,t1,max_gap_ns=20_000_000):
    if t1<=t0:return {'supported':False,'reason':'nonpositive_interval'}
    left=int(np.searchsorted(times,t0,side='right')-1);right=int(np.searchsorted(times,t1,side='left'))
    if left<0 or right>=len(times):return {'supported':False,'reason':'outside_imu_support'}
    gap=int(np.diff(times[left:right+1]).max())
    if gap>max_gap_ns:return {'supported':False,'reason':'imu_gap','max_gap_ns':gap}
    boundaries=[t0]+[int(t) for t in times[left+1:right] if t0<int(t)<t1]+[t1];r=np.eye(3)
    for a,b in zip(boundaries,boundaries[1:]):
        j=int(np.searchsorted(times,a,side='right')-1);r=r@Rotation.from_rotvec(gyro[j]*((b-a)*1e-9)).as_matrix()
    target=r.T
    return {'supported':True,'max_gap_ns':gap,'target_from_source_imu':target.tolist(),'rotation_deg':float(np.degrees(Rotation.from_matrix(target).magnitude()))}

def summarize(rows,decisions,labels,population,n):
    values={(r['frontend'],r['arm'],r['index']):r for r in rows};refs={(r['arm'],r['index']):r for r in labels};common=set(population['common_gyro']);moving=set(population['paired_rotation_ge2deg']);groups={'all_declared':set(range(n)),'common_gyro':common,'paired_rotation_ge2deg':moving,**{'third'+str(j+1):{i for i in common if i*3//n==j} for j in range(3)}};summary={}
    for f in ['sift','learned']:
        summary[f]={}
        for name,ids in groups.items():
            eligible=ids&common;group=[d for d in decisions if d['frontend']==f and d['index'] in ids];record={}
            for policy in ['rgb','thermal','rgb_first']:
                errors=[values[f,d['policies'][policy],d['index']]['disagreement_deg'] for d in group if d['index'] in eligible and d['policies'][policy]]
                assert all(e is not None for e in errors)
                record[policy]={'anchors':len(ids),'outputs':sum(d['policies'][policy] is not None for d in group),'common_gyro_eligible':len(eligible),'compared_outputs':len(errors),'missing_outputs_on_common':len(eligible)-len(errors),'within1deg':sum(e<=1 for e in errors),'within5deg':sum(e<=5 for e in errors),'conditional_median_deg':float(np.median(errors)) if errors else None,'pose_accuracy':None}
            added=[d for d in group if d['policies']['rgb_first']=='thermal'];valid=[d for d in added if d['index'] in eligible];ee=[values[f,'thermal',d['index']]['disagreement_deg'] for d in valid];zero=[refs['thermal',d['index']]['rotation_deg'] for d in valid]
            record['added_thermal']={'outputs':len(added),'gyro_eligible':len(valid),'within1deg':sum(e<=1 for e in ee),'within5deg':sum(e<=5 for e in ee),'zero_within1deg':sum(e<=1 for e in zero),'zero_within5deg':sum(e<=5 for e in zero),'conditional_median_deg':float(np.median(ee)) if ee else None}
            record['zero_rotation']={a:{str(t):sum(refs[a,i]['rotation_deg']<=t for i in eligible) for t in [1,5]} for a in ['rgb','thermal']};summary[f][name]=record
    return {'scope':'Nominal gyro angular agreement, not independent pose accuracy or translation truth','pose_accuracy':None,'frontends':summary}

class Diagnostic:
    def __init__(self,config):
        self.config=Path(config).resolve();self.cfg=read(self.config);self.out=ROOT/self.cfg['output'];self.parent=ROOT/self.cfg['matching'];self.qual=ROOT/self.cfg['qualification'];self.raw=ROOT/self.cfg['raw_binding']
    def verify(self):
        for p,h in read(self.out/'target-lock.json')['pins'].items():assert sha(ROOT/p)==h,p
    def freeze(self):
        self.out.mkdir(parents=True,exist_ok=False);assert read(self.raw/'qualification.json')['status']=='pass';assert read(self.raw/'independent-imu-audit.json')['status']=='pass';pins={}
        for p,h in read(self.raw/'completion.json')['pins'].items():assert sha(ROOT/p)==h,p;pins[p]=h
        for p,h in read(self.parent/'execution-freeze.json')['pins'].items():assert sha(ROOT/p)==h,p;pins[p]=h
        assert sha(self.parent/'predictions.jsonl')==read(self.parent/'prediction-lock.json')['predictions_sha256'];assert sha(self.parent/'decisions.json')==read(self.parent/'decision-lock.json')['decisions_sha256'];assert read(self.parent/'audit.json')['status']=='pass'
        inputs=read(self.parent/'matcher-inputs.json');imu=np.load(self.qual/'export-imu.npz');times=imu['time_ns'];gyro=imu['values'][:,4:7];assert np.all(np.diff(times)>0) and np.isfinite(gyro).all();labels=[]
        for row in inputs:
            r={k:row[k] for k in ['arm','index','source','target','times_ns','qualified']};r.update(integrate_gyro(times,gyro,*row['times_ns']));labels.append(r)
        n=read(self.parent/'input-freeze.json')['anchors'];lookup={(r['arm'],r['index']):r for r in labels};common=[i for i in range(n) if all(lookup[a,i]['qualified'] and lookup[a,i]['supported'] for a in ['rgb','thermal'])];moving=[i for i in common if all(lookup[a,i]['rotation_deg']>=2 for a in ['rgb','thermal'])];population={'common_gyro':common,'paired_rotation_ge2deg':moving}
        write(self.out/'gyro-labels.json',labels);write(self.out/'population.json',population)
        files=[self.config,Path(__file__),ROOT/self.cfg['protocol'],self.parent/'predictions.jsonl',self.parent/'prediction-lock.json',self.parent/'decisions.json',self.parent/'decision-lock.json',self.parent/'audit.json',self.parent/'matcher-inputs.json',self.qual/'export-imu.npz',self.qual/'calibration.json',self.raw/'qualification.json',self.raw/'completion.json',self.raw/'independent-imu-audit.json',self.out/'gyro-labels.json',self.out/'population.json']
        files.extend([ROOT/'tools/audit_subt_gyro_consistency.py',ROOT/'tools/audit_subt_raw_imu.py',ROOT/'tools/test_subt_gyro_consistency.py'])
        pins.update({str(p.relative_to(ROOT)):sha(p) for p in files});write(self.out/'target-lock.json',{'pins':pins,'anchors':n,'rows':len(labels),'common_gyro':len(common),'paired_moving':len(moving),'prediction_errors_computed':False,'scope':'Gyro endpoints/cohorts frozen before disagreement calculations'});print('Gyro targets frozen',len(common),'common,',len(moving),'moving anchors',flush=True)
    def score(self):
        self.verify();labels=read(self.out/'gyro-labels.json');by={(r['arm'],r['index']):r for r in labels};cal=read(self.qual/'calibration.json');predictions=[json.loads(line) for line in (self.parent/'predictions.jsonl').read_text().splitlines()];rows=[]
        for p in predictions:
            ref=by[p['arm'],p['index']];r={k:p[k] for k in ['frontend','arm','index','status']};r.update(input_supported=p['qualified'],gyro_supported=ref['supported'],disagreement_deg=None)
            if p['status']=='ok' and ref['supported']:
                rc=np.array(cal[p['arm']]['effective_camera_to_imu'])[:3,:3];estimate=rc@np.array(p['R'])@rc.T;target=np.array(ref['target_from_source_imu']);r['disagreement_deg']=float(np.degrees(Rotation.from_matrix(estimate@target.T).magnitude()))
            rows.append(r)
        write(self.out/'disagreements.json',rows);population=read(self.out/'population.json');decisions=read(self.parent/'decisions.json');n=read(self.out/'target-lock.json')['anchors'];summary=summarize(rows,decisions,labels,population,n);write(self.out/'summary.json',summary);self.verify();write(self.out/'completion.json',{'rows':len(rows),'disagreements_sha256':sha(self.out/'disagreements.json'),'summary_sha256':sha(self.out/'summary.json'),'scope':summary['scope']});print('Gyro agreement scored',len(rows),'rows',flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['freeze','score']);p.add_argument('config',type=Path);a=p.parse_args();getattr(Diagnostic(a.config),a.stage)()
