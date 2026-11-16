"""Locked-output priority controls, label-assisted ceilings and documentary bounds.

No inference or fitted selector. Historical summary bounds are not raw replays.
"""
import argparse,json,math,re
from pathlib import Path
import numpy as np
from deployment_lighting import read,write,sha
from smoke_rgb_first_fallback import choose
ROOT=Path(__file__).resolve().parents[1]
POLICIES=['rgb','thermal','rgb_first','thermal_first']

def priority(primary,rows):
    secondary='thermal' if primary=='rgb' else 'rgb'
    assert all(r['qualified'] or r['status']!='ok' for r in rows.values())
    selected,request=choose(rows[primary]['status'],rows[secondary]['qualified'],rows[secondary]['status'])
    sensor={None:None,'rgb':primary,'thermal':secondary}[selected]
    calls=([primary] if rows[primary]['qualified'] else [])+([secondary] if request else [])
    return sensor,calls

def priority_bounds(n,nr,nt,union,cr,ct):
    assert 0<=nr<=n and 0<=nt<=n and max(nr,nt)<=union<=min(n,nr+nt)
    assert 0<=cr<=nr and 0<=ct<=nt
    rgb_only=union-nt;both=nr-rgb_only
    return {'lower':ct+max(0,cr-both),'upper':ct+min(cr,rgb_only),'rgb_only_outputs':rgb_only,'both_supported_outputs':both}

def learned_table(path):
    lines=Path(path).read_text().splitlines();start=next(i for i,l in enumerate(lines) if l.startswith('| Learned policy |') or l.startswith('| Policy | Valid proposal'))
    ns=[int(x) for x in re.findall(r'/\s*(\d+)',lines[start])];assert ns and len(set(ns))==1
    n=ns[0];rows={}
    for line in lines[start+2:]:
        if not line.startswith('|'):break
        cells=[c.strip() for c in line.split('|')[1:-1]];label=cells[0]
        key='rgb_first' if label.startswith('RGB-first') else 'rgb' if label=='RGB' else 'thermal' if label=='Thermal' else None
        if key:rows[key]=[int(re.match(r'\d+',c).group()) for c in cells[1:4]]
    assert set(rows)=={'rgb','thermal','rgb_first'}
    return {'anchors':n,'rgb_outputs':rows['rgb'][0],'thermal_outputs':rows['thermal'][0],'union_outputs':rows['rgb_first'][0],'thresholds':{str(t):{'rgb':rows['rgb'][j],'thermal':rows['thermal'][j],'rgb_first':rows['rgb_first'][j]} for j,t in [(1,1),(2,5)]},'target':'Supplied-reference rotation error; summary-level evidence, not a new raw replay','source':str(Path(path).relative_to(ROOT))}

def choices(raw,inputs,n):
    by={(r['frontend'],r['arm'],r['index']):r for r in raw};meta={(r['arm'],r['index']):r for r in inputs};result=[]
    for f in ['sift','learned']:
        for i in range(n):
            rows={a:by[f,a,i] for a in ['rgb','thermal']};policies={}
            for name in POLICIES:
                if name in ['rgb','thermal']:sensor=name if rows[name]['status']=='ok' else None;calls=[name] if rows[name]['qualified'] else []
                else:sensor,calls=priority('rgb' if name=='rgb_first' else 'thermal',rows)
                policies[name]={'sensor':sensor,'requested':calls,'endpoints':{k:meta[sensor,i][k] for k in ['source','target','times_ns']} if sensor else None}
            result.append({'frontend':f,'index':i,'policies':policies})
    return result

def evaluate(raw,selected,disagreements,pop,n):
    pred={(r['frontend'],r['arm'],r['index']):r for r in raw};errors={(r['frontend'],r['arm'],r['index']):r['disagreement_deg'] for r in disagreements};common=set(pop['common_gyro']);groups={'all_declared':set(range(n)),'common_gyro':common,'paired_rotation_ge2deg':set(pop['paired_rotation_ge2deg']),**{'third'+str(j+1):{i for i in common if i*3//n==j} for j in range(3)}};result={}
    for f in ['sift','learned']:
        result[f]={}
        for name,ids in groups.items():
            eligible=ids&common;rows=[d for d in selected if d['frontend']==f and d['index'] in ids];entry={}
            for policy in POLICIES:
                picks=[d['policies'][policy] for d in rows];ee=[errors[f,d['policies'][policy]['sensor'],d['index']] for d in rows if d['index'] in eligible and d['policies'][policy]['sensor']];assert all(e is not None for e in ee)
                called=[pred[f,a,d['index']] for d in rows for a in d['policies'][policy]['requested']];assert all(p['qualified'] for p in called)
                entry[policy]={'anchors':len(ids),'outputs':sum(p['sensor'] is not None for p in picks),'gyro_eligible':len(eligible),'compared_outputs':len(ee),'within1deg':sum(e<=1 for e in ee),'within5deg':sum(e<=5 for e in ee),'conditional_median_deg':float(np.median(ee)) if ee else None,'matcher_calls':len(called),'projected_seconds':math.fsum(p['seconds'] for p in called),'pose_accuracy':None}
            entry['headroom']={}
            for t in [1,5]:
                good={a:{i for i in eligible if pred[f,a,i]['status']=='ok' and errors[f,a,i] is not None and errors[f,a,i]<=t} for a in ['rgb','thermal']};union=good['rgb']|good['thermal'];both={i for i in eligible if all(pred[f,a,i]['status']=='ok' for a in ['rgb','thermal'])};best=max(len(s) for s in good.values())
                entry['headroom'][str(t)]={'gyro_eligible':len(eligible),'rgb_agrees':len(good['rgb']),'thermal_agrees':len(good['thermal']),'best_fixed':best,'oracle_either_agrees':len(union),'oracle_increment_over_best_fixed':len(union)-best,'both_supported':len(both),'rgb_disagrees_thermal_agrees':len(both&(good['thermal']-good['rgb'])),'thermal_disagrees_rgb_agrees':len(both&(good['rgb']-good['thermal'])),'both_supported_neither_agrees':len(both-union),'deployable':False}
            result[f][name]=entry
    return {'target':'Nominal gyro agreement, not calibrated pose accuracy','pose_accuracy':None,'frontends':result}

class Replay:
    def __init__(self,config):
        self.config=Path(config).resolve();self.cfg=read(self.config);self.out=ROOT/self.cfg['output'];self.parent=ROOT/self.cfg['matching'];self.gyro=ROOT/self.cfg['gyro']
    def verify(self):
        for p,h in read(self.out/'input-freeze.json')['pins'].items():assert sha(ROOT/p)==h,p
    def freeze(self):
        self.out.mkdir(parents=True,exist_ok=False);assert read(self.parent/'audit.json')['status']=='pass' and read(self.gyro/'audit.json')['status']=='pass'
        assert sha(self.parent/'predictions.jsonl')==read(self.parent/'prediction-lock.json')['predictions_sha256'];assert sha(self.gyro/'disagreements.json')==read(self.gyro/'completion.json')['disagreements_sha256']
        files=[self.config,Path(__file__),ROOT/'tools/smoke_rgb_first_fallback.py',ROOT/'MAC-VO-main/Scripts/Setup/compare_factor_proxy.py',ROOT/'tools/audit_camera_priority_controls.py',ROOT/'tools/test_camera_priority_controls.py',ROOT/self.cfg['protocol']]
        files.extend(self.parent/p for p in ['predictions.jsonl','prediction-lock.json','decisions.json','decision-lock.json','matcher-inputs.json','input-freeze.json','audit.json'])
        files.extend(self.gyro/p for p in ['disagreements.json','population.json','target-lock.json','completion.json','audit.json'])
        history={}
        for label,name in self.cfg['historical_findings'].items():
            path=ROOT/name;history[label]=learned_table(path);files.append(path)
        write(self.out/'historical-counts.json',history);files.append(self.out/'historical-counts.json');write(self.out/'input-freeze.json',{'pins':{str(p.relative_to(ROOT)):sha(p) for p in files},'anchors':read(self.parent/'input-freeze.json')['anchors'],'scope':'Exposed fixed-priority control; historical bounds are not new paired benchmarks'})
    def select(self):
        self.verify();raw=[json.loads(s) for s in (self.parent/'predictions.jsonl').read_text().splitlines()];n=read(self.out/'input-freeze.json')['anchors'];selected=choices(raw,read(self.parent/'matcher-inputs.json'),n);write(self.out/'choices.json',selected);write(self.out/'choice-lock.json',{'rows':len(selected),'choices_sha256':sha(self.out/'choices.json'),'gyro_errors_used_for_choice':False});print('Status choices locked',len(selected),flush=True)
    def score(self):
        self.verify();assert sha(self.out/'choices.json')==read(self.out/'choice-lock.json')['choices_sha256'];raw=[json.loads(s) for s in (self.parent/'predictions.jsonl').read_text().splitlines()];selected=read(self.out/'choices.json');n=read(self.out/'input-freeze.json')['anchors'];summary=evaluate(raw,selected,read(self.gyro/'disagreements.json'),read(self.gyro/'population.json'),n);history=read(self.out/'historical-counts.json');bounds={}
        for name,c in history.items():
            bounds[name]={'anchors':c['anchors'],'target':c['target'],'thresholds':{}}
            for t,counts in c['thresholds'].items():
                b=priority_bounds(c['anchors'],c['rgb_outputs'],c['thermal_outputs'],c['union_outputs'],counts['rgb'],counts['thermal']);b.update(fixed_rgb_agrees=counts['rgb'],rgb_first_agrees=counts['rgb_first'],universal_thermal_priority_rejected_against_rgb=b['upper']<counts['rgb']);bounds[name]['thresholds'][t]=b
        write(self.out/'summary.json',summary);write(self.out/'historical-bounds.json',bounds);self.verify();write(self.out/'completion.json',{'summary_sha256':sha(self.out/'summary.json'),'bounds_sha256':sha(self.out/'historical-bounds.json'),'scope':'Nominal OutdoorNight gyro replay plus separate documentary supplied-reference bounds; no inference, pooling or deployable oracle'})

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['freeze','select','score']);p.add_argument('config',type=Path);a=p.parse_args();getattr(Replay(a.config),a.stage)()
