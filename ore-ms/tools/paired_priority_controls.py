"""Supplied-reference adapter for frozen status-only camera priority controls.

Reuses the existing routing helper; keeps the gyro-specific predecessor frozen.
"""
import argparse,json,math
from pathlib import Path
import numpy as np
from camera_priority_controls import choices,POLICIES
from deployment_lighting import read,write,sha
ROOT=Path(__file__).resolve().parents[1]
TARGET='Nominal supplied-reference camera-relative rotation and directed translation; not independent physical truth'

def groups(pop,n):
    common=set(pop['common_reference'])
    return {'all_declared':set(range(n)),'common_reference':common,'paired_rotation_ge2deg':set(pop['paired_rotation_ge2deg']),'translation_ge1cm_both':set(pop['translation_ge1cm_both']),**{'third'+str(j+1):{i for i in common if i*3//n==j} for j in range(3)}}

def evaluate(raw,selected,scores,pop,n):
    pred={(p['frontend'],p['arm'],p['index']):p for p in raw};sm={(s['frontend'],s['arm'],s['index']):s for s in scores};common=set(pop['common_reference']);trans=set(pop['translation_ge1cm_both']);result={}
    for f in ['sift','learned']:
        result[f]={}
        for name,ids in groups(pop,n).items():
            rows=[d for d in selected if d['frontend']==f and d['index'] in ids];entry={}
            for policy in POLICIES:
                picked=[sm[f,d['policies'][policy]['sensor'],d['index']] for d in rows if d['policies'][policy]['sensor']];rr=[s['R_error'] for s in picked if s['R_error'] is not None];tt=[s['t_error'] for s in picked if s['t_error'] is not None];called=[pred[f,a,d['index']] for d in rows for a in d['policies'][policy]['requested']]
                primary=policy if policy in ['rgb','thermal'] else 'rgb' if policy=='rgb_first' else 'thermal';overhead=math.fsum(pred[f,primary,d['index']]['seconds'] for d in rows if not pred[f,primary,d['index']]['qualified'])
                entry[policy]={'anchors':len(ids),'outputs':len(picked),'rotation_eligible':len(ids&common),'translation_eligible':len(ids&trans),'rotation_compared_outputs':len(rr),'translation_compared_outputs':len(tt),'R1':sum(s['R1'] for s in picked),'R5':sum(s['R5'] for s in picked),'joint':sum(s['joint'] for s in picked),'R_median':float(np.median(rr)) if rr else None,'t_median':float(np.median(tt)) if tt else None,'matcher_calls':len(called),'unsupported_primary_seconds':overhead,'projected_seconds':math.fsum(p['seconds'] for p in called)+overhead}
            entry['headroom']={}
            for endpoint,eligible in [('R1',ids&common),('R5',ids&common),('joint',ids&trans)]:
                good={a:{i for i in eligible if sm[f,a,i][endpoint]} for a in ['rgb','thermal']};union=good['rgb']|good['thermal'];both={i for i in eligible if all(pred[f,a,i]['status']=='ok' for a in ['rgb','thermal'])};best_fixed=max(entry[a][endpoint] for a in ['rgb','thermal']);best_priority=max(entry[a][endpoint] for a in ['rgb_first','thermal_first'])
                entry['headroom'][endpoint]={'eligible':len(eligible),'best_fixed':best_fixed,'best_priority':best_priority,'oracle_either_correct':len(union),'oracle_increment_over_best_fixed':len(union)-best_fixed,'oracle_increment_over_best_priority':len(union)-best_priority,'both_supported':len(both),'rgb_wrong_thermal_correct':len(both&(good['thermal']-good['rgb'])),'thermal_wrong_rgb_correct':len(both&(good['rgb']-good['thermal'])),'both_supported_neither_correct':len(both-union),'no_correct_available_proposal':len(eligible-union),'deployable':False}
            entry['rgb_or_zero_rotation']={str(t):sum(sm[f,'rgb',i][key] if pred[f,'rgb',i]['status']=='ok' else sm[f,'rgb',i]['reference_rotation_deg']<=t for i in ids&common) for t,key in [(1,'R1'),(5,'R5')]}
            result[f][name]=entry
    return {'target':TARGET,'independent_physical_truth':False,'prior_results_exposed':True,'frontends':result}

class Replay:
    def __init__(self,config):
        self.config=Path(config).resolve();self.cfg=read(self.config);self.out=ROOT/self.cfg['output'];self.parent=ROOT/self.cfg['matching'];self.qual=ROOT/self.cfg['qualification']
    def verify(self):
        for p,h in read(self.out/'input-freeze.json')['pins'].items():assert sha(ROOT/p)==h,p
    def freeze(self):
        assert read(self.parent/'audit.json')['status']==read(self.qual/'audit.json')['status']=='pass'
        for name,key,receipt in [('predictions.jsonl','predictions_sha256','prediction-lock.json'),('decisions.json','decisions_sha256','decision-lock.json'),('scores.json','scores_sha256','completion.json'),('summary.json','summary_sha256','completion.json')]:assert sha(self.parent/name)==read(self.parent/receipt)[key],name
        n=read(self.parent/'input-freeze.json')['anchors'];raw=[json.loads(line) for line in (self.parent/'predictions.jsonl').read_text().splitlines()];inputs=read(self.parent/'matcher-inputs.json');scores=read(self.parent/'scores.json');expected={(f,a,i) for f in ['sift','learned'] for a in ['rgb','thermal'] for i in range(n)};meta={(r['arm'],r['index']):r for r in inputs};assert len(raw)==len(scores)==4*n and len(inputs)==len(meta)==2*n;assert {(r['frontend'],r['arm'],r['index']) for r in raw}=={(r['frontend'],r['arm'],r['index']) for r in scores}==expected
        for r in raw:
            m=meta[r['arm'],r['index']];assert r['qualified']==m['qualified'] and (r['qualified'] or r['status']=='input_unsupported');assert len(m['times_ns'])==2 and all(type(t)==int for t in m['times_ns']);assert m['source']!=m['target'] and m['times_ns'][0]<m['times_ns'][1];assert math.isfinite(r['seconds']) and r['seconds']>=0 
        pop=read(self.qual/'population.json');assert set(pop['paired_rotation_ge2deg'])<=set(pop['common_reference']) and set(pop['translation_ge1cm_both'])<=set(pop['common_reference'])<=set(range(n))
        files=[self.config,Path(__file__),ROOT/'tools/camera_priority_controls.py',ROOT/'tools/smoke_rgb_first_fallback.py',ROOT/'tools/deployment_lighting.py',ROOT/'MAC-VO-main/Scripts/Setup/compare_factor_proxy.py',ROOT/'tools/audit_paired_priority_controls.py',ROOT/'tools/test_paired_priority_controls.py',ROOT/self.cfg['protocol'],ROOT/self.cfg['historical_bounds']]
        files.extend(self.parent/p for p in ['predictions.jsonl','prediction-lock.json','matcher-inputs.json','decisions.json','decision-lock.json','scores.json','summary.json','completion.json','audit.json','input-freeze.json']);files.extend(self.qual/p for p in ['population.json','audit.json','qualification.json']);self.out.mkdir(parents=True,exist_ok=False);write(self.out/'input-freeze.json',{'pins':{str(p.relative_to(ROOT)):sha(p) for p in files},'anchors':n,'target':TARGET,'preflight_prediction_rows':len(raw),'preflight_input_rows':len(inputs)})
    def select(self):
        self.verify();raw=[json.loads(line) for line in (self.parent/'predictions.jsonl').read_text().splitlines()];selected=choices(raw,read(self.parent/'matcher-inputs.json'),read(self.out/'input-freeze.json')['anchors']);write(self.out/'choices.json',selected);write(self.out/'choice-lock.json',{'rows':len(selected),'choices_sha256':sha(self.out/'choices.json'),'reference_errors_used_for_choice':False});print('Status choices locked',len(selected))
    def score(self):
        self.verify();assert sha(self.out/'choices.json')==read(self.out/'choice-lock.json')['choices_sha256'];raw=[json.loads(line) for line in (self.parent/'predictions.jsonl').read_text().splitlines()];summary=evaluate(raw,read(self.out/'choices.json'),read(self.parent/'scores.json'),read(self.qual/'population.json'),read(self.out/'input-freeze.json')['anchors']);old=read(self.parent/'summary.json');checks=0
        for f,cohorts in summary['frontends'].items():
            for g,values in cohorts.items():
                for policy in ['rgb','thermal','rgb_first']:
                    for key,expected in old[f][g][policy].items():
                        actual=values[policy][key];assert abs(actual-expected)<1e-8 if key in ['projected_seconds','R_median','t_median'] and expected is not None else actual==expected,(f,g,policy,key);checks+=1
                assert values['rgb_or_zero_rotation']==old[f][g]['rgb_or_zero_rotation'];checks+=1
        bounds=read(ROOT/self.cfg['historical_bounds'])[self.cfg['recording']];observed=summary['frontends']['learned']['paired_rotation_ge2deg'];bound_checks={}
        for t,key in [('1','R1'),('5','R5')]:
            b=bounds['thresholds'][t];v=observed['thermal_first'][key];assert b['lower']<=v<=b['upper'];bound_checks[key]={'lower':b['lower'],'upper':b['upper'],'observed':v,'inside':True}
        write(self.out/'summary.json',summary);write(self.out/'controls.json',{'parent_parity_checks':checks,'learned_moving_historical_bounds':bound_checks});self.verify();write(self.out/'completion.json',{'summary_sha256':sha(self.out/'summary.json'),'controls_sha256':sha(self.out/'controls.json'),'scope':TARGET+'; cached projected computation, no fitted selector'})

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['freeze','select','score']);p.add_argument('config',type=Path);a=p.parse_args();getattr(Replay(a.config),a.stage)()
