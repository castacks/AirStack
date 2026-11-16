"""Independent fixed-priority, gyro headroom and documentary-bound audit."""
import json,math,re,hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];checks=0
def read(p):return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def check(v,label):
    global checks
    assert v,label
    checks+=1
def main(config):
    cfg=read(config);out=ROOT/cfg['output'];parent=ROOT/cfg['matching'];gyro=ROOT/cfg['gyro'];freeze=read(out/'input-freeze.json');lock=read(out/'choice-lock.json');done=read(out/'completion.json')
    for p,h in freeze['pins'].items():check(sha(ROOT/p)==h,'frozen '+p)
    for name,key,receipt in [('choices.json','choices_sha256',lock),('summary.json','summary_sha256',done),('historical-bounds.json','bounds_sha256',done)]:check(sha(out/name)==receipt[key],'locked '+name)
    check(lock['gyro_errors_used_for_choice'] is False,'status choices without errors');n=freeze['anchors'];raw=[json.loads(s) for s in (parent/'predictions.jsonl').read_text().splitlines()];pred={(r['frontend'],r['arm'],r['index']):r for r in raw};inputs={(r['arm'],r['index']):r for r in read(parent/'matcher-inputs.json')};old={(r['frontend'],r['index']):r for r in read(parent/'decisions.json')};selected=read(out/'choices.json');dmap={(d['frontend'],d['index']):d for d in selected};check(len(selected)==len(dmap)==2*n and len(pred)==4*n,'full populations')
    for (f,i),d in dmap.items():
        for policy in ['rgb','thermal','rgb_first','thermal_first']:
            order=[policy] if policy in ['rgb','thermal'] else ['rgb','thermal'] if policy=='rgb_first' else ['thermal','rgb'];sensor=None;requests=[]
            for a in order:
                p=pred[f,a,i]
                if p['qualified']:requests.append(a)
                if p['status']=='ok':sensor=a;break
            choice=d['policies'][policy];check(choice['sensor']==sensor and choice['requested']==requests,'priority/request truth table')
            expected={k:inputs[sensor,i][k] for k in ['source','target','times_ns']} if sensor else None;check(choice['endpoints']==expected,'own sensor endpoints')
        check(d['policies']['rgb_first']['sensor']==old[f,i]['policies']['rgb_first'],'old RGB-first unchanged');check((d['policies']['rgb_first']['sensor'] is None)==(d['policies']['thermal_first']['sensor'] is None),'same mechanical status union')
    errors={(r['frontend'],r['arm'],r['index']):r['disagreement_deg'] for r in read(gyro/'disagreements.json')};pop=read(gyro/'population.json');common=set(pop['common_gyro']);groups={'all_declared':set(range(n)),'common_gyro':common,'paired_rotation_ge2deg':set(pop['paired_rotation_ge2deg']),**{'third'+str(j+1):{i for i in common if i*3//n==j} for j in range(3)}};summary=read(out/'summary.json');check(summary['pose_accuracy'] is None,'nominal gyro target only')
    for f in ['sift','learned']:
        for name,ids in groups.items():
            eligible=ids&common;rows=[dmap[f,i] for i in sorted(ids)];record=summary['frontends'][f][name]
            for policy in ['rgb','thermal','rgb_first','thermal_first']:
                picks=[(d['index'],d['policies'][policy]) for d in rows];compared=[errors[f,c['sensor'],i] for i,c in picks if i in eligible and c['sensor']];called=[pred[f,a,i] for i,c in picks for a in c['requested']];r=record[policy]
                check(r['anchors']==len(ids) and r['outputs']==sum(c['sensor'] is not None for _,c in picks) and r['gyro_eligible']==len(eligible) and r['compared_outputs']==len(compared),'policy denominators')
                check(r['matcher_calls']==len(called) and all(p['qualified'] for p in called) and abs(r['projected_seconds']-math.fsum(p['seconds'] for p in called))<1e-8,'failed/supported cost accounting')
                check(r['within1deg']==sum(e<=1 for e in compared) and r['within5deg']==sum(e<=5 for e in compared) and r['pose_accuracy'] is None,'agreement counts and semantics')
                check(r['conditional_median_deg'] is None if not compared else abs(r['conditional_median_deg']-np.median(compared))<1e-6,'conditional median')
            for threshold in [1,5]:
                bits=[]
                for i in eligible:
                    both=all(pred[f,a,i]['status']=='ok' for a in ['rgb','thermal']);agrees=[pred[f,a,i]['status']=='ok' and errors[f,a,i] is not None and errors[f,a,i]<=threshold for a in ['rgb','thermal']];bits.append((both,*agrees))
                a=sum(r for _,r,t in bits);b=sum(t for _,r,t in bits);union=sum(r or t for _,r,t in bits);h=record['headroom'][str(threshold)]
                expected={'gyro_eligible':len(eligible),'rgb_agrees':a,'thermal_agrees':b,'best_fixed':max(a,b),'oracle_either_agrees':union,'oracle_increment_over_best_fixed':union-max(a,b),'both_supported':sum(x for x,_,_ in bits),'rgb_disagrees_thermal_agrees':sum(x and not r and t for x,r,t in bits),'thermal_disagrees_rgb_agrees':sum(x and r and not t for x,r,t in bits),'both_supported_neither_agrees':sum(x and not r and not t for x,r,t in bits),'deployable':False};check(h==expected,'nondeployable oracle and conflict attribution')
    counts=read(out/'historical-counts.json');bounds=read(out/'historical-bounds.json')
    for label,c in counts.items():
        text=(ROOT/c['source']).read_text();parsed={}
        for line in text.splitlines():
            if not re.match(r'^\|\s*(RGB\s*\||Thermal\s*\||RGB-first)',line):continue
            clean=re.sub(r'\([^)]*\)','',line);parts=[p.strip() for p in clean.split('|')[1:-1]];name='rgb_first' if parts[0].startswith('RGB-first') else 'rgb' if parts[0]=='RGB' else 'thermal'
            if name not in parsed:parsed[name]=[int(x) for x in re.findall(r'\d+', '|'.join(parts[1:]))[:3]]
            if len(parsed)==3:break
        check(parsed['rgb']==[c['rgb_outputs'],c['thresholds']['1']['rgb'],c['thresholds']['5']['rgb']] and parsed['thermal']==[c['thermal_outputs'],c['thresholds']['1']['thermal'],c['thresholds']['5']['thermal']] and parsed['rgb_first']==[c['union_outputs'],c['thresholds']['1']['rgb_first'],c['thresholds']['5']['rgb_first']],'documentary count extraction')
        check(bounds[label]['target']==c['target'] and bounds[label]['anchors']==c['anchors'],'heterogeneous target/population retained')
        q=c['union_outputs']-c['thermal_outputs'];overlap=c['rgb_outputs']-q
        for t,data in c['thresholds'].items():
            # Enumerate every possible number of RGB-correct rows in RGB-only cells.
            feasible=[k for k in range(q+1) if 0<=data['rgb']-k<=overlap];expected={'lower':data['thermal']+min(feasible),'upper':data['thermal']+max(feasible),'rgb_only_outputs':q,'both_supported_outputs':overlap,'fixed_rgb_agrees':data['rgb'],'rgb_first_agrees':data['rgb_first'],'universal_thermal_priority_rejected_against_rgb':data['thermal']+max(feasible)<data['rgb']};check(bounds[label]['thresholds'][t]==expected,'tight historical bound by feasible allocations')
    result={'status':'pass','checks':checks,'decision_rows':len(selected),'auditor_sha256':sha(__file__),'scope':'Independent priority/own endpoints/cost/gyro/headroom and tight documentary bounds; no new historical replay, inference or pose truth'};(out/'audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':
    import sys
    main(Path(sys.argv[1]))
