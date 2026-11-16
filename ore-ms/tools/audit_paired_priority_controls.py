"""Independent supplied-reference priority and oracle audit; no production imports."""
import hashlib,json,math,statistics
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def read(p):return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def main(config):
    checks=0
    def check(v,label):
        nonlocal checks
        assert v,label
        checks+=1
    cfg=read(config);out=ROOT/cfg['output'];parent=ROOT/cfg['matching'];qual=ROOT/cfg['qualification'];freeze=read(out/'input-freeze.json');lock=read(out/'choice-lock.json');done=read(out/'completion.json')
    for path,h in freeze['pins'].items():check(sha(ROOT/path)==h,'frozen '+path)
    for name,key,receipt in [('choices.json','choices_sha256',lock),('summary.json','summary_sha256',done),('controls.json','controls_sha256',done)]:check(sha(out/name)==receipt[key],'locked '+name)
    check(lock['reference_errors_used_for_choice'] is False,'status-only selection');n=freeze['anchors'];raw=[json.loads(l) for l in (parent/'predictions.jsonl').read_text().splitlines()];pred={(r['frontend'],r['arm'],r['index']):r for r in raw};scored=read(parent/'scores.json');scores={(r['frontend'],r['arm'],r['index']):r for r in scored};inputs={(r['arm'],r['index']):r for r in read(parent/'matcher-inputs.json')};oldchoices={(d['frontend'],d['index']):d for d in read(parent/'decisions.json')};selected=read(out/'choices.json');ds={(d['frontend'],d['index']):d for d in selected};pop=read(qual/'population.json');common=set(pop['common_reference']);trans=set(pop['translation_ge1cm_both']);expected={(f,a,i) for f in ['sift','learned'] for a in ['rgb','thermal'] for i in range(n)}
    check(len(raw)==len(scored)==len(pred)==len(scores)==4*n and set(pred)==set(scores)==expected,'complete unique source rows');check(len(ds)==len(selected)==lock['rows']==2*n and len(inputs)==2*n,'complete unique decisions/inputs')
    for (f,a,i),p in pred.items():
        r=scores[f,a,i];m=inputs[a,i];check(p['qualified']==m['qualified'] and (p['qualified'] or p['status']=='input_unsupported'),'native support');check(math.isfinite(p['seconds']) and p['seconds']>=0 ,'valid duration');check(r['status']==p['status'],'source score status')
        rotation=i in common and p['status']=='ok';translation=i in trans and rotation;check((r['R_error'] is not None)==rotation and (r['t_error'] is not None)==translation,'reference eligibility remains distinct')
        for field in ['R_error','t_error']:
            check(r[field] is None or math.isfinite(r[field]) and 0<=r[field]<=180,'finite angular errors')
        check(r['R1']==bool(rotation and r['R_error']<=1) and r['R5']==bool(rotation and r['R_error']<=5) and r['joint']==bool(translation and r['R_error']<=5 and r['t_error']<=15),'own numeric endpoint labels')
    for (f,i),d in ds.items():
        check(f in ['sift','learned'] and 0<=i<n,'decision key')
        for policy in ['rgb','thermal','rgb_first','thermal_first']:
            order=[policy] if policy in ['rgb','thermal'] else ['rgb','thermal'] if policy=='rgb_first' else ['thermal','rgb'];calls=[];sensor=None
            for a in order:
                if pred[f,a,i]['qualified']:calls.append(a)
                if pred[f,a,i]['status']=='ok':sensor=a;break
            c=d['policies'][policy];check(c['sensor']==sensor and c['requested']==calls,'routing truth table');endpoints={k:inputs[sensor,i][k] for k in ['source','target','times_ns']} if sensor else None;check(c['endpoints']==endpoints,'own source target timestamps')
        check(d['policies']['rgb_first']['sensor']==oldchoices[f,i]['policies']['rgb_first'],'original routing unchanged');check((d['policies']['rgb_first']['sensor'] is None)==(d['policies']['thermal_first']['sensor'] is None),'same output union')
    groups={'all_declared':set(range(n)),'common_reference':common,'paired_rotation_ge2deg':set(pop['paired_rotation_ge2deg']),'translation_ge1cm_both':trans}
    for j in range(3):groups['third'+str(j+1)]={i for i in common if (3*i)//n==j}
    summary=read(out/'summary.json');check(summary['target']=='Nominal supplied-reference camera-relative rotation and directed translation; not independent physical truth' and summary['independent_physical_truth'] is False and summary['prior_results_exposed'] is True,'target and exposure semantics');old=read(parent/'summary.json');parity=0
    for f in ['sift','learned']:
        for name,ids in groups.items():
            record=summary['frontends'][f][name]
            for policy in ['rgb','thermal','rgb_first','thermal_first']:
                picks=[(i,ds[f,i]['policies'][policy]) for i in sorted(ids)];ss=[scores[f,c['sensor'],i] for i,c in picks if c['sensor']];rr=[r['R_error'] for r in ss if r['R_error'] is not None];tt=[r['t_error'] for r in ss if r['t_error'] is not None];calls=[pred[f,a,i] for i,c in picks for a in c['requested']]
                primary=policy if policy in ['rgb','thermal'] else 'rgb' if policy=='rgb_first' else 'thermal';overhead=math.fsum(pred[f,primary,i]['seconds'] for i in ids if not pred[f,primary,i]['qualified'])
                values={'anchors':len(ids),'outputs':len(ss),'rotation_eligible':len(ids&common),'translation_eligible':len(ids&trans),'rotation_compared_outputs':len(rr),'translation_compared_outputs':len(tt),'R1':sum(r['R_error'] is not None and r['R_error']<=1 for r in ss),'R5':sum(r['R_error'] is not None and r['R_error']<=5 for r in ss),'joint':sum(r['R_error'] is not None and r['t_error'] is not None and r['R_error']<=5 and r['t_error']<=15 for r in ss),'R_median':statistics.median(rr) if rr else None,'t_median':statistics.median(tt) if tt else None,'matcher_calls':len(calls),'unsupported_primary_seconds':overhead,'projected_seconds':math.fsum(r['seconds'] for r in calls)+overhead}
                check(set(record[policy])==set(values),'all policy fields checked')
                for key,v in values.items():check(abs(record[policy][key]-v)<1e-8 if isinstance(v,float) else record[policy][key]==v,'aggregate '+key)
                if policy!='thermal_first':
                    for key,v in old[f][name][policy].items():check(abs(record[policy][key]-v)<1e-8 if isinstance(v,float) else record[policy][key]==v,'parent parity '+key);parity+=1
            for field,eligible in [('R1',common&ids),('R5',common&ids),('joint',trans&ids)]:
                bits=[]
                for i in eligible:
                    flags=[scores[f,a,i][field] for a in ['rgb','thermal']];both=all(pred[f,a,i]['status']=='ok' for a in ['rgb','thermal']);bits.append((both,*flags))
                union=sum(r or t for _,r,t in bits);bestfixed=max(record[a][field] for a in ['rgb','thermal']);bestpriority=max(record[a][field] for a in ['rgb_first','thermal_first']);expected={'eligible':len(eligible),'best_fixed':bestfixed,'best_priority':bestpriority,'oracle_either_correct':union,'oracle_increment_over_best_fixed':union-bestfixed,'oracle_increment_over_best_priority':union-bestpriority,'both_supported':sum(b for b,r,t in bits),'rgb_wrong_thermal_correct':sum(b and not r and t for b,r,t in bits),'thermal_wrong_rgb_correct':sum(b and r and not t for b,r,t in bits),'both_supported_neither_correct':sum(b and not r and not t for b,r,t in bits),'no_correct_available_proposal':len(eligible)-union,'deployable':False};check(record['headroom'][field]==expected,'oracle conflicts and ceiling')
            zero={str(t):sum(scores[f,'rgb',i][field] if pred[f,'rgb',i]['status']=='ok' else scores[f,'rgb',i]['reference_rotation_deg']<=t for i in ids&common) for t,field in [(1,'R1'),(5,'R5')]};check(record['rgb_or_zero_rotation']==zero==old[f][name]['rgb_or_zero_rotation'],'identity control parity');parity+=1
    control=read(out/'controls.json');check(control['parent_parity_checks']==parity,'all parity comparisons counted');bounds=read(ROOT/cfg['historical_bounds'])[cfg['recording']]
    for t,key in [('1','R1'),('5','R5')]:
        v=summary['frontends']['learned']['paired_rotation_ge2deg']['thermal_first'][key];b=bounds['thresholds'][t];check(b['lower']<=v<=b['upper'] and control['learned_moving_historical_bounds'][key]=={'lower':b['lower'],'upper':b['upper'],'observed':v,'inside':True},'actual replay inside old bound')
    result={'status':'pass','checks':checks,'decision_rows':len(selected),'auditor_sha256':sha(__file__),'scope':'Independent complete routing/endpoints/reference eligibility/cost/aggregates/conflicts/oracle/parent parity and historical bounds; no physical truth or deployable oracle claim'};(out/'audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return result
if __name__=='__main__':
    import sys
    main(Path(sys.argv[1]))
