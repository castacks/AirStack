"""Independent trace/skew angular amount, identity and aggregation audit."""
import hashlib,json,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
def read(p):return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def quartiles(values):
    if not values:return None
    values=sorted(values);out=[]
    for q in [.25,.5,.75]:
        x=(len(values)-1)*q;l=int(x);h=math.ceil(x);out.append(values[l]*(1-(x-l))+values[h]*(x-l))
    return out

def main(config):
    cfg=read(config);out=ROOT/cfg['output'];count=0
    def check(v,label):
        nonlocal count
        assert v,label
        count+=1
    freeze=read(out/'input-freeze.json');check(freeze['near_identity_deg']==.1,'fixed assay cut')
    for p,h in freeze['pins'].items():check(sha(ROOT/p)==h,'source '+p)
    check(sha(out/'angles.json')==read(out/'angle-lock.json')['angles_sha256'] and read(out/'angle-lock.json')['reference_values_parsed'] is False,'angle lock before reference')
    done=read(out/'completion.json');check(done['new_inference_calls']==0 and done['selector_changed'] is False,'diagnostic semantics')
    for file in ['rows','summary']:check(sha(out/(file+'.json'))==done[file+'_sha256'],'locked '+file)
    angle=read(out/'angles.json');rows=read(out/'rows.json');summary=read(out/'summary.json');cases=read(ROOT/cfg['strata']/'cases.json')
    for name,c in cfg['records'].items():
        raw=[json.loads(l) for l in (ROOT/c['matching']/'predictions.jsonl').read_text().splitlines()];scores={(s['frontend'],s['arm'],s['index']):s for s in read(ROOT/c['matching']/'scores.json')};a_map={(r['frontend'],r['arm'],r['index']):r for r in angle[name]};r_map={(r['frontend'],r['arm'],r['index']):r for r in rows[name]};n=freeze['anchors'][name];check(len(a_map)==len(r_map)==len(raw)==len(angle[name])==len(rows[name])==4*n,'complete unique populations')
        for p in raw:
            key=p['frontend'],p['arm'],p['index'];a=a_map[key];r=r_map[key];s=scores[key];check(all(a[k]==p[k]==r[k]==s[k] for k in ['frontend','arm','index','status']),'own source/status keys')
            if p['status']=='ok':
                R=np.asarray(p['R']);check(R.shape==(3,3) and np.isfinite(R).all() and np.max(np.abs(R.T@R-np.eye(3)))<=1e-5 and abs(np.linalg.det(R)-1)<=1e-5,'SO3 input');skew=np.array([R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]])/2;theta=math.degrees(math.atan2(float(np.linalg.norm(skew)),float((np.trace(R)-1)/2)));check(abs(theta-a['estimated_deg'])<1e-6,'independent atan2 angle');check(a['near_identity']==(a['estimated_deg']<=.1),'near-identity cutoff')
            else:check(a['estimated_deg'] is None and a['near_identity'] is False,'missing estimates remain null')
            check(r['estimated_deg']==a['estimated_deg'] and r['near_identity']==a['near_identity'],'locked estimate unchanged');check(r['reference_deg']==s['reference_rotation_deg'] and r['error_deg']==s['R_error'] and r['R1']==s['R1'] and r['R5']==s['R5'],'unaltered own reference labels')
            expected_ratio=a['estimated_deg']/s['reference_rotation_deg'] if a['estimated_deg'] is not None and s['reference_rotation_deg'] is not None and s['reference_rotation_deg']>=2 else None;check(r['motion_ratio']==expected_ratio,'moving amount ratio eligibility')
            if r['error_deg'] is not None:
                e,t,error=r['estimated_deg'],r['reference_deg'],r['error_deg'];check(abs(e-t)<=error+1e-5 and error<=e+t+1e-5,'SO3 triangle bounds');check(r['error_minus_identity_deg']==error-t,'signed identity error gap');check(r['R1']==(error<=1) and r['R5']==(error<=5),'full orientation thresholds')
            else:check(r['error_minus_identity_deg'] is None and r['R1'] is False and r['R5'] is False,'no fabricated errors')
        pop=read(ROOT/c['qualification']/'population.json');common=set(pop['common_reference']);groups={'all_declared':set(range(n)),'common_reference':common,'paired_rotation_ge2deg':set(pop['paired_rotation_ge2deg'])};jobs=[]
        for f in ['sift','learned']:
            for arm in ['rgb','thermal']:
                for group,ids in groups.items():jobs.append(([r_map[f,arm,i] for i in sorted(ids)],summary[name][f][arm][group]))
        for cell in ['both_correct','rgb_only_correct','thermal_only_correct','neither_correct']:
            ids=[r['index'] for r in cases[name] if r['endpoint']=='R5' and r['cell']==cell]
            for arm in ['rgb','thermal']:jobs.append(([r_map['learned',arm,i] for i in ids],summary[name]['learned_R5_strata'][cell][arm]))
        for rr,d in jobs:
            available=[r for r in rr if r['estimated_deg'] is not None];compared=[r for r in rr if r['error_deg'] is not None];expected={'anchors':len(rr),'outputs':len(available),'reference_eligible':sum(r['index'] in common for r in rr),'compared_outputs':len(compared),'near_identity_outputs':sum(r['near_identity'] for r in available)}
            for key,t in [('R1',1),('R5',5)]:expected.update({key+'_correct':sum(r['error_deg']<=t for r in compared),key+'_wrong_supported':sum(r['error_deg']>t for r in compared),key+'_wrong_near_identity':sum(r['error_deg']>t and r['near_identity'] for r in compared),'identity_'+key+'_correct':sum(r['reference_deg'] is not None and r['reference_deg']<=t for r in rr if r['index'] in common)})
            for key,v in expected.items():check(d[key]==v,'aggregate '+key)
            for field in ['estimated_deg','reference_deg','error_deg','motion_ratio','error_minus_identity_deg']:
                values=[r[field] for r in (compared if field=='reference_deg' else rr) if r[field] is not None];q=quartiles(values);check(d[field]['n']==len(values),'quartile n');check(d[field]['quartiles'] is None if q is None else len(d[field]['quartiles'])==3 and all(abs(x-y)<1e-7 for x,y in zip(d[field]['quartiles'],q)),'independent quartiles')
    result={'status':'pass','checks':count,'rows':sum(len(v) for v in rows.values()),'auditor_sha256':sha(__file__),'scope':'Independent trace/skew angular amount, source/status/reference/identity/triangle inequalities and full cohort/stratum aggregates; no physical truth or deployable detector claim'};(out/'audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return result
if __name__=='__main__':
    import sys
    main(Path(sys.argv[1]))
