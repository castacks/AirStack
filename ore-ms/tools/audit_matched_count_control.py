"""Independent hash-ranked count/control/pose/reference audit."""
import hashlib,json,heapq,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
VARIANTS=['original','bottom','count']
FIELDS=['status','matches','E','essential_mask','cheirality_inliers','R','t','pose_mask','error']
def read(p):return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def lines(p):return [json.loads(s) for s in Path(p).read_text().splitlines()]
def angle(r):return math.degrees(math.atan2(float(np.linalg.norm([r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1]]))/2,float((np.trace(r)-1)/2)))
def rank_indices(n,k,name,f,a,i):
    assert 0<=k<=n
    if k==n:return np.arange(n,dtype='<i8')
    prefix='|'.join(['pre-E-matched-count-v1','seed=0',name,f,a,str(i)])+'|'
    selected=heapq.nsmallest(k,range(n),key=lambda j:(hashlib.sha256((prefix+str(j)).encode()).digest(),j));return np.array(sorted(selected),dtype='<i8')
def main(config):
    cfg=read(config);out=ROOT/cfg['output'];checks=0
    def check(v,label):
        nonlocal checks
        assert v,label
        checks+=1
    freeze=read(out/'input-freeze.json')
    for p,h in freeze['pins'].items():check(sha(ROOT/p)==h,'frozen '+p)
    prior=ROOT/cfg['parent'];check(read(prior/'audit.json')['status']=='pass' and read(prior/'audit.json')['original_raw_E_equality_gate']=='fail_retained','parent corrected contract/history');parent=lines(prior/'predictions.jsonl');controls={(r['recording'],r['variant'],r['frontend'],r['arm'],r['index']):r for r in parent if r['variant'] in ['original','bottom']};oldplan={(r['recording'],r['variant'],r['frontend'],r['arm'],r['index']):r for r in read(prior/'preflight.json')};oldscore={(r['recording'],r['variant'],r['frontend'],r['arm'],r['index']):r for r in read(prior/'scores.json')};raw=lines(out/'predictions.jsonl');scores=read(out/'scores.json');pred={(r['recording'],r['variant'],r['frontend'],r['arm'],r['index']):r for r in raw};sm={(r['recording'],r['variant'],r['frontend'],r['arm'],r['index']):r for r in scores};plan=read(out/'preflight.json');pm={(r['recording'],r['variant'],r['frontend'],r['arm'],r['index']):r for r in plan};lock=read(out/'prediction-lock.json');check(sha(out/'predictions.jsonl')==lock['predictions_sha256'] and lock['reference_values_parsed'] is False and lock['new_learned_inference_calls']==0,'locked predictions before labels');check(len(raw)==len(pred)==len(scores)==len(sm)==len(plan)==len(pm)==lock['rows']==freeze['expected_rows'],'full unique variant population');decisions=read(out/'choices.json');dm={(r['recording'],r['variant'],r['frontend'],r['index']):r for r in decisions};dlock=read(out/'choice-lock.json');check(sha(out/'choices.json')==dlock['choices_sha256'] and dlock['reference_errors_used'] is False,'status choices before labels');done=read(out/'completion.json')
    for file in ['scores','summary','transitions']:check(sha(out/(file+'.json'))==done[file+'_sha256'],'completion '+file)
    check(sha(out/'crop-count-contrast.json')==read(out/'contrast-lock.json')['crop_count_contrast_sha256'],'paired contrast lock');calls=0;reuse=0;inherited=0;pops={};inputs={}
    for name,c in cfg['records'].items():
        source=lines(ROOT/c['matching']/'predictions.jsonl');pops[name]=read(ROOT/c['qualification']/'population.json');refs={(r['arm'],r['index']):r for r in read(ROOT/c['qualification']/'intervals.json')};inputs[name]={(r['arm'],r['index']):r for r in read(ROOT/c['matching']/'matcher-inputs.json')};common=set(pops[name]['common_reference']);trans=set(pops[name]['translation_ge1cm_both'])
        for p in source:
            f,a,i=p['frontend'],p['arm'],p['index'];n=p.get('matches',0);target=oldplan[name,'bottom',f,a,i]['kept'];ids=rank_indices(n,target,name,f,a,i);digest=hashlib.sha256(ids.tobytes()).hexdigest();base=name,f,a,i
            for variant in VARIANTS:
                key=name,variant,f,a,i;r=pred[key];pre=pm[key]
                if variant in ['original','bottom']:
                    old=controls[key];check(r==dict(old,origin='parent'),'immutable parent control row');check(pre['kept']==old['matches']==oldplan[key]['kept'] and pre['kept_sha256']==r['kept_sha256']==oldplan[key]['kept_sha256'],'parent subset plan parity');inherited+=1
                else:
                    check(r['matches']==pre['kept']==target and r['kept_sha256']==pre['kept_sha256']==digest,'independent hash-rank/order/count');reference_variant=next((v for v in ['original','bottom'] if controls[name,v,f,a,i]['kept_sha256']==digest),None)
                    if reference_variant:
                        old=controls[name,reference_variant,f,a,i];check(r['origin']=='cache' and r['reused'] and r['reuse_variant']==reference_variant and not r['solver_called'] and {k:r[k] for k in FIELDS if k in r}=={k:old[k] for k in FIELDS if k in old},'exact identical-subset cache');reuse+=1
                    else:check(r['origin']=='fresh' and not r['reused'] and r['reuse_variant'] is None and r['solver_called']==bool(p['qualified'] and target>=8),'fresh count solver receipt');calls+=r['solver_called']
                check(r['qualified']==pre['qualified']==p['qualified'] and r['input_matches']==pre['input_matches']==n and pre['fit_eligible']==bool(p['qualified'] and r['matches']>=8),'same input qualification/counts')
                if not p['qualified']:check(r['status']=='input_unsupported' and not r['solver_called'],'unsupported stays unsupported')
                elif r['matches']<8:check(r['status']=='insufficient_matches','unchanged minimum-match gate')
                if 'pose_mask' in r:
                    em=np.asarray(r['essential_mask']).reshape(-1)!=0;ch=np.asarray(r['pose_mask']).reshape(-1)!=0;check(len(em)==len(ch)==r['matches'] and not np.any(ch&~em) and ch.sum()==r['cheirality_inliers'],'subset masks/count');check((r['status']=='ok')==(r['cheirality_inliers']>=8),'same positive-depth gate');R=np.asarray(r['R']);t=np.asarray(r['t']);check(np.isfinite(R).all() and np.max(abs(R.T@R-np.eye(3)))<1e-5 and abs(np.linalg.det(R)-1)<1e-5 and abs(np.linalg.norm(t)-1)<1e-5,'finite proper R/t');u,_,vt=np.linalg.svd(np.asarray(r['E']));e=u@np.diag([1.,1.,0.])@vt;x,y,z=t;b=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])@R;e/=np.linalg.norm(e);b/=np.linalg.norm(b);check(min(np.linalg.norm(e-b),np.linalg.norm(e+b))<1e-5,'SVD E/R/t contract')
                check(math.isfinite(r['geometry_seconds']) and r['geometry_seconds']>=0,'finite time');s=sm[key];check(all(s[k]==r[k] for k in ['recording','variant','frontend','arm','index','status']),'score source key');err=et=None
                if i in common and r['status']=='ok':
                    truth=np.asarray(refs[a,i]['target_from_source']);err=angle(np.asarray(r['R'])@truth[:3,:3].T)
                    if i in trans:
                        t=np.asarray(r['t']);b=truth[:3,3];et=math.degrees(math.atan2(float(np.linalg.norm(np.cross(t,b))),float(t@b)))
                check(s['R_error'] is None if err is None else abs(s['R_error']-err)<1e-5,'own rotation error');check(s['t_error'] is None if et is None else abs(s['t_error']-et)<1e-5,'own translation error');check(s['R1']==bool(err is not None and err<=1) and s['R5']==bool(err is not None and err<=5) and s['joint']==bool(err is not None and et is not None and err<=5 and et<=15),'fixed error thresholds')
                if variant in ['original','bottom']:check(s==oldscore[key],'exact inherited original/crop score')
    check(calls==lock['new_solver_calls'] and reuse==lock['new_reused_rows'] and inherited==lock['inherited_parent_rows'],'all origins/calls counted');check(len(decisions)==len(dm)==dlock['rows']==sum(freeze['anchors'].values())*6,'complete choices')
    for (name,v,f,i),d in dm.items():
        for policy in ['rgb','thermal','rgb_first','thermal_first']:
            order=[policy] if policy in ['rgb','thermal'] else ['rgb','thermal'] if policy=='rgb_first' else ['thermal','rgb'];sensor=None;request=[]
            for a in order:
                p=pred[name,v,f,a,i]
                if p['qualified']:request.append(a)
                if p['status']=='ok':sensor=a;break
            endpoints={k:inputs[name][sensor,i][k] for k in ['source','target','times_ns']} if sensor else None;check(d['policies'][policy]=={'sensor':sensor,'requested':request,'endpoints':endpoints},'independent policy/frame/time binding')
    summary=read(out/'summary.json');transitions=read(out/'transitions.json');contrast=read(out/'crop-count-contrast.json');parent_summary=read(prior/'summary.json')
    for name,n in freeze['anchors'].items():
        common=set(pops[name]['common_reference']);trans=set(pops[name]['translation_ge1cm_both']);groups={'all_declared':set(range(n)),'common_reference':common,'paired_rotation_ge2deg':set(pops[name]['paired_rotation_ge2deg']),'translation_ge1cm_both':trans,**{'third'+str(j+1):{i for i in common if i*3//n==j} for j in range(3)}}
        for variant in VARIANTS:
            for f in ['sift','learned']:
                for group,ids in groups.items():
                    for policy in ['rgb','thermal','rgb_first','thermal_first']:
                        def pick(v,i):
                            a=dm[name,v,f,i]['policies'][policy]['sensor'];return sm[name,v,f,a,i] if a else None
                        pairs=[(pick('original',i),pick(variant,i),i) for i in ids];expect={'anchors':len(ids),'outputs':sum(b is not None for a,b,i in pairs),'rotation_eligible':len(ids&common),'translation_eligible':len(ids&trans)};delta={'new_outputs':sum(a is None and b is not None for a,b,i in pairs),'lost_outputs':sum(a is not None and b is None for a,b,i in pairs)}
                        for field,eligible in [('R1',common),('R5',common),('joint',trans)]:
                            bits=[(bool(a and a[field]),bool(b and b[field])) for a,b,i in pairs if i in eligible];expect[field]=sum(b for a,b in bits);delta[field]={'eligible':len(bits),'gained':sum(not a and b for a,b in bits),'lost':sum(a and not b for a,b in bits),'retained_correct':sum(a and b for a,b in bits)}
                        check(summary[name][variant][f][group][policy]==expect and transitions[name][variant][f][group][policy]==delta,'full cohort/original transition')
                        if variant in ['original','bottom']:check(expect==parent_summary[name][variant][f][group][policy],'all inherited summary parity')
                        if variant=='count':
                            pairs=[(pick('count',i),pick('bottom',i),i) for i in ids];expect={'anchors':len(ids),'crop_new_outputs':sum(a is None and b is not None for a,b,i in pairs),'crop_lost_outputs':sum(a is not None and b is None for a,b,i in pairs)}
                            for field,eligible in [('R1',common),('R5',common),('joint',trans)]:
                                bits=[(bool(a and a[field]),bool(b and b[field])) for a,b,i in pairs if i in eligible];expect[field]={'eligible':len(bits),'count_correct':sum(a for a,b in bits),'crop_correct':sum(b for a,b in bits),'crop_gained':sum(not a and b for a,b in bits),'crop_lost':sum(a and not b for a,b in bits)}
                            check(contrast[name][f][group][policy]==expect,'paired count→crop contrast')
    result={'status':'pass','checks':checks,'geometry_rows':len(raw),'new_count_rows':freeze['expected_rows']//3,'choice_rows':len(decisions),'auditor_sha256':sha(__file__),'scope':'Independent hash ranking/counts/parent identity, SVD geometry, status policies/own-reference errors, all cohorts and paired count-crop contrasts; one fixed draw, no causal rig-only or physical-truth claim'};(out/'audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return result
if __name__=='__main__':
    import sys
    main(Path(sys.argv[1]))
