"""Independent image masks, subset binding, geometry and outcome audit."""
import hashlib,json,math
from pathlib import Path
import numpy as np,yaml
from audit_essential_inlier_mixture import project
ROOT=Path(__file__).resolve().parents[1]
VARIANTS=['original','zero','bottom','combined']
FIELDS=['status','matches','E','essential_mask','cheirality_inliers','R','t','pose_mask','error']
def read(p):return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def lines(p):return [json.loads(l) for l in Path(p).read_text().splitlines()]
def angle(r):return math.degrees(math.atan2(float(np.linalg.norm([r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1]]))/2,float((np.trace(r)-1)/2)))
def main(config):
    cfg=read(config);out=ROOT/cfg['output'];checks=0
    def check(v,label):
        nonlocal checks
        assert v,label
        checks+=1
    freeze=read(out/'input-freeze.json')
    for p,h in freeze['pins'].items():check(sha(ROOT/p)==h,'frozen '+p)
    qual=read(out/'mask-qualification.json');check(sha(out/'mask-qualification.json')==read(out/'mask-lock.json')['qualification_sha256'] and read(out/'mask-lock.json')['reference_errors_used'] is False,'image-only mask lock');masks={};sources={};metadata={};populations={};references={};parent_scores={}
    for name,c in cfg['records'].items():
        images=read(ROOT/c['qualification']/'images.json');masks[name]={};metadata[name]=read(ROOT/c['matching']/'matcher-inputs.json');sources[name]=lines(ROOT/c['matching']/'predictions.jsonl');populations[name]=read(ROOT/c['qualification']/'population.json');references[name]={(r['arm'],r['index']):r for r in read(ROOT/c['qualification']/'intervals.json')};parent_scores[name]={(r['frontend'],r['arm'],r['index']):r for r in read(ROOT/c['matching']/'scores.json')}
        for arm in ['rgb','thermal']:
            frames=sorted((r for r in images if r['arm']==arm),key=lambda r:r['index']);selected=[frames[round(j*(len(frames)-1)/4)] for j in range(5)];entry=qual['records'][name][arm];check(entry['samples']==selected,'fixed ordinal image samples');arrays=[]
            for r in selected:check(sha(ROOT/r['path'])==r['sha256'],'own sampled image hash');arrays.append(np.load(ROOT/r['path'],allow_pickle=False))
            h,w=arrays[0].shape;zero=np.ones((h,w),bool) if arm=='thermal' else np.any(np.stack(arrays)!=0,axis=0);bottom=np.tile((np.arange(h)<h*2/3)[:,None],(1,w));expected={'original':np.ones_like(zero),'zero':zero,'bottom':bottom,'combined':np.logical_and(zero,bottom)};masks[name][arm]={}
            for variant,mask in expected.items():
                r=entry['masks'][variant];actual=np.load(ROOT/r['path'],allow_pickle=False);check(actual.dtype==np.bool_ and np.array_equal(actual,mask),'mask reconstructed from pixels/rows');check(sha(ROOT/r['path'])==r['sha256'] and r['kept_pixels']==int(mask.sum()) and r['pixels']==mask.size,'mask receipt');masks[name][arm][variant]=mask
    parity=read(out/'baseline-parity.json');baseline=lines(out/'baseline.jsonl');check(parity['status']=='pass' and sha(out/'baseline.jsonl')==parity['baseline_sha256'],'exact runtime baseline lock');raw=lines(out/'predictions.jsonl');plock=read(out/'prediction-lock.json');check(sha(out/'predictions.jsonl')==plock['predictions_sha256'] and len(raw)==plock['rows']==freeze['expected_rows'],'complete locked geometry');check(plock['new_learned_inference_calls']==0 and plock['reference_values_parsed'] is False,'no inference/reference-based inclusion');decisions=read(out/'choices.json');dlock=read(out/'choice-lock.json');check(sha(out/'choices.json')==dlock['choices_sha256'] and dlock['reference_errors_used'] is False,'choices before errors');done=read(out/'completion.json')
    for file in ['scores','summary','transitions']:check(sha(out/(file+'.json'))==done[file+'_sha256'],'completion '+file)
    scores=read(out/'scores.json');pred={(r['recording'],r['variant'],r['frontend'],r['arm'],r['index']):r for r in raw};sm={(r['recording'],r['variant'],r['frontend'],r['arm'],r['index']):r for r in scores};bm={(r['recording'],r['frontend'],r['arm'],r['index']):r for r in baseline};dm={(d['recording'],d['variant'],d['frontend'],d['index']):d for d in decisions};plan=read(out/'preflight.json');pm={(r['recording'],r['variant'],r['frontend'],r['arm'],r['index']):r for r in plan};check(len(pred)==len(raw)==len(sm)==len(scores)==len(pm)==len(plan)==freeze['expected_rows'],'unique full rows');field_checks=0;baseline_calls=0;calls=0;reuses=0
    for name,c in cfg['records'].items():
        cal=yaml.safe_load((ROOT/read(ROOT/c['comparison_config'])['intrinsics']).read_text());common=set(populations[name]['common_reference']);trans=set(populations[name]['translation_ge1cm_both'])
        for old in sources[name]:
            front,arm,i=old['frontend'],old['arm'],old['index'];n=old.get('matches',0);original=bm[name,front,arm,i];check({k:original[k] for k in FIELDS if k in original}=={k:old[k] for k in FIELDS if k in old},'unmasked exact pose fields');field_checks+=len([k for k in FIELDS if k in old]);check(pred[name,'original',front,arm,i]==original,'original output matches baseline');baseline_calls+=bool(old['qualified'] and n>=8);cam=cal[arm+'_camera'];native=[]
            for field in ['points0','points1']:
                a=project(np.asarray(old.get(field,[]),float).reshape(-1,2),cam);px=np.floor(a+.5).astype(int);h,w=masks[name][arm]['original'].shape;px[:,0]=np.clip(px[:,0],0,w-1);px[:,1]=np.clip(px[:,1],0,h-1);native.append(px)
            cache={}
            for variant in VARIANTS:
                keep=np.ones(n,bool);mask=masks[name][arm][variant]
                for px in native:keep&=mask[px[:,1],px[:,0]]
                ids=np.arange(n,dtype='<i8')[keep];digest=hashlib.sha256(ids.tobytes()).hexdigest();key=name,variant,front,arm,i;r=pred[key];p=pm[key];check(r['kept_sha256']==p['kept_sha256']==digest and r['matches']==p['kept']==len(ids),'both-endpoint subset/order/hash');check(r['qualified']==p['input_qualified']==old['qualified'] and r['input_matches']==p['input_matches']==n and p['fit_eligible']==bool(old['qualified'] and len(ids)>=8),'source/input/fit support')
                if not old['qualified']:check(r['status']=='input_unsupported' and not r['solver_called'],'unsupported input unchanged')
                elif len(ids)<8:check(r['status']=='insufficient_matches','insufficient retained matches explicit')
                if variant=='original':check(not r['reused'] and r['reuse_variant'] is None,'original fresh control')
                else:
                    if digest in cache:
                        previous,stored=cache[digest];check(r['reused'] and not r['solver_called'] and r['reuse_variant']==previous and {k:r[k] for k in FIELDS if k in r}==stored,'identity-subset reuse');reuses+=1
                    else:check(not r['reused'] and r['reuse_variant'] is None and r['solver_called']==bool(old['qualified'] and len(ids)>=8),'fresh masked solver receipt');cache[digest]=(variant,{k:r[k] for k in FIELDS if k in r})
                    calls+=r['solver_called']
                if digest not in cache:cache[digest]=(variant,{k:r[k] for k in FIELDS if k in r})
                check(math.isfinite(r['geometry_seconds']) and r['geometry_seconds']>=0,'finite geometry time')
                if 'pose_mask' in r:
                    e=np.asarray(r['essential_mask']).reshape(-1)!=0;m=np.asarray(r['pose_mask']).reshape(-1)!=0;check(len(e)==len(m)==len(ids) and not np.any(m&~e) and int(m.sum())==r['cheirality_inliers'],'retained mask binding/nesting');check((r['status']=='ok')==(r['cheirality_inliers']>=8),'same cheirality gate');R=np.asarray(r['R']);t=np.asarray(r['t']);check(np.isfinite(R).all() and np.max(abs(R.T@R-np.eye(3)))<1e-5 and abs(np.linalg.det(R)-1)<1e-5 and abs(np.linalg.norm(t)-1)<1e-5,'proper output R/t');tx=np.array([[0,-t[2],t[1]],[t[2],0,-t[0]],[-t[1],t[0],0]]);a=np.asarray(r['E']);b=tx@R;a=a/np.linalg.norm(a);b=b/np.linalg.norm(b);check(min(np.linalg.norm(a-b),np.linalg.norm(a+b))<1e-5,'E-R-t numerical consistency')
                ref=references[name][arm,i];s=sm[key];check(all(s[k]==r[k] for k in ['recording','variant','frontend','arm','index','status']),'score binding');err=et=None
                if i in common and r['status']=='ok':
                    truth=np.asarray(ref['target_from_source']);err=angle(np.asarray(r['R'])@truth[:3,:3].T)
                    if i in trans:
                        t=np.asarray(r['t']);b=truth[:3,3];et=math.degrees(math.atan2(float(np.linalg.norm(np.cross(t,b))),float(t@b)))
                check(s['R_error'] is None if err is None else abs(s['R_error']-err)<1e-5,'own rotation error');check(s['t_error'] is None if et is None else abs(s['t_error']-et)<1e-5,'own directed translation error');check(s['R1']==bool(err is not None and err<=1) and s['R5']==bool(err is not None and err<=5) and s['joint']==bool(err is not None and et is not None and err<=5 and et<=15),'fixed endpoint labels')
                if variant=='original':
                    oldscore=parent_scores[name][front,arm,i]
                    for k in ['R_error','t_error','R1','R5','joint']:check(s[k]==oldscore[k],'exact original score parity '+k)
    check(parity['rows']==len(baseline) and parity['pose_field_checks']==field_checks and parity['solver_calls']==baseline_calls,'all original parity counted');check(plock['additional_solver_calls']==calls and plock['reused_rows']==reuses,'all masked calls/reuse counted');check(len(decisions)==len(dm)==dlock['rows']==sum(freeze['anchors'].values())*8,'full status choice population')
    for (name,variant,front,i),d in dm.items():
        inputs={(r['arm'],r['index']):r for r in metadata[name]}
        for policy in ['rgb','thermal','rgb_first','thermal_first']:
            order=[policy] if policy in ['rgb','thermal'] else ['rgb','thermal'] if policy=='rgb_first' else ['thermal','rgb'];requested=[];sensor=None
            for a in order:
                r=pred[name,variant,front,a,i]
                if r['qualified']:requested.append(a)
                if r['status']=='ok':sensor=a;break
            chosen=d['policies'][policy];expected={k:inputs[sensor,i][k] for k in ['source','target','times_ns']} if sensor else None;check(chosen=={'sensor':sensor,'requested':requested,'endpoints':expected},'independent routing/own endpoints')
    ss=read(out/'summary.json');tt=read(out/'transitions.json')
    for name,n in freeze['anchors'].items():
        common=set(populations[name]['common_reference']);trans=set(populations[name]['translation_ge1cm_both']);groups={'all_declared':set(range(n)),'common_reference':common,'paired_rotation_ge2deg':set(populations[name]['paired_rotation_ge2deg']),'translation_ge1cm_both':trans,**{'third'+str(j+1):{i for i in common if i*3//n==j} for j in range(3)}}
        for variant in VARIANTS:
            for front in ['sift','learned']:
                for group,ids in groups.items():
                    for policy in ['rgb','thermal','rgb_first','thermal_first']:
                        def pick(v,i):
                            a=dm[name,v,front,i]['policies'][policy]['sensor'];return sm[name,v,front,a,i] if a else None
                        pairs=[(pick('original',i),pick(variant,i),i) for i in ids];expected={'anchors':len(ids),'outputs':sum(b is not None for a,b,i in pairs),'rotation_eligible':len(ids&common),'translation_eligible':len(ids&trans)};delta={'new_outputs':sum(a is None and b is not None for a,b,i in pairs),'lost_outputs':sum(a is not None and b is None for a,b,i in pairs)}
                        for field,eligible in [('R1',common),('R5',common),('joint',trans)]:
                            bits=[(bool(a and a[field]),bool(b and b[field])) for a,b,i in pairs if i in eligible];expected[field]=sum(b for a,b in bits);delta[field]={'eligible':len(bits),'gained':sum(not a and b for a,b in bits),'lost':sum(a and not b for a,b in bits),'retained_correct':sum(a and b for a,b in bits)}
                        check(ss[name][variant][front][group][policy]==expected,'complete cohort counts');check(tt[name][variant][front][group][policy]==delta,'paired gains/losses/missing transitions')
    result={'status':'pass','checks':checks,'geometry_rows':len(raw),'score_rows':len(scores),'choice_rows':len(decisions),'auditor_sha256':sha(__file__),'scope':'Independent mask pixels/indices, exact original controls, E-R-t/mask/source bindings, own-reference labels, status policies and every cohort/paired transition; no physical truth or end-to-end latency/adoption certificate'};(out/'audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return result
if __name__=='__main__':
    import sys
    main(Path(sys.argv[1]))
