"""Independent analytic distortion/subsets, quaternion gyro and paired audit."""
import json,hashlib,heapq,math
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation
from audit_subt_gyro_consistency import gyro_quaternion
from audit_subt_export import mul,inv,angle
ROOT=Path(__file__).resolve().parents[1];VARIANTS=['original','bottom','count'];POLICIES=['rgb','thermal','rgb_first','thermal_first'];FIELDS=['status','matches','E','essential_mask','cheirality_inliers','R','t','pose_mask','error']
def read(p):return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def lines(p):return [json.loads(s) for s in Path(p).read_text().splitlines()]
def native_pixels(points,c):
    p=np.asarray(points,float).reshape(-1,2)
    if not len(p):return p
    x,y=((p-np.array([c['image_width'],c['image_height']])/2)/400).T;xi=c.get('mirror_parameters',{}).get('xi',0);den=1+xi*np.sqrt(1+x*x+y*y);x=x/den;y=y/den;d=c['distortion_parameters'];r2=x*x+y*y;rad=1+d['k1']*r2+d['k2']*r2*r2;xx=x*rad+2*d['p1']*x*y+d['p2']*(r2+2*x*x);yy=y*rad+d['p1']*(r2+2*y*y)+2*d['p2']*x*y;k=c['projection_parameters'];return np.column_stack([k.get('fx',k.get('gamma1'))*xx+k.get('cx',k.get('u0')),k.get('fy',k.get('gamma2'))*yy+k.get('cy',k.get('v0'))])
def crop_indices(p,c):
    n=p.get('matches',0);keep=np.ones(n,bool);h,w=c['image_height'],c['image_width']
    for field in ['points0','points1']:
        uv=native_pixels(p.get(field,[]),c);assert len(uv)==n and np.isfinite(uv).all();assert (uv>=-.05).all() and (uv[:,0]<=w-1+.05).all() and (uv[:,1]<=h-1+.05).all();uv=np.round(uv,4);uv=np.clip(uv,[0,0],[w-1,h-1]);y=np.floor(uv[:,1]+.5).astype(int);keep&=y<2*h/3
    return np.flatnonzero(keep).astype('<i8')
def rank(n,k,f,a,i):
    prefix=f'pre-E-matched-count-v1|seed=0|OutdoorNight|{f}|{a}|{i}|';return np.asarray(sorted(heapq.nsmallest(k,range(n),key=lambda j:(hashlib.sha256((prefix+str(j)).encode()).digest(),j))),dtype='<i8')
def main(config):
    cfg=read(config);out=ROOT/cfg['output'];parent=ROOT/cfg['matching'];gyro=ROOT/cfg['gyro'];mcfg=read(ROOT/cfg['comparison_config']);qual=ROOT/mcfg['qualification'];cams=yaml.safe_load((ROOT/mcfg['intrinsics']).read_text());checks=0
    def check(v,label):
        nonlocal checks
        assert v,label
        checks+=1
    freeze=read(out/'input-freeze.json')
    for p,h in freeze['pins'].items():check(sha(ROOT/p)==h,'frozen '+p)
    n=freeze['anchors'];check(n==971 and freeze['expected_rows']==11652,'full fixed population');baseline=read(out/'baseline-parity.json');check(baseline['status']=='pass' and baseline['rows']==3884 and baseline['numpy']=='2.2.6' and baseline['opencv']=='5.0.0','exact replay receipt');lock=read(out/'prediction-lock.json');check(sha(out/'predictions.jsonl')==lock['predictions_sha256'] and sha(out/'baseline-parity.json')==lock['baseline_parity_sha256'] and lock['gyro_disagreements_computed'] is False and lock['new_inference_calls']==0,'predictions before errors');dl=read(out/'choice-lock.json');check(sha(out/'choices.json')==dl['choices_sha256'] and dl['gyro_errors_used'] is False,'choices before errors');done=read(out/'completion.json')
    for name in ['disagreements','summary','controls','transitions']:check(sha(out/(name+'.json'))==done[name+'_sha256'],'locked '+name)
    inputs=read(parent/'matcher-inputs.json');im={(p['arm'],p['index']):p for p in inputs};images={p['path']:p for p in read(qual/'images.json')}
    check(len(inputs)==len(im)==1942,'all endpoint bindings')
    for path,shape in freeze['image_shapes'].items():
        a='thermal' if '/thermal/' in path else 'rgb';c=cams[a+'_camera'];image=np.load(ROOT/path,allow_pickle=False);check(list(image.shape)==shape==[c['image_height'],c['image_width']] and image.dtype==np.uint8 and freeze['pins'][path]==images[path]['sha256'],'all own image shapes/hashes')
    check(set(freeze['image_shapes'])=={p[k] for p in inputs for k in ['source','target']},'every declared endpoint checked')
    raw=lines(parent/'predictions.jsonl');rows=lines(out/'predictions.jsonl');pred={(p['variant'],p['frontend'],p['arm'],p['index']):p for p in rows};plan=read(out/'preflight.json');pm={(p['variant'],p['frontend'],p['arm'],p['index']):p for p in plan};check(len(raw)==3884 and len(rows)==len(pred)==len(plan)==len(pm)==lock['rows']==11652,'unique complete rows');calls=reuses=inherited=0
    for p in raw:
        f,a,i=p['frontend'],p['arm'],p['index'];k=crop_indices(p,cams[a+'_camera']);sets={'original':np.arange(p.get('matches',0),dtype='<i8'),'bottom':k,'count':rank(p.get('matches',0),len(k),f,a,i)};seen={}
        for v in VARIANTS:
            key=v,f,a,i;r=pred[key];pre=pm[key];ids=sets[v];digest=hashlib.sha256(ids.tobytes()).hexdigest();check(r['matches']==pre['kept']==len(ids) and r['kept_sha256']==pre['kept_sha256']==digest,'independent native/rank/count/order');check(r['qualified']==pre['qualified']==p['qualified']==im[a,i]['qualified'] and r['input_matches']==pre['input_matches']==p.get('matches',0) and pre['fit_eligible']==bool(p['qualified'] and len(ids)>=8),'same source and fit qualification');check(r['seconds']==p['seconds'],'inherited matching time separate from geometry')
            fields={k:r[k] for k in FIELDS if k in r};prev=next((x for x in seen if np.array_equal(sets[x],ids)),None)
            if v=='original':check(fields=={k:p[k] for k in FIELDS if k in p} and r['origin']=='parent' and not r['solver_called'] and r['reuse_variant'] is None and r['geometry_seconds']==0,'original unchanged');inherited+=1
            elif prev is not None:check(fields==seen[prev] and r['origin']=='cache' and not r['solver_called'] and r['reuse_variant']==prev and r['geometry_seconds']==0,'identical subset cache');reuses+=1
            else:check(r['origin']=='fresh' and r['solver_called']==bool(p['qualified'] and len(ids)>=8) and r['reuse_variant'] is None,'unchanged solver gate');calls+=r['solver_called']
            seen[v]=fields;check(math.isfinite(r['geometry_seconds']) and r['geometry_seconds']>=0,'finite timing')
            if not p['qualified']:check(r['status']=='input_unsupported' and not r['solver_called'],'unsupported retained')
            elif len(ids)<8:check(r['status']=='insufficient_matches','min-match gate')
            if 'pose_mask' in r:
                em=np.asarray(r['essential_mask']).reshape(-1)!=0;ch=np.asarray(r['pose_mask']).reshape(-1)!=0;check(len(em)==len(ch)==len(ids) and not np.any(ch&~em) and ch.sum()==r['cheirality_inliers'] and (r['status']=='ok')==(ch.sum()>=8),'mask/positive-depth gate');R=np.asarray(r['R']);t=np.asarray(r['t']);check(np.isfinite(R).all() and np.isfinite(t).all() and np.linalg.norm(R.T@R-np.eye(3))<1e-5 and abs(np.linalg.det(R)-1)<1e-5 and abs(np.linalg.norm(t)-1)<1e-5,'proper R/t');u,_,vt=np.linalg.svd(np.asarray(r['E']));e=u@np.diag([1.,1.,0.])@vt;x,y,z=t;b=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])@R;e/=np.linalg.norm(e);b/=np.linalg.norm(b);check(min(np.linalg.norm(e-b),np.linalg.norm(e+b))<1e-5,'SVD E/R/t')
    check(calls==lock['new_solver_calls'] and reuses==lock['new_reused_rows'] and inherited==lock['inherited_rows'],'all origin receipts')
    labels=read(gyro/'gyro-labels.json');refs={(p['arm'],p['index']):p for p in labels};imu=np.load(qual/'export-imu.npz');times=imu['time_ns'];rates=imu['values'][:,4:7];qs={}
    for key,p in im.items():
        ref=refs[key];check(all(p[k]==ref[k] for k in ['arm','index','source','target','times_ns','qualified']),'gyro exact endpoints');q=gyro_quaternion(times,rates,*p['times_ns']);check(ref['supported']==(q is not None),'independent gyro support')
        if q is not None:check(np.max(abs(Rotation.from_quat(inv(q)).as_matrix()-np.asarray(ref['target_from_source_imu'])))<1e-10 and abs(angle(q)-ref['rotation_deg'])<1e-6,'independent gyro integration/direction');qs[key]=q
    common={i for i in range(n) if all(im[a,i]['qualified'] and (a,i) in qs for a in ['rgb','thermal'])};moving={i for i in common if all(angle(qs[a,i])>=2 for a in ['rgb','thermal'])};pop=read(gyro/'population.json');check(pop=={'common_gyro':sorted(common),'paired_rotation_ge2deg':sorted(moving)} and len(common)==964 and len(moving)==242,'original fixed cohorts');groups={'all_declared':set(range(n)),'common_gyro':common,'paired_rotation_ge2deg':moving,**{'third'+str(j+1):{i for i in common if i*3//n==j} for j in range(3)}};cal=read(qual/'calibration.json');ds=read(out/'choices.json');dm={(d['variant'],d['frontend'],d['index']):d for d in ds};check(len(ds)==len(dm)==dl['rows']==5826,'complete choices')
    for (v,f,i),d in dm.items():
        for policy in POLICIES:
            order=[policy] if policy in ['rgb','thermal'] else ['rgb','thermal'] if policy=='rgb_first' else ['thermal','rgb'];request=[];sensor=None
            for a in order:
                p=pred[v,f,a,i]
                if p['qualified']:request.append(a)
                if p['status']=='ok':sensor=a;break
            endpoints={k:im[sensor,i][k] for k in ['source','target','times_ns']} if sensor else None;check(d['policies'][policy]=={'sensor':sensor,'requested':request,'endpoints':endpoints},'independent policies/endpoints')
    saved=read(out/'disagreements.json');sc={(p['variant'],p['frontend'],p['arm'],p['index']):p for p in saved};check(len(saved)==len(sc)==11652,'all disagreements');errors={};original={(p['frontend'],p['arm'],p['index']):p for p in read(gyro/'disagreements.json')}
    for key,p in pred.items():
        v,f,a,i=key;q=qs.get((a,i));err=None
        if p['status']=='ok' and q is not None:
            rc=Rotation.from_matrix(np.asarray(cal[a]['effective_camera_to_imu'])[:3,:3]).as_quat();estimate=mul(mul(rc,Rotation.from_matrix(p['R']).as_quat()),inv(rc));err=angle(mul(estimate,q))
        r=sc[key];check(r['status']==p['status'] and r['input_supported']==p['qualified'] and r['gyro_supported']==(q is not None),'disagreement input status');check(r['disagreement_deg'] is None if err is None else abs(r['disagreement_deg']-err)<1e-6,'independent gyro error');errors[key]=err
        if v=='original':check({k:r[k] for k in r if k!='variant'}==original[f,a,i],'original exact gyro score')
    summary=read(out/'summary.json');controls=read(out/'controls.json');changes=read(out/'transitions.json');prior=read(ROOT/'results/camera-priority-controls/run-001/summary.json');check(summary['original']==prior,'entire original policy summary parity')
    for v in VARIANTS:
        check(summary[v]['pose_accuracy'] is None,'nominal tier')
        for f in ['sift','learned']:
            for group,ids in groups.items():
                eligible=ids&common;selected=[dm[v,f,i] for i in ids]
                for policy in POLICIES:
                    picked=[(i,dm[v,f,i]['policies'][policy]['sensor']) for i in ids];ee=[errors[v,f,a,i] for i,a in picked if a and i in common];called=[pred[v,f,a,d['index']] for d in selected for a in d['policies'][policy]['requested']];r=summary[v]['frontends'][f][group][policy];check(r['anchors']==len(ids) and r['outputs']==sum(a is not None for i,a in picked) and r['gyro_eligible']==len(eligible) and r['compared_outputs']==len(ee),'summary populations');check(r['within1deg']==sum(e<=1 for e in ee) and r['within5deg']==sum(e<=5 for e in ee) and r['pose_accuracy'] is None,'summary agreements');check(r['conditional_median_deg'] is None if not ee else abs(r['conditional_median_deg']-np.median(ee))<1e-6,'summary conditional median');check(r['matcher_calls']==len(called) and r['projected_seconds']==math.fsum(p['seconds'] for p in called),'cached projection, not new calls')
                for t in [1,5]:
                    good={a:{i for i in eligible if pred[v,f,a,i]['status']=='ok' and errors[v,f,a,i]<=t} for a in ['rgb','thermal']};both={i for i in eligible if all(pred[v,f,a,i]['status']=='ok' for a in ['rgb','thermal'])};union=good['rgb']|good['thermal'];best=max(map(len,good.values()));expected={'gyro_eligible':len(eligible),'rgb_agrees':len(good['rgb']),'thermal_agrees':len(good['thermal']),'best_fixed':best,'oracle_either_agrees':len(union),'oracle_increment_over_best_fixed':len(union)-best,'both_supported':len(both),'rgb_disagrees_thermal_agrees':len(both&(good['thermal']-good['rgb'])),'thermal_disagrees_rgb_agrees':len(both&(good['rgb']-good['thermal'])),'both_supported_neither_agrees':len(both-union),'deployable':False};check(summary[v]['frontends'][f][group]['headroom'][str(t)]==expected,'all oracle descriptive fields')
                ctrl=controls[v][f][group];zero={a:{str(t):sum(angle(qs[a,i])<=t for i in eligible) for t in [1,5]} for a in ['rgb','thermal']};added=[i for i in ids if dm[v,f,i]['policies']['rgb_first']['sensor']=='thermal'];valid=set(added)&eligible;expected={'outputs':len(added),'gyro_eligible':len(valid),**{'within'+str(t)+'deg':sum(errors[v,f,'thermal',i]<=t for i in valid) for t in [1,5]},**{'zero_within'+str(t)+'deg':sum(angle(qs['thermal',i])<=t for i in valid) for t in [1,5]}};check(ctrl=={'zero_rotation':zero,'added_thermal':expected},'same-endpoint zero/addition controls')
    for before,after in [('original','count'),('original','bottom'),('count','bottom')]:
        for f in ['sift','learned']:
            for group,ids in groups.items():
                for policy in POLICIES:
                    def selected(v,i):
                        a=dm[v,f,i]['policies'][policy]['sensor'];return a,errors[v,f,a,i] if a else None
                    pp=[(selected(before,i),selected(after,i),i) for i in ids];expected={'anchors':len(ids),'gained_outputs':sum(a[0] is None and b[0] is not None for a,b,i in pp),'lost_outputs':sum(a[0] is not None and b[0] is None for a,b,i in pp)}
                    for t in [1,5]:
                        bits=[(a[1] is not None and a[1]<=t,b[1] is not None and b[1]<=t) for a,b,i in pp if i in common];expected[str(t)]={'eligible':len(bits),'before_agrees':sum(a for a,b in bits),'after_agrees':sum(b for a,b in bits),'gained':sum(not a and b for a,b in bits),'lost':sum(a and not b for a,b in bits)}
                    check(changes[before+'_to_'+after][f][group][policy]==expected,'all paired transitions')
    result={'status':'pass','checks':checks,'geometry_rows':len(rows),'choice_rows':len(ds),'common_gyro':len(common),'paired_moving':len(moving),'auditor_sha256':sha(__file__),'scope':'Independent native distortion/hash subset, quaternion gyro, SVD pose, original parity, all policies/cohorts/zero controls/paired changes. Nominal agreement only.'};(out/'audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return result
if __name__=='__main__':
    import sys
    main(Path(sys.argv[1]))
