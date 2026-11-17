"""Independent rays, plane residuals and SVD candidate-set audit."""
import json,hashlib,math
from pathlib import Path
import numpy as np,yaml
ROOT=Path(__file__).resolve().parents[1]
def read(p):return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def angle(r):return math.degrees(math.atan2(float(np.linalg.norm([r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1]]))/2,float((np.trace(r)-1)/2)))
def percentile(v,q):
    v=sorted(v);x=(len(v)-1)*q;l=int(x);h=math.ceil(x);return v[l]+(v[h]-v[l])*(x-l)
def main(config):
    cfg=read(config);out=ROOT/cfg['output'];checks=0
    def check(v,label):
        nonlocal checks
        assert v,label
        checks+=1
    def close(x,y,label):check(x is None if y is None else x is not None and abs(x-y)<1e-5,label)
    freeze=read(out/'input-freeze.json')
    for p,h in freeze['pins'].items():check(sha(ROOT/p)==h,'frozen '+p)
    lock=read(out/'feature-lock.json');check(sha(out/'features.json')==lock['features_sha256'] and lock['reference_values_parsed'] is False,'feature lock');done=read(out/'completion.json');check(done['new_inference_calls']==done['new_E_or_pose_fit_calls']==0,'fixed-cache semantics')
    for file in ['rows','summary']:check(sha(out/(file+'.json'))==done[file+'_sha256'],'locked '+file)
    features=read(out/'features.json');rows=read(out/'rows.json');summary=read(out/'summary.json');cases=read(ROOT/cfg['strata']/'cases.json');modes=['accepted','essential','cheirality'];fields=['ray_change','estimated_warp','estimated_epipolar','reference_warp','reference_epipolar']
    for name,c in cfg['records'].items():
        raw={(r['frontend'],r['arm'],r['index']):r for r in [json.loads(l) for l in (ROOT/c['matching']/'predictions.jsonl').read_text().splitlines()]};refs={(r['arm'],r['index']):r for r in read(ROOT/c['qualification']/'intervals.json')};pop=read(ROOT/c['qualification']/'population.json');ids=set(pop['paired_rotation_ge2deg']);trans=set(pop['translation_ge1cm_both']);intr=yaml.safe_load((ROOT/read(ROOT/c['comparison_config'])['intrinsics']).read_text());scored={(r['frontend'],r['arm'],r['index']):r for r in read(ROOT/c['matching']/'scores.json')};fm={(r['frontend'],r['arm'],r['index']):r for r in features[name]};rm={(r['frontend'],r['arm'],r['index']):r for r in rows[name]};expected={(f,a,i) for f in ['sift','learned'] for a in ['rgb','thermal'] for i in ids};check(set(fm)==set(rm)==expected and len(features[name])==len(rows[name])==len(expected),'complete moving population')
        for key,r in rm.items():
            p=raw[key];f=fm[key];arm=p['arm'];i=p['index'];check(all(r[k]==f[k]==p[k] for k in ['frontend','arm','index','status']),'source/status binding');cam=intr[arm+'_camera'];center=np.array([cam['image_width']/2,cam['image_height']/2]);rays=[]
            for field in ['points0','points1']:
                points=np.asarray(p.get(field,[])).reshape(-1,2);v=np.column_stack([(points-center)/400,np.ones(len(points))]);v/=np.linalg.norm(v,axis=1)[:,None];rays.append(v)
            a,b=rays;n=len(a);check(n==len(b)==p.get('matches',0),'point population');truth=np.array(refs[arm,i]['target_from_source']);e=np.asarray(p.get('E',[]));valid=e.shape==(3,3) and np.isfinite(e).all();check(r['E_valid']==f['E_valid']==valid,'E availability');candidates=[]
            if valid:
                u,_,vt=np.linalg.svd(e);u[:,-1]*=np.linalg.det(u);vt[-1,:]*=np.linalg.det(vt);w=np.array([[0.,-1,0],[1,0,0],[0,0,1]]);candidates=[u@w@vt,u@w.T@vt]
            check(len(r['candidates'])==len(f['candidates'])==len(candidates),'two E candidates')
            if candidates:check(all(min(angle(other@np.asarray(stored['R']).T) for stored in r['candidates'])<1e-4 for other in candidates),'complete independent candidate set')
            for stored,old in zip(r['candidates'],f['candidates']):
                rr=np.asarray(stored['R']);check(stored['R']==old['R'],'locked candidate unchanged');check(min(angle(rr@other.T) for other in candidates)<1e-4,'independent SVD candidate set');close(stored['angle_deg'],angle(rr),'candidate amount');close(stored['error_deg'],angle(rr@truth[:3,:3].T),'candidate full orientation error')
            check(r['translation_eligible']==(i in trans),'reference epipolar eligibility');close(r['reference_angle_deg'],angle(truth[:3,:3]),'reference angle');close(r['output_error_deg'],scored[key]['R_error'],'old score parity')
            if p['status']=='ok':
                check(min(angle(np.asarray(p['R'])@rr.T) for rr in candidates)<1e-4,'recovered R belongs to E');close(r['output_error_deg'],angle(np.asarray(p['R'])@truth[:3,:3].T),'direct output error')
            masks=[np.ones(n,bool),np.asarray(p.get('essential_mask',[0]*n)).reshape(-1)!=0,np.asarray(p.get('pose_mask',[0]*n)).reshape(-1)!=0];check(all(len(m)==n for m in masks) and int(masks[2].sum())==p.get('cheirality_inliers',0),'mask binding')
            t=truth[:3,3];cross=np.array([[0,-t[2],t[1]],[t[2],0,-t[0]],[-t[1],t[0],0]]);eref=cross@truth[:3,:3]
            for mode,mask in zip(modes,masks):
                aa,bb=a[mask],b[mask];d=r['subsets'][mode];old=f['subsets'][mode];check(d['points']==old['points']==len(aa),'subset count');vv={}
                for field in fields:
                    if field in ['ray_change','estimated_warp','reference_warp']:
                        rr=np.eye(3) if field=='ray_change' else truth[:3,:3] if field=='reference_warp' else np.asarray(p['R']) if p['status']=='ok' else None
                        v=np.degrees(np.arctan2(np.linalg.norm(np.cross(aa@rr.T,bb),axis=1),np.sum((aa@rr.T)*bb,axis=1))) if rr is not None else np.array([])
                    else:
                        ee=e if field=='estimated_epipolar' and valid else eref if field=='reference_epipolar' and i in trans else None
                        if ee is None:v=np.array([])
                        else:
                            normals=(ee@aa.T).T;norm=np.sqrt(np.sum(normals*normals,axis=1));keep=norm>1e-12;v=np.degrees(np.arcsin(np.clip(np.abs(np.einsum('ij,ij->i',bb[keep],normals[keep]))/norm[keep],0,1)))
                    check(d[field]['n']==len(v),'metric valid count');close(d[field]['median'],percentile(v,.5) if len(v) else None,'pair median');close(d[field]['p90'],percentile(v,.9) if len(v) else None,'pair p90')
                    if field in ['ray_change','estimated_warp','estimated_epipolar']:check(d[field]==old[field],'feature unchanged after reference join')
        jobs=[]
        for front in ['sift','learned']:
            for arm in ['rgb','thermal']:jobs.append(([r for k,r in rm.items() if k[:2]==(front,arm)],summary[name][front][arm]))
        for cell in ['both_correct','rgb_only_correct','thermal_only_correct','neither_correct']:
            keep={r['index'] for r in cases[name] if r['endpoint']=='R5' and r['cell']==cell}
            for arm in ['rgb','thermal']:jobs.append(([r for k,r in rm.items() if k[:2]==('learned',arm) and k[2] in keep],summary[name]['learned_R5_strata'][cell][arm]))
        for rs,d in jobs:
            check(d['anchors']==len(rs) and d['outputs']==sum(r['status']=='ok' for r in rs) and d['E_available']==sum(r['E_valid'] for r in rs),'group population')
            for threshold in [1,5]:
                original=sum(r['output_error_deg'] is not None and r['output_error_deg']<=threshold for r in rs);either=sum(any(c['error_deg']<=threshold for c in r['candidates']) for r in rs);alternative=sum(r['output_error_deg'] is not None and r['output_error_deg']>threshold and any(c['error_deg']<=threshold for c in r['candidates']) for r in rs);check(d['R'+str(threshold)]=={'original_correct':original,'either_candidate_correct':either,'wrong_output_alternative_correct':alternative},'E branch diagnosis')
            for mode in modes:
                for field in fields:
                    v=[r['subsets'][mode][field]['median'] for r in rs if r['subsets'][mode][field]['median'] is not None];q=d['subsets'][mode][field];check(q['n']==len(v) and q['missing']==len(rs)-len(v),'per-pair missingness');expected=[percentile(v,x) for x in [.25,.5,.75]] if v else None;check(q['quartiles'] is None if expected is None else len(q['quartiles'])==3 and all(abs(x-y)<1e-5 for x,y in zip(q['quartiles'],expected)),'pair-weighted quartiles')
    result={'status':'pass','checks':checks,'rows':sum(len(v) for v in rows.values()),'auditor_sha256':sha(__file__),'scope':'Independent rays/SVD branches/masks/reference geometry/residuals/old-error parity and all aggregates; not causal matcher truth or new policy'};(out/'audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return result
if __name__=='__main__':
    import sys
    main(Path(sys.argv[1]))
