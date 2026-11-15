"""Independent Pillow/world-frame/SciPy audit of the nominal geometry diagnostic."""
import argparse,json,hashlib
from pathlib import Path
from datetime import datetime,timezone
import numpy as np
from PIL import Image
from scipy.ndimage import map_coordinates
from scipy.stats import spearmanr


def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def correlation(a,b):
    return float(spearmanr(a,b).statistic) if len(a)>1 and max(a)>min(a) and max(b)>min(b) else None


def main(plan_path,out):
    plan=read(plan_path); lock=read(out/'prediction-lock.json'); scores=read(out/'scores.json'); checks=0; maximum=0.
    def check(ok,message):
        nonlocal checks
        assert ok,message; checks+=1
    def close(a,b,atol=2e-9):
        nonlocal maximum,checks
        a=np.asarray(a); b=np.asarray(b)
        np.testing.assert_allclose(a,b,rtol=1e-10,atol=atol)
        if a.size:maximum=max(maximum,float(np.max(np.abs(a-b))))
        checks+=1
    for p,h in plan['pins'].items():check(sha(p)==h,p)
    check(lock['forward_calls']==114 and lock['upstream_finite_uncertainty_calls']==1368 and lock['model_state_unchanged'],'inference contract')
    check(lock['pose_values_parsed'] is False and lock['geometry_endpoints_constructed'] is False,'prediction lock phase')
    check(scores['prediction_lock_sha256']==sha(out/'prediction-lock.json'),'score lock hash')
    check(len(lock['rows'])==len(scores['rows'])==57,'all pairs')
    inputs={(r['condition'],r['stem']):r for r in plan['inputs']}; pose_checks=set(); independent=[]
    for index,(prediction,result) in enumerate(zip(lock['rows'],scores['rows'],strict=True)):
        pair=plan['pairs'][index]; c=pair['condition']; a=inputs[c,pair['source']]; b=inputs[c,pair['target']]
        check(all(prediction[k]==result[k]==pair[k] for k in ['condition','source','target']),'pair identity')
        check(sha(out/prediction['path'])==prediction['sha256'],'prediction array hash')
        check(sha(out/result['geometry_path'])==result['geometry_sha256'],'geometry array hash')
        E=np.array(plan['thermal_from_rgb'][c]); Ps=[]
        for row in [a,b]:
            P=np.loadtxt(row['rgb_pose_path']).reshape(4,4); Ps.append(P)
            if row['rgb_pose_path'] not in pose_checks:
                close(P[3],[0,0,0,1]); close(P[:3,:3].T@P[:3,:3],np.eye(3),atol=1e-5);close(np.linalg.det(P[:3,:3]),1,atol=1e-5); pose_checks.add(row['rgb_pose_path'])
        # Construct absolute THERMAL camera poses, then explicitly go via world.
        W0=Ps[0]@np.linalg.inv(E); W1=Ps[1]@np.linalg.inv(E)
        K=np.array(a['K']); K1=np.array(b['K']); close(K,K1)
        source_depth=np.asarray(Image.open(a['depth_path'])); target_depth=np.asarray(Image.open(b['depth_path']))
        points=np.argwhere(source_depth>0); uv=points[:,::-1].astype(float); n=len(points)
        xyz=np.linalg.solve(K,np.vstack([uv.T,np.ones(n)]))*(source_depth[points[:,0],points[:,1]].astype(float)/256)[None]
        homogeneous=np.vstack([xyz,np.ones(n)]); world=W0@homogeneous; local=np.linalg.solve(W1,world)
        valid_z=local[2]>0; projected=K1@local[:3]; endpoint=np.zeros((n,2)); endpoint[valid_z]=(projected[:2,valid_z]/projected[2,valid_z]).T
        inside=valid_z&(endpoint[:,0]>=0)&(endpoint[:,0]<=639)&(endpoint[:,1]>=0)&(endpoint[:,1]<=255)
        nearest=np.floor(endpoint+.5).astype(int)
        # Independent scalar dictionary nearest-depth rasterization, stable ties by source index.
        cells={}
        for j in np.flatnonzero(inside):
            key=tuple(nearest[j]); previous=cells.get(key)
            if previous is None or local[2,j]<local[2,previous]: cells[key]=int(j)
        m1=np.zeros(n,bool); m1[list(cells.values())]=True
        depth=np.zeros(n); ids=np.flatnonzero(m1)
        depth[ids]=target_depth[nearest[ids,1],nearest[ids,0]].astype(float)/256
        m2=m1&(depth>0); agree=np.abs(local[2]-depth)<=.1+.02*depth
        backward=np.full(n,np.inf)
        for j in np.flatnonzero(m2):
            x=np.linalg.solve(K1,np.r_[nearest[j],1.])*depth[j]
            world_point=W1@np.r_[x,1.]; source_point=np.linalg.solve(W0,world_point)
            if source_point[2]>0:
                pixel=K@source_point[:3]; backward[j]=np.linalg.norm(pixel[:2]/pixel[2]-uv[j])
        m3=m2&agree&(backward<=1)
        masks=np.array([m1,m2,m3]); coord=uv/2-.25
        sampling=(coord[:,0]>=0)&(coord[:,0]<=319)&(coord[:,1]>=0)&(coord[:,1]<=127)
        reference=(endpoint-uv)*.5
        expected_counts={'source_depth':n,'behind_camera':int((~valid_z).sum()),'outside_target':int((valid_z&~inside).sum()),'target_collisions':int(inside.sum()-len(cells)),'tier1':int(m1.sum()),'missing_target_depth':int((m1&~m2).sum()),'tier2':int(m2.sum()),'depth_disagreement':int((m2&~agree).sum()),'backward_reprojection_rejection_after_depth':int((m2&agree&~m3).sum()),'tier3':int(m3.sum())}
        check(expected_counts==result['counts'],'support rejection accounting')
        with np.load(out/result['geometry_path']) as z:
            check(np.array_equal(z['source_uv'],uv),'source UV exact'); close(z['target_uv'],endpoint); close(z['projected_z'],local[2]); close(z['reference_flow'],reference)
            check(np.array_equal(z['tier_masks'],masks),'all three support masks'); check(np.array_equal(z['sampling_mask'],sampling),'sampling eligibility')
            close(z['target_from_source'],np.linalg.solve(W1,W0))
            saved_flow=z['sampled_flow']; saved_alpha=z['sampled_alpha']
        with np.load(out/prediction['path']) as z:
            dense=z['flow'].astype(float); alpha=z['alpha'].astype(float)
            check(dense.shape==(2,2,128,320) and alpha.shape==(2,1,128,320),'output shape')
            check(np.isfinite(dense).all() and np.isfinite(alpha).all(),'outputs finite')
            # Source recipe identity against already locked preceding assay inputs.
            for stem,key in [(pair['source'],'source_uint8'),(pair['target'],'target_uint8')]:
                old=out.parents[1]/'u2flow-thermal-warp/run-001/capture'/(c+'-'+stem+'.npz')
                with np.load(old) as previous:check(np.array_equal(z[key],previous['source_uint8']),'unchanged thermal preparation')
        flow=np.zeros((2,n,2)); unc=np.zeros((2,n))
        for arm in range(2):
            for ch in range(2):flow[arm,sampling,ch]=map_coordinates(dense[arm,ch],coord[sampling,::-1].T,order=1,mode='nearest',prefilter=False)
            unc[arm,sampling]=map_coordinates(alpha[arm,0],coord[sampling,::-1].T,order=1,mode='nearest',prefilter=False)
        close(flow,saved_flow); close(unc,saved_alpha)
        tiers={}
        for ti,tier in enumerate(['tier1','tier2','tier3']):
            mask=masks[ti]&sampling; entry=result['tiers'][tier]
            check(entry['geometry_points']==int(masks[ti].sum()) and entry['scored_points']==int(mask.sum()) and entry['sampling_border_excluded']==int((masks[ti]&~sampling).sum()),'tier populations')
            e=np.linalg.norm(flow[:,mask]-reference[None,mask],axis=-1); consistency=np.linalg.norm(flow[1,mask]-flow[0,mask],axis=-1)
            values={}
            for arm,method in enumerate(['clean','gamma','zero_flow']):
                error=e[arm] if arm<2 else np.linalg.norm(reference[mask],axis=-1)
                expected={'mean_px':float(np.mean(error)) if len(error) else None,'rmse_px':float(np.linalg.norm(error)/np.sqrt(len(error))) if len(error) else None,'agreement1px':float(sum(error<=1)/len(error)) if len(error) else None,'agreement3px':float(sum(error<=3)/len(error)) if len(error) else None,'alpha_disagreement_spearman':correlation(unc[arm,mask],error) if arm<2 else None}
                values[method]=expected
            bad=e[1]>1; wrong=bad&(consistency<.1)
            values['predictive_consistency']={'mean_px':float(np.mean(consistency)) if len(consistency) else None,'vs_clean_geometry_disagreement_spearman':correlation(consistency,e[0]),'vs_gamma_geometry_disagreement_spearman':correlation(consistency,e[1]),'gamma_disagreement_gt1_count':int(sum(bad)),'consistent_disagreement_gt1_count':int(sum(wrong)),'fraction_of_gt1_disagreements_consistent':float(sum(wrong)/sum(bad)) if sum(bad)>0 else None}
            for method,metrics in values.items():
                for key,value in metrics.items():
                    if value is None:check(entry[method][key] is None,'null metric')
                    else:close(entry[method][key],value)
            tiers[tier]={**values,'geometry_points':int(masks[ti].sum()),'scored_points':int(mask.sum()),'sampling_border_excluded':int((masks[ti]&~sampling).sum())}
        independent.append({'condition':c,'tiers':tiers})
    for c in ['day','night','rain']:
      records=[r for r in independent if r['condition']==c]; check(len(records)==19,'19 pair slots')
      for tier in ['tier1','tier2','tier3']:
        aggregate=scores['recordings'][c][tier]; check(aggregate['pairs']==19,'aggregate pairs')
        for key in ['geometry_points','scored_points','sampling_border_excluded']:check(aggregate[key]==sum(r['tiers'][tier][key] for r in records),'aggregate support')
        check(aggregate['supported_pairs']==sum(r['tiers'][tier]['scored_points']>0 for r in records),'supported pairs')
        for method,metrics in aggregate['metrics'].items():
          for key,item in metrics.items():
            v=[r['tiers'][tier][method][key] for r in records if r['tiers'][tier][method][key] is not None]
            check(item['valid_pairs']==len(v),'macro denominator')
            if v:close(item['mean'],np.mean(v))
            else:check(item['mean'] is None,'null macro')
    report={'created_utc':datetime.now(timezone.utc).isoformat(),'status':'PASS','checks':checks,'pairs':57,'pose_files_checked':len(pose_checks),'maximum_numeric_difference':maximum,'scores_sha256':sha(out/'scores.json'),'auditor_sha256':sha(__file__),'scope':'Independent world-frame/Pillow/SciPy arithmetic verifies supplied-geometry mechanics, not physical calibration, timing, visibility or true flow.'}
    with (out/'independent-audit.json').open('x') as f:json.dump(report,f,indent=2); f.write('\n')
    print(json.dumps(report))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--protocol',required=True);p.add_argument('--output',required=True);a=p.parse_args();main(a.protocol,Path(a.output))
