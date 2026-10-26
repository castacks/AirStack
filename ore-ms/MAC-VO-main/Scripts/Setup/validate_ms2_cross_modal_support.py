"""Independently check saved support/metric/control results against raw inputs."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image


def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def validate(root, base, output):
    if output.exists():raise FileExistsError(output)
    support=base/'run-001';metric=base/'metric-run-001';control=base/'geometry-control-run-001'
    sr=json.loads((support/'report.json').read_text());mr=json.loads((metric/'report.json').read_text());cr=json.loads((control/'report.json').read_text())
    rawbase=root/'results/ms2-rain-replication';pilot=rawbase/'pilot';seq='_2021-08-06-16-19-00'
    cal={k:np.array(v['values'],float) for k,v in json.loads((rawbase/'calibration.json').read_text()).items()}
    # Independent composition uses 4x4 matrices with inversion rather than the producer's direct formula.
    nr=np.eye(4);nt=np.eye(4)
    for h,m in [(nr,'rgb'),(nt,'thr')]:h[:3,:3]=cal[f'R_nir2{m}'];h[:3,3]=cal[f'T_nir2{m}'][:,0]/1000
    rt=nt@np.linalg.inv(nr);tr=np.linalg.inv(rt)
    candidate_points=raw_predictions=sampled_nearest=aggregate_checks=depth_checks=subset_risk_checks=0
    for srow,mrow,crow in zip(sr['rows'],mr['rows'],cr['rows']):
        stem=srow['stem'];assert stem==mrow['stem']==crow['stem']
        with np.load(support/f'pairs-{stem}.npz') as f:p={k:f[k] for k in f.files}
        with np.load(metric/f'metric-{stem}.npz') as f:m={k:f[k] for k in f.files}
        with np.load(control/f'control-{stem}.npz') as f:ctl={k:f[k] for k in f.files}
        n=len(p['rgb_xy']);candidate_points+=n;assert n==srow['stages']['mutual_nearest']==mrow['candidates']
        counts=list(srow['stages'].values());assert all(a>=b for a,b in zip(counts,counts[1:]))
        assert len(set(map(tuple,p['rgb_xy'])))==len(set(map(tuple,p['thr_xy'])))==n
        native={};depths={}
        for mod in ('rgb','thr'):
            k=cal[f'K_{mod}L'];depth=np.asarray(Image.open(pilot/'proj_depth'/seq/mod/'depth'/f'{stem}.png'),float)/256;depths[mod]=depth
            pts=[]
            for x,y in p[f'{mod}_xy']:
                z=float(depth[y,x]);assert z>0;pts.append([(int(x)-float(k[0,2]))*z/float(k[0,0]),(int(y)-float(k[1,2]))*z/float(k[1,1]),z])
            native[mod]=np.array(pts)
        assert np.allclose(native['rgb'],p['rgb_reference_xyz'],rtol=1e-12,atol=1e-10)
        transformed=np.column_stack((native['thr'],np.ones(n)))@tr.T
        assert np.allclose(transformed[:,:3],p['thr_reference_xyz_in_rgb'],rtol=1e-12,atol=1e-10)
        for rel in (0,.01,.02,.05):
            forward=np.column_stack((native['rgb'],np.ones(n)))@rt.T
            reverse=np.column_stack((native['thr'],np.ones(n)))@tr.T
            expected_mask=np.array([abs(float(q[2])-float(b[2]))<=max(2/256,rel*float(b[2])) and abs(float(v[2])-float(a[2]))<=max(2/256,rel*float(a[2])) for q,b,v,a in zip(forward,native['thr'],reverse,native['rgb'])])
            assert np.array_equal(expected_mask,p[f'depth_consistent_{rel:g}']);depth_checks+=1
        # Independent nearest/round-cell visibility checks for fixed evenly spaced candidate samples.
        for source,target,h in [('rgb','thr',rt),('thr','rgb',tr)]:
            k=cal[f'K_{target}L'];xyall=p[f'source_supported_{target}_xy'];sourceall=p[f'source_supported_{source}_xy'];z=depths[source][sourceall[:,1],sourceall[:,0]];ks=cal[f'K_{source}L']
            allp=np.column_stack(((sourceall[:,0]-ks[0,2])*z/ks[0,0],(sourceall[:,1]-ks[1,2])*z/ks[1,1],z,np.ones(len(z))))@h.T
            with np.errstate(divide='ignore',invalid='ignore'):
                alluv=allp[:,:2]/allp[:,2,None]*[k[0,0],k[1,1]]+[k[0,2],k[1,2]]
            dh,dw=depths[target].shape
            inview=(allp[:,2]>0)&(alluv[:,0]>=0)&(alluv[:,0]<=dw-1)&(alluv[:,1]>=0)&(alluv[:,1]<=dh-1)
            for i in np.unique(np.linspace(0,n-1,min(n,3),dtype=int)):
                point=np.append(native[source][i],1);q=[sum(float(h[a,b])*float(point[b]) for b in range(4)) for a in range(3)]
                uv=[float(k[a,a])*q[a]/q[2]+float(k[a,2]) for a in range(2)]
                dist=[math.hypot((int(x)-uv[0])/float(k[0,0]),(int(y)-uv[1])/float(k[1,1])) for x,y in xyall]
                order=sorted(range(len(dist)),key=lambda j:dist[j]);targetxy=p[f'{target}_xy'][i]
                assert tuple(xyall[order[0]])==tuple(targetxy)
                assert dist[order[0]]<=sr['protocol']['angular_radius_normalized'] and (len(order)<2 or dist[order[1]]-dist[order[0]]>1e-12)
                cell=np.rint(uv).astype(int);same=inview&(np.rint(alluv[:,0])==cell[0])&(np.rint(alluv[:,1])==cell[1]);zs=allp[same,2]
                assert len(zs) and math.isclose(q[2],float(zs.min()),rel_tol=1e-12,abs_tol=1e-10)
                sampled_nearest+=1
        reconstructed={};variances={};validity={};geometry={};angular={}
        for mod in ('rgb','thr'):
            k=cal[f'K_{mod}L'];b=-float(cal[f'T_{mod}R'][0,0])/1000;f=float(k[0,0])
            with np.load(rawbase/'frontend-run-001'/f'{mod}-stereo-{stem}-{stem}.npz') as fz:flow=fz['flow'][0];cov=fz['covariance'][0]
            points=[];vlist=[];valid=[];glist=[];alist=[]
            for x,y in p[f'{mod}_xy']:
                du,dv=[float(v) for v in flow[:,y,x]];vu,vv=[float(v) for v in cov[:,y,x]]
                d=-du;good=all(math.isfinite(v) for v in [du,dv,vu,vv]) and vu>0 and vv>0 and d>0
                if good:
                    ray=[(int(x)-float(k[0,2]))/f,(int(y)-float(k[1,2]))/float(k[1,1]),1.];z=f*b/d
                    point=[a*z for a in ray];variance=sum((a*f*b/d**2)**2*vu for a in ray)
                    good=all(math.isfinite(a) for a in point+[variance])
                if not good:point=[math.nan]*3;variance=math.nan;ray=[math.nan]*3;z=math.nan
                points.append(point);vlist.append(variance);valid.append(good);glist.append(sum(a*a for a in ray)*z**4/b**2);alist.append(vu/f**2 if good else math.nan)
            nativep=np.array(points);points=np.column_stack((nativep,np.ones(n)))@tr.T if mod=='thr' else np.column_stack((nativep,np.ones(n)))
            reconstructed[mod]=points[:,:3];variances[mod]=np.array(vlist);validity[mod]=np.array(valid);geometry[mod]=np.array(glist);angular[mod]=np.array(alist)
            saved=m['rgb_predicted_xyz' if mod=='rgb' else 'thr_predicted_xyz_in_rgb']
            assert np.array_equal(validity[mod],m[f'{mod}_valid_mask'])
            assert np.allclose(reconstructed[mod][valid],saved[valid],rtol=1e-10,atol=1e-9)
            assert np.allclose(variances[mod][valid],m[f'{mod}_metric_variance'][valid],rtol=1e-10,atol=1e-9)
            raw_predictions+=n
        both=validity['rgb']&validity['thr'];assert np.array_equal(both,m['both_valid_mask'])
        def choose(a,b):return np.array([1. if x<y else 0. if x>y else .5 for x,y in zip(a,b)])
        choices={'learned_propagated':choose(variances['rgb'][both],variances['thr'][both]),'geometry_angular':choose(geometry['rgb'][both],geometry['thr'][both]),'geometry_pixel':choose(geometry['rgb'][both]/cal['K_rgbL'][0,0]**2,geometry['thr'][both]/cal['K_thrL'][0,0]**2),'learned_unpropagated_angular':choose(angular['rgb'][both],angular['thr'][both])}
        for name,v in choices.items():assert np.array_equal(v,ctl[name])
        for anchor in ('midpoint','rgb','thr'):
            refs={'rgb':p['rgb_reference_xyz'],'thr':p['thr_reference_xyz_in_rgb']};refs['midpoint']=(refs['rgb']+refs['thr'])/2
            er=np.linalg.norm(reconstructed['rgb'][both]-refs[anchor][both],axis=1);et=np.linalg.norm(reconstructed['thr'][both]-refs[anchor][both],axis=1)
            assert np.allclose(er,m[f'{anchor}_rgb_error_m'],rtol=1e-10,atol=1e-9) and np.allclose(et,m[f'{anchor}_thr_error_m'],rtol=1e-10,atol=1e-9)
            for loss,power in [('mse',2),('mean_l2',1)]:
                a=[float(x)**power for x in er];b=[float(x)**power for x in et];cnt=len(a)
                for name,sel in choices.items():
                    risk=math.fsum(float(s)*x+(1-float(s))*y for x,y,s in zip(a,b,sel))/cnt
                    oracle=math.fsum(min(x,y) for x,y in zip(a,b))/cnt;random=math.fsum((x+y)/2 for x,y in zip(a,b))/cnt
                    recorded=crow['scores'][anchor][loss][name]
                    assert math.isclose(risk,recorded['uncertainty'],rel_tol=1e-10,abs_tol=1e-8)
                    assert risk>=oracle-1e-8 and math.isclose(random,recorded['random'],rel_tol=1e-10,abs_tol=1e-8)
                    if name=='learned_propagated':assert math.isclose(risk,mrow['anchors'][anchor][loss]['uncertainty'],rel_tol=1e-10,abs_tol=1e-8)
                    aggregate_checks+=1
        for sensitivity in mrow['depth_sensitivities']:
            mask=p[f"depth_consistent_{sensitivity['relative_bound']:g}"][both]
            for loss,power in [('mse',2),('mean_l2',1)]:
                er=m['midpoint_rgb_error_m'][mask]**power;et=m['midpoint_thr_error_m'][mask]**power
                sel=choices['learned_propagated'][mask]
                if len(er):
                    risk=math.fsum(float(s)*float(x)+(1-float(s))*float(y) for x,y,s in zip(er,et,sel))/len(er)
                    assert math.isclose(risk,sensitivity[loss]['uncertainty'],rel_tol=1e-10,abs_tol=1e-8)
                else:assert sensitivity[loss]['available_pairs']==0
                subset_risk_checks+=1
    historical=0
    for report,key in [(sr,'input_sha256'),(mr,'frozen_input_sha256'),(cr,'frozen_input_sha256')]:
        for path,expected in report[key].items():assert digest(Path(path) if Path(path).is_absolute() else root/path)==expected;historical+=1
    result={'status':'PASS','frame_pairs':20,'candidate_pairs':candidate_points,'raw_model_predictions_recomputed':raw_predictions,'nearest_and_zbuffer_sample_checks':sampled_nearest,'aggregate_selector_risks_recomputed':aggregate_checks,'depth_consistency_masks_checked':depth_checks,'depth_subset_risks_recomputed':subset_risk_checks,'frozen_hash_checks_including_repeats':historical,'physical_reference_validity_certified':False,'validator_sha256':digest(__file__)}
    output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for n in ['root','base','output']:p.add_argument('--'+n,type=Path,required=True)
    a=p.parse_args();validate(a.root.resolve(),a.base,a.output)
