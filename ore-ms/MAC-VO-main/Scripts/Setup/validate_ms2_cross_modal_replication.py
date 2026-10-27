"""Independent dataset-parameterized raw-data audit of frozen cross-modal replication."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import numpy as np
from PIL import Image


def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def independent_risk(a,b,choices):
    n=len(a)
    if not n:return {'available_pairs':0}
    avg=lambda xs:math.fsum(xs)/n
    ca=avg(a);cb=avg(b);chosen=avg([s*x+(1-s)*y for s,x,y in zip(choices,a,b)]);random=(ca+cb)/2
    accuracy=avg([.5 if x==y else s if x<y else 1-s for s,x,y in zip(choices,a,b)])
    return {'available_pairs':n,'rgb':ca,'thr':cb,'uncertainty':chosen,'random':random,'oracle':avg([min(x,y) for x,y in zip(a,b)]),'uncertainty_over_random':chosen/random if random>0 else None,'uncertainty_over_rgb':chosen/ca if ca>0 else None,'uncertainty_over_thr':chosen/cb if cb>0 else None,'lower_error_choice_fraction':accuracy,'rgb_lower_error_fraction':sum(x<y for x,y in zip(a,b))/n,'error_tie_fraction':sum(x==y for x,y in zip(a,b))/n}


def validate(root,dataset,support,metric,recovery,output):
    if output.exists():raise FileExistsError(output)
    base=root/dataset;sr=json.loads((support/'report.json').read_text());mr=json.loads((metric/'report.json').read_text());manifest=json.loads((base/'source-manifest.json').read_text());seq=manifest['sequence']
    assert sr['protocol']['sequence']==mr['protocol']['sequence']==seq
    c={k:np.array(v['values'],float) for k,v in json.loads((base/'calibration.json').read_text()).items()}
    matrices={}
    for mod in ('rgb','thr'):
        h=np.eye(4);h[:3,:3]=c[f'R_nir2{mod}'];h[:3,3]=c[f'T_nir2{mod}'][:,0]/1000;matrices[mod]=h
    rt=matrices['thr']@np.linalg.inv(matrices['rgb']);tr=np.linalg.inv(rt)
    counts={'candidate_pairs':0,'raw_model_predictions':0,'nearest_and_zbuffer_samples':0,'depth_masks':0,'selector_risks':0,'risk_fields':0,'subset_risks':0,'timestamp_joins':0}
    eps=1/float(c['K_rgbL'][0,0]);assert eps==sr['protocol']['angular_radius_normalized']
    for i,(srow,row) in enumerate(zip(sr['rows'],mr['rows'])):
        stem=f'{i:06d}';assert srow['stem']==row['stem']==stem
        with np.load(support/f'pairs-{stem}.npz') as f:p={k:f[k] for k in f.files}
        with np.load(metric/f'metric-{stem}.npz') as f:m={k:f[k] for k in f.files}
        n=len(p['rgb_xy']);counts['candidate_pairs']+=n;assert n==srow['stages']['mutual_nearest']==row['candidates']
        assert len(set(map(tuple,p['rgb_xy'])))==len(set(map(tuple,p['thr_xy'])))==n
        stages=list(srow['stages'].values());assert all(x>=y for x,y in zip(stages,stages[1:]))
        native={};depth={};prediction={};variance={};valid={};geometry={};angular={}
        for mod in ('rgb','thr'):
            k=c[f'K_{mod}L'];fx=float(k[0,0]);fy=float(k[1,1]);b=-float(c[f'T_{mod}R'][0,0])/1000
            depth[mod]=np.asarray(Image.open(base/'pilot/proj_depth'/seq/mod/'depth'/f'{stem}.png'),float)/256
            native[mod]=np.array([[(int(x)-float(k[0,2]))*float(depth[mod][y,x])/fx,(int(y)-float(k[1,2]))*float(depth[mod][y,x])/fy,float(depth[mod][y,x])] for x,y in p[f'{mod}_xy']]).reshape(-1,3)
            time_lines=(base/'pilot/sync_data'/seq/mod/'img_left_timestamp.txt').read_text().splitlines();assert int(time_lines[i])==manifest['records'][i]['cameras'][mod]['left']['recorded_time_ns'];counts['timestamp_joins']+=1
            with np.load(base/'frontend-run-001'/f'{mod}-stereo-{stem}-{stem}.npz') as f:
                flow=f['flow'][0];cov=f['covariance'][0];assert f['source_ids'].tolist()==[stem,stem]
                assert f['input_time_ns'].tolist()==[manifest['records'][i]['cameras'][mod][side]['recorded_time_ns'] for side in ('left','right')]
            pts=[];vars=[];available=[];gs=[];ang=[]
            for x,y in p[f'{mod}_xy']:
                du,dv=[float(v) for v in flow[:,y,x]];vu,vv=[float(v) for v in cov[:,y,x]];d=-du
                good=all(math.isfinite(v) for v in [du,dv,vu,vv]) and vu>0 and vv>0 and d>0
                if good:
                    q=[(int(x)-float(k[0,2]))/fx,(int(y)-float(k[1,2]))/fy,1.];z=fx*b/d;point=[v*z for v in q];v3=sum((v*fx*b/d**2)**2*vu for v in q);g=sum(v*v for v in q)*z**4/b**2;va=vu/fx**2
                    good=all(math.isfinite(v) for v in point+[v3])
                if not good:point=[math.nan]*3;v3=g=va=math.nan
                pts.append(point);vars.append(v3);available.append(good);gs.append(g);ang.append(va)
            prediction[mod]=np.array(pts).reshape(-1,3);variance[mod]=np.array(vars);valid[mod]=np.array(available,bool);geometry[mod]=np.array(gs);angular[mod]=np.array(ang)
            expected_points=np.column_stack((prediction[mod],np.ones(n)))@tr.T if mod=='thr' else np.column_stack((prediction[mod],np.ones(n)))
            key='rgb_predicted_xyz' if mod=='rgb' else 'thr_predicted_xyz_in_rgb'
            assert np.array_equal(valid[mod],m[f'{mod}_valid_mask'])
            assert np.allclose(expected_points[valid[mod],:3],m[key][valid[mod]],atol=1e-8,rtol=1e-10)
            assert np.allclose(variance[mod][valid[mod]],m[f'{mod}_metric_variance'][valid[mod]],atol=1e-8,rtol=1e-10)
            counts['raw_model_predictions']+=n
        assert np.allclose(native['rgb'],p['rgb_reference_xyz'],atol=1e-10,rtol=1e-12)
        thr_ref=(np.column_stack((native['thr'],np.ones(n)))@tr.T)[:,:3]
        assert np.allclose(thr_ref,p['thr_reference_xyz_in_rgb'],atol=1e-10,rtol=1e-12)
        assert np.allclose(np.linalg.norm(native['rgb']-thr_ref,axis=1),p['reference_point_distance_m'],atol=1e-10,rtol=1e-12)
        fwd=np.column_stack((native['rgb'],np.ones(n)))@rt.T;rev=np.column_stack((native['thr'],np.ones(n)))@tr.T
        depth_masks={}
        for rel in [0,.01,.02,.05]:
            mask=np.array([abs(float(q[2])-float(b[2]))<=max(2/256,rel*float(b[2])) and abs(float(v[2])-float(a[2]))<=max(2/256,rel*float(a[2])) for q,b,v,a in zip(fwd,native['thr'],rev,native['rgb'])],bool)
            assert np.array_equal(mask,p[f'depth_consistent_{rel:g}']);depth_masks[rel]=mask;counts['depth_masks']+=1
        for source,target,h in [('rgb','thr',rt),('thr','rgb',tr)]:
            ks=c[f'K_{source}L'];kt=c[f'K_{target}L'];allxy=p[f'source_supported_{source}_xy'];targetxy=p[f'source_supported_{target}_xy'];z=depth[source][allxy[:,1],allxy[:,0]]
            allpoints=np.column_stack(((allxy[:,0]-ks[0,2])*z/ks[0,0],(allxy[:,1]-ks[1,2])*z/ks[1,1],z,np.ones(len(z))))@h.T
            with np.errstate(divide='ignore',invalid='ignore'):uv=allpoints[:,:2]/allpoints[:,2,None]*[kt[0,0],kt[1,1]]+[kt[0,2],kt[1,2]]
            ht,wt=depth[target].shape;inview=(allpoints[:,2]>0)&(uv[:,0]>=0)&(uv[:,0]<=wt-1)&(uv[:,1]>=0)&(uv[:,1]<=ht-1)
            for idx in np.unique(np.linspace(0,n-1,min(n,3),dtype=int)) if n else []:
                q=[sum(float(h[a,b])*float(np.append(native[source][idx],1)[b]) for b in range(4)) for a in range(3)]
                point=[float(kt[a,a])*q[a]/q[2]+float(kt[a,2]) for a in range(2)]
                ds=[math.hypot((int(x)-point[0])/float(kt[0,0]),(int(y)-point[1])/float(kt[1,1])) for x,y in targetxy];order=sorted(range(len(ds)),key=lambda j:ds[j])
                assert tuple(targetxy[order[0]])==tuple(p[f'{target}_xy'][idx]) and ds[order[0]]<=eps
                assert len(order)<2 or ds[order[1]]-ds[order[0]]>1e-12
                cell=np.rint(point);same=inview&(np.rint(uv[:,0])==cell[0])&(np.rint(uv[:,1])==cell[1]);zs=allpoints[same,2]
                assert len(zs) and math.isclose(q[2],float(zs.min()),abs_tol=1e-10,rel_tol=1e-12);counts['nearest_and_zbuffer_samples']+=1
        both=valid['rgb']&valid['thr'];assert np.array_equal(both,m['both_valid_mask'])
        for key,mask in [('both_valid',both),('rgb_only_valid',valid['rgb']&~valid['thr']),('thr_only_valid',valid['thr']&~valid['rgb']),('neither_valid',~valid['rgb']&~valid['thr'])]:assert int(mask.sum())==row[key]
        def choose(a,b):return np.array([1. if x<y else 0. if x>y else .5 for x,y in zip(a,b)])
        scales={mod:statistics.median(angular[mod][both]) if both.any() else None for mod in ['rgb','thr']}
        for mod in scales:
            assert row['median_angular_variances'][mod] is None if scales[mod] is None else math.isclose(scales[mod],row['median_angular_variances'][mod],rel_tol=1e-10)
        choices={'learned_propagated':choose(variance['rgb'][both],variance['thr'][both]),'geometry_angular':choose(geometry['rgb'][both],geometry['thr'][both]),'geometry_pixel':choose(geometry['rgb'][both]/c['K_rgbL'][0,0]**2,geometry['thr'][both]/c['K_thrL'][0,0]**2),'learned_unpropagated_angular':choose(angular['rgb'][both],angular['thr'][both]),'camera_scale':choose(geometry['rgb'][both]*scales['rgb'],geometry['thr'][both]*scales['thr']) if both.any() else np.empty(0)}
        for key,value in choices.items():assert np.array_equal(value,m[key]),(stem,key)
        anchors={'rgb':native['rgb'],'thr':thr_ref,'midpoint':(native['rgb']+thr_ref)/2}
        def compare_fields(a,b):
            assert set(a)==set(b)
            for key,value in a.items():
                assert b[key] is None if value is None else math.isclose(value,b[key],rel_tol=1e-10,abs_tol=1e-8),(stem,key,value,b[key]);counts['risk_fields']+=1
        errors={}
        thrpred=(np.column_stack((prediction['thr'],np.ones(n)))@tr.T)[:,:3]
        for anchor,ref in anchors.items():
            er=np.linalg.norm(prediction['rgb'][both]-ref[both],axis=1);et=np.linalg.norm(thrpred[both]-ref[both],axis=1);errors[anchor]=(er,et)
            for key,values in [('rgb',er),('thr',et)]:assert np.allclose(values,m[f'{anchor}_{key}_error_m'],rtol=1e-10,atol=1e-8)
            for loss,power in [('mse',2),('mean_l2',1)]:
                for name,sel in choices.items():compare_fields(independent_risk(er**power,et**power,sel),row['scores'][anchor][loss][name]);counts['selector_risks']+=1
        assert int(np.count_nonzero(np.sign(errors['rgb'][0]-errors['rgb'][1])!=np.sign(errors['thr'][0]-errors['thr'][1])))==row['reference_anchor_winner_change_count']
        for sensitivity in row['depth_sensitivities']:
            rel=sensitivity['relative_bound'];mask=depth_masks[rel][both];assert np.array_equal(mask,m[f'depth_consistent_{rel:g}']) and int(mask.sum())==sensitivity['both_valid_pairs']
            for loss,power in [('mse',2),('mean_l2',1)]:
                for name,sel in choices.items():compare_fields(independent_risk(errors['midpoint'][0][mask]**power,errors['midpoint'][1][mask]**power,sel[mask]),sensitivity['scores'][loss][name]);counts['subset_risks']+=1
    expected=json.loads(recovery.read_text())['member_sha256'];hashchecks=0
    for report in [sr,mr]:
        for path,value in report['frozen_input_sha256'].items():
            assert digest(path)==value
            relative=str(Path(path).relative_to(root))
            if 'ore-ms/'+relative in expected:assert expected['ore-ms/'+relative]==value
            hashchecks+=1
    result={'status':'PASS','sequence':seq,'frames':20,**counts,'frozen_hash_checks_including_repeats':hashchecks,'physical_reference_certification':False,'validator_sha256':digest(__file__)}
    output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['root','dataset','support','metric','recovery','output']:p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();validate(a.root.resolve(),a.dataset,a.support,a.metric,a.recovery,a.output)
