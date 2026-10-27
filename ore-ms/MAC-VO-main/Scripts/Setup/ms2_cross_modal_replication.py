"""Dataset-parameterized frozen EXP056 support and metric controls; no inference."""
import argparse
from datetime import datetime
import json
from pathlib import Path
from zoneinfo import ZoneInfo

import numpy as np
from PIL import Image
from ms2_cross_modal_support import cloud, direction, fixtures, overlay, quartiles, sha
from ms2_cross_modal_metric import reconstruct, risk_record


def dump(path,obj):path.write_text(json.dumps(obj,indent=2,allow_nan=False)+'\n')


def inputs(root,dataset,recovery,models=False):
    base=root/dataset;manifest=json.loads((base/'source-manifest.json').read_text());seq=manifest['sequence']
    assert [r['stem'] for r in manifest['records']]==[f'{i:06d}' for i in range(20)]
    c={k:np.array(v['values'],float) for k,v in json.loads((base/'calibration.json').read_text()).items()}
    expected=json.loads(recovery.read_text())['member_sha256'];paths=[base/'calibration.json',base/'source-manifest.json']
    for mod in ('rgb','thr'):
        paths.append(base/'pilot/sync_data'/seq/mod/'img_left_timestamp.txt')
        for i in range(20):
            stem=f'{i:06d}'
            paths.extend([base/'pilot/proj_depth'/seq/mod/'depth'/f'{stem}.png',base/'pilot/sync_data'/seq/mod/'img_left'/f'{stem}.png'])
            if models:paths.append(base/'frontend-run-001'/f'{mod}-stereo-{stem}-{stem}.npz')
    frozen={str(p):sha(p) for p in paths}
    assert all(expected['ore-ms/'+str(Path(p).relative_to(root))]==h for p,h in frozen.items())
    r=c['R_nir2thr']@c['R_nir2rgb'].T;t=(c['T_nir2thr']-r@c['T_nir2rgb']).reshape(3)/1000
    return base,manifest,seq,c,r,t,frozen


def prepare(root,dataset,recovery,output):
    if output.exists():raise FileExistsError(output)
    base,manifest,seq,c,r,t,frozen=inputs(root,dataset,recovery)
    epsilon=1/c['K_rgbL'][0,0]
    output.mkdir(parents=True)
    protocol={'sequence':seq,'ids':[r['stem'] for r in manifest['records']],'angular_radius_normalized':float(epsilon),'depth_floor_m':2/256,'depth_relative_sensitivities':[0,.01,.02,.05],'primary_uses_depth_threshold':False,'nearest_tie_tolerance':1e-12,'overlays':['000000','000009','000019'],'source_commit':'6bed1f34d','script_sha256':sha(__file__),'helper_sha256':{name:sha(Path(__file__).parent/name) for name in ['ms2_cross_modal_support.py','ms2_cross_modal_metric.py']}}
    dump(output/'protocol.json',protocol);fixture_checks=fixtures();rows=[]
    for record in manifest['records']:
        stem=record['stem'];depth={m:np.asarray(Image.open(base/'pilot/proj_depth'/seq/m/'depth'/f'{stem}.png'),float)/256 for m in ('rgb','thr')}
        xy,a=cloud(depth['rgb'],c['K_rgbL']);uv,b=cloud(depth['thr'],c['K_thrL'])
        fw=direction(a,uv,c['K_thrL'],r,t,depth['thr'].shape);bw=direction(b,xy,c['K_rgbL'],r.T,-r.T@t,depth['rgb'].shape)
        mask=np.ones(len(a),bool);stages={'rgb_nonzero':int(mask.sum())}
        steps=[('forward_in_FOV',fw['inside']),('forward_unique_nearest',fw['unique']),('forward_angular_bound',fw['distance']<=epsilon),('forward_frontmost',fw['front'])]
        for name,cond in steps:mask&=cond;stages[name]=int(mask.sum())
        safe=np.maximum(fw['match'],0)
        steps=[('reverse_in_FOV',bw['inside'][safe]),('reverse_unique_nearest',bw['unique'][safe]),('reverse_angular_bound',bw['distance'][safe]<=epsilon),('reverse_frontmost',bw['front'][safe]),('mutual_nearest',bw['match'][safe]==np.arange(len(a)))] if len(b) else [(name,np.zeros(len(a),bool)) for name in ['reverse_in_FOV','reverse_unique_nearest','reverse_angular_bound','reverse_frontmost','mutual_nearest']]
        for name,cond in steps:mask&=cond;stages[name]=int(mask.sum())
        ids=np.flatnonzero(mask);j=fw['match'][ids];assert len(np.unique(j))==len(j)
        bt=b[j]@r-t@r;dz_thr=fw['points'][ids,2]-b[j,2];dz_rgb=bw['points'][j,2]-a[ids,2]
        mismatch=np.linalg.norm(a[ids]-bt,axis=1)
        pair={'rgb_xy':xy[ids],'thr_xy':uv[j],'rgb_reference_xyz':a[ids],'thr_reference_xyz_in_rgb':bt,'forward_angular_distance':fw['distance'][ids],'reverse_angular_distance':bw['distance'][j],'source_supported_rgb_xy':xy,'source_supported_thr_xy':uv,'rgb_support_mask':mask,'depth_difference_thr_m':dz_thr,'depth_difference_rgb_m':dz_rgb,'reference_point_distance_m':mismatch}
        sensitivity=[]
        for rel in protocol['depth_relative_sensitivities']:
            agree=(abs(dz_thr)<=np.maximum(2/256,rel*b[j,2]))&(abs(dz_rgb)<=np.maximum(2/256,rel*a[ids,2]))
            pair[f'depth_consistent_{rel:g}']=agree;sensitivity.append({'relative_bound':rel,'count':int(agree.sum()),'fraction_of_candidates':float(agree.mean()) if len(agree) else None})
        np.savez_compressed(output/f'pairs-{stem}.npz',**pair)
        delta=record['cameras']['thr']['left']['recorded_time_ns']-record['cameras']['rgb']['left']['recorded_time_ns']
        rows.append({'stem':stem,'stages':stages,'thr_nonzero':len(b),'point_distance_m_quartiles':quartiles(mismatch),'depth_sensitivities':sensitivity,'thr_minus_rgb_left_time_ns':delta})
        if stem in protocol['overlays']:overlay(root,base/'pilot',seq,stem,pair,output)
    assert all(sha(p)==h for p,h in frozen.items())
    dump(output/'report.json',{'status':'PASS','created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'physical_correspondences_verified':False,'protocol':protocol,'fixture_checks':fixture_checks,'frozen_input_sha256':frozen,'rows':rows})
    print('Support',seq,'counts min/median/max:',quartiles([row['stages']['mutual_nearest'] for row in rows]))


def score(root,dataset,recovery,support,output):
    if output.exists():raise FileExistsError(output)
    assert (support/'inspection-decision.md').exists()
    base,manifest,seq,c,r,t,frozen=inputs(root,dataset,recovery,models=True)
    sr=json.loads((support/'report.json').read_text());assert sr['protocol']['sequence']==seq
    for p,h in sr['frozen_input_sha256'].items():assert sha(p)==h
    for p in [support/'report.json',support/'inspection-decision.md']+[support/f"pairs-{x['stem']}.npz" for x in sr['rows']]:frozen[str(p)]=sha(p)
    output.mkdir(parents=True)
    protocol={'sequence':seq,'targets':['midpoint','rgb','thr'],'population':'unchanged both-valid candidates; all availability failures retained','selector':'rank-one metric depth variance, exact ties equal mixture','controls':['always RGB/thermal','equal random/oracle','equal angular geometry','equal pixel geometry','unpropagated learned angular variance','per-image camera median angular variance'],'script_sha256':sha(__file__),'same_exp056_math':True}
    dump(output/'protocol.json',protocol);rows=[]
    for srow in sr['rows']:
        stem=srow['stem']
        with np.load(support/f'pairs-{stem}.npz') as f:pair={k:f[k] for k in f.files}
        predictions={}
        for mod in ('rgb','thr'):
            with np.load(base/'frontend-run-001'/f'{mod}-stereo-{stem}-{stem}.npz') as f:
                assert f['source_ids'].tolist()==[stem,stem]
                camera_times=[manifest['records'][int(stem)]['cameras'][mod][side]['recorded_time_ns'] for side in ('left','right')]
                assert f['input_time_ns'].tolist()==camera_times
                xyz,var,valid,_,_=reconstruct(pair[f'{mod}_xy'],f['flow'][0],f['covariance'][0],c[f'K_{mod}L'],-c[f'T_{mod}R'][0,0]/1000)
            if mod=='thr':xyz=xyz@r-t@r
            predictions[mod]=(xyz,var,valid)
        ar,ur,vr=predictions['rgb'];at,ut,vt=predictions['thr'];both=vr&vt;n=int(both.sum())
        out={'both_valid_mask':both,'rgb_valid_mask':vr,'thr_valid_mask':vt,'rgb_predicted_xyz':ar,'thr_predicted_xyz_in_rgb':at,'rgb_metric_variance':ur,'thr_metric_variance':ut}
        def choose(a,b):return np.where(a<b,1.,np.where(a>b,0.,.5))
        choices={'learned_propagated':choose(ur[both],ut[both])};geom={};scales={}
        for mod,x in [('rgb',ar[both]),('thr',at[both]@r.T+t)]:
            b=-c[f'T_{mod}R'][0,0]/1000;geom[mod]=np.sum(x*x,axis=1)*x[:,2]**2/b**2
            scales[mod]=float(np.median(predictions[mod][1][both]/geom[mod])) if n else None
            out[f'{mod}_geometry_angular']=geom[mod]
        raw={mod:predictions[mod][1][both]/geom[mod] for mod in ('rgb','thr')}
        choices.update({'geometry_angular':choose(geom['rgb'],geom['thr']),'geometry_pixel':choose(geom['rgb']/c['K_rgbL'][0,0]**2,geom['thr']/c['K_thrL'][0,0]**2),'learned_unpropagated_angular':choose(raw['rgb'],raw['thr']),'camera_scale':choose(geom['rgb']*scales['rgb'],geom['thr']*scales['thr']) if n else np.empty(0)})
        out.update(choices)
        a,b=pair['rgb_reference_xyz'],pair['thr_reference_xyz_in_rgb'];anchors={'midpoint':(a+b)/2,'rgb':a,'thr':b}
        row={'stem':stem,'candidates':len(both),'both_valid':n,'rgb_only_valid':int((vr&~vt).sum()),'thr_only_valid':int((vt&~vr).sum()),'neither_valid':int((~vr&~vt).sum()),'median_angular_variances':scales,'rgb_choice_fractions':{name:float(v.mean()) if n else None for name,v in choices.items()},'scores':{},'depth_sensitivities':[]}
        winners=[]
        for anchor,target in anchors.items():
            er=np.linalg.norm(ar[both]-target[both],axis=1);et=np.linalg.norm(at[both]-target[both],axis=1);out[f'{anchor}_rgb_error_m']=er;out[f'{anchor}_thr_error_m']=et;row['scores'][anchor]={};winners.append(np.sign(er-et))
            for loss,power in [('mse',2),('mean_l2',1)]:row['scores'][anchor][loss]={name:risk_record(er**power,et**power,sel) for name,sel in choices.items()}
        row['reference_anchor_winner_change_count']=int(np.count_nonzero(winners[1]!=winners[2]))
        for rel in [0,.01,.02,.05]:
            mask=pair[f'depth_consistent_{rel:g}'][both];out[f'depth_consistent_{rel:g}']=mask
            result={'relative_bound':rel,'both_valid_pairs':int(mask.sum()),'scores':{}}
            for loss,power in [('mse',2),('mean_l2',1)]:result['scores'][loss]={name:risk_record(out['midpoint_rgb_error_m'][mask]**power,out['midpoint_thr_error_m'][mask]**power,sel[mask]) for name,sel in choices.items()}
            row['depth_sensitivities'].append(result)
        np.savez_compressed(output/f'metric-{stem}.npz',**out);rows.append(row)
    assert all(sha(p)==h for p,h in frozen.items())
    dump(output/'report.json',{'status':'PASS','created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'protocol':protocol,'frozen_input_sha256':frozen,'rows':rows})
    print('Metric',seq,'candidates/both/RGBonly/THRonly/neither',[sum(r[k] for r in rows) for k in ['candidates','both_valid','rgb_only_valid','thr_only_valid','neither_valid']])
    for anchor in ['midpoint','rgb','thr']:
        for loss in ['mse','mean_l2']:
            for name in ['geometry_angular','camera_scale']:
                ratios=[r['scores'][anchor][loss]['learned_propagated']['uncertainty']/r['scores'][anchor][loss][name]['uncertainty'] for r in rows if r['both_valid']]
                print(anchor,loss,'learned/'+name,quartiles(ratios),'below1',sum(v<1 for v in ratios),'available frames',len(ratios))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('stage',choices=['prepare','score'])
    for name in ['root','dataset','recovery','output']:p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--support',type=Path)
    a=p.parse_args()
    if a.stage=='prepare':prepare(a.root.resolve(),a.dataset,a.recovery,a.output)
    else:score(a.root.resolve(),a.dataset,a.recovery,a.support,a.output)
