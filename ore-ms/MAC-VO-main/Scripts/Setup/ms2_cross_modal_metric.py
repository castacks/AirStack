"""Conditional metric comparison on frozen candidate pairs, not physical truth."""
import argparse
from datetime import datetime
import json
from pathlib import Path
from zoneinfo import ZoneInfo

import numpy as np

from ms2_cross_modal_support import sha, quartiles


def reconstruct(xy, flow, variance, k, baseline):
    u, v = xy.T
    f = flow[0, v, u].astype(np.float64)
    var = variance[0, v, u].astype(np.float64)
    d = -f
    valid = np.isfinite(flow[:,v,u]).all(axis=0) & np.isfinite(variance[:,v,u]).all(axis=0) & (variance[:,v,u] > 0).all(axis=0) & (d > 0)
    ray = np.column_stack(((u-k[0,2])/k[0,0], (v-k[1,2])/k[1,1], np.ones(len(u))))
    xyz = np.full((len(u),3), np.nan)
    scalar_variance = np.full(len(u), np.nan)
    xyz[valid] = ray[valid]*(k[0,0]*baseline/d[valid,None])
    scalar_variance[valid] = np.sum(ray[valid]**2,axis=1)*(k[0,0]*baseline/d[valid]**2)**2*var[valid]
    valid &= np.isfinite(xyz).all(axis=1) & np.isfinite(scalar_variance)
    return xyz, scalar_variance, valid, d, var


def risk_record(error_rgb, error_thr, choose_rgb):
    a, b = error_rgb, error_thr
    if not len(a):
        return {'available_pairs':0}
    selected = choose_rgb*a+(1-choose_rgb)*b
    oracle, random = np.minimum(a,b), (a+b)/2
    best = (a < b).astype(float)+.5*(a==b)
    accuracy = np.where(best==.5,.5,np.where(best==1,choose_rgb,1-choose_rgb))
    return {'available_pairs':len(a),'rgb':float(a.mean()),'thr':float(b.mean()),'uncertainty':float(selected.mean()),'random':float(random.mean()),'oracle':float(oracle.mean()),'uncertainty_over_random':float(selected.mean()/random.mean()) if random.mean()>0 else None,'uncertainty_over_rgb':float(selected.mean()/a.mean()) if a.mean()>0 else None,'uncertainty_over_thr':float(selected.mean()/b.mean()) if b.mean()>0 else None,'lower_error_choice_fraction':float(accuracy.mean()),'rgb_lower_error_fraction':float((a<b).mean()),'error_tie_fraction':float((a==b).mean())}


def metric(root, support, output, recovery):
    if output.exists():
        raise FileExistsError(output)
    inspection = support/'inspection-decision.md'
    assert inspection.exists()
    support_report = json.loads((support/'report.json').read_text())
    assert support_report['status']=='PASS' and not support_report['physical_correspondences_verified']
    base=root/'results/ms2-rain-replication'
    c={k:np.array(v['values'],float) for k,v in json.loads((base/'calibration.json').read_text()).items()}
    r=c['R_nir2thr']@c['R_nir2rgb'].T
    t=(c['T_nir2thr']-r@c['T_nir2rgb']).reshape(3)/1000
    expected=json.loads(recovery.read_text())['member_sha256']
    frozen={str(support/'report.json'):sha(support/'report.json'),str(inspection):sha(inspection),str(base/'calibration.json'):sha(base/'calibration.json')}
    for row in support_report['rows']:
        stem=row['stem'];p=support/f'pairs-{stem}.npz';frozen[str(p)]=sha(p)
        for mod in ('rgb','thr'):
            p=base/'frontend-run-001'/f'{mod}-stereo-{stem}-{stem}.npz'
            assert sha(p)==expected['ore-ms/'+str(p.relative_to(root))]
            frozen[str(p)]=sha(p)
    for p,value in support_report['input_sha256'].items():
        assert sha(root/p)==value
        frozen[str(root/p)]=value
    protocol={'target':'RGB-frame midpoint of candidate reference points, endpoint anchors separate','primary_support':'all mutual/frontmost angular candidates, no depth threshold','validity':'finite both flow/covariance channels, positive both variances and horizontal disparity; finite reconstructed outputs','primary_population':'both-valid; all candidates and one/neither-valid counts retained','selector':'lower rank-one propagated metric depth variance trace; exact ties equal mixture','primary_loss':'squared Euclidean point error','sensitivity_loss':'Euclidean point error','reference_anchors':['midpoint','rgb','thr'],'depth_relative_sensitivities':[0,.01,.02,.05],'new_inference':False,'physical_truth_claim':False,'script_sha256':sha(__file__)}
    output.mkdir(parents=True)
    (output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    rows=[];independent=0
    # Numerical derivative check at two disparities, without model/reference outcomes.
    for disparity in (2.,20.):
        f,b,v=500.,.3,4.;ray=np.array([.2,-.1,1.]);eps=disparity*1e-5
        derivative=(ray*f*b/(disparity+eps)-ray*f*b/(disparity-eps))/(2*eps)
        expected_j=-ray*f*b/disparity**2
        assert np.allclose(derivative,expected_j,rtol=1e-8,atol=1e-10)
        assert np.allclose(np.trace(np.outer(expected_j,expected_j)*v),np.sum(expected_j**2)*v)
    for support_row in support_report['rows']:
        stem=support_row['stem']
        with np.load(support/f'pairs-{stem}.npz') as data:
            pair={k:data[k] for k in data.files}
        predictions={}
        for mod in ('rgb','thr'):
            with np.load(base/'frontend-run-001'/f'{mod}-stereo-{stem}-{stem}.npz') as data:
                assert data['source_ids'].tolist()==[stem,stem]
                flow,var=data['flow'][0],data['covariance'][0]
            k=c[f'K_{mod}L'];b=-c[f'T_{mod}R'][0,0]/1000
            xyz,trace,valid,disparity,variance=reconstruct(pair[f'{mod}_xy'],flow,var,k,b)
            for i in np.flatnonzero(valid)[np.unique(np.linspace(0,int(valid.sum())-1,min(3,int(valid.sum())),dtype=int))] if valid.any() else []:
                u,vv=map(int,pair[f'{mod}_xy'][i]);d=float(-flow[0,vv,u]);f=float(k[0,0])
                q=[(u-float(k[0,2]))/f,(vv-float(k[1,2]))/float(k[1,1]),1.]
                ref=[a*f*b/d for a in q];vt=sum(a*a for a in q)*(f*b/d**2)**2*float(var[0,vv,u])
                assert np.allclose(ref,xyz[i],atol=1e-10,rtol=1e-12) and np.isclose(vt,trace[i],rtol=1e-12)
                independent+=2
            if mod=='thr':
                xyz=xyz@r-t@r
            predictions[mod]=(xyz,trace,valid)
        ar,ur,vr=predictions['rgb'];at,ut,vt=predictions['thr']
        both=vr&vt
        choose=np.where(ur[both]<ut[both],1.,np.where(ur[both]>ut[both],0.,.5))
        a,b=pair['rgb_reference_xyz'],pair['thr_reference_xyz_in_rgb']
        anchors={'midpoint':(a+b)/2,'rgb':a,'thr':b}
        out={'both_valid_mask':both,'rgb_valid_mask':vr,'thr_valid_mask':vt,'rgb_predicted_xyz':ar,'thr_predicted_xyz_in_rgb':at,'rgb_metric_variance':ur,'thr_metric_variance':ut,'choose_rgb_on_both_valid':choose}
        row={'stem':stem,'candidates':len(both),'both_valid':int(both.sum()),'rgb_only_valid':int((vr&~vt).sum()),'thr_only_valid':int((vt&~vr).sum()),'neither_valid':int((~vr&~vt).sum()),'choose_rgb_fraction':float(choose.mean()) if len(choose) else None,'anchors':{},'depth_sensitivities':[]}
        anchor_winners=[]
        for name,target in anchors.items():
            er=np.linalg.norm(ar[both]-target[both],axis=1);et=np.linalg.norm(at[both]-target[both],axis=1)
            out[f'{name}_rgb_error_m']=er;out[f'{name}_thr_error_m']=et
            row['anchors'][name]={'mse':risk_record(er**2,et**2,choose),'mean_l2':risk_record(er,et,choose)}
            anchor_winners.append(np.sign(er-et))
        row['reference_anchor_winner_change_count']=int(np.count_nonzero(anchor_winners[1]!=anchor_winners[2]))
        for rel in protocol['depth_relative_sensitivities']:
            mask=pair[f'depth_consistent_{rel:g}'][both]
            er=out['midpoint_rgb_error_m'][mask];et=out['midpoint_thr_error_m'][mask]
            row['depth_sensitivities'].append({'relative_bound':rel,'both_valid_pairs':int(mask.sum()),'mse':risk_record(er**2,et**2,choose[mask]),'mean_l2':risk_record(er,et,choose[mask])})
        np.savez_compressed(output/f'metric-{stem}.npz',**out)
        rows.append(row)
    assert all(sha(p)==value for p,value in frozen.items())
    result={'status':'PASS','created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'scope':'conditional candidate support; supplied references, not independent physical truth','protocol':protocol,'independent_scalar_prediction_variance_checks':independent,'jacobian_fixture_cases':2,'frozen_input_sha256':frozen,'rows':rows}
    (output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    print('Metric candidate benchmark PASS;',len(rows),'frames;',independent,'scalar checks;',len(frozen),'frozen files.')
    print('Pairs, bothvalid, onlyRGB, onlyTHR, neither:',[sum(r[k] for r in rows) for k in ['candidates','both_valid','rgb_only_valid','thr_only_valid','neither_valid']])
    for anchor in protocol['reference_anchors']:
        for loss in ('mse','mean_l2'):
            print(anchor,loss,{key:quartiles([r['anchors'][anchor][loss][key] for r in rows if r['anchors'][anchor][loss].get(key) is not None]) for key in ['uncertainty_over_random','uncertainty_over_rgb','uncertainty_over_thr','lower_error_choice_fraction']})
    print('Anchor winner changed:',sum(r['reference_anchor_winner_change_count'] for r in rows))
    print('RGB selection fraction across image medians:',quartiles([r['choose_rgb_fraction'] for r in rows]))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['root','support','output','recovery-manifest']:
        p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();metric(a.root.resolve(),a.support,a.output,a.recovery_manifest)
