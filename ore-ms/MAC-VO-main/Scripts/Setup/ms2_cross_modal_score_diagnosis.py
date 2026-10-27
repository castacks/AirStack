"""Frozen scale/depth-Jacobian interventions on existing candidate-support results.

Reference-depth scores are supplied-label diagnostics, not deployment policies.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
from zoneinfo import ZoneInfo
import numpy as np
from ms2_cross_modal_support import sha, quartiles
from ms2_cross_modal_metric import risk_record


def sources(root):
    return {
        'rain':(root/'results/ms2-day-cross-modal-replication/rain-control',root/'results/ms2-rain-replication'),
        'day':(root/'results/ms2-day-cross-modal-replication/day',root/'results/ms2-day-replication'),
        'night':(root/'results/ms2-night-cross-modal-replication/night',root/'results/ms2-night-cross-modal-replication/input'),
    }


def choice(a,b):return np.where(a<b,1.,np.where(a>b,0.,.5))


def run(root,output):
    if output.exists():raise FileExistsError(output)
    frozen={};records=sources(root)
    for base,data in records.values():
        for p in [base/'metric/report.json',data/'calibration.json']+[base/'support'/f'pairs-{i:06d}.npz' for i in range(20)]+[base/'metric'/f'metric-{i:06d}.npz' for i in range(20)]:frozen[str(p)]=sha(p)
    protocol={'start_commit':'0894c80ff','windows':'existing20 originalIDs in rain/day/night, no additional sampling','primary_population':'unchanged both-valid candidate pairs','reference_depth':'own single-scan reference Z at the original camera candidate pixel','policies':['original','median_normalized','reference_depth','reference_depth_median_normalized'],'reference_depth_policies_deployable':False,'normalization':'per-image per-camera median angular variance; no errors used; support remains reference-selected','losses':['mse','mean_l2'],'anchors':['midpoint','rgb','thr'],'depth_ratio_bins':['<0.5','[0.5,2]','>2'],'no_inference_fit_or_prediction_changes':True,'script_sha256':sha(__file__)}
    output.mkdir(parents=True);(output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n');rows=[]
    # Analytic invariances, separate from scientific outputs.
    a=np.array([2.,5.,7.]);b=np.array([3.,1.,9.]);assert np.array_equal(choice(a,b),choice(a*13,b*13))
    assert np.allclose((a*7)/np.median(a*7),a/np.median(a));assert np.isclose(6.**4/3.**4,16)
    for window,(base,data) in records.items():
        cal={k:np.array(v['values'],float) for k,v in json.loads((data/'calibration.json').read_text()).items()}
        r=cal['R_nir2thr']@cal['R_nir2rgb'].T;t=(cal['T_nir2thr']-r@cal['T_nir2rgb']).reshape(3)/1000
        old=json.loads((base/'metric/report.json').read_text())['rows'];folder=output/window;folder.mkdir()
        for row in old:
            stem=row['stem']
            with np.load(base/'support'/f'pairs-{stem}.npz') as f:p={k:f[k] for k in f.files}
            with np.load(base/'metric'/f'metric-{stem}.npz') as f:m={k:f[k] for k in f.files}
            mask=m['both_valid_mask'];n=int(mask.sum());assert n>0
            u={};v={};pred={};ref={};rayfactor={};med={};reference_u={};saved={};details={}
            for mod in ['rgb','thr']:
                k=cal[f'K_{mod}L'];b=-cal[f'T_{mod}R'][0,0]/1000;xy=p[f'{mod}_xy'][mask]
                ray=np.column_stack(((xy[:,0]-k[0,2])/k[0,0],(xy[:,1]-k[1,2])/k[1,1],np.ones(n)))
                x=m['rgb_predicted_xyz'][mask] if mod=='rgb' else m['thr_predicted_xyz_in_rgb'][mask]@r.T+t
                reference=p['rgb_reference_xyz'][mask] if mod=='rgb' else p['thr_reference_xyz_in_rgb'][mask]@r.T+t
                pred[mod]=x[:,2];ref[mod]=reference[:,2];assert np.all(pred[mod]>0) and np.all(ref[mod]>0)
                rayfactor[mod]=np.sum(ray*ray,axis=1)/b**2
                u[mod]=m[f'{mod}_metric_variance'][mask];v[mod]=u[mod]/m[f'{mod}_geometry_angular'];med[mod]=float(np.median(v[mod]))
                assert np.allclose(u[mod],rayfactor[mod]*pred[mod]**4*v[mod],rtol=1e-10,atol=1e-8)
                reference_u[mod]=rayfactor[mod]*ref[mod]**4*v[mod]
                ratio=pred[mod]/ref[mod]
                assert np.allclose(u[mod]/reference_u[mod],ratio**4,rtol=1e-10,atol=1e-8)
                details[mod]={'median_angular_variance':med[mod],'predicted_over_reference_depth_quartiles':quartiles(ratio),'predicted_over_reference_score_quartiles':quartiles(ratio**4),'depth_ratio_strata_counts':[int((ratio<.5).sum()),int(((ratio>=.5)&(ratio<=2)).sum()),int((ratio>2).sum())]}
                saved.update({f'{mod}_angular_variance':v[mod],f'{mod}_predicted_z':pred[mod],f'{mod}_reference_z':ref[mod],f'{mod}_ray_baseline_factor':rayfactor[mod]})
            policies={
                'original':choice(u['rgb'],u['thr']),
                'median_normalized':choice(u['rgb']/med['rgb'],u['thr']/med['thr']),
                'reference_depth':choice(reference_u['rgb'],reference_u['thr']),
                'reference_depth_median_normalized':choice(reference_u['rgb']/med['rgb'],reference_u['thr']/med['thr']),
            }
            assert np.array_equal(policies['original'],m['learned_propagated'])
            camera=np.full(n,np.log(med['rgb']/med['thr']));local=np.log(v['rgb']/v['thr'])-camera;geometry=np.log(rayfactor['rgb']/rayfactor['thr']);depth=4*np.log(pred['rgb']/pred['thr'])
            assert np.allclose(camera+local+geometry+depth,np.log(u['rgb']/u['thr']),rtol=1e-10,atol=1e-10)
            log_terms={k:quartiles(vv) for k,vv in [('camera_scale',camera),('local_variance',local),('ray_baseline',geometry),('predicted_depth',depth),('total',camera+local+geometry+depth)]}
            result={'window':window,'stem':stem,'pairs':n,'camera_median_angular_variance_ratio':med['rgb']/med['thr'],'details':details,'log_rgb_over_thr_score_terms_quartiles':log_terms,'rgb_choice_fractions':{k:float(val.mean()) for k,val in policies.items()},'scores':{},'depth_sensitivities':[]}
            er=m['midpoint_rgb_error_m'];et=m['midpoint_thr_error_m'];s=policies['original'];wrong=(er>et)*s+(et>er)*(1-s)
            under_rgb=(pred['rgb']/ref['rgb'])<.5;under_thr=(pred['thr']/ref['thr'])<.5
            result['original_wrong_choice_fraction']=float(wrong.mean())
            result['original_selected_under_half_depth_fraction']=float(np.mean(s*under_rgb+(1-s)*under_thr))
            result['original_wrong_and_selected_under_half_depth_fraction']=float(np.mean(s*(er>et)*under_rgb+(1-s)*(et>er)*under_thr))
            saved.update(policies);saved.update({'camera_scale_log_term':camera,'local_variance_log_term':local,'ray_baseline_log_term':geometry,'predicted_depth_log_term':depth})
            for anchor in protocol['anchors']:
                result['scores'][anchor]={}
                for loss,power in [('mse',2),('mean_l2',1)]:
                    a=m[f'{anchor}_rgb_error_m']**power;b=m[f'{anchor}_thr_error_m']**power
                    result['scores'][anchor][loss]={name:risk_record(a,b,sel) for name,sel in policies.items()}
                    for key,value in row['scores'][anchor][loss]['learned_propagated'].items():
                        new=result['scores'][anchor][loss]['original'][key]
                        assert new==value if value is None else np.isclose(new,value,rtol=1e-12,atol=1e-10)
            for rel in [0,.01,.02,.05]:
                keep=p[f'depth_consistent_{rel:g}'][mask];sub={'relative_bound':rel,'pairs':int(keep.sum()),'scores':{}}
                for loss,power in [('mse',2),('mean_l2',1)]:sub['scores'][loss]={name:risk_record(m['midpoint_rgb_error_m'][keep]**power,m['midpoint_thr_error_m'][keep]**power,sel[keep]) for name,sel in policies.items()}
                result['depth_sensitivities'].append(sub)
            np.savez_compressed(folder/f'diagnosis-{stem}.npz',**saved);rows.append(result)
    assert all(sha(path)==value for path,value in frozen.items())
    report={'status':'PASS','created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'protocol':protocol,'synthetic_invariance_checks':3,'frozen_input_sha256':frozen,'rows':rows}
    (output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    for window in records:
        group=[row for row in rows if row['window']==window]
        print(window,'median RGB/THR angular variance scale',np.median([r['camera_median_angular_variance_ratio'] for r in group]))
        for name in protocol['policies']:
            scores=[r['scores']['midpoint']['mean_l2'][name] for r in group]
            print(name,'RGB ratio median',np.median([a['uncertainty_over_rgb'] for a in scores]),'random ratio median',np.median([a['uncertainty_over_random'] for a in scores]),'better RGB',sum(a['uncertainty_over_rgb']<1 for a in scores),'better random',sum(a['uncertainty_over_random']<1 for a in scores))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.root,a.output)
