"""Independently reconstruct score interventions from raw covariance/disparity/depth."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import numpy as np
from PIL import Image
from validate_ms2_cross_modal_replication import independent_risk


def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def validate(root,source,output):
    if output.exists():raise FileExistsError(output)
    report=json.loads((source/'report.json').read_text());locations={
        'rain':(root/'results/ms2-day-cross-modal-replication/rain-control',root/'results/ms2-rain-replication'),
        'day':(root/'results/ms2-day-cross-modal-replication/day',root/'results/ms2-day-replication'),
        'night':(root/'results/ms2-night-cross-modal-replication/night',root/'results/ms2-night-cross-modal-replication/input'),
    }
    fields=risks=subrisks=factors=rawhashes=0
    for row in report['rows']:
        window,stem=row['window'],row['stem'];base,data=locations[window]
        old=json.loads((base/'metric/report.json').read_text());cal={k:np.array(v['values'],float) for k,v in json.loads((data/'calibration.json').read_text()).items()};seq=json.loads((data/'source-manifest.json').read_text())['sequence']
        with np.load(base/'support'/f'pairs-{stem}.npz') as f:p={k:f[k] for k in f.files}
        with np.load(base/'metric'/f'metric-{stem}.npz') as f:m={k:f[k] for k in f.files}
        with np.load(source/window/f'diagnosis-{stem}.npz') as f:saved={k:f[k] for k in f.files}
        mask=m['both_valid_mask'];n=int(mask.sum());assert n==row['pairs']
        angular={};gpred={};gref={};med={};predicted={};reference={};u={}
        for mod in ['rgb','thr']:
            k=cal[f'K_{mod}L'];fx=float(k[0,0]);fy=float(k[1,1]);b=-float(cal[f'T_{mod}R'][0,0])/1000
            model=data/'frontend-run-001'/f'{mod}-stereo-{stem}-{stem}.npz';depthfile=data/'pilot/proj_depth'/seq/mod/'depth'/f'{stem}.png'
            for path in [model,depthfile]:assert digest(path)==old['frozen_input_sha256'][str(path)];rawhashes+=1
            with np.load(model) as f:flow=f['flow'][0];cov=f['covariance'][0];assert f['source_ids'].tolist()==[stem,stem]
            depth=np.asarray(Image.open(depthfile),float)/256
            av=[];gp=[];gr=[];zp=[];zr=[];ufull=[];rayfactor=[]
            for x,y in p[f'{mod}_xy'][mask]:
                disparity=-float(flow[0,y,x]);variance=float(cov[0,y,x]);z=fx*b/disparity;zref=float(depth[y,x])
                q2=((int(x)-float(k[0,2]))/fx)**2+((int(y)-float(k[1,2]))/fy)**2+1
                assert disparity>0 and variance>0 and zref>0
                angular_variance=variance/fx**2;factor=q2/b**2;av.append(angular_variance);rayfactor.append(factor);gp.append(factor*z**4);gr.append(factor*zref**4);zp.append(z);zr.append(zref);ufull.append(q2*(fx*b/disparity**2)**2*variance);factors+=1
            angular[mod]=np.array(av);gpred[mod]=np.array(gp);gref[mod]=np.array(gr);predicted[mod]=np.array(zp);reference[mod]=np.array(zr);u[mod]=np.array(ufull);med[mod]=statistics.median(av)
            for key,values in [('angular_variance',av),('predicted_z',zp),('reference_z',zr),('ray_baseline_factor',rayfactor)]:assert np.allclose(values,saved[f'{mod}_{key}'],rtol=1e-10,atol=1e-9),(window,stem,mod,key)
            assert np.allclose(u[mod],m[f'{mod}_metric_variance'][mask],rtol=1e-10,atol=1e-8)
            assert math.isclose(med[mod],row['details'][mod]['median_angular_variance'],rel_tol=1e-10)
            ratio=predicted[mod]/reference[mod]
            assert [int((ratio<.5).sum()),int(((ratio>=.5)&(ratio<=2)).sum()),int((ratio>2).sum())]==row['details'][mod]['depth_ratio_strata_counts']
        choose=lambda a,b:np.array([1. if x<y else 0. if x>y else .5 for x,y in zip(a,b)])
        choices={'original':choose(u['rgb'],u['thr']),'median_normalized':choose(u['rgb']/med['rgb'],u['thr']/med['thr']),'reference_depth':choose(gref['rgb']*angular['rgb'],gref['thr']*angular['thr']),'reference_depth_median_normalized':choose(gref['rgb']*angular['rgb']/med['rgb'],gref['thr']*angular['thr']/med['thr'])}
        for name,value in choices.items():assert np.array_equal(value,saved[name]),(window,stem,name)
        scale=math.log(med['rgb']/med['thr']);local=np.log(angular['rgb']/angular['thr'])-scale;ray=np.log(gpred['rgb']/predicted['rgb']**4)-np.log(gpred['thr']/predicted['thr']**4);z=4*np.log(predicted['rgb']/predicted['thr'])
        for name,val in [('camera_scale',np.full(n,scale)),('local_variance',local),('ray_baseline',ray),('predicted_depth',z)]:assert np.allclose(saved[name+'_log_term'],val,rtol=1e-10,atol=1e-10)
        assert np.allclose(scale+local+ray+z,np.log(u['rgb']/u['thr']),rtol=1e-10,atol=1e-10)
        def compare(a,b):
            nonlocal fields
            assert set(a)==set(b)
            for key,value in a.items():
                assert b[key] is None if value is None else math.isclose(value,b[key],rel_tol=1e-10,abs_tol=1e-8),(window,stem,key,value,b[key]);fields+=1
        for anchor in ['midpoint','rgb','thr']:
            for loss,power in [('mse',2),('mean_l2',1)]:
                er=m[f'{anchor}_rgb_error_m']**power;et=m[f'{anchor}_thr_error_m']**power
                for name,sel in choices.items():compare(independent_risk(er,et,sel),row['scores'][anchor][loss][name]);risks+=1
        for subset in row['depth_sensitivities']:
            keep=p[f"depth_consistent_{subset['relative_bound']:g}"][mask];assert int(keep.sum())==subset['pairs']
            for loss,power in [('mse',2),('mean_l2',1)]:
                for name,sel in choices.items():compare(independent_risk(m['midpoint_rgb_error_m'][keep]**power,m['midpoint_thr_error_m'][keep]**power,sel[keep]),subset['scores'][loss][name]);subrisks+=1
    for path,value in report['frozen_input_sha256'].items():assert digest(path)==value
    result={'status':'PASS','recordings':3,'image_pairs':60,'raw_camera_score_factors_checked':factors,'primary_selector_risks_checked':risks,'depth_subset_risks_checked':subrisks,'scalar_risk_fields_checked':fields,'raw_model_depth_hash_checks':rawhashes,'frozen_analysis_hash_checks':len(report['frozen_input_sha256']),'reference_assisted_scores_are_deployable':False,'validator_sha256':digest(__file__)}
    output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['root','source','output']:p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();validate(a.root,a.source,a.output)
