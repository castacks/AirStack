"""Replicate frozen MAC ranking rules using historical RGB and MSA-thermal outputs."""
import argparse,ast,hashlib,importlib.util,json
from datetime import datetime,timezone
from pathlib import Path
import cv2
import numpy as np


def sha(p):return hashlib.file_digest(Path(p).open('rb'),'sha256').hexdigest()
def save(p,d):Path(p).write_text(json.dumps(d,indent=2,allow_nan=False)+'\n')
def load_rules(root):
    path=root/'MAC-VO-main/Scripts/Setup/td3_relative_ranking.py';spec=importlib.util.spec_from_file_location('frozen_rules',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module

def helpers(root):
    env=dict(cv2=cv2,np=np)
    for filename,names in [('td3_low_motion.py',{'lk_track','track_masks'}),('td3_temporal_angular.py',{'rays','ray_jacobian','angular_variance','angle'})]:
        p=root/'MAC-VO-main/Scripts/Setup'/filename;tree=ast.parse(p.read_text());exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in names],type_ignores=[]),str(p),'exec'),env)
    return env

def run(root,run):
    root,run=Path(root),Path(run);plan=json.loads((run/'plan.json').read_text());extraction=json.loads((run/'image-extraction.json').read_text());rules=load_rules(root);env=helpers(root);cv2.setNumThreads(1);cv2.setRNGSeed(0)
    files={str(Path(__file__)):sha(__file__),str(run/'plan.json'):sha(run/'plan.json'),str(run/'image-extraction.json'):sha(run/'image-extraction.json')}
    for rel,rec in extraction['files'].items():assert sha(run/'source'/rel)==rec['sha256'];files[str(run/'source'/rel)]=rec['sha256']
    for p,digest in {**plan['prior_sha256'],**plan['source_sha256']}.items():assert sha(p)==digest
    for c in plan['cases']:
        for p in [c['rgb_report'],*c['rgb_predictions'],*c['thermal_arrays']]:files[p]=sha(p)
    save(run/'calculation-plan.json',dict(created_at_utc=datetime.now(timezone.utc).isoformat(),base_commit='6be7ef589',files_sha256=files,protocol='EXP038 frozen choice,summarize and POPULATIONS;EXP030 unchanged angular/LK helpers'))
    records=[];derived=run/'derived-rgb';derived.mkdir(exist_ok=False)
    for case in plan['cases']:
        window=case['window'];dest=derived/window;dest.mkdir();rows=case['rows'];images=[cv2.imread(str(run/'source'/r['rgb_left']['path']),cv2.IMREAD_GRAYSCALE) for r in rows];assert len(images)==20
        for i,raw_path in enumerate(case['rgb_predictions']):
            with np.load(raw_path) as raw:
                uv=raw['grid_uv0'];model=raw['grid_uv1'];covariance=raw['grid_raw_flow_cov'];k=raw['K'].reshape(3,3);height,width=map(int,raw['height_width']);assert int(raw['time0_ns'])==rows[i]['rgb_left']['time_ns']
            assert all(im is not None and im.dtype==np.uint8 and im.shape==(height,width) for im in images)
            end,back,status,reverse,*_=env['lk_track'](images[i],images[i+1],uv);valid,tight,fb,_=env['track_masks'](uv,end,back,status,reverse,width,height)
            first=images[i].astype(float);mean=cv2.boxFilter(first,-1,(21,21));std=np.sqrt(np.maximum(cv2.boxFilter(first*first,-1,(21,21))-mean*mean,0));texture=std[uv[:,1].astype(int),uv[:,0].astype(int)]
            variance,psd=env['angular_variance'](model,covariance,k)
            arrays=dict(uv0=uv,model_endpoint=model,lk_endpoint=end,lk_backward=back,covariance=covariance,K=k,covariance_valid=psd,lk_status=status,lk_reverse_status=reverse,fb_px=fb,
                variance_rad2=variance,variance_px2=np.where(psd,covariance[:,:2].sum(1),np.nan),disagreement_rad=env['angle'](model,end,k),disagreement_px=np.linalg.norm(model-end,axis=1),
                source_std=texture,mask_all=np.ones(len(uv),bool),mask_lk_valid=valid,mask_lk_tight=tight,mask_lk_texture=valid&(texture>=5))
            np.savez_compressed(dest/f'{i:03d}.npz',**arrays)
            offsets=[];eligible=True
            for r in rows[i:i+2]:
                ts={key:r[key]['time_ns'] for key in ['rgb_left','rgb_right','thermal_left','thermal_right']};offset=dict(rgb_stereo_ns=ts['rgb_right']-ts['rgb_left'],thermal_stereo_ns=ts['thermal_right']-ts['thermal_left'],cross_ns=ts['thermal_left']-ts['rgb_left']);offsets.append(offset)
                eligible &= abs(offset['rgb_stereo_ns'])<=2000000 and abs(offset['thermal_stereo_ns'])<=2000000 and abs(offset['cross_ns'])<=30000000
            assert all(rows[i+1][key]['time_ns']>rows[i][key]['time_ns'] for key in ['rgb_left','rgb_right','thermal_left','thermal_right'])
            with np.load(case['thermal_arrays'][i]) as thermal:
                for population,(um,em) in rules.POPULATIONS.items():
                    row=dict(window=window,pair=i,population=population,eligible=bool(eligible),offsets=offsets,times={key:[r[key]['time_ns'] for r in rows[i:i+2]] for key in ['rgb_left','rgb_right','thermal_left','thermal_right']})
                    for modality,data in [('rgb',arrays),('thermal',thermal)]:
                        assert data['covariance_valid'].all()
                        for key,field,mask in [('uncertainty_'+modality+'_rad2','variance_rad2',um),('uncertainty_'+modality+'_px2','variance_px2',um),('error_'+modality+'_rad','disagreement_rad',em)]:
                            values=np.asarray(data[field][data['mask_'+mask]],float);finite=np.isfinite(values);assert np.all(values[finite]>=0);row[key]=float(np.median(values[finite])) if finite.any() else None;row[key+'_finite_count']=int(finite.sum());row[key+'_nonfinite_count']=int((~finite).sum())
                        row[modality+'_grid']=len(data['uv0']);row[modality+'_u_count']=int(data['mask_'+um].sum());row[modality+'_error_count']=int(data['mask_'+em].sum())
                    row['choice_angular']=rules.choice(row['uncertainty_rgb_rad2'],row['uncertainty_thermal_rad2']);row['choice_pixel']=rules.choice(row['uncertainty_rgb_px2'],row['uncertainty_thermal_px2']);row['winner']=rules.choice(row['error_rgb_rad'],row['error_thermal_rad']);records.append(row)
        print('Derived RGB and applied frozen rules:',window,flush=True)
    summaries=[]
    for population in rules.POPULATIONS:
        for window in [*[c['window'] for c in plan['cases']],'pooled']:
            rows=[r for r in records if r['population']==population and (window=='pooled' or r['window']==window)]
            for units in ['angular','pixel']:summaries.append(dict(population=population,window=window,units=units,**rules.summarize(rows,units)))
    for p,digest in files.items():assert sha(p)==digest
    for p,digest in plan['prior_sha256'].items():assert sha(p)==digest
    result=dict(created_at_utc=datetime.now(timezone.utc).isoformat(),recording=plan['recording'],runtime=dict(numpy=np.__version__,opencv=cv2.__version__),records=records,summaries=summaries,verified_files=len(files),prior_unchanged=len(plan['prior_sha256']))
    save(run/'analysis.json',result)
    for r in summaries:
        if r['window']=='pooled':print(r,flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--run',type=Path,required=True);a=p.parse_args();run(a.root,a.run)
