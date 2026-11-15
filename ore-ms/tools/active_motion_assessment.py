"""Fixed matched-motion images and native estimator assay; not a motion planner."""
import argparse,hashlib
from pathlib import Path
import cv2
import numpy as np
from scipy.ndimage import map_coordinates
from qualify_native_frontend import read,write,sha,now,recipe,deterministic,load_model
from paired_frontend_assessment import native_algorithms,pick,fit
from sensor_estimator_comparison import safe_mask,build,joint_ids,qualify
from continuity_sequence import error
ARMS=['sharp','memory']
def verify(p):
    for path,h in p['pins'].items():assert sha(path)==h,path

def progress(u,beta):
    u=np.clip(u,0,1)
    return u**3*(10+u*(-15+6*u))+beta*u**3*(1-u)**3

def plane_view(textures,x,side,p):
    h,w=p['shape'];v,u=np.mgrid[:h,:w]
    U=u+p['K'][0][0]*(x+side*p['baseline_m'])/p['plane_depth_m']
    mask=(U>=1)&(U<=w-2)&(v>=1)&(v<=h-2)
    values=np.stack([map_coordinates(t,[v,U],order=1,mode='constant',cval=0.,prefilter=False) for t in textures])
    return values,mask

def render(p,out):
    out.mkdir(parents=True,exist_ok=False)
    names=list(p['textures']);textures=[recipe()(cv2.imread(p['textures'][c],cv2.IMREAD_UNCHANGED)).astype(float)/255 for c in names]
    H,D,tau=p['horizon_s'],p['distance_m'],p['tau_s'];tail=np.exp(-H/tau)
    initial=[plane_view(textures,0,side,p) for side in [0,1]]
    terminal=[plane_view(textures,D,side,p) for side in [0,1]]
    support=np.logical_and.reduce([x[1] for x in initial+terminal])
    levels={}
    for n in [64,128]:
        nodes,weights=np.polynomial.legendre.leggauss(n);ages=(nodes+1)*H/2
        weights=H/2*weights*np.exp(-ages/tau)/tau;weights*=((1-tail)/weights.sum())
        levels[n]={}
        for beta in p['betas']:
            values=np.empty((3,2,2,2,*p['shape']))
            for side in [0,1]:
                values[:,0,0,side]=initial[side][0];values[:,1,0,side]=initial[side][0]
                values[:,0,1,side]=terminal[side][0]
                integrated=tail*initial[side][0]
                for age,weight in zip(ages,weights):
                    sample,_=plane_view(textures,D*progress((H-age)/H,beta),side,p);integrated+=weight*sample
                values[:,1,1,side]=integrated
            levels[n][beta]=values
            print('render',n,beta,flush=True)
    maximum,squares,count=0.,0.,0
    for beta in p['betas']:
        diff=(levels[128][beta]-levels[64][beta])[...,support]*255
        maximum=max(maximum,float(np.abs(diff).max()));squares+=float(np.sum(diff**2));count+=diff.size
    convergence={'max_code':maximum,'rms_code':float(np.sqrt(squares/count)),'supported_values':count,'pass':bool(maximum<=p['max_code'] and np.sqrt(squares/count)<=p['rms_code'])}
    write(out/'convergence.json',convergence)
    assert convergence['pass'],'Fixed64/128integration not qualified; no estimator allowed'
    records=[]
    for beta in p['betas']:
        left=np.repeat(np.eye(4)[None],2,axis=0);left[1,0,3]=D;right=left.copy();right[:,0,3]+=p['baseline_m']
        for i,name in enumerate(names):
            values=levels[128][beta][i];images=np.rint(np.round(np.clip(values,0,1)*255,10)).astype(np.uint8)
            case=name+'-beta'+str(beta);path=out/(case+'.npz')
            np.savez_compressed(path,images=images,continuous=values,coarse=levels[64][beta][i],common_pixel_support=support,texture=textures[i],world_from_left=left,world_from_right=right,target_to_source=np.linalg.solve(left[0],left[1]))
            records.append({'id':case,'condition':name,'beta':beta,'motion':'beta'+str(beta),'path':str(path),'sha256':sha(path),'support_fraction':float(support.mean())})
    write(out/'inventory.json',{'created_utc':now(),'records':records,'arms':ARMS,'views':72,'protocol_sha256':sha(p['protocol_path']),'convergence':convergence,'scope':'Static artificial4mplane; virtual complete exponential history with stationary prehistory. No exact integrated-pixel flow labels.'})
    write(out/'artifact-lock.json',{'files':{x.name:sha(x) for x in out.iterdir() if x.is_file()},'created_utc':now()})

def capture(p,out):
    import torch
    out.mkdir(parents=True,exist_ok=False);deterministic();model,loading=load_model(p['checkpoint']);assert loading['migration']=='none'
    state=lambda:{k:hashlib.sha256(v.cpu().numpy().tobytes()).hexdigest() for k,v in model.state_dict().items()};original=state()
    selector,cov_model=native_algorithms();camera={'K':p['K'],'baseline_m':p['baseline_m']};cases=read(p['inventory'])['records'];rows=[];dense_rows=[];null_checks=0
    for condition in ['day','night','rain']:
        group=[c for c in cases if c['condition']==condition];dense=[];meta=[];mask=None
        for case in group:
            assert sha(case['path'])==case['sha256']
            with np.load(case['path']) as z:
                images=z['images'].astype(np.float32)/255;current=safe_mask(z['common_pixel_support'])
            if mask is None:mask=current
            else:assert np.array_equal(mask,current)
            for j,arm in enumerate(ARMS):
                im=images[j];src=np.stack([im[0,0],im[1,0],im[0,0]]);dst=np.stack([im[0,1],im[1,1],im[1,0]])
                a=torch.from_numpy(np.repeat(src[:,None],3,axis=1)).cuda();b=torch.from_numpy(np.repeat(dst[:,None],3,axis=1)).cuda()
                with torch.inference_mode():f,c=model.inference(a,b)
                assert torch.isfinite(f).all() and torch.isfinite(c).all() and (c>0).all()
                f,c=f.cpu().numpy(),c.cpu().numpy();dense.append((f,c));folder=out/case['id']/arm;folder.mkdir(parents=True);path=folder/'dense.npz';np.savez_compressed(path,flow=f,covariance=c)
                ids=pick(f,c,camera,selector,p['selection_seed'])
                m={'case':case['id'],'arm':arm,'condition':condition,'beta':case['beta'],'dense_path':str(path),'dense_sha256':sha(path),'source_input_sha256':hashlib.sha256(src.tobytes()).hexdigest(),'target_input_sha256':hashlib.sha256(dst.tobytes()).hexdigest(),'native_proposed_ids':ids.tolist(),'median_temporal_covariance_trace':float(np.median(c[2].sum(0)[mask]))};meta.append((m,folder,ids));dense_rows.append(m)
        for i in [2,4]:
            assert all(np.array_equal(x,y) for x,y in zip(dense[0],dense[i])),'Sharp dense null';null_checks+=1
        def solve(i,mode,ids):
            m,folder,_=meta[i]
            try:
                packet=build(*dense[i],camera,ids,cov_model,mask,p['minimum_points']);path=folder/(mode+'.npz');np.savez_compressed(path,**packet)
                result=fit(packet,p['solver']);result.update(packet_path=str(path),packet_sha256=sha(path),accepted_ids=packet['source_ids'].tolist(),points=len(packet['source_ids']))
            except (ValueError,np.linalg.LinAlgError) as exc:result={'successful':False,'status':'INSUFFICIENT_OR_INVALID_PACKET','reason':str(exc),'accepted_ids':[]}
            row={'case':m['case'],'arm':m['arm'],'mode':mode,'requested_ids':np.asarray(ids).tolist(),'fit':result};rows.append(row);write(folder/(mode+'-receipt.json'),row);print(m['case'],m['arm'],mode,result['status'],flush=True);return result
        primaries=[solve(i,'regenerated',x[2]) for i,x in enumerate(meta)]
        common=joint_ids(dense,primaries[0]['accepted_ids'],camera,cov_model,mask,p['minimum_points'])
        fixed=[solve(i,'common_ids',common) for i in range(6)]
        for fits in [primaries,fixed]:
            for i in [2,4]:
                assert fits[i]['successful']==fits[0]['successful'] and fits[i].get('target_to_source')==fits[0].get('target_to_source') and fits[i].get('information_covariance')==fits[0].get('information_covariance'),'Sharp estimator null';null_checks+=1
    assert state()==original
    write(out/'predictions.json',{'created_utc':now(),'rows':rows,'dense_rows':dense_rows,'loading':loading,'model_state_unchanged':True,'true_pose_values_read':False,'sharp_dense_pose_null_checks':null_checks})
    write(out/'prediction-lock.json',{'created_utc':now(),'files':{str(x.relative_to(out)):sha(x) for x in out.rglob('*') if x.is_file()}})

def score(p,out):
    folder=Path(p['capture_output'])
    for path,h in read(folder/'prediction-lock.json')['files'].items():assert sha(folder/path)==h
    pred=read(folder/'predictions.json');cases=read(p['inventory'])['records'];rows=[]
    assert pred['sharp_dense_pose_null_checks']==18
    for entry in pred['rows']:
        case=next(x for x in cases if x['id']==entry['case']);result=entry['fit'];metrics=None
        with np.load(case['path']) as z:truth=z['target_to_source']
        if result['successful']:
            metrics=error(np.array(result['target_to_source']),truth);P=np.array(result['information_covariance'])
            metrics.update(translation_information_std_proxy_m=float(np.sqrt(np.trace(P[:3,:3]))),rotation_information_std_proxy_deg=float(np.rad2deg(np.sqrt(np.trace(P[3:,3:])))))
        meta=next(x for x in pred['dense_rows'] if x['case']==entry['case'] and x['arm']==entry['arm'])
        rows.append({'case':case['id'],'condition':case['condition'],'beta':case['beta'],'arm':entry['arm'],'mode':entry['mode'],'successful':result['successful'],'status':result['status'],'points':result.get('points',0),'median_temporal_covariance_trace':meta['median_temporal_covariance_trace'],'metrics':metrics})
    out.mkdir(parents=True,exist_ok=False);write(out/'scores.json',{'created_utc':now(),'rows':rows,'planned_dense_batches':18,'planned_pose_attempts':36,'prediction_lock_sha256':sha(folder/'prediction-lock.json'),'protocol_sha256':sha(p['protocol_path']),'scope':'Fixed actions, no learned/selected scheduling policy; nativefrontend with opticalIRLS, not fullnativeVO or physicalsensor model.'})

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['render','qualify','capture','score']);ap.add_argument('--protocol',required=True);ap.add_argument('--output',required=True);a=ap.parse_args();p=read(a.protocol);verify(p)
    {'render':render,'qualify':qualify,'capture':capture,'score':score}[a.mode](p,Path(a.output));verify(p)
