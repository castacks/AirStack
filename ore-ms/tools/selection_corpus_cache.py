"""Materialize the fixed policy corpus and cache unchanged native predictions."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.ndimage import map_coordinates
from scipy.spatial.transform import Rotation

from selection_policy_contract import ray_hit, visible_projection, utility_features, training_statistics
from paired_frontend_assessment import native_algorithms, pick, packet
from qualify_native_frontend import deterministic, load_model, now, read, recipe, sha, write


def verify(plan):
    for path, digest in plan['pins'].items():
        assert sha(path) == digest, path


def cameras(plan, motion):
    R = Rotation.from_rotvec(np.deg2rad(motion['rotvec_deg'])).as_matrix()
    t, b = np.asarray(motion['translation_m']), np.array([plan['baseline_m'], 0., 0.])
    rotations = np.stack([np.eye(3), np.eye(3), R, R])
    centers = np.stack([np.zeros(3), b, t, t+R@b])
    T = np.eye(4);T[:3,:3],T[:3,3] = R,t
    return rotations,centers,T


def projected_reference(world,K,rotation,center,geometry):
    local=(world-center)@rotation
    homogeneous=local@K.T
    uv=homogeneous[...,:2]/homogeneous[...,2:]
    for coordinate,high in [(uv[...,0],639),(uv[...,1],255)]:
        coordinate[np.abs(coordinate)<1e-9]=0.
        coordinate[np.abs(coordinate-high)<1e-9]=float(high)
    hit,_,valid=ray_hit(uv,K,rotation,center,geometry)
    inside=(uv[...,0]>=0)&(uv[...,0]<=639)&(uv[...,1]>=0)&(uv[...,1]<=255)
    return uv,valid&(local[...,2]>0)&inside&(np.linalg.norm(hit-world,axis=-1)<=1e-7)


def render(plan, out):
    base = read(plan['corpus_contract'])
    K = np.asarray(base['K'])
    textures = {t['id']:t for t in base['textures']}
    geometries = {g['id']:g for g in base['geometries']}
    motions = {m['id']:m for m in base['motions']}
    vv,uu = np.indices((256,640));pixels = np.stack([uu,vv],-1)
    prepare = recipe();current_id=None;texture=None;records=[]
    for i,scene in enumerate(base['scenes']):
        if current_id!=scene['texture_id']:
            source=textures[scene['texture_id']];assert sha(source['path'])==source['sha256']
            texture=prepare(cv2.imread(source['path'],cv2.IMREAD_UNCHANGED));current_id=scene['texture_id']
            assert texture.shape==(256,640) and texture.dtype==np.uint8
        geometry=geometries[scene['geometry_id']];motion=motions[scene['motion_id']]
        rotations,centers,T=cameras(base,motion)
        worlds=[];surfaces=[];support=[];images=[]
        for R,c in zip(rotations,centers,strict=True):
            world,surface,valid=ray_hit(pixels,K,R,c,geometry)
            assert np.all(valid)
            canonical=world@K.T;uv=canonical[...,:2]/canonical[...,2:]
            for coordinate,high in [(uv[...,0],639),(uv[...,1],255)]:
                coordinate[np.abs(coordinate)<1e-9]=0.
                coordinate[np.abs(coordinate-high)<1e-9]=float(high)
            supported=(uv[...,0]>=0)&(uv[...,0]<=639)&(uv[...,1]>=0)&(uv[...,1]<=255)
            sample=map_coordinates(texture.astype(float)/255,[uv[...,1],uv[...,0]],order=1,mode='constant',cval=0.,prefilter=False)
            images.append(np.rint(np.clip(sample,0,1)*255).astype(np.uint8));worlds.append(world);surfaces.append(surface.astype(np.uint8));support.append(supported)
        flows=[];visible=[]
        for source,target in [(0,1),(2,3),(0,2)]:
            uv,seen=projected_reference(worlds[source],K,rotations[target],centers[target],geometry)
            flows.append((uv-pixels).transpose(2,0,1))
            visible.append(seen&support[source])
        path=out/'scenes'/(scene['id']+'.npz')
        np.savez_compressed(path,images=np.asarray(images),surface=np.asarray(surfaces),texture_valid=np.asarray(support),true_flow=np.asarray(flows),visible=np.asarray(visible),K=K,rotations=rotations,centers=centers,target_to_source=T)
        prior_repeat=False
        if scene['id'] in plan.get('prior_complete_scene_ids',[]):
            with np.load(Path(plan['prior_render_attempt'])/'scenes'/(scene['id']+'.npz')) as prior:
                assert np.array_equal(prior['images'],np.asarray(images)), 'prior image changed'
                assert np.array_equal(prior['surface'],np.asarray(surfaces)), 'prior surface changed'
                assert np.array_equal(prior['texture_valid'],np.asarray(support)), 'prior atlas changed'
            prior_repeat=True
        record={**scene,'prior_image_bitwise_repeat':prior_repeat,'path':str(path),'sha256':sha(path),'texture_sha256':textures[scene['texture_id']]['sha256'],'support_fractions':np.mean(support,axis=(1,2)).tolist(),'visible_fractions':np.mean(visible,axis=(1,2)).tolist()}
        records.append(record)
        with (out/'journal.jsonl').open('a') as f:f.write(json.dumps(record)+'\n')
        if (i+1)%50==0:print('rendered',i+1,flush=True)
    write(out/'inventory.json',{'created_utc':now(),'records':records,'contract_sha256':sha(plan['corpus_contract']),'no_native_predictions':True,'prior_images_bitwise_repeated':sum(r['prior_image_bitwise_repeat'] for r in records)})
    write(out/'artifact-lock.json',{'created_utc':now(),'files':{str(p.relative_to(out)):sha(p) for p in out.rglob('*') if p.is_file()}})


def capture(plan,out):
    import torch
    deterministic();model,loading=load_model(plan['checkpoint']);assert loading['migration']=='none'
    selector,covariance=native_algorithms();base=read(plan['corpus_contract']);camera={'K':base['K'],'baseline_m':base['baseline_m']}
    state=lambda:{k:hashlib.sha256(v.cpu().numpy().tobytes()).hexdigest() for k,v in model.state_dict().items()}
    initial=state();rows=[]
    inventory=read(plan['render_inventory'])
    def inference(images):
        a=np.stack([images[0],images[2],images[0]]);b=np.stack([images[1],images[3],images[2]])
        aa=torch.from_numpy(np.repeat(a[:,None],3,1)).cuda();bb=torch.from_numpy(np.repeat(b[:,None],3,1)).cuda()
        with torch.inference_mode():flow,cov=model.inference(aa,bb)
        assert torch.isfinite(flow).all() and torch.isfinite(cov).all() and (cov>0).all()
        return flow.cpu().numpy(),cov.cpu().numpy()
    for i,scene in enumerate(inventory['records']):
        assert sha(scene['path'])==scene['sha256']
        # References are not opened during native prediction/candidate capture.
        with np.load(scene['path']) as z:images=z['images'].astype(np.float32)/255
        flow,cov=inference(images)
        dense=out/'dense'/(scene['id']+'.npz');np.savez_compressed(dense,flow=flow,covariance=cov)
        if i==0:
            f2,c2=inference(images);assert np.array_equal(flow,f2) and np.array_equal(cov,c2)
            np.savez_compressed(out/'null-repeat.npz',flow=f2,covariance=c2)
        ids=pick(flow,cov,camera,selector,20261114)
        row={'id':scene['id'],'group':scene['group'],'partition':scene['partition'],'render_sha256':scene['sha256'],'dense_path':str(dense),'dense_sha256':sha(dense),'proposed_source_ids':ids.tolist()}
        try:
            p=packet(flow,cov,camera,ids,covariance)
            order=np.lexsort((p['source_ids'][:,0],p['source_ids'][:,1]));n=len(order)
            p={k:(v[order] if v.ndim>0 and len(v)==n else v) for k,v in p.items()}
            uv=p['source_ids'];y,x=uv[:,1],uv[:,0]
            features=utility_features(p,cov[2,:,y,x],flow[2,:,y,x],np.asarray(base['K']))
            path=out/'packets'/(scene['id']+'.npz');np.savez_compressed(path,**p,features=features)
            row.update(status='AVAILABLE',points=n,packet_path=str(path),packet_sha256=sha(path))
        except (ValueError,np.linalg.LinAlgError) as error:
            row.update(status='INVALID_PACKET',reason=str(error))
        rows.append(row)
        with (out/'journal.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
        if (i+1)%50==0:print('cached',i+1,{s:sum(r['status']==s for r in rows) for s in {r['status'] for r in rows}},flush=True)
    assert initial==state()
    write(out/'predictions.json',{'created_utc':now(),'rows':rows,'loading':loading,'model_state_unchanged':True,'reference_values_read':False,'primary_dense_batches':len(rows),'null_dense_batches':1})
    write(out/'prediction-lock.json',{'created_utc':now(),'files':{str(p.relative_to(out)):sha(p) for p in out.rglob('*') if p.is_file()}})


def labels_and_statistics(plan,out):
    base=read(plan['corpus_contract']);cache=Path(plan['native_cache'])
    for rel,digest in read(cache/'prediction-lock.json')['files'].items():assert sha(cache/rel)==digest,rel
    rows={r['id']:r for r in read(cache/'predictions.json')['rows']};renders={r['id']:r for r in read(plan['render_inventory'])['records']};labels=[]
    for key,row in rows.items():
        if row['status']!='AVAILABLE':continue
        with np.load(row['packet_path']) as p:
            uv=p['source_ids'];x,y=uv[:,0],uv[:,1];predicted=p['target_uv'].copy()
        with np.load(renders[key]['path']) as ref:
            true_target=uv+ref['true_flow'][2,:,y,x]
            visible=ref['visible'][2,y,x]
        errors=np.sum((predicted-true_target)**2,axis=1)
        reward=np.exp(-errors/2)*visible
        path=out/'labels'/(key+'.npz');np.savez_compressed(path,reward=reward,visible=visible,true_target=true_target)
        labels.append({'id':key,'path':str(path),'sha256':sha(path)})
    statistics=[]
    for fold in base['folds']:
        sources=[rows[key] for key in fold['training_scene_ids'] if rows[key]['status']=='AVAILABLE'];arrays=[]
        for row in sources:
            with np.load(row['packet_path']) as p:arrays.append(p['features'].copy())
        if not arrays:raise ValueError('NO_TRAINING_FEATURES')
        features=np.concatenate(arrays);mean,scale=training_statistics(features)
        record={'excluded_group':fold['excluded_group'],'candidate_count':len(features),'source_scene_ids':[r['id'] for r in sources],'unavailable_scene_ids':[key for key in fold['training_scene_ids'] if rows[key]['status']!='AVAILABLE'],'mean':mean.tolist(),'scale':scale.tolist()}
        path=out/(fold['excluded_group']+'-statistics.json');write(path,record);statistics.append({'path':str(path),'sha256':sha(path),'excluded_group':fold['excluded_group']})
    write(out/'inventory.json',{'created_utc':now(),'labels':labels,'statistics':statistics,'prediction_lock_sha256':sha(cache/'prediction-lock.json'),'scope':'Reference reward labels separate from native candidate selection/features; normalization from training candidates only. No utility fitting.'})
    write(out/'artifact-lock.json',{'created_utc':now(),'files':{str(p.relative_to(out)):sha(p) for p in out.rglob('*') if p.is_file()}})
    print('labels',len(labels),'statistics',len(statistics),flush=True)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('stage',choices=['render','capture','labels']);parser.add_argument('--protocol',required=True);parser.add_argument('--output',required=True);args=parser.parse_args()
    plan,out=read(args.protocol),Path(args.output);verify(plan);out.mkdir(parents=True,exist_ok=False)
    for folder in {'render':['scenes'],'capture':['dense','packets'],'labels':['labels']}[args.stage]:(out/folder).mkdir()
    {'render':render,'capture':capture,'labels':labels_and_statistics}[args.stage](plan,out)
    verify(plan)


if __name__=='__main__':main()
