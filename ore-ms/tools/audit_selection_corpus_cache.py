"""Independent generalized image/reference and native-cache audits."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from audit_paired_frontend import independent_selection,point_covariance
from qualify_native_frontend import now,read,recipe,sha


def independent_hit(pixels,K,R,c,g):
    homogeneous=np.concatenate([pixels,np.ones(pixels.shape[:-1]+(1,))],-1)
    direction=(homogeneous@np.linalg.inv(K).T)@R.T
    first=(g['depths_m'][0]-c[2])/direction[...,2]
    candidate=c+first[...,None]*direction
    near=(candidate[...,0]*g['near_normal_xy'][0]+candidate[...,1]*g['near_normal_xy'][1]<=g['near_offset_m'])&(first>0)
    depth=np.where(near,g['depths_m'][0],g['depths_m'][1])
    distance=(depth-c[2])/direction[...,2]
    return c+distance[...,None]*direction,np.where(near,0,1)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('stage',choices=['render','cache']);parser.add_argument('--protocol',required=True);parser.add_argument('--output',required=True);a=parser.parse_args()
    plan,out=read(a.protocol),Path(a.output);base=read(plan['corpus_contract']);checks=0
    def check(value,text):
        nonlocal checks
        assert value,text;checks+=1
    def close(x,y,rtol=1e-10,atol=1e-10):
        nonlocal checks
        np.testing.assert_allclose(x,y,rtol=rtol,atol=atol);checks+=1
    for path,h in plan['pins'].items():check(sha(path)==h,path)
    def locked(folder,filename):
        for rel,h in read(folder/filename)['files'].items():check(sha(folder/rel)==h,rel)
    if a.stage=='render':
        locked(out,'artifact-lock.json');inventory=read(out/'inventory.json');records=inventory['records']
        check(inventory['contract_sha256']==sha(plan['corpus_contract']),'contract binding')
        expected={s['id']:s for s in base['scenes']};check(len(records)==600 and {s['id'] for s in records}==set(expected),'600 exact scenes')
        geometries={g['id']:g for g in base['geometries']};motions={m['id']:m for m in base['motions']};textures={t['id']:t for t in base['textures']}
        K=np.array(base['K']);vv,uu=np.indices((256,640));pixels=np.stack([uu,vv],-1);texture_cache={};ray_count=0;visibility_count=0
        for i,scene in enumerate(records):
            check(all(scene[k]==v for k,v in expected[scene['id']].items()),'scene metadata')
            check(sha(scene['path'])==scene['sha256'],'scene hash')
            g,m=geometries[scene['geometry_id']],motions[scene['motion_id']]
            R,_=cv2.Rodrigues(np.deg2rad(np.array(m['rotvec_deg'])));t=np.array(m['translation_m']);b=np.array([base['baseline_m'],0,0])
            rotations=np.array([np.eye(3),np.eye(3),R,R]);centers=np.array([np.zeros(3),b,t,t+R@b]);T=np.eye(4);T[:3,:3],T[:3,3]=R,t
            texture_id=scene['texture_id']
            if texture_id not in texture_cache:
                texture=textures[texture_id];check(sha(texture['path'])==texture['sha256']==scene['texture_sha256'],'raw texture')
                texture_cache[texture_id]=recipe()(cv2.imread(texture['path'],cv2.IMREAD_UNCHANGED)).astype(float)
            texture=texture_cache[texture_id]
            worlds=[];supports=[]
            with np.load(scene['path']) as z:
                close(z['K'],K);close(z['rotations'],rotations);close(z['centers'],centers);close(z['target_to_source'],T)
                check(z['images'].shape==(4,256,640) and z['images'].dtype==np.uint8,'native image shape')
                for view,(Q,c) in enumerate(zip(rotations,centers,strict=True)):
                    world,surface=independent_hit(pixels,K,Q,c,g);worlds.append(world)
                    check(np.array_equal(surface,z['surface'][view]),'nearest surface')
                    proj=world@K.T;uv=proj[...,:2]/proj[...,2:]
                    for coord,high in [(uv[...,0],639),(uv[...,1],255)]:
                        coord[np.abs(coord)<1e-9]=0.;coord[np.abs(coord-high)<1e-9]=float(high)
                    x,y=uv[...,0],uv[...,1];valid=(x>=0)&(x<=639)&(y>=0)&(y<=255);supports.append(valid)
                    check(np.array_equal(valid,z['texture_valid'][view]),'atlas mask')
                    x0=np.clip(np.floor(x).astype(int),0,639);y0=np.clip(np.floor(y).astype(int),0,255);x1=np.minimum(x0+1,639);y1=np.minimum(y0+1,255);dx=x-x0;dy=y-y0
                    code=(1-dy)*((1-dx)*texture[y0,x0]+dx*texture[y0,x1])+dy*((1-dx)*texture[y1,x0]+dx*texture[y1,x1])
                    expected_code=np.rint(np.where(valid,code,0)).astype(np.uint8);difference=np.abs(z['images'][view].astype(int)-expected_code.astype(int))
                    check(np.max(difference)<=1,'bilinear quantization bound')
                    check(np.all(np.abs(code[difference>0]-np.floor(code[difference>0])-.5)<1e-7),'only half-code ties differ')
                    check(np.all(difference[~valid]==0),'zero atlas boundary')
                    ray_count+=uu.size
                for pair,(source,target) in enumerate([(0,1),(2,3),(0,2)]):
                    world=worlds[source];local=(world-centers[target])@rotations[target];projected=local@K.T;uv=projected[...,:2]/projected[...,2:]
                    for coordinate,high in [(uv[...,0],639),(uv[...,1],255)]:
                        coordinate[np.abs(coordinate)<1e-9]=0.
                        coordinate[np.abs(coordinate-high)<1e-9]=float(high)
                    close(z['true_flow'][pair],(uv-pixels).transpose(2,0,1),atol=1e-8)
                    hit,_=independent_hit(uv,K,rotations[target],centers[target],g)
                    inside=(uv[...,0]>=0)&(uv[...,0]<=639)&(uv[...,1]>=0)&(uv[...,1]<=255)
                    # At the canvas boundary the independently computed pixels can differ by ULPs.
                    # Exact mask comparison remains required; profile choices are not tuned here.
                    visible=(local[...,2]>0)&inside&(np.linalg.norm(hit-world,axis=-1)<=1e-7)&supports[source]
                    check(np.array_equal(z['visible'][pair],visible),'independent true visibility')
                    close(scene['visible_fractions'][pair],visible.mean())
                    visibility_count+=uu.size
                close(scene['support_fractions'],np.mean(supports,axis=(1,2)))
            if (i+1)%100==0:print('render audit',i+1,flush=True)
        report={'created_utc':now(),'status':'PASS','checks':checks,'scenes':600,'view_ray_pixels':ray_count,'flow_visibility_pixels':visibility_count,'auditor_sha256':sha(__file__),'scope':'All materialized image, closest-surface, interpolation, exact-pose and three-pair visibility references. No native model scores.'}
        (out/'independent-audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True);return
    # Native cache and derived labels/statistics.
    cache=Path(plan['native_cache']);locked(cache,'prediction-lock.json');pred=read(cache/'predictions.json');rows=pred['rows'];rendered={r['id']:r for r in read(plan['render_inventory'])['records']}
    check(len(rows)==600 and {r['id'] for r in rows}==set(rendered),'all native scenes')
    check(pred['model_state_unchanged'] and pred['reference_values_read'] is False,'model/reference provenance');check(pred['loading']['migration']=='none','strict model')
    check(pred['primary_dense_batches']==600 and pred['null_dense_batches']==1,'all neural calls')
    K=np.array(base['K']);fb=np.float32(K[0,0]*base['baseline_m']);features_by_id={};row_by_id={r['id']:r for r in rows};invalid=0
    for i,row in enumerate(rows):
        check(row['render_sha256']==rendered[row['id']]['sha256'],'render binding');check(sha(row['dense_path'])==row['dense_sha256'],'dense hash')
        with np.load(row['dense_path']) as z:flow,cov=z['flow'],z['covariance']
        check(flow.shape==cov.shape==(3,2,256,640) and np.isfinite(flow).all() and np.isfinite(cov).all() and np.all(cov>0),'dense output contract')
        if i==0:
            with np.load(cache/'null-repeat.npz') as repeat:check(np.array_equal(flow,repeat['flow']) and np.array_equal(cov,repeat['covariance']),'null repeat')
        ids=independent_selection(cov,20261114);check(row['proposed_source_ids']==ids.tolist(),'native selection')
        if row['status']!='AVAILABLE':
            invalid+=1;check(row['status']=='INVALID_PACKET' and bool(row['reason']),'retained unavailable reason');continue
        check(sha(row['packet_path'])==row['packet_sha256'],'packet hash')
        target=ids.astype(np.float32)+flow[2,:,ids[:,1],ids[:,0]];good=(target[:,0]>32)&(target[:,0]<608)&(target[:,1]>32)&(target[:,1]<224);u0,u1=ids[good],target[good]
        depths=[fb/np.abs(flow[j,0]) for j in [0,1]];a0,b0=u0[:,0],u0[:,1];a1,b1=u1[:,0].astype(int),u1[:,1].astype(int)
        C0=point_covariance(depths[0],u0,np.full(len(u0),.25,np.float32),np.full(len(u0),.25,np.float32),K)
        C1=point_covariance(depths[1],u1,np.maximum(cov[2,0,b0,a0],.0625),np.maximum(cov[2,1,b0,a0],.0625),K)
        sane=np.isfinite(C0).all((1,2))&np.isfinite(C1).all((1,2));u0,u1,C0,C1=u0[sane],u1[sane],C0[sane],C1[sane]
        order=np.lexsort((u0[:,0],u0[:,1]));u0,u1,C0,C1=u0[order],u1[order],C0[order],C1[order]
        back=lambda uv,d:np.column_stack([(uv[:,0]-K[0,2])*d/K[0,0],(uv[:,1]-K[1,2])*d/K[1,1],d])
        X0=back(u0,depths[0][u0[:,1],u0[:,0]].astype(float));X1=back(u1,depths[1][u1[:,1].astype(int),u1[:,0].astype(int)].astype(float))
        with np.load(row['packet_path']) as p:
            check(np.array_equal(p['source_ids'],u0),'canonical admitted IDs');check(row['points']==len(u0),'count');close(p['target_uv'],u1,rtol=3e-5,atol=1e-8)
            close(p['X0'],X0,rtol=3e-5,atol=1e-8);close(p['X1'],X1,rtol=3e-5,atol=1e-8);close(p['C0'],C0,rtol=4e-4,atol=2e-7);close(p['C1'],C1,rtol=4e-4,atol=2e-7)
            # Feature arithmetic uses exact independently checked native inputs, with raw head values.
            y,x=u0[:,1],u0[:,0];v=cov[2,:,y,x];f=flow[2,:,y,x]
            expected=np.column_stack([(p['source_uv'][:,0]-K[0,2])/K[0,0],(p['source_uv'][:,1]-K[1,2])/K[1,1],np.log(p['X0'][:,2]),np.log(p['X1'][:,2]),np.log(np.trace(p['C0'],axis1=1,axis2=2)),np.log(np.trace(p['C1'],axis1=1,axis2=2)),np.log(v.sum(1)),np.log1p(np.sqrt((f*f).sum(1)))])
            close(p['features'],expected,rtol=3e-5,atol=1e-7);features_by_id[row['id']]=p['features'].copy()
        if (i+1)%100==0:print('cache audit',i+1,flush=True)
    derived=Path(plan['derived_cache']);locked(derived,'artifact-lock.json');inv=read(derived/'inventory.json');check(inv['prediction_lock_sha256']==sha(cache/'prediction-lock.json'),'labels after prediction lock')
    check({r['id'] for r in inv['labels']}==set(features_by_id),'all available labels')
    for label in inv['labels']:
        row=row_by_id[label['id']];check(sha(label['path'])==label['sha256'],'label binding')
        with np.load(row['packet_path']) as p, np.load(rendered[label['id']]['path']) as ref, np.load(label['path']) as l:
            x,y=p['source_ids'][:,0],p['source_ids'][:,1];truth=p['source_ids']+ref['true_flow'][2,:,y,x];visible=ref['visible'][2,y,x]
            close(l['true_target'],truth);check(np.array_equal(l['visible'],visible),'label visibility');close(l['reward'],np.exp(-np.sum((p['target_uv']-truth)**2,1)/2)*visible)
    for fold,source in zip(base['folds'],inv['statistics'],strict=True):
        record=read(source['path']);check(sha(source['path'])==source['sha256'],'statistics binding');check(record['excluded_group']==fold['excluded_group'],'fold identity')
        ids=[key for key in fold['training_scene_ids'] if key in features_by_id];check(record['source_scene_ids']==ids,'training-only scene list')
        check(not(set(ids)&set(fold['evaluation_scene_ids'])),'no evaluation normalization');features=np.concatenate([features_by_id[key] for key in ids]);mean=features.mean(0);scale=features.std(0);scale[scale==0]=1
        close(record['mean'],mean);close(record['scale'],scale);check(record['candidate_count']==len(features),'normalization denominator')
    report={'created_utc':now(),'status':'PASS','checks':checks,'primary_dense_batches':600,'null_dense_batches':1,'available_packets':len(features_by_id),'invalid_packets':invalid,'label_packets':len(inv['labels']),'training_only_normalizations':3,'auditor_sha256':sha(__file__),'scope':'Independent native selection/packet/features and reference-label/training-only statistics audit. No utility fitting or policy accuracy claim.'}
    (cache/'independent-audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)


if __name__=='__main__':main()
