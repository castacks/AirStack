"""Fixed-budget selection on accepted native matches; no learned component."""
import argparse,hashlib,json,time
from pathlib import Path
from datetime import datetime,timezone
import numpy as np
from scipy.spatial.transform import Rotation


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def write(p,d):
    with Path(p).open('x') as f:json.dump(d,f,indent=2,allow_nan=False);f.write('\n')
def stamp():return datetime.now(timezone.utc).isoformat()
def skew(x):
    a,b,c=np.asarray(x).T;z=np.zeros_like(a)
    return np.stack([z,-c,b,c,z,-a,-b,a,z],axis=-1).reshape(np.shape(x)[:-1]+(3,3))
def points(uv,d,K):
    return np.column_stack([uv,np.ones(len(uv))])@np.linalg.inv(K).T*d[:,None]

def prepare(z,pair):
    ids=np.flatnonzero((z['edge/match2frame1/mapping']==pair)&(z['edge/match2frame2/mapping']==pair+1))
    uv0=z['match//pixel1_uv'][ids].astype(float);uv1=z['match//pixel2_uv'][ids].astype(float)
    d0=z['match//pixel1_d'][ids].reshape(-1).astype(float);d1=z['match//pixel2_d'][ids].reshape(-1).astype(float)
    K0=z['frames//K'][pair].astype(float);K1=z['frames//K'][pair+1].astype(float)
    C0=z['match//obs1_covTc'][ids].astype(float);C1=z['match//obs2_covTc'][ids].astype(float)
    if len(ids)<3 or not all(np.isfinite(x).all() for x in [uv0,uv1,d0,d1,K0,K1,C0,C1]) or np.any(d0<=0) or np.any(d1<=0):raise ValueError('INVALID_ACCEPTED_POOL')
    Q=np.array([[0.,1,0],[0,0,1],[1,0,0]])
    S=Q@(C0+C1)@Q.T
    if not np.allclose(S,S.transpose(0,2,1),rtol=0,atol=1e-12):raise ValueError('ASYMMETRIC_COVARIANCE')
    np.linalg.cholesky(S) # Reject the whole packet, never prune factors or fit floors.
    return {'ids':ids,'uv':uv0,'X0':points(uv0,d0,K0),'X1':points(uv1,d1,K1),'S':S,'length':np.array(float(np.median(d0)))}

def system(p,R,t):
    q=p['X1']@R.T+t;r=q-p['X0'];J=np.concatenate([np.broadcast_to(np.eye(3),(len(q),3,3)),-skew(q)],axis=2)
    J[:,:,:3]*=float(p['length']);L=np.linalg.cholesky(p['S'])
    return np.linalg.solve(L,r[...,None])[...,0].reshape(-1),np.linalg.solve(L,J).reshape(-1,6)

def info_blocks(p):
    _,A=system(p,np.eye(3),np.zeros(3));B=A.reshape(-1,3,6)
    return np.einsum('nji,njk->nik',B,B)

def select(p,method,k,seed=None,image_shape=None):
    n=len(p['ids']);k=min(k,n)
    if method=='all_pool' or k==n:return np.arange(n,dtype=int)
    rng=np.random.default_rng(seed)
    if method=='uniform':return np.sort(rng.choice(n,k,replace=False))
    if method=='spatial':
        h,w=image_shape;uv=p['uv'];cell=np.minimum(np.maximum(np.floor(uv/np.array([w,h])*4).astype(int),0),3)
        keys=cell[:,1]*4+cell[:,0];groups=[list(rng.permutation(np.flatnonzero(keys==c))) for c in range(16)]
        cells=rng.permutation(16);chosen=[]
        while len(chosen)<k:
            for cell_id in cells:
                if groups[cell_id]:chosen.append(groups[cell_id].pop())
                if len(chosen)==k:break
        return np.sort(chosen)
    if method=='reliability':return np.sort(np.lexsort((p['ids'],np.trace(p['S'],axis1=1,axis2=2)))[:k])
    if method=='information':
        blocks=info_blocks(p);H=blocks.sum(axis=0);ridge=1e-6*np.trace(H)/6
        if not np.isfinite(blocks).all() or not ridge>0:raise ValueError('INVALID_INFORMATION')
        current=np.eye(6)*ridge;available=np.ones(n,bool);chosen=[]
        for _ in range(k):
            ids=np.flatnonzero(available);sign,logdet=np.linalg.slogdet(current[None]+blocks[ids]);assert np.all(sign>0)
            best=ids[int(np.argmax(logdet))];chosen.append(best);available[best]=False;current+=blocks[best]
        return np.sort(chosen)
    raise ValueError(method)

def subset(p,idx):return {k:(v[idx] if k!='length' else v) for k,v in p.items()}

def solve(p,settings):
    R=np.eye(3);t=np.zeros(3);history=[]
    status='ITERATION_CAP';iterations=0
    for iteration in range(settings['max_iterations']+1):
        r,A=system(p,R,t);cost=float(r@r);g=A.T@r;singular=np.linalg.svd(A,compute_uv=False)
        gradient=float(np.max(np.abs(g)));limit=settings['gradient_relative']*max(1.,singular[0]*np.linalg.norm(r))
        history.append(cost)
        if singular[-1]<=settings['rank_ratio']*singular[0]:status='DEGENERATE_GEOMETRY';break
        if gradient<=limit:status='CONVERGED_GRADIENT';break
        if iteration==settings['max_iterations']:break
        delta=np.linalg.lstsq(A,-r,rcond=settings['rank_ratio'])[0];direction=float(g@delta)
        if not np.isfinite(delta).all():status='NONFINITE_STEP';break
        accepted=False
        for exponent in range(settings['line_search_steps']):
            step=2.**(-exponent);dR=Rotation.from_rotvec(step*delta[3:]).as_matrix()
            candidate_R=dR@R;candidate_t=dR@t+step*float(p['length'])*delta[:3]
            rr,_=system(p,candidate_R,candidate_t);candidate_cost=float(rr@rr)
            if candidate_cost<=cost+settings['armijo']*step*2*direction:
                R,t=candidate_R,candidate_t;iterations+=1;accepted=True;break
        if not accepted:status='LINE_SEARCH_FAILED';break
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=t
    return {'status':status,'successful':status=='CONVERGED_GRADIENT','iterations':iterations,'target_to_source':T.tolist(),'final_cost':cost,'gradient_inf':gradient,'gradient_limit':float(limit),'singular_values':singular.tolist(),'cost_history':history}

def arms(seeds):
    return [(f'{method}_{s}',method,s) for method in ['uniform','spatial'] for s in seeds]+[(m,m,None) for m in ['reliability','information','all_pool']]

def verify(plan):
    for name,h in plan['pins'].items():assert sha(name)==h,name

def capture(plan,out):
    packets=out/'packets';packets.mkdir();rows=[]
    for stream in plan['streams']:
      with np.load(stream['graph_path']) as z:
       for pair in range(19):
        c=stream['condition'];camera=stream['camera'];prefix=f'{c}-{camera}-{pair:03}'
        try:p=prepare(z,pair)
        except (ValueError,np.linalg.LinAlgError) as e:
            rows.append({'condition':c,'camera':camera,'pair':pair,'status':'UNAVAILABLE_POOL','reason':str(e),'methods':{name:{'status':'UNAVAILABLE_POOL','successful':False} for name,_,_ in arms(plan['seeds'])}});continue
        path=packets/(prefix+'.npz');np.savez_compressed(path,**p)
        methods={}
        for name,method,seed in arms(plan['seeds']):
            mixed=int.from_bytes(hashlib.sha256(f'{seed}:{c}:{camera}:{pair}'.encode()).digest()[:8],'little') if seed is not None else None
            started=time.perf_counter();idx=select(p,method,plan['budget'],mixed,plan['image_shapes'][camera]);selection_seconds=time.perf_counter()-started
            selected=subset(p,idx)
            try:result=solve(selected,plan['solver'])
            except (ValueError,np.linalg.LinAlgError) as e:result={'status':'NUMERICAL_FAILURE','successful':False,'reason':str(e)}
            blocks=info_blocks(selected);eig=np.linalg.eigvalsh(blocks.sum(0))
            methods[name]={**result,'selected_local_ids':idx.tolist(),'selected_native_ids':p['ids'][idx].tolist(),'selected_count':len(idx),'selection_seconds':selection_seconds,'information_eigenvalues':eig.tolist(),'derived_seed':mixed}
        row={'condition':c,'camera':camera,'pair':pair,'status':'AVAILABLE_POOL','pool_size':len(p['ids']),'packet_path':str(path.relative_to(out)),'packet_sha256':sha(path),'methods':methods};rows.append(row)
        with (out/'journal.jsonl').open('a') as f:f.write(json.dumps(row,allow_nan=False)+'\n')
        print(prefix,{name:v['status'] for name,v in methods.items()},flush=True)
    write(out/'predictions.json',{'created_utc':stamp(),'rows':rows,'reference_values_parsed':False})
    write(out/'prediction-lock.json',{'created_utc':stamp(),'files':{str(p.relative_to(out)):sha(p) for p in out.rglob('*') if p.is_file()},'reference_values_parsed':False,'camera_pairs':len(rows),'attempts':sum(len(r['methods']) for r in rows)})

def metric_summary(values):
    return {key:float(np.mean(values)) if key=='mean' else float(np.sqrt(np.mean(np.square(values)))) for key in ['mean','rmse']} if len(values) else {'mean':None,'rmse':None}

def summarize(rows,names):
    result={}
    for name in names:
        successful=[r['methods'][name]['errors'] for r in rows if r['methods'][name]['successful']]
        result[name]={'attempted':len(rows),'successful':len(successful),'translation_m':metric_summary([v['translation_m'] for v in successful]),'rotation_deg':metric_summary([v['rotation_deg'] for v in successful])}
    return result

def evaluate(plan,out):
    lock=read(out/'prediction-lock.json');assert lock['reference_values_parsed'] is False
    for name,h in lock['files'].items():assert sha(out/name)==h
    predictions=read(out/'predictions.json');rows=[]
    for r in predictions['rows']:
        c=r['condition'];camera=r['camera'];pair=r['pair'];p0=np.loadtxt(plan['references'][c][pair]).reshape(4,4);p1=np.loadtxt(plan['references'][c][pair+1]).reshape(4,4)
        for P in [p0,p1]:assert np.isfinite(P).all() and np.linalg.norm(P[:3,:3].T@P[:3,:3]-np.eye(3))<1e-5
        truth=np.linalg.solve(p0,p1)
        if camera=='thermal':
            E=np.array(plan['thermal_from_rgb'][c]);truth=E@truth@np.linalg.inv(E)
        methods={}
        for name,pred in r['methods'].items():
            err=None
            if pred['successful']:
                T=np.array(pred['target_to_source']);err={'translation_m':float(np.linalg.norm(T[:3,3]-truth[:3,3])),'rotation_deg':float(np.degrees(Rotation.from_matrix(truth[:3,:3].T@T[:3,:3]).magnitude()))}
            methods[name]={'successful':pred['successful'],'status':pred['status'],'errors':err}
        rows.append({'condition':c,'camera':camera,'pair':pair,'truth_target_to_source':truth.tolist(),'methods':methods})
    streams={};names=[a[0] for a in arms(plan['seeds'])]
    for c in ['day','night','rain']:
      for camera in ['rgb','thermal']:
        rr=[r for r in rows if r['condition']==c and r['camera']==camera];own=summarize(rr,names);comparisons={}
        for control in ['uniform','spatial']:
            controls=[f'{control}_{s}' for s in plan['seeds']]
            common=[r for r in rr if all(r['methods'][n]['successful'] for n in controls+['information'])]
            ss=summarize(common,['information']+controls);means={}
            for metric in ['translation_m','rotation_deg']:
                means[metric]={stat:float(np.mean([ss[n][metric][stat] for n in controls])) if common else None for stat in ['mean','rmse']}
            coverage=all(own['information']['successful']>=own[n]['successful'] for n in controls)
            passed=bool(common) and coverage and all(ss['information']['translation_m'][s]<=.95*means['translation_m'][s] and ss['information']['rotation_deg'][s]<=means['rotation_deg'][s] for s in ['mean','rmse'])
            comparisons[control]={'matched_pair_count':len(common),'matched_pair_ids':[r['pair'] for r in common],'information':ss['information'],'control_per_seed':{n:ss[n] for n in controls},'control_seed_mean':means,'coverage_not_lower':coverage,'pass':passed}
        streams[c+'-'+camera]={'own':own,'comparisons':comparisons,'primary_pass':all(v['pass'] for v in comparisons.values())}
    write(out/'scores.json',{'created_utc':stamp(),'rows':rows,'streams':streams,'prediction_lock_sha256':sha(out/'prediction-lock.json'),'decision':'ADVANCE_TO_SEPARATE_INTEGRATION_REVIEW' if all(s['primary_pass'] for s in streams.values()) else 'CLOSE_THIS_ACCEPTED_POOL_SELECTION_CONFIGURATION_NO_RETUNING','scope':'Nominal supplied camera-local motion, exposed development; fixed-covariance3D registration, not native full-image selector/VO reproduction.'})
    print(json.dumps({k:v['primary_pass'] for k,v in streams.items()}))

def main():
    ap=argparse.ArgumentParser();ap.add_argument('stage',choices=['capture','evaluate']);ap.add_argument('--protocol',required=True);ap.add_argument('--output',required=True);a=ap.parse_args();p=read(a.protocol);verify(p);out=Path(a.output)
    if a.stage=='capture':out.mkdir(parents=True,exist_ok=False);capture(p,out)
    else:evaluate(p,out)
    verify(p)
if __name__=='__main__':main()
