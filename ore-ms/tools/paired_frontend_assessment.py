"""Paired fixed-source-ID vs regenerated-selection thermal frontend assessment."""
import ast,argparse,hashlib,json,time,types,copy
from pathlib import Path
from datetime import datetime,timezone
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import chi2
from qualify_native_frontend import ROOT,load_model,recipe,deterministic,read,write,sha,now
from pose_covariance_replay import exp_se3,log_se3,skew


def native_algorithms():
    import torch,math
    class ConfigBase:
        def __init__(self,config):self.config=config
    ns={'torch':torch,'math':math,'SimpleNamespace':types.SimpleNamespace,'IKeypointSelector':ConfigBase,'ICovariance2to3':ConfigBase}
    requested=[('Module/KeypointSelector.py',{'GridSelector','CovAwareSelector_NoDepth'}),('Utility/Math.py',{'gaussain_full_kernels'}),('Module/Covariance/Project2to3.py',{'MatchCovariance','Covariance_2to3_full','create_2x2_matrix','create_3x3_matrix'})]
    for rel,names in requested:
        path=ROOT/'MAC-VO-main'/rel;nodes=[copy.deepcopy(n) for n in ast.parse(path.read_text()).body if isinstance(n,(ast.ClassDef,ast.FunctionDef)) and n.name in names]
        assert {n.name for n in nodes}==names
        for node in nodes:
            for child in ast.walk(node):
                if isinstance(child,ast.FunctionDef):child.decorator_list=[] # Decorators omitted; call bodies unchanged and all inputs detached.
        future=ast.parse('from __future__ import annotations').body
        exec(compile(ast.Module(body=future+nodes,type_ignores=[]),str(path),'exec'),ns)
    selector=ns['CovAwareSelector_NoDepth'](types.SimpleNamespace(device='cpu',kernel_size=7,mask_width=32,max_match_cov=100.))
    covariance=ns['MatchCovariance'](types.SimpleNamespace(device='cpu',kernel_size=31,match_cov_default=.25,min_depth_cov=.05,min_flow_cov=.25))
    return selector,covariance

def frame_data(row):
    K=np.array(row['K']);return types.SimpleNamespace(fx=K[0,0],fy=K[1,1],cx=K[0,2],cy=K[1,2],height=256,width=640)

def dense_fields(flow,cov,row):
    import torch
    f=torch.from_numpy(np.array(flow,copy=True));v=torch.from_numpy(np.array(cov,copy=True));fb=row['K'][0][0]*row['baseline_m'];depths=[]
    for i in range(2):
        disp=f[i:i+1,:1].abs();dv=v[i:i+1,:1];depth=fb/disp;variance=(fb**2)*dv/disp**4
        depths.append(types.SimpleNamespace(depth=depth,cov=variance,disparity=disp,disparity_uncertainty=dv,mask=None))
    packed=torch.cat([v[2:3],torch.zeros_like(v[2:3,:1])],1)
    return depths,types.SimpleNamespace(flow=f[2:3],cov=packed,mask=None)

def pick(flow,cov,row,selector,seed):
    import torch
    depths,match=dense_fields(flow,cov,row);torch.manual_seed(seed)
    return selector.select_point(frame_data(row),200,depths[0],depths[1],match).numpy().astype(np.int64)

def packet(flow,cov,row,source_uv,covariance):
    import torch
    depths,match=dense_fields(flow,cov,row);uv0=torch.from_numpy(np.array(source_uv,copy=True)).float();temporal=match.flow[0,:,uv0[:,1].long(),uv0[:,0].long()].T;uv1=uv0+temporal
    inbound=(uv1[:,0]>32)&(uv1[:,0]<608)&(uv1[:,1]>32)&(uv1[:,1]<224);uv0=uv0[inbound];uv1=uv1[inbound]
    def values(field,uv):return field[0,:,uv[:,1].long(),uv[:,0].long()].T
    d0=values(depths[0].depth,uv0)[:,0];d1=values(depths[1].depth,uv1)[:,0]
    s0=torch.ones((len(uv0),3),dtype=torch.float32)*.25;s0[:,2]=0
    s1=values(match.cov,uv0).clone();frame=frame_data(row)
    C0=covariance.estimate(frame,uv0,depths[0],values(depths[0].cov,uv0)[:,0],s0).numpy()
    C1=covariance.estimate(frame,uv1,depths[1],values(depths[1].cov,uv1)[:,0],s1).numpy()
    sane=np.isfinite(C0).all((1,2))&np.isfinite(C1).all((1,2))
    ids=np.asarray(source_uv)[inbound.numpy()][sane];u0=uv0.numpy()[sane];u1=uv1.numpy()[sane];d0=d0.numpy()[sane].astype(float);d1=d1.numpy()[sane].astype(float)
    K=np.array(row['K']);back=lambda uv,d:np.column_stack([(uv[:,0]-K[0,2])*d/K[0,0],(uv[:,1]-K[1,2])*d/K[1,1],d])
    C0=C0[sane][:,[1,2,0]][:,:,[1,2,0]].astype(float);C1=C1[sane][:,[1,2,0]][:,:,[1,2,0]].astype(float)
    X0=back(u0.astype(float),d0);X1=back(u1.astype(float),d1)
    if len(ids)<3 or not np.isfinite(X0).all() or not np.isfinite(X1).all() or np.any(d0<=0) or np.any(d1<=0):raise ValueError('INVALID_MEAN_OR_SUPPORT')
    np.linalg.cholesky(C0);np.linalg.cholesky(C1)
    return {'source_ids':ids,'source_uv':u0,'target_uv':u1,'X0':X0,'X1':X1,'C0':C0,'C1':C1,'proposed_count':np.array(len(source_uv)),'inbound_count':np.array(int(inbound.sum())),'length':np.array(float(np.median(d0)))}

def frozen_system(p,T):
    R=T[:3,:3];q=p['X1']@R.T+T[:3,3];res=q-p['X0'];C=p['C0']+R@p['C1']@R.T;L=np.linalg.cholesky(C)
    J=np.concatenate([np.broadcast_to(np.eye(3),(len(q),3,3)),-skew(q)],2);D=np.diag([float(p['length'])]*3+[1.]*3)
    r=np.linalg.solve(L,res[...,None])[...,0].reshape(-1);A=np.linalg.solve(L,J@D).reshape(-1,6)
    return r,A,L,D

def fit(p,settings):
    T=np.eye(4);history=[];status='ITERATION_CAP';covariance=None
    for iteration in range(settings['max_iterations']+1):
        r,A,L,D=frozen_system(p,T);g=A.T@r;H=A.T@A;s=np.linalg.svd(A,compute_uv=False)
        if s[-1]<=settings['rank_ratio']*s[0]:status='RANK_DEFICIENT';break
        inverse=np.linalg.inv(H);decrement=float(np.sqrt(max(g@inverse@g,0)));history.append({'iteration':iteration,'frozen_weight_cost':float(r@r),'score_norm':decrement})
        if decrement<=settings['score_tolerance']:status='CONVERGED_FROZEN_WEIGHT_SCORE';covariance=D@inverse@D;break
        if iteration==settings['max_iterations']:break
        direction=-inverse@g;accepted=False;cost=float(r@r)
        # Each step freezes its current weights as in IRLS. No derivative or determinant of C is claimed.
        for j in range(settings['line_search_steps']):
            step=2.**(-j);candidate=exp_se3(step*(D@direction))@T
            rr=p['X1']@candidate[:3,:3].T+candidate[:3,3]-p['X0'];new=np.linalg.solve(L,rr[...,None])[...,0].reshape(-1)
            if float(new@new)<=cost+settings['armijo']*step*2*float(g@direction):T=candidate;accepted=True;break
        if not accepted:status='LINE_SEARCH_FAILED';break
    return {'status':status,'successful':covariance is not None,'target_to_source':T.tolist(),'information_covariance':None if covariance is None else covariance.tolist(),'score_norm':None if not history else history[-1]['score_norm'],'history':history}

def evaluate_dense(flow,cov,row,baseline_ids,selector,covariance,seed,settings,folder):
    regenerated=pick(flow,cov,row,selector,seed);arms={}
    for name,ids in [('fixed_ids',baseline_ids),('regenerated',regenerated)]:
        try:
            p=packet(flow,cov,row,ids,covariance);path=folder/(name+'.npz');np.savez_compressed(path,**p);result=fit(p,settings)
            result.update({'proposed_source_ids':np.asarray(ids).tolist(),'accepted_source_ids':p['source_ids'].tolist(),'points':len(p['source_ids']),'packet_sha256':sha(path),'packet_path':str(path)})
        except (ValueError,np.linalg.LinAlgError) as e:result={'status':'INVALID_PACKET','successful':False,'reason':str(e),'proposed_source_ids':np.asarray(ids).tolist()}
        arms[name]=result
    return arms

def verify(plan):
    for p,h in plan['pins'].items():assert sha(p)==h,p

def capture(plan,out):
    import torch,cv2
    deterministic();model,loading=load_model(plan['checkpoint']);assert loading['migration']=='none';selector,covariance=native_algorithms();prepare=recipe();rows=[]
    state=lambda:{k:hashlib.sha256(v.cpu().numpy().tobytes()).hexdigest() for k,v in model.state_dict().items()}
    initial=state();dense=out/'dense';dense.mkdir();packets=out/'packets';packets.mkdir()
    def inference(images):
        a=np.stack([images[0,0],images[1,0],images[0,0]]);b=np.stack([images[0,1],images[1,1],images[1,0]])
        a=torch.from_numpy(np.repeat(a[:,None],3,axis=1)).cuda();b=torch.from_numpy(np.repeat(b[:,None],3,axis=1)).cuda()
        with torch.inference_mode():f,v=model.inference(a,b)
        assert torch.isfinite(f).all() and torch.isfinite(v).all() and (v>0).all()
        return f.cpu().numpy(),v.cpu().numpy()
    for condition in ['day','night','rain']:
        case=[r for r in plan['rows'] if r['condition']==condition];assert len(case)==2
        images=np.stack([[prepare(cv2.imread(r[s]['path'],cv2.IMREAD_UNCHANGED)) for s in ['left','right']] for r in case]).astype(np.float32)/255
        basef,basev=inference(images);baseids=pick(basef,basev,case[0],selector,plan['selection_seed']);canonical=out/(condition+'-prepared.npz');np.savez_compressed(canonical,images=images)
        for trial in [-2,-1]+list(range(plan['trials'])):
            if trial==-2:f,v=basef,basev;noise=np.zeros_like(images);seed=None
            elif trial==-1:f,v=inference(images);noise=np.zeros_like(images);seed=None
            else:
                seed=int.from_bytes(hashlib.sha256(f"{plan['noise_seed']}:{condition}:{trial}".encode()).digest()[:8],'little');rng=np.random.default_rng(seed);noise=rng.uniform(-plan['half_lsb'],plan['half_lsb'],images.shape).astype(np.float32);f,v=inference(np.clip(images+noise,0,1))
            path=dense/f'{condition}-{trial+2:03}.npz';np.savez_compressed(path,flow=f,covariance=v)
            folder=packets/f'{condition}-{trial+2:03}';folder.mkdir();arms=evaluate_dense(f,v,case[0],baseids,selector,covariance,plan['selection_seed'],plan['solver'],folder)
            entry={'condition':condition,'trial':trial,'noise_seed':seed,'noise_sha256':hashlib.sha256(noise.tobytes()).hexdigest(),'dense_path':str(path),'dense_sha256':sha(path),'arms':arms};rows.append(entry)
            with (out/'journal.jsonl').open('a') as log:log.write(json.dumps(entry,allow_nan=False)+'\n')
            print(condition,trial,{name:r['status'] for name,r in arms.items()},flush=True)
            if trial==-1:
                assert np.array_equal(f,basef) and np.array_equal(v,basev)
                original=rows[-2]['arms'];assert all(arms[name]['target_to_source']==original[name]['target_to_source'] and arms[name].get('information_covariance')==original[name].get('information_covariance') for name in arms)
    assert initial==state();write(out/'predictions.json',{'created_utc':now(),'rows':rows,'model_state_unchanged':True,'new_reference_values_read':False})
    write(out/'prediction-lock.json',{'created_utc':now(),'files':{str(p.relative_to(out)):sha(p) for p in out.rglob('*') if p.is_file()},'new_reference_values_read':False,'rows':len(rows),'fit_attempts':len(rows)*2})

def transport_cov(T,T0,P):
    relative=T@np.linalg.inv(T0);xi=log_se3(relative);eps=1e-6;B=np.column_stack([(log_se3(exp_se3(np.eye(6)[i]*eps)@relative)-log_se3(exp_se3(-np.eye(6)[i]*eps)@relative))/(2*eps) for i in range(6)])
    return xi,B@P@B.T

def score(plan,out):
    lock=read(out/'prediction-lock.json')
    for name,h in lock['files'].items():assert sha(out/name)==h,name
    records=read(out/'predictions.json')['rows'];report={};saved={}
    for c in ['day','night','rain']:
        selected=[r for r in records if r['condition']==c];base=next(r for r in selected if r['trial']==-2);trials=[r for r in selected if r['trial']>=0];matched=[r for r in trials if all(r['arms'][a]['successful'] for a in ['fixed_ids','regenerated'])];arms={}
        for arm in ['fixed_ids','regenerated']:
            baseline=base['arms'][arm]
            if not baseline['successful']:arms[arm]={'status':'BASELINE_UNAVAILABLE','successful_trials':sum(r['arms'][arm]['successful'] for r in trials)};continue
            rows=[r for r in trials if r['arms'][arm]['successful']];T0=np.array(baseline['target_to_source']);L=np.linalg.cholesky(np.array(baseline['information_covariance']))
            def summarize(rows,label):
                X=[];P=[]
                for r in rows:
                    fit=r['arms'][arm];x,p=transport_cov(np.array(fit['target_to_source']),T0,np.array(fit['information_covariance']));X.append(x);P.append(p)
                if len(X)<2:return {'valid_trials':len(X),'metrics':None}
                X=np.array(X);P=np.array(P);C=np.cov(X,rowvar=False,ddof=1);M=np.mean(P,axis=0);mean=X.mean(0);W=np.linalg.solve(L,X.T).T;CW=np.cov(W,rowvar=False,ddof=1);MW=np.linalg.solve(L,M);MW=np.linalg.solve(L,MW.T).T
                empirical_over_readout=np.linalg.eigvalsh(np.linalg.solve(np.linalg.cholesky(MW),CW)@np.linalg.inv(np.linalg.cholesky(MW).T))
                q=np.array([x@np.linalg.solve(p,x) for x,p in zip(X,P)]);saved[c+'_'+arm+'_'+label+'_xi']=X;saved[c+'_'+arm+'_'+label+'_readout']=P
                return {'valid_trials':len(X),'mean_baseline_relative_shift':mean.tolist(),'centered_empirical_covariance':C.tolist(),'mean_transported_information_covariance':M.tolist(),'baseline_relative_second_moment':(X.T@X/len(X)).tolist(),'readout_whitened_empirical_eigenvalues':empirical_over_readout.tolist(),'baseline_information_whitened_variance_trace':float(np.trace(CW)),'mean_translation_shift_norm_m':float(np.linalg.norm(X[:,:3],axis=1).mean()),'mean_rotation_shift_norm_deg':float(np.degrees(np.linalg.norm(X[:,3:],axis=1)).mean()),'baseline_relative_95pct_containment':float(np.mean(q<=chi2.ppf(.95,6))),'mean_baseline_relative_quadratic':float(q.mean())}
            arms[arm]={'own':summarize(rows,'own'),'matched':summarize(matched,'matched'),'statuses':{status:sum(r['arms'][arm]['status']==status for r in trials) for status in sorted({r['arms'][arm]['status'] for r in trials})}}
        changed=sum({tuple(x) for x in r['arms']['fixed_ids'].get('proposed_source_ids',[])}!={tuple(x) for x in r['arms']['regenerated'].get('proposed_source_ids',[])} for r in trials)
        baseids={tuple(x) for x in base['arms']['fixed_ids'].get('proposed_source_ids',[])}
        overlaps=[len(baseids&{tuple(x) for x in r['arms']['regenerated'].get('proposed_source_ids',[])})/len(baseids) if baseids else None for r in trials]
        report[c]={'attempted_trials':len(trials),'paired_successful_trials':len(matched),'selector_changed_trials':changed,'mean_original_source_id_retention':float(np.mean(overlaps)) if baseids else None,'arms':arms}
        if all('matched' in arms[a] and arms[a]['matched'].get('valid_trials',0)>1 for a in arms):
            left=arms['fixed_ids']['matched']['baseline_information_whitened_variance_trace'];right=arms['regenerated']['matched']['baseline_information_whitened_variance_trace'];report[c]['matched_regenerated_over_fixed_variance_trace']=right/left if left>0 else None
    np.savez_compressed(out/'score-arrays.npz',**saved)
    write(out/'scores.json',{'created_utc':now(),'recordings':report,'prediction_lock_sha256':sha(out/'prediction-lock.json'),'score_arrays_sha256':sha(out/'score-arrays.npz'),'decision':'DESCRIPTIVE_PAIRED_PERTURBATION_ASSESSMENT_COMPLETE_NO_MODEL_PROMOTION','scope':'Post-preparation half-LSB uniform input-jitter ensemble; static-seed source-ID control. Covariance containment is baseline-relative robustness, not physical calibration or true pose coverage. Native thermalfrontend/selector/covariance with explicit IRLS registration harness, not complete MACI2 VI reproduction.'})
    print(json.dumps({c:{k:v for k,v in r.items() if k!='arms'} for c,r in report.items()}))

def main():
    ap=argparse.ArgumentParser();ap.add_argument('stage',choices=['capture','score']);ap.add_argument('--protocol',required=True);ap.add_argument('--output',required=True);a=ap.parse_args();plan=read(a.protocol);verify(plan);out=Path(a.output)
    if a.stage=='capture':out.mkdir(parents=True,exist_ok=False);capture(plan,out)
    else:score(plan,out)
    verify(plan)
if __name__=='__main__':main()
