"""Independent shared-input/native-selection, point covariance and covariance-statistics audit."""
import argparse,json,hashlib
from pathlib import Path
from datetime import datetime,timezone
import numpy as np
from scipy.ndimage import minimum_filter
from scipy.spatial.transform import Rotation
from scipy.linalg import expm
from scipy.stats import chi2


def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def hat(v):
    x,y,z=v;return np.array([[0,-z,y],[z,0,-x],[-y,x,0.]])
def logarithm(T):
    w=Rotation.from_matrix(T[:3,:3]).as_rotvec();theta=np.linalg.norm(w);K=hat(w)
    coef=1/12+theta**2/720+theta**4/30240 if theta<1e-3 else (1-.5*theta/np.tan(theta/2))/theta**2
    return np.r_[(np.eye(3)-.5*K+coef*K@K)@T[:3,3],w]
def transported(T,T0,P):
    x=logarithm(T@np.linalg.inv(T0));ad=np.zeros((6,6));ad[:3,:3]=hat(x[3:]);ad[:3,3:]=hat(x[:3]);ad[3:,3:]=hat(x[3:]);aug=np.zeros((12,12));aug[:6,:6]=ad;aug[:6,6:]=np.eye(6);J=expm(aug)[:6,6:];inverse=np.linalg.inv(J)
    return x,inverse@P@inverse.T

def independent_selection(cov,seed):
    import torch
    quality=(cov[2,0]+cov[2,1]).astype(np.float32);nms=quality==minimum_filter(quality,size=7,mode='constant',cval=np.inf)
    values=quality[nms];median=np.partition(values,(len(values)-1)//2)[(len(values)-1)//2];threshold=np.float32(min(100.,float(median)*1.5));mask=nms&(quality<threshold);mask[:32]=False;mask[-32:]=False;mask[:,:32]=False;mask[:,-32:]=False
    y,x=np.nonzero(mask);candidate=np.column_stack([x,y]);g=torch.Generator().manual_seed(seed);idx=torch.randperm(len(candidate),generator=g)[:200].numpy()
    return candidate[idx]

def point_covariance(depth,uv,vu,vv,K):
    uv=np.asarray(uv);iu=uv[:,0].astype(int);iv=uv[:,1].astype(int);axis=np.arange(-15,16);xx,yy=np.meshgrid(axis,axis,indexing='xy')
    patches=depth[iv[:,None,None]+yy,iu[:,None,None]+xx]
    # Preserve native kernel/patch axis convention, including its transpose.
    kernel=np.exp(np.float32(-.5)*(yy[None].astype(np.float32)**2/vu[:,None,None]+xx[None].astype(np.float32)**2/vv[:,None,None]));kernel=kernel/kernel.sum((1,2),keepdims=True)
    mean=(kernel*patches).sum((1,2));variance=(kernel*(patches-mean[:,None,None])**2).sum((1,2));variance=np.maximum(variance,np.float32(.05))
    u=uv[:,0].astype(float);v=uv[:,1].astype(float);d=mean.astype(float);var=variance.astype(float);a=vu.astype(float);b=vv.astype(float);fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
    C=np.empty((len(uv),3,3));C[:,0,0]=((u-cx)**2*var+(d*d+var)*a)/fx**2;C[:,1,1]=((v-cy)**2*var+(d*d+var)*b)/fy**2;C[:,2,2]=var
    C[:,0,1]=C[:,1,0]=(u-cx)*(v-cy)*var/(fx*fy);C[:,0,2]=C[:,2,0]=var*(u-cx)/fx;C[:,1,2]=C[:,2,1]=var*(v-cy)/fy
    return C

def audit(plan_path,out):
    plan=read(plan_path);lock=read(out/'prediction-lock.json');pred=read(out/'predictions.json');scores=read(out/'scores.json');checks=0;maximum=0.;fits=0;invalid=0;nulls=0
    def check(a,msg):
        nonlocal checks
        assert a,msg;checks+=1
    def close(a,b,rtol=3e-5,atol=1e-8):
        nonlocal checks,maximum
        np.testing.assert_allclose(a,b,rtol=rtol,atol=atol);checks+=1
        if np.asarray(a).size:maximum=max(maximum,float(np.max(np.abs(np.asarray(a)-np.asarray(b)))))
    for p,h in plan['pins'].items():check(sha(p)==h,p)
    for rel,h in lock['files'].items():check(sha(out/rel)==h,rel)
    check(lock['rows']==390 and lock['fit_attempts']==780,'all planned attempts')
    check(lock['new_reference_values_read'] is pred['new_reference_values_read'] is False,'no reference values')
    check(pred['model_state_unchanged'],'model unchanged');check(scores['prediction_lock_sha256']==sha(out/'prediction-lock.json'),'score lock');check(scores['score_arrays_sha256']==sha(out/'score-arrays.npz'),'score arrays')
    primitives={};rows=pred['rows'];base_dense={}
    for c in ['day','night','rain']:
        with np.load(out/(c+'-prepared.npz')) as z:images=z['images']
        check(images.shape==(2,2,256,640) and images.dtype==np.float32 and np.min(images)>=0 and np.max(images)<=1,'image contract')
    for row in rows:
        c=row['condition'];trial=row['trial'];entry=next(x for x in plan['rows'] if x['condition']==c);K=np.array(entry['K']);fb=np.float32(K[0,0]*entry['baseline_m'])
        check(sha(row['dense_path'])==row['dense_sha256'],'dense hash')
        with np.load(row['dense_path']) as z:f=z['flow'];cov=z['covariance']
        check(f.shape==cov.shape==(3,2,256,640) and np.isfinite(f).all() and np.isfinite(cov).all() and np.all(cov>0),'dense schema/finite')
        if trial==-2:base_dense[c]=(f,cov)
        if trial==-1:
            check(np.array_equal(f,base_dense[c][0]) and np.array_equal(cov,base_dense[c][1]),'null dense repeat');nulls+=1
        if trial<0:noise=np.zeros((2,2,256,640),np.float32);check(row['noise_seed'] is None,'null seed')
        else:
            seed=int.from_bytes(hashlib.sha256(f"{plan['noise_seed']}:{c}:{trial}".encode()).digest()[:8],'little');check(seed==row['noise_seed'],'input seed');noise=np.random.default_rng(seed).uniform(-plan['half_lsb'],plan['half_lsb'],(2,2,256,640)).astype(np.float32)
        check(hashlib.sha256(noise.tobytes()).hexdigest()==row['noise_sha256'],'shared input draw')
        expected=independent_selection(cov,plan['selection_seed']);check(row['arms']['regenerated']['proposed_source_ids']==expected.tolist(),'native NMS/median/seed selection')
        baseline=next(x for x in rows if x['condition']==c and x['trial']==-2)
        check(row['arms']['fixed_ids']['proposed_source_ids']==baseline['arms']['fixed_ids']['proposed_source_ids'],'fixed queries')
        depth=[fb/np.abs(f[i,0]) for i in [0,1]]
        for arm,result in row['arms'].items():
            if result['status']=='INVALID_PACKET':invalid+=1;continue
            check(sha(result['packet_path'])==result['packet_sha256'],'packet hash')
            source=np.array(result['proposed_source_ids'],int);target=source.astype(np.float32)+f[2,:,source[:,1],source[:,0]]
            good=(target[:,0]>32)&(target[:,0]<608)&(target[:,1]>32)&(target[:,1]<224);u0=source[good];u1=target[good];a,b=u0[:,0],u0[:,1];x,y=u1[:,0].astype(int),u1[:,1].astype(int)
            C0=point_covariance(depth[0],u0,np.full(len(u0),.25,np.float32),np.full(len(u0),.25,np.float32),K)
            C1=point_covariance(depth[1],u1,np.maximum(cov[2,0,b,a],.0625),np.maximum(cov[2,1,b,a],.0625),K)
            d0=depth[0][b,a].astype(float);d1=depth[1][y,x].astype(float)
            back=lambda uv,d:np.column_stack([(uv[:,0]-K[0,2])*d/K[0,0],(uv[:,1]-K[1,2])*d/K[1,1],d])
            X0=back(u0.astype(float),d0);X1=back(u1.astype(float),d1)
            with np.load(result['packet_path']) as packet:
                check(np.array_equal(packet['source_ids'],u0),'range/sanity IDs');close(packet['target_uv'],u1,atol=0);close(packet['X0'],X0);close(packet['X1'],X1);close(packet['C0'],C0,rtol=4e-4,atol=2e-7);close(packet['C1'],C1,rtol=4e-4,atol=2e-7)
                # For solver audit use exact serialized native covariance, independently checked above.
                S0=packet['C0'];S1=packet['C1'];length=float(packet['length'])
                # Input backprojection was checked above; stationarity must use the exact serialized
                # float32-derived means, not NumPy division's slightly different reconstruction.
                X0=packet['X0'].copy();X1=packet['X1'].copy()
            T=np.array(result['target_to_source']);R=T[:3,:3];t=T[:3,3];close(R.T@R,np.eye(3),atol=1e-9);close(np.linalg.det(R),1,atol=1e-9)
            q=X1@R.T+t;error=q-X0;C=S0+R@S1@R.T;W=np.linalg.inv(C);H=np.zeros((6,6));g=np.zeros(6)
            for point,e,w in zip(q,error,W,strict=True):
                J=np.concatenate([np.eye(3),-hat(point)],axis=1);H+=J.T@w@J;g+=J.T@w@e
            covp=np.linalg.inv(H);score=float(np.sqrt(max(g@covp@g,0)))
            close(result['score_norm'],score,rtol=2e-3,atol=1e-7)
            check(result['successful']==(result['status']=='CONVERGED_FROZEN_WEIGHT_SCORE'),'fit eligibility')
            if result['successful']:
                check(score<=plan['solver']['score_tolerance']*1.01+1e-8,'stationarity');close(result['information_covariance'],covp,rtol=2e-5,atol=1e-8)
            else:check(result['information_covariance'] is None,'no failed covariance')
            fits+=1;primitives[c,trial,arm]=(T,None if result['information_covariance'] is None else np.array(result['information_covariance']))
        if trial<0:
            check(all(row['arms']['fixed_ids'][k]==row['arms']['regenerated'][k] for k in ['target_to_source','information_covariance','status']),'null arm equality')
            if trial==-1:check(all(row['arms'][arm][k]==baseline['arms'][arm][k] for arm in row['arms'] for k in ['target_to_source','information_covariance','status']),'null fit repeat')
    # Independent analytic SE3 logs and matrix-exponential Log Jacobian transport.
    with np.load(out/'score-arrays.npz') as arrays:
      for c,record in scores['recordings'].items():
        cr=[x for x in rows if x['condition']==c and x['trial']>=0];paired=[x for x in cr if all(x['arms'][a]['successful'] for a in ['fixed_ids','regenerated'])]
        check(record['attempted_trials']==len(cr)==128 and record['paired_successful_trials']==len(paired),'trial coverage')
        base=next(x for x in rows if x['condition']==c and x['trial']==-2);bs={tuple(x) for x in base['arms']['fixed_ids']['proposed_source_ids']};changed=sum({tuple(x) for x in x['arms']['regenerated']['proposed_source_ids']}!=bs for x in cr);check(changed==record['selector_changed_trials'],'changed source sets')
        close(record['mean_original_source_id_retention'],np.mean([len(bs&{tuple(v) for v in x['arms']['regenerated']['proposed_source_ids']})/len(bs) for x in cr]))
        for arm in ['fixed_ids','regenerated']:
          T0,P0=primitives[c,-2,arm];L=np.linalg.cholesky(P0)
          for label,selected in [('own',[x for x in cr if x['arms'][arm]['successful']]),('matched',paired)]:
            summary=record['arms'][arm][label];check(summary['valid_trials']==len(selected),'summary denominator')
            if len(selected)<2:continue
            values=[transported(*primitives[c,x['trial'],arm][:1],T0,primitives[c,x['trial'],arm][1]) for x in selected]
            X=np.array([x for x,p in values]);P=np.array([p for x,p in values]);close(arrays[c+'_'+arm+'_'+label+'_xi'],X,atol=1e-9);close(arrays[c+'_'+arm+'_'+label+'_readout'],P,rtol=2e-5,atol=1e-8)
            centered=X-X.mean(0);C=centered.T@centered/(len(X)-1);M=P.mean(0);close(summary['mean_baseline_relative_shift'],X.mean(0));close(summary['centered_empirical_covariance'],C,atol=1e-10);close(summary['mean_transported_information_covariance'],M);close(summary['baseline_relative_second_moment'],X.T@X/len(X),atol=1e-10)
            close(summary['mean_translation_shift_norm_m'],np.linalg.norm(X[:,:3],axis=1).mean());close(summary['mean_rotation_shift_norm_deg'],np.rad2deg(np.linalg.norm(X[:,3:],axis=1)).mean())
            whitened=np.linalg.solve(L,X.T).T;trace=float(np.trace(np.cov(whitened,rowvar=False,ddof=1)));close(summary['baseline_information_whitened_variance_trace'],trace)
            LP=np.linalg.cholesky(M);ratio=np.linalg.solve(LP,C);ratio=np.linalg.solve(LP,ratio.T).T;close(summary['readout_whitened_empirical_eigenvalues'],np.linalg.eigvalsh(ratio),atol=1e-8)
            quadratic=np.array([x@np.linalg.solve(p,x) for x,p in zip(X,P,strict=True)]);close(summary['mean_baseline_relative_quadratic'],quadratic.mean());close(summary['baseline_relative_95pct_containment'],np.mean(quadratic<=chi2.ppf(.95,6)))
        if paired:
            ratio=record['arms']['regenerated']['matched']['baseline_information_whitened_variance_trace']/record['arms']['fixed_ids']['matched']['baseline_information_whitened_variance_trace'];close(record['matched_regenerated_over_fixed_variance_trace'],ratio)
    report={'created_utc':datetime.now(timezone.utc).isoformat(),'status':'PASS','checks':checks,'fit_receipts_verified':fits,'invalid_packet_receipts':invalid,'null_dense_repeats':nulls,'maximum_absolute_numeric_difference_all_arrays':maximum,'scores_sha256':sha(out/'scores.json'),'auditor_sha256':sha(__file__),'scope':'Independent selection/point-covariance/fit/readout/sensitivity-statistic audit; no model rerun, true-pose calibration or paper reproduction.'}
    with (out/'independent-audit.json').open('x') as f:json.dump(report,f,indent=2);f.write('\n')
    print(json.dumps(report))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--protocol',required=True);p.add_argument('--output',required=True);a=p.parse_args();audit(a.protocol,Path(a.output))
