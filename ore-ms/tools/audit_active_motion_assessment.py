"""Independent image integration, native packet and matched-action pose checks."""
import argparse,hashlib,math
from pathlib import Path
import cv2
import numpy as np
from scipy.integrate import quad
from scipy.optimize import brentq
from qualify_native_frontend import read,write,sha,now,recipe
from audit_paired_frontend import independent_selection,point_covariance,hat

def image_audit(p,folder,out):
    checks=0
    def ck(x,msg):
        nonlocal checks
        assert x,msg
        checks+=1
    def close(a,b,atol=1e-10,rtol=1e-9):
        nonlocal checks
        np.testing.assert_allclose(a,b,atol=atol,rtol=rtol);checks+=1
    for path,h in p['pins'].items():ck(sha(path)==h,'source/input pin')
    for path,h in read(folder/'artifact-lock.json')['files'].items():ck(sha(folder/path)==h,'image lock')
    inv=read(folder/'inventory.json');ck(len(inv['records'])==9 and inv['views']==72,'9cases/72views');ck(inv['protocol_sha256']==sha(p['protocol_path']),'render protocol')
    h,w=p['shape'];K=np.array(p['K']);D,H,tau=p['distance_m'],p['horizon_s'],p['tau_s'];v,u=np.mgrid[:h,:w]
    expected_mask=(u>=1)&(u+K[0,0]*(D+p['baseline_m'])/p['plane_depth_m']<=w-2)&(v>=1)&(v<=h-2)
    references={};maximum=squares=0.;count=0;max_quad=0.;pixels=0
    grid=[(x,y) for x in [64,128,256,384,512,576] for y in [48,96,160,208]]
    for c in inv['records']:
        ck(sha(c['path'])==c['sha256'],'case lock')
        with np.load(c['path']) as z:
            images,values,coarse,texture=[z[k] for k in ['images','continuous','coarse','texture']];mask=z['common_pixel_support'];left=z['world_from_left'];right=z['world_from_right'];truth=z['target_to_source']
        ck(images.shape==values.shape==coarse.shape==(2,2,2,h,w),'image schema');ck(images.dtype==np.uint8,'one8bit preparation');ck(np.isfinite(values).all() and values.min()>=-1e-12 and values.max()<=1+1e-12,'bounded values')
        original=recipe()(cv2.imread(p['textures'][c['condition']],cv2.IMREAD_UNCHANGED)).astype(float)/255;ck(np.array_equal(texture,original),'texture identity')
        ck(np.array_equal(mask,expected_mask),'analytic complete-path support');close(mask.mean(),c['support_fraction'])
        ck(np.max(np.abs(images.astype(float)-values*255))<=.5000000001,'quantization')
        expected=np.repeat(np.eye(4)[None],2,axis=0);expected[1,0,3]=D;close(left,expected);expected_right=expected.copy();expected_right[:,0,3]+=p['baseline_m'];close(right,expected_right);close(truth,expected[1])
        if c['condition'] not in references:references[c['condition']]=(images.copy(),values.copy())
        else:
            base,continuous=references[c['condition']];ck(np.array_equal(images[0],base[0]),'sharp image null');ck(np.array_equal(values[0],continuous[0]),'sharp continuous null');ck(np.array_equal(images[1,0],base[1,0]),'memory source image null');ck(np.array_equal(values[1,0],continuous[1,0]),'memory source continuous null')
        diff=(values-coarse)[...,mask]*255;maximum=max(maximum,float(np.abs(diff).max()));squares+=float(np.sum(diff**2));count+=diff.size
        beta=c['beta']
        def position(t):
            z=max(0.,min(1.,t/H));return D*((10+beta)*z**3+(-15-3*beta)*z**4+(6+3*beta)*z**5-beta*z**6)
        for arm in range(2):
            for frame,t in enumerate([0.,H]):
                for side in [0,1]:
                    for x,y in grid:
                        offset=K[0,0]*side*p['baseline_m']/p['plane_depth_m']
                        def sample(age):
                            X=x+offset+K[0,0]*position(t-age)/p['plane_depth_m'];a=math.floor(X);fraction=X-a
                            if X<0 or X>w-1:return 0.
                            return float((1-fraction)*texture[y,a]+fraction*texture[y,min(a+1,w-1)])
                        if arm==0 or frame==0:expected=sample(0.)
                        else:
                            knots=[]
                            lo=x+offset;hi=lo+K[0,0]*D/p['plane_depth_m']
                            for cell in range(math.floor(lo)+1,math.ceil(hi)):
                                if lo<cell<hi:
                                    t_cross=brentq(lambda z:lo+K[0,0]*position(z)/p['plane_depth_m']-cell,0,H,xtol=1e-14);knots.append(H-t_cross)
                            expected,estimate=quad(lambda age:math.exp(-age/tau)/tau*sample(age),0,H,points=sorted(knots),epsabs=1e-11,epsrel=1e-11,limit=200)
                            expected+=math.exp(-H/tau)*sample(H)
                            ck(estimate*255<1e-5,'reference integration error estimate')
                        discrepancy=abs(expected-values[arm,frame,side,y,x])*255;max_quad=max(max_quad,discrepancy);ck(discrepancy<=.5,'independent pixel integral');pixels+=1
    close(maximum,inv['convergence']['max_code'],atol=1e-8);close(math.sqrt(squares/count),inv['convergence']['rms_code'],atol=1e-10);ck(count==inv['convergence']['supported_values'],'convergence population');ck(maximum<=p['max_code'] and math.sqrt(squares/count)<=p['rms_code'],'fixed grid bound')
    write(out,{'created_utc':now(),'status':'PASS','checks':checks,'sampled_pixel_integrals':pixels,'max_reference_discrepancy_code':max_quad,'supported_values':count,'max_64_128_code':maximum,'rms_64_128_code':math.sqrt(squares/count),'scope':'Independent bilinear interpolation/time integration knots/geometry/support/nulls; not physical sensor calibration.'});print('PASS image',checks,'checks',pixels,'integrals',max_quad,'codes',flush=True)

def reconstruct(f,c,K,B,ids,mask,minimum):
    ids=np.asarray(ids,int).reshape(-1,2);uv=ids.astype(np.float32)+f[2,:,ids[:,1],ids[:,0]];xy=np.clip(uv,[0,0],[639,255]).astype(int)
    keep=(uv[:,0]>32)&(uv[:,0]<608)&(uv[:,1]>32)&(uv[:,1]<224)&mask[ids[:,1],ids[:,0]]&mask[xy[:,1],xy[:,0]];u0,u1=ids[keep],uv[keep]
    if len(u0)<minimum:return None
    depths=[np.float32(K[0,0]*B)/np.abs(f[j,0]) for j in [0,1]];x,y=u0[:,0],u0[:,1]
    C0=point_covariance(depths[0],u0,np.full(len(u0),.25,np.float32),np.full(len(u0),.25,np.float32),K)
    C1=point_covariance(depths[1],u1,np.maximum(c[2,0,y,x],.0625),np.maximum(c[2,1,y,x],.0625),K);sane=np.isfinite(C0).all((1,2))&np.isfinite(C1).all((1,2))
    if sane.sum()<minimum:return None
    fields={'source_ids':u0[sane],'C0':C0[sane],'C1':C1[sane],'target_uv':u1[sane]}
    for j,pixels in enumerate([u0,u1]):
        d=depths[j][pixels[:,1].astype(int),pixels[:,0].astype(int)].astype(float);fields['X'+str(j)]=np.column_stack([(pixels[:,0].astype(float)-K[0,2])*d/K[0,0],(pixels[:,1].astype(float)-K[1,2])*d/K[1,1],d])[sane]
    return fields

def estimator_audit(p,folder,out):
    checks=0;successes=0
    def ck(x,msg):
        nonlocal checks
        assert x,msg
        checks+=1
    def close(a,b,rtol=1e-8,atol=1e-10):
        nonlocal checks
        np.testing.assert_allclose(a,b,rtol=rtol,atol=atol);checks+=1
    for path,h in p['pins'].items():ck(sha(path)==h,'source/input pin')
    for path,h in read(folder/'prediction-lock.json')['files'].items():ck(sha(folder/path)==h,'prediction lock')
    pred=read(folder/'predictions.json');scores=read(Path(p['score_output'])/'scores.json');cases=read(p['inventory'])['records'];K=np.array(p['K']);B=p['baseline_m']
    ck(len(pred['rows'])==len(scores['rows'])==36 and len(pred['dense_rows'])==18,'attempt denominator');ck(len({(r['case'],r['arm'],r['mode']) for r in pred['rows']})==36,'unique attempts');ck(pred['model_state_unchanged'] and not pred['true_pose_values_read'] and pred['loading']['migration']=='none','frozen strict frontend');ck(scores['prediction_lock_sha256']==sha(folder/'prediction-lock.json'),'score binding')
    for condition in ['day','night','rain']:
        group=[x for x in cases if x['condition']==condition];dense=[];proposed=[];medians=[];keys=[];mask=None
        for case in group:
            ck(sha(case['path'])==case['sha256'],'image lock')
            with np.load(case['path']) as z:images=z['images'].astype(np.float32)/255;raw=z['common_pixel_support']
            m=cv2.erode(raw.astype(np.uint8),np.ones((31,31),np.uint8),borderType=cv2.BORDER_CONSTANT,borderValue=0).astype(bool);m[:32]=False;m[-32:]=False;m[:,:32]=False;m[:,-32:]=False
            if mask is None:mask=m
            else:ck(np.array_equal(mask,m),'common group mask')
            for j,arm in enumerate(['sharp','memory']):
                meta=next(x for x in pred['dense_rows'] if x['case']==case['id'] and x['arm']==arm);im=images[j];src=np.stack([im[0,0],im[1,0],im[0,0]]);dst=np.stack([im[0,1],im[1,1],im[1,0]])
                ck(hashlib.sha256(src.tobytes()).hexdigest()==meta['source_input_sha256'] and hashlib.sha256(dst.tobytes()).hexdigest()==meta['target_input_sha256'],'input binding');ck(sha(meta['dense_path'])==meta['dense_sha256'],'dense hash')
                with np.load(meta['dense_path']) as z:f,c=z['flow'],z['covariance']
                ck(f.shape==c.shape==(3,2,256,640) and np.isfinite(f).all() and np.isfinite(c).all() and (c>0).all(),'dense schema')
                ids=independent_selection(c,p['selection_seed']);ck(ids.tolist()==meta['native_proposed_ids'],'native selector');median=float(np.median((c[2,0]+c[2,1])[mask]));close(median,meta['median_temporal_covariance_trace']);dense.append((f,c));proposed.append(ids);medians.append(median);keys.append((case['id'],arm))
        seed=reconstruct(*dense[0],K,B,proposed[0],mask,p['minimum_points']);common=np.empty((0,2),int) if seed is None else seed['source_ids']
        for f,c in dense:
            q=reconstruct(f,c,K,B,common,mask,p['minimum_points']);common=np.empty((0,2),int) if q is None else q['source_ids']
        for i,(caseid,arm) in enumerate(keys):
            for mode in ['regenerated','common_ids']:
                row=next(x for x in pred['rows'] if (x['case'],x['arm'],x['mode'])==(caseid,arm,mode));scored=next(x for x in scores['rows'] if (x['case'],x['arm'],x['mode'])==(caseid,arm,mode));result=row['fit'];ids=proposed[i] if mode=='regenerated' else common;ck(row['requested_ids']==ids.tolist(),'joint-six-arm query control');q=reconstruct(*dense[i],K,B,ids,mask,p['minimum_points']);ck(scored['successful']==result['successful'] and scored['status']==result['status'],'retained status');close(scored['median_temporal_covariance_trace'],medians[i])
                if q is None:ck(not result['successful'] and scored['metrics'] is None,'prefit abstention');continue
                ck(sha(result['packet_path'])==result['packet_sha256'],'packet hash')
                with np.load(result['packet_path']) as z:
                    ck(np.array_equal(z['source_ids'],q['source_ids']),'admitted IDs');ck(scored['points']==len(q['source_ids']) and result['accepted_ids']==q['source_ids'].tolist(),'counts')
                    for k in ['X0','X1','target_uv']:close(z[k],q[k],rtol=3e-5,atol=1e-8)
                    for k in ['C0','C1']:close(z[k],q[k],rtol=4e-4,atol=2e-7)
                    X0,X1,C0,C1=[z[k] for k in ['X0','X1','C0','C1']]
                if not result['successful']:ck(scored['metrics'] is None,'fit failure retained');continue
                successes+=1;T=np.array(result['target_to_source']);R=T[:3,:3];close(R.T@R,np.eye(3));v=X1@R.T+T[:3,3];H,g=np.zeros((6,6)),np.zeros(6)
                for point,residual,C in zip(v,v-X0,C0+R@C1@R.T):
                    J=np.concatenate([np.eye(3),-hat(point)],axis=1);H+=J.T@np.linalg.solve(C,J);g+=J.T@np.linalg.solve(C,residual)
                ck(np.sqrt(max(g@np.linalg.solve(H,g),0))<=p['solver']['score_tolerance']*1.01,'pose stationarity');P=np.linalg.inv(H);close(result['information_covariance'],P,rtol=1e-7)
                case=next(x for x in group if x['id']==caseid)
                with np.load(case['path']) as z:truth=np.linalg.solve(z['world_from_left'][0],z['world_from_left'][1]);close(truth,z['target_to_source'])
                m=scored['metrics'];close(m['translation_m'],np.linalg.norm(T[:3,3]-truth[:3,3]));A=truth[:3,:3].T@R;sine=.5*np.linalg.norm([A[2,1]-A[1,2],A[0,2]-A[2,0],A[1,0]-A[0,1]]);close(m['rotation_deg'],np.rad2deg(np.arctan2(sine,(np.trace(A)-1)/2)));close(m['translation_information_std_proxy_m'],np.sqrt(np.trace(P[:3,:3])));close(m['rotation_information_std_proxy_deg'],np.rad2deg(np.sqrt(np.trace(P[3:,3:]))))
        for i in [2,4]:ck(all(np.array_equal(x,y) for x,y in zip(dense[0],dense[i])),'sharp dense exactnull')
        for mode in ['regenerated','common_ids']:
            fits=[x['fit'] for x in pred['rows'] if x['case'].startswith(condition+'-') and x['arm']=='sharp' and x['mode']==mode]
            for q in fits[1:]:ck(q['successful']==fits[0]['successful'] and q.get('target_to_source')==fits[0].get('target_to_source') and q.get('information_covariance')==fits[0].get('information_covariance'),'sharp fitted exactnull')
    ck(scores['planned_dense_batches']==18 and scores['planned_pose_attempts']==36,'score denominator');write(out,{'created_utc':now(),'status':'PASS','checks':checks,'planned_dense_batches':18,'planned_pose_attempts':36,'successful_pose_fits':successes,'scope':'Independent shared support/native selector/packet/stationarity/errors/sharp nulls; no physical coverage or planner benefit.'});print('PASS estimator',checks,'checks',successes,'/36fits',flush=True)

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['images','estimator']);ap.add_argument('--protocol',required=True);ap.add_argument('--folder',required=True);ap.add_argument('--output',required=True);a=ap.parse_args();p=read(a.protocol)
    {'images':image_audit,'estimator':estimator_audit}[a.mode](p,Path(a.folder),Path(a.output))
