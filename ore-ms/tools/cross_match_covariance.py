"""Empirical cross-match influence decomposition, not a covariance correction."""
import argparse
from pathlib import Path
import numpy as np
from qualify_native_frontend import read,write,sha,now
from pose_covariance_replay import log_se3,skew
ROOT=Path(__file__).resolve().parents[1]
def path(s):
    p=Path(s);return p if p.is_absolute() else ROOT.parent/p

def covariance(x):
    x=x-x.mean(0);return x.T@x/(len(x)-1)

def split(influences):
    full=covariance(influences.sum(1));center=influences-influences.mean(0)
    block=np.einsum('tni,tnj->ij',center,center)/(len(center)-1)
    return full,block,full-block

def maps(X0,X1,C0,C1,T):
    R=T[:3,:3];q=X1@R.T+T[:3,3];C=C0+R@C1@R.T
    J=np.concatenate([np.broadcast_to(np.eye(3),(len(q),3,3)),-skew(q)],axis=2)
    WJ=np.linalg.solve(C,J);H=np.einsum('nki,nkj->ij',J,WJ)
    G=-np.linalg.solve(H,np.transpose(WJ,(0,2,1)).transpose(1,0,2).reshape(6,-1)).reshape(6,len(q),3).transpose(1,0,2)
    return H,G

def qualify(output):
    checks=0
    def close(a,b):
        nonlocal checks
        np.testing.assert_allclose(a,b,rtol=1e-9,atol=1e-12);checks+=1
    s=np.array([-1.,-1,1,1]);t=np.array([-1.,1,-1,1]);e=np.eye(6)[0]
    for a,b,ratio in [(s,s,2),(s,-s,0),(s,t,1)]:
        z=np.stack([a[:,None]*e,b[:,None]*e],axis=1);full,block,cross=split(z);close(full[0,0],ratio*block[0,0]);close(full,block+cross)
    X=np.array([[0,0,3],[1,0,4],[0,1,5],[-1,-1,4]],float);C=np.repeat(np.eye(3)[None],4,axis=0);H,G=maps(X,X,C,C,np.eye(4))
    close(H@np.linalg.inv(H),np.eye(6));shift=np.array([.001,0,0]);delta=np.einsum('nij,nj->ni',G,np.tile(shift,(4,1))).sum(0);close(delta,np.r_[-shift,[0,0,0]])
    write(output,{'created_utc':now(),'status':'PASS','checks':checks,'scope':'Common/anticorrelated/orthogonal influence covariance identities and translation-sign/GN map; not empirical study results.'});print('PASS',checks,'mechanics checks')

def assess(p,out):
    for f,h in p['pins'].items():assert sha(f)==h,f
    out.mkdir(parents=True,exist_ok=False);pred=read(p['predictions']);rows=[]
    for scene in p['scenes']:
        records=sorted([r for r in pred['rows'] if r['condition']==scene],key=lambda r:r['trial']);base=records[0]['arms']['fixed_ids'];null=records[1]['arms']['fixed_ids'];T0=np.array(base['target_to_source'])
        with np.load(path(base['packet_path'])) as z:packet={k:z[k] for k in ['source_ids','X0','X1','C0','C1']}
        with np.load(path(null['packet_path'])) as z:
            for k,v in packet.items():assert np.array_equal(z[k],v),'baseline null'
        H,G=maps(packet['X0'],packet['X1'],packet['C0'],packet['C1'],T0)
        np.testing.assert_allclose(np.linalg.inv(H),base['information_covariance'],rtol=1e-7,atol=1e-10)
        residuals=[];actual=[];ids=packet['source_ids'];baseline=packet['X1']@T0[:3,:3].T+T0[:3,3]-packet['X0']
        trials=[r for r in records if r['trial']>=0];assert [r['trial'] for r in trials]==list(range(128))
        for row in trials:
            fit=row['arms']['fixed_ids'];assert fit['successful'];f=path(fit['packet_path']);assert sha(f)==fit['packet_sha256']
            with np.load(f) as z:
                assert np.array_equal(z['source_ids'],ids),'fixed IDs changed'
                residuals.append(z['X1']@T0[:3,:3].T+T0[:3,3]-z['X0']-baseline)
            actual.append(log_se3(np.array(fit['target_to_source'])@np.linalg.inv(T0)))
        residuals=np.array(residuals);actual=np.array(actual);influences=np.einsum('nij,tnj->tni',G,residuals);linear=influences.sum(1);Vfull,Vblock,Vcross=split(influences);Vactual=covariance(actual)
        centered_linear=linear-linear.mean(0);centered_actual=actual-actual.mean(0);remainder=centered_actual-centered_linear
        tr=lambda M:float(np.trace(H@M));vals={'full':tr(Vfull),'block':tr(Vblock),'cross':tr(Vcross),'actual':tr(Vactual)}
        numerator=float(np.mean(np.einsum('ti,ij,tj->t',remainder,H,remainder)));denominator=float(np.mean(np.einsum('ti,ij,tj->t',centered_actual,H,centered_actual)))
        result={'scene':scene,'trials':128,'points':len(ids),'normalized_trace':vals,'full_over_block_trace':vals['full']/vals['block'],'linear_over_actual_trace':vals['full']/vals['actual'],'relative_centered_linear_remainder_rms':float(np.sqrt(numerator/denominator)),'translation_trace_m2':{k:float(np.trace(M[:3,:3])) for k,M in [('full',Vfull),('block',Vblock),('cross',Vcross),('actual',Vactual)]},'rotation_trace_rad2':{k:float(np.trace(M[3:,3:])) for k,M in [('full',Vfull),('block',Vblock),('cross',Vcross),('actual',Vactual)]}}
        np.savez_compressed(out/(scene+'.npz'),source_ids=ids,residual_changes=residuals,actual_left_tangent=actual,linear_tangent=linear,per_point_influence=influences,H=H,G=G,V_full=Vfull,V_block=Vblock,V_cross=Vcross,V_actual=Vactual)
        rows.append(result);print(scene,'full/block',result['full_over_block_trace'],'linear/actual',result['linear_over_actual_trace'],'remainder',result['relative_centered_linear_remainder_rms'],flush=True)
    write(out/'results.json',{'created_utc':now(),'rows':rows,'scenes':6,'trials':768,'new_model_calls':0,'new_pose_fits':0,'protocol_sha256':sha(p['protocol_path']),'scope':'Conditional input-ensemble covariance attribution, not physical calibration, full estimator derivative or fitted/deployable covariance correction.'})
    write(out/'artifact-lock.json',{'files':{f.name:sha(f) for f in out.iterdir() if f.is_file()}})
    for f,h in p['pins'].items():assert sha(f)==h,f

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['qualify','assess']);ap.add_argument('--protocol');ap.add_argument('--output',required=True);a=ap.parse_args()
    if a.mode=='qualify':qualify(Path(a.output))
    else:assess(read(a.protocol),Path(a.output))
