"""Reference-free saved-field audit; no native solver or producer imports."""
import argparse,hashlib,json
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import numpy as np
from scipy.linalg import block_diag


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def projection(u,depth,K,b,R,t):
    fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
    ray=np.array([(u[0]-cx)/fx,(u[1]-cy)/fy,1.]);q=R@(depth*ray)+t
    x,y,z=q;D=np.array([[fx/z,0,-fx*x/z**2],[0,fy/z,-fy*y/z**2],[0,0,-fx*b/z**2]])
    B=D@R@(depth*np.array([[1/fx,0],[0,1/fy],[0,0]]))
    return np.array([fx*x/z+cx,fy*y/z+cy,fx*b/z]),B


def main(root,protocol,output):
    plan=json.loads(Path(protocol).read_text());root=Path(root);out=Path(output)
    assert not out.exists();assert sha(__file__)==plan['source_sha256']
    for n,h in plan['inputs'].items():assert sha(root/n)==h,n
    counts={'camera_windows':0,'admitted_tracks':0,'target_uv_blocks':0,'root_jacobian_checks':0}
    extrema={'marginal_absolute_error':0.,'marginal_relative_error':0.,'conditional_min_eigenvalue':float('inf'),'residual_min_eigenvalue':float('inf'),'jacobian_absolute_error':0.,'joint_residual_reconstruction_error':0.}
    details=[];base=root/'results/persistent-track-replay/run-001'
    for condition in ['day','night','rain']:
      for camera in ['rgb','thermal']:
        trackdir=base/condition/'tracks'/camera
        with np.load(base/condition/'off'/camera/'tensor-map.npz',allow_pickle=False) as m:
            Ks=m['frames//K'].astype(float);bs=m['frames//baseline'].astype(float).reshape(-1)
        for start in range(0,100,5):
          with np.load(trackdir/'windows'/f'{start:03}.npz') as z:a={k:z[k] for k in z.files}
          witnesses=[]
          for step in range(4):
            with np.load(trackdir/'witnesses'/f'{start+step:03}.npz') as z:witnesses.append({k:z[k] for k in ['active_track_ids','accepted','flow_spatial_jacobian','flow_cov_corners','source_uv']})
          admitted=np.flatnonzero(a['valid'].sum(1)>=3);assert len(admitted)>=6
          first=True
          for track in admitted:
            valid=np.flatnonzero(a['valid'][track]);assert np.array_equal(valid,np.arange(len(valid)))
            length=len(valid);C0=a['uv_cov'][track,0];Qs=[];Fs=[]
            for step in range(length-1):
              w=witnesses[step];where=np.flatnonzero(w['active_track_ids']==track);assert len(where)==1;i=int(where[0]);assert w['accepted'][i]
              uv=w['source_uv'][i];dx,dy=uv-np.floor(uv)
              weights=np.array([(1-dx)*(1-dy),dx*(1-dy),(1-dx)*dy,dx*dy])
              packed=weights@w['flow_cov_corners'][i]
              Q=np.array([[packed[0],packed[2]],[packed[2],packed[1]]]);assert np.linalg.eigvalsh(Q).min()>0
              Qs.append(Q);Fs.append(np.eye(2)+w['flow_spatial_jacobian'][i])
            # Independent latent-noise mapping [seed,increment1,...].
            G=np.zeros((length*2,length*2));G[:2,:2]=np.eye(2)
            for t in range(1,length):
              G[2*t:2*t+2]=Fs[t-1]@G[2*t-2:2*t]
              G[2*t:2*t+2,2*t:2*t+2]=np.eye(2)
            primitive=block_diag(C0,*Qs);joint=G@primitive@G.T
            S=G[:,2:]@block_diag(*Qs)@G[:,2:].T
            for t in range(length):
              delta=np.abs(joint[2*t:2*t+2,2*t:2*t+2]-a['uv_cov'][track,t])
              err=float(delta.max());rel=err/max(float(np.abs(a['uv_cov'][track,t]).max()),1e-300)
              extrema['marginal_absolute_error']=max(extrema['marginal_absolute_error'],err);extrema['marginal_relative_error']=max(extrema['marginal_relative_error'],rel)
              assert np.allclose(joint[2*t:2*t+2,2*t:2*t+2],a['uv_cov'][track,t],atol=1e-9,rtol=1e-9)
            minimum=float(np.linalg.eigvalsh(S[2:,2:]).min());assert minimum>0
            extrema['conditional_min_eigenvalue']=min(extrema['conditional_min_eigenvalue'],minimum)
            counts['target_uv_blocks']+=length-1;counts['admitted_tracks']+=1
            if first:
              # Fixed synthetic camera-local motion: no saved/reference poses read.
              K=Ks[start];b=bs[start];u=a['uv'][track,0];depth=K[0,0]*b/a['disparity'][track,0]
              B=[]
              for t in range(length):
                angle=.01*t;R=np.array([[np.cos(angle),0,np.sin(angle)],[0,1,0],[-np.sin(angle),0,np.cos(angle)]])
                trans=np.array([.02*t,-.01*t,.005*t])
                _,analytic=projection(u,depth,K,b,R,trans);finite=np.zeros((3,2));h=1e-4
                for col in range(2):
                  v=np.eye(2)[col]*h
                  finite[:,col]=(projection(u+v,depth,K,b,R,trans)[0]-projection(u-v,depth,K,b,R,trans)[0])/(2*h)
                err=float(np.abs(finite-analytic).max());assert err<1e-7
                extrema['jacobian_absolute_error']=max(extrema['jacobian_absolute_error'],err);B.append(analytic);counts['root_jacobian_checks']+=1
              L=np.vstack([np.eye(2),np.zeros((1,2))]);embed=block_diag(*([L]*length));stack=np.vstack(B)
              A=stack-embed@G[:,:2]
              stereo=np.diag(np.repeat(a['disparity_cov'][track,:length],3)*np.tile([0.,0.,1.],length))
              residual=A@C0@A.T+embed@S@embed.T+stereo
              # Reconstruct from a single linear map of primitive noises.
              M=-embed@G;M[:,:2]+=stack
              direct=M@primitive@M.T+stereo
              err=float(np.abs(residual-direct).max());assert np.allclose(residual,direct,atol=1e-8,rtol=1e-9)
              assert np.max(np.abs(residual[:2]))<1e-9
              keep=np.arange(2,length*3);minimum=float(np.linalg.eigvalsh(residual[np.ix_(keep,keep)]).min());assert minimum>0
              extrema['residual_min_eigenvalue']=min(extrema['residual_min_eigenvalue'],minimum);extrema['joint_residual_reconstruction_error']=max(extrema['joint_residual_reconstruction_error'],err)
              first=False
          counts['camera_windows']+=1;details.append({'condition':condition,'camera':camera,'start':start,'admitted_tracks':len(admitted)})
    for n,h in plan['inputs'].items():assert sha(root/n)==h,n
    assert counts['camera_windows']==120
    out.mkdir(parents=True)
    result={'decision':'FIELDS_SUFFICIENT_FOR_DECLARED_INDEPENDENT_INCREMENT_VARIABLE_DEPTH_MODEL','run_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'counts':counts,'extrema':extrema,'windows':details,'source_sha256':sha(__file__),'protocol_sha256':sha(protocol),'reference_values_read':False,'scope':'Saved marginals, conditional innovation SPD and synthetic-state residual/root Jacobians. No real solver, empirical correlation or statistical calibration qualification.'}
    (out/'report.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='windows'},indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',required=True);p.add_argument('--protocol',required=True);p.add_argument('--output',required=True);a=p.parse_args();main(a.root,a.protocol,a.output)
