"""Independent final-cost/geometry/scoring audit, no producer imports."""
import json,hashlib,argparse
from pathlib import Path
import numpy as np
from scipy.linalg import block_diag


def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main(protocol,output):
    plan=read(protocol);root=Path(plan['recovery_root']);out=Path(output);lock=read(out/'prediction-lock.json')
    for n,h in lock['files'].items():assert sha(out/n)==h,n
    report={'scope':'Independent saved-state projection/cost and score reconstruction; no optimizer or neural inference rerun.','attempts':0,'available':0,'final_costs':0,'historical_original_controls':0,'score_rows':0,'max_cost_error':0.,'max_old_pose_error':0.,'max_scale_pose_error':0.,'status_counts':{}}
    for c in ['day','night','rain']:
      windows=read(out/'capture'/f'predictions-{c}.json')['rows'];old={x['start']:x for x in read(root/f'results/windowed-geometry/run-001/predictions-{c}.json')['rows']};scores=read(out/f'scores-{c}.json');scorerows={x['pair']:x for x in scores['rows']}
      for window in windows:
        start=window['start']
        for mode in ['rgb','joint']:
          folder=root/'results/windowed-geometry/run-001/packets';stem=f'{c}-{start:03}-{mode}'
          with np.load(folder/(stem+'.npz')) as z:p={k:z[k] for k in z.files}
          ids=read(folder/(stem+'-ids.json'))['ids'];cache={}
          for cam in set(x[0] for x in ids):
            base=root/f'results/persistent-track-replay/run-001/{c}/tracks/{cam}'
            with np.load(base/'windows'/f'{start:03}.npz') as z:a={k:z[k] for k in z.files}
            ws=[]
            for t in range(4):
              with np.load(base/'witnesses'/f'{start+t:03}.npz') as z:ws.append({k:z[k] for k in ['active_track_ids','flow_spatial_jacobian','source_uv','flow_cov_corners']})
            cache[cam]=(a,ws)
          covariance={}
          for j,(cam,identity) in enumerate(ids):
            a,ws=cache[cam];i=int(np.flatnonzero(a['native_match_ids']==identity)[0]);n=int(a['valid'][i].sum());S=np.zeros((n,2,n,2))
            for t in range(1,n):
              w=ws[t-1];q=int(np.flatnonzero(w['active_track_ids']==i)[0]);u=w['source_uv'][q];dx,dy=u-np.floor(u);corn=w['flow_cov_corners'][q]
              v=(1-dx)*(1-dy)*corn[0]+dx*(1-dy)*corn[1]+(1-dx)*dy*corn[2]+dx*dy*corn[3]
              Q=np.array([[v[0],v[2]],[v[2],v[1]]]);F=np.eye(2)+w['flow_spatial_jacobian'][q]
              for s in range(t):S[t,:,s,:]=F@S[t-1,:,s,:];S[s,:,t,:]=S[t,:,s,:].T
              S[t,:,t,:]=F@S[t-1,:,t-1,:]@F.T+Q
            for arm in ['conditioned_marginal','conditioned_joint','conditioned_scale4']:
              C=np.zeros((3*n,3*n))
              for t in range(n):
                C[3*t+2,3*t+2]=a['disparity_cov'][i,t]
                for s in range(n):
                  if t==s or arm=='conditioned_joint':C[3*t:3*t+2,3*s:3*s+2]=S[t,:,s,:]
              if arm=='conditioned_scale4':C*=4
              covariance[j,arm]=C[2:,2:]
          for arm in ['original','conditioned_marginal','conditioned_joint','conditioned_scale4']:
            name=mode+'_'+arm;pred=window['methods'][name];report['attempts']+=1;status=pred['status'];report['status_counts'][status]=report['status_counts'].get(status,0)+1
            if arm=='original':
              previous=old[start]['methods'][mode+'_variable'];assert status==previous['status']
              if status!='UNAVAILABLE':
                delta=float(np.max(np.abs(np.array(pred['root_to_body'])-previous['root_to_body'])));assert delta<1e-8;report['max_old_pose_error']=max(report['max_old_pose_error'],delta)
              report['historical_original_controls']+=1
            if status=='UNAVAILABLE':continue
            report['available']+=1
            T=np.array(pred['root_to_body']);alpha=np.array(pred['log_inverse_depth']);assert np.array_equal(T[0],np.eye(4))
            for t in T:assert np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-8)
            rootpos=np.einsum('nji,nj->ni',p['source_R'],p['rays']*np.exp(-alpha)[:,None]-p['source_t'])
            j=p['track'];k=p['frame'];body=np.einsum('nij,nj->ni',T[k,:3,:3],rootpos[j])+T[k,:3,3];q=np.einsum('nij,nj->ni',p['camera_R'],body)+p['camera_t'];assert np.all(q[:,2]>0)
            fx,fy,cx,cy,b=p['Kb'].T;prediction=np.column_stack([fx*q[:,0]/q[:,2]+cx,fy*q[:,1]/q[:,2]+cy,fx*b/q[:,2]]);res=prediction-p['observations'];cost=0.
            for track in range(len(ids)):
              selected=np.flatnonzero(j==track);v=res[selected].reshape(-1)
              C=block_diag(*p['covariance'][selected]) if arm=='original' else covariance[track,arm]
              if arm!='original':v=v[2:]
              cost+=float(.5*v@np.linalg.solve(C,v))
            delta=abs(cost-pred['final_cost']);assert delta<=1e-7+1e-8*abs(cost),(c,start,name,cost,pred['final_cost'])
            report['max_cost_error']=max(report['max_cost_error'],delta);report['final_costs']+=1
            for t in range(4):
              D=T[t]@np.linalg.inv(T[t+1]);assert np.allclose(D,pred['relative_target_to_source'][t],atol=1e-10,rtol=1e-10)
              row=scorerows[start+t];truth=np.array(row['truth']);errors=row['errors'][name]
              trans=float(np.linalg.norm(D[:3,3]-truth[:3,3]));relative=truth[:3,:3].T@D[:3,:3]
              # atan2 trace/skew rotation angle, separate from scorer quaternion path.
              sine=np.linalg.norm([relative[2,1]-relative[1,2],relative[0,2]-relative[2,0],relative[1,0]-relative[0,1]])/2
              angle=float(np.degrees(np.arctan2(sine,(np.trace(relative)-1)/2)))
              assert abs(trans-errors['translation_m'])<1e-9 and abs(angle-errors['rotation_deg'])<1e-7
              report['score_rows']+=1
          m=window['methods'][mode+'_conditioned_marginal'];s=window['methods'][mode+'_conditioned_scale4']
          # Finite precision can alter a line-search boundary. Record, do not
          # silently substitute/retry a scale arm or use parity as accuracy gate.
          if m['status']!='UNAVAILABLE' and s['status']!='UNAVAILABLE':report['max_scale_pose_error']=max(report['max_scale_pose_error'],float(np.max(np.abs(np.array(m['root_to_body'])-s['root_to_body']))))
      candidate='joint_conditioned_joint'
      for control,pair in scores['matched'].items():
        rows=[r for r in scores['rows'] if r['errors'][candidate] is not None and r['errors'][control] is not None]
        assert len(rows)==pair['count']
        for name in [candidate,control]:
          for metric in ['translation_m','rotation_deg']:
            x=np.array([r['errors'][name][metric] for r in rows]);s=pair['metrics'][name][metric]
            if len(x):assert np.isclose(x.mean(),s['mean'],rtol=1e-12) and np.isclose(np.sqrt(np.mean(x*x)),s['rmse'],rtol=1e-12)
        def vals(n,m):return np.array([r['errors'][n][m] for r in rows])
        gate=bool(rows)
        if rows:
          xt,yt=vals(candidate,'translation_m'),vals(control,'translation_m');xr,yr=vals(candidate,'rotation_deg'),vals(control,'rotation_deg')
          gate=xt.mean()<=.95*yt.mean() and np.mean(xt*xt)<=.95**2*np.mean(yt*yt) and xr.mean()<=yr.mean() and np.mean(xr*xr)<=np.mean(yr*yr)
          gate=bool(gate and sum(r['errors'][candidate] is not None for r in scores['rows'])>=sum(r['errors'][control] is not None for r in scores['rows']))
        assert gate==scores['gates'][control]
    assert report['attempts']==480 and report['historical_original_controls']==120
    report['status']='PASS';(out/'independent-audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--protocol',required=True);p.add_argument('--output',required=True);a=p.parse_args();main(a.protocol,a.output)
