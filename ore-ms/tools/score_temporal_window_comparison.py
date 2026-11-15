"""Score globally locked predictions; all reference payload reads occur here."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def stats(x):return {'count':len(x),'mean':float(np.mean(x)),'rmse':float(np.sqrt(np.mean(np.array(x)**2))),'p95':float(np.percentile(x,95)),'maximum':float(max(x))} if x else {'count':0,'mean':None,'rmse':None,'p95':None,'maximum':None}
def main(protocol,output):
    plan=read(protocol);out=Path(output);root=Path(plan['recovery_root']);lock=read(out/'prediction-lock.json');assert lock['reference_values_parsed'] is False
    for n,h in lock['files'].items():assert sha(out/n)==h
    for n,h in plan['reference_sha256'].items():assert sha(root/n)==h
    summary={}
    for c in ['day','night','rain']:
        poses=[]
        for n in plan['references'][c]:
            T=np.loadtxt(root/n).reshape(4,4);R=T[:3,:3];assert np.isfinite(T).all() and np.linalg.norm(R.T@R-np.eye(3))<1e-5
            u,_,v=np.linalg.svd(R);T[:3,:3]=u@np.diag([1,1,np.linalg.det(u@v)])@v;poses.append(T)
        rows=[]
        for window in read(out/'capture'/f'predictions-{c}.json')['rows']:
          for k in range(4):
            pair=window['start']+k;truth=np.linalg.solve(poses[pair],poses[pair+1]);errors={}
            for name,pred in window['methods'].items():
                if pred['status']=='UNAVAILABLE':errors[name]=None;continue
                D=np.array(pred['relative_target_to_source'][k]);errors[name]={'translation_m':float(np.linalg.norm(D[:3,3]-truth[:3,3])),'rotation_deg':float(np.degrees(Rotation.from_matrix(truth[:3,:3].T@D[:3,:3]).magnitude()))}
            rows.append({'pair':pair,'truth':truth.tolist(),'errors':errors})
        assert len(rows)==80;names=list(rows[0]['errors'])
        def summarize(sel,selected):return {name:{m:stats([r['errors'][name][m] for r in sel if r['errors'][name] is not None]) for m in ['translation_m','rotation_deg']} for name in selected}
        matched={};gates={};candidate='joint_conditioned_joint'
        for control in ['joint_conditioned_marginal','rgb_conditioned_joint','rgb_original','joint_original','joint_conditioned_scale4']:
            selected=[r for r in rows if r['errors'][candidate] is not None and r['errors'][control] is not None];s=summarize(selected,[candidate,control]);matched[control]={'count':len(selected),'metrics':s}
            gates[control]=bool(selected and all(s[candidate]['translation_m'][k]<=.95*s[control]['translation_m'][k] for k in ['mean','rmse']) and all(s[candidate]['rotation_deg'][k]<=s[control]['rotation_deg'][k] for k in ['mean','rmse']) and sum(r['errors'][candidate] is not None for r in rows)>=sum(r['errors'][control] is not None for r in rows))
        scores={'rows':rows,'own':summarize(rows,names),'matched':matched,'gates':gates,'primary_pass':all(gates[k] for k in plan['primary_controls'])}
        (out/f'scores-{c}.json').write_text(json.dumps(scores,indent=2)+'\n');summary[c]={k:v for k,v in scores.items() if k!='rows'}
    report={'recordings':summary,'decision':'ADVANCE_TO_SEPARATE_REPLICATION_REVIEW' if all(v['primary_pass'] for v in summary.values()) else 'CLOSE_THIS_CONDITIONED_TEMPORAL_CONFIGURATION_NO_RETUNING','prediction_lock_sha256':sha(out/'prediction-lock.json'),'protocol_sha256':sha(protocol)}
    (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({'decision':report['decision'],'gates':{c:v['gates'] for c,v in summary.items()}},indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--protocol',required=True);p.add_argument('--output',required=True);a=p.parse_args();main(a.protocol,a.output)
