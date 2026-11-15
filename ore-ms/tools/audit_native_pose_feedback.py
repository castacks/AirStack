"""Audit state continuity and path metrics directly from committed poses."""
from pathlib import Path
import hashlib
import json
import datetime
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
RUN=ROOT/'results/native-pose-feedback/run-001'
REC=ROOT/'results/longer-fixed-thermal/recovery'


def read(path):return json.loads(path.read_text())
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def pose(v):
    value=np.eye(4);value[:3,:3]=Rotation.from_quat(v[3:]).as_matrix();value[:3,3]=v[:3]
    return value


def audit():
    scores=read(RUN/'scores.json');lock=read(RUN/'prediction-lock.json');freeze=read(RUN/'freeze.json')
    assert scores['prediction_lock_sha256']==sha(RUN/'prediction-lock.json')
    assert lock['freeze_sha256']==sha(RUN/'freeze.json')
    assert freeze['source_sha256']==sha(ROOT/'tools/native_pose_feedback.py')
    assert freeze['native_extension_sha256']==sha(ROOT/'tools/native_thermal_constraints.py')
    checks=4;summary=[];basis=np.eye(4);basis[:3,:3]=[[0,1,0],[0,0,1],[1,0,0]]
    originals=read(ROOT/'results/longer-native-control/run-001/scores.json')['summary']
    for receipt in lock['rows']:
        c=receipt['condition'];p=RUN/f'predictions-{c}.json'
        assert sha(p)==receipt['prediction_sha256']
        for path,expected in freeze['input_pins'][c].items():assert sha(ROOT/path)==expected;checks+=1
        rows=read(p)['rows'];assert [r['pair'] for r in rows]==list(range(599))
        refs=[]
        for i in range(600):
            t=np.loadtxt(REC/'references'/c/f'{i:06}.txt').reshape(4,4)
            u,_,vt=np.linalg.svd(t[:3,:3]);assert np.linalg.det(u@vt)>0;t[:3,:3]=u@vt;refs.append(t)
        condition={'condition':c,'methods':{}}
        for method,result in scores['summary'][c].items():
            prior=[0.,0.,0.,0.,0.,0.,1.];states=[pose(prior)];missing=[];statuses={}
            for row in rows:
                x=row['methods'][method];assert x['parent_xyzw']==prior
                statuses[x['status']]=statuses.get(x['status'],0)+1
                parent=pose(x['parent_xyzw']);current=pose(x['committed_xyzw'])
                if x['transform'] is None:
                    assert x['parent_xyzw']==x['committed_xyzw'];missing.append(row['pair']);checks+=1
                else:
                    expected=basis@np.linalg.solve(parent,current)@basis.T
                    assert np.allclose(expected,x['transform'],rtol=0,atol=1e-10);checks+=1
                assert np.array_equal(np.asarray(x['committed_xyzw'],np.float32).astype(float),x['committed_xyzw'])
                prior=x['committed_xyzw'];states.append(current);checks+=2
            for start,end,reported in [(0,599,result['full_path'])]+[
                    (b['start'],b['start']+99,b) for b in result['blocks']]:
                gaps=[i for i in missing if start<=i<end]
                if gaps:
                    assert reported['status']=='INCOMPLETE_NO_GAP_FILL' and reported['missing_pairs']==gaps
                    checks+=1;continue
                # Direct committed-state endpoints; no accumulation of emitted deltas.
                predicted=[basis@np.linalg.solve(states[start],x)@basis.T for x in states[start:end+1]]
                reference=[np.linalg.solve(refs[start],x) for x in refs[start:end+1]]
                distances=[sum(float(z)**2 for z in a[:3,3]-b[:3,3]) for a,b in zip(predicted,reference)]
                angles=[float(Rotation.from_matrix(b[:3,:3].T@a[:3,:3]).magnitude())**2
                        for a,b in zip(predicted,reference)]
                position=float(np.sqrt(sum(distances)/len(distances)))
                orientation=float(np.rad2deg(np.sqrt(sum(angles)/len(angles))))
                assert abs(position-reported['position_rmse_m'])<1e-7,(c,method,start,position,reported)
                assert abs(orientation-reported['orientation_rmse_deg'])<1e-7
                checks+=2
            method_result={'available':599-len(missing),'missing':missing,'statuses':statuses,
                           'state_chain_verified':True,'direct_committed_pose_metrics_agree':True}
            if method=='rgb_feedback' and result['full_path']['status']=='COMPLETE':
                native=originals[c]['native_rgb']['full_path'];current=result['full_path']
                method_result['original_native_position_rmse_difference_m']=current['position_rmse_m']-native['position_rmse_m']
                method_result['original_native_orientation_rmse_difference_deg']=current['orientation_rmse_deg']-native['orientation_rmse_deg']
            condition['methods'][method]=method_result
        summary.append(condition)
    value={'at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'checks':checks,
           'summary':summary,'status':'PASS','source_sha256':sha(Path(__file__))}
    (RUN/'audit.json').write_text(json.dumps(value,indent=2)+'\n');print(json.dumps(value))


if __name__=='__main__':audit()
