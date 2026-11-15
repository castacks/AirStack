"""Verify exact rotation preservation and score from committed state endpoints."""
from pathlib import Path
import json
import hashlib
import datetime
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
RUN=ROOT/'results/component-constrained-native/run-001'
BASE=ROOT/'results/native-pose-feedback/run-001'
REC=ROOT/'results/longer-fixed-thermal/recovery'


def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def pose(v):
    t=np.eye(4);t[:3,:3]=Rotation.from_quat(v[3:]).as_matrix();t[:3,3]=v[:3];return t


def audit():
    lock=read(RUN/'prediction-lock.json');freeze=read(RUN/'freeze.json');scores=read(RUN/'scores.json')
    assert lock['freeze_sha256']==sha(RUN/'freeze.json')
    assert freeze['source_sha256']==sha(ROOT/'tools/component_constrained_native.py')
    assert freeze['baseline_lock_sha256']==sha(BASE/'prediction-lock.json')
    assert scores['prediction_lock_sha256']==sha(RUN/'prediction-lock.json')
    baseline_lock=read(BASE/'prediction-lock.json');base_scores=read(BASE/'scores.json')['summary']
    checks=4;summary=[];basis=np.eye(4);basis[:3,:3]=[[0,1,0],[0,0,1],[1,0,0]]
    for entry in lock['rows']:
        c=entry['condition'];path=RUN/f'predictions-{c}.json';assert sha(path)==entry['prediction_sha256']
        for p,digest in freeze['input_pins'][c].items():assert sha(ROOT/p)==digest;checks+=1
        base_path=BASE/f'predictions-{c}.json';pin=next(x for x in baseline_lock['rows'] if x['condition']==c)
        assert sha(base_path)==pin['prediction_sha256'];backbone=read(base_path)['rows']
        rows=read(path)['rows'];assert [x['pair'] for x in rows]==list(range(599))
        refs=[]
        for i in range(600):
            t=np.loadtxt(REC/'references'/c/f'{i:06}.txt').reshape(4,4)
            u,_,vt=np.linalg.svd(t[:3,:3]);assert np.linalg.det(u@vt)>0;t[:3,:3]=u@vt;refs.append(t)
        methods={}
        for method,value in scores['summary'][c].items():
            previous=[0.,0.,0.,0.,0.,0.,1.];states=[pose(previous)];missing=[];statuses={}
            for i,row in enumerate(rows):
                result=row['methods'][method];rgb=backbone[i]['methods']['rgb_feedback']
                assert result['parent_xyzw']==previous
                statuses[result['status']]=statuses.get(result['status'],0)+1
                if result['transform'] is None:
                    assert result['parent_xyzw']==result['committed_xyzw'];missing.append(i)
                else:
                    assert result['committed_xyzw'][3:]==rgb['committed_xyzw'][3:]
                    expected=basis@np.linalg.solve(pose(previous),pose(result['committed_xyzw']))@basis.T
                    assert np.allclose(expected,result['transform'],atol=1e-10,rtol=0)
                previous=result['committed_xyzw'];states.append(pose(previous));checks+=3
            metrics=[(0,599,value['full_path'],base_scores[c]['rgb_feedback']['full_path'])]+[
                (a['start'],a['start']+99,a,b) for a,b in zip(value['blocks'],base_scores[c]['rgb_feedback']['blocks'])]
            for start,end,reported,original in metrics:
                gaps=[i for i in missing if start<=i<end]
                if gaps:
                    assert reported['status']=='INCOMPLETE_NO_GAP_FILL' and reported['missing_pairs']==gaps;checks+=1
                    continue
                predicted=[basis@np.linalg.solve(states[start],t)@basis.T for t in states[start:end+1]]
                reference=[np.linalg.solve(refs[start],t) for t in refs[start:end+1]]
                distances=[float(np.sum((a[:3,3]-b[:3,3])**2)) for a,b in zip(predicted,reference)]
                angles=[float(Rotation.from_matrix(b[:3,:3].T@a[:3,:3]).magnitude())**2
                        for a,b in zip(predicted,reference)]
                position=float(np.sqrt(sum(distances)/len(distances)))
                orientation=float(np.rad2deg(np.sqrt(sum(angles)/len(angles))))
                assert abs(position-reported['position_rmse_m'])<1e-7
                assert abs(orientation-reported['orientation_rmse_deg'])<1e-7
                assert abs(reported['orientation_rmse_deg']-original['orientation_rmse_deg'])<1e-8
                checks+=3
            methods[method]={'missing':missing,'statuses':statuses,'native_RGB_rotation_exact':True,
                             'direct_committed_state_scores_agree':True}
        summary.append({'condition':c,'methods':methods})
    value={'at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'checks':checks,
           'summary':summary,'status':'PASS','source_sha256':sha(Path(__file__))}
    (RUN/'audit.json').write_text(json.dumps(value,indent=2)+'\n');print(json.dumps(value))


if __name__=='__main__':audit()
