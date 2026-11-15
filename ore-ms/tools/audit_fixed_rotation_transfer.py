"""Audit unchanged method pins, reference binding, native control and state scores."""
from pathlib import Path
import json
import hashlib
import datetime
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/fixed-rotation-transfer'
RUN=OUT/'run-001'


def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def pose(v):
    t=np.eye(4);t[:3,:3]=Rotation.from_quat(v[3:]).as_matrix();t[:3,3]=v[:3];return t


def audit():
    freeze=read(RUN/'freeze.json');lock=read(RUN/'prediction-lock.json');scores=read(RUN/'scores.json')
    assert freeze['source_sha256']==sha(ROOT/'tools/fixed_rotation_recording_transfer.py')
    assert lock['freeze_sha256']==sha(RUN/'freeze.json')
    assert scores['prediction_lock_sha256']==sha(RUN/'prediction-lock.json')
    checks=3
    for p,digest in freeze['implementation_pins'].items():assert sha(ROOT/p)==digest;checks+=1
    for p,digest in freeze['input_pins'].items():assert sha(ROOT/p)==digest;checks+=1
    proof=read(OUT/'recovery/recovery-proof.json')
    for entry in proof['files']:assert sha(ROOT/entry['path'])==entry['sha256'];checks+=1
    basis=np.eye(4);basis[:3,:3]=[[0,1,0],[0,0,1],[1,0,0]];summaries=[]
    for entry in lock['rows']:
        c=entry['condition'];path=RUN/f'predictions-{c}.json';assert sha(path)==entry['prediction_sha256']
        payload=read(path);rows=payload['rows'];assert len(rows)==99
        assert [r['pair'] for r in rows]==list(range(99))
        for check in payload['native_reproduction']:
            assert check['translation_m']<1e-3 and check['rotation_rad']<1e-4;checks+=2
        refs=[]
        for i in range(100):
            t=np.loadtxt(OUT/'recovery/references'/c/f'{i:06}.txt').reshape(4,4)
            u,_,vt=np.linalg.svd(t[:3,:3]);assert np.linalg.det(u@vt)>0;t[:3,:3]=u@vt;refs.append(t)
        methods={}
        for method,result in scores['summary'][c]['methods'].items():
            prior=[0.,0.,0.,0.,0.,0.,1.];states=[pose(prior)];missing=[];statuses={}
            for i,row in enumerate(rows):
                current=row['methods'][method];backbone=row['methods']['native_rgb']
                assert current['parent_xyzw']==prior
                statuses[current['status']]=statuses.get(current['status'],0)+1
                if current['transform'] is None:
                    assert current['parent_xyzw']==current['committed_xyzw'];missing.append(i)
                else:
                    assert current['committed_xyzw'][3:]==backbone['committed_xyzw'][3:]
                    delta=basis@np.linalg.solve(pose(prior),pose(current['committed_xyzw']))@basis.T
                    assert np.allclose(delta,current['transform'],atol=1e-10,rtol=0)
                prior=current['committed_xyzw'];states.append(pose(prior));checks+=3
            if missing:
                assert result['full_path']['status']=='INCOMPLETE_NO_GAP_FILL'
            else:
                predicted=[basis@np.linalg.solve(states[0],s)@basis.T for s in states]
                reference=[np.linalg.solve(refs[0],r) for r in refs]
                position=float(np.sqrt(sum(float(np.sum((a[:3,3]-b[:3,3])**2)) for a,b in zip(predicted,reference))/100))
                orientation=float(np.rad2deg(np.sqrt(sum(float(Rotation.from_matrix(b[:3,:3].T@a[:3,:3]).magnitude())**2
                                                      for a,b in zip(predicted,reference))/100)))
                assert abs(position-result['full_path']['position_rmse_m'])<1e-7
                assert abs(orientation-result['full_path']['orientation_rmse_deg'])<1e-7
                native=scores['summary'][c]['methods']['native_rgb']['full_path']
                assert abs(result['full_path']['orientation_rmse_deg']-native['orientation_rmse_deg'])<1e-8
                checks+=3
            # Independent interval arithmetic from committed states.
            values=[]
            for i,row in enumerate(rows):
                if row['methods'][method]['transform'] is None:continue
                predicted=basis@np.linalg.solve(states[i],states[i+1])@basis.T
                expected=np.linalg.solve(refs[i],refs[i+1]);difference=np.linalg.solve(expected,predicted)
                values.append(float(np.linalg.norm(difference[:3,3])))
            if values:
                stats=result['increment_translation_m'];computed=[sum(values)/len(values),
                    (sum(x*x for x in values)/len(values))**.5,float(np.percentile(values,95))]
                for key,value in zip(['mean','rmse','p95'],computed):assert abs(stats[key]-value)<1e-10;checks+=1
            methods[method]={'available':99-len(missing),'missing':missing,'statuses':statuses,
                             'native_rotation_exact':True,'direct_state_metrics_agree':True}
        summaries.append({'condition':c,'methods':methods,'native_reproduction_count':len(payload['native_reproduction']),
            'max_native_translation_difference_m':max(x['translation_m'] for x in payload['native_reproduction']),
            'max_native_rotation_difference_rad':max(x['rotation_rad'] for x in payload['native_reproduction'])})
    value={'at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'checks':checks,'summary':summaries,
           'status':'PASS','source_sha256':sha(Path(__file__))}
    (RUN/'audit.json').write_text(json.dumps(value,indent=2)+'\n');print(json.dumps(value))


if __name__=='__main__':audit()
