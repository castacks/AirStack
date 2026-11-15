"""Independently verify prefix reuse, continuous state, fixed rotations and scores."""
from pathlib import Path
import json
import hashlib
import datetime
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/fixed-rotation-continuation'
RUN=OUT/'run-001'
EVAL=OUT/'evaluation-001'
RGB=ROOT/'results/native-pose-feedback/run-001'
FIXED=ROOT/'results/component-constrained-native/run-001'


def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def pose(v):
    t=np.eye(4);t[:3,:3]=Rotation.from_quat(v[3:]).as_matrix();t[:3,3]=v[:3];return t


def audit():
    freeze=read(RUN/'freeze.json');lock=read(RUN/'prediction-lock.json');scores=read(EVAL/'scores.json')
    assert freeze['source_sha256']==sha(ROOT/'tools/fixed_rotation_continuation.py')
    assert lock['freeze_sha256']==sha(RUN/'freeze.json')
    assert scores['prediction_lock_sha256']==sha(RUN/'prediction-lock.json')
    assert freeze['reference_recovery_proof_sha256']==sha(OUT/'recovery/proof.json')
    assert scores['protocol_sha256']==sha(EVAL/'protocol.json')
    protocol=read(EVAL/'protocol.json')
    assert protocol['native_translation_bound_m']==1e-3 and protocol['native_rotation_bound_rad']==1e-4
    assert protocol['source_sha256']==sha(ROOT/'tools/score_qualified_continuation.py')
    checks=7
    for path,digest in freeze['implementation_pins'].items():assert sha(ROOT/path)==digest;checks+=1
    for path,digest in freeze['prefix_pins'].items():assert sha(ROOT/path)==digest;checks+=1
    for entry in read(OUT/'recovery/proof.json')['files']:assert sha(ROOT/entry['path'])==entry['sha256'];checks+=1
    basis=np.eye(4);basis[:3,:3]=[[0,1,0],[0,0,1],[1,0,0]];summary=[]
    for receipt in lock['rows']:
        c=receipt['condition'];path=RUN/f'predictions-{c}.json';assert sha(path)==receipt['prediction_sha256']
        for p,digest in freeze['input_pins'][c].items():assert sha(ROOT/p)==digest;checks+=1
        payload=read(path);rows=payload['rows'];assert [x['pair'] for x in rows]==list(range(1199))
        rgb=read(RGB/f'predictions-{c}.json')['rows'];fixed=read(FIXED/f'predictions-{c}.json')['rows']
        for i in range(599):
            assert rows[i]['methods']['native_rgb']==rgb[i]['methods']['rgb_feedback']
            assert rows[i]['methods']['rgb_refine']==fixed[i]['methods']['rgb_fixed_rotation']
            assert rows[i]['methods']['thermal_refine']==fixed[i]['methods']['thermal_fixed_rotation'];checks+=3
        failed_reproductions=[]
        for value in payload['new_native_reproduction']:
            assert 599<=value['pair']<1199
            if value['translation_m']>=1e-3 or value['rotation_rad']>=1e-4:
                failed_reproductions.append(value)
            checks+=1
        assert scores['summary'][c]['native_reproduction_failures']==failed_reproductions
        assert scores['summary'][c]['native_reproduction_gate_passed']==(not failed_reproductions)
        expected_scope='diagnostic_only_native_reproduction_gate_failed' if failed_reproductions else 'qualified_saved_measurement_replay'
        assert scores['summary'][c]['accuracy_interpretation']==expected_scope
        checks+=3
        refs=[];invalid=[]
        for i in range(1200):
            t=np.loadtxt(OUT/'recovery/references'/c/f'{i:06}.txt').reshape(4,4);r=t[:3,:3]
            valid=(np.isfinite(t).all() and np.array_equal(t[3],[0,0,0,1]) and
                   np.linalg.norm(r.T@r-np.eye(3))<=1e-5 and abs(np.linalg.det(r)-1)<=1e-5)
            if not valid:refs.append(None);invalid.append(i);continue
            u,_,vt=np.linalg.svd(r);t[:3,:3]=u@np.diag([1,1,np.linalg.det(u@vt)])@vt;refs.append(t)
        assert set(invalid)==set(int(i) for i in scores['summary'][c]['invalid_references'])
        methods={}
        for method,report in scores['summary'][c]['methods'].items():
            prior=[0.,0.,0.,0.,0.,0.,1.];states=[pose(prior)];missing=[];statuses={}
            for i,row in enumerate(rows):
                value=row['methods'][method];backbone=row['methods']['native_rgb'];assert value['parent_xyzw']==prior
                if i>=599:statuses[value['status']]=statuses.get(value['status'],0)+1
                if value['transform'] is None:
                    assert value['parent_xyzw']==value['committed_xyzw'];missing.append(i)
                else:
                    assert value['committed_xyzw'][3:]==backbone['committed_xyzw'][3:]
                    expected=basis@np.linalg.solve(pose(prior),pose(value['committed_xyzw']))@basis.T
                    assert np.allclose(expected,value['transform'],rtol=0,atol=1e-10)
                prior=value['committed_xyzw'];states.append(pose(prior));checks+=3
            spans=[(599,1199,report['new_span']),(0,1199,report['full_path'])]+[
                (b['start'],b['start']+99,b) for b in report['new_blocks']]
            for start,end,value in spans:
                bad=[i for i in invalid if start<=i<=end]
                if bad:
                    assert value['status']=='UNQUALIFIED_REFERENCE' and value['reference_source_ids']==bad;checks+=1;continue
                gaps=[i for i in missing if start<=i<end]
                if gaps:
                    assert value['status']=='INCOMPLETE_NO_GAP_FILL' and value['missing_pairs']==gaps;checks+=1;continue
                predicted=[basis@np.linalg.solve(states[start],t)@basis.T for t in states[start:end+1]]
                reference=[np.linalg.solve(refs[start],t) for t in refs[start:end+1]]
                position=float(np.sqrt(sum(float(np.sum((a[:3,3]-b[:3,3])**2)) for a,b in zip(predicted,reference))/len(predicted)))
                orientation=float(np.rad2deg(np.sqrt(sum(float(Rotation.from_matrix(b[:3,:3].T@a[:3,:3]).magnitude())**2
                                                       for a,b in zip(predicted,reference))/len(predicted))))
                assert abs(position-value['position_rmse_m'])<1e-7
                assert abs(orientation-value['orientation_rmse_deg'])<1e-7;checks+=2
                original=scores['summary'][c]['methods']['native_rgb']
                comparison=original['new_span'] if start==599 else original['full_path'] if start==0 else next(b for b in original['new_blocks'] if b['start']==start)
                assert abs(comparison['orientation_rmse_deg']-value['orientation_rmse_deg'])<1e-8;checks+=1
            methods[method]={'new_available':600-sum(i>=599 for i in missing),'all_missing_pairs':missing,
                'new_statuses':statuses,'prefix_reused_exactly':True,'state_and_fixed_rotation_verified':True}
        native_errors=payload['new_native_reproduction']
        summary.append({'condition':c,'invalid_reference_ids':invalid,'methods':methods,
            'native_reproduction_gate_passed':not failed_reproductions,
            'failed_reproductions':failed_reproductions,
            'native_reproduction_count':len(native_errors),
            'max_native_translation_difference_m':max(x['translation_m'] for x in native_errors),
            'max_native_rotation_difference_rad':max(x['rotation_rad'] for x in native_errors)})
    value={'at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'checks':checks,'summary':summary,
           'status':'PASS_STRUCTURAL_AND_ELIGIBILITY_AUDIT',
           'all_native_reproduction_gates_passed':all(x['native_reproduction_gate_passed'] for x in summary),
           'source_sha256':sha(Path(__file__))}
    (EVAL/'audit.json').write_text(json.dumps(value,indent=2)+'\n');print(json.dumps(value))


if __name__=='__main__':audit()
