"""Independent saved-state, reference-metric and cost audit of the day-prefix control."""
import copy,json,math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from audit_optimization_budget_control import sha,read,pose,proper,measure,METHODS,BASIS
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/optimization-day-transfer/run-001'
REC=ROOT/'results/fixed-rotation-transfer/recovery'


def audit(rows,scores,freeze,events,refs):
    count=0
    def check(value,label):
        nonlocal count
        assert value,label
        count+=1
    def same(actual,expected):
        for k,v in expected.items():check(math.isclose(actual[k],v,rel_tol=1e-10,abs_tol=1e-10),k)
    check([r['pair'] for r in rows]==list(range(99)),'population')
    states={m:freeze['initial_state'] for m in METHODS}
    for row in rows:
        i=row['pair'];check(row['order']==METHODS[i%3:]+METHODS[:i%3],'order')
        for m in METHODS:
            v=row['methods'][m];stage=row['stages'][m]
            check(v['parent_xyzw']==states[m],'own state')
            expected=BASIS@np.linalg.inv(pose(states[m]))@pose(v['committed_xyzw'])@BASIS.T
            check(np.allclose(v['transform'],expected,rtol=0,atol=1e-10),'state motion')
            rs=stage['receipts'];check(len(rs)==(1 if m=='native_rgb' else 2),'solve count')
            for r in rs:
                check(1<=r['steps']<=10 and len(r['losses'])==r['steps'],'step budget')
                check(r['solver_cpu_seconds']>0 and r['solver_wall_seconds']>0,'timing')
            check(v['losses']==sum([r['losses'] for r in rs],[]),'loss accounting')
            for key in ['steps','solver_cpu_seconds','solver_wall_seconds']:check(v[key]==sum(r[key] for r in rs),'stage costs')
            if m=='native_rgb':check(v['committed_xyzw']==stage['first'],'native state')
            if m=='extra_translation':check(v['committed_xyzw'][3:]==stage['first'][3:],'own first rotation')
            states[m]=v['committed_xyzw']
        delta=np.linalg.inv(BASIS@pose(events[(i,i+1)]['derived_camera_delta_xyzw'])@BASIS.T)@np.asarray(row['methods']['native_rgb']['transform'])
        t=np.linalg.norm(delta[:3,3]);a=Rotation.from_matrix(delta[:3,:3]).magnitude()
        same(row['reproduction'],{'translation_m':t,'rotation_rad':a})
        check(row['reproduction']['passed']==bool(t<1e-3 and a<1e-4),'native bound')
    check(scores['native_gate_passed']==all(r['reproduction']['passed'] for r in rows),'aggregate gate')
    for m in METHODS:
        tv=[];rv=[];pv=[0.];av=[0.];rot=np.eye(3);pos=np.zeros(3)
        for row in rows:
            i=row['pair'];d=proper(row['methods'][m]['transform']);ref=np.linalg.solve(refs[i],refs[i+1])
            tv.append(np.linalg.norm(d[:3,3]-ref[:3,3]));rv.append(math.degrees(Rotation.from_matrix(ref[:3,:3].T@d[:3,:3]).magnitude()))
            pos=pos+rot@d[:3,3];rot=rot@d[:3,:3];truth=np.linalg.solve(refs[0],refs[i+1])
            pv.append(np.linalg.norm(pos-truth[:3,3]));av.append(math.degrees(Rotation.from_matrix(truth[:3,:3].T@rot).magnitude()))
        s=scores['summary'][m];check(s['available']==99,'availability')
        for field,vals in [('translation_m',tv),('rotation_deg',rv),('path_position_m',pv),('path_orientation_deg',av)]:same(s[field],measure(vals))
        same(s,{'cpu_total_seconds':math.fsum(r['methods'][m]['solver_cpu_seconds'] for r in rows),'wall_total_seconds':math.fsum(r['methods'][m]['solver_wall_seconds'] for r in rows),'steps_total':sum(r['methods'][m]['steps'] for r in rows)})
    return count


def main():
    f=read(OUT/'freeze.json');lock=read(OUT/'prediction-lock.json');scores=read(OUT/'scores.json');rows=read(OUT/'predictions.json')['rows']
    assert sha(OUT/'freeze.json')==lock['freeze_sha256'] and sha(OUT/'predictions.json')==lock['prediction_sha256']
    assert scores['prediction_lock_sha256']==sha(OUT/'prediction-lock.json')
    for k in ['source_pins','input_pins']:
        for p,h in f[k].items():assert sha(ROOT/p)==h,p
    assert rows==[json.loads(x) for x in (OUT/'capture.jsonl').read_text().splitlines()]
    events={}
    for line in (REC/'captures/day/rgb/arbiter-events/events.jsonl').read_text().splitlines():
        e=json.loads(line)
        if e['kind']=='motion_result_written' and e['first_write_for_submission']:events[(e['from_idx'],e['frame_idx'])]=e
    refs={i:proper(np.loadtxt(REC/f'references/day/{i:06}.txt').reshape(4,4)) for i in range(100)}
    count=audit(rows,scores,f,events,refs);mutations=[]
    for kind in ['population','state','rotation','cost','score']:
        bad=copy.deepcopy(rows);bs=copy.deepcopy(scores)
        if kind=='population':bad.pop()
        elif kind=='state':bad[1]['methods']['extra_full_pose']['parent_xyzw'][0]+=1
        elif kind=='rotation':bad[0]['methods']['extra_translation']['committed_xyzw'][3]+=.01
        elif kind=='cost':bad[0]['stages']['extra_full_pose']['receipts'][1]['steps']=11
        else:bs['summary']['extra_translation']['path_position_m']['rmse']+=1
        try:audit(bad,bs,f,events,refs)
        except AssertionError:mutations.append(kind)
        else:raise AssertionError('escaped mutation '+kind)
    result={'status':'PASS','checks':count,'mutations_rejected':mutations,'auditor_sha256':sha(__file__),'scores_sha256':sha(OUT/'scores.json'),'prediction_lock_sha256':sha(OUT/'prediction-lock.json')}
    with (OUT/'audit.json').open('x') as stream:json.dump(result,stream,indent=2);stream.write('\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
