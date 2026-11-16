"""Independent continuity, transform, metric and cost audit of feedback replay."""
import copy
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from audit_optimization_budget_control import sha,read,pose,proper,measure,METHODS,BASIS

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/optimization-feedback-control/run-001'


def check_output(rows,scores,freeze,events,refs):
    checks=0
    def check(value,label):
        nonlocal checks
        assert value,label
        checks+=1
    def compare(a,b):
        for k,v in b.items():check(math.isclose(a[k],v,rel_tol=1e-10,abs_tol=1e-10),k)
    check([r['pair'] for r in rows]==list(range(599,1199)),'population')
    check(scores['common_pairs']==600,'common support')
    states={m:freeze['initial_state'] for m in METHODS}
    for row in rows:
        i=row['pair'];check(row['order']==METHODS[(i-599)%3:]+METHODS[:(i-599)%3],'order')
        for m in METHODS:
            v=row['methods'][m];s=row['stages'][m]
            check(v['parent_xyzw']==states[m],'own state continuity')
            check(v['transform'] is not None,'complete availability')
            expected=BASIS@np.linalg.inv(pose(states[m]))@pose(v['committed_xyzw'])@BASIS.T
            check(np.allclose(v['transform'],expected,rtol=0,atol=1e-10),'pose binding')
            if m=='native_rgb':check(v['committed_xyzw']==s['first_committed_xyzw'],'native one pass')
            else:
                if m=='extra_translation':check(v['committed_xyzw'][3:]==s['first_committed_xyzw'][3:],'fixed first-pass rotation')
            receipts=[s['first_receipt']]+([s['second_receipt']] if m!='native_rgb' else [])
            check(v['losses']==sum([x['losses'] for x in receipts],[]),'loss accounting')
            for x in receipts:check(1<=x['steps']<=10 and x['steps']==len(x['losses']),'ten step cap')
            for key in ['steps','solver_cpu_seconds','solver_wall_seconds']:
                check(v[key]==sum(x[key] for x in receipts),'combined cost')
            states[m]=v['committed_xyzw']
        expected=BASIS@pose(events[(i,i+1)]['derived_camera_delta_xyzw'])@BASIS.T
        delta=np.linalg.inv(expected)@np.asarray(row['methods']['native_rgb']['transform'])
        t=float(np.linalg.norm(delta[:3,3]));r=float(Rotation.from_matrix(delta[:3,:3]).magnitude())
        compare(row['reproduction'],{'translation_m':t,'rotation_rad':r})
        check(row['reproduction']['passed']==bool(t<1e-3 and r<1e-4),'reproduction gate')
    check(scores['all_native_reproduction_passed']==all(r['reproduction']['passed'] for r in rows),'aggregate native gate')
    for m in METHODS:
        te=[];re=[];pe=[0.];ae=[0.];acc=np.eye(4)
        for row in rows:
            i=row['pair'];v=proper(row['methods'][m]['transform']);ref=np.linalg.inv(refs[i])@refs[i+1]
            te.append(np.linalg.norm(v[:3,3]-ref[:3,3]));re.append(math.degrees(Rotation.from_matrix(ref[:3,:3].T@v[:3,:3]).magnitude()))
            acc=acc@v;truth=np.linalg.inv(refs[599])@refs[i+1]
            pe.append(np.linalg.norm(acc[:3,3]-truth[:3,3]));ae.append(math.degrees(Rotation.from_matrix(truth[:3,:3].T@acc[:3,:3]).magnitude()))
        s=scores['summary'][m];check(s['available']==600,'availability')
        for field,values in [('interval_translation_m',te),('interval_rotation_deg',re)]:compare(s[field],measure(values))
        compare(s['fixed_pair_path']['position_m'],measure(pe));compare(s['fixed_pair_path']['orientation_deg'],measure(ae))
        for field in ['solver_cpu_seconds','solver_wall_seconds','scheduler_steps']:
            key='steps' if field=='scheduler_steps' else field
            values=[r['methods'][m][key] for r in rows]
            compare(s[field],{**measure(values),'total':math.fsum(values)})
    return checks


def main():
    f=read(OUT/'freeze.json');lock=read(OUT/'prediction-lock.json');rows=read(OUT/'predictions.json')['rows'];s=read(OUT/'scores.json')
    assert sha(OUT/'freeze.json')==lock['freeze_sha256'] and sha(OUT/'predictions.json')==lock['prediction_sha256']
    assert s['prediction_lock_sha256']==sha(OUT/'prediction-lock.json')
    for kind in ['source_pins','input_pins','reference_pins']:
        for p,h in f[kind].items():assert sha(ROOT/p)==h,p
    assert rows==[json.loads(x) for x in (OUT/'capture.jsonl').read_text().splitlines()]
    events={}
    for line in (ROOT/'results/longer-fixed-thermal/recovery/captures/night/rgb/arbiter-events/events.jsonl').read_text().splitlines():
        e=json.loads(line)
        if e['kind']=='motion_result_written' and e['first_write_for_submission']:events[(e['from_idx'],e['frame_idx'])]=e
    refs={i:proper(np.loadtxt(ROOT/f'results/fixed-rotation-continuation/recovery/references/night/{i:06}.txt').reshape(4,4)) for i in range(599,1200)}
    checks=check_output(rows,s,f,events,refs);mutations=[]
    for kind in ['state','rotation','cost','score']:
        bad=copy.deepcopy(rows);bs=copy.deepcopy(s)
        if kind=='state':bad[1]['methods']['extra_full_pose']['parent_xyzw'][0]+=1
        elif kind=='rotation':bad[0]['methods']['extra_translation']['committed_xyzw'][3]+=.01
        elif kind=='cost':bad[0]['stages']['extra_full_pose']['second_receipt']['steps']=11
        else:bs['summary']['extra_translation']['fixed_pair_path']['position_m']['rmse']+=1
        try:check_output(bad,bs,f,events,refs)
        except AssertionError:mutations.append(kind)
        else:raise AssertionError('Mutation escaped: '+kind)
    result={'status':'PASS','checks':checks,'mutations_rejected':mutations,'auditor_sha256':sha(__file__),
            'scores_sha256':sha(OUT/'scores.json'),'prediction_lock_sha256':sha(OUT/'prediction-lock.json')}
    with (OUT/'audit.json').open('x') as stream:json.dump(result,stream,indent=2);stream.write('\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
