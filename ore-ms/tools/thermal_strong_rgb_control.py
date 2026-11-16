"""Single new thermal arm against locked, qualified RGB day controls."""
from pathlib import Path
import copy,json,sys,datetime
import numpy as np
import torch
import optimization_day_transfer as day
import audit_optimization_day_transfer as independent

base=day.base;native=day.native;ROOT=day.ROOT;REC=day.REC
OUT=ROOT/'results/thermal-strong-rgb-day'
PROTOCOL=ROOT/'docs/research/thermal-strong-rgb-protocol-2026-11-15.md'
THERMAL='thermal_translation'


def capture():
    torch.set_num_threads(1);source,rgb,events=day.inputs()
    old=day.OUT/'run-001';lock=native.read(old/'prediction-lock.json');parent=native.read(old/'freeze.json')
    assert native.sha(old/'predictions.json')==lock['prediction_sha256']
    assert native.sha(old/'freeze.json')==lock['freeze_sha256']
    rows=native.read(old/'predictions.json')['rows'];assert len(rows)==99
    for kind in ['source_pins','input_pins']:
        for p,h in parent[kind].items():assert native.sha(ROOT/p)==h,p
    manifest=native.read('/tmp/ore-transfer-manifest.json')
    tpins={f['path']:f['sha256'] for f in manifest['files'] if '/captures/day/thermal/' in f['path']}
    assert len(tpins)==3
    for p,h in tpins.items():assert native.sha(ROOT/p)==h,p
    with np.load(REC/'captures/day/thermal/tensor-map.npz') as z:thermal={k:z[k] for k in z.files if not k.startswith('edge/point2match')}
    tevents={}
    for line in (REC/'captures/day/thermal/arbiter-events/events.jsonl').read_text().splitlines():
        e=json.loads(line)
        if e['kind']=='motion_result_written' and e['first_write_for_submission']:tevents[(e['from_idx'],e['frame_idx'])]=e
    assert thermal['frames//time_ns'].reshape(-1).tolist()==[r['cameras']['thr']['left']['recorded_time_ns'] for r in source['records'][:100]]
    extrinsics=native.geometry(native.read(REC/'inputs/day/calibration.json'))['thermal']
    target=OUT/'run-001';target.mkdir(parents=True,exist_ok=False)
    freeze={**parent,'at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'source_pins':{**parent['source_pins'],str(Path(__file__).relative_to(ROOT)):native.sha(__file__),str(PROTOCOL.relative_to(ROOT)):native.sha(PROTOCOL),'tools/audit_optimization_day_transfer.py':native.sha(independent.__file__)},
        'input_pins':{**parent['input_pins'],**tpins},'rgb_prediction_lock_sha256':native.sha(old/'prediction-lock.json'),
        'rgb_predictions_sha256':native.sha(old/'predictions.json'),'thermal_precision':.1,
        'scope':'Single fixed thermal increment vs locked RGB day own-state controls; separate timing captures'}
    native.write(target/'freeze.json',freeze);state=torch.tensor(parent['initial_state'],dtype=torch.float32)
    with (target/'capture.jsonl').open('x') as stream:
        for row in rows:
            i=row['pair'];e=events[(i,i+1)];prior=state.clone()
            packet,meta=native.bind_camera(thermal,tevents[(i,i+1)],i,source['records'],'thr',extrinsics)
            def data():return base.feedback.feedback_input(rgb,e,i,source['records'],prior)
            first,t1=base.timed_solve(native.Analytic_ReprojDisp_TwoFramePGO(data()))
            graph=native.NativeThermalGraph(data(),packet,native.pose(prior.numpy()),thermal_precision=.1)
            graph=base.component.FixedRotationGraph(graph,native.pp.SE3(first.double()[None]))
            state,t2=base.timed_solve(graph);assert state[3:].tolist()==first[3:].tolist()
            receipts=[t1,t2];combined={'losses':t1['losses']+t2['losses'],**{k:t1[k]+t2[k] for k in ['steps','solver_cpu_seconds','solver_wall_seconds']}}
            row['methods'][THERMAL]=base.result(prior,state,combined);row['stages'][THERMAL]={'first':first.tolist(),'receipts':receipts}
            row['thermal_binding']=meta
            stream.write(json.dumps(row,allow_nan=False)+'\n');stream.flush()
    native.write(target/'predictions.json',{'rows':rows})
    native.write(target/'prediction-lock.json',{'freeze_sha256':native.sha(target/'freeze.json'),'prediction_sha256':native.sha(target/'predictions.json'),'reference_scores_read':False})
    print('LOCKED99',flush=True)


def score():
    day.OUT=OUT;day.METHODS=base.METHODS+[THERMAL];day.score()


def audit():
    path=OUT/'run-001';rows=native.read(path/'predictions.json')['rows'];f=native.read(path/'freeze.json')
    old=day.OUT/'run-001';original=native.read(old/'predictions.json')['rows']
    assert native.sha(old/'prediction-lock.json')==f['rgb_prediction_lock_sha256']
    assert native.sha(old/'predictions.json')==f['rgb_predictions_sha256']
    for r,oldrow in zip(rows,original):
        copied=copy.deepcopy(r);del copied['methods'][THERMAL];del copied['stages'][THERMAL];del copied['thermal_binding']
        assert copied==oldrow
        assert r['methods'][THERMAL]['committed_xyzw'][3:]==r['stages'][THERMAL]['first'][3:]
    scores=native.read(path/'scores.json');lock=native.read(path/'prediction-lock.json')
    assert native.sha(path/'freeze.json')==lock['freeze_sha256'] and native.sha(path/'predictions.json')==lock['prediction_sha256']
    for kind in ['source_pins','input_pins']:
        for name,h in f[kind].items():assert native.sha(ROOT/name)==h,name
    assert rows==[json.loads(x) for x in (path/'capture.jsonl').read_text().splitlines()]
    _,_,events=day.inputs()
    refs={i:independent.proper(np.loadtxt(REC/f'references/day/{i:06}.txt').reshape(4,4)) for i in range(100)}
    rgb_rows=copy.deepcopy(rows)
    for r in rgb_rows:del r['methods'][THERMAL];del r['stages'][THERMAL];del r['thermal_binding']
    rgb_scores={**scores,'summary':{m:scores['summary'][m] for m in base.METHODS}}
    checks=independent.audit(rgb_rows,rgb_scores,f,events,refs)
    def thermal_checks(rs,ss):
        import math
        count=0
        def check(value):
            nonlocal count
            assert value
            count+=1
        state=f['initial_state'];rot=np.eye(3);pos=np.zeros(3);pe=[0.];ae=[0.];tv=[];rv=[]
        for r in rs:
            i=r['pair'];v=r['methods'][THERMAL];stage=r['stages'][THERMAL]
            check(v['parent_xyzw']==state)
            check(v['committed_xyzw'][3:]==stage['first'][3:])
            expected=independent.BASIS@np.linalg.inv(independent.pose(state))@independent.pose(v['committed_xyzw'])@independent.BASIS.T
            check(np.allclose(expected,v['transform'],rtol=0,atol=1e-10));state=v['committed_xyzw']
            for receipt in stage['receipts']:check(1<=receipt['steps']<=10 and len(receipt['losses'])==receipt['steps'])
            for k in ['steps','solver_cpu_seconds','solver_wall_seconds']:check(v[k]==sum(x[k] for x in stage['receipts']))
            check(v['losses']==sum([x['losses'] for x in stage['receipts']],[]))
            d=independent.proper(v['transform']);ref=np.linalg.solve(refs[i],refs[i+1])
            tv.append(np.linalg.norm(d[:3,3]-ref[:3,3]));rv.append(math.degrees(independent.Rotation.from_matrix(ref[:3,:3].T@d[:3,:3]).magnitude()))
            pos=pos+rot@d[:3,3];rot=rot@d[:3,:3];truth=np.linalg.solve(refs[0],refs[i+1])
            pe.append(np.linalg.norm(pos-truth[:3,3]));ae.append(math.degrees(independent.Rotation.from_matrix(truth[:3,:3].T@rot).magnitude()))
        actual=ss['summary'][THERMAL];check(actual['available']==99)
        for key,vals in [('translation_m',tv),('rotation_deg',rv),('path_position_m',pe),('path_orientation_deg',ae)]:
            for k,v in independent.measure(vals).items():check(math.isclose(actual[key][k],v,rel_tol=1e-10,abs_tol=1e-10))
        for key,leaf in [('cpu_total_seconds','solver_cpu_seconds'),('wall_total_seconds','solver_wall_seconds'),('steps_total','steps')]:check(math.isclose(actual[key],math.fsum(r['methods'][THERMAL][leaf] for r in rs),rel_tol=1e-10,abs_tol=1e-10))
        return count
    checks+=thermal_checks(rows,scores);mutations=[]
    for kind in ['state','rotation','cost','score']:
        bad=copy.deepcopy(rows);bs=copy.deepcopy(scores)
        if kind=='state':bad[1]['methods'][THERMAL]['parent_xyzw'][0]+=1
        elif kind=='rotation':bad[0]['methods'][THERMAL]['committed_xyzw'][3]+=.01
        elif kind=='cost':bad[0]['stages'][THERMAL]['receipts'][1]['steps']=11
        else:bs['summary'][THERMAL]['path_position_m']['rmse']+=1
        try:thermal_checks(bad,bs)
        except AssertionError:mutations.append(kind)
        else:raise AssertionError('escaped mutation '+kind)
    receipt={'status':'PASS','checks':checks,'mutations_rejected':mutations,'rgb_objects_unchanged':True,'implementation_sha256':native.sha(__file__),'scores_sha256':native.sha(path/'scores.json')}
    with (path/'audit.json').open('x') as stream:json.dump(receipt,stream,indent=2);stream.write('\n')
    print(json.dumps(receipt,indent=2))


if __name__=='__main__':{'capture':capture,'score':score,'audit':audit}[sys.argv[1]]()
