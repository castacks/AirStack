"""Day-prefix transfer driver; reuse frozen native solver/wrapper, retain own state."""
from pathlib import Path
import datetime,json,sys
import numpy as np
import torch
from scipy.spatial.transform import Rotation
import optimization_budget_control as base
from audit_optimization_budget_control import proper,pose,measure

ROOT=base.ROOT;native=base.native
REC=ROOT/'results/fixed-rotation-transfer/recovery'
OUT=ROOT/'results/optimization-day-transfer'
PROTOCOL=ROOT/'docs/research/optimization-day-transfer-protocol-2026-11-15.md'
METHODS=base.METHODS
B=np.eye(4);B[:3,:3]=native.O


def inputs():
    source=native.read(REC/'inputs/day/source-manifest.json')
    assert source['sequence']=='_2021-08-06-11-23-45'
    config=native.read(REC/'captures/day/rgb/effective-config.json')['Odometry']
    assert config['args']['mapping'] is False and config['motion']['type']=='StaticMotionModel'
    assert config['keyframe']['type']=='AllKeyframe' and config['optimizer']['args']['graph_type']=='disp'
    with np.load(REC/'captures/day/rgb/tensor-map.npz') as z:saved={k:z[k] for k in z.files if not k.startswith('edge/point2match')}
    assert saved['frames//time_ns'].reshape(-1).tolist()==[r['cameras']['rgb']['left']['recorded_time_ns'] for r in source['records'][:100]]
    events={}
    for line in (REC/'captures/day/rgb/arbiter-events/events.jsonl').read_text().splitlines():
        e=json.loads(line)
        if e['kind']=='motion_result_written' and e['first_write_for_submission']:
            key=(e['from_idx'],e['frame_idx']);assert key not in events;events[key]=e
    return source,saved,events


def capture():
    torch.set_num_threads(1);source,saved,events=inputs()
    manifest=native.read('/tmp/ore-transfer-manifest.json')
    pins={f['path']:f['sha256'] for f in manifest['files'] if (ROOT/f['path']).is_file() and 'fixed-rotation-transfer/recovery/' in f['path']}
    assert len(pins)==105
    for p,h in pins.items():assert native.sha(ROOT/p)==h,p
    prior=native.read(ROOT/'results/optimization-feedback-control/run-001/freeze.json')
    for p,h in prior['source_pins'].items():assert native.sha(ROOT/p)==h,p
    target=OUT/'run-001';target.mkdir(parents=True,exist_ok=False)
    freeze={'at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'source_pins':{**prior['source_pins'],str(Path(__file__).relative_to(ROOT)):native.sha(__file__),str(PROTOCOL.relative_to(ROOT)):native.sha(PROTOCOL)},'input_pins':pins,'initial_state':events[(0,1)]['parent_pose_camera_map_xyzw'],'pairs':[0,98],'scope':'Exposed100frame day recording, cached own-state RGB feedback'}
    native.write(target/'freeze.json',freeze);rows=[];states={m:torch.tensor(freeze['initial_state'],dtype=torch.float32) for m in METHODS}
    with (target/'capture.jsonl').open('x') as stream:
        for i in range(99):
            methods={};stages={};e=events[(i,i+1)]
            assert not e['original_need_interp']
            order=METHODS[i%3:]+METHODS[:i%3]
            for m in order:
                parent=states[m].clone()
                def graph():
                    data=base.feedback.feedback_input(saved,e,i,source['records'],parent)
                    return native.Analytic_ReprojDisp_TwoFramePGO(data)
                first,t1=base.timed_solve(graph());current=first;receipts=[t1]
                if m!='native_rgb':
                    g=graph().double()
                    with torch.no_grad():g.pose2opt.copy_(native.pp.SE3(first.double()[None]))
                    if m=='extra_translation':g=base.component.FixedRotationGraph(g,native.pp.SE3(first.double()[None]))
                    current,t2=base.timed_solve(g);receipts.append(t2)
                total={'losses':sum([r['losses'] for r in receipts],[]),'steps':sum(r['steps'] for r in receipts),'solver_cpu_seconds':sum(r['solver_cpu_seconds'] for r in receipts),'solver_wall_seconds':sum(r['solver_wall_seconds'] for r in receipts)}
                methods[m]=base.result(parent,current,total);stages[m]={'first':first.tolist(),'receipts':receipts};states[m]=current
            delta=np.linalg.solve(B@native.pose(e['derived_camera_delta_xyzw'])@B.T,np.asarray(methods['native_rgb']['transform']))
            repro={'translation_m':float(np.linalg.norm(delta[:3,3])),'rotation_rad':float(Rotation.from_matrix(delta[:3,:3]).magnitude())}
            repro['passed']=repro['translation_m']<1e-3 and repro['rotation_rad']<1e-4
            row={'pair':i,'methods':methods,'stages':stages,'reproduction':repro,'order':order};rows.append(row);stream.write(json.dumps(row,allow_nan=False)+'\n');stream.flush()
    native.write(target/'predictions.json',{'rows':rows})
    native.write(target/'prediction-lock.json',{'freeze_sha256':native.sha(target/'freeze.json'),'prediction_sha256':native.sha(target/'predictions.json'),'reference_scores_read':False})
    print('LOCKED99',flush=True)


def evaluate(rows,refs,independent=False):
    stats=measure if independent else base.metrics
    result={}
    for m in METHODS:
        position=[0.];angle=[0.];te=[];re=[];acc=np.eye(4)
        for row in rows:
            i=row['pair'];d=proper(row['methods'][m]['transform']);truth=np.linalg.inv(refs[i])@refs[i+1]
            te.append(np.linalg.norm(d[:3,3]-truth[:3,3]));re.append(np.rad2deg(Rotation.from_matrix(truth[:3,:3].T@d[:3,:3]).magnitude()))
            acc=acc@d;path=np.linalg.inv(refs[0])@refs[i+1]
            position.append(np.linalg.norm(acc[:3,3]-path[:3,3]));angle.append(np.rad2deg(Rotation.from_matrix(path[:3,:3].T@acc[:3,:3]).magnitude()))
        result[m]={'available':len(rows),'translation_m':stats(te),'rotation_deg':stats(re),'path_position_m':stats(position),'path_orientation_deg':stats(angle),'cpu_total_seconds':sum(r['methods'][m]['solver_cpu_seconds'] for r in rows),'wall_total_seconds':sum(r['methods'][m]['solver_wall_seconds'] for r in rows),'steps_total':sum(r['methods'][m]['steps'] for r in rows)}
    return result


def score():
    p=OUT/'run-001';lock=native.read(p/'prediction-lock.json');f=native.read(p/'freeze.json')
    assert native.sha(p/'predictions.json')==lock['prediction_sha256'] and native.sha(p/'freeze.json')==lock['freeze_sha256']
    for kind in ['source_pins','input_pins']:
        for name,h in f[kind].items():assert native.sha(ROOT/name)==h,name
    rows=native.read(p/'predictions.json')['rows'];refs={i:proper(np.loadtxt(REC/f'references/day/{i:06}.txt').reshape(4,4)) for i in range(100)}
    scores={'summary':evaluate(rows,refs),'native_gate_passed':all(r['reproduction']['passed'] for r in rows),'prediction_lock_sha256':native.sha(p/'prediction-lock.json')}
    native.write(p/'scores.json',scores);print(json.dumps(scores,indent=2))


if __name__=='__main__':{'capture':capture,'score':score}[sys.argv[1]]()
