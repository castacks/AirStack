"""Longer day control driver, reusing native graph, solver and fixed-R implementation."""
from pathlib import Path
import datetime,json,sys,copy,math
import numpy as np
import torch
from scipy.spatial.transform import Rotation
import optimization_budget_control as base
import optimization_day_transfer as scoring
from audit_optimization_budget_control import proper,pose,measure

ROOT=base.ROOT;native=base.native
OUT=ROOT/'results/long-day-strong-controls'
REC=native.REC
PROTOCOL=ROOT/'docs/research/long-day-strong-controls-protocol-2026-11-15.md'
METHODS=base.METHODS+['thermal_translation']
B=np.eye(4);B[:3,:3]=native.O


def load():
    source=native.read(REC/'inputs/day/source-manifest.json');assert source['sequence']=='_2021-08-13-16-31-10'
    saved={};events={}
    for cam,sensor in [('rgb','rgb'),('thermal','thr')]:
        folder=REC/f'captures/day/{cam}'
        cfg=native.read(folder/'effective-config.json')['Odometry']
        assert cfg['args']['mapping'] is False and cfg['motion']['type']=='StaticMotionModel'
        assert cfg['keyframe']['type']=='AllKeyframe' and cfg['optimizer']['args']['graph_type']=='disp'
        with np.load(folder/'tensor-map.npz') as z:saved[cam]={k:z[k] for k in z.files if not k.startswith('edge/point2match')}
        assert saved[cam]['frames//time_ns'].reshape(-1)[:600].tolist()==[r['cameras'][sensor]['left']['recorded_time_ns'] for r in source['records'][:600]]
        events[cam]={}
        for line in (folder/'arbiter-events/events.jsonl').read_text().splitlines():
            e=json.loads(line)
            if e['kind']=='motion_result_written' and e['first_write_for_submission']:
                key=(e['from_idx'],e['frame_idx']);assert key not in events[cam];events[cam][key]=e
    return source,saved,events


def capture():
    torch.set_num_threads(1);source,saved,events=load()
    old=native.read(ROOT/'results/component-constrained-native/run-001/freeze.json')
    pins=old['input_pins']['day']
    prior=native.read(ROOT/'results/optimization-feedback-control/run-001/freeze.json')
    for kind,values in [('source',prior['source_pins']),('input',pins)]:
        for p,h in values.items():assert native.sha(ROOT/p)==h,(kind,p)
    configs={str((REC/f'captures/day/{c}/effective-config.json').relative_to(ROOT)):native.sha(REC/f'captures/day/{c}/effective-config.json') for c in ['rgb','thermal']}
    refs={str((ROOT/f'results/fixed-rotation-continuation/recovery/references/day/{i:06}.txt').relative_to(ROOT)):native.sha(ROOT/f'results/fixed-rotation-continuation/recovery/references/day/{i:06}.txt') for i in range(600)}
    target=OUT/'run-001';target.mkdir(parents=True,exist_ok=False)
    freeze={'at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'source_pins':{**prior['source_pins'],'tools/optimization_day_transfer.py':native.sha(scoring.__file__),str(Path(__file__).relative_to(ROOT)):native.sha(__file__),str(PROTOCOL.relative_to(ROOT)):native.sha(PROTOCOL)},'input_pins':{**pins,**configs},'reference_pins':refs,'initial_state':events['rgb'][(0,1)]['parent_pose_camera_map_xyzw'],'pairs':[0,598],'methods':METHODS,'thermal_precision':.1,'scope':'Exposed first600frames originalday; own-state cached RGB/thermal controls'}
    native.write(target/'freeze.json',freeze);rows=[];states={m:torch.tensor(freeze['initial_state'],dtype=torch.float32) for m in METHODS}
    extrinsic=native.geometry(native.read(REC/'inputs/day/calibration.json'))['thermal']
    with (target/'capture.jsonl').open('x') as stream:
        for i in range(599):
            e=events['rgb'][(i,i+1)];assert not e['original_need_interp'];methods={};stages={}
            packet,binding=native.bind_camera(saved['thermal'],events['thermal'][(i,i+1)],i,source['records'],'thr',extrinsic)
            order=METHODS[i%4:]+METHODS[:i%4]
            for m in order:
                parent=states[m].clone()
                def data():return base.feedback.feedback_input(saved['rgb'],e,i,source['records'],parent)
                first,t1=base.timed_solve(native.Analytic_ReprojDisp_TwoFramePGO(data()));current=first;receipts=[t1]
                if m!='native_rgb':
                    g=(native.NativeThermalGraph(data(),packet,native.pose(parent.numpy()),thermal_precision=.1) if m=='thermal_translation' else native.Analytic_ReprojDisp_TwoFramePGO(data())).double()
                    with torch.no_grad():g.pose2opt.copy_(native.pp.SE3(first.double()[None]))
                    if m in ['extra_translation','thermal_translation']:g=base.component.FixedRotationGraph(g,native.pp.SE3(first.double()[None]))
                    current,t2=base.timed_solve(g);receipts.append(t2)
                total={'losses':sum([r['losses'] for r in receipts],[]),**{k:sum(r[k] for r in receipts) for k in ['steps','solver_cpu_seconds','solver_wall_seconds']}}
                methods[m]=base.result(parent,current,total);stages[m]={'first':first.tolist(),'receipts':receipts};states[m]=current
            delta=np.linalg.solve(B@native.pose(e['derived_camera_delta_xyzw'])@B.T,np.asarray(methods['native_rgb']['transform']))
            repro={'translation_m':float(np.linalg.norm(delta[:3,3])),'rotation_rad':float(Rotation.from_matrix(delta[:3,:3]).magnitude())};repro['passed']=repro['translation_m']<1e-3 and repro['rotation_rad']<1e-4
            row={'pair':i,'methods':methods,'stages':stages,'order':order,'reproduction':repro,'thermal_binding':binding}
            rows.append(row);stream.write(json.dumps(row,allow_nan=False)+'\n');stream.flush()
            if i%100==0:print('LONG_DAY',i,flush=True)
    native.write(target/'predictions.json',{'rows':rows})
    native.write(target/'prediction-lock.json',{'freeze_sha256':native.sha(target/'freeze.json'),'prediction_sha256':native.sha(target/'predictions.json'),'reference_scores_read':False})
    print('LOCKED599',flush=True)


def score():
    p=OUT/'run-001';f=native.read(p/'freeze.json');lock=native.read(p/'prediction-lock.json')
    assert native.sha(p/'freeze.json')==lock['freeze_sha256'] and native.sha(p/'predictions.json')==lock['prediction_sha256']
    for kind in ['source_pins','input_pins','reference_pins']:
        for name,h in f[kind].items():assert native.sha(ROOT/name)==h,name
    rows=native.read(p/'predictions.json')['rows'];refs={i:proper(np.loadtxt(ROOT/f'results/fixed-rotation-continuation/recovery/references/day/{i:06}.txt').reshape(4,4)) for i in range(600)}
    scoring.METHODS=METHODS
    scores={'summary':scoring.evaluate(rows,refs),'native_gate_passed':all(r['reproduction']['passed'] for r in rows),'prediction_lock_sha256':native.sha(p/'prediction-lock.json'),'scope':f['scope']}
    native.write(p/'scores.json',scores);print(json.dumps(scores,indent=2))


if __name__=='__main__':{'capture':capture,'score':score}[sys.argv[1]]()
