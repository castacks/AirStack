"""Own-state feedback validation of the frozen RGB optimization-budget control."""
from pathlib import Path
import datetime
import json
import sys
import numpy as np
import torch
from scipy.spatial.transform import Rotation
import optimization_budget_control as base

native=base.native
ROOT=base.ROOT
OUT=ROOT/'results/optimization-feedback-control'
PROTOCOL=ROOT/'docs/research/optimization-feedback-protocol-2026-11-15.md'


def capture():
    torch.set_num_threads(1)
    source,saved,events,pins=base.load()
    qpath=base.OUT/'qualification-001/checks.json';q=native.read(qpath)
    assert q['passed'] and q['runner_sha256']==native.sha(base.__file__)
    target=OUT/'run-001';target.mkdir(parents=True,exist_ok=False)
    previous=native.read(base.OUT/'run-001/freeze.json')
    for p,h in previous['source_pins'].items():assert native.sha(ROOT/p)==h
    freeze={**previous,'at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'source_pins':{**previous['source_pins'],str(Path(__file__).relative_to(ROOT)):native.sha(__file__),
                       str(PROTOCOL.relative_to(ROOT)):native.sha(PROTOCOL)},
        'scope':'Own-state cached-measurement continuous feedback; no new frontend inference',
        'initial_state':events[(599,600)]['parent_pose_camera_map_xyzw']}
    native.write(target/'freeze.json',freeze)
    states={m:torch.tensor(freeze['initial_state'],dtype=torch.float32) for m in base.METHODS}
    rows=[];basis=np.eye(4);basis[:3,:3]=native.O
    with (target/'capture.jsonl').open('x') as stream:
        for pair in base.PAIRS:
            event=events[(pair,pair+1)];methods={};stage={}
            order=base.METHODS[(pair-599)%3:]+base.METHODS[:(pair-599)%3]
            for method in order:
                parent=states[method].clone()
                data=base.feedback.feedback_input(saved,event,pair,source['records'],parent)
                graph=native.Analytic_ReprojDisp_TwoFramePGO(data)
                first,receipt=base.timed_solve(graph)
                stage[method]={'first_committed_xyzw':first.tolist(),'first_receipt':receipt}
                current=first;combined=dict(receipt)
                if method!='native_rgb':
                    data2=base.feedback.feedback_input(saved,event,pair,source['records'],parent)
                    second=native.Analytic_ReprojDisp_TwoFramePGO(data2).double()
                    with torch.no_grad():second.pose2opt.copy_(native.pp.SE3(first.double()[None]))
                    if method=='extra_translation':second=base.component.FixedRotationGraph(second,native.pp.SE3(first.double()[None]))
                    current,extra=base.timed_solve(second)
                    stage[method]['second_receipt']=extra
                    combined={'losses':receipt['losses']+extra['losses'],
                        'steps':receipt['steps']+extra['steps'],
                        'solver_cpu_seconds':receipt['solver_cpu_seconds']+extra['solver_cpu_seconds'],
                        'solver_wall_seconds':receipt['solver_wall_seconds']+extra['solver_wall_seconds']}
                    if method=='extra_translation':assert current[3:].tolist()==first[3:].tolist()
                methods[method]=base.result(parent,current,combined);states[method]=current
            expected=basis@native.pose(event['derived_camera_delta_xyzw'])@basis.T
            error=np.linalg.solve(expected,np.asarray(methods['native_rgb']['transform']))
            reproduction={'translation_m':float(np.linalg.norm(error[:3,3])),
                          'rotation_rad':float(Rotation.from_matrix(error[:3,:3]).magnitude())}
            reproduction['passed']=reproduction['translation_m']<1e-3 and reproduction['rotation_rad']<1e-4
            row={'pair':pair,'methods':methods,'stages':stage,'reproduction':reproduction,'order':order}
            rows.append(row);stream.write(json.dumps(row,allow_nan=False)+'\n');stream.flush()
            if (pair-599)%100==0:print('FEEDBACK',pair,flush=True)
    native.write(target/'predictions.json',{'rows':rows})
    native.write(target/'prediction-lock.json',{'prediction_sha256':native.sha(target/'predictions.json'),
        'freeze_sha256':native.sha(target/'freeze.json'),'reference_scores_read':False})
    print('LOCKED',len(rows),flush=True)


def score():
    base.OUT=OUT
    base.score()


if __name__=='__main__':{'capture':capture,'score':score}[sys.argv[1]]()
