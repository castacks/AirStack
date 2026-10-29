"""Bounded EXP077 adapter: unchanged history rule, separate day middle condition."""
import argparse
from datetime import datetime
from zoneinfo import ZoneInfo
import json
from pathlib import Path
from Scripts.Setup.arbiter_history import sha,predict

CASE='day-middle'
DATASET='results/long-salon/prepared-001/datasets/middle'
SOURCE='data/tartandrive3/20251027/0_salon_so_day'


def prepare_history(root,output):
    path=root/'results/history-day-replication/run-001'
    audit=json.loads((path/'validation.json').read_text())
    if audit['status']!='PASS' or audit['timeline_sha256']!=sha(path/'timeline.jsonl'):
        raise ValueError('shared replay must pass independent audit first')
    files=[path/n for n in ('timeline.jsonl','protocol.json','summary.json','validation.json')]
    files += [path/'experts'/c/'arbiter-events/events.jsonl' for c in ('rgb','thermal')]
    files += [Path(__file__).with_name(n) for n in ('arbiter_history.py','shared_arbiter_replay.py','pair_arbiter_events.py')]
    output.mkdir(parents=True,exist_ok=False)
    protocol={'created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'base_commit':'fbdedfaa2',
        'confirmation_intervals':2,'cold_start':'abstain, no prior interval',
        'newer_than_target_intervals_used':False,'labels_used_for_prediction':False,'fitted':False,
        'rule':'unchanged EXP076 target+prior eligibility, symmetric length2, RGB tie preference',
        'scope':'separate-recording replication; historical VO outcomes known; not independent physical/heldout accuracy',
        'windows':[{'window':CASE,'run_relative':str(path.relative_to(root))}],
        'input_sha256':{str(p.relative_to(root)):sha(p) for p in files}}
    (output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    predict(root,output)


def label(root,output):
    # This stage is deliberately separate from saved label-free predictions.
    import numpy as np
    from Scripts.Setup.arbiter_pose_proxy import at,body_delta,transport,losses,extrinsic,summarize
    from Scripts.Setup.audit_td3_geometry import pose_matrix
    from Scripts.Setup.td3_reference import load_reference
    path=root/'results/history-day-replication/run-001';dataset=root/DATASET;source=root/SOURCE
    prediction=root/'results/history-day-replication/history-001/predictions.json'
    if not prediction.exists():raise ValueError('save causal predictions before loading labels')
    if sha(root/'data/calibration/yamaha.yaml')!='031c199619916bf1466a807d9ed91896d58b1428f0bdf74951dabc3c1b59e27d':
        raise ValueError('supplied calibration changed')
    files=[path/n for n in ('timeline.jsonl','protocol.json','summary.json','validation.json')]
    for c,name in (('rgb','multisense_left_rect_color_camera_info'),('thermal','thermal_left_processed_camera_info')):
        files += [path/'experts'/c/'arbiter-events/events.jsonl',path.parent/(c+'-preservation.json'),dataset/'sensors'/name/'data.yaml']
    files += [source/'super_odometry/odometry'/n for n in ('info.yaml','interp_data.txt','interp_timestamps.txt')]
    files += [root/'data/calibration/yamaha.yaml',Path(__file__),Path(__file__).with_name('arbiter_pose_proxy.py')]
    frozen={str(p.relative_to(root)):sha(p) for p in files}
    output.mkdir(parents=True,exist_ok=False)
    protocol={'created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'base_commit':'fbdedfaa2',
        'input_sha256':frozen,'physical_body_transform_verified':False,'independent_ground_truth':False,
        'causal_feature_eligible':False,'gate_fitted':False,'reference_used_for_offline_transport':True,
        'source_recording':SOURCE,'dataset':DATASET,'common_interval':'original RGB interval',
        'math':'unchanged EXP075 E*D*inv(E), offline endpoint bridges, separate translation meters/rotation degrees; bounded SO3 roundoff projection',
        'scope':'conditional nominal-body SLAM-proxy labels; not independent physical accuracy'}
    (output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    reference,_=load_reference(source);groups=json.loads((path/'protocol.json').read_text())['input_rows']
    events={c:[json.loads(s) for s in (path/'experts'/c/'arbiter-events/events.jsonl').read_text().splitlines()] for c in ('rgb','thermal')}
    es={c:extrinsic(root/'data/calibration/yamaha.yaml',dataset,c) for c in events};rows=[]
    for line in (path/'timeline.jsonl').read_text().splitlines():
        decision=json.loads(line)
        if decision['kind']!='decision':continue
        pair=decision['pair_ordinal'];common=[groups[i]['rgb_left']['time_ns'] for i in (pair,pair+1)]
        r0,r1=at(reference,common);truth=np.linalg.inv(r0)@r1
        row={'window':CASE,'pair_ordinal':pair,'source_decision_sequence':decision['sequence'],'action':decision['action'],
            'common_interval_ns':common,'reference_delta':truth.tolist(),'experts':{}}
        for c in events:
            if not decision['availability'][c]:row['experts'][c]=None;continue
            index=decision['evidence'][c]['motion_event_sequence'];event=events[c][index];own=[event['time0_ns'],event['time1_ns']]
            if index>=decision['observed_event_counts'][c] or own!=[groups[i][c+'_left']['time_ns'] for i in (pair,pair+1)] or not event['first_write_for_submission']:
                raise ValueError('invalid motion prefix/interval')
            a,b=at(reference,own);db=body_delta(pose_matrix(event['derived_camera_delta_xyzw']),es[c]);aligned=transport(db,r0,a,b,r1)
            row['experts'][c]={'sensor_interval_ns':own,'motion_event_sequence':index,'nominal_body_from_camera':es[c].tolist(),
                'own_body_delta':db.tolist(),'common_body_delta':aligned.tolist(),
                'own_loss':losses(db,np.linalg.inv(a)@b),'common_loss':losses(aligned,truth)}
        rows.append(row)
    if len(rows)!=99:raise ValueError('declared100 frame window required')
    (output/'pose-proxy.json').write_text(json.dumps({'scope':protocol['scope'],'rows':rows,'windows':summarize(rows)},indent=2,allow_nan=False)+'\n')
    if any(sha(root/p)!=h for p,h in frozen.items()):raise ValueError('source changed during labeling')
    print('Saved99 offline label rows after frozen causal prediction')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('stage',choices=('predict','label'))
    p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();(prepare_history if a.stage=='predict' else label)(a.root,a.output)
