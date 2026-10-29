"""Two-interval causal confirmation. Prediction path uses no pose-loss labels."""
import argparse
import hashlib
import json
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
from Scripts.Setup.shared_arbiter_replay import advisory

CAMERAS=('rgb','thermal')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def history_advisory(rows,pair,events):
    if type(pair) is not int or not 0<=pair<len(rows)-1:
        raise ValueError('valid target pair ordinal required')
    current=advisory(rows,pair,events)
    previous=advisory(rows,pair-1,events) if pair else None
    eligible={c:bool(current['availability'][c] and previous is not None and previous['availability'][c]) for c in CAMERAS}
    action='RGB' if eligible['rgb'] else 'THERMAL' if eligible['thermal'] else 'ABSTAIN'
    return {'pair_ordinal':pair,'action':action,'baseline_action':current['action'],
        'base_availability':current['availability'],'confirmed_availability':eligible,
        'current_evidence':current['evidence'],
        'previous_pair_ordinal':pair-1 if pair else None,
        'previous_availability':previous['availability'] if previous else {c:False for c in CAMERAS},
        'previous_evidence':previous['evidence'] if previous else None,
        'observed_event_counts':current['observed_event_counts'],
        'reason':'cold_start' if previous is None else 'both_confirmed_fixed_rgb_preference' if all(eligible.values()) else
            'only_rgb_confirmed' if eligible['rgb'] else 'only_thermal_confirmed' if eligible['thermal'] else 'no_confirmed_expert',
        'confirmation_intervals':2,'reference_or_loss_used':False,'pose_gate_training_ready':False}


def prepare(root,output):
    output.mkdir(parents=True,exist_ok=False)
    paths=[];windows=[]
    for window,directory in (('early','shared-arbiter-events'),('middle','arbiter-support-collapse')):
        path=root/'results'/directory/'run-001'
        paths += [path/n for n in ('timeline.jsonl','protocol.json','summary.json','validation.json')]
        paths += [path/'experts'/c/'arbiter-events/events.jsonl' for c in CAMERAS]
        windows.append({'window':window,'run_relative':str(path.relative_to(root))})
    base=Path(__file__).parent
    paths += [Path(__file__),base/'shared_arbiter_replay.py',base/'pair_arbiter_events.py']
    protocol={'created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'base_commit':'19ef530c8',
        'rule':'target and immediately preceding interval must both be eligible by original deadline; symmetric length2 confirmation; fixed RGB tie preference',
        'confirmation_intervals':2,'cold_start':'abstain, no prior interval',
        'newer_than_target_intervals_used':False,'labels_used_for_prediction':False,'fitted':False,
        'scope':'retrospective causal-prefix rule replay on already examined windows; no realtime/accuracy/generalization claim',
        'windows':windows,'input_sha256':{str(p.relative_to(root)):sha(p) for p in paths}}
    (output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')


def predict(root,output):
    if (output/'predictions.json').exists():raise FileExistsError('predictions already exist')
    protocol=json.loads((output/'protocol.json').read_text())
    if protocol['confirmation_intervals']!=2 or protocol['labels_used_for_prediction'] or protocol['newer_than_target_intervals_used']:
        raise ValueError('incorrect frozen history protocol')
    for rel,digest in protocol['input_sha256'].items():
        if sha(root/rel)!=digest:raise ValueError('frozen source changed: '+rel)
    predictions=[]
    for window in protocol['windows']:
        path=root/window['run_relative'];audit=json.loads((path/'validation.json').read_text())
        if audit['status']!='PASS' or audit['timeline_sha256']!=sha(path/'timeline.jsonl'):
            raise ValueError('unverified source timeline')
        groups=json.loads((path/'protocol.json').read_text())['input_rows'];events={c:[] for c in CAMERAS}
        for line in (path/'timeline.jsonl').read_text().splitlines():
            record=json.loads(line)
            if record['kind']=='observer_event':
                camera=record['camera'];event=record['event']
                if event['sequence']!=len(events[camera]):raise ValueError('nonsequential source event')
                events[camera].append(event)
            elif record['kind']=='decision':
                original=advisory(groups,record['pair_ordinal'],events)
                if any(original[k]!=record[k] for k in ('action','availability','evidence','observed_event_counts')):
                    raise ValueError('source decision/prefix mismatch')
                decision=history_advisory(groups,record['pair_ordinal'],events)
                decision.update(window=window['window'],source_decision_sequence=record['sequence'],
                    source_deadline_monotonic_ns=record['controller_monotonic_ns'],boundary=record['boundary'])
                predictions.append(decision)
    for rel,digest in protocol['input_sha256'].items():
        if sha(root/rel)!=digest:raise ValueError('source changed during prediction')
    (output/'predictions.json').write_text(json.dumps({'protocol_sha256':sha(output/'protocol.json'),'decisions':predictions},indent=2)+'\n')
    print(json.dumps({'predictions':len(predictions),'labels_loaded':False}))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=('prepare','predict'));p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();(prepare if a.action=='prepare' else predict)(a.root,a.output)
