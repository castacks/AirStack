"""Offline, serial sensor-ordered dual-expert replay with causal advisory decisions.

Spawned workers use the unchanged v3 run function. Only their sequence iterator
and copied recorder emissions are adapted, locally inside each worker process.
No wall-clock pacing, pose fusion or feedback into either estimator is implied.
"""
import argparse
from collections import Counter
import hashlib
import json
import multiprocessing as mp
from pathlib import Path
import time
import traceback
from types import SimpleNamespace

from Scripts.Setup.pair_arbiter_events import camera_record, index_events, integer

CAMERAS = ('rgb', 'thermal')
POSE_FRAME = 'rectified left camera NED; identity body/sensor transform'


def advisory(rows, pair, events):
    """Evaluate a supplied observed prefix only. RGB preference is not accuracy."""
    availability, evidence = {}, {}
    for camera in CAMERAS:
        first, last = [integer(rows[i][camera+'_left']['time_ns']) for i in (pair, pair+1)]
        health, motions, _ = index_events(events[camera])
        record = camera_record({'coordinate_frame': POSE_FRAME}, health, motions, first, last)
        ok = record['health_status'] == 'COMPLETE' and record['motion_available_camera_local']
        availability[camera] = ok
        evidence[camera] = {k: record[k] for k in ('sensor_interval_ns', 'health_status', 'health',
            'motion_status', 'motion_available_camera_local', 'body_transform_verified')}
        evidence[camera]['health_event_sequence'] = health.get((first,last), {}).get('sequence')
        evidence[camera]['motion_event_sequence'] = record['motion']['sequence'] if record['motion'] else None
    action = 'RGB' if availability['rgb'] else 'THERMAL' if availability['thermal'] else 'ABSTAIN'
    return {'pair_ordinal': pair, 'action': action, 'availability': availability, 'evidence': evidence,
        'reason': 'both_available_fixed_rgb_preference' if all(availability.values()) else
                  'only_rgb_available' if availability['rgb'] else 'only_thermal_available' if availability['thermal'] else 'no_eligible_result',
        'observed_event_counts': {c: len(events[c]) for c in CAMERAS},
        'camera_local_advice_only': True, 'pose_gate_training_ready': False}


def schedule(rows):
    entries = [(integer(row[c+'_left']['time_ns']), c, i) for i,row in enumerate(rows) for c in CAMERAS]
    for c in CAMERAS:
        times = [integer(r[c+'_left']['time_ns']) for r in rows]
        if any(b <= a for a,b in zip(times,times[1:])):
            raise ValueError('nonincreasing sensor clock')
    return sorted(entries)


def worker(camera, options, pipe):
    """All estimator changes are process-local observation/scheduling adapters."""
    import contextlib
    with (Path(options['output'])/(camera+'-worker.log')).open('w', buffering=1) as log, contextlib.redirect_stdout(log), contextlib.redirect_stderr(log):
        try:
            import torch
            from Scripts.Setup import run_td3_execution_v3 as runner
            from Scripts.Setup import arbiter_event_recorder as recorder_module
            original_sequence = runner.TD3ManifestSequence
            original_recorder = recorder_module.ArbiterEventRecorder

            class GatedSequence(original_sequence):
                def __iter__(self):
                    for i in range(len(self)):
                        pipe.send({'kind':'ready', 'frame':i})
                        if pipe.recv() != ('release',i):
                            raise RuntimeError('unexpected frame release')
                        yield self[i]
                        torch.cuda.synchronize()
                        pipe.send({'kind':'completed', 'frame':i})
                    pipe.send({'kind':'drain_ready'})
                    if pipe.recv() != ('drain',):
                        raise RuntimeError('unexpected drain release')

            class StreamingRecorder(original_recorder):
                def _emit(self, kind, **fields):
                    super()._emit(kind, **fields)
                    pipe.send({'kind':'observer_event', 'event':json.loads(self.events[-1])})

            runner.TD3ManifestSequence = GatedSequence
            recorder_module.ArbiterEventRecorder = StreamingRecorder
            args = SimpleNamespace(**options)
            args.dataset, args.manifest = Path(args.dataset), Path(args.manifest)
            args.output = Path(args.output)/'experts'
            runner.run(args,camera)
            pipe.send({'kind':'finished'})
        except BaseException:
            pipe.send({'kind':'worker_error', 'traceback':traceback.format_exc()})
        finally:
            pipe.close()


def run(args):
    if not args.acknowledge_provisional_geometry:
        raise ValueError('explicit provisional geometry acknowledgement required')
    if not 2 <= args.frames <= 100 or args.start < 0:
        raise ValueError('bounded replay requires 2–100 frames and nonnegative start')
    rows = json.loads(args.manifest.read_text())['pairs'][args.start:args.start+args.frames]
    if len(rows) != args.frames:
        raise ValueError('manifest shorter than requested window')
    ordered = schedule(rows)
    # The lag policy requires noninterleaved groups, while permitting either
    # camera order within a group. Refuse a manifest that violates that contract.
    if [i for _,_,i in ordered] != [i for i in range(args.frames) for _ in CAMERAS]:
        raise ValueError('overlapping groups require a different deadline policy')
    args.output.mkdir(parents=True,exist_ok=False)
    options = dict(dataset=str(args.dataset),manifest=str(args.manifest),output=str(args.output),
        frames=args.frames,start=args.start,acknowledge_provisional_geometry=True,
        record_uncertainty=True,record_arbiter_events=True)
    protocol = {'schema_version':1,'mode':'offline_serial_sensor_ordered', 'input_rows':rows,
        'deadline_policy':'pair i after group i+2; last pair at final frame boundary before optimizer drain',
        'priority':['rgb','thermal'],'tie_break':'fixed preference, not quality ranking',
        'wall_clock_paced':False,'group_lag':1,'sensor_clock_is_monotonic_clock':False,
        'policy_fitted':False,'body_transform_verified':False,'estimator_feedback':False,
        'source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in
            (Path(__file__),Path(__file__).with_name('pair_arbiter_events.py'),
             Path(__file__).with_name('run_td3_execution_v3.py'),Path(__file__).with_name('arbiter_event_recorder.py'))}}
    (args.output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    ctx = mp.get_context('spawn')
    processes, pipes, events, decisions = {}, {}, {c:[] for c in CAMERAS}, []
    journal = (args.output/'timeline.jsonl').open('w',buffering=1)
    sequence = 0
    def emit(kind, **values):
        nonlocal sequence
        record = {'sequence':sequence,'kind':kind,'controller_monotonic_ns':time.monotonic_ns(),**values}
        journal.write(json.dumps(record,allow_nan=False)+'\n');sequence += 1
        return record
    def receive(camera, expected, frame=None):
        while True:
            if not pipes[camera].poll(args.timeout_seconds):
                raise TimeoutError(f'{camera}: waiting for {expected}')
            message = pipes[camera].recv()
            kind = message['kind']
            if kind == 'worker_error':
                raise RuntimeError(message['traceback'])
            if kind == 'observer_event':
                event = message['event']
                if event['modality'] != camera or event['sequence'] != len(events[camera]):
                    raise ValueError('invalid streamed event identity')
                events[camera].append(event)
                emit('observer_event',camera=camera,event=event)
                continue
            if kind != expected or (frame is not None and message.get('frame') != frame):
                raise RuntimeError(f'{camera}: expected {expected}/{frame}, got {message}')
            return emit(kind,camera=camera,**({'frame':frame} if frame is not None else {}))
    def decide(pair, boundary):
        decision = advisory(rows,pair,events)
        decision['boundary'] = boundary
        record = emit('decision',**decision)
        decisions.append(record)
    status = 'FAILED'
    try:
        for camera in CAMERAS:
            parent,child = ctx.Pipe()
            process = ctx.Process(target=worker,args=(camera,options,child))
            process.start();child.close()
            processes[camera],pipes[camera] = process,parent
        for camera in CAMERAS:
            receive(camera,'ready',0)
        for n,(sensor,camera,frame) in enumerate(ordered):
            emit('release',camera=camera,frame=frame,sensor_time_ns=sensor)
            pipes[camera].send(('release',frame))
            receive(camera,'completed',frame)
            if n % 2 == 1 and frame >= 2:
                decide(frame-2,{'kind':'group_complete','group':frame})
            receive(camera,'ready',frame+1) if frame+1 < args.frames else receive(camera,'drain_ready')
        decide(args.frames-2,{'kind':'final_frame_before_drain','group':args.frames-1})
        for camera in CAMERAS:
            emit('drain_release',camera=camera)
            pipes[camera].send(('drain',))
            receive(camera,'finished')
        for camera,process in processes.items():
            process.join(args.timeout_seconds)
            if process.exitcode != 0:
                raise RuntimeError(f'{camera}: process exit {process.exitcode}')
        status = 'PASS_REPLAY_ONLY'
    except BaseException as error:
        emit('controller_error',error_type=type(error).__name__,error=str(error))
        raise
    finally:
        for process in processes.values():
            if process.is_alive():
                process.terminate();process.join(10)
        for pipe in pipes.values():
            pipe.close()
        journal.close()
        summary = {'status':status,'frames_per_expert':args.frames,'decisions':len(decisions),
            'actions':dict(Counter(d['action'] for d in decisions)),
            'streamed_events':{c:len(events[c]) for c in CAMERAS},
            'scope':'observed offline availability advice; no pose accuracy or realtime latency claim',
            'timeline_sha256':hashlib.sha256((args.output/'timeline.jsonl').read_bytes()).hexdigest()}
        (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('dataset','manifest','output'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--frames',type=int,default=20)
    p.add_argument('--start',type=int,default=0)
    p.add_argument('--timeout-seconds',type=float,default=300)
    p.add_argument('--acknowledge-provisional-geometry',action='store_true')
    run(p.parse_args())
