"""Serial native sparse feedback using only its own received event prefix/clock."""
import argparse
import collections
import json
import multiprocessing as mp
from pathlib import Path
import time
from Scripts.Setup.shared_arbiter_replay import schedule,CAMERAS
from Scripts.Setup.sparse_feedback import SparseFeedback
from Scripts.Setup.sparse_native_worker import worker


def run(args):
    rows=json.loads(args.manifest.read_text())['pairs']
    if len(rows)!=args.frames or not 4<=args.frames<=100:raise ValueError('bounded exact source window required')
    ordered=schedule(rows)
    if [i for _,_,i in ordered]!=[i for i in range(len(rows)) for _ in CAMERAS]:raise ValueError('nonoverlapping source groups required')
    args.output.mkdir(parents=True,exist_ok=False)
    feedback=SparseFeedback(rows,args.mode);ctx=mp.get_context('spawn');processes={};pipes={};decisions=[];events={c:0 for c in CAMERAS}
    journal=(args.output/'timeline.jsonl').open('x',buffering=1);sequence=0;status='FAILED'
    def emit(kind,**values):
        nonlocal sequence
        record={'sequence':sequence,'kind':kind,'controller_monotonic_ns':time.monotonic_ns(),**values}
        journal.write(json.dumps(record,allow_nan=False)+'\n');sequence+=1;return record
    def receive(camera,expected,source):
        while True:
            if not pipes[camera].poll(args.timeout_seconds):raise TimeoutError(camera+':'+expected)
            m=pipes[camera].recv();kind=m['kind']
            if kind=='worker_error':raise RuntimeError(m['traceback'])
            if kind=='observer_event':
                event=m['event']
                if event['modality']!=camera or event['sequence']!=events[camera]:raise ValueError('invalid event identity')
                events[camera]+=1;record=emit('observer_event',camera=camera,trigger_source=source,event=event)
                ingress=feedback.ingest(camera,event,record['controller_monotonic_ns'],source)
                emit('ingress',camera=camera,event_sequence=event['sequence'],receipt_ns=record['controller_monotonic_ns'],**ingress)
                continue
            if kind!=expected or ('source' in m and m['source']!=source):raise RuntimeError('unexpected worker boundary: '+str(m))
            return emit(kind,camera=camera,**{k:v for k,v in m.items() if k!='kind'})
    options={'dataset':str(args.dataset),'manifest':str(args.manifest),'output':str(args.output),'frames':args.frames}
    try:
        for camera in CAMERAS:
            parent,child=ctx.Pipe();process=ctx.Process(target=worker,args=(camera,options,child));process.start();child.close()
            processes[camera]=process;pipes[camera]=parent
        for camera in CAMERAS:receive(camera,'ready',0)
        current_decision=None
        for n,(sensor,camera,source) in enumerate(ordered):
            if n%2==0 and source>0:
                now=time.monotonic_ns();current_decision=feedback.plan(source-1,now)
                emit('preselection',**current_decision);decisions.append(current_decision)
            requested=source==0 or camera in current_decision['requests']
            binding=feedback.declare(camera,source) if requested and source>0 else None
            emit('release' if requested else 'skip_release',camera=camera,source=source,sensor_time_ns=sensor,binding=binding)
            pipes[camera].send(('step',source,requested));receive(camera,'completed',source)
            if n%2==1:emit('group_complete',source=source)
            if source+1<len(rows):receive(camera,'ready',source+1)
            else:receive(camera,'drain_ready',source)
        for camera in CAMERAS:
            emit('drain_release',camera=camera);pipes[camera].send(('drain',));receive(camera,'finished',None)
        for camera,process in processes.items():
            process.join(args.timeout_seconds)
            if process.exitcode!=0:raise RuntimeError(camera+' worker nonzero exit')
        status='PASS_OFFLINE_NATIVE_FEEDBACK'
    except BaseException as error:
        emit('controller_error',error_type=type(error).__name__,error=str(error));raise
    finally:
        for p in processes.values():
            if p.is_alive():p.terminate();p.join(10)
        for pipe in pipes.values():pipe.close()
        journal.close()
        summary={'status':status,'mode':args.mode,'source_frames':len(rows),'decisions':len(decisions),
                 'actions':dict(collections.Counter(d['action'] for d in decisions)),
                 'requests':{c:sum(c in d['requests'] for d in decisions) for c in CAMERAS},
                 'scope':'own-prefix own-clock feedback; offline serial, not live queues or physical accuracy'}
        (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for n in ['dataset','manifest','output']:p.add_argument('--'+n,type=Path,required=True)
    p.add_argument('--frames',type=int,required=True);p.add_argument('--mode',choices=['full_read_cache','bounded_probe'],required=True)
    p.add_argument('--timeout-seconds',type=float,default=300)
    run(p.parse_args())
