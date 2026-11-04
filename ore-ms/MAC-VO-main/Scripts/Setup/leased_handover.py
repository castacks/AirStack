"""Opt-in native handover with bounded store, shared lease and serial command credit."""
import argparse
import collections
import json
import multiprocessing as mp
from pathlib import Path
import time
from Scripts.Setup.shared_arbiter_replay import schedule,CAMERAS
from Scripts.Setup.transition_feedback import TransitionFeedback,requested_delivery
from Scripts.Setup.leased_native_worker import worker
from Scripts.Setup.leased_transport import BoundedConnection,EncodedLeaseArena,lease_identity
from Scripts.Setup.capacity_retained_inputs import CapacityInputBuffer
from Scripts.Setup.readiness_feedback import ReadinessFeedback
from Scripts.Setup.buffered_feedback import BufferedReadinessFeedback,FrozenRequestFeedback
from Scripts.Setup.retained_inputs import EncodedInputBuffer,missing_previous_source,register_retained


def run(args):
    rows=json.loads(args.manifest.read_text())['pairs']
    if len(rows)!=args.frames or not 4<=args.frames<=100:raise ValueError('bounded exact source window required')
    ordered=schedule(rows)
    if [i for _,_,i in ordered]!=[i for i in range(len(rows)) for _ in CAMERAS]:raise ValueError('nonoverlapping source groups required')
    args.output.mkdir(parents=True,exist_ok=False)
    fixture=json.loads(args.fixture.read_text())['decisions'] if args.fixture else None
    feedback=(FrozenRequestFeedback(rows,fixture) if fixture else
              BufferedReadinessFeedback(rows,args.mode) if args.readiness and args.catchup else
              ReadinessFeedback(rows,args.mode) if args.readiness else TransitionFeedback(rows,args.mode))
    retained=CapacityInputBuffer(args.dataset,rows,args.perturb,args.store_bytes);catchup_used=0
    arena=EncodedLeaseArena(args.arena_bytes)
    ctx=mp.get_context('spawn');processes={};pipes={};decisions=[];events={c:0 for c in CAMERAS}
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
                events[camera]+=1
                invocation=m['invocation_source'];delivery=m['delivery_group']
                if delivery!=source:raise ValueError('worker delivery group differs from current group')
                record=emit('observer_event',camera=camera,trigger_source=invocation,delivery_group=delivery,event=event)
                if hasattr(feedback,'ingest_group'):
                    ingress=feedback.ingest_group(camera,event,record['controller_monotonic_ns'],invocation,delivery)
                else:
                    if invocation is not None and invocation!=delivery:raise ValueError('historical input without buffered feedback')
                    ingress=feedback.ingest(camera,event,record['controller_monotonic_ns'],invocation)
                emit('ingress',camera=camera,event_sequence=event['sequence'],receipt_ns=record['controller_monotonic_ns'],**ingress)
                continue
            if kind!=expected or ('source' in m and m['source']!=source):raise RuntimeError('unexpected worker boundary: '+str(m))
            return emit(kind,camera=camera,**{k:v for k,v in m.items() if k!='kind'})
    options={'dataset':str(args.dataset),'manifest':str(args.manifest),'output':str(args.output),'frames':args.frames}
    try:
        for camera in CAMERAS:
            parent,child=ctx.Pipe();process=ctx.Process(target=worker,args=(camera,options,child));process.start();child.close()
            processes[camera]=process;pipes[camera]=BoundedConnection(parent)
        for camera in CAMERAS:receive(camera,'ready',0)
        current_decision=None
        for n,(sensor,camera,source) in enumerate(ordered):
            if n%2==0 and source>0:
                now=time.monotonic_ns();current_decision=feedback.plan(source-1,now)
                emit('preselection',**current_decision);decisions.append(current_decision)
            requested=source==0 or camera in current_decision['requests']
            retained.advance(camera,source)
            delivery=requested_delivery(camera,source,requested,args.perturb)
            delivered=delivery['delivered']
            payload=None;descriptor=None
            previous=missing_previous_source(source,feedback.last_source[camera],
                       current_decision is not None and current_decision['action']==camera.upper(),
                       delivered,catchup_used) if args.catchup else None
            if previous is not None:
                catchup_used+=1
                emit('buffer_query',camera=camera,source=previous,delivery_group=source,attempt=catchup_used)
                payload=retained.retrieve(camera,previous,source)
                emit('buffer_lookup',camera=camera,source=previous,delivery_group=source,retained=payload is not None)
                if payload is not None:
                    descriptor=arena.publish(payload,camera,previous,source)
                    emit('lease_publish' if descriptor else 'lease_rejected',camera=camera,source=previous,delivery_group=source,descriptor=descriptor)
                if descriptor is not None:
                    oldbinding=register_retained(feedback,camera,previous)
                    metadata={side:{k:v for k,v in image.items() if k!='data'} for side,image in payload['images'].items()}
                    emit('catchup_release',camera=camera,source=previous,delivery_group=source,binding=oldbinding,
                         policy_requested=True,delivered=True,encoded=metadata)
            binding=feedback.declare(camera,source) if delivered and source>0 else None
            emit('release' if delivered else 'transport_drop' if requested else 'skip_release',
                 camera=camera,source=source,delivery_group=source,sensor_time_ns=sensor,binding=binding,
                 policy_requested=requested,**delivery)
            payload=None  # no controller payload lease persists beyond publication
            emit('command_send',camera=camera,source=source,lease=lease_identity(descriptor))
            pipes[camera].send(('step',source,delivered,descriptor));completion=receive(camera,'completed',source)
            if completion['lease_ack']!=lease_identity(descriptor):raise ValueError('completion lease identity differs')
            if descriptor is not None:
                arena.acknowledge(completion['lease_ack'])
                emit('lease_acknowledged',**completion['lease_ack'])
            emit('command_acknowledged',camera=camera,source=source)
            if completion['executed']!=delivered or completion['buffered_count']!=int(descriptor is not None):
                raise ValueError('worker did not respect native input batch')
            if requested and not delivered:
                receipt=emit('transport_unavailable',camera=camera,source=source,synthetic=True)
                outcome=feedback.delivery_failure(camera,source,receipt['controller_monotonic_ns'])
                emit('transport_ingress',receipt_ns=receipt['controller_monotonic_ns'],**outcome)
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
        service={'store_capacity_bytes':args.store_bytes,'arena':arena.report(),
                 'allocated_encoded_ceiling_bytes':args.store_bytes+args.arena_bytes,
                 'control_channels':{c:p.report() for c,p in pipes.items()},
                 'step_ceiling_bytes':4096,'message_ceiling_bytes':65536,
                 'command_credit':1,'scope':'store+shared arena only; staging/control/decode/GPU/RSS excluded'}
        (args.output/'lease-service.json').write_text(json.dumps(service,indent=2)+'\n')
        arena.close()
        for pipe in pipes.values():pipe.close()
        journal.close()
        (args.output/'retained-inputs.json').write_text(json.dumps(retained.report(),indent=2)+'\n')
        summary={'status':status,'mode':args.mode,'source_frames':len(rows),'decisions':len(decisions),
                 'actions':dict(collections.Counter(d['action'] for d in decisions)),
                 'requests':{c:sum(c in d['requests'] for d in decisions) for c in CAMERAS},
                 'synthetic_input_outages':args.perturb,'readiness_enabled':args.readiness,
                 'readiness_sessions':feedback.session_history if args.readiness and not args.fixture else [],
                 'readiness_extra_used':feedback.extra_used if args.readiness and not args.fixture else 0,
                 'catchup_enabled':args.catchup,'catchup_queries':catchup_used,'frozen_request_fixture':bool(args.fixture),'scope':'requested synthetic input failures; serial offline native feedback, not natural faults'}
        (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for n in ['dataset','manifest','output']:p.add_argument('--'+n,type=Path,required=True)
    p.add_argument('--frames',type=int,required=True);p.add_argument('--mode',choices=['full_read_cache','bounded_probe','fixed_rgb','fixed_thermal'],required=True)
    p.add_argument('--timeout-seconds',type=float,default=300)
    p.add_argument('--perturb',action='store_true')
    p.add_argument('--readiness',action='store_true')
    p.add_argument('--catchup',action='store_true')
    p.add_argument('--fixture',type=Path)
    p.add_argument('--store-bytes',type=int,required=True)
    p.add_argument('--arena-bytes',type=int,default=2097152)
    run(p.parse_args())
