"""Deterministic supplied-event integration; synthetic model never controls rig."""
from pathlib import Path
from reliable_recorder import DurableRecorder,scan_bytes
from observation_adapters import Stamp,RecorderAdapter,StartupLifecycleAdapter,MavrosStateAdapter,VehicleGraphAdapter
from guard_reference import GuardReference


class Counter:
    def __init__(self):self.value=0
    def __call__(self):self.value+=1_000;return self.value


def run_scenario(path,events,*,config='a'*64,write=None,sync=None,close=True):
    clock=Counter();options=dict(monotonic=clock,wall=clock)
    if write is not None:options['write']=write
    if sync is not None:options['sync']=sync
    journal=DurableRecorder(path,'synthetic-run',config,**options)
    emitter=RecorderAdapter(journal,config)
    lifecycle=StartupLifecycleAdapter(emitter);mavros=MavrosStateAdapter(emitter)
    graph=VehicleGraphAdapter(emitter,{'mavros/state','mavros/extended_state','get_parameters','set_parameters'})
    reference=GuardReference();sequence={};outcomes=[]
    try:
        for event in events:
            kind=event['kind'];source=event.get('source','mavros' if kind=='connection' else 'graph' if kind=='graph' else 'guard' if kind=='tick' else 'startup')
            sequence[source]=sequence.get(source,0)+1
            stamp=Stamp(source,event.get('boot_id','synthetic-boot'),event.get('pid',1),event.get('clock_id','synthetic-shared-clock'),
                        event.get('sequence',sequence[source]),event['time_ns'],event.get('wall_ns',event['time_ns']))
            if kind=='connection':
                result=mavros.observe(event['connected'],stamp,{'boot_id':'synthetic-boot','clock_id':'synthetic-shared-clock','deadline_ns':10_000_000_000})
                reference.state(event['time_ns'],event['connected'])
            elif kind=='graph':result=graph.observe(event['interfaces'],stamp)
            elif kind=='tick':
                source='guard'
                stamp=Stamp(source,'synthetic-boot',2,'synthetic-shared-clock',sequence[source],event['time_ns'],event['time_ns'])
                if event.get('grounded'):reference.extended_state(event['time_ns'],True)
                reason=reference.tick(event['time_ns'],service_available=event.get('transaction',False),
                                      set_ok=event.get('transaction',False),typed_double_readback=event.get('transaction',False))
                result=emitter.emit('deadline_failure' if reason=='STARTUP_DEADLINE' else 'guard_initialized' if reason=='READY' else 'guard_status',stamp,{'reason':reason,'guard_instance':'synthetic-guard','synthetic_reference_only':True})
            else:
                result=lifecycle.observe(kind,stamp)
                if kind.startswith('recovery_') or kind in {'evidence_sealed'}:
                    reference.recovery_observed({'recovery_requested':'requested','evidence_sealed':'sealed','recovery_isolated':'isolated','recovery_approved':'approved'}[kind])
            outcomes.append(result)
    except BaseException:
        journal.interrupt_for_test()
        raise
    if close:journal.close()
    else:journal.interrupt_for_test()
    report=scan_bytes(Path(path).read_bytes(),expected_config=config,required_kinds=('process_start','application_complete'))
    kinds=[x['kind'] for x in outcomes]
    semantic=[]
    if 'vehicle_graph_ready' in kinds and ('application_complete' not in kinds or kinds.index('vehicle_graph_ready')<kinds.index('application_complete')):
        semantic.append('Graph readiness observed before application-complete; retain ordering discrepancy')
    return dict(journal=report,semantic_warnings=semantic,guard_reason=reference.reason,
                guard_initialized=reference.initialized,outcomes=outcomes)
