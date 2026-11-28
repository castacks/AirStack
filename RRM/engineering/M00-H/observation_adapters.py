"""Pure observation adapters. No ROS import, subscriptions, commands or startup.

Future ROS bindings must convert actual messages to these declared observations;
producer source/clock identities are supplied, not authenticated by these classes.
"""
from dataclasses import dataclass
from reliable_recorder import IDENTITY,JournalError


@dataclass(frozen=True)
class Stamp:
    source:str
    boot_id:str
    pid:int
    clock_id:str
    sequence:int
    monotonic_ns:int
    wall_ns:int

    def checked(self):
        for x in [self.source,self.boot_id,self.clock_id]:
            if not isinstance(x,str) or not IDENTITY.fullmatch(x):raise JournalError('Invalid source identity')
        for x in [self.pid,self.sequence,self.monotonic_ns,self.wall_ns]:
            if type(x) is not int or x<0:raise JournalError('Invalid observation timestamp/sequence')
        return self


class RecorderAdapter:
    def __init__(self,journal,configuration_sha256):
        if journal.config!=configuration_sha256:raise JournalError('Configuration mismatch')
        self.journal=journal

    def emit(self,kind,stamp,data):
        stamp.checked()
        return self.journal.emit(kind,source=stamp.source,boot_id=stamp.boot_id,pid=stamp.pid,
                                 clock_id=stamp.clock_id,producer_sequence=stamp.sequence,
                                 monotonic_ns=stamp.monotonic_ns,wall_ns=stamp.wall_ns,data=data)

    @property
    def health(self):return 'DEGRADED' if self.journal.failed else 'CLOSED' if self.journal.closed else 'LOCAL_WRITER_READY'


class StartupLifecycleAdapter:
    def __init__(self,emitter):
        self.emitter=emitter;self.started=False;self.app_complete=False;self.failed=False;self.recovery_step=0

    def observe(self,event,stamp,**data):
        if event=='process_start':
            if self.started:raise JournalError('Duplicate startup event')
        elif not self.started:raise JournalError('Missing process-start observation')
        if event=='application_complete' and self.app_complete:raise JournalError('Duplicate application-complete event')
        recovery=['recovery_requested','evidence_sealed','recovery_isolated','recovery_approved']
        if event in recovery:
            if not self.failed or self.recovery_step>=4 or event!=recovery[self.recovery_step]:raise JournalError('Invalid recovery event ordering')
        elif event not in {'process_start','application_complete','deadline_failure','startup_failure'}:raise JournalError('Unknown startup lifecycle event')
        result=self.emitter.emit(event,stamp,{'observational':True,**data})
        if event=='process_start':self.started=True
        if event=='application_complete':self.app_complete=True
        if event in {'deadline_failure','startup_failure'}:self.failed=True
        if event in recovery:self.recovery_step+=1
        return result


class MavrosStateAdapter:
    def __init__(self,emitter):self.emitter=emitter;self.connected=None

    def observe(self,connected,stamp,deadline=None):
        if type(connected) is not bool:raise JournalError('Connection state must be Boolean')
        classification='NOT_APPLICABLE'
        if connected and deadline is not None:
            expected={'boot_id','clock_id','deadline_ns'}
            if not isinstance(deadline,dict) or set(deadline)!=expected or type(deadline['deadline_ns']) is not int:raise JournalError('Malformed deadline provenance')
            if deadline['boot_id']!=stamp.boot_id or deadline['clock_id']!=stamp.clock_id:
                classification='UNMEASURABLE_CLOCK_MAPPING_REQUIRED'
            else:classification='LATE' if stamp.monotonic_ns>=deadline['deadline_ns'] else 'BEFORE_DEADLINE'
        kind='mavros_connection' if connected else 'mavros_disconnection'
        result=self.emitter.emit(kind,stamp,{'connected':connected,'transition':self.connected is None or connected!=self.connected,
                                            'late_classification':classification,'observational':True})
        self.connected=connected
        return result


class VehicleGraphAdapter:
    def __init__(self,emitter,required_interfaces):
        if not required_interfaces:raise JournalError('Required interfaces must be explicit')
        self.emitter=emitter;self.required=frozenset(required_interfaces);self.ready=False

    def observe(self,interfaces,stamp):
        if not isinstance(interfaces,(set,frozenset,list)) or not all(isinstance(x,str) for x in interfaces):raise JournalError('Invalid graph observation')
        available=set(interfaces);ready=self.required<=available
        kind='vehicle_graph_ready' if ready and not self.ready else 'graph_readiness_lost' if self.ready and not ready else 'graph_discovery'
        result=self.emitter.emit(kind,stamp,{'required':sorted(self.required),'available':sorted(available),
                                            'missing':sorted(self.required-available),'ready':ready,
                                            'observation_only_not_guard_or_safety_readiness':True})
        self.ready=ready
        return result
