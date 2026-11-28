"""Observational JSONL recorder library for supplied events; no control interfaces.

No automatic collection, ROS subscription, simulator log parser or process control.
Callers provide producer clocks/provenance; receipt clocks never replace them.
File-like sinks are injected; tests use memory/failing sinks. Flush is not fsync.
"""
import hashlib
import json
import time
from common import canonical

KINDS = {'recorder_init','recorder_ready','recorder_failure','data_loss','recorder_end',
         'guard_constructed','deadline_failure','mavros_connection','application_complete',
         'vehicle_graph_ready','guard_initialized','guard_status','first_play','px4_spawn',
         'recovery_requested','evidence_sealed','recovery_isolated','recovery_approved'}
REQUIRED = {'schema_version','run_id','sequence','event_id','kind','source','clock_id',
            'producer_boot_id','producer_pid','producer_sequence','producer_monotonic_ns',
            'producer_wall_ns','receipt_monotonic_ns','receipt_wall_ns','configuration_sha256',
            'evidence_class','data','previous_sha256','sha256'}


class RecorderError(RuntimeError):
    pass


class Recorder:
    def __init__(self, sink, run_id, configuration_sha256, *, monotonic=time.monotonic_ns,
                 wall=time.time_ns, boot_id='UNATTESTED_RECORDER_BOOT', pid=0):
        if not isinstance(run_id,str) or not run_id or not isinstance(configuration_sha256,str) or len(configuration_sha256)!=64 or any(x not in '0123456789abcdef' for x in configuration_sha256):
            raise RecorderError('Invalid run/configuration identity')
        self.sink,self.run_id,self.config=sink,run_id,configuration_sha256
        self.monotonic,self.wall=monotonic,wall
        self.boot_id,self.pid=boot_id,pid
        self.sequence,self.previous,self.failed,self.closed=0,'0'*64,False,False
        self._health('recorder_init',{'state':'INITIALIZING'})
        self._health('recorder_ready',{'state':'READY_OFFLINE_WRITER_ONLY'})

    def _health(self,kind,data):
        return self.emit(kind,source='recorder',clock_id='recorder_monotonic',
                         boot_id=self.boot_id,pid=self.pid,producer_sequence=self.sequence+1,
                         monotonic_ns=self.monotonic(),wall_ns=self.wall(),data=data)

    def emit(self,kind,*,source,clock_id,boot_id,pid,producer_sequence,monotonic_ns,wall_ns,data):
        if self.closed or self.failed:
            raise RecorderError('Recorder closed or failed; no silent resume')
        if kind not in KINDS or not isinstance(data,dict):
            raise RecorderError('Unknown event kind or malformed event data')
        for field in [source,clock_id,boot_id]:
            if not isinstance(field,str) or not field:raise RecorderError('Missing producer provenance')
        for field in [pid,producer_sequence,monotonic_ns,wall_ns]:
            if type(field) is not int or field<0:raise RecorderError('Invalid integer timestamp/identity')
        record=dict(schema_version=1,run_id=self.run_id,sequence=self.sequence+1,
                    event_id=f'{self.run_id}:{self.sequence+1}',kind=kind,source=source,
                    clock_id=clock_id,producer_boot_id=boot_id,producer_pid=pid,
                    producer_sequence=producer_sequence,producer_monotonic_ns=monotonic_ns,
                    producer_wall_ns=wall_ns,receipt_monotonic_ns=self.monotonic(),
                    receipt_wall_ns=self.wall(),configuration_sha256=self.config,
                    evidence_class='B_RRM_ENGINEERING_DIAGNOSTIC',data=data,previous_sha256=self.previous)
        record['sha256']=hashlib.sha256(canonical(record)).hexdigest()
        line=canonical(record).decode()+'\n'
        try:
            written=self.sink.write(line)
            if written!=len(line):raise OSError('Short write / data loss')
            self.sink.flush()
        except (OSError,ValueError) as exc:
            self.failed=True
            raise RecorderError('Recorder I/O failure; stream incomplete; independent supervisor must retain failure evidence') from exc
        self.sequence+=1;self.previous=record['sha256']
        if kind in {'data_loss','recorder_failure'}:self.failed=True
        return record

    def close(self):
        if self.closed or self.failed:raise RecorderError('Missing healthy terminal; supervisor required')
        self._health('recorder_end',{'state':'CLOSED'})
        self.closed=True


def audit(records, expected_events=()):
    """Verify one ordered recorder stream; do not sort away errors or infer clocks."""
    errors=[];previous='0'*64;seen=set();clocks={};producer_seq={};kinds=[];last_receipt=-1
    config=None;run=None;recover=0;failed_guard=False;last_latch=None
    if not isinstance(records,list) or not records:return ['Missing recorder stream']
    for position,row in enumerate(records,1):
        if not isinstance(row,dict) or set(row)!=REQUIRED:
            errors.append('Invalid event schema');continue
        try:
            if row['schema_version']!=1 or row['kind'] not in KINDS or row['evidence_class']!='B_RRM_ENGINEERING_DIAGNOSTIC':errors.append('Invalid event version/class/kind')
            key=row['event_id']
            if key in seen:errors.append('Duplicate event')
            seen.add(key)
            if row['sequence']!=position or key!=f"{row['run_id']}:{position}":errors.append('Sequence gap or duplicate')
            unsigned={k:v for k,v in row.items() if k!='sha256'}
            if row['previous_sha256']!=previous or hashlib.sha256(canonical(unsigned)).hexdigest()!=row['sha256']:errors.append('Event checksum chain mismatch')
            previous=row['sha256']
            for field in ['sequence','producer_pid','producer_sequence','producer_monotonic_ns','producer_wall_ns','receipt_monotonic_ns','receipt_wall_ns']:
                if type(row[field]) is not int or row[field]<0:raise ValueError('Invalid numeric field')
            if not all(isinstance(row[k],str) and row[k] for k in ['source','clock_id','producer_boot_id','run_id']):raise ValueError('Missing producer provenance')
            identity=(row['source'],row['clock_id'],row['producer_boot_id'],row['producer_pid'])
            if row['producer_monotonic_ns']<clocks.get(identity,-1):errors.append('Nonmonotonic producer timestamp')
            if row['producer_sequence']<=producer_seq.get(identity,-1):errors.append('Duplicate/nonincreasing producer sequence')
            clocks[identity]=row['producer_monotonic_ns'];producer_seq[identity]=row['producer_sequence']
            if row['receipt_monotonic_ns']<last_receipt:errors.append('Nonmonotonic receipt timestamp')
            last_receipt=row['receipt_monotonic_ns']
            if config is None:config=row['configuration_sha256'];run=row['run_id']
            if row['configuration_sha256']!=config or row['run_id']!=run:errors.append('Mixed provenance/run')
            if not isinstance(config,str) or len(config)!=64 or any(x not in '0123456789abcdef' for x in config):errors.append('Invalid configuration identity')
            if not isinstance(row['data'],dict):raise ValueError('Invalid event payload')
            kind=row['kind'];kinds.append(kind)
            if kind in {'data_loss','recorder_failure'}:errors.append('Recorder failure/data loss')
            if kind=='deadline_failure':failed_guard=True;last_latch=row['data'].get('guard_instance')
            if kind=='guard_initialized' and failed_guard and row['data'].get('guard_instance')==last_latch:errors.append('Failure latch silently reset')
            if kind=='vehicle_graph_ready' and 'application_complete' not in kinds:errors.append('Graph readiness before application complete or missing app event')
            if kind=='recorder_ready' and (position!=2 or kinds[0]!='recorder_init'):errors.append('Recorder readiness ordering')
            recovery_order=['recovery_requested','evidence_sealed','recovery_isolated','recovery_approved']
            if kind in recovery_order:
                if recover>=4 or kind!=recovery_order[recover]:errors.append('Recovery event ordering')
                else:recover+=1
        except (ValueError,TypeError,KeyError):errors.append('Invalid event data')
    if kinds[:2]!=['recorder_init','recorder_ready']:errors.append('Recorder initialization/readiness missing')
    if kinds.count('recorder_init')!=1 or kinds.count('recorder_ready')!=1:errors.append('Duplicate/missing recorder initialization')
    if not kinds or kinds[-1]!='recorder_end' or kinds.count('recorder_end')!=1:errors.append('Missing/duplicate terminal event')
    if 'recorder_end' in kinds[:-1]:errors.append('Events after terminal event')
    if recover not in {0,4}:errors.append('Incomplete recovery sequence')
    for kind in expected_events:
        if kind not in kinds:errors.append('Missing expected event: '+kind)
    return sorted(set(errors))


if __name__=='__main__':
    raise SystemExit('BLOCKED: observational library only; runtime collector/transport adapter not implemented or authorized')
