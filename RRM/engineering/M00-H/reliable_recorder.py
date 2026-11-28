"""Durable observational offline journal v2; no vehicle/process control APIs.

Exclusive new file, complete writes, sync each record, fail-closed health. Restart
creates a separate linked segment; it never truncates or repairs original bytes.
"""
from pathlib import Path
import os
import re
import time
from h_support import canonical,sha_bytes,decode

KINDS={'recorder_init','recorder_ready','recorder_end','recorder_failure','data_loss',
       'process_start','application_complete','deadline_failure','startup_failure',
       'mavros_connection','mavros_disconnection','graph_discovery','vehicle_graph_ready',
       'graph_readiness_lost','guard_initialized','guard_status','recovery_requested',
       'evidence_sealed','recovery_isolated','recovery_approved'}
FIELDS={'version','run_id','segment_id','sequence','kind','source','boot_id','pid','clock_id',
        'producer_sequence','monotonic_ns','wall_ns','receipt_monotonic_ns','receipt_wall_ns',
        'configuration_sha256','evidence_class','data','previous_sha256','sha256'}
IDENTITY=re.compile(r'^[a-zA-Z0-9_.:-]{1,128}$')
HASH=re.compile(r'^[0-9a-f]{64}$')


class JournalError(RuntimeError):pass


def validate_record(row):
    if not isinstance(row,dict) or set(row)!=FIELDS:raise JournalError('Invalid schema fields')
    if row['version']!=2 or type(row['version']) is not int or row['kind'] not in KINDS:raise JournalError('Invalid schema version/kind')
    for name in ['run_id','segment_id','source','boot_id','clock_id']:
        if not isinstance(row[name],str) or not IDENTITY.fullmatch(row[name]):raise JournalError('Invalid source identity')
    for name in ['sequence','pid','producer_sequence','monotonic_ns','wall_ns','receipt_monotonic_ns','receipt_wall_ns']:
        if type(row[name]) is not int or row[name]<0:raise JournalError('Invalid integer clock/sequence')
    for name in ['sha256','previous_sha256','configuration_sha256']:
        if not isinstance(row[name],str) or not HASH.fullmatch(row[name]):raise JournalError('Invalid checksum field')
    if row['evidence_class']!='B_SYNTHETIC_OR_ENGINEERING_DIAGNOSTIC' or not isinstance(row['data'],dict):raise JournalError('Invalid evidence class/payload')
    if sha_bytes(canonical({k:v for k,v in row.items() if k!='sha256'}))!=row['sha256']:raise JournalError('Corrupted record checksum')


def scan_bytes(raw,expected_config=None,required_kinds=()):
    errors=[];records=[];previous='0'*64;clocks={};sequences={};identity=None;last_receipt=-1
    lines=raw.splitlines(keepends=True)
    for index,line in enumerate(lines,1):
        if not line.endswith(b'\n'):
            errors.append('Interrupted/truncated record');break
        try:
            row=decode(line);validate_record(row)
            if row['sequence']!=index or row['previous_sha256']!=previous:raise JournalError('Missing/duplicate/out-of-order record')
            group=(row['run_id'],row['segment_id'],row['configuration_sha256'])
            if identity is None:identity=group
            if group!=identity:raise JournalError('Configuration/run/segment mismatch')
            if expected_config is not None and row['configuration_sha256']!=expected_config:raise JournalError('Configuration mismatch')
            key=(row['source'],row['boot_id'],row['pid'],row['clock_id'])
            if row['producer_sequence']!=sequences.get(key,0)+1:raise JournalError('Producer gap/duplicate/out-of-order event')
            if row['monotonic_ns']<clocks.get(key,-1):raise JournalError('Nonmonotonic producer timestamp')
            if row['receipt_monotonic_ns']<last_receipt:raise JournalError('Nonmonotonic receipt timestamp')
            clocks[key]=row['monotonic_ns'];sequences[key]=row['producer_sequence'];last_receipt=row['receipt_monotonic_ns']
            previous=row['sha256'];records.append(row)
        except (ValueError,TypeError,KeyError,UnicodeError,JournalError) as exc:
            errors.append(str(exc));break
    kinds=[r['kind'] for r in records]
    if kinds[:2]!=['recorder_init','recorder_ready']:errors.append('Recorder initialization/readiness missing')
    if kinds.count('recorder_init')!=1 or kinds.count('recorder_ready')!=1:errors.append('Duplicate/missing recorder initialization')
    if kinds.count('recorder_end')!=1 or not kinds or kinds[-1]!='recorder_end':errors.append('Missing/duplicate terminal event')
    if 'recorder_end' in kinds[:-1]:errors.append('Events after terminal')
    if any(x in kinds for x in ['data_loss','recorder_failure']):errors.append('Recorder health degraded/data loss')
    if any(isinstance(r['data'].get('recovery'),dict) and r['data']['recovery'].get('data_loss_possible') for r in records):
        errors.append('Recovery segment linked to prior interrupted/lost data')
    for kind in required_kinds:
        if kind not in kinds:errors.append('Missing required event: '+kind)
    return {'status':'PASS' if not errors else 'DEGRADED','errors':sorted(set(errors)),
            'records':records,'verified_prefix_records':len(records),'raw_sha256':sha_bytes(raw),
            'terminal_health':'STRUCTURALLY_COMPLETE' if not errors else 'DEGRADED',
            'durability_verified':False,'runtime_reliability_verified':False}


class DurableRecorder:
    def __init__(self,path,run_id,configuration_sha256,*,segment_id='segment-1',
                 monotonic=time.monotonic_ns,wall=time.time_ns,write=os.write,sync=os.fsync,
                 recovery=None,allowed_sources=('startup','mavros','graph','guard'),repository=None):
        self.path=Path(path).resolve()
        root=Path(repository).resolve() if repository else Path(__file__).resolve().parents[3]
        if not self.path.is_relative_to(root/'RRM'):raise JournalError('Journal output must remain under RRM')
        if not all(isinstance(x,str) and IDENTITY.fullmatch(x) for x in [run_id,segment_id]) or not HASH.fullmatch(configuration_sha256):raise JournalError('Invalid run/config identity')
        self.run,self.segment,self.config=run_id,segment_id,configuration_sha256
        self.monotonic,self.wall,self.write,self.sync=monotonic,wall,write,sync
        self.sequence=0;self.previous='0'*64;self.closed=False;self.failed=False
        self.producer_seq={};self.producer_time={};self.allowed=set(allowed_sources)|{'recorder'}
        self.fd=None
        try:
            # Parent must already exist (e.g. an explicitly supplied private attempt dir).
            self.fd=os.open(self.path,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW,0o600)
            directory=os.open(self.path.parent,os.O_RDONLY|os.O_DIRECTORY)
            try:self.sync(directory)
            finally:os.close(directory)
            self._health('recorder_init',{'durability':'complete writes + fsync per record','recovery':recovery})
            self._health('recorder_ready',{'state':'READY_LOCAL_JOURNAL'})
        except (OSError,JournalError) as exc:
            self.failed=True;self._release();raise JournalError('Recorder initialization failure') from exc

    def _release(self):
        if self.fd is not None:
            os.close(self.fd);self.fd=None

    def _health(self,kind,data):
        return self.emit(kind,source='recorder',boot_id='recorder-local-unattested',pid=0,
                         clock_id='recorder-monotonic',producer_sequence=self.producer_seq.get(('recorder','recorder-local-unattested',0,'recorder-monotonic'),0)+1,
                         monotonic_ns=self.monotonic(),wall_ns=self.wall(),data=data)

    def emit(self,kind,*,source,boot_id,pid,clock_id,producer_sequence,monotonic_ns,wall_ns,data):
        if self.closed or self.failed:raise JournalError('Recorder closed/degraded; no silent resume')
        key=(source,boot_id,pid,clock_id)
        try:
            if source not in self.allowed:raise JournalError('Undeclared source identity')
            if producer_sequence!=self.producer_seq.get(key,0)+1:raise JournalError('Producer gap/duplicate/out-of-order event')
            if monotonic_ns<self.producer_time.get(key,-1):raise JournalError('Nonmonotonic observation')
            row=dict(version=2,run_id=self.run,segment_id=self.segment,sequence=self.sequence+1,kind=kind,
                     source=source,boot_id=boot_id,pid=pid,clock_id=clock_id,producer_sequence=producer_sequence,
                     monotonic_ns=monotonic_ns,wall_ns=wall_ns,receipt_monotonic_ns=self.monotonic(),
                     receipt_wall_ns=self.wall(),configuration_sha256=self.config,
                     evidence_class='B_SYNTHETIC_OR_ENGINEERING_DIAGNOSTIC',data=data,previous_sha256=self.previous)
            row['sha256']=sha_bytes(canonical(row));validate_record(row)
            pending=canonical(row)+b'\n';offset=0;interruptions=0
            if len(pending)>65536:raise JournalError('Record exceeds65536-byte local buffer bound')
            while offset<len(pending):
                try:count=self.write(self.fd,pending[offset:])
                except InterruptedError:
                    interruptions+=1
                    if interruptions>8:raise JournalError('Write interruption bound exceeded')
                    continue
                if type(count) is not int or count<=0 or count>len(pending)-offset:raise JournalError('Write made no valid progress')
                offset+=count
            # Interrupted synchronization is a failure: cannot acknowledge durable commit.
            self.sync(self.fd)
        except (OSError,ValueError,TypeError,JournalError) as exc:
            self.failed=True;self._release();raise JournalError('Recorder failure/data loss; retain segment and consult external health report') from exc
        self.sequence+=1;self.previous=row['sha256'];self.producer_seq[key]=producer_sequence;self.producer_time[key]=monotonic_ns
        if kind in {'data_loss','recorder_failure'}:self.failed=True;self._release()
        return row

    def close(self):
        if self.closed or self.failed:raise JournalError('Recorder shutdown failure/degraded')
        try:self._health('recorder_end',{'state':'CLOSED_DURABLE_LOCAL'})
        finally:self._release();self.closed=True

    def health_report(self):
        return {'state':'DEGRADED' if self.failed else 'CLOSED' if self.closed else 'READY',
                'acknowledged_sequence':self.sequence,'acknowledged_sha256':self.previous,
                'runtime_reliability_verified':False}

    def interrupt_for_test(self):
        """Synthetic descriptor close with missing terminal; no external process action."""
        self.failed=True;self._release()

    @classmethod
    def restart(cls,old_path,new_path,run_id,configuration_sha256,**kwargs):
        original=Path(old_path).resolve()
        root=Path(kwargs.get('repository') or Path(__file__).resolve().parents[3]).resolve()
        if not original.is_relative_to(root/'RRM'):raise JournalError('Recovery source outside RRM')
        raw=original.read_bytes();report=scan_bytes(raw,expected_config=configuration_sha256)
        if any('Configuration mismatch' in x for x in report['errors']):raise JournalError('Cannot link a different configuration')
        link={'original_path':str(original.relative_to(root)),'original_sha256':sha_bytes(raw),
              'original_status':report['status'],'data_loss_possible':report['status']!='PASS',
              'original_errors':report['errors'],'verified_prefix_records':report['verified_prefix_records']}
        return cls(new_path,run_id,configuration_sha256,recovery=link,**kwargs)
