"""Synthetic integration, guard correspondence and local-file reliability tests."""
from pathlib import Path
import copy
import json
import os
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
from h_support import canonical,sha_bytes,decode,file_sha
from guard_reference import GuardReference
from reliable_recorder import DurableRecorder,JournalError,scan_bytes
from observation_adapters import Stamp,RecorderAdapter,MavrosStateAdapter,VehicleGraphAdapter
from integration_harness import run_scenario,Counter
from check_build_dependencies import check,HEADER,GUARD_SHA
from h_preflight import evaluate,source_correspondence

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]


class Files(unittest.TestCase):
    def setUp(self):
        (HERE/'.tmp').mkdir(exist_ok=True)
        self.temp=tempfile.TemporaryDirectory(dir=HERE/'.tmp');self.dir=Path(self.temp.name)
    def tearDown(self):self.temp.cleanup()
    def fixture(self,name):return decode((HERE/'fixtures'/(name+'.json')).read_bytes())['events']
    def scenario(self,name,**kwargs):return run_scenario(self.dir/(name+'.jsonl'),self.fixture(name),**kwargs)
    def recorder(self,**kwargs):
        clock=Counter();return DurableRecorder(self.dir/'journal.jsonl','synthetic-run','a'*64,monotonic=clock,wall=clock,**kwargs)
    def event(self,rec,**kwargs):
        options=dict(source='startup',boot_id='synthetic-boot',pid=1,clock_id='clock',producer_sequence=1,
                     monotonic_ns=1,wall_ns=1,data={'synthetic':True});options.update(kwargs)
        return rec.emit('process_start',**options)


class IntegrationTests(Files):
    def test_01_nominal(self):
        result=self.scenario('nominal');self.assertEqual(result['journal']['status'],'PASS');self.assertTrue(result['guard_initialized'])
    def test_02_deadline_expiration_before_connection(self):
        result=self.scenario('late_connection');self.assertEqual(result['guard_reason'],'STARTUP_DEADLINE')
    def test_03_connection_after_deadline(self):
        result=self.scenario('late_connection');connection=next(x for x in result['outcomes'] if x['kind']=='mavros_connection')
        self.assertEqual(connection['data']['late_classification'],'LATE');self.assertFalse(result['guard_initialized'])
    def test_04_app_before_graph(self):self.assertEqual(self.scenario('nominal')['semantic_warnings'],[])
    def test_05_graph_before_app(self):self.assertTrue(self.scenario('graph_before_app')['semantic_warnings'])
    def test_06_failure_latch(self):self.assertFalse(self.scenario('late_connection')['guard_initialized'])
    def test_07_recovery_does_not_reset(self):
        result=self.scenario('late_recovery');self.assertFalse(result['guard_initialized']);self.assertEqual(result['guard_reason'],'STARTUP_DEADLINE')
    def test_08_disconnection(self):self.assertIn('mavros_disconnection',[x['kind'] for x in self.scenario('disconnect_graph_loss')['outcomes']])
    def test_09_graph_loss(self):self.assertIn('graph_readiness_lost',[x['kind'] for x in self.scenario('disconnect_graph_loss')['outcomes']])
    def test_10_duplicate_start(self):
        with self.assertRaises(JournalError):self.scenario('duplicate_start')
    def test_11_missing_app(self):self.assertIn('Missing required event: application_complete',self.scenario('missing_app')['journal']['errors'])
    def test_12_nonmonotonic(self):
        with self.assertRaises(JournalError):self.scenario('nonmonotonic')
    def test_13_write_failure(self):
        def fail(fd,value):raise OSError('synthetic disk full')
        with self.assertRaises(JournalError):self.recorder(write=fail)
    def test_14_restart_after_interrupted_output(self):
        rec=self.recorder();self.event(rec);rec.interrupt_for_test();old=rec.path.read_bytes()
        with rec.path.open('ab') as f:f.write(b'{"interrupted":')
        old=rec.path.read_bytes()
        successor=DurableRecorder.restart(rec.path,self.dir/'new.jsonl','synthetic-run','a'*64,segment_id='segment-2')
        successor.close();report=scan_bytes(successor.path.read_bytes())
        self.assertEqual(rec.path.read_bytes(),old);self.assertEqual(report['status'],'DEGRADED')
        self.assertTrue(report['records'][0]['data']['recovery']['data_loss_possible'])
    def test_15_invalid_source(self):
        with self.assertRaises(JournalError):self.scenario('invalid_source')
    def test_16_configuration_mismatch(self):
        rec=self.recorder()
        with self.assertRaises(JournalError):RecorderAdapter(rec,'b'*64)
        rec.interrupt_for_test()
    def test_17_missing_dependency(self):self.assertIn('rclcpp',check(ROOT,prefixes=[],which=lambda _:None)['missing'])
    def test_18_guard_build_unavailable(self):self.assertEqual(check(ROOT,prefixes=[],which=lambda _:None)['status'],'BLOCKED')
    def test_19_degraded_recorder_health(self):
        rec=self.recorder()
        with self.assertRaises(JournalError):self.event(rec,source='unregistered')
        self.assertEqual(rec.health_report()['state'],'DEGRADED')
    def test_20_end_to_end_integrity(self):
        result=self.scenario('nominal');records=result['journal']['records']
        self.assertEqual(result['journal']['status'],'PASS');self.assertEqual([x['sequence'] for x in records],list(range(1,len(records)+1)))
        self.assertTrue(all(x['configuration_sha256']=='a'*64 for x in records))


class GuardTests(unittest.TestCase):
    def safe(self,g,now=1):g.state(now,True);g.extended_state(now,True)
    def test_original_identity_and_correspondence(self):self.assertEqual(source_correspondence(ROOT)['source_identity'],'PASS')
    def test_failed_transaction_latches_regression(self):
        import importlib.util
        spec=importlib.util.spec_from_file_location('_old_deadline_reference',ROOT/'RRM/engineering/M00-G/startup_reference.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        old=module.DeadlineReference();old.tick(2,connected=True,transaction_ok=False)
        self.assertFalse(old.attempted)  # Explicit G subset omission; never change its sealed source.
        g=GuardReference();self.safe(g)
        self.assertEqual(g.tick(2,service_available=True,set_ok=False),'STARTUP_NOT_VERIFIED')
        self.assertTrue(g.attempted);self.safe(g,3)
        self.assertEqual(g.tick(4,service_available=True,set_ok=True,typed_double_readback=True),'STARTUP_NOT_VERIFIED')
    def test_waiting_service_does_not_attempt(self):
        g=GuardReference();self.safe(g);self.assertEqual(g.tick(2),'WAITING_FOR_PARAMETER_SERVICE');self.assertFalse(g.attempted)
    def test_deadline_exact_boundary_wins(self):
        g=GuardReference();self.safe(g,10_000_000_000)
        self.assertEqual(g.tick(10_000_000_000,service_available=True,set_ok=True,typed_double_readback=True),'STARTUP_DEADLINE')
    def test_query_busy_delays_evaluation(self):
        g=GuardReference();self.assertEqual(g.tick(11_000_000_000,query_available=False),'QUERY_BUSY');self.assertFalse(g.attempted)
    def test_unsafe_latch_priority(self):
        g=GuardReference();g.state(1,True,armed=True);g.state(2,True,armed=False);g.extended_state(2,True)
        self.assertEqual(g.tick(11_000_000_000),'UNSAFE_BEFORE_INITIALIZATION')
    def test_in_air_latches(self):
        g=GuardReference();g.extended_state(1,False,in_air=True);g.extended_state(2,True)
        self.assertTrue(g.unsafe)
    def test_freshness(self):
        g=GuardReference();self.safe(g);self.assertTrue(g.safe(500_000_001));self.assertFalse(g.safe(500_000_002))
    def test_config_rejected(self):
        g=GuardReference(config_valid=False);self.safe(g);self.assertEqual(g.tick(2),'CONFIG_REJECTED')
    def test_readback_can_recover_only_after_initial_success(self):
        g=GuardReference();self.safe(g);self.assertEqual(g.tick(2,service_available=True,set_ok=True,typed_double_readback=True),'READY')
        self.assertEqual(g.tick(3,typed_double_readback=False),'READBACK_UNVERIFIED')
        self.assertEqual(g.tick(4,typed_double_readback=True),'READY')
    def test_app_and_graph_are_not_guard_inputs(self):
        self.assertFalse(hasattr(GuardReference(),'application_complete'));self.assertFalse(hasattr(GuardReference(),'graph_ready'))
    def test_transaction_exception_latches(self):
        g=GuardReference();self.assertEqual(g.tick(1,exception=True),'EXCEPTION');self.assertTrue(g.attempted);self.assertFalse(g.initialized)


class ReliabilityTests(Files):
    def test_partial_writes_complete(self):
        def short(fd,value):return os.write(fd,value[:7])
        rec=self.recorder(write=short);self.event(rec);rec.close();self.assertEqual(scan_bytes(rec.path.read_bytes())['status'],'PASS')
    def test_retry_interrupted_write(self):
        state={'first':True}
        def interrupted(fd,value):
            if state['first']:state['first']=False;raise InterruptedError()
            return os.write(fd,value)
        rec=self.recorder(write=interrupted);rec.close();self.assertEqual(scan_bytes(rec.path.read_bytes())['status'],'PASS')
    def test_repeated_interruption_bounded(self):
        def interrupted(fd,value):raise InterruptedError()
        with self.assertRaises(JournalError):self.recorder(write=interrupted)
    def test_zero_progress(self):
        with self.assertRaises(JournalError):self.recorder(write=lambda fd,value:0)
    def test_initialization_failure(self):
        with self.assertRaises(JournalError):DurableRecorder(self.dir/'absent'/'j','synthetic','a'*64)
    def test_interrupted_sync(self):
        def interrupted(fd):raise InterruptedError()
        with self.assertRaises(JournalError):self.recorder(sync=interrupted)
    def test_shutdown_sync_failure(self):
        rec=self.recorder()
        def fail(fd):raise OSError('synthetic shutdown sync')
        rec.sync=fail
        with self.assertRaises(JournalError):rec.close()
        self.assertEqual(rec.health_report()['state'],'DEGRADED')
    def test_invalid_schema(self):
        report=scan_bytes(b'{"invalid":true}\n');self.assertEqual(report['status'],'DEGRADED')
    def test_corruption(self):
        rec=self.recorder();rec.close();raw=rec.path.read_bytes().replace(b'READY_LOCAL_JOURNAL',b'WRONG_LOCAL_JOURNAL')
        self.assertEqual(scan_bytes(raw)['status'],'DEGRADED')
    def test_duplicate_records(self):
        rec=self.recorder();rec.close();lines=rec.path.read_bytes().splitlines(keepends=True)
        self.assertEqual(scan_bytes(lines[0]+lines[0]+b''.join(lines[1:]))['status'],'DEGRADED')
    def test_missing_record(self):
        rec=self.recorder();self.event(rec);rec.close();lines=rec.path.read_bytes().splitlines(keepends=True)
        self.assertEqual(scan_bytes(b''.join(lines[:2]+lines[3:]))['status'],'DEGRADED')
    def test_producer_gap_and_duplicate(self):
        rec=self.recorder()
        with self.assertRaises(JournalError):self.event(rec,producer_sequence=2)
    def test_invalid_event_kind(self):
        rec=self.recorder()
        with self.assertRaises(JournalError):rec.emit('arm',source='startup',boot_id='boot',pid=1,clock_id='clock',producer_sequence=1,monotonic_ns=1,wall_ns=1,data={})
    def test_exclusive_creation_preserves_prior(self):
        rec=self.recorder();rec.close();raw=rec.path.read_bytes()
        with self.assertRaises(JournalError):DurableRecorder(rec.path,'new','a'*64)
        self.assertEqual(rec.path.read_bytes(),raw)
    def test_real_offline_child_abrupt_exit(self):
        path=self.dir/'crash.jsonl'
        done=subprocess.run([sys.executable,'-B',str(HERE/'crash_fixture.py'),str(path)],capture_output=True,timeout=10)
        self.assertEqual(done.returncode,23);self.assertEqual(scan_bytes(path.read_bytes())['status'],'DEGRADED')
    def test_out_of_order_records(self):
        rec=self.recorder();self.event(rec);rec.close();lines=rec.path.read_bytes().splitlines(keepends=True)
        lines[1],lines[2]=lines[2],lines[1]
        self.assertEqual(scan_bytes(b''.join(lines))['status'],'DEGRADED')
    def test_structural_health_not_runtime_durability(self):
        rec=self.recorder();rec.close();report=scan_bytes(rec.path.read_bytes())
        self.assertFalse(report['durability_verified']);self.assertFalse(report['runtime_reliability_verified'])
    def test_wall_clock_steps_do_not_fake_monotonic_failure(self):
        rec=self.recorder();self.event(rec,wall_ns=100)
        self.event(rec,producer_sequence=2,monotonic_ns=2,wall_ns=1);rec.close()
        self.assertEqual(scan_bytes(rec.path.read_bytes())['status'],'PASS')
    def test_failed_attempt_does_not_resume_writer(self):
        rec=self.recorder()
        with self.assertRaises(JournalError):self.event(rec,producer_sequence=9)
        with self.assertRaises(JournalError):self.event(rec)

    def test_explicit_data_loss_indicator(self):
        rec=self.recorder()
        rec.emit('data_loss',source='startup',boot_id='synthetic',pid=1,clock_id='clock',
                 producer_sequence=1,monotonic_ns=1,wall_ns=1,data={'lost':1})
        self.assertEqual(rec.health_report()['state'],'DEGRADED')
        self.assertEqual(scan_bytes(rec.path.read_bytes())['status'],'DEGRADED')
    def test_record_size_bound(self):
        rec=self.recorder()
        with self.assertRaises(JournalError):self.event(rec,data={'too_large':'x'*70000})
    def test_terminal_missing(self):
        rec=self.recorder();rec.interrupt_for_test();self.assertEqual(scan_bytes(rec.path.read_bytes())['status'],'DEGRADED')
    def test_buffered_clock_types_rejected(self):
        rec=self.recorder()
        with self.assertRaises(JournalError):self.event(rec,monotonic_ns=True)


class AdapterTests(Files):
    def test_cross_clock_late_connection_unmeasurable(self):
        rec=self.recorder();adapter=MavrosStateAdapter(RecorderAdapter(rec,'a'*64))
        row=adapter.observe(True,Stamp('mavros','boot',1,'different',1,20,20),{'boot_id':'boot','clock_id':'guard','deadline_ns':10})
        self.assertIn('UNMEASURABLE',row['data']['late_classification']);rec.close()
    def test_graph_missing_required_interface(self):
        rec=self.recorder();adapter=VehicleGraphAdapter(RecorderAdapter(rec,'a'*64),{'state','get'})
        row=adapter.observe(['state'],Stamp('graph','boot',1,'clock',1,1,1));self.assertFalse(row['data']['ready']);rec.close()
    def test_no_control_interfaces(self):
        from observation_adapters import StartupLifecycleAdapter
        for cls in [RecorderAdapter,StartupLifecycleAdapter,MavrosStateAdapter,VehicleGraphAdapter]:
            self.assertFalse(any(hasattr(cls,name) for name in ['arm','start','restart','reset_guard','set_mode']))


class PreflightTests(unittest.TestCase):
    def test_runtime_never_promoted(self):
        root=ROOT;path='RRM/engineering/M00-H/test_h.py'
        tests={'status':'PASS','test_count':50,'source_sha256':{path:file_sha(root/path)}}
        builds={'build_status':'BLOCKED','dependency_free_python':'PASS','python_sources':[{'path':path,'sha256':file_sha(root/path)}],'dependencies':{'missing':['cxx']}}
        result=evaluate(root,builds,tests)
        self.assertEqual(result['BUILD STATUS'],'BLOCKED');self.assertEqual(result['OFFLINE TEST STATUS'],'PASS')
        self.assertEqual(result['OVERALL EXECUTION READINESS'],'BLOCKED');self.assertFalse(result['execution_ready'])
    def test_stale_test_identity_fails(self):
        result=evaluate(ROOT,{},dict(status='PASS',test_count=50,source_sha256={'not-here':'0'*64}))
        self.assertEqual(result['OFFLINE TEST STATUS'],'FAIL');self.assertFalse(result['execution_ready'])


if __name__=='__main__':unittest.main(verbosity=2)
