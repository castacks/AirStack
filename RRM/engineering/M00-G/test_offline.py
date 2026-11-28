"""M00-G synthetic unit tests only; no Docker/ROS/PX4/Isaac/MAVROS/network."""
import copy
import hashlib
import io
import json
from pathlib import Path
import unittest
from unittest.mock import patch
from common import canonical, file_sha, load
from config_validator import inspect, check_package
from recorder import Recorder, RecorderError, audit, REQUIRED
from startup_reference import DeadlineReference
from preflight import evaluate

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]


def fake_clock():
    value=0
    def now():
        nonlocal value
        value+=100
        return value
    return now


def stream(kinds=('application_complete','vehicle_graph_ready'),close=True):
    sink=io.StringIO();clock=fake_clock()
    rec=Recorder(sink,'SYNTHETIC-RUN','a'*64,monotonic=clock,wall=clock,boot_id='SYNTHETIC_BOOT',pid=10)
    for index,kind in enumerate(kinds,1):
        rec.emit(kind,source='synthetic_sim',clock_id='synthetic_steady',boot_id='SYNTHETIC_BOOT',
                 pid=11,producer_sequence=index,monotonic_ns=index*1000,wall_ns=index*2000,
                 data={'guard_instance':'synthetic-guard-1','connected':kind=='mavros_connection','synthetic':True})
    if close:rec.close()
    return [json.loads(x) for x in sink.getvalue().splitlines()]


def rechain(rows):
    previous='0'*64
    for item in rows:
        item['previous_sha256']=previous
        item['sha256']=hashlib.sha256(canonical({k:v for k,v in item.items() if k!='sha256'})).hexdigest()
        previous=item['sha256']
    return rows


class ConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.c=load(HERE/'config/compose.json');self.m=load(HERE/'PROVENANCE.json')
        for name,s in self.c['services'].items():
            s['image']='synthetic.invalid/offline/'+name+'@sha256:'+'a'*64
            self.m['images'][name]={'reference':s['image'],'identity_status':'RESOLVED'}
        self.m['required_assets']=[{'identity_status':'RESOLVED','path':'fixtures/synthetic_asset.txt',
                                   'sha256':file_sha(HERE/'fixtures/synthetic_asset.txt')}]

    def result(self):return inspect(self.c,self.m,HERE,ROOT)
    def rejection(self,text):
        result=self.result();self.assertEqual(result['status'],'FAIL');self.assertTrue(any(text in x for x in result['failures']),result)

    def test_01_valid_synthetic_configuration(self):
        self.assertEqual(self.result()['status'],'PASS')
        self.assertEqual(self.result()['runtime_containment'],'NOT VERIFIED')

    def test_02_privileged_rejected(self):
        self.c['services']['sim']['privileged']=True;self.rejection('privileged')

    def test_03_device_mount_rejected(self):
        self.c['services']['mavros']['volumes'].append({'source':'/dev','target':'/dev','type':'bind','read_only':True});self.rejection('physical-device')

    def test_04_host_network_rejected(self):
        self.c['services']['sim']['network_mode']='host';self.rejection('service settings')

    def test_05_unsafe_mavlink_rejected(self):
        self.c['services']['mavros']['environment']['FCU_URL']='/dev/ttyACM0:57600';self.rejection('endpoint')

    def test_06_unsafe_dds_routing_rejected(self):
        self.c['services']['mavros']['environment']['ROS_DOMAIN_ID']='0';self.rejection('DDS')

    def test_07_missing_image_and_asset_identity(self):
        for image in self.m['images'].values():image['reference']=None
        self.m['required_assets'][0]['identity_status']='UNRESOLVED'
        self.assertEqual(self.result()['status'],'BLOCKED')

    def test_08_missing_required_configuration(self):
        del self.c['services']['mavros']['environment'];self.rejection('service settings')

    def test_19_configuration_checksum_mismatch(self):
        self.m['configuration_inputs'][0]['sha256']='0'*64;self.rejection('configuration_inputs checksum')

    def test_20_upstream_change(self):
        target=(ROOT/'docker-compose.yaml').resolve()
        original=Path.read_bytes
        def changed(path):
            value=original(path)
            return value+b'\n# SYNTHETIC UPSTREAM DRIFT' if path.resolve()==target else value
        with patch.object(Path,'read_bytes',changed):self.rejection('upstream_sources checksum')

    def test_missing_provenance(self):
        self.m['upstream_sources']=[];self.assertEqual(self.result()['status'],'BLOCKED')

    def test_public_ports(self):
        self.c['services']['sim']['ports']=['0.0.0.0:14550:14550/udp'];self.rejection('ports')

    def test_namespaces_capabilities_devices(self):
        for key,value in [('ipc','host'),('pid','host'),('cap_add',['SYS_ADMIN']),('devices',['/dev/ttyUSB0']),('device_cgroup_rules',['c *:* rwm'])]:
            with self.subTest(setting=key):
                previous=self.c['services']['sim'].get(key);self.c['services']['sim'][key]=value
                self.rejection('service settings' if key=='pid' else key)
                if previous is None:del self.c['services']['sim'][key]
                else:self.c['services']['sim'][key]=previous

    def test_unknown_environment_and_overlays(self):
        self.c['services']['guard']['environment']['FCU_URL']='udp://@192.168.1.1:14550';self.rejection('environment')
        self.c['include']=['hardware.yaml'];self.rejection('root configuration')

    def test_dds_xml_rejects_external_peer_even_with_checksum_metadata(self):
        original=Path.read_text
        def changed(path,*args,**kwargs):
            text=original(path,*args,**kwargs)
            return text.replace('172.30.50.2','192.168.1.2') if path.name=='dds.xml' else text
        with patch.object(Path,'read_text',changed):self.rejection('DDS external')

    def test_mavlink_forwarding_rejected(self):
        # Endpoint forwarding is independently rejected in the effective environment policy.
        self.c['services']['mavros']['environment']['GCS_URL']='udp://@192.168.1.2:14550'
        self.rejection('endpoint')

    def test_missing_asset_file(self):
        self.m['required_assets'][0]['path']='fixtures/ABSENT.usd';self.assertEqual(self.result()['status'],'BLOCKED')

    def test_missing_required_provenance_path(self):
        self.m['configuration_inputs']=[self.m['configuration_inputs'][0]];self.assertEqual(self.result()['status'],'BLOCKED')

    def test_synthetic_mutation_fixtures(self):
        for case in load(HERE/'fixtures/config_mutations.json'):
            with self.subTest(case=case['name']):
                mutated=copy.deepcopy(self.c);mutated['services'][case['service']][case['field']]=case['value']
                self.assertEqual(inspect(mutated,self.m,HERE,ROOT)['status'],'FAIL')

    def test_boolean_type_not_numeric(self):
        self.c['services']['sim']['privileged']=0;self.rejection('privileged')


class StartupTests(unittest.TestCase):
    def test_09_deadline_failure(self):
        g=DeadlineReference();self.assertEqual(g.tick(10_000_000_000),'STARTUP_DEADLINE')

    def test_10_late_mavros_connection(self):
        g=DeadlineReference();g.tick(10_000_000_000)
        self.assertEqual(g.tick(29_276_162_179,connected=True,transaction_ok=True),'STARTUP_DEADLINE')
        self.assertFalse(g.initialized)

    def test_11_application_complete_before_graph(self):
        records=load(HERE/'fixtures/positive_events.json');self.assertEqual(audit(records,('application_complete','vehicle_graph_ready')),[])
        self.assertTrue(any('Graph readiness' in x for x in audit(stream(('vehicle_graph_ready','application_complete')))))

    def test_12_failure_latch_persists_through_recovery(self):
        g=DeadlineReference();g.tick(11_000_000_000)
        for kind in ['requested','sealed','isolated','approved']:g.recovery_event(kind)
        self.assertEqual(g.tick(100_000_000_000,True,True),'STARTUP_DEADLINE')
        self.assertTrue(any('latch' in x for x in audit(stream(('deadline_failure','mavros_connection','guard_initialized')))))

    def test_13_recovery_order(self):
        good=stream(('recovery_requested','evidence_sealed','recovery_isolated','recovery_approved'))
        self.assertEqual(audit(good),[])
        self.assertTrue(any('Recovery event ordering' in x for x in audit(stream(('recovery_approved','recovery_requested')))))
        self.assertTrue(any('Incomplete recovery' in x for x in audit(stream(('recovery_requested',)))))

    def test_deadline_boundary_before_success(self):
        g=DeadlineReference();self.assertEqual(g.tick(10_000_000_000,True,True),'STARTUP_DEADLINE')
        h=DeadlineReference();self.assertEqual(h.tick(9_000_000_000,True,True),'INITIALIZED')


class RecorderTests(unittest.TestCase):
    def test_14_initialization_failure(self):
        class Broken(io.StringIO):
            def write(self,value):raise OSError('synthetic sink failure')
        with self.assertRaises(RecorderError):Recorder(Broken(),'SYNTHETIC','a'*64)

    def test_15_data_loss(self):
        rows=stream(('data_loss',),close=False)
        self.assertTrue(any('data loss' in x for x in audit(rows)))

    def test_16_duplicate_event(self):
        rows=stream();rows.insert(3,copy.deepcopy(rows[2]));self.assertIn('Duplicate event',audit(rows))

    def test_17_nonmonotonic_timestamp(self):
        rows=stream();rows[3]['producer_monotonic_ns']=0;rechain(rows)
        self.assertIn('Nonmonotonic producer timestamp',audit(rows))

    def test_18_missing_terminal(self):
        self.assertTrue(any('terminal' in x for x in audit(stream(close=False))))

    def test_receipt_clock_regression(self):
        rows=stream();rows[3]['receipt_monotonic_ns']=0;rechain(rows)
        self.assertIn('Nonmonotonic receipt timestamp',audit(rows))

    def test_wall_clock_step_not_monotonic_clock(self):
        rows=stream();rows[3]['producer_wall_ns']=0;rechain(rows)
        self.assertEqual(audit(rows),[])

    def test_incomplete_initialization(self):
        self.assertTrue(any('initialization' in x for x in audit(stream()[1:])))

    def test_short_write_and_flush_failure(self):
        class Short(io.StringIO):
            def write(self,value):return len(value)-1
        class FlushFail(io.StringIO):
            def flush(self):raise OSError('synthetic flush failure')
        for sink in [Short(),FlushFail()]:
            with self.subTest(sink=type(sink).__name__):
                with self.assertRaises(RecorderError):Recorder(sink,'SYNTHETIC','a'*64)

    def test_bad_checksum(self):
        rows=stream();rows[2]['data']['tampered']=True
        self.assertIn('Event checksum chain mismatch',audit(rows))

    def test_schema_matches_writer(self):
        self.assertEqual(set(load(HERE/'EVENT_SCHEMA.json')['required']),REQUIRED)
        self.assertEqual(audit(stream()),[])

    def test_failed_writer_cannot_resume(self):
        sink=io.StringIO();r=Recorder(sink,'SYNTHETIC','a'*64)
        r.emit('recorder_failure',source='recorder',clock_id='clock',boot_id='synthetic',pid=1,
               producer_sequence=1,monotonic_ns=1,wall_ns=1,data={'synthetic':True})
        with self.assertRaises(RecorderError):r.close()

    def test_truncated_and_negative_fixtures(self):
        for name in ['missing_terminal_events.json','duplicate_events.json','nonmonotonic_events.json']:
            with self.subTest(fixture=name):self.assertTrue(audit(load(HERE/'fixtures'/name)))


class PreflightTests(unittest.TestCase):
    def test_real_package_blocked_not_unsafe(self):
        result=evaluate(HERE,ROOT)
        self.assertEqual(result['static_configuration']['static_policy'],'PASS',result)
        self.assertEqual(result['overall'],'BLOCKED');self.assertFalse(result['execution_ready'])
        self.assertEqual(len(result['acceptance_gate']),14)

    def test_static_pass_never_promotes_runtime(self):
        with patch('preflight.check_package',return_value={'status':'PASS','static_policy':'PASS'}):
            result=evaluate(HERE,ROOT)
        self.assertEqual(result['overall'],'BLOCKED');self.assertFalse(result['execution_ready'])
        self.assertEqual(result['runtime_containment']['status'],'NOT VERIFIED')

    def test_guard_source_identity_and_no_command_clients(self):
        p=ROOT/'robot/ros_ws/src/interface/mavros_interface/include/mavros_interface/px4_actuation_guard.hpp'
        self.assertEqual(file_sha(p),'e522a5f6ac10e293e6985b1aaca52d59fad7d80f91a5d5eed0aca0876f785ea2')
        text=(HERE/'guard_host/src/guard_host.cpp').read_text()
        self.assertIn('Px4ActuationGuard',text)
        self.assertNotIn('create_client',text);self.assertNotIn('set_mode',text);self.assertNotIn('arming',text)


if __name__=='__main__':unittest.main(verbosity=2)
