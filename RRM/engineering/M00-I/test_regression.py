"""Focused offline regressions; metadata fixtures never substitute native headers."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
import native_environment as env
from interface_mapping import StateMapping,RosInterfacePresence,extended_state,STATE,EXTENDED,GET,SET,REQUIRED
from observation_adapters import Stamp,RecorderAdapter
from reliable_recorder import DurableRecorder,JournalError,scan_bytes

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]

class FocusedTests(unittest.TestCase):
    def setUp(self):
        (HERE/'.tmp').mkdir(exist_ok=True)
        self.temp=tempfile.TemporaryDirectory(dir=HERE/'.tmp');self.dir=Path(self.temp.name)
    def tearDown(self):self.temp.cleanup()
    def metadata(self,package,version='1.2.3'):
        share=self.dir/'share'/package;cmake=share/'cmake';cmake.mkdir(parents=True)
        (cmake/(package+'Config.cmake')).write_text('# metadata-only test; never compiled\n')
        (share/'package.xml').write_text(f'<package><version>{version}</version></package>')
        return cmake
    def record(self):
        rec=DurableRecorder(self.dir/'journal.jsonl','typed-test','a'*64,allowed_sources=('observer',))
        self.addCleanup(lambda: None if rec.closed or rec.failed else rec.close())
        return rec,RecorderAdapter(rec,'a'*64)
    def stamp(self,sequence=1):return Stamp('observer','boot',1,'steady',sequence,sequence*20,1000+sequence)
    def test_isolated_workspace_prefix_discovery(self):
        prefix=self.dir/'robot/ros_ws/install/interface_bringup';prefix.mkdir(parents=True)
        self.assertIn(prefix,env.prefix_candidates(self.dir))
    def test_runtime_resource_not_native_requirement(self):
        self.metadata('interface_bringup','0.0.0')
        result=env.inspect_package('interface_bringup',[self.dir])
        self.assertEqual(result['availability'],'AVAILABLE AND VERIFIED')
        self.assertNotIn('interface_bringup',env.PACKAGES)
        package=ET.parse(HERE/'guard_host/package.xml').getroot()
        self.assertEqual(package.findtext('exec_depend'),'interface_bringup')
    def test_config_metadata_cannot_replace_generated_interfaces(self):
        self.metadata('mavros_msgs')
        result=env.inspect_package('mavros_msgs',[self.dir])
        self.assertEqual(result['availability'],'GENERATED ARTIFACT NOT AVAILABLE')
        self.assertFalse(any(result['required_headers'].values()))
        self.assertFalse(any(result['required_libraries'].values()))
    def test_broken_cmake_config_not_available(self):
        cmake=self.dir/'share/ament_cmake/cmake';cmake.mkdir(parents=True)
        (cmake/'ament_cmakeConfig.cmake').symlink_to('absent.cmake')
        self.assertEqual(env.inspect_package('ament_cmake',[self.dir])['availability'],'DECLARED BUT NOT LOCALLY AVAILABLE')
    def test_version_evidence_and_unresolved_version(self):
        cmake=self.metadata('ament_cmake');(cmake.parent/'package.xml').unlink()
        self.assertEqual(env.inspect_package('ament_cmake',[self.dir])['availability'],'VERSION UNRESOLVED')
        (cmake/'ament_cmakeConfigVersion.cmake').write_text('set(PACKAGE_VERSION "2.5.4")\n')
        result=env.inspect_package('ament_cmake',[self.dir])
        self.assertEqual(result['availability'],'AVAILABLE AND VERIFIED')
        self.assertEqual(result['versions'][0]['version'],'2.5.4')
    def test_cmake_and_guard_body_preserved(self):
        self.assertEqual(env.sha(ROOT/env.GUARD),env.GUARD_SHA)
        source=(HERE/'guard_host/src/guard_host.cpp').read_text()
        original=(ROOT/'RRM/engineering/M00-G/guard_host/src/guard_host.cpp').read_text()
        self.assertTrue(source.endswith(original))
        self.assertIn('#include <future>\n#include <stdexcept>',source)
        cmake=(HERE/'guard_host/CMakeLists.txt').read_text()
        for name in env.PACKAGES:self.assertIn(f'find_package({name} REQUIRED)',cmake)
        self.assertIn('cxx_std_17',cmake);self.assertIn(env.GUARD_SHA,cmake)
        self.assertNotIn('find_package(interface_bringup',cmake)
    def test_state_types_and_receipt_clock(self):
        rec,emitter=self.record();adapter=StateMapping(emitter)
        stamp=self.stamp()
        row=adapter.observe(STATE,REQUIRED[STATE],{'connected':True,'armed':False,'header':{'stamp':999999}},stamp,
                            {'boot_id':'boot','clock_id':'steady','deadline_ns':10})
        self.assertTrue(row['data']['connected']);self.assertFalse(row['data']['armed'])
        self.assertEqual(row['monotonic_ns'],20);self.assertEqual(row['data']['late_classification'],'LATE')
        for topic,type_name,fields in [('state',REQUIRED[STATE],{'connected':True,'armed':False}),
              (STATE,'std_msgs/msg/Bool',{'connected':True,'armed':False}),
              (STATE,REQUIRED[STATE],{'connected':1,'armed':False}),
              (STATE,REQUIRED[STATE],{'connected':True})]:
            with self.assertRaises(JournalError):adapter.observe(topic,type_name,fields,self.stamp(2))
        rec.close();self.assertEqual(scan_bytes(rec.path.read_bytes())['status'],'PASS')
    def test_landed_enums_unknown_and_transition(self):
        for value in range(5):
            row=extended_state(EXTENDED,REQUIRED[EXTENDED],{'landed_state':value})
            self.assertEqual(row['on_ground'],value==1);self.assertEqual(row['in_air'],value==2)
            self.assertEqual(row['unknown_or_transition'],value in {0,3,4})
        for value in [True,-1,5,'1']:
            with self.assertRaises(JournalError):extended_state(EXTENDED,REQUIRED[EXTENDED],{'landed_state':value})
    def test_typed_graph_scope_and_persisted_journal(self):
        rec,emitter=self.record();adapter=RosInterfacePresence(emitter)
        row=adapter.observe({key:[value] for key,value in REQUIRED.items()},self.stamp())
        self.assertEqual(row['kind'],'vehicle_graph_ready')
        self.assertEqual(row['data']['readiness_scope'],'ROS_INTERFACE_PRESENCE_ONLY_NOT_SIMULATOR_VEHICLE_GRAPH')
        broken=dict(REQUIRED);broken[GET]='mavros_msgs/srv/ParamGet'
        row=adapter.observe(broken,self.stamp(2));self.assertEqual(row['kind'],'graph_readiness_lost')
        self.assertFalse(row['data']['ready']);rec.close()
        result=scan_bytes(rec.path.read_bytes());self.assertEqual(result['status'],'PASS')
        self.assertIn(b'READINESS',rec.path.read_bytes().upper())
        self.assertIn(b'READINESS_SCOPE',rec.path.read_bytes().upper())
    def test_no_alias_or_ambiguous_graph_type(self):
        rec,emitter=self.record();adapter=RosInterfacePresence(emitter)
        row=adapter.observe({'state':REQUIRED[STATE],'get':REQUIRED[GET]},self.stamp())
        self.assertFalse(row['data']['ready'])
        snapshot={**REQUIRED,STATE:[REQUIRED[STATE],'std_msgs/msg/String']}
        self.assertFalse(adapter.observe(snapshot,self.stamp(2))['data']['ready'])
        self.assertTrue(GET.endswith('/setpoint_raw/get_parameters'))
        self.assertTrue(SET.endswith('/setpoint_raw/set_parameters'))
    def test_blocked_build_never_invokes_native_tools(self):
        import build_native
        snapshot=env.inspect(ROOT,which=lambda name:None)
        with patch.object(build_native,'inspect',return_value=snapshot),patch.object(build_native.subprocess,'run') as run:
            result=build_native.verify(ROOT,self.dir)
        run.assert_not_called();self.assertEqual(result['outcome'],'I2')
        self.assertFalse(result['genuine_native_build']);self.assertEqual(result['commands'],[])
        self.assertEqual(result['stages']['PRODUCTION RUNTIME INTEGRATION'],'NOT VERIFIED')

if __name__=='__main__':unittest.main(verbosity=2)
