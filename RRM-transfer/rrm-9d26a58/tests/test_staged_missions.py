"""Exact-plan admission tests; no ROS, Docker or flight dispatch."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch
from types import SimpleNamespace
from http.server import ThreadingHTTPServer
from urllib.request import Request, urlopen
from urllib.error import HTTPError

from test_command_console import Console, FakeProcess, make_handler


class StagedMissionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        office = Path(__file__).parents[1] / 'examples/office_visual_eval'
        self.app = Console(None, Path(self.tmp.name), '/unused',
                           context_template=office/'navigation_context.json',
                           scene_manifest=office/'scene_manifest.json')
        self.run = self.app.save('Take off to 1 meter at 0.5 m/s, then land.')['request_id']
        self.discovery = dict(missing_state=[], stale_state=[], connected=True,
                             armed=False, airborne=False, flight_state_consistent=True,
                             clock_epoch_consistent=True, frame_id='map', child_frame_id='base_link',
                             position=dict(x=0., y=0., z=0.), yaw_rad=0.,
                             task_servers={'/robot_1/tasks/takeoff':['task_msgs/action/TakeoffTask'],
                                           '/robot_1/tasks/land':['task_msgs/action/LandTask']})
        for obj, name, value in ((self.app,'discover_tasks',self.discovery),
                                 (self.app,'_command_runtime_identity',['identity']),
                                 (self.app,'require_isaac_runtime',{'compatible':True})):
            p = patch.object(obj, name, return_value=value)
            p.start(); self.addCleanup(p.stop)
        self.path = Path(self.tmp.name)/self.run/'command-plan.json'

    def stage(self):
        with patch('rrm_command_console.subprocess.Popen') as launch, \
                patch('rrm_command_console.subprocess.run') as docker:
            staged = self.app.stage_command_mission(self.run)
            launch.assert_not_called(); docker.assert_not_called()
        return staged

    def remote_ok(self, staged):
        def result(args, **kwargs):
            value = (json.dumps(staged['plan']['source_manifest'])
                     if 'python3' in args else staged['plan_sha256']+' plan')
            return SimpleNamespace(stdout=value)
        return result

    def test_stage_no_launch_and_idempotent_exact_bytes(self):
        staged = self.stage()
        self.assertFalse(staged['execution_dispatch'])
        self.assertFalse(staged['plan']['execution_dispatch'])
        self.assertEqual(staged['plan']['recovery']['action']['kind'],'LAND')
        self.assertEqual(self.app.stage_command_mission(self.run),staged)
        self.assertEqual(staged['plan_sha256'],hashlib.sha256(self.path.read_bytes()).hexdigest())

    def test_execute_requires_stage_and_strict_review_hash(self):
        for digest in (None, 'bad', '0'*64):
            with self.assertRaises((ValueError,RuntimeError)):
                self.app.start_command_mission(self.run,digest)
        staged=self.stage()
        with self.assertRaisesRegex(RuntimeError,'hashes'):
            self.app.start_command_mission(self.run,'0'*64)
        self.assertEqual(self.app.store.get_run(self.run)['execution_state'],'REVIEW_REQUIRED')

    def test_tampered_disk_and_store_fail_closed(self):
        staged=self.stage(); raw=self.path.read_bytes()
        self.path.write_bytes(raw+b' ')
        with self.assertRaisesRegex(RuntimeError,'hashes'):
            self.app.start_command_mission(self.run,staged['plan_sha256'])
        self.path.write_bytes(raw)
        self.app.store.set_lifecycle(self.run,proposal_sha256='0'*64)
        with self.assertRaisesRegex(RuntimeError,'hashes'):
            self.app.start_command_mission(self.run,staged['plan_sha256'])

    def test_fresh_state_scene_identity_and_servers_fail_closed(self):
        staged=self.stage(); original=copy.deepcopy(self.discovery)
        changes=({'stale_state':['odom']},{'clock_epoch_consistent':False},
                 {'armed':True},{'position':dict(x=.2,y=0,z=0)},
                 {'position':dict(x=float('nan'),y=0,z=0)}, {'yaw_rad':.2},
                 {'task_servers':{'/wrong/tasks/land':['task_msgs/action/LandTask']}})
        for change in changes:
            with self.subTest(change=change):
                self.discovery.clear(); self.discovery.update(copy.deepcopy(original)); self.discovery.update(change)
                with self.assertRaises(RuntimeError):
                    self.app.start_command_mission(self.run,staged['plan_sha256'])
                self.assertEqual(self.app.store.get_run(self.run)['execution_state'],'REVIEW_REQUIRED')
        self.discovery.clear();self.discovery.update(original)
        with patch.object(self.app,'_command_runtime_identity',return_value=['changed']), \
                self.assertRaisesRegex(RuntimeError,'identity'):
            self.app.start_command_mission(self.run,staged['plan_sha256'])
        self.app.active_scene_shortname='warehouse'
        with self.assertRaisesRegex(RuntimeError,'scene'):
            self.app.start_command_mission(self.run,staged['plan_sha256'])

    def test_launch_failure_consumed_and_restart_cannot_replay(self):
        staged=self.stage()
        with patch.object(self.app,'_launch_staged_command',side_effect=OSError('launch failed')):
            with self.assertRaises(OSError):
                self.app.start_command_mission(self.run,staged['plan_sha256'])
        self.assertEqual(self.app.store.get_run(self.run)['execution_state'],'FINISHED')
        with self.assertRaisesRegex(RuntimeError,'unconsumed'):
            self.app.start_command_mission(self.run,staged['plan_sha256'])
        self.assertTrue((self.path.parent/'command-launch-failed.json').exists())

    def test_remote_digest_mismatch_consumed_without_launch(self):
        staged=self.stage()
        with patch('rrm_command_console.subprocess.run',return_value=SimpleNamespace(stdout='0'*64+' plan')), \
                patch('rrm_command_console.subprocess.Popen') as launch:
            with self.assertRaisesRegex(RuntimeError,'Remote'):
                self.app.start_command_mission(self.run,staged['plan_sha256'])
            launch.assert_not_called()
        self.assertEqual(self.app.store.get_run(self.run)['execution_state'],'FINISHED')

    def test_exact_bytes_success_once_no_recompile(self):
        staged=self.stage(); raw=self.path.read_bytes()
        with patch('rrm_command_console.subprocess.run',side_effect=self.remote_ok(staged)), \
                patch('rrm_command_console.subprocess.Popen',return_value=FakeProcess()) as launch, \
                patch('rrm_command_console.ground_command',side_effect=AssertionError('recompiled')):
            result=self.app.start_command_mission(self.run,staged['plan_sha256'])
            self.addCleanup(self.app.mission_runtime['log_handle'].close)
            self.assertTrue(result['active']);launch.assert_called_once()
            self.assertEqual((self.path.parent/'reviewed-command-plan.json').read_bytes(),raw)
            with self.assertRaises(RuntimeError):
                self.app.start_command_mission(self.run,staged['plan_sha256'])

    def test_concurrent_execute_only_one_claim(self):
        staged=self.stage(); calls=[]; errors=[]
        def launch(*args): calls.append(args);return {'active':False}
        def run():
            try:self.app.start_command_mission(self.run,staged['plan_sha256'])
            except RuntimeError as e:errors.append(e)
        with patch.object(self.app,'_launch_staged_command',side_effect=launch):
            threads=[threading.Thread(target=run) for _ in range(2)]
            for t in threads:t.start()
            for t in threads:t.join()
        self.assertEqual(len(calls),1);self.assertEqual(len(errors),1)

    def test_two_console_instances_share_durable_one_shot_claim(self):
        staged=self.stage()
        office=Path(__file__).parents[1]/'examples/office_visual_eval'
        other=Console(None,Path(self.tmp.name),'/unused',
                      context_template=office/'navigation_context.json',
                      scene_manifest=office/'scene_manifest.json')
        self.assertEqual(other.stage_command_mission(self.run),staged)
        barrier=threading.Barrier(2);calls=[];errors=[]
        def recheck(*args):barrier.wait(timeout=5);return self.discovery
        def launch(*args):calls.append(args);return {'active':False}
        def run(app):
            try:app.start_command_mission(self.run,staged['plan_sha256'])
            except RuntimeError as error:errors.append(error)
        with patch.object(self.app,'_recheck_staged_command',side_effect=recheck), \
                patch.object(other,'_recheck_staged_command',side_effect=recheck), \
                patch.object(self.app,'_launch_staged_command',side_effect=launch), \
                patch.object(other,'_launch_staged_command',side_effect=launch):
            threads=[threading.Thread(target=run,args=(app,)) for app in (self.app,other)]
            for t in threads:t.start()
            for t in threads:t.join(timeout=10)
        self.assertEqual(len(calls),1);self.assertEqual(len(errors),1)

    def test_last_recheck_failure_consumes_without_popen(self):
        staged=self.stage()
        real=self.app._recheck_staged_command; calls=[]
        def check(*args):
            calls.append(1)
            if len(calls)>1:raise RuntimeError('fresh state lost')
            return real(*args)
        with patch.object(self.app,'_recheck_staged_command',side_effect=check), \
                patch('rrm_command_console.subprocess.run',side_effect=self.remote_ok(staged)), \
                patch('rrm_command_console.subprocess.Popen') as launch:
            with self.assertRaisesRegex(RuntimeError,'lost'):
                self.app.start_command_mission(self.run,staged['plan_sha256'])
            launch.assert_not_called()
        self.assertEqual(self.app.store.get_run(self.run)['execution_state'],'FINISHED')

    def test_executor_source_change_is_rejected_before_claim(self):
        staged=self.stage()
        with patch.object(self.app,'_command_source_manifest',return_value={'changed':'x'}), \
                self.assertRaisesRegex(RuntimeError,'source'):
            self.app.start_command_mission(self.run,staged['plan_sha256'])
        self.assertEqual(self.app.store.get_run(self.run)['execution_state'],'REVIEW_REQUIRED')

    def test_remote_source_mismatch_consumed_without_launch(self):
        staged=self.stage()
        def remote(args, **kwargs):
            return SimpleNamespace(stdout='{}' if 'python3' in args else staged['plan_sha256']+' plan')
        with patch('rrm_command_console.subprocess.run',side_effect=remote), \
                patch('rrm_command_console.subprocess.Popen') as launch:
            with self.assertRaisesRegex(RuntimeError,'source manifest'):
                self.app.start_command_mission(self.run,staged['plan_sha256'])
            launch.assert_not_called()
        self.assertEqual(self.app.store.get_run(self.run)['execution_state'],'FINISHED')

    def test_no_persistence_write_after_successful_popen(self):
        staged=self.stage()
        def launched(*args, **kwargs):
            self.app.store.set_lifecycle = lambda *a, **k: (_ for _ in ()).throw(AssertionError('post-launch write'))
            return FakeProcess()
        with patch('rrm_command_console.subprocess.run',side_effect=self.remote_ok(staged)), \
                patch('rrm_command_console.subprocess.Popen',side_effect=launched):
            result=self.app.start_command_mission(self.run,staged['plan_sha256'])
            self.addCleanup(self.app.mission_runtime['log_handle'].close)
            self.assertTrue(result['active'])

    def test_http_stage_and_review_hash_boundary(self):
        server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(self.app))
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        def post(path, value):
            req=Request('http://127.0.0.1:'+str(server.server_port)+path,
                        data=json.dumps(value).encode(),headers={'X-RRM-Token':self.app.token,
                                                               'Content-Type':'application/json'})
            with urlopen(req,timeout=5) as result:return json.load(result)
        try:
            with patch('rrm_command_console.subprocess.Popen') as launch, \
                    patch('rrm_command_console.subprocess.run') as docker:
                staged=post('/api/runs/'+self.run+'/stage',{})
                restored=post('/api/runs/'+self.run+'/stage',{})
                self.assertEqual(staged,restored)
                for value in ({},{'reviewed_plan_sha256':'0'*64}):
                    with self.assertRaises(HTTPError) as error:
                        post('/api/runs/'+self.run+'/execute',value)
                    self.assertIn(error.exception.code,(400,409))
                launch.assert_not_called();docker.assert_not_called()
        finally:
            server.shutdown();server.server_close();thread.join(timeout=5)
