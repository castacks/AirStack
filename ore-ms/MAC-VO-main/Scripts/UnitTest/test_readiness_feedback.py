"""Third invocation, exact readiness evidence and hard request budgets."""
import unittest
from Scripts.Setup.readiness_feedback import ReadinessFeedback
from Scripts.Setup.probe_state_cache import SECOND

class ReadinessTests(unittest.TestCase):
    def setUp(self):
        self.rows=[{c+'_left':{'time_ns':100+10*i} for c in ['rgb','thermal']} for i in range(100)]
        self.f=ReadinessFeedback(self.rows,'bounded_probe');self.f.cache.last_pair=3
        self.f.last_source={'rgb':4,'thermal':2}
        self.f.cache.last_request={'rgb':0,'thermal':0}
        self.clock=2*SECOND
        self.f.cache.resolved['thermal'][0]=(False,self.clock-1)
    def plan(self,pair,now=None):
        now=self.clock+(pair-4)*1000 if now is None else now
        self.f.cache.resolved['rgb'][max(0,pair-1)]=(True,now-1)
        return self.f.plan(pair,now)
    def health(self,a,b,good=True):
        return {'kind':'pair_end','stage':'pair_processing','time0_ns':100+10*a,'time1_ns':100+10*b,'execution_error':None,'original_need_interp':not good}
    def motion(self,a,b,stage='pair_processing'):
        return {'sequence':7,'kind':'motion_result_written','stage':stage,'time0_ns':100+10*a,'time1_ns':100+10*b,
                'first_write_for_submission':True,'original_need_interp':False,'derived_camera_delta_xyzw':[0,0,0,0,0,0,1]}
    def test_third_request_writes_adjacent_after_gap(self):
        a=self.plan(4);self.assertEqual(a['requests'],a['base_decision']['requests'])
        self.f.declare('thermal',5);self.f.ingest('thermal',self.health(2,5),self.clock+1,5)
        self.plan(5);self.f.declare('thermal',6)
        self.f.ingest('thermal',self.motion(2,5),self.clock+1001,6)
        self.f.ingest('thermal',self.health(5,6),self.clock+1002,6)
        self.assertEqual(self.f.live['thermal']['status'],'pending')
        d=self.plan(6);self.assertEqual(d['readiness_overlay']['extra_requests'],['thermal'])
        self.assertEqual(d['action'],d['base_decision']['action'])
        self.f.declare('thermal',7);self.f.ingest('thermal',self.motion(5,6),self.clock+2001,7)
        self.assertEqual(self.f.live['thermal']['status'],'satisfied')
        d=self.plan(7);self.assertEqual(d['readiness_overlay']['extra_requests'],[])
        self.assertEqual(self.f.extra_used,1)
    def test_native_failure_does_not_claim_ready_or_abort(self):
        self.plan(4);self.f.declare('thermal',5);self.f.ingest('thermal',self.health(2,5,False),self.clock+1,5)
        self.assertEqual(self.f.live['thermal']['status'],'pending')
    def test_terminal_gap_and_late_writes_do_not_satisfy(self):
        self.plan(4);self.f.declare('thermal',5);self.f.ingest('thermal',self.health(2,5),self.clock+1,5)
        self.plan(5);self.f.declare('thermal',6)
        self.f.ingest('thermal',self.motion(2,5,'termination'),self.clock+1001,None)
        self.assertEqual(self.f.live['thermal']['status'],'pending')
        self.f.ingest('thermal',self.motion(2,5),self.clock+1002,6)
        self.assertEqual(self.f.live['thermal']['status'],'pending')
        self.f.ingest('thermal',self.health(5,6),self.clock+1003,6)
        self.plan(6);self.f.declare('thermal',7)
        self.plan(7);self.f.declare('thermal',8)
        self.f.ingest('thermal',self.motion(5,6),self.clock+3001,8)
        self.assertEqual(self.f.live['thermal']['status'],'pending')
    def test_transport_failure_cancels_without_endpoint_change(self):
        self.plan(4);self.f.delivery_failure('thermal',5,self.clock+1)
        self.assertEqual(self.f.live['thermal']['status'],'transport_failed')
        self.assertEqual(self.f.last_source['thermal'],2)
        d=self.plan(5);self.assertEqual(d['readiness_overlay']['extra_requests'],[])
    def test_session_limit_four(self):
        for pair in range(4,8):self.plan(pair)
        self.assertEqual(self.f.live['thermal']['attempts'],4)
        d=self.plan(8);self.assertEqual(d['readiness_overlay']['extra_requests'],[])
        self.assertEqual(self.f.session_history[0]['status'],'attempt_cap')
        self.assertEqual(self.f.extra_used,2)
    def test_global_twenty_extra_cap(self):
        for pair in range(4,97):
            now=((pair-4)//5+1)*2*SECOND+((pair-4)%5)*1000
            self.f.cache.resolved['thermal'][pair-1]=(False,now-1)
            self.plan(pair,now)
        self.assertEqual(self.f.extra_used,20)
        self.assertTrue(all(s['attempts']<=4 for s in self.f.session_history))
        self.assertEqual(sum(len(s['extra_sources']) for s in self.f.session_history),20)
    def test_promotion_and_refresh_do_not_force_extra(self):
        self.plan(4)
        self.f.cache.active='thermal';self.f.cache.resolved['thermal'][4]=(True,self.clock+999)
        d=self.plan(5);self.assertEqual(d['action'],'THERMAL')
        self.assertEqual(self.f.session_history[0]['status'],'became_active')
        self.assertEqual(d['readiness_overlay']['extra_requests'],[])
    def test_only_bounded_policy_supported(self):
        with self.assertRaises(ValueError):ReadinessFeedback(self.rows,'full_read_cache')


class SpawnedReadinessTests(unittest.TestCase):
    def test_common_refresh_stops_overlay(self):
        rows=[{c+'_left':{'time_ns':i*10} for c in ['rgb','thermal']} for i in range(100)]
        f=ReadinessFeedback(rows,'bounded_probe');f.cache.last_pair=3
        f.cache.last_request={'rgb':0,'thermal':0}
        f.cache.resolved['rgb'][0]=(True,2*SECOND-1);f.cache.resolved['thermal'][0]=(False,2*SECOND-1)
        f.plan(4,2*SECOND);f.cache.resolved={'rgb':{},'thermal':{}}
        d=f.plan(5,2*SECOND+1)
        self.assertEqual(d['action'],'ABSTAIN');self.assertEqual(f.session_history[0]['status'],'shared_refresh')
        self.assertEqual(d['readiness_overlay']['extra_requests'],[])

    def test_actual_spawned_controller_journals_qualifying_sessions(self):
        import json
        from pathlib import Path
        from types import SimpleNamespace
        import tempfile
        from Scripts.Setup import readiness_continuation
        from Scripts.UnitTest.test_closed_loop_controller import fake_worker
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);rows=[{c+'_left':{'time_ns':100+i*10} for c in ['rgb','thermal']} for i in range(100)]
            manifest=p/'manifest.json';manifest.write_text(json.dumps({'pairs':rows}))
            original=readiness_continuation.worker;readiness_continuation.worker=fake_worker
            try:readiness_continuation.run(SimpleNamespace(manifest=manifest,dataset=p,output=p/'out',frames=100,mode='bounded_probe',timeout_seconds=10,perturb=True,readiness=True))
            finally:readiness_continuation.worker=original
            result=json.loads((p/'out/summary.json').read_text())
            self.assertLessEqual(result['readiness_extra_used'],20)
            self.assertTrue(all(s['attempts']<=4 for s in result['readiness_sessions']))
            log=[json.loads(l) for l in (p/'out/timeline.jsonl').read_text().splitlines()]
            for d in [e for e in log if e['kind']=='preselection']:
                base=d['base_decision'];self.assertEqual(base['action'],d['action'])
                self.assertTrue(set(base['requests'])<=set(d['requests']))

if __name__=='__main__':unittest.main()
