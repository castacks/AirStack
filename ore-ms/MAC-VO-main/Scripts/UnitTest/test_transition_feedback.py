"""Request-scoped synthetic transport failures and exact native return endpoints."""
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from Scripts.Setup.transition_feedback import TransitionFeedback,input_available,requested_delivery
from Scripts.Setup.sparse_feedback import execute_request
from Scripts.Setup import availability_transitions
from Scripts.UnitTest.test_closed_loop_controller import fake_worker

class TransitionTests(unittest.TestCase):
    def setUp(self):
        self.rows=[{c+'_left':{'time_ns':100+10*i} for c in ['rgb','thermal']} for i in range(100)]
        self.f=TransitionFeedback(self.rows,'full_read_cache')
    def test_inclusive_fixed_masks(self):
        for camera,start,end in [('rgb',20,34),('thermal',50,64),('rgb',75,79),('thermal',75,79)]:
            self.assertTrue(input_available(camera,start-1,True));self.assertFalse(input_available(camera,start,True))
            self.assertFalse(input_available(camera,end,True));self.assertTrue(input_available(camera,end+1,True))
            self.assertTrue(input_available(camera,start,False))
    def test_unrequested_input_does_not_expose_availability(self):
        def forbidden(*args):raise AssertionError('queried unrequested environment')
        self.assertEqual(requested_delivery('rgb',20,False,True,forbidden),{'delivered':False,'availability_observed':False})
    def test_drop_performs_no_native_work(self):
        d=requested_delivery('rgb',20,True,True);calls=[]
        execute_request(d['delivered'],20,lambda i:calls.append('decode'),lambda f:calls.append('run'))
        self.assertEqual(calls,[])
    def test_failure_does_not_advance_native_endpoint(self):
        self.f.plan(0,0);self.f.declare('rgb',1);self.f.plan(1,1);self.f.delivery_failure('rgb',2,2)
        self.assertEqual(self.f.last_source['rgb'],1);self.assertEqual(self.f.cache.view('rgb',2)['value'],False)
        self.f.plan(2,3);b=self.f.declare('rgb',3);self.assertEqual(b['sensor_interval_ns'],[110,130])
    def test_failure_must_be_requested_and_not_delivered(self):
        with self.assertRaises(ValueError):self.f.delivery_failure('rgb',1,0)
        self.f.plan(0,0);self.f.declare('rgb',1)
        with self.assertRaises(ValueError):self.f.delivery_failure('rgb',1,1)
    def test_old_motion_cannot_overwrite_new_transport_failure(self):
        self.f.plan(0,0);self.f.declare('rgb',1)
        h={'kind':'pair_end','stage':'pair_processing','time0_ns':100,'time1_ns':110,'original_need_interp':False,'execution_error':None}
        self.f.ingest('rgb',h,1,1);self.f.plan(1,2);self.f.delivery_failure('rgb',2,3)
        self.f.plan(2,4);self.f.declare('rgb',3)
        m={'kind':'motion_result_written','stage':'pair_processing','time0_ns':100,'time1_ns':110,'original_need_interp':False,'first_write_for_submission':True,'derived_camera_delta_xyzw':[0,0,0,0,0,0,1]}
        self.f.ingest('rgb',m,5,3);v=self.f.cache.view('rgb',5);self.assertFalse(v['value']);self.assertEqual(v['pair'],1)
    def test_spawned_controller_separates_requests_drops_and_native_gaps(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);manifest=p/'manifest.json';manifest.write_text(json.dumps({'pairs':self.rows}))
            old=availability_transitions.worker;availability_transitions.worker=fake_worker
            try:availability_transitions.run(SimpleNamespace(manifest=manifest,dataset=p,output=p/'out',frames=100,mode='full_read_cache',timeout_seconds=10,perturb=True))
            finally:availability_transitions.worker=old
            log=[json.loads(l) for l in (p/'out/timeline.jsonl').read_text().splitlines()]
            drops=[e for e in log if e['kind']=='transport_drop'];self.assertEqual(len(drops),40)
            self.assertEqual(len([e for e in log if e['kind']=='transport_ingress']),40)
            resumed=next(e for e in log if e['kind']=='release' and e['camera']=='rgb' and e['source']==35)
            self.assertEqual(resumed['binding']['source_start'],19)
            for e in log:
                if e['kind']=='preselection' and 76<=e['pair']+1<=80:self.assertEqual(e['action'],'ABSTAIN')
    def test_fixed_camera_baseline_is_explicit(self):
        f=TransitionFeedback(self.rows,'fixed_rgb')
        for i in range(3):d=f.plan(i,i)
        self.assertEqual(d['action'],'RGB');self.assertEqual(d['requests'],['rgb'])

if __name__=='__main__':unittest.main()
