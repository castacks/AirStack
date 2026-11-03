"""Native gap identity, requested feedback and zero work on skipped frames."""
import copy
import unittest
from Scripts.Setup.sparse_feedback import SparseFeedback,execute_request

class FeedbackTests(unittest.TestCase):
    def setUp(self):
        self.rows=[{c+'_left':{'time_ns':100+i*10} for c in ['rgb','thermal']} for i in range(9)]
        self.f=SparseFeedback(self.rows,'bounded_probe')
    def health(self,a,b,good=True):
        return {'kind':'pair_end','stage':'pair_processing','time0_ns':100+10*a,'time1_ns':100+10*b,'original_need_interp':not good,'execution_error':None}
    def motion(self,a,b,stage='pair_processing'):
        return {'kind':'motion_result_written','stage':stage,'time0_ns':100+10*a,'time1_ns':100+10*b,'original_need_interp':False,'first_write_for_submission':True,'derived_camera_delta_xyzw':[0,0,0,0,0,0,1]}
    def begin(self):
        self.f.plan(0,0)
        for c in ['rgb','thermal']:self.f.declare(c,1)
    def test_skip_does_no_decode_or_run(self):
        calls=[];decode=lambda i:calls.append(('decode',i));run=lambda x:calls.append(('run',x))
        self.assertFalse(execute_request(False,7,decode,run));self.assertEqual(calls,[])
        self.assertTrue(execute_request(True,7,decode,run));self.assertEqual(calls,[('decode',7),('run',None)])
    def test_requested_health_pending_until_first_write(self):
        self.begin();self.f.ingest('rgb',self.health(0,1),1,1)
        self.assertEqual(self.f.cache.view('rgb',1)['status'],'UNKNOWN')
        self.f.plan(1,2);self.f.declare('rgb',2)
        r=self.f.ingest('rgb',self.motion(0,1),3,2);self.assertTrue(r['accepted']);self.assertTrue(self.f.cache.view('rgb',3)['value'])
    def test_gap_bound_to_processed_start_not_adjacent_source(self):
        self.begin();self.f.plan(1,1);self.f.declare('rgb',2)
        self.f.plan(2,2);self.f.plan(3,3);bound=self.f.declare('rgb',4)
        self.assertEqual(bound['sensor_interval_ns'],[120,140]);self.assertEqual(bound['source_start'],2)
        self.assertTrue(self.f.ingest('rgb',self.health(2,4),4,4)['accepted'])
        with self.assertRaises(ValueError):self.f.ingest('rgb',self.motion(3,4),5,4)
    def test_unrequested_origin_and_delivery_rejected(self):
        self.begin();r=self.f.ingest('rgb',self.motion(1,2),1,1);self.assertEqual(r['reason'],'unrequested_origin')
        r=self.f.ingest('rgb',self.motion(0,1),2,4);self.assertEqual(r['reason'],'unrequested_delivery')
    def test_terminal_and_repeated_writes_cannot_freshen(self):
        self.begin();r=self.f.ingest('rgb',self.motion(0,1,'termination'),1,None);self.assertEqual(r['reason'],'non_processing_stage')
        e=self.motion(0,1);e['first_write_for_submission']=False;r=self.f.ingest('rgb',e,2,1);self.assertEqual(r['reason'],'not_first_write')
        self.assertEqual(self.f.cache.view('rgb',2)['status'],'UNKNOWN')
    def test_late_old_write_cannot_replace_new_failure(self):
        self.begin();self.f.ingest('rgb',self.health(0,1),1,1);self.f.plan(1,2);self.f.declare('rgb',2)
        self.f.ingest('rgb',self.health(1,2,False),3,2);self.f.ingest('rgb',self.motion(0,1),4,2)
        view=self.f.cache.view('rgb',4);self.assertFalse(view['value']);self.assertEqual(view['pair'],1)
    def test_prefix_replay_and_future_events_do_not_affect_prior_decision(self):
        self.begin();self.f.ingest('rgb',self.health(0,1),1,1);before=copy.deepcopy(self.f.plan(1,2))
        self.f.declare('rgb',2);self.f.ingest('rgb',self.motion(0,1),3,2)
        g=SparseFeedback(self.rows,'bounded_probe');g.plan(0,0)
        for c in ['rgb','thermal']:g.declare(c,1)
        g.ingest('rgb',self.health(0,1),1,1);self.assertEqual(before,g.plan(1,2))
    def test_declare_requires_request_and_increasing_source(self):
        with self.assertRaises(ValueError):self.f.declare('rgb',1)
        self.begin()
        with self.assertRaises(ValueError):self.f.declare('rgb',1)

if __name__=='__main__':unittest.main()
