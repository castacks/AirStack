import copy
import math
import unittest
from rrm.evaluation_capture import GroundCompletion, grounded_snapshot, physical_rollover


def rows(t=100.0):
    return {k: {'receipt_monotonic_s':t,'message':m} for k,m in {
        'state': {'connected':True,'armed':False},
        'landed': {'landed_state':1}, 'armed': {'data':False},
        'authority': {'data':False},
        'odom': {'twist':{'twist':{'linear':{'x':0.,'y':0.,'z':0.}}}}
    }.items()}


class CapturePolicyTests(unittest.TestCase):
    def test_review_delay_does_not_finish_session_or_infer_ground(self):
        finish=GroundCompletion()
        for t in (100,220,400):
            self.assertFalse(finish.update(requested=False,mission_active=False,rows=rows(t),now=t))
        self.assertFalse(finish.update(requested=True,mission_active=False,rows=rows(401),now=401))
        self.assertTrue(finish.update(requested=True,mission_active=False,rows=rows(402),now=402))

    def test_explicit_finish_needs_inactive_mission_and_two_distinct_receipts(self):
        finish=GroundCompletion()
        self.assertFalse(finish.update(requested=True,mission_active=False,rows=rows(),now=100))
        self.assertFalse(finish.update(requested=True,mission_active=False,rows=rows(),now=100.5))
        self.assertFalse(finish.update(requested=True,mission_active=True,rows=rows(101),now=101))
        self.assertFalse(finish.update(requested=True,mission_active=None,rows=rows(102),now=102))
        self.assertFalse(finish.update(requested=True,mission_active=False,rows=rows(103),now=103))
        self.assertTrue(finish.update(requested=True,mission_active=False,rows=rows(104),now=104))

    def test_stale_future_missing_airborne_and_invalid_speed_cannot_finish(self):
        for mutation in ['stale','future','missing','armed','airborne','authority','disconnected','speed','nan']:
            with self.subTest(mutation=mutation):
                r=rows()
                if mutation=='stale':r['state']['receipt_monotonic_s']=97.9
                elif mutation=='future':r['odom']['receipt_monotonic_s']=101
                elif mutation=='missing':del r['landed']
                elif mutation=='armed':r['armed']['message']['data']=True
                elif mutation=='airborne':r['landed']['message']['landed_state']=2
                elif mutation=='authority':r['authority']['message']['data']=True
                elif mutation=='disconnected':r['state']['message']['connected']=False
                else:r['odom']['message']['twist']['twist']['linear']['x']=.2 if mutation=='speed' else math.nan
                self.assertFalse(grounded_snapshot(r,100))
                f=GroundCompletion();f.update(requested=True,mission_active=False,rows=rows(),now=100)
                self.assertFalse(f.update(requested=True,mission_active=False,rows=r,now=100))
                self.assertFalse(f.update(requested=True,mission_active=False,rows=rows(101),now=101))

    def test_land_rollover_deferral_has_hard_cap_margin(self):
        self.assertFalse(physical_rollover(elapsed_s=119,bytes_written=1,landing_active=False))
        self.assertTrue(physical_rollover(elapsed_s=120,bytes_written=1,landing_active=False))
        self.assertFalse(physical_rollover(elapsed_s=150,bytes_written=50*1024**2,landing_active=True))
        self.assertTrue(physical_rollover(elapsed_s=240,bytes_written=1,landing_active=True))
        self.assertTrue(physical_rollover(elapsed_s=121,bytes_written=56*1024**2,landing_active=True))

class LandingProtectionTests(unittest.TestCase):
    def test_active_land_protected_before_first_feedback_and_when_status_unknown(self):
        from rrm.evaluation_capture import landing_protection
        self.assertTrue(landing_protection({'active':True,'plan':{'actions':[{'kind':'LAND'}]},'events':[]}))
        self.assertTrue(landing_protection({'active':None,'observation_error':'timeout'}))
        self.assertFalse(landing_protection({'active':False,'plan':{'actions':[{'kind':'LAND'}]}}))

    def test_finalization_requires_success_complete_streams_and_frozen_hash(self):
        from rrm.evaluation_capture import validate_control_finalization
        good={'stop_reason':'interrupted','events':20,'missing_channels':[],
              'channels':{str(i):{} for i in range(18)},'recorder_sha256':'hash'}
        validate_control_finalization(0,good,'hash')
        for bad in ({'stop_reason':'error'},{'stop_reason':'event_limit'},
                    {'events':0},{'missing_channels':['state']},{'recorder_sha256':'changed'}):
            with self.assertRaises(RuntimeError):validate_control_finalization(0,{**good,**bad},'hash')
        with self.assertRaises(RuntimeError):validate_control_finalization(1,good,'hash')
