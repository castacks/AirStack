"""Timing and validity counterexamples for the research accounting contract."""
import copy
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'tools'))
from decision_headroom_study import observe,summarize


def fixture():
 rows=[{c+'_left':{'time_ns':10*i} for c in ['rgb','thermal']} for i in range(5)]
 log=[{'kind':'release','camera':'rgb','source':3,'policy_requested':True,'delivered':True,'controller_monotonic_ns':101}]
 for kind,clock,seq in [('pair_end',105,0),('motion_result_written',120,1)]:
  event={'kind':kind,'time0_ns':20,'time1_ns':30,'stage':'pair_processing','execution_error':None,
         'original_need_interp':False,'first_write_for_submission':True,'derived_camera_delta_xyzw':[0,0,0,0,0,0,1],'sequence':seq}
  log.append({'kind':'observer_event','camera':'rgb','controller_monotonic_ns':clock,'trigger_source':4 if seq else 3,'delivery_group':4 if seq else 3,'event':event})
 return rows,log


class TimingTests(unittest.TestCase):
 def test_deadline_equality_allowed_but_future_not_known_at_decision(self):
  rows,log=fixture();r=observe(rows,log,2,'rgb',100,120)
  self.assertTrue(r['eligible_at_deadline']);self.assertFalse(r['eligible_at_decision'])
  self.assertEqual(r['motion_lag_after_decision_ns'],20)
 def test_late_motion_cannot_backfill(self):
  rows,log=fixture();log[-1]['controller_monotonic_ns']=121
  self.assertFalse(observe(rows,log,2,'rgb',100,120)['eligible_at_deadline'])
 def test_late_health_also_disqualifies(self):
  rows,log=fixture();log[1]['controller_monotonic_ns']=121
  self.assertFalse(observe(rows,log,2,'rgb',100,120)['eligible_at_deadline'])
 def test_shutdown_and_gap_cannot_count(self):
  for change in ['stage','gap']:
   rows,log=fixture()
   if change=='stage':log[-1]['event']['stage']='termination'
   else:log[1]['event']['time0_ns']=10
   self.assertFalse(observe(rows,log,2,'rgb',100,120)['eligible_at_deadline'])
 def test_native_failure_and_missing_motion(self):
  for change in ['failure','interp','missing','notfirst']:
   rows,log=fixture()
   if change=='failure':log[1]['event']['execution_error']='fail'
   elif change=='interp':log[-1]['event']['original_need_interp']=True
   elif change=='missing':log[-1]['event']['derived_camera_delta_xyzw']=None
   else:log[-1]['event']['first_write_for_submission']=False
   self.assertFalse(observe(rows,log,2,'rgb',100,120)['eligible_at_deadline'])
 def test_undelivered_or_later_release_cannot_count(self):
  for change in ['drop','late']:
   rows,log=fixture()
   if change=='drop':log[0]['delivered']=False
   else:log[0]['controller_monotonic_ns']=121
   self.assertFalse(observe(rows,log,2,'rgb',100,120)['eligible_at_deadline'])
 def test_other_camera_event_cannot_count(self):
  rows,log=fixture();log[-1]['camera']='thermal'
  self.assertFalse(observe(rows,log,2,'rgb',100,120)['eligible_at_deadline'])
 def test_partition_headroom_uses_complementarity_not_both_count(self):
  masks=['none','rgb_only','rgb_only','thermal_only','both'];rows=[]
  for i,mask in enumerate(masks):
   selected=i in [2,4];action='THERMAL' if i==1 else 'RGB' if selected else 'ABSTAIN'
   category='delivered' if selected else 'no_eligible_output' if mask=='none' else 'chosen_unavailable_with_alternate' if i==1 else 'abstain_with_opportunity'
   rows.append({'mask':mask,'selected_eligible':selected,'action':action,'loss_category':category,
                'experts':{c:{'eligible_at_decision':False} for c in ['rgb','thermal']}})
  s=summarize(rows,{'pair_calls':9});self.assertEqual(s['conditional_union_ceiling'],4)
  self.assertEqual(s['ceiling_above_best_constant'],1);self.assertEqual(s['union_minus_gate'],2)
  self.assertEqual(s['chosen_unavailable_with_alternate'],1);self.assertEqual(s['abstain_with_opportunity'],1)

if __name__=='__main__':unittest.main()
