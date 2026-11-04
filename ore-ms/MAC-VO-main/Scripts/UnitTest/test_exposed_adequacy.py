"""Research inventory semantics: deadlines, support, complementarity and runs."""
from pathlib import Path
import sys,unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'tools'))
from exposed_adequacy_study import native_pair,runs,count


def fixture():
 rows=[{'rgb_left':{'time_ns':i*10}} for i in range(4)];log=[]
 for kind,time,seq in [('pair_end',90,0),('motion_result_written',100,1)]:
  log.append({'kind':'observer_event','camera':'rgb','controller_monotonic_ns':time,'event':{'kind':kind,
    'time0_ns':10,'time1_ns':20,'stage':'pair_processing','sequence':seq,'first_write_for_submission':True,
    'original_need_interp':False,'execution_error':None,'derived_camera_delta_xyzw':[0,0,0,0,0,0,1]}})
 return rows,log

class AdequacyTests(unittest.TestCase):
 def test_exact_deadline_and_late(self):
  rows,log=fixture();self.assertTrue(native_pair(rows,log,1,'rgb',100)['eligible'])
  self.assertFalse(native_pair(rows,log,1,'rgb',99)['eligible'])
 def test_terminal_cannot_fill(self):
  rows,log=fixture();log[-1]['event']['stage']='termination'
  self.assertFalse(native_pair(rows,log,1,'rgb',100)['eligible'])
 def test_gap_cannot_fill(self):
  rows,log=fixture();log[-1]['event']['time0_ns']=0
  self.assertFalse(native_pair(rows,log,1,'rgb',100)['eligible'])
 def test_supported_without_motion_is_not_available(self):
  rows,log=fixture();r=native_pair(rows,log[:-1],1,'rgb',100)
  self.assertTrue(r['supported']);self.assertFalse(r['eligible'])
 def test_native_failed_or_interpolated(self):
  for field,value in [('execution_error','failed'),('original_need_interp',True)]:
   rows,log=fixture();log[0]['event'][field]=value
   self.assertFalse(native_pair(rows,log,1,'rgb',100)['eligible'])
 def test_one_dominating_camera_has_zero_headroom(self):
  r=count([{'mask':m} for m in ['thermal_only','thermal_only','both','none']])
  self.assertEqual(r['headroom_above_best_constant'],0);self.assertEqual(r['union_eligible'],3)
 def test_complementarity_not_joint_support_supplies_headroom(self):
  r=count([{'mask':m} for m in ['rgb_only','rgb_only','thermal_only','both']])
  self.assertEqual(r['headroom_above_best_constant'],1)
 def test_runs_are_local_and_single_pair_return_is_not_sustained(self):
  a=runs([True]*3+[False]*3+[True]);self.assertEqual(len(a),3)
  self.assertFalse(all(x['end_pair']-x['start_pair']+1>=3 for x in a))
  b=runs([True]*3);self.assertEqual(b[0]['start_pair'],0)

if __name__=='__main__':unittest.main()
