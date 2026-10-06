import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from test_loop_timing import loop, truth

class SubphaseTests(unittest.TestCase):
 def setUp(self):
  self.wall=1.;self.cpu=.1
  self.o=loop.LoopTiming(lambda:self.wall,lambda:self.cpu);self.o.begin()
 def test_partial_completion_separate_from_outer_phases(self):
  with self.o.span('body_read'):
   partial=self.o.snapshot();self.assertFalse(partial['current_partial']['active_subphase']['complete'])
   self.wall+=.5;self.cpu+=.001
  self.o.mark('truth_sample');self.o.finish();s=self.o.snapshot();r=s['previous_completed']
  self.assertAlmostEqual(r['subphases'][0]['wall_s'],.5);self.assertAlmostEqual(r['subphases'][0]['thread_cpu_s'],.001)
  self.assertEqual(len(r['phases']),1);self.assertEqual(s['max_subphase_wall_s'],{'body_read':r['subphases'][0]['wall_s']})
  r['subphases'].clear();self.assertEqual(len(self.o.snapshot()['previous_completed']['subphases']),1)
 def test_bounded_history_and_exact_counts(self):
  for _ in range(40):
   with self.o.span('request_poll'):self.wall+=.001
  s=self.o.snapshot();r=s['current_partial'];self.assertEqual(len(r['subphases']),32);self.assertEqual(r['subphase_history_dropped'],8)
  self.assertEqual(r['subphase_count'],40);self.assertEqual(s['subphase_count'],40)
 def test_original_operation_error_is_preserved(self):
  with self.assertRaisesRegex(OSError,'original'):
   with self.o.span('write'):self.wall+=.2;raise OSError('original')
  self.assertTrue(self.o.snapshot()['current_partial']['subphases'][0]['operation_failed'])
 def test_start_clock_failure_runs_body_once_and_disables_only_measurement(self):
  calls=[];self.o.monotonic=lambda:float('nan')
  with self.o.span('write'):calls.append(1)
  with self.o.span('write'):calls.append(2)
  s=self.o.snapshot();self.assertEqual(calls,[1,2]);self.assertFalse(s['subphases_enabled']);self.assertEqual(s['subphase_error_count'],1)
 def test_end_clock_failure_does_not_replace_operation_error(self):
  with self.assertRaisesRegex(OSError,'original'):
   with self.o.span('write'):self.o.thread_cpu=lambda:float('nan');raise OSError('original')
  s=self.o.snapshot();self.assertFalse(s['subphases_enabled']);self.assertIsNone(s['current_partial']['active_subphase'])
 def test_capture_visibility_and_status_write_failure(self):
  with TemporaryDirectory() as directory:
   folder=Path(directory);cap=truth.PhysicalTruthCapture(folder)
   (folder/'capture.json').write_text(json.dumps({'capture_id':'spans','enabled':True}))
   with patch.object(truth,'body_record',return_value={}):
    cap.sample(1.,{'body':SimpleNamespace()},now=10.,loop_observation=self.o)
    row=json.loads((folder/'spans.jsonl').read_text())
    active=row['loop_timing']['current_partial']['active_subphase'];self.assertEqual(active['name'],'loop_snapshot');self.assertFalse(active['complete'])
    names=[r['name'] for r in self.o.current['subphases']]
    self.assertEqual(names,['request_poll','body_read','clock_snapshot','backend_metadata','loop_snapshot','record_encode','record_write_flush','status_build_encode','status_write_replace'])
    self.o.mark('truth_sample');self.o.finish();self.o.begin()
    with patch.object(Path,'replace',side_effect=OSError('status blocked')):
     cap.sample(2.,{'body':SimpleNamespace()},now=11.,loop_observation=self.o)
    rows=[json.loads(x) for x in (folder/'spans.jsonl').read_text().splitlines()]
    self.assertEqual(len(rows),2);self.assertEqual(len(rows[-1]['loop_timing']['previous_completed']['subphases']),9)
    self.assertTrue(self.o.current['subphases'][-1]['operation_failed']);self.assertEqual(cap.records,2)
   cap.close('test_end')
 def test_disabled_capture_and_byte_cap_do_not_write_records(self):
  with TemporaryDirectory() as directory:
   folder=Path(directory);cap=truth.PhysicalTruthCapture(folder,max_bytes=1)
   with patch.object(truth,'body_record',return_value={}) as body:
    cap.sample(1.,{},now=10.,loop_observation=self.o);body.assert_not_called()
    (folder/'capture.json').write_text(json.dumps({'capture_id':'limited','enabled':True}))
    cap.sample(2.,{'body':SimpleNamespace()},now=12.,loop_observation=self.o)
   self.assertEqual(cap.records,0);self.assertEqual(cap.loop_history_cursor,-1)
   self.assertNotIn('record_write_flush',[r['name'] for r in self.o.current['subphases']]);cap.close('test_end')

 def test_record_write_failure_is_contained_without_consuming_history(self):
  with TemporaryDirectory() as directory:
   folder=Path(directory);cap=truth.PhysicalTruthCapture(folder)
   (folder/'capture.json').write_text(json.dumps({'capture_id':'failed','enabled':True}))
   cap._poll(10.)
   class BrokenFile:
    def write(self,_):raise OSError('record blocked')
    def close(self):pass
   cap.file.close();cap.file=BrokenFile()
   with patch.object(truth,'body_record',return_value={}):
    cap.sample(1.,{'body':SimpleNamespace()},now=11.,loop_observation=self.o)
   self.assertEqual(cap.records,0);self.assertEqual(cap.loop_history_cursor,-1)
   self.assertIsNone(cap.file);self.assertEqual(cap.last_error,'record blocked')
   record=next(r for r in self.o.current['subphases'] if r['name']=='record_write_flush')
   self.assertTrue(record['operation_failed'])
 def test_duration_cap_preserved(self):
  with TemporaryDirectory() as directory:
   folder=Path(directory);cap=truth.PhysicalTruthCapture(folder,max_duration_s=1.)
   (folder/'capture.json').write_text(json.dumps({'capture_id':'duration','enabled':True}))
   cap._poll(10.)
   with patch.object(truth,'body_record') as body:
    cap.sample(1.,{'body':SimpleNamespace()},now=12.,loop_observation=self.o);body.assert_not_called()
   self.assertIsNone(cap.file);self.assertEqual(cap.records,0)

if __name__=='__main__':unittest.main()
