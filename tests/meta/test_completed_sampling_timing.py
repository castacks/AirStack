import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from test_loop_timing import loop,truth

class SamplingTests(unittest.TestCase):
 def setUp(self):
  self.wall=1.;self.o=loop.LoopTiming(lambda:self.wall,lambda:self.wall/10);self.o.begin()
 def finish(self):
  self.wall+=.01;self.o.mark('truth_sample');self.o.finish()
 def test_tail_survives_nonsampling_loops_and_matches_capture(self):
  with self.o.span('record_write_flush'):self.wall+=.002
  self.o.note_sample_written('one',0);self.finish()
  for _ in range(5):self.o.begin();self.finish()
  s=self.o.snapshot(sampling_capture_id='one');tail=s['latest_sampling_completed']
  self.assertEqual(tail['sequence'],0);self.assertEqual(s['previous_completed']['sequence'],5)
  self.assertEqual(tail['sampling_record']['last_record_sequence'],0)
  self.assertIsNone(self.o.snapshot(sampling_capture_id='other')['latest_sampling_completed'])
  self.assertIsNone(self.o.snapshot(sampling_capture_id='one',sampling_since_sequence=0)['latest_sampling_completed'])
  tail['subphases'].clear();self.assertEqual(len(self.o.snapshot()['latest_sampling_completed']['subphases']),1)
 def test_latest_only_bound_and_stable_argmax_identity(self):
  for n in range(3):
   if n:self.o.begin()
   with self.o.span('write'):self.wall+=.005
   self.o.note_sample_written('one',n);self.finish()
  s=self.o.snapshot();self.assertEqual(s['sampling_retention_capacity'],1);self.assertEqual(s['sampling_loop_count'],3)
  self.assertEqual(s['latest_sampling_completed']['sampling_record']['last_record_sequence'],2)
  maximum=s['max_subphase_records']['write'];self.assertEqual(maximum['loop_sequence'],0);self.assertEqual(maximum['observer_id'],self.o.observer_id)
  self.assertEqual(maximum['wall_s'],s['max_subphase_wall_s']['write']);self.assertTrue(maximum['complete'])
 def test_marker_failure_disables_only_retention(self):
  self.o.note_sample_written('one',0);self.o.note_sample_written('two',1)
  self.assertFalse(self.o.sampling_retention_enabled);self.assertTrue(self.o.subphases_enabled)
  self.finish();self.assertIsNone(self.o.latest_sampling_completed)
 def test_real_capture_delivers_tail_and_final_status_and_reset(self):
  with TemporaryDirectory() as directory:
   folder=Path(directory);cap=truth.PhysicalTruthCapture(folder)
   def request(name): (folder/'capture.json').write_text(json.dumps({'capture_id':name,'enabled':True}))
   request('one')
   with patch.object(truth,'body_record',return_value={}):
    cap.sample(1.,{'body':SimpleNamespace()},now=10.,loop_observation=self.o);self.finish()
    for _ in range(4):self.o.begin();self.finish()
    self.o.begin();cap.sample(2.,{'body':SimpleNamespace()},now=11.,loop_observation=self.o)
    rows=[json.loads(x) for x in (folder/'one.jsonl').read_text().splitlines()];tail=rows[1]['loop_timing']['latest_sampling_completed']
    self.assertEqual(tail['sampling_record']['last_record_sequence'],0);self.assertEqual(cap.sampling_delivery_cursor,0)
    names={s['name'] for s in tail['subphases']};self.assertTrue({'record_encode','record_write_flush','status_write_replace'}<=names)
    before=cap.status(11.,self.o)['sampling_delivery'];self.assertIsNone(before['pending_completed_loop_sequence']);self.assertEqual(before['pending_written_loop_sequence'],5)
    self.finish();after=cap.status(12.,self.o)['sampling_delivery'];self.assertEqual(after['pending_completed_loop_sequence'],5)
    request('two');self.o.begin();cap.sample(3.,{'body':SimpleNamespace()},now=13.,loop_observation=self.o)
    first=json.loads((folder/'two.jsonl').read_text());self.assertIsNone(first['loop_timing']['latest_sampling_completed']);self.assertEqual(cap.sampling_delivery_cursor,-1)
   cap.close('test_end')
 def test_byte_limit_does_not_ack_or_mark_failed_record(self):
  with TemporaryDirectory() as directory:
   folder=Path(directory);cap=truth.PhysicalTruthCapture(folder);(folder/'capture.json').write_text(json.dumps({'capture_id':'one','enabled':True}))
   with patch.object(truth,'body_record',return_value={}):
    cap.sample(1.,{'body':SimpleNamespace()},now=10.,loop_observation=self.o);self.finish();self.o.begin();cap.max_bytes=cap.bytes
    cap.sample(2.,{'body':SimpleNamespace()},now=11.,loop_observation=self.o)
   self.assertEqual(cap.sampling_delivery_cursor,-1);self.assertEqual(cap.records,1);self.assertIsNone(self.o.current['sampling_record']);cap.close('end')

 def test_failed_record_write_preserves_pending_tail(self):
  with TemporaryDirectory() as directory:
   folder=Path(directory);cap=truth.PhysicalTruthCapture(folder);(folder/'capture.json').write_text(json.dumps({'capture_id':'one','enabled':True}))
   class BrokenFile:
    def write(self,_):raise OSError('failed write')
    def close(self):pass
   with patch.object(truth,'body_record',return_value={}):
    cap.sample(1.,{'body':SimpleNamespace()},now=10.,loop_observation=self.o);self.finish();self.o.begin()
    cap.file.close();cap.file=BrokenFile();cap.sample(2.,{'body':SimpleNamespace()},now=11.,loop_observation=self.o)
   self.assertEqual(cap.sampling_delivery_cursor,-1);self.assertEqual(cap.records,1)
   self.assertIsNone(self.o.current['sampling_record']);self.assertEqual(cap.last_written_sampling_loop,0)
 def test_observer_change_resets_delivery_cursor_after_success(self):
  with TemporaryDirectory() as directory:
   folder=Path(directory);cap=truth.PhysicalTruthCapture(folder);(folder/'capture.json').write_text(json.dumps({'capture_id':'one','enabled':True}))
   with patch.object(truth,'body_record',return_value={}):
    cap.sample(1.,{'body':SimpleNamespace()},now=10.,loop_observation=self.o);self.finish();self.o.begin()
    cap.sample(2.,{'body':SimpleNamespace()},now=11.,loop_observation=self.o)
    self.assertEqual(cap.sampling_delivery_cursor,0)
    new=loop.LoopTiming(lambda:self.wall,lambda:self.wall/10);new.begin()
    cap.sample(3.,{'body':SimpleNamespace()},now=12.,loop_observation=new)
   self.assertEqual(cap.sampling_delivery_cursor,-1);self.assertEqual(cap.sampling_delivery_observer_id,new.observer_id);cap.close('end')

 def test_final_status_exposes_retention_failure(self):
  with TemporaryDirectory() as directory:
   cap=truth.PhysicalTruthCapture(directory);cap.capture_id='one';cap.sampling_delivery_observer_id=self.o.observer_id
   self.o.note_sample_written('one',0);self.finish();cap.last_written_sampling_loop=0
   self.assertEqual(cap.status(10.,self.o)['sampling_delivery']['pending_completed_loop_sequence'],0)
   self.o.note_sample_written('one',1)
   status=cap.status(11.,self.o)['sampling_delivery']
   self.assertTrue(status['retention_available']);self.assertFalse(status['retention_enabled'])
   self.assertEqual(status['retention_error_count'],1);self.assertIsNotNone(status['retention_error'])
   self.assertIsNone(status['pending_completed_loop_sequence'])

if __name__=='__main__':unittest.main()
