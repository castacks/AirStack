import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import unittest

root=Path(__file__).parents[2]/'simulation/isaac-sim/launch_scripts'
def module(name):
 spec=importlib.util.spec_from_file_location(name,root/(name+'.py'))
 m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
loop=module('loop_timing');truth=module('physical_truth')

class LoopTests(unittest.TestCase):
 def observer(self):
  self.wall=1.;self.cpu=.5
  return loop.LoopTiming(lambda:self.wall,lambda:self.cpu)
 def test_partial_complete_and_between_loop_timing(self):
  o=self.observer();phys=SimpleNamespace(count=5,last_receipt_wall_s=.9)
  o.begin(phys);self.wall+=.7;self.cpu+=.02;phys.count=6;phys.last_receipt_wall_s=1.69;o.mark('world_step',phys)
  partial=o.snapshot();self.assertFalse(partial['current_partial']['complete']);self.assertIsNone(partial['previous_completed'])
  phase=partial['current_partial']['phases'][0];self.assertAlmostEqual(phase['wall_s'],.7);self.assertAlmostEqual(phase['thread_cpu_s'],.02);self.assertEqual(phase['end']['physics_callback_count'],6)
  self.wall+=.01;self.cpu+=.001;o.mark('truth_sample',phys);o.finish();s=o.snapshot();self.assertIsNone(s['current_partial']);self.assertTrue(s['previous_completed']['complete']);self.assertEqual(s['slow_loop_count'],1)
  s['slow_loops'][0]['phases'].clear();self.assertEqual(len(o.snapshot()['slow_loops'][0]['phases']),2)
  self.wall+=.3;self.cpu+=.003;o.begin(phys);self.assertAlmostEqual(o.snapshot()['current_partial']['between_loops']['wall_s'],.3)
 def test_ring_is_bounded_and_dropped_count_explicit(self):
  o=self.observer()
  for _ in range(40):
   o.begin();self.wall+=.2;self.cpu+=.01;o.mark('world_step');o.finish()
  s=o.snapshot();self.assertEqual(s['slow_loop_count'],40);self.assertEqual(len(s['slow_loops']),32);self.assertEqual(s['slow_history_dropped'],8);self.assertEqual(s['slow_loops'][0]['sequence'],8)
 def test_invalid_clocks_and_missing_boundaries(self):
  o=self.observer();o.begin();self.wall-=1
  with self.assertRaises(ValueError):o.mark('world_step')
  with self.assertRaises(ValueError):o.finish()
  self.wall=float('nan')
  with self.assertRaises(ValueError):o.begin()
 def test_instances_have_separate_identity_history(self):
  a=self.observer();b=self.observer();self.assertNotEqual(a.observer_id,b.observer_id);self.assertIsNot(a.slow_loops,b.slow_loops)
 def test_callback_gap_history_resets_with_epoch(self):
  o=truth.ClockObservation()
  for n in range(40):
   with patch.object(truth.time,'monotonic',side_effect=[n*.2,n*.2+.001]):o.on_step(.01)
  s=o.snapshot();self.assertEqual(s['long_callback_gap_count'],39);self.assertEqual(len(s['long_callback_gaps']),32);self.assertAlmostEqual(s['long_callback_gaps'][-1]['wall_gap_s'],.2)
  o._reset();self.assertIsNone(o.last_receipt_wall_s);self.assertEqual(o.snapshot()['long_callback_gaps'],[])
 def test_invalid_callback_does_not_advance_receipt_or_counters(self):
  o=truth.ClockObservation(warn=lambda _:None)
  with patch.object(truth.time,'monotonic',side_effect=[1.,1.001]):o.on_step(float('nan'))
  self.assertIsNone(o.last_receipt_wall_s);self.assertEqual(o.count,0)

class IntegrationTests(unittest.TestCase):
 def test_new_slow_loop_after_partial_sample_is_retained_and_capture_resets(self):
  import tempfile,json
  wall=[1.]
  o=loop.LoopTiming(lambda:wall[0],lambda:wall[0]/10)
  with tempfile.TemporaryDirectory() as directory:
   root=Path(directory);cap=truth.PhysicalTruthCapture(root)
   (root/'capture.json').write_text(json.dumps({'capture_id':'first','enabled':True}))
   with patch.object(truth,'body_record',return_value={}):
    o.begin();wall[0]+=.2;o.mark('world_step')
    cap.sample(1.,{'/body':SimpleNamespace()},now=10.,loop_observation=o)
    first=json.loads((root/'first.jsonl').read_text())['loop_timing']
    self.assertFalse(first['current_partial']['complete']);self.assertEqual(first['slow_loops'],[])
    o.finish();o.begin();wall[0]+=.2;o.mark('world_step')
    cap.sample(2.,{'/body':SimpleNamespace()},now=11.,loop_observation=o)
    o.finish();o.begin();wall[0]+=.2;o.mark('world_step')
    cap.sample(3.,{'/body':SimpleNamespace()},now=12.,loop_observation=o)
    rows=[json.loads(x)['loop_timing'] for x in (root/'first.jsonl').read_text().splitlines()]
    self.assertEqual([r['sequence'] for r in rows[1]['slow_loops']],[0])
    self.assertEqual([r['sequence'] for r in rows[2]['slow_loops']],[1])
    (root/'capture.json').write_text(json.dumps({'capture_id':'second','enabled':True}))
    cap.sample(4.,{'/body':SimpleNamespace()},now=13.,loop_observation=o)
    self.assertEqual([r['sequence'] for r in json.loads((root/'second.jsonl').read_text())['loop_timing']['slow_loops']],[0,1])
   cap.close('test_end')
 def test_between_loop_stall_retained_even_with_short_engine_call(self):
  wall=[1.];o=loop.LoopTiming(lambda:wall[0],lambda:wall[0]/10)
  o.begin();wall[0]+=.01;o.mark('world_step');o.finish()
  wall[0]+=.7;o.begin();wall[0]+=.01;o.mark('world_step');o.finish()
  s=o.snapshot();self.assertEqual(s['slow_loop_count'],1);self.assertAlmostEqual(s['max_between_loop_wall_s'],.7)
  self.assertLess(s['slow_loops'][0]['wall_s'],.1)
 def test_actual_observer_error_boundary_disables_only_observation(self):
  import ast
  tree=ast.parse((root/'pegasus_app.py').read_text())
  function=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='observe')
  class Broken:
   def begin(self):raise RuntimeError('observer failure')
  owner=SimpleNamespace(loop_observation=Broken());warnings=[]
  env={'self':owner,'carb':SimpleNamespace(log_warn=warnings.append)}
  exec(compile(ast.Module(body=[function],type_ignores=[]),'actual_observe','exec'),env)
  env['observe']('begin');self.assertIsNone(owner.loop_observation);self.assertEqual(len(warnings),1)
  env['observe']('begin');self.assertEqual(len(warnings),1)

class BoundaryTests(unittest.TestCase):
 def test_epoch_change_labels_counts_without_fabricated_delta(self):
  wall=[1.];o=loop.LoopTiming(lambda:wall[0],lambda:wall[0]/10)
  a=SimpleNamespace(observer_id='old',count=10,last_receipt_wall_s=.9)
  b=SimpleNamespace(observer_id='new',count=0,last_receipt_wall_s=None)
  o.begin(a);wall[0]+=.01;o.mark('clock_bind',b)
  phase=o.snapshot()['current_partial']['phases'][0]
  self.assertEqual(phase['start']['physics_observer_id'],'old');self.assertEqual(phase['end']['physics_observer_id'],'new')
 def test_failed_write_does_not_consume_loop_history(self):
  import tempfile,json
  wall=[1.];o=loop.LoopTiming(lambda:wall[0],lambda:wall[0]/10)
  o.begin();wall[0]+=.2;o.mark('world_step');o.finish();o.begin()
  with tempfile.TemporaryDirectory() as directory:
   folder=Path(directory);cap=truth.PhysicalTruthCapture(folder,max_bytes=1)
   (folder/'capture.json').write_text(json.dumps({'capture_id':'limited','enabled':True}))
   with patch.object(truth,'body_record',return_value={}):cap.sample(1.,{'/body':SimpleNamespace()},now=10.,loop_observation=o)
   self.assertEqual(cap.records,0);self.assertEqual(cap.loop_history_cursor,-1)
   self.assertEqual([r['sequence'] for r in o.snapshot(since_sequence=cap.loop_history_cursor)['slow_loops']],[0])
   cap.close('test_end')
 def test_engine_exception_propagates_after_observer_failure(self):
  import ast
  tree=ast.parse((root/'pegasus_app.py').read_text())
  function=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='observe')
  body=next(n for n in ast.walk(tree) if isinstance(n,ast.While) and 'SIMULATION_APP.is_running()' in ast.unparse(n.test))
  class BrokenObserver:
   def begin(self,*args):raise ValueError('observer failed')
  class BrokenWorld:
   _scene=True
   def step(self,render):raise RuntimeError('engine failed')
  owner=SimpleNamespace(loop_observation=BrokenObserver(),clock_observation=None,stop_sim=False,_update_follow_cam=lambda:None)
  env={'self':owner,'carb':SimpleNamespace(log_warn=lambda _:None),'World':SimpleNamespace(instance=lambda:BrokenWorld()),'SIMULATION_APP':SimpleNamespace(is_running=lambda:True)}
  with self.assertRaisesRegex(RuntimeError,'engine failed'):
   exec(compile(ast.Module(body=[function,body],type_ignores=[]),'actual_loop','exec'),env)
  self.assertIsNone(owner.loop_observation)

if __name__=='__main__':unittest.main()
