"""Earlier observation order, native preservation, pending-state and prefix guards."""
import copy,json
import numpy as np
import pytest
from Scripts.Setup.early_signal_events import EarlySignalsMixin
from Scripts.Setup.arbiter_event_recorder import ArbiterEventRecorder
from Scripts.Setup.early_availability import snapshot,choose
from Scripts.UnitTest.test_arbiter_event_recorder import System,frame,events
class EarlyTestRecorder(EarlySignalsMixin,ArbiterEventRecorder):pass

def test_observer_order_native_results_and_detached_values(tmp_path):
 a,b=System(),System();b.min_num_point=10;rec=EarlyTestRecorder(b,tmp_path/'early','rgb',clock=iter(range(1000)).__next__)
 for system in [a,b]:system.run_pair(frame(10),frame(20));system.run_pair(frame(20),frame(30,'unsupported'))
 out=events(rec);kinds=[e['kind'] for e in out];cov=[e for e in out if e['kind']=='early_covariance_available'];health=[e for e in out if e['kind']=='early_tracking_available'];assert len(cov)==len(health)==2
 assert [e['tracking_count_threshold_met'] for e in health]==[True,False] and all(e['native_min_num_point']==10 for e in health)
 for c,h,end in zip(cov,health,[e for e in out if e['kind']=='pair_end']):assert c['sequence']<h['sequence']<end['sequence'] and h['tracking_count_threshold_met']==(not end['original_need_interp'])
 before=copy.deepcopy(cov[0]);b.match.cov[:]=999;assert cov[0]==before
 for k in ['pose','need_interp','time_ns']:np.testing.assert_array_equal(a.graph.frames.data[k].tensor,b.graph.frames.data[k].tensor)
 assert a.Optimizer.starts==b.Optimizer.starts and a.Optimizer.write_calls==b.Optimizer.write_calls
 rec.finish();assert 'run_pair' not in vars(b)

def test_return_identity_and_error_isolation(tmp_path):
 s=System();s.min_num_point=10;rec=EarlyTestRecorder(s,tmp_path/'events','rgb');original=s.KeypointSelector.select_point;result=original(frame(10).stereo,12,None,None,s.match);rec.current={'time0_ns':10,'time1_ns':20};returned=rec._select(lambda *a,**k:result,frame(10).stereo,12,None,None,s.match);assert returned is result
 rec._emit=lambda *a,**k:(_ for _ in ()).throw(ValueError('observation failure'))
 assert rec._select(lambda *a,**k:result,frame(10).stereo,12,None,None,s.match) is result
 assert any(e['operation']=='early_covariance' for e in rec.errors)

def test_original_exception_not_suppressed(tmp_path):
 s=System();s.min_num_point=10;rec=EarlyTestRecorder(s,tmp_path/'events','thermal')
 with pytest.raises(RuntimeError,match='original estimator failure'):s.run_pair(frame(10),frame(20,'error'))
 assert next(e for e in events(rec) if e['kind']=='pair_end')['execution_error']=='RuntimeError'

def fixture():
 rows=[{c+'_left':{'time_ns':i*100} for c in ['rgb','thermal']} for i in range(5)];timeline=[]
 def add(cam,event,receipt):
  event=dict(event,sequence=sum(e.get('camera')==cam for e in timeline));timeline.append({'kind':'observer_event','camera':cam,'event':event,'controller_monotonic_ns':receipt})
 for i in range(3):
  for cam in ['rgb','thermal']:
   add(cam,{'kind':'pair_end','time0_ns':i*100,'time1_ns':(i+1)*100,'original_need_interp':False,'execution_error':None},100+i*100)
   if i<2:add(cam,{'kind':'motion_result_written','time0_ns':i*100,'time1_ns':(i+1)*100,'first_write_for_submission':True,'original_need_interp':False,'derived_camera_delta_xyzw':[0,0,0,0,0,0,1],'stage':'pair_processing'},120+i*100)
 return rows,timeline

def test_pending_keeps_previous_resolved_true_and_future_motion_ignored():
 rows,timeline=fixture();a=snapshot(rows,2,timeline)
 assert a['states']['rgb']['resolved'] is True and a['provenance']['rgb']['resolved']['pair']==1
 assert a['provenance']['rgb']['supported_pending_intervals_skipped']==1
 timeline.append({'kind':'observer_event','camera':'rgb','controller_monotonic_ns':999,'event':{'kind':'motion_result_written','first_write_for_submission':True,'time0_ns':200,'time1_ns':300,'derived_camera_delta_xyzw':None,'stage':'pair_processing','original_need_interp':False}})
 assert snapshot(rows,2,timeline)==a

def test_unsupported_resolves_false_before_any_motion():
 rows,timeline=fixture()
 for e in timeline:
  if e['camera']=='rgb' and e['event']['kind']=='pair_end' and e['event']['time0_ns']==200:e['event']['original_need_interp']=True
 a=snapshot(rows,2,timeline);assert a['states']['rgb']=={'health':False,'confirmed':False,'resolved':False};assert a['actions']['resolved']=='THERMAL'

def test_pending_without_past_result_is_unknown_not_failure():
 rows,timeline=fixture();timeline=[e for e in timeline if e['event']['kind']!='motion_result_written'];a=snapshot(rows,2,timeline);assert a['states']['rgb']['resolved'] is None and a['actions']['resolved']=='ABSTAIN'
 assert choose({'rgb':None,'thermal':True})=='THERMAL'

def test_receipt_max_and_duplicate_guard():
 rows,timeline=fixture();a=snapshot(rows,2,timeline);assert a['epoch_ns']==300
 duplicate=copy.deepcopy(next(e for e in timeline if e['camera']=='rgb' and e['event']['kind']=='pair_end' and e['event']['time0_ns']==200));timeline.append(duplicate)
 with pytest.raises(ValueError,match='unique'):snapshot(rows,2,timeline)
