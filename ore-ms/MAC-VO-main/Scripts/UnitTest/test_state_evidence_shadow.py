"""Eligibility intersection, bound evidence, explicit fallbacks and fail-closed context."""
from copy import deepcopy
from pathlib import Path
import pytest
from Scripts.Setup.state_evidence_shadow import state_evidence_advisory
from Scripts.UnitTest.test_predecessor_state import fixture, TIME


def inputs(root=False):
 journals={};roots={}
 for ci,camera in enumerate(('rgb','thermal')):
  args,init=fixture(root=root)
  for item in args['observed_events']:
   item.update(camera=camera,sequence=item['sequence']*2+ci,received_ns=item['received_ns']*2+ci)
   event=item['event'];event.update(modality=camera,observation_monotonic_ns=event['sequence'])
   if event['kind']=='pair_end':
    event.update(selected_count=12,inbound_count=11,retained_count=10,occupied_3x3_cells=7,
                 grid_count=20,grid_positive_uu_vv_count=20,grid_trace_q10_q50_q90_px2=[1.,2.,3.],motion_status='submitted')
   elif event['kind']=='motion_result_written':event.update(derived_camera_delta_xyzw=[0.,0.,0.,0.,0.,0.,1.],derived_delta_unavailable_reason=None)
  journals[camera]=args['observed_events']
  init.update(camera=camera,sequence=init['sequence']*2+ci,received_ns=init['received_ns']*2+ci);roots[camera]=init
 rows=[{c+'_left':{'time_ns':t} for c in journals} for t in (TIME,TIME+100)]
 return dict(rows=rows,pair=0,journals=journals,scope_id='capture-A',clock_id='controller',deadline_ns=100,root_initializations=roots if root else None)


def parent_unsupported(args,camera):
 health=next(e['event'] for e in args['journals'][camera] if e['event']['kind']=='pair_end')
 health['original_need_interp']=True


@pytest.mark.parametrize('rgb,thermal,action',[(True,True,'RGB'),(True,False,'RGB'),(False,True,'THERMAL'),(False,False,'ABSTAIN')])
def test_eligibility_intersection_truth_table(rgb,thermal,action):
 args=inputs()
 for c,ok in [('rgb',rgb),('thermal',thermal)]:
  if not ok:parent_unsupported(args,c)
 before=deepcopy(args);r=state_evidence_advisory(**args)
 assert r['action']==action and r['raw_action']=='RGB' and r['raw_availability']=={'rgb':True,'thermal':True}
 assert r['evidence_eligible']=={'rgb':rgb,'thermal':thermal} and args==before
 assert r['shadow_only'] is True and r['physical_motion_admitted'] is False and r['reference_or_loss_used'] is False
 assert 'motion' not in r and 'delta' not in r


def test_root_not_inferred_and_explicit_documentation_changes_only_eligibility():
 args=inputs(root=True);args['root_initializations']=None
 r=state_evidence_advisory(**args);assert r['action']=='ABSTAIN'
 assert all(v['reason']=='missing_root_initialization' for v in r['predecessor_evidence'].values())
 args=inputs(root=True);r=state_evidence_advisory(**args)
 assert r['action']=='RGB' and all(v['root_evidence_used'] for v in r['predecessor_evidence'].values())


@pytest.mark.parametrize('fault',['pose','late','scope','clock','time','undocumented'])
def test_invalid_preferred_root_falls_back_to_valid_thermal(fault):
 args=inputs(root=True);root=args['root_initializations']['rgb']
 if fault=='pose':root['event']['pose_camera_map_xyzw'][0]=1
 elif fault=='late':root['received_ns']=100
 elif fault=='scope':root['scope_id']='other'
 elif fault=='clock':root['clock_id']='worker'
 elif fault=='time':root['event']['time_ns']+=1
 else:root['event']['documented']=False
 r=state_evidence_advisory(**args)
 assert r['raw_action']=='RGB' and r['action']=='THERMAL' and r['evidence_eligible']=={'rgb':False,'thermal':True}


def test_raw_unavailable_expert_is_not_promoted_by_valid_root():
 args=inputs(root=True)
 for camera in args['journals']:
  next(e['event'] for e in args['journals'][camera] if e['event']['kind']=='pair_end')['original_need_interp']=True
 r=state_evidence_advisory(**args)
 assert r['action']=='ABSTAIN' and not any(r['evidence_eligible'].values())
 assert all(v['reason']=='raw_unavailable' for v in r['predecessor_evidence'].values())


def test_exact_child_parent_pose_mismatch_cannot_reuse_earlier_pass():
 args=inputs();assert state_evidence_advisory(**args)['action']=='RGB'
 args['journals']['rgb'][-1]['event']['parent_pose_camera_map_xyzw'][0]=1
 r=state_evidence_advisory(**args)
 assert r['action']=='THERMAL' and r['predecessor_evidence']['rgb']['reason']=='parent_pose_mismatch'


def test_explicit_root_cannot_override_unsupported_nonroot():
 args=inputs();_,roots=fixture(root=True);args['root_initializations']={'rgb':roots}
 parent_unsupported(args,'rgb');r=state_evidence_advisory(**args)
 assert r['action']=='THERMAL' and r['predecessor_evidence']['rgb']['reason']=='unsupported_parent_health'
 assert not r['predecessor_evidence']['rgb']['root_evidence_used']


@pytest.mark.parametrize('fault',['scope','camera','clock','late','receipt','sequence','worker','modality','bool_deadline','float_deadline','negative_deadline','pair','missing_camera','roots','controller_collision','global_receipt'])
def test_invalid_context_raises_without_advice(fault):
 args=inputs();item=args['journals']['rgb'][0]
 if fault=='scope':item['scope_id']='other'
 elif fault=='camera':item['camera']='thermal'
 elif fault=='clock':item['clock_id']='worker'
 elif fault=='late':args['deadline_ns']=0
 elif fault=='receipt':item['received_ns']=-1
 elif fault=='sequence':item['sequence']=True
 elif fault=='worker':item['event']['sequence']=1
 elif fault=='modality':item['event']['modality']='thermal'
 elif fault=='bool_deadline':args['deadline_ns']=True
 elif fault=='float_deadline':args['deadline_ns']=100.
 elif fault=='negative_deadline':args['deadline_ns']=-1
 elif fault=='pair':args['pair']=True
 elif fault=='missing_camera':args['journals'].pop('thermal')
 elif fault=='roots':args['root_initializations']={'other':{}}
 elif fault=='controller_collision':args['journals']['thermal'][0]['sequence']=item['sequence']
 else:args['journals']['thermal'][0]['received_ns']=item['received_ns']-1
 with pytest.raises(ValueError):state_evidence_advisory(**args)


def test_root_already_in_prefix_explicit_binding_and_input_preservation():
 args=inputs(root=True)
 for camera,journal in args['journals'].items():
  root=args['root_initializations'][camera];root['event'].update(sequence=0,modality=camera,observation_monotonic_ns=0)
  journal.insert(0,root)
  for i,e in enumerate(journal):e['event']['sequence']=i;e['event']['observation_monotonic_ns']=i
 before=deepcopy(args);r=state_evidence_advisory(**args)
 assert r['action']=='RGB' and args==before
 assert all(v['source']['parent_time_ns']==TIME>2**53 for v in r['predecessor_evidence'].values())


def test_function_reads_no_files(monkeypatch):
 args=inputs()
 def deny(*a,**kw):raise AssertionError('file read in pure boundary')
 monkeypatch.setattr(Path,'read_text',deny);monkeypatch.setattr(Path,'read_bytes',deny)
 monkeypatch.setattr('builtins.open',deny)
 assert state_evidence_advisory(**args)['action']=='RGB'
