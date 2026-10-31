"""Scoped receipt boundaries, explicit roots and immutable state evidence."""
from copy import deepcopy
import pytest
from Scripts.Setup.predecessor_state import validate_predecessor

POSE=[0.,0.,0.,0.,0.,0.,1.]
TIME=1761615394809864998


def fixture(root=False):
    events=[]
    def add(kind,**fields):
        i=len(events)
        event={'kind':kind,'sequence':i,**fields}
        item={'scope_id':'capture-A','camera':'rgb','clock_id':'controller',
              'sequence':i+10,'received_ns':i+30,'event':event}
        events.append(item);return item
    def motion(job,parent,target,first,last):
        return dict(submission_id=job,from_idx=parent,frame_idx=target,time0_ns=first,time1_ns=last,
                    stage='pair_processing')
    if not root:
        parent=motion(0,0,1,TIME-100,TIME)
        add('optimization_submitted',**parent)
        add('pair_end',stage='pair_processing',new_frame_idx=1,time0_ns=TIME-100,time1_ns=TIME,original_need_interp=False,execution_error=None)
        add('motion_result_written',**parent,first_write_for_submission=True,write_ordinal_for_submission=1,
            original_need_interp=False,parent_pose_camera_map_xyzw=POSE.copy(),committed_pose_camera_map_xyzw=POSE.copy())
    child=motion(1 if not root else 0,1 if not root else 0,2 if not root else 1,TIME,TIME+100)
    add('optimization_submitted',**child)
    add('pair_end',stage='pair_processing',new_frame_idx=child['frame_idx'],time0_ns=TIME,time1_ns=TIME+100,original_need_interp=False,execution_error=None)
    write=add('motion_result_written',**child,first_write_for_submission=True,write_ordinal_for_submission=1,
              original_need_interp=False,parent_pose_camera_map_xyzw=POSE.copy(),committed_pose_camera_map_xyzw=POSE.copy())
    args=dict(scope_id='capture-A',camera='rgb',clock_id='controller',deadline_ns=write['received_ns'],child=write,observed_events=events)
    init={'scope_id':'capture-A','camera':'rgb','clock_id':'controller','sequence':5,'received_ns':20,
          'event':{'kind':'graph_root_initialized','frame_idx':0,'time_ns':TIME,'pose_camera_map_xyzw':POSE.copy(),
                   'documented':True,'evidence_ref':'synthetic initialization witness'}}
    return args,init


def test_supported_evidence_exact_large_time_input_preservation_and_no_motion():
    args,_=fixture();before=deepcopy(args);result=validate_predecessor(**args)
    assert result['status']=='PASS_PREDECESSOR_EVIDENCE' and result['reason']=='supported_parent_with_observed_pose'
    assert result['source']['parent_time_ns']==TIME>2**53 and args==before
    assert result['physical_motion_admitted'] is False and 'action' not in result and 'delta' not in result


def test_root_identity_alone_is_insufficient_and_explicit_record_can_pass():
    args,init=fixture(root=True)
    assert validate_predecessor(**args)['reason']=='missing_root_initialization'
    result=validate_predecessor(**args,root_initialization=init)
    assert result['status']=='PASS_PREDECESSOR_EVIDENCE' and result['root_evidence_used'] is True
    assert result['source']['initialization_evidence_ref']==init['event']['evidence_ref']
    assert result['physical_motion_admitted'] is False


@pytest.mark.parametrize('fault,reason',[
    ('scope','root_scope_mismatch'),('camera','root_scope_mismatch'),('clock','root_clock_mismatch'),
    ('late','root_not_received_before_child'),('sequence','root_not_received_before_child'),
    ('conflict','root_journal_sequence_conflict'),('undocumented','undocumented_root_initialization'),
    ('truthy','undocumented_root_initialization'),('frame','root_identity_mismatch'),
    ('time','root_identity_mismatch'),('pose','root_pose_mismatch'),('empty','invalid_input')])
def test_root_rejections(fault,reason):
    args,init=fixture(root=True)
    if fault=='scope':init['scope_id']='capture-B'
    elif fault=='camera':init['camera']='thermal'
    elif fault=='clock':init['clock_id']='worker'
    elif fault=='late':init['received_ns']=args['child']['received_ns']+1
    elif fault=='sequence':init['sequence']=args['child']['sequence']+1
    elif fault=='conflict':init['sequence']=args['observed_events'][0]['sequence']
    elif fault=='undocumented':init['event'].pop('documented')
    elif fault=='truthy':init['event']['documented']=1
    elif fault=='frame':init['event']['frame_idx']=1
    elif fault=='time':init['event']['time_ns']+=1
    elif fault=='pose':init['event']['pose_camera_map_xyzw'][0]=1.
    else:init['event']['evidence_ref']=' '
    result=validate_predecessor(**args,root_initialization=init)
    assert result['status']=='REJECT_PREDECESSOR_EVIDENCE'
    # An absent schema field is invalid input, distinct from a false attestation.
    assert result['reason']==('invalid_input' if fault=='undocumented' else reason)


@pytest.mark.parametrize('fault,reason',[
    ('scope','journal_scope_mismatch'),('clock','journal_clock_mismatch'),('late','journal_after_deadline'),
    ('worker_sequence','invalid_worker_sequence'),('order','invalid_journal_order'),
    ('first','child_not_first_write'),('ordinal','child_not_first_write'),('shutdown','unsupported_child_state'),
    ('child_support','unsupported_child_state'),('submission','child_submission_mismatch'),
    ('parent_health_time','parent_health_identity_mismatch'),('support','unsupported_parent_health'),
    ('error','unsupported_parent_health'),('partial','incomplete_parent_health'),
    ('parent_write_time','parent_write_identity_mismatch'),('parent_stage','unsupported_parent_write'),
    ('pose','parent_pose_mismatch'),('bad_quaternion','invalid_input'),('nan','invalid_input'),
    ('boolean_time','invalid_input'),('boolean_parent','invalid_input'),('boolean_submission','invalid_input')])
def test_supported_path_rejections(fault,reason):
    args,_=fixture();events=args['observed_events'];child=args['child']['event']
    if fault=='scope':events[0]['scope_id']='other'
    elif fault=='clock':events[0]['clock_id']='worker'
    elif fault=='late':args['deadline_ns']-=1
    elif fault=='worker_sequence':events[0]['event']['sequence']=99
    elif fault=='order':events[1]['sequence']=events[0]['sequence']
    elif fault=='first':child['first_write_for_submission']=1
    elif fault=='ordinal':child['write_ordinal_for_submission']=2
    elif fault=='shutdown':child['stage']='shutdown'
    elif fault=='child_support':child['original_need_interp']=True
    elif fault=='submission':events[3]['event']['time0_ns']+=1
    elif fault=='parent_health_time':events[1]['event']['time1_ns']+=1
    elif fault=='support':events[1]['event']['original_need_interp']=True
    elif fault=='error':events[1]['event']['execution_error']='failure'
    elif fault=='partial':events[1]['event']['original_need_interp']=1
    elif fault=='parent_write_time':events[2]['event']['time1_ns']+=1
    elif fault=='parent_stage':events[2]['event']['stage']='shutdown'
    elif fault=='pose':events[2]['event']['committed_pose_camera_map_xyzw'][0]=1.
    elif fault=='bad_quaternion':child['parent_pose_camera_map_xyzw'][-1]=2.
    elif fault=='nan':child['parent_pose_camera_map_xyzw'][0]=float('nan')
    elif fault=='boolean_time':child['time0_ns']=True
    elif fault=='boolean_parent':child['from_idx']=True
    else:events[3]['event']['submission_id']=True
    result=validate_predecessor(**args)
    assert result['status']=='REJECT_PREDECESSOR_EVIDENCE' and result['reason']==reason


def test_later_parent_evidence_cannot_repair_child_time_state():
    args,_=fixture();events=args['observed_events'];parent=events[1]['event'];parent['new_frame_idx']=99
    later=deepcopy(events[1]);later.update(sequence=16,received_ns=36);later['event'].update(sequence=6,new_frame_idx=1)
    events.append(later);args['deadline_ns']=36
    assert validate_predecessor(**args)['reason']=='missing_parent_health_at_write'


def test_root_declaration_cannot_override_unsupported_nonroot():
    args,init=fixture();args['observed_events'][1]['event']['original_need_interp']=True
    assert validate_predecessor(**args,root_initialization=init)['reason']=='unsupported_parent_health'


def test_parent_repeat_writes_and_latest_snapshot_binding():
    args,_=fixture();events=args['observed_events'];repeat=deepcopy(events[2]);repeat['event'].update(first_write_for_submission=False,write_ordinal_for_submission=2)
    events.insert(3,repeat)
    for i,item in enumerate(events):item.update(sequence=i+10,received_ns=i+30);item['event']['sequence']=i
    args['deadline_ns']=events[-1]['received_ns']
    assert validate_predecessor(**args)['status']=='PASS_PREDECESSOR_EVIDENCE'
    repeat['event']['submission_id']=999
    assert validate_predecessor(**args)['reason']=='conflicting_parent_writes'


def test_duplicate_child_first_write_is_ambiguous():
    args,_=fixture();duplicate=deepcopy(args['child']);duplicate.update(sequence=16,received_ns=36);duplicate['event']['sequence']=6
    args['observed_events'].append(duplicate);args['deadline_ns']=36
    assert validate_predecessor(**args)['reason']=='ambiguous_child_first_write'


def test_no_file_reads(monkeypatch):
    import builtins
    args,_=fixture()
    monkeypatch.setattr(builtins,'open',lambda *a,**k:(_ for _ in ()).throw(AssertionError('file read')))
    assert validate_predecessor(**args)['status']=='PASS_PREDECESSOR_EVIDENCE'


@pytest.mark.parametrize('fault,reason',[
    ('parent_orphan','parent_submission_mismatch'),('parent_health_start','parent_health_identity_mismatch'),
    ('parent_health_stage','incomplete_parent_health'),('missing_parent_write','missing_parent_write_at_child'),
    ('duplicate_health','ambiguous_parent_health'),('duplicate_parent_first','ambiguous_parent_first_write'),
    ('unbound_child','child_not_bound_to_prefix'),('deadline_bool','invalid_input')])
def test_additional_identity_and_missing_state_rejections(fault,reason):
    args,_=fixture();events=args['observed_events']
    if fault=='parent_orphan':events[2]['event']['submission_id']=999
    elif fault=='parent_health_start':events[1]['event']['time0_ns']=TIME
    elif fault=='parent_health_stage':events[1]['event']['stage']='shutdown'
    elif fault=='missing_parent_write':events[2]['event']['kind']='diagnostic'
    elif fault in ('duplicate_health','duplicate_parent_first'):
        duplicate=deepcopy(events[1 if fault=='duplicate_health' else 2]);events.insert(3,duplicate)
        for i,e in enumerate(events):e.update(sequence=i+10,received_ns=i+30);e['event']['sequence']=i
        args['deadline_ns']=events[-1]['received_ns']
    elif fault=='unbound_child':args['child']=deepcopy(args['child']);args['child']['event']['committed_pose_camera_map_xyzw'][0]=1.
    else:args['deadline_ns']=True
    result=validate_predecessor(**args)
    assert result['status']=='REJECT_PREDECESSOR_EVIDENCE' and result['reason']==reason


def test_root_record_cannot_override_observed_conflicting_root_state():
    args,root=fixture(root=True);args['observed_events'][1]['event']['new_frame_idx']=0
    assert validate_predecessor(**args,root_initialization=root)['reason']=='conflicting_root_state'
