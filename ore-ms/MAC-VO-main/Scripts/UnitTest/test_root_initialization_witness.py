"""Actual initialization observation, unchanged estimator behavior and receipt binding."""
from copy import deepcopy
import json
import numpy as np
import pytest

from Scripts.Setup.arbiter_event_recorder import ArbiterEventRecorder
from Scripts.Setup.predecessor_state import validate_predecessor
from Scripts.UnitTest.test_arbiter_event_recorder import System, Store, frame, events, IDENTITY


class InitializingSystem(System):
    def __init__(self):
        super().__init__()
        self.graph.frames.data = {'pose': Store(np.empty((0, 7))),
                                  'time_ns': Store(np.array([], dtype=np.int64)),
                                  'need_interp': Store(np.array([], dtype=bool))}
        self.initialize_calls = 0
        self.return_value = object()

    def initialize(self, first):
        self.initialize_calls += 1
        self.append(first.stereo.frame_ns, False)
        return self.return_value


TIME = 1761615394809864998


def record(tmp_path, enabled=True):
    system = InitializingSystem()
    rec = ArbiterEventRecorder(system, tmp_path, 'rgb', clock=iter(range(1000)).__next__,
                               record_root_initialization=enabled)
    assert system.initialize(frame(TIME)) is system.return_value
    return system, rec


def envelopes(rec):
    return [dict(scope_id='capture/root-A', camera='rgb', clock_id='controller',
                 sequence=i + 10, received_ns=i + 100, event=e)
            for i, e in enumerate(events(rec))]


def arguments(journal):
    child = next(e for e in journal if e['event']['kind'] == 'motion_result_written')
    return dict(scope_id='capture/root-A', camera='rgb', clock_id='controller',
                deadline_ns=journal[-1]['received_ns'], observed_events=journal, child=child)


def test_actual_initialization_copies_exact_time_pose_and_restores(tmp_path):
    system, rec = record(tmp_path/'events')
    root = events(rec)[0]
    assert root['kind'] == 'graph_root_initialized' and root['stage'] == 'initialization'
    assert root['time_ns'] == TIME > 2**53 and root['frame_idx'] == 0
    assert root['documented'] is True and root['physical_motion_admitted'] is False
    assert root['pose_camera_map_xyzw'] == IDENTITY.tolist()
    system.graph.frames.data['pose'].tensor[0, 0] = 8
    assert events(rec)[0]['pose_camera_map_xyzw'] == IDENTITY.tolist()
    assert rec.finish()['status'] == 'COMPLETE'
    assert system.initialize_calls == 1 and 'initialize' not in vars(system)


def test_streaming_receipt_binding_and_existing_validator(tmp_path):
    streamed = []
    class StreamingRecorder(ArbiterEventRecorder):
        def _emit(self, kind, **fields):
            super()._emit(kind, **fields)
            streamed.append(json.loads(self.events[-1]))
    system = InitializingSystem()
    rec = StreamingRecorder(system, tmp_path/'events', 'rgb', record_root_initialization=True)
    system.initialize(frame(TIME))
    system.run_pair(frame(TIME), frame(TIME+100))
    system.run_pair(frame(TIME+100), frame(TIME+200))  # delayed write during pair processing
    assert streamed == events(rec)
    journal = envelopes(rec); args = arguments(journal); before = deepcopy(journal)
    assert validate_predecessor(**args)['reason'] == 'missing_root_initialization'
    result = validate_predecessor(**args, root_initialization=journal[0])
    assert result['status'] == 'PASS_PREDECESSOR_EVIDENCE' and result['root_evidence_used']
    assert result['physical_motion_admitted'] is False and journal == before
    rec.finish()


@pytest.mark.parametrize('fault', ['different_pose', 'different_scope', 'future_receipt', 'duplicate_root'])
def test_conflicting_or_late_witness_never_passes(tmp_path, fault):
    system, rec = record(tmp_path/'events')
    system.run_pair(frame(TIME), frame(TIME+100)); system.run_pair(frame(TIME+100), frame(TIME+200))
    journal = envelopes(rec); root = deepcopy(journal[0])
    if fault == 'different_pose': root['event']['pose_camera_map_xyzw'][0] += 1
    elif fault == 'different_scope': root['scope_id'] = 'other'
    elif fault == 'future_receipt': root['received_ns'] = journal[-1]['received_ns'] + 1
    else:
        extra = deepcopy(root); extra['sequence'] -= 1
        journal.insert(0, extra)
        for i, item in enumerate(journal): item['event']['sequence'] = i
        root = journal[1]
    result = validate_predecessor(**arguments(journal), root_initialization=root)
    assert result['status'] == 'REJECT_PREDECESSOR_EVIDENCE'
    assert result['reason'] in ('root_journal_sequence_conflict', 'ambiguous_root_initialization')
    rec.finish()


def test_default_and_explicit_off_preserve_all_legacy_event_bytes(tmp_path):
    outputs=[]; logs=[]; calls=[]
    for mode in ('none', 'default', 'off', 'on'):
        system = InitializingSystem()
        kw = {} if mode == 'default' else dict(record_root_initialization=mode == 'on')
        rec = None if mode == 'none' else ArbiterEventRecorder(system, tmp_path/mode, 'rgb',
                                                    clock=iter(range(1000)).__next__, **kw)
        system.initialize(frame(TIME)); system.run_pair(frame(TIME), frame(TIME+100))
        system.run_pair(frame(TIME+100), frame(TIME+200)); system.terminate()
        outputs.append({k: (v.tensor.dtype, v.tensor.tobytes()) for k, v in system.graph.frames.data.items()})
        calls.append((system.initialize_calls, system.Optimizer.starts, system.Optimizer.write_calls))
        if rec:
            assert rec.finish()['status'] == 'COMPLETE'
            log = events(rec)
            if mode == 'on':
                log = log[1:]
                for e in log:
                    e['sequence'] -= 1; e['observation_monotonic_ns'] -= 1
            logs.append(log)
    assert outputs[0] == outputs[1] == outputs[2] == outputs[3]
    assert calls[0] == calls[1] == calls[2] == calls[3]
    assert logs[0] == logs[1] == logs[2]


@pytest.mark.parametrize('fault', ['preexisting', 'failed', 'two_frames', 'wrong_time', 'bad_pose'])
def test_non_initializations_emit_no_root_and_preserve_original_call(tmp_path, fault):
    system = InitializingSystem()
    if fault == 'preexisting': system.initialize(frame(TIME))
    original = system.initialize
    def altered(first):
        if fault == 'failed': raise RuntimeError('original init failure')
        result = original(first)
        if fault == 'two_frames': system.append(TIME+1, False)
        if fault == 'wrong_time': system.graph.frames.data['time_ns'].tensor[0] += 1
        if fault == 'bad_pose': system.graph.frames.data['pose'].tensor[0] = 0
        return result
    system.initialize = altered
    rec = ArbiterEventRecorder(system, tmp_path/'events', 'rgb', record_root_initialization=True)
    if fault == 'failed':
        with pytest.raises(RuntimeError, match='original init failure'): system.initialize(frame(TIME))
    else: assert system.initialize(frame(TIME)) is system.return_value
    assert not any(e['kind'] == 'graph_root_initialized' for e in events(rec))
    summary = rec.finish()
    if fault != 'failed': assert summary['status'] == 'OBSERVER_ERRORS'
    assert system.initialize is altered


def test_observer_clock_failure_does_not_change_initialization(tmp_path):
    def bad_clock(): raise OSError('clock failed')
    system = InitializingSystem()
    rec = ArbiterEventRecorder(system, tmp_path/'events', 'rgb', clock=bad_clock, record_root_initialization=True)
    assert system.initialize(frame(TIME)) is system.return_value
    assert system.initialize_calls == 1 and not events(rec)
    np.testing.assert_array_equal(system.graph.frames.data['pose'].tensor[0], IDENTITY)
    assert rec.finish()['status'] == 'OBSERVER_ERRORS'


def test_opt_in_requires_initialize_before_wrapping(tmp_path):
    system = System()
    with pytest.raises(TypeError, match='requires initialize'):
        ArbiterEventRecorder(system, tmp_path/'events', 'rgb', record_root_initialization=True)
    assert 'run_pair' not in vars(system) and not (tmp_path/'events').exists()


def test_native_macvo_initialize_preserves_every_graph_array(tmp_path):
    import torch
    import pypose as pp
    from types import MethodType, SimpleNamespace as NS
    from Odometry.MACVO import MACVO
    from Module.Map import VisualMap
    from Scripts.Setup.arbiter_event_recorder import array
    snapshots=[]; previous=[]; calls=[]
    # Deliberately nonidentity: the witness must record the initializer's pose,
    # rather than inventing identity from frame index zero.
    pose=torch.tensor([1.,2.,3.,0.,0.,0.,1.], dtype=torch.float32)
    for enabled in (False, True):
        system=InitializingSystem(); system.graph=VisualMap(); seen=[]
        depth=NS(depth=torch.ones((1,1,4,4)))
        system.Frontend=NS(estimate_depth=lambda stereo:(seen.append('depth'),depth)[1])
        system.MotionEstimator=NS(predict=lambda *args:(seen.append('predict'),pp.SE3(pose.clone()))[1])
        system.OutlierFilter.set_meta=lambda stereo:seen.append('metadata')
        system.initialize=MethodType(MACVO.initialize,system)
        first=frame(TIME)
        first.stereo.T_BS=pp.identity_SE3(1)
        first.stereo.K=torch.eye(3)[None]
        first.stereo.baseline=torch.tensor([.2])
        rec=ArbiterEventRecorder(system,tmp_path/str(enabled),'rgb',record_root_initialization=enabled)
        assert system.initialize(first) is None
        snapshots.append({k:(array(v.tensor).dtype,array(v.tensor).tobytes()) for k,v in system.graph.frames.data.items()})
        previous.append((system.prev_keyframe[0] is first,system.prev_keyframe[1],system.prev_keyframe[2] is depth))
        calls.append(seen)
        assert rec.finish()['status']=='COMPLETE'
        if enabled:
            root=events(rec)[0]
            assert root['pose_camera_map_xyzw']==pose.tolist() and root['time_ns']==TIME
        else: assert not events(rec)
    assert snapshots[0]==snapshots[1] and previous[0]==previous[1]
    assert calls[0]==calls[1]==['depth','predict','metadata']


def test_root_observation_does_not_change_advisory_actions_or_availability():
    from Scripts.UnitTest.test_pair_arbiter_events import fixture, resequence
    from Scripts.Setup.shared_arbiter_replay import advisory
    reports, streams=fixture()
    rows=reports['rgb']['input_rows']
    before=[advisory(rows,i,streams) for i in range(2)]
    for camera, stream in streams.items():
        stream.insert(0,dict(kind='graph_root_initialized',frame_idx=0,
                            time_ns=rows[0][camera+'_left']['time_ns'],
                            pose_camera_map_xyzw=IDENTITY.tolist(),documented=True,
                            evidence_ref='synthetic fixture'))
        resequence(stream)
    after=[advisory(rows,i,streams) for i in range(2)]
    for old,new in zip(before,after):
        assert old['action']==new['action'] and old['availability']==new['availability']
        for camera in streams:
            assert new['observed_event_counts'][camera]==old['observed_event_counts'][camera]+1
            evidence=new['evidence'][camera]
            evidence['health_event_sequence']-=1; evidence['motion_event_sequence']-=1
            assert old['evidence'][camera]==evidence
