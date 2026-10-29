"""Causal event ordering, stale results, failure isolation and numerical parity."""
import json
from types import SimpleNamespace as NS

import numpy as np
import pytest

from Scripts.Setup.arbiter_event_recorder import ArbiterEventRecorder, relative_pose


IDENTITY = np.array([0., 0., 0., 0., 0., 0., 1.])


class Store:
    def __init__(self, value):
        self.tensor = np.array(value, copy=True)

    def __getitem__(self, key):
        return self.tensor[key]

    def __setitem__(self, key, value):
        from Scripts.Setup.arbiter_event_recorder import array
        self.tensor[key] = array(value)


class Optimizer:
    def __init__(self):
        self.pending = None
        self.starts = 0
        self.write_calls = 0

    def start_optimize(self, data):
        self.starts += 1
        target = IDENTITY.copy();target[0] = float(data.frame_idx)
        self.pending = NS(from_idx=data.from_idx, frame_idx=data.frame_idx, motion=target[None])

    def write_graph_data(self, result, graph):
        self.write_calls += 1
        if result is not None:
            graph.frames.data['pose'][result.frame_idx] = result.motion[0]

    def write_map(self, graph):
        return self.write_graph_data(self.pending, graph)

    def get_result(self):
        raise AssertionError('observer must not consume/poll optimizer results')


class Selector:
    def select_point(self, frame, count, depth0, depth1, match):
        return np.array([[8,8],[16,8],[24,8],[8,16],[16,16],[24,16],[8,24],[16,24],[24,24],[10,10],[12,12],[14,14]])[:count]


class Filter:
    def filter(self, observations, device):
        return np.ones(len(observations.data['pixel1_uv']), bool)


class Refiner:
    def elaborate_map(self, frames):
        frames.data['pose'].tensor[:, :3] += 1000
        frames.data['need_interp'].tensor[:] = False
        return frames


def frame(time_ns, mode='supported'):
    return NS(stereo=NS(frame_ns=time_ns, height=32, width=32), mode=mode)


class System:
    def __init__(self, start=10):
        self.graph = NS(frames=NS(data={'pose': Store([IDENTITY]), 'time_ns': Store([start]),
            'need_interp': Store(np.array([False]))}))
        self.Optimizer, self.KeypointSelector, self.OutlierFilter, self.MapRefiner = Optimizer(), Selector(), Filter(), Refiner()
        self.edge_width = 4
        self.terminated = False
        self.match = NS(cov=np.stack([np.full((32,32),2.),np.full((32,32),3.),np.full((32,32),.1)])[None])

    def append(self, time_ns, unsupported):
        values = self.graph.frames.data
        values['pose'].tensor = np.vstack([values['pose'].tensor, IDENTITY])
        values['time_ns'].tensor = np.append(values['time_ns'].tensor, time_ns)
        values['need_interp'].tensor = np.append(values['need_interp'].tensor, unsupported)

    def make_graph_data(self, index):
        return NS(from_idx=index-1, frame_idx=index)

    def run_pair(self, first, second):
        if second.mode == 'skip':
            self.append(second.stereo.frame_ns, True)
            return
        self.Optimizer.write_map(self.graph)
        points = self.KeypointSelector.select_point(first.stereo, 0 if second.mode=='unsupported' else 12, None, None, self.match)
        self.OutlierFilter.filter(NS(data={'pixel1_uv':points}), None)
        if second.mode == 'error':
            raise RuntimeError('original estimator failure')
        self.append(second.stereo.frame_ns, second.mode=='unsupported')
        if second.mode != 'unsupported':
            self.Optimizer.start_optimize(self.make_graph_data(len(self.graph.frames.data['time_ns'].tensor)-1))

    def terminate(self):
        self.terminated = True
        self.Optimizer.write_map(self.graph)
        self.MapRefiner.elaborate_map(self.graph.frames)


def events(recorder):
    return [json.loads(value) for value in recorder.events]


def test_delayed_write_and_last_flush_precede_refinement(tmp_path):
    system = System();rec = ArbiterEventRecorder(system,tmp_path/'events','rgb',clock=iter(range(1000)).__next__)
    system.run_pair(frame(10),frame(20));assert not [e for e in events(rec) if e['kind']=='motion_result_written']
    system.run_pair(frame(20),frame(30));system.terminate();summary=rec.finish();out=events(rec)
    writes=[e for e in out if e['kind']=='motion_result_written']
    assert [(e['time0_ns'],e['time1_ns'],e['stage']) for e in writes]==[(10,20,'pair_processing'),(20,30,'termination')]
    assert writes[0]['sequence'] > [e for e in out if e['kind']=='pair_end'][0]['sequence']
    assert writes[-1]['sequence'] < next(e['sequence'] for e in out if e['kind']=='refinement_begin')
    assert writes[-1]['committed_pose_camera_map_xyzw'][0]==2 and system.graph.frames.data['pose'].tensor[-1,0]==1002
    assert writes[-1]['derived_camera_delta_xyzw'][0]==1
    assert all(e['optimizer_convergence'] is None and e['pose_covariance'] is None for e in writes)
    assert summary['status']=='COMPLETE' and summary['optimization_submissions']==2
    assert 'run_pair' not in vars(system)  # restore inherited method exactly
    assert (tmp_path/'events/events.jsonl').read_text().count('\n')==len(out)


def test_stale_write_not_a_new_submission(tmp_path):
    system=System();rec=ArbiterEventRecorder(system,tmp_path/'events','rgb')
    system.run_pair(frame(10),frame(20));system.run_pair(frame(20),frame(30,'unsupported'));system.terminate();rec.finish()
    writes=[e for e in events(rec) if e['kind']=='motion_result_written']
    assert [e['submission_id'] for e in writes]==[1,1]
    assert [e['first_write_for_submission'] for e in writes]==[True,False]
    assert [e['write_ordinal_for_submission'] for e in writes]==[1,2]
    assert all(e['time1_ns']==20 for e in writes)
    pair=[e for e in events(rec) if e['kind']=='pair_end'][-1]
    assert pair['retained_count']==0 and pair['original_need_interp'] and pair['motion_status']=='not_submitted'


def test_skipped_pair_is_explicit_and_has_no_invented_health(tmp_path):
    system=System();rec=ArbiterEventRecorder(system,tmp_path/'events','thermal')
    system.run_pair(frame(10),frame(20,'skip'));system.terminate();rec.finish()
    pair=next(e for e in events(rec) if e['kind']=='pair_end')
    assert pair['original_need_interp'] and pair['selected_count'] is None and pair['retained_count'] is None
    assert not any(e['kind']=='motion_result_written' for e in events(rec))


def test_estimator_failure_is_preserved_and_partial_events_survive(tmp_path):
    system=System();rec=ArbiterEventRecorder(system,tmp_path/'events','rgb')
    with pytest.raises(RuntimeError,match='original estimator failure'):system.run_pair(frame(10),frame(20,'error'))
    system.terminate();rec.finish()
    pair=next(e for e in events(rec) if e['kind']=='pair_end')
    assert pair['execution_error']=='RuntimeError' and pair['new_frame_idx'] is None and pair['original_need_interp'] is None


def test_observer_failure_does_not_change_estimator_outputs(tmp_path):
    def bad_clock():raise OSError('observer clock unavailable')
    outputs=[]
    for enabled in [False,True]:
        system=System();rec=ArbiterEventRecorder(system,tmp_path/'events','rgb',clock=bad_clock) if enabled else None
        system.run_pair(frame(10),frame(20));system.terminate()
        outputs.append({k:v.tensor.copy() for k,v in system.graph.frames.data.items()})
        if rec:assert rec.finish()['status']=='OBSERVER_ERRORS'
    for key in outputs[0]:np.testing.assert_array_equal(outputs[0][key],outputs[1][key])


def test_copy_immutability_sensor_time_and_health(tmp_path):
    start=1761615290010015011;system=System(start);rec=ArbiterEventRecorder(system,tmp_path/'events','rgb')
    system.run_pair(frame(start),frame(start+1));system.Optimizer.write_map(system.graph)
    out=events(rec);write=next(e for e in out if e['kind']=='motion_result_written');pair=next(e for e in out if e['kind']=='pair_end')
    assert write['time0_ns']==start and write['time1_ns']==start+1
    assert pair['retained_count']==12 and pair['occupied_3x3_cells']==9 and pair['grid_trace_q10_q50_q90_px2']==[5.,5.,5.]
    system.Optimizer.pending.motion[:]=0;system.graph.frames.data['pose'].tensor[:]=0
    out[0]['kind']='mutated reader'
    assert next(e for e in events(rec) if e['kind']=='motion_result_written')['committed_pose_camera_map_xyzw'][0]==1
    assert events(rec)[0]['kind']=='pair_begin';rec.finish()


def test_invalid_pose_is_missing_delta_without_fabricated_identity(tmp_path):
    system=System();rec=ArbiterEventRecorder(system,tmp_path/'events','rgb')
    system.run_pair(frame(10),frame(20));system.Optimizer.pending.motion[:]=np.nan;system.terminate();rec.finish()
    write=next(e for e in events(rec) if e['kind']=='motion_result_written')
    assert write['derived_camera_delta_xyzw'] is None and write['derived_delta_unavailable_reason']
    assert write['committed_pose_camera_map_xyzw']==[None]*7


def test_finish_restore_even_if_save_fails(tmp_path):
    system=System();rec=ArbiterEventRecorder(system,tmp_path/'events','rgb');rec.output.rmdir()
    with pytest.raises(FileNotFoundError):rec.finish()
    assert 'run_pair' not in vars(system) and 'write_graph_data' not in vars(system.Optimizer)


def test_keyword_method_calls_preserve_original_signatures(tmp_path):
    system=System();rec=ArbiterEventRecorder(system,tmp_path/'events','rgb')
    system.run_pair(first=frame(10),second=frame(20))
    system.Optimizer.write_graph_data(result=system.Optimizer.pending,graph=system.graph)
    system.MapRefiner.elaborate_map(frames=system.graph.frames)
    assert rec.finish()['status']=='COMPLETE'


def test_optimizer_and_refinement_errors_are_explicit(tmp_path):
    system=System()
    def fail(data):raise RuntimeError('optimizer failed')
    system.Optimizer.start_optimize=fail
    rec=ArbiterEventRecorder(system,tmp_path/'events','rgb')
    with pytest.raises(RuntimeError,match='optimizer failed'):system.run_pair(frame(10),frame(20))
    rec.finish();out=events(rec)
    assert any(e['kind']=='optimization_error' for e in out)
    assert next(e for e in out if e['kind']=='pair_end')['motion_status']=='not_submitted'
    system=System()
    def refine(frames):raise RuntimeError('refinement failed')
    system.MapRefiner.elaborate_map=refine
    rec=ArbiterEventRecorder(system,tmp_path/'refine-events','rgb')
    with pytest.raises(RuntimeError,match='refinement failed'):system.terminate()
    rec.finish();assert next(e for e in events(rec) if e['kind']=='refinement_end')['execution_error']=='RuntimeError'


def test_write_error_is_not_swallowed(tmp_path):
    system=System()
    def write(result,graph):raise RuntimeError('write failed')
    system.Optimizer.write_graph_data=write
    rec=ArbiterEventRecorder(system,tmp_path/'events','rgb')
    with pytest.raises(RuntimeError,match='write failed'):system.run_pair(frame(10),frame(20))
    rec.finish();assert any(e['kind']=='motion_result_write_error' for e in events(rec))


def test_native_parameter_mutation_and_repeated_writes_are_only_observed(tmp_path):
    outputs=[]
    for enabled in [False,True]:
        system=System()
        def native_write(result,graph):
            if result is not None:
                result.motion[0,0]+=3  # local-to-world style in-place native behavior
                graph.frames.data['pose'][result.frame_idx]=result.motion[0]
        system.Optimizer.write_graph_data=native_write
        rec=ArbiterEventRecorder(system,tmp_path/'events','rgb') if enabled else None
        system.run_pair(frame(10),frame(20));system.Optimizer.write_map(system.graph);system.terminate()
        outputs.append(system.graph.frames.data['pose'].tensor.copy())
        if rec:
            rec.finish();writes=[e for e in events(rec) if e['kind']=='motion_result_written']
            assert writes[0]['optimizer_pose_before_write_native_parameterization'][0][0]==1
            assert writes[0]['committed_pose_camera_map_xyzw'][0]==4
            assert writes[1]['committed_pose_camera_map_xyzw'][0]==7
    np.testing.assert_array_equal(*outputs)


def test_relative_pose_rotation_and_no_mutation():
    p=np.r_[[3.,2.,1.],[0.,0.,np.sin(np.pi/4),np.cos(np.pi/4)]];t=p.copy();t[:3]+=[0,2,0]
    before=p.copy();out=relative_pose(p,t)
    np.testing.assert_allclose(out,np.r_[[2.,0.,0.],[0.,0.,0.,1.]],atol=1e-14)
    np.testing.assert_array_equal(p,before)
    for pose in [np.zeros(7),np.full(7,np.nan),np.ones(6)]:
        with pytest.raises(ValueError):relative_pose(pose,t)


def test_native_cpu_optimizer_parity(tmp_path,monkeypatch):
    import torch
    import pypose as pp
    from Module.Optimization.TwoFramePGO.Optimizer import TwoFrame_PGO
    from Module.Optimization.TwoFramePGO.Graphs import GraphInput
    from Module.Map import VisualMap, FrameNode, MatchObs, PointNode
    from Scripts.Setup.arbiter_event_recorder import array
    from Utility.Point import pixel2point_NED
    torch.set_num_threads(1)
    # Native optimizer asks for a CUDA timing stream even for CPU graph data;
    # profiling is disabled. This bypasses timing only, not the numerical solve.
    monkeypatch.setattr(torch.cuda,'current_stream',lambda:None)
    outputs=[];committed=[];native_outputs=[]
    for enabled in [False,True]:
        system=System();system.Optimizer=TwoFrame_PGO(NS(graph_type='icp',device='cpu',vectorize=False,parallel=False,autodiff=True))
        system.graph=VisualMap()
        def append_native(time_ns,unsupported):
            system.graph.frames.push(FrameNode.init({'pose':torch.tensor(IDENTITY[None],dtype=torch.float32),
                'time_ns':torch.tensor([time_ns]),'need_interp':torch.tensor([unsupported]),
                'K':torch.eye(3)[None],'baseline':torch.tensor([.2]),'T_BS':torch.tensor(IDENTITY[None],dtype=torch.float32)}))
        system.append=append_native;system.append(10,False)
        uv=torch.tensor([[5.+(i%6)*4,5.+(i//6)*4] for i in range(30)],dtype=torch.float64)
        depth=torch.linspace(4,8,len(uv),dtype=torch.float64);k=torch.tensor([[40.,0.,16.],[0.,40.,16.],[0.,0.,1.]],dtype=torch.float64)
        local=pixel2point_NED(uv,depth,k);truth=pp.SE3(torch.tensor([.2,.1,-.05,0.,0.,0.,1.],dtype=torch.float64))
        observations=MatchObs.init({'pixel2_uv':uv,'pixel2_d':depth[:,None],'obs2_covTc':torch.eye(3,dtype=torch.float64).repeat(len(uv),1,1)*.01})
        points=PointNode.init({'pos_Tw':truth.Act(local),'cov_Tw':torch.eye(3,dtype=torch.float64).repeat(len(uv),1,1)*.01})
        system.make_graph_data=lambda index:GraphInput(torch.tensor([index]),torch.tensor([index-1]),pp.identity_SE3(1,dtype=torch.float64),torch.tensor([.2]),observations,points,k,torch.zeros(len(uv),dtype=torch.long),'cpu')
        rec=ArbiterEventRecorder(system,tmp_path/'native-events','rgb') if enabled else None
        system.run_pair(frame(10),frame(20));native_outputs.append(system.Optimizer.optimize_res.motion.detach().numpy().copy())
        system.terminate();outputs.append({k:array(v.tensor) for k,v in system.graph.frames.data.items()})
        if rec:
            rec.finish();e=next(e for e in events(rec) if e['kind']=='motion_result_written');committed.append(e['committed_pose_camera_map_xyzw'])
            np.testing.assert_allclose(e['derived_camera_delta_xyzw'],truth.tensor().numpy(),atol=1e-5)
    for key in outputs[0]:assert outputs[0][key].dtype==outputs[1][key].dtype and outputs[0][key].tobytes()==outputs[1][key].tobytes()
    assert native_outputs[0].tobytes()==native_outputs[1].tobytes()
    np.testing.assert_allclose(committed[0],truth.tensor().numpy(),atol=1e-5)
