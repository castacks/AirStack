"""Instance adapter and teardown controls; no model inference/GPU required."""
import json
from pathlib import Path
from types import SimpleNamespace, ModuleType
import sys
import numpy as np
import pytest
import torch
from Scripts.Setup.bounded_solver_adapter import install, copied_pose


def fixture():
    calls = []; result = SimpleNamespace(motion=torch.tensor([[0., 0, 0, 0, 0, 0, 1.]], dtype=torch.float64))
    def original(context, data): calls.append((context, data)); return context, result
    cls = type('TwoFrame_PGO', (), {'_optimize': staticmethod(original)})
    optimizer = cls(); optimizer.config = SimpleNamespace(device='cpu', vectorize=True, parallel=False, graph_type='disp', autodiff=False); optimizer.is_parallel_mode = False
    graph = SimpleNamespace(frames=SimpleNamespace(data={'time_ns': torch.tensor([100, 200], dtype=torch.int64)}))
    data = SimpleNamespace(from_idx=torch.tensor([0]), frame_idx=torch.tensor([1])); return optimizer, graph, data, result, calls


def test_passthrough_single_original_call_and_inherited_restore():
    optimizer, graph, data, expected, calls = fixture(); context = {}; records = []
    original = optimizer._optimize; restore = install(optimizer, graph, 'passthrough', records)
    returned_context, result = optimizer._optimize(context, data)
    assert returned_context is context and result is expected and calls == [(context, data)]
    assert records[0]['time0_ns'] == 100 and records[0]['time1_ns'] == 200 and records[0]['pose_xyzw'] == [0, 0, 0, 0, 0, 0, 1]
    assert records[0]['elapsed_ns'] >= 0 and records[0]['numerically_unresolved']
    restore(); assert '_optimize' not in vars(optimizer) and optimizer._optimize is original


def test_owned_restore():
    optimizer, graph, _, _, _ = fixture(); original = lambda c, d: (c, None); optimizer._optimize = original
    restore = install(optimizer, graph, 'passthrough', []); restore(); assert vars(optimizer)['_optimize'] is original


@pytest.mark.parametrize('field,value', [('parallel', True), ('device', 'cuda'), ('graph_type', 'icp'), ('autodiff', True)])
def test_incompatible_configuration(field, value):
    optimizer, graph, _, _, _ = fixture(); setattr(optimizer.config, field, value)
    with pytest.raises(ValueError): install(optimizer, graph, 'repair', [])


@pytest.mark.parametrize('bad', [float('nan'), float('inf')])
def test_invalid_return_is_explicit(bad):
    optimizer, graph, data, result, _ = fixture(); result.motion[0, 0] = bad; records = []
    restore = install(optimizer, graph, 'passthrough', records)
    try:
        with pytest.raises(ValueError): optimizer._optimize({}, data)
    finally: restore()
    assert records[0]['status'] == 'INVALID_RESULT' and records[0]['pose_xyzw'] is None and records[0]['error_type'] == 'ValueError'


class Model(torch.nn.Module):
    def __init__(self, data):
        super().__init__(); self.pose = torch.nn.Parameter(torch.tensor([[0., 0, 0, 0, 0, 0, 1.]], dtype=torch.float64))
    def covariance_array(self): return torch.ones(1, 1, 1, dtype=torch.float64)
    def forward(self): return self.pose[:, :1]
    def jacobian(self): return torch.tensor([[1., 0, 0, 0, 0, 0, 0]], dtype=torch.float64)
    def write_back(self): return SimpleNamespace(motion=self.pose)


def test_repair_uses_best_state_without_original_call():
    optimizer, graph, data, _, calls = fixture(); records = []; restore = install(optimizer, graph, 'repair', records)
    try: _, result = optimizer._optimize({'pose_graph_class': Model}, data)
    finally: restore()
    assert not calls and copied_pose(result) == [0, 0, 0, 0, 0, 0, 1]
    assert records[0]['status'] == 'GRADIENT_TOLERANCE' and records[0]['solver']['returned_loss'] == 0


@pytest.mark.parametrize('status', ['INVALID_INPUT', 'LINEARIZATION_ERROR', 'LINEAR_SOLVER_ERROR', 'REJECTION_LIMIT'])
def test_fatal_status_is_not_written_as_motion(status, monkeypatch):
    import Scripts.Setup.bounded_solver_adapter as adapter
    monkeypatch.setattr(adapter, 'solve', lambda *a, **k: {'termination': status})
    optimizer, graph, data, _, _ = fixture(); records = []; restore = install(optimizer, graph, 'repair', records)
    try:
        with pytest.raises(RuntimeError, match=status): optimizer._optimize({'pose_graph_class': Model}, data)
    finally: restore()
    assert records[0]['status'] == status and records[0]['pose_xyzw'] is None


@pytest.mark.parametrize('fail', [False, True])
def test_runner_restores_factory_and_instance_on_exit(tmp_path, monkeypatch, fail):
    import Scripts.Setup as package
    from Scripts.Setup.run_td3_solver_arm import run
    optimizer, graph, data, _, _ = fixture(); system = SimpleNamespace(Optimizer=optimizer, graph=graph)
    class Factory:
        @classmethod
        def from_config(cls, config): return system
    descriptor = vars(Factory)['from_config']; original = optimizer._optimize
    fake = ModuleType('Scripts.Setup.run_td3_execution_v3'); fake.MACVO = Factory
    def execute(args, modality):
        s = Factory.from_config(None); s.Optimizer._optimize({}, data)
        if fail: raise RuntimeError('runner failure control')
    fake.run = execute; monkeypatch.setitem(sys.modules, fake.__name__, fake); monkeypatch.setattr(package, 'run_td3_execution_v3', fake, raising=False)
    monkeypatch.setattr(torch.cuda, 'synchronize', lambda: None); monkeypatch.setattr(torch.cuda, 'is_initialized', lambda: False)
    args = SimpleNamespace(arm='passthrough', output=tmp_path, modality='rgb')
    if fail:
        with pytest.raises(RuntimeError, match='runner failure'): run(args)
    else: run(args)
    assert vars(Factory)['from_config'] is descriptor and optimizer._optimize is original and '_optimize' not in vars(optimizer)
    report = json.loads((tmp_path/'rgb/solver-adapter.json').read_text())
    assert report['instance_method_restored'] and report['factory_restored'] and (report['error'] is not None) == fail
