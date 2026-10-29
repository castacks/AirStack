"""Bounded native LM_analytic contract probes; original optimizer stays unchanged.

Tiny affine fixtures isolate equations and acceptance behavior. Scripted solver
directions are controls, not evidence that native VO generated those steps.
"""
import argparse
import contextlib
from datetime import datetime
from zoneinfo import ZoneInfo
import hashlib
import importlib.util
import inspect
import io
import json
from pathlib import Path
import platform
import numpy as np
import torch
import pypose
from pypose.optim.corrector import FastTriggs
from pypose.optim.kernel import Huber
from pypose.optim.optimizer import RobustModel, _Optimizer
from pypose.optim.scheduler import StopOnPlateau
from pypose.optim.solver import PINV
from pypose.optim.strategy import TrustRegion

CORE = Path(__file__).resolve().parents[2]/'Module/Optimization/PyposeOptimizers.py'
spec = importlib.util.spec_from_file_location('original_optimizer_contract_core', CORE)
core = importlib.util.module_from_spec(spec); spec.loader.exec_module(core)


def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def arr(x): return x.detach().cpu().numpy().copy().tolist()


class Affine(core.AnalyticModule):
    def __init__(self, matrix, target, initial):
        super().__init__(); self.theta = torch.nn.Parameter(torch.tensor(initial, dtype=torch.float64))
        self.register_buffer('matrix', torch.tensor(matrix, dtype=torch.float64)); self.register_buffer('target', torch.tensor(target, dtype=torch.float64))
    def forward(self): return self.matrix @ self.theta-self.target
    def build_jacobian(self): return self.matrix.reshape(-1, len(self.theta))


class RecordCorrector:
    def __init__(self, delta): self.native = FastTriggs(Huber(delta)) if delta is not None else None; self.calls = []
    def __call__(self, R, J):
        before = {'raw_R': arr(R), 'raw_J': arr(J)}
        r, j = self.native(R, J) if self.native is not None else (R, J)
        self.calls.append(before | {'corrected_R': arr(r), 'corrected_J': arr(j)})
        return r, j


class RecordSolver:
    def __init__(self, steps, fail=False): self.steps = steps; self.fail = fail; self.calls = []; self.native = PINV()
    def __call__(self, A, b):
        record = {'A': arr(A), 'b': arr(b)}; index = len(self.calls); self.calls.append(record)
        if self.fail: record['injected_error'] = 'RuntimeError'; raise RuntimeError('deliberate contract probe solver failure')
        if self.steps is None: d = self.native(A=A, b=b)
        else: d = torch.tensor(self.steps[min(index, len(self.steps)-1)], dtype=b.dtype).reshape_as(b)
        record['D'] = arr(d); return d


class RecordStrategy:
    def __init__(self, native):
        self.native = TrustRegion(radius=1e3) if native else None
        self.defaults = self.native.defaults.copy() if native else {'damping': .001}; self.calls = []
    def update(self, pg, last, loss, J, D, R):
        record = {'last': float(last), 'loss': float(loss), 'J': arr(J), 'D': arr(D), 'R': arr(R), 'before': {k: float(pg[k]) for k in self.defaults}}
        if self.native is not None: self.native.update(pg, last=last, loss=loss, J=J, D=D, R=R)
        record['after'] = {k: float(pg[k]) for k in self.defaults}; self.calls.append(record)


def fixtures():
    simple = {'matrix': [[[1.]], [[1.]]], 'target': [[-1.], [1.]], 'initial': [0.], 'delta': None,
              'constructor_weight': None, 'step_weight': None, 'steps': [[0.]], 'native_strategy': False, 'reject': 16, 'fail': False}
    values = []
    def add(name, **updates): values.append(simple | {'name': name} | updates)
    add('unweighted_matrix')
    add('explicit_weight_matrix', step_weight=[[1., 0], [0, 9.]])
    add('constructor_only_weight', constructor_weight=[[1., 0], [0, 9.]])
    add('explicit_overrides_constructor', constructor_weight=[[9., 0], [0, 1.]], step_weight=[[1., 0], [0, 9.]])
    add('swapped_explicit_weight', step_weight=[[9., 0], [0, 1.]])
    add('huber_inliers', target=[[-.02], [.03]], delta=.1, step_weight=[[1., 0], [0, 9.]])
    add('huber_outliers', delta=.1, step_weight=[[1., 0], [0, 9.]])
    covariance = np.array([[[2., .3, 0], [.3, 4., 0], [0, 0, .5]], [[1., .1, 0], [.1, 3., 0], [0, 0, 2.]]])
    weight = np.zeros((6, 6)); weight[:3, :3], weight[3:, 3:] = np.linalg.inv(covariance)
    add('correlated_factor_huber', matrix=[[[1., 0], [0, 1.], [1., 1.]], [[2., 0], [0, -1.], [1., -1.]]],
        target=[[1., .1, -.2], [-.03, .02, .04]], initial=[0., 0.], delta=.1, step_weight=weight.tolist(), steps=[[0., 0.]])
    common = {'target': [[0.], [.2]], 'initial': [.05], 'delta': .1, 'step_weight': [[9., 0], [0, 1.]]}
    add('accept_raw_improves_mahalanobis_worsens', **common, steps=[[.025]], native_strategy=True)
    add('reject_raw_worsens_mahalanobis_improves', **common, steps=[[-.025], [0.]], native_strategy=True)
    add('native_pinv_huber_conflict', **common, steps=None, native_strategy=True)
    add('native_pinv_identity_weight_control', **(common | {'step_weight': [[1., 0], [0, 1.]]}), steps=None, native_strategy=True)
    add('forced_rejection_cap', target=[[0.], [0.]], steps=[[1.]], reject=2, native_strategy=True)
    add('forced_solver_failure', fail=True, native_strategy=True)
    return values


def objectives(model, weight, delta):
    r = model().detach(); w = torch.eye(r.numel(), dtype=r.dtype) if weight is None else torch.tensor(weight, dtype=r.dtype)
    blocks = w.reshape(r.shape[0], r.shape[1], r.shape[0], r.shape[1])
    q = torch.stack([r[i] @ blocks[i, :, i, :] @ r[i] for i in range(len(r))])
    raw = r.square().sum(-1); kernel = Huber(delta) if delta is not None else lambda x: x
    return {'raw_kernel_sum': float(kernel(raw).sum()), 'mahalanobis_kernel_sum': float(kernel(q).sum()),
            'weighted_quadratic': float(q.sum()), 'raw_quadratic': float(raw.sum())}


def run_case(case):
    m = Affine(case['matrix'], case['target'], case['initial']); solver = RecordSolver(case['steps'], case['fail'])
    corrector = RecordCorrector(case['delta']); strategy = RecordStrategy(case['native_strategy'])
    constructor = None if case['constructor_weight'] is None else torch.tensor(case['constructor_weight'], dtype=torch.float64)
    optimizer = core.LM_analytic(m, solver=solver, strategy=strategy, corrector=corrector, kernel=Huber(case['delta']) if case['delta'] is not None else None, weight=constructor, reject=case['reject'])
    effective = case['step_weight'] # Native branch tests explicit argument, ignoring constructor-only weight.
    before = objectives(m, effective, case['delta']); output = io.StringIO()
    explicit = None if case['step_weight'] is None else torch.tensor(case['step_weight'], dtype=torch.float64)
    with contextlib.redirect_stdout(output): loss = optimizer.step(input=(), weight=explicit)
    after = objectives(m, effective, case['delta'])
    scheduler = StopOnPlateau(optimizer, steps=10, patience=2, decreasing=1e-5, verbose=False); scheduler.step(loss)
    return {'case': case, 'before_objectives': before, 'after_objectives': after, 'final_parameter': arr(m.theta), 'returned_loss': float(loss),
            'last': float(optimizer.last), 'reject_count': optimizer.reject_count, 'solver_calls': solver.calls, 'corrector_calls': corrector.calls,
            'strategy_calls': strategy.calls, 'stdout': output.getvalue(), 'scheduler_continual_after_one_step': bool(scheduler.continual()),
            'scheduler_patience_count': scheduler.patience_count}


def run(output):
    output.mkdir(parents=True, exist_ok=False)
    frozen = {str(CORE): sha(CORE), str(Path(__file__).resolve()): sha(Path(__file__))}
    specification = {'created_at': datetime.now(ZoneInfo('America/New_York')).isoformat(), 'base_commit': '0817cae27',
        'scope': 'toy native solver contract only, no saved trajectory replay or default change', 'fixtures': fixtures(), 'source_sha256': frozen}
    (output/'protocol.json').write_text(json.dumps(specification, indent=2)+'\n')
    objects = {'RobustModel.loss': RobustModel.loss, 'FastTriggs.forward': FastTriggs.forward, 'TrustRegion.update': TrustRegion.update,
               'Huber.forward': Huber.forward, 'StopOnPlateau.step': StopOnPlateau.step, '_Optimizer.update_parameter': _Optimizer.update_parameter, 'LM_analytic.step': core.LM_analytic.step}
    dependency_sources = {name: inspect.getsource(obj) for name, obj in objects.items()}
    (output/'native-source.json').write_text(json.dumps(dependency_sources, indent=2)+'\n')
    rows = [run_case(case) for case in specification['fixtures']]
    for path, digest in frozen.items():
        if sha(Path(path)) != digest: raise ValueError('original source changed')
    report = {'created_at': datetime.now(ZoneInfo('America/New_York')).isoformat(), 'protocol_sha256': sha(output/'protocol.json'),
        'native_source_sha256': sha(output/'native-source.json'), 'torch': torch.__version__, 'pypose': pypose.__version__, 'numpy': np.__version__, 'python': platform.python_version(),
        'dependency_method_sha256': {name: hashlib.sha256(src.encode()).hexdigest() for name, src in dependency_sources.items()},
        'rows': rows, 'native_vo_solver_replayed': False, 'estimator_defaults_changed': False}
    (output/'probes.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({r['case']['name']: {'parameter': r['final_parameter'], 'loss': r['returned_loss'], 'solver_calls': len(r['solver_calls']), 'reject_count': r['reject_count']} for r in rows}, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('--output', type=Path, required=True)
    run(p.parse_args().output)
