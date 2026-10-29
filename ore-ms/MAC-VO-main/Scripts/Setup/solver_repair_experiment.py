"""Isolated toy and six-case fixed-factor repair arm; no pipeline/default changes."""
import argparse
import ast
import contextlib
from datetime import datetime
from zoneinfo import ZoneInfo
import hashlib
import io
import json
from pathlib import Path
from types import SimpleNamespace
import typing as T
import numpy as np
import torch
import pypose as pp
from pypose.optim.corrector import FastTriggs
from pypose.optim.kernel import Huber
from pypose.optim.scheduler import StopOnPlateau
from pypose.optim.solver import PINV
from pypose.optim.strategy import TrustRegion
from scipy.spatial.transform import Rotation
from Scripts.Setup.solver_contract_probe import core, Affine, fixtures, run_case, RecordSolver
from Scripts.Setup.weighted_huber_research import solve, objective

CASES = {'early': 'shared-arbiter-events', 'middle': 'arbiter-support-collapse', 'day-middle': 'history-day-replication'}


def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p): return json.loads(Path(p).read_text())
def write(p, v): Path(p).write_text(json.dumps(v, indent=2, allow_nan=False)+'\n')
def array(x): return x.detach().cpu().numpy().copy().tolist()


def native_graph_class(root):
    # Compile unchanged source definitions, avoiding Module/__init__ neural imports.
    graph_file = root/'MAC-VO-main/Module/Optimization/TwoFramePGO/Graphs.py'
    tree = ast.parse(graph_file.read_text())
    names = ('Reproj_TwoFramePGO', 'ReprojDisp_TwoFramePGO', 'Analytic_ReprojDisp_TwoFramePGO')
    nodes = [n for n in tree.body if isinstance(n, ast.ClassDef) and n.name in names]
    assert [n.name for n in nodes] == list(names)
    point_file = root/'MAC-VO-main/Utility/Point.py'
    projection, = [n for n in ast.parse(point_file.read_text()).body if isinstance(n, ast.FunctionDef) and n.name == 'point2pixel_NED']
    scope = {'__name__': 'isolated_original_graph', 'torch': torch, 'pp': pp, 'T': T,
             'FactorGraph': core.FactorGraph, 'AnalyticModule': core.AnalyticModule,
             'GraphInput': SimpleNamespace, 'GraphOutput': SimpleNamespace}
    exec(compile(ast.Module(body=[projection], type_ignores=[]), str(point_file), 'exec'), scope)
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(graph_file), 'exec'), scope)
    return scope['Analytic_ReprojDisp_TwoFramePGO']


def make_graph(root, case, cls):
    expert = root/case['expert_relative']; i = case['pair_ordinal']
    with np.load(expert/'tensor-map.npz', allow_pickle=False) as saved:
        mask = (saved['edge/match2frame1/mapping'] == i) & (saved['edge/match2frame2/mapping'] == i+1)
        ids = saved['edge/match2point/mapping'][mask]
        obs = {k.split('//')[1]: torch.from_numpy(saved[k][mask].copy()) for k in saved.files if k.startswith('match//')}
        points = {k.split('//')[1]: torch.from_numpy(saved[k][ids].copy()) for k in saved.files if k.startswith('points//')}
        data = SimpleNamespace(from_idx=torch.tensor([i]), frame_idx=torch.tensor([i+1]),
            init_motion=pp.SE3(torch.tensor([case['initial_pose_xyzw']], dtype=torch.float32)),
            edges_index=torch.zeros(int(mask.sum()), dtype=torch.long), observations=SimpleNamespace(data=obs), points=SimpleNamespace(data=points),
            images_intrinsic=torch.from_numpy(saved['frames//K'][i+1].copy()), baseline=torch.from_numpy(saved['frames//baseline'][i+1:i+2].copy()))
    return cls(data).to(device='cpu', dtype=torch.float64)


def costs(model):
    r = model().detach(); c = model.covariance_array().detach(); l = torch.linalg.cholesky(c)
    return {'raw_huber': float(Huber(.1)(r.square().sum(-1)).sum()), 'whitened_huber': float(objective(r, l, .1)[0])}


def original_solve(model):
    solver = RecordSolver(None); optimizer = core.LM_analytic(model, solver=solver, strategy=TrustRegion(radius=1e3),
        kernel=Huber(.1), corrector=FastTriggs(Huber(.1)), min=1e-6)
    scheduler = StopOnPlateau(optimizer, steps=10, patience=2, decreasing=1e-5, verbose=False)
    initial = costs(model); history = []; stdout = io.StringIO()
    with contextlib.redirect_stdout(stdout):
        while scheduler.continual():
            weight = torch.block_diag(*(torch.pinverse(model.covariance_array().double())))
            loss = optimizer.step(input=(), weight=weight); scheduler.step(loss)
            history.append({'returned_loss': float(loss), 'reject_count': optimizer.reject_count,
                            'pose_xyzw': array(model.pose2opt)[0], 'costs': costs(model), 'scheduler_continual': bool(scheduler.continual())})
    return {'initial_costs': initial, 'final_costs': costs(model), 'pose_xyzw': array(model.pose2opt)[0],
            'history': history, 'linear_solve_calls': len(solver.calls), 'stdout': stdout.getvalue(),
            'stop_scope': 'original scheduler stop; not a convergence or accuracy assertion'}


def verify(root, protocol):
    if protocol['delta_whitened'] != .1 or protocol['max_steps'] != 10 or protocol['max_attempts'] != 17 or protocol['production_defaults_changed']:
        raise ValueError('frozen repair contract changed')
    for path, digest in protocol['input_sha256'].items():
        if sha(root/path) != digest: raise ValueError('frozen case/source changed: '+path)


def prepare(root, output):
    output.mkdir(parents=True, exist_ok=False)
    pred_path = root/'results/factor-proxy-comparison/run-001/predictions.json'; predictions = read(pred_path)
    factor_path = root/'results/optimizer-factor-audit/run-005/audit.json'; factor = read(factor_path)
    proof = read(factor_path.parent/'validation.json')
    if proof['status'] != 'PASS' or proof['audit_sha256'] != sha(factor_path) or predictions['labels_loaded']: raise ValueError('unverified source factors/predictions')
    frozen = {root/p: digest for p, digest in factor['input_sha256'].items()}
    for path, digest in frozen.items():
        if sha(path) != digest: raise ValueError('original factor input changed')
    files = set(frozen) | {pred_path, factor_path, factor_path.parent/'validation.json'}
    lookup = {(r['case'], r['camera'], r['pair_ordinal']): r for r in factor['rows']}; cases = []
    for window, directory in CASES.items():
        rows = [r for r in predictions['rows'] if r['window'] == window]
        for camera in ('rgb', 'thermal'):
            first = min((r for r in rows if r['availability'][camera]), key=lambda r: r['pair_ordinal']); i = first['pair_ordinal']
            f = lookup[directory, camera, i]; expert = root/'results'/directory/'run-001/experts'/camera
            events = [json.loads(line) for line in (expert/'arbiter-events/events.jsonl').read_text().splitlines()]
            m = events[f['motion_event_sequence']]; config = read(expert/'effective-config.json')['Odometry']
            if config['motion']['type'] != 'StaticMotionModel' or config['optimizer']['type'] != 'TwoFrame_PGO' or config['optimizer']['args']['graph_type'] != 'disp' or config['args']['mapping']:
                raise ValueError('case requires static-motion global disp graph without mapping')
            if m['sequence'] != first['motion_event_sequences'][camera] or m['observation_monotonic_ns'] > first['deadline_monotonic_ns']: raise ValueError('wrong source motion')
            cases.append({'window': window, 'camera': camera, 'pair_ordinal': i, 'expert_relative': str(expert.relative_to(root)),
                'initial_pose_xyzw': m['parent_pose_camera_map_xyzw'], 'recorded_native_pose_xyzw': np.asarray(m['optimizer_pose_before_write_native_parameterization']).reshape(7).tolist(),
                'motion_event_sequence': m['sequence'], 'factor_count': f['retained_factor_count']})
    expected = [('early', 'rgb', 0), ('early', 'thermal', 0), ('middle', 'rgb', 96), ('middle', 'thermal', 0), ('day-middle', 'rgb', 0), ('day-middle', 'thermal', 0)]
    if [(c['window'], c['camera'], c['pair_ordinal']) for c in cases] != expected: raise ValueError('chronological cases changed')
    for rel in ('Scripts/Setup/solver_repair_experiment.py', 'Scripts/Setup/weighted_huber_research.py', 'Scripts/Setup/solver_contract_probe.py',
                'Module/Optimization/TwoFramePGO/Graphs.py', 'Module/Optimization/TwoFramePGO/Optimizer.py', 'Utility/Point.py', 'Module/MotionModel.py'):
        files.add(root/'MAC-VO-main'/rel)
    write(output/'protocol.json', {'created_at': datetime.now(ZoneInfo('America/New_York')).isoformat(), 'base_commit': 'fd397fe25',
        'delta_whitened': .1, 'delta_scope': 'dimensionless whitened-block norm; not equivalent to legacy raw-pixel threshold',
        'max_steps': 10, 'max_attempts': 17, 'production_defaults_changed': False, 'labels_loaded': False,
        'case_selection': 'first deadline-eligible pair per window/camera, no loss-based selection', 'cases': cases,
        'original_pose_tolerance': {'translation_m': .001, 'rotation_deg': .01},
        'input_sha256': {str(p.relative_to(root)): sha(p) for p in sorted(files)}, 'toy_fixtures': fixtures(),
        'scope': 'isolated affine controls and six fixed-landmark graph problems; no composed trajectory or accuracy claim'})


def run(root, output):
    if (output/'comparison.json').exists(): raise FileExistsError('comparison already exists')
    protocol = read(output/'protocol.json'); verify(root, protocol); toys = []
    for case in protocol['toy_fixtures']:
        m = Affine(case['matrix'], case['target'], case['initial'])
        w = case['step_weight'] if case['step_weight'] is not None else case['constructor_weight']
        n, d = np.array(case['target']).shape; w = np.eye(n*d) if w is None else np.array(w)
        c = np.stack([np.linalg.inv(w[i*d:(i+1)*d, i*d:(i+1)*d]) for i in range(n)])
        # Match one legacy toy outer step; saved-factor arms both have ten-step caps.
        control = RecordSolver(case['steps'], case['fail']) if case['steps'] is not None or case['fail'] else None
        repaired = solve(m, torch.tensor(c, dtype=torch.float64), max_steps=1, solver=control)
        toys.append({'name': case['name'], 'original': run_case(case), 'repair': repaired,
            'explicit_covariance_source': 'step_weight' if case['step_weight'] is not None else 'constructor_weight' if case['constructor_weight'] is not None else 'identity',
            'scope': 'repair always uses whitened Huber .1, including legacy no-kernel equation controls; not equal objective arms'})
    cls = native_graph_class(root); saved = []
    for case in protocol['cases']:
        a = make_graph(root, case, cls); original = original_solve(a)
        q0, q1 = np.array(original['pose_xyzw']), np.array(case['recorded_native_pose_xyzw'])
        translation = float(np.linalg.norm(q0[:3]-q1[:3])); rotation = float(np.degrees((Rotation.from_quat(q0[3:]).inv()*Rotation.from_quat(q1[3:])).magnitude()))
        b = make_graph(root, case, cls); repair_initial = costs(b)
        repaired = solve(b, b.covariance_array().detach().clone())
        saved.append({'case': case, 'original': original, 'repair': repaired, 'repair_initial_costs': repair_initial,
            'repair_final_costs': costs(b), 'repair_pose_xyzw': array(b.pose2opt)[0],
            'original_vs_recorded': {'translation_m': translation, 'rotation_deg': rotation,
                'within_declared_tolerance': translation <= .001 and rotation <= .01}})
    verify(root, protocol)
    write(output/'comparison.json', {'created_at': datetime.now(ZoneInfo('America/New_York')).isoformat(), 'protocol_sha256': sha(output/'protocol.json'),
        'toys': toys, 'saved_cases': saved, 'all_original_pose_controls_pass': all(r['original_vs_recorded']['within_declared_tolerance'] for r in saved),
        'labels_loaded': False, 'production_defaults_changed': False, 'torch': torch.__version__, 'pypose': pp.__version__,
        'scope': 'isolated fixed-factor objective/robustification repair, not replayed VO or calibrated/physical pose accuracy'})
    print(json.dumps({r['case']['window']+'/'+r['case']['camera']: {'control_pass': r['original_vs_recorded']['within_declared_tolerance'],
        'initial_weighted': r['repair']['initial_loss'], 'original_final_weighted': r['original']['final_costs']['whitened_huber'],
        'repair_final_weighted': r['repair']['returned_loss'], 'termination': r['repair']['termination']} for r in saved}, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('action', choices=('prepare', 'run')); p.add_argument('--root', type=Path, required=True); p.add_argument('--output', type=Path, required=True)
    a = p.parse_args(); {'prepare': prepare, 'run': run}[a.action](a.root, a.output)
