"""Frozen fixed-pair RGB second-pass control; reuse native solver and fixed-R wrapper."""
from pathlib import Path
import datetime
import hashlib
import json
import platform
import sys
import time
import numpy as np
import torch
from scipy.spatial.transform import Rotation
import component_constrained_native as component
from selected_motion_trajectory import rigid

native = component.native
feedback = component.feedback
ROOT = native.ROOT
OUT = ROOT/'results/optimization-budget-control'
PROTOCOL = ROOT/'docs/research/optimization-budget-control-protocol-2026-11-15.md'
METHODS = ['native_rgb', 'extra_full_pose', 'extra_translation']
PAIRS = range(599, 1199)


def load():
    pins = native.read(ROOT/'results/component-constrained-native/run-001/freeze.json')['input_pins']['night']
    pins = {p: digest for p, digest in pins.items() if '/thermal/' not in p}
    for p, digest in pins.items():
        assert native.sha(ROOT/p) == digest, p
    rec = native.REC
    source = native.read(rec/'inputs/night/source-manifest.json')
    assert source['sequence'] == '_2021-08-13-22-16-02'
    with np.load(rec/'captures/night/rgb/tensor-map.npz') as archive:
        saved = {k: archive[k] for k in archive.files
                 if not k.startswith('edge/point2match') and k != 'points//color'}
    events = {}
    for line in (rec/'captures/night/rgb/arbiter-events/events.jsonl').read_text().splitlines():
        event = json.loads(line)
        if event['kind'] == 'motion_result_written' and event['first_write_for_submission']:
            key = (event['from_idx'], event['frame_idx'])
            assert key not in events
            events[key] = event
    assert saved['frames//time_ns'].reshape(-1)[:1200].tolist() == [
        r['cameras']['rgb']['left']['recorded_time_ns'] for r in source['records'][:1200]]
    return source, saved, events, pins


def tensor_hash(tensor):
    array = tensor.detach().cpu().numpy()
    return hashlib.sha256(str((array.shape, array.dtype)).encode()+array.tobytes()).hexdigest()


def make_graph(saved, event, pair, source):
    parent = torch.tensor(event['parent_pose_camera_map_xyzw'], dtype=torch.float32)
    data = feedback.feedback_input(saved, event, pair, source['records'], parent)
    return native.Analytic_ReprojDisp_TwoFramePGO(data), parent


def timed_solve(graph):
    start_wall, start_cpu = time.perf_counter(), time.process_time()
    _, losses = native.optimize(graph)
    cpu, wall = time.process_time()-start_cpu, time.perf_counter()-start_wall
    committed = graph.pose2opt.detach().tensor()[0].float().clone()
    assert torch.isfinite(committed).all()
    return committed, {'losses': losses, 'steps': len(losses), 'solver_cpu_seconds': cpu,
                       'solver_wall_seconds': wall}


def result(parent, current, receipt):
    basis = np.eye(4); basis[:3, :3] = native.O
    transform = basis @ np.linalg.solve(native.pose(parent.numpy()), native.pose(current.numpy())) @ basis.T
    return {'status': 'AVAILABLE', 'transform': transform.tolist(), 'parent_xyzw': parent.tolist(),
            'committed_xyzw': current.tolist(), **receipt}


def pair_run(saved, event, pair, source, qualify=False):
    assert not event['original_need_interp'] and event['derived_camera_delta_xyzw'] is not None
    graph, parent = make_graph(saved, event, pair, source)
    first, receipt = timed_solve(graph)
    methods = {'native_rgb': result(parent, first, receipt)}
    basis = np.eye(4); basis[:3, :3] = native.O
    expected = basis @ native.pose(event['derived_camera_delta_xyzw']) @ basis.T
    error = np.linalg.solve(expected, np.asarray(methods['native_rgb']['transform']))
    reproduction = {'translation_m': float(np.linalg.norm(error[:3, 3])),
                    'rotation_rad': float(Rotation.from_matrix(error[:3, :3]).magnitude())}
    reproduction['passed'] = reproduction['translation_m'] < 1e-3 and reproduction['rotation_rad'] < 1e-4
    graphs = {}
    for name in METHODS[1:]:
        base, _ = make_graph(saved, event, pair, source)
        base = base.double()
        with torch.no_grad():
            base.pose2opt.copy_(native.pp.SE3(first.double()[None]))
        graphs[name] = (base if name == 'extra_full_pose' else
                        component.FixedRotationGraph(base, native.pp.SE3(first.double()[None])))
    full, fixed = [graphs[name] for name in METHODS[1:]]
    assert torch.equal(full.pose2opt.tensor(), fixed.pose2opt.tensor())
    assert torch.equal(full(), fixed())
    assert torch.equal(full.covariance_array(), fixed.covariance_array())
    assert torch.equal(full.build_jacobian()[:, :3], fixed.build_jacobian())
    parity = {'initial_xyzw': first.tolist(), 'residual_sha256': tensor_hash(full()),
              'covariance_sha256': tensor_hash(full.covariance_array()),
              'translation_jacobian_sha256': tensor_hash(full.build_jacobian()[:, :3]),
              'exact_initial_pose_residual_covariance_translation_jacobian': True}
    if qualify:
        analytic = fixed.build_jacobian().numpy().copy()
        initial = fixed.translation.detach().clone(); columns = []
        for axis in range(3):
            with torch.no_grad():
                fixed.translation.copy_(initial); fixed.translation[0, axis] += 1e-6
            plus = fixed().detach().numpy().reshape(-1).copy()
            with torch.no_grad():
                fixed.translation.copy_(initial); fixed.translation[0, axis] -= 1e-6
            minus = fixed().detach().numpy().reshape(-1).copy()
            columns.append((plus-minus)/2e-6)
        with torch.no_grad():
            fixed.translation.copy_(initial)
        numeric = np.stack(columns, axis=-1)
        parity['finite_difference_relative_error'] = float(np.max(np.abs(analytic-numeric)/(1+np.abs(numeric))))
        assert parity['finite_difference_relative_error'] < 1e-4
    order = METHODS[1:] if pair % 2 else METHODS[:0:-1]
    for name in order:
        try:
            current, timing = timed_solve(graphs[name])
            if name == 'extra_translation':
                assert torch.equal(current[3:], first[3:])
            methods[name] = result(parent, current, timing)
        except (RuntimeError, ValueError, np.linalg.LinAlgError) as error:
            methods[name] = {'status': 'FIT_FAILED', 'transform': None, 'reason': str(error)}
    return {'pair': pair, 'factor_count': len(graph.pos_Tw), 'methods': methods,
            'reproduction': reproduction, 'parity': parity, 'second_pass_order': order}


def qualify():
    torch.set_num_threads(1)
    target = OUT/'qualification-001'; target.mkdir(parents=True, exist_ok=False)
    source, saved, events, _ = load()
    rows = []
    for pair in [599, 600, 601]:
        rows.append(pair_run(saved, events[(pair, pair+1)], pair, source, qualify=True))
    passed = all(r['reproduction']['passed'] and all(v['transform'] is not None
                 for v in r['methods'].values()) for r in rows)
    native.write(target/'checks.json', {'rows': rows, 'passed': passed,
                 'runner_sha256': native.sha(__file__), 'protocol_sha256': native.sha(PROTOCOL)})
    print('QUALIFICATION', passed, flush=True)
    assert passed


def source_paths():
    paths = [Path(__file__), PROTOCOL, Path(component.__file__), Path(feedback.__file__),
             Path(native.__file__), ROOT/'tools/longer_fixed_thermal.py',
             ROOT/'tools/longer_native_control.py', ROOT/'tools/nominal_joint_motion.py',
             ROOT/'tools/selected_motion_trajectory.py']
    paths += list((ROOT/'MAC-VO-main/Module/Map').rglob('*.py'))
    paths += list((ROOT/'MAC-VO-main/Module/Optimization').rglob('*.py'))
    paths += list((ROOT/'MAC-VO-main/Utility').rglob('*.py'))
    return sorted(set(paths))


def capture():
    torch.set_num_threads(1)
    qualification = OUT/'qualification-001/checks.json'
    q = native.read(qualification)
    assert q['passed'] and q['runner_sha256'] == native.sha(__file__)
    assert q['protocol_sha256'] == native.sha(PROTOCOL)
    target = OUT/'run-001'; target.mkdir(parents=True, exist_ok=False)
    source, saved, events, pins = load()
    refs = {str(p.relative_to(ROOT)): native.sha(p) for p in sorted(
        (ROOT/'results/fixed-rotation-continuation/recovery/references/night').glob('*.txt'))
        if 599 <= int(p.stem) <= 1199}
    assert len(refs) == 601
    import pypose, scipy
    freeze = {'at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'source_pins': {str(p.relative_to(ROOT)): native.sha(p) for p in source_paths()},
              'input_pins': pins, 'reference_pins': refs,
              'qualification_sha256': native.sha(qualification), 'pairs': [599, 1198],
              'methods': METHODS, 'reference_payloads_opened_for_scoring': False,
              'runtime': {'python': platform.python_version(), 'torch': torch.__version__,
                          'pypose': pypose.__version__, 'numpy': np.__version__,
                          'scipy': scipy.__version__, 'torch_threads': torch.get_num_threads()},
              'scope': 'Historical-parent fixed-pair RGB replay; no new feedback histories'}
    native.write(target/'freeze.json', freeze)
    rows = []
    with (target/'capture.jsonl').open('x') as stream:
        for pair in PAIRS:
            row = pair_run(saved, events[(pair, pair+1)], pair, source)
            stream.write(json.dumps(row, allow_nan=False)+'\n'); stream.flush(); rows.append(row)
            if (pair-599) % 50 == 0:
                print('CAPTURE', pair, flush=True)
    native.write(target/'predictions.json', {'rows': rows})
    native.write(target/'prediction-lock.json', {'prediction_sha256': native.sha(target/'predictions.json'),
                 'freeze_sha256': native.sha(target/'freeze.json'), 'reference_scores_read': False})
    print('LOCKED', len(rows), 'pairs', flush=True)


def metrics(values):
    values = np.asarray(values)
    return {'count': len(values), 'mean': float(np.mean(values)),
            'rmse': float(np.sqrt(np.mean(values**2))), 'p95': float(np.percentile(values, 95))}


def score():
    target = OUT/'run-001'; lock = native.read(target/'prediction-lock.json')
    assert native.sha(target/'predictions.json') == lock['prediction_sha256']
    assert native.sha(target/'freeze.json') == lock['freeze_sha256']
    freeze = native.read(target/'freeze.json')
    for path, digest in freeze['reference_pins'].items():
        assert native.sha(ROOT/path) == digest
    refs = {i: rigid(np.loadtxt(ROOT/f'results/fixed-rotation-continuation/recovery/references/night/{i:06}.txt').reshape(4, 4))[0]
            for i in range(599, 1200)}
    rows = native.read(target/'predictions.json')['rows']
    common = [r for r in rows if all(r['methods'][m]['transform'] is not None for m in METHODS)]
    summary = {}
    for method in METHODS:
        te, re = [], []
        for row in common:
            i = row['pair']; expected = np.linalg.solve(refs[i], refs[i+1])
            got = rigid(row['methods'][method]['transform'])[0]
            te.append(np.linalg.norm(got[:3, 3]-expected[:3, 3]))
            re.append(np.rad2deg(Rotation.from_matrix(expected[:3, :3].T @ got[:3, :3]).magnitude()))
        available = [r['methods'][method] for r in rows if r['methods'][method]['transform'] is not None]
        path = None
        if len(available) == 600:
            predicted = np.eye(4); pe, ae = [0.], [0.]
            for row in rows:
                predicted = predicted @ rigid(row['methods'][method]['transform'])[0]
                truth = np.linalg.solve(refs[599], refs[row['pair']+1])
                pe.append(np.linalg.norm(predicted[:3, 3]-truth[:3, 3]))
                ae.append(np.rad2deg(Rotation.from_matrix(truth[:3, :3].T @ predicted[:3, :3]).magnitude()))
            path = {'position_m': metrics(pe), 'orientation_deg': metrics(ae)}
        cpu = [v['solver_cpu_seconds'] for v in available]
        wall = [v['solver_wall_seconds'] for v in available]
        summary[method] = {'available': len(available), 'interval_translation_m': metrics(te),
            'interval_rotation_deg': metrics(re), 'fixed_pair_path': path,
            'solver_cpu_seconds': {**metrics(cpu), 'total': sum(cpu)},
            'solver_wall_seconds': {**metrics(wall), 'total': sum(wall)},
            'scheduler_steps': {**metrics([v['steps'] for v in available]),
                                'total': sum(v['steps'] for v in available)}}
    native.write(target/'scores.json', {'summary': summary, 'common_pairs': len(common),
        'all_native_reproduction_passed': all(r['reproduction']['passed'] for r in rows),
        'prediction_lock_sha256': native.sha(target/'prediction-lock.json'),
        'scope': freeze['scope']})
    print(json.dumps(native.read(target/'scores.json'), indent=2), flush=True)


if __name__ == '__main__':
    {'qualify': qualify, 'capture': capture, 'score': score}[sys.argv[1]]()
