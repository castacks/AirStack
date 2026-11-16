"""Independent saved-output/data audit; no experiment runner or optimizer import."""
from pathlib import Path
import copy
import hashlib
import json
import math
import numpy as np
import torch
import pypose as pp
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results/optimization-budget-control/run-001'
METHODS = ['native_rgb', 'extra_full_pose', 'extra_translation']
BASIS = np.eye(4); BASIS[:3, :3] = [[0, 1, 0], [0, 0, 1], [1, 0, 0]]


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def pose(values):
    t = np.eye(4); t[:3, 3] = values[:3]
    assert abs(np.linalg.norm(values[3:])-1) < 1e-5
    t[:3, :3] = Rotation.from_quat(values[3:]).as_matrix()
    return t


def proper(value):
    value = np.asarray(value, dtype=float)
    assert value.shape == (4, 4) and np.isfinite(value).all()
    assert np.array_equal(value[3], [0, 0, 0, 1])
    r = value[:3, :3]
    assert np.linalg.norm(r.T@r-np.eye(3)) <= 1e-5 and abs(np.linalg.det(r)-1) <= 1e-5
    u, _, vt = np.linalg.svd(r); value = value.copy()
    value[:3, :3] = u @ np.diag([1., 1., np.linalg.det(u@vt)]) @ vt
    return value


def tensor_hash(t):
    a = t.detach().numpy()
    return hashlib.sha256(str((a.shape, a.dtype)).encode()+a.tobytes()).hexdigest()


def measure(values):
    values = sorted(float(x) for x in values)
    n = len(values); q = (n-1)*.95; lo = math.floor(q); hi = math.ceil(q)
    return {'count': n, 'mean': math.fsum(values)/n,
            'rmse': math.sqrt(math.fsum(x*x for x in values)/n),
            'p95': values[lo]+(values[hi]-values[lo])*(q-lo)}


def audit(rows, scores, events, saved, refs):
    count = 0
    def check(value, label):
        nonlocal count
        if not value:
            raise AssertionError(label)
        count += 1
    def compare(actual, expected, label):
        for key, value in expected.items():
            check(math.isclose(actual[key], value, rel_tol=1e-10, abs_tol=1e-10), f'{label}/{key}')
    check([r['pair'] for r in rows] == list(range(599, 1199)), 'population')
    common = [r for r in rows if all(r['methods'][m]['transform'] is not None for m in METHODS)]
    check(scores['common_pairs'] == len(common), 'common support')
    for row in rows:
        pair = row['pair']; e = events[(pair, pair+1)]
        check(not e['original_need_interp'], 'native availability')
        mask = (saved['edge/match2frame1/mapping'] == pair) & (saved['edge/match2frame2/mapping'] == pair+1)
        check(row['factor_count'] == np.count_nonzero(mask), 'factor count')
        check(row['second_pass_order'] == (METHODS[1:] if pair % 2 else METHODS[:0:-1]), 'order')
        first = row['methods']['native_rgb']['committed_xyzw']
        check(row['parity']['initial_xyzw'] == first, 'second pass initialization')
        check(row['parity']['exact_initial_pose_residual_covariance_translation_jacobian'], 'parity recorded')
        parent = torch.tensor(e['parent_pose_camera_map_xyzw'], dtype=torch.float32)
        # Independently rebuild native float32 point registration from accepted measurements.
        pixels = torch.from_numpy(saved['match//pixel1_uv'][mask].copy())
        depth = torch.from_numpy(saved['match//pixel1_d'][mask].copy()).squeeze(-1)
        k0 = torch.from_numpy(saved['frames//K'][pair].copy())
        local = pp.pixel2point(pixels, depth, k0).roll(shifts=1, dims=-1)
        world = pp.SE3_type.Act(pp.SE3(parent), local)[..., :3].double()
        current = pp.SE3(torch.tensor(first, dtype=torch.float64)[None])
        camera = current.Inv() * world
        k = torch.from_numpy(saved['frames//K'][pair+1].copy()).double()
        uv = pp.point2pixel(camera.roll(shifts=-1, dims=-1), k)
        target_uv = torch.from_numpy(saved['match//pixel2_uv'][mask].copy()).double()
        disp = torch.from_numpy(saved['match//pixel2_disp'][mask].copy()).double()
        baseline = torch.from_numpy(saved['frames//baseline'][pair+1:pair+2].copy()).double()
        residual = torch.cat((uv-target_uv, camera[:, 0:1].reciprocal()*(k[0, 0]*baseline)-disp), dim=-1)
        check(tensor_hash(residual) == row['parity']['residual_sha256'], 'independent initial residual')
        cov = torch.zeros((row['factor_count'], 3, 3), dtype=torch.float32)
        c = torch.from_numpy(saved['match//pixel2_uv_cov'][mask].copy())
        cov[:, 0, 0] = c[:, 0]; cov[:, 1, 1] = c[:, 1]
        cov[:, 0, 1] = c[:, 2]; cov[:, 1, 0] = c[:, 2]
        cov[:, 2, 2] = torch.from_numpy(saved['match//pixel2_disp_cov'][mask].copy()).squeeze(-1)
        check(tensor_hash(cov.double()) == row['parity']['covariance_sha256'], 'independent initial precision input')
        for method in METHODS:
            got = row['methods'][method]
            if got['transform'] is None:
                check(got['status'] == 'FIT_FAILED', 'failure retained'); continue
            check(got['status'] == 'AVAILABLE', 'status')
            check(got['parent_xyzw'] == parent.tolist(), 'historical parent')
            expected = BASIS @ np.linalg.inv(pose(got['parent_xyzw'])) @ pose(got['committed_xyzw']) @ BASIS.T
            check(np.allclose(expected, got['transform'], rtol=0, atol=1e-10), 'pose-delta binding')
            check(1 <= got['steps'] <= 10 and got['steps'] == len(got['losses']), 'solver step accounting')
            check(np.isfinite(got['losses']).all(), 'loss finite')
            check(got['solver_cpu_seconds'] > 0 and got['solver_wall_seconds'] > 0, 'positive measured cost')
        if row['methods']['extra_translation']['transform'] is not None:
            check(row['methods']['extra_translation']['committed_xyzw'][3:] == first[3:], 'rotation exact')
        original = BASIS @ pose(e['derived_camera_delta_xyzw']) @ BASIS.T
        delta = np.linalg.inv(original) @ np.asarray(row['methods']['native_rgb']['transform'])
        translation = np.linalg.norm(delta[:3, 3]); angle = Rotation.from_matrix(delta[:3, :3]).magnitude()
        compare(row['reproduction'], {'translation_m': translation, 'rotation_rad': angle}, 'native reproduction')
        check(row['reproduction']['passed'] == bool(translation < 1e-3 and angle < 1e-4), 'native gate')
    check(scores['all_native_reproduction_passed'] == all(r['reproduction']['passed'] for r in rows), 'aggregate gate')
    for method in METHODS:
        actual = scores['summary'][method]
        available = [r['methods'][method] for r in rows if r['methods'][method]['transform'] is not None]
        check(actual['available'] == len(available), 'availability count')
        te, re = [], []
        for row in common:
            i = row['pair']; expected = np.linalg.inv(refs[i]) @ refs[i+1]
            got = proper(row['methods'][method]['transform'])
            te.append(np.linalg.norm(expected[:3, 3]-got[:3, 3]))
            re.append(math.degrees(Rotation.from_matrix(expected[:3, :3].T @ got[:3, :3]).magnitude()))
        compare(actual['interval_translation_m'], measure(te), 'translation metrics')
        compare(actual['interval_rotation_deg'], measure(re), 'rotation metrics')
        for field in ['solver_cpu_seconds', 'solver_wall_seconds', 'scheduler_steps']:
            key = 'steps' if field == 'scheduler_steps' else field
            vals = [r[key] for r in available]
            compare(actual[field], {**measure(vals), 'total': math.fsum(vals)}, 'cost/steps')
        if len(available) == 600:
            r_acc = np.eye(3); t_acc = np.zeros(3); pos, angles = [0.], [0.]
            for row in rows:
                motion = proper(row['methods'][method]['transform'])
                t_acc = t_acc+r_acc@motion[:3, 3]; r_acc = r_acc@motion[:3, :3]
                truth = np.linalg.inv(refs[599]) @ refs[row['pair']+1]
                pos.append(np.linalg.norm(t_acc-truth[:3, 3]))
                angles.append(math.degrees(Rotation.from_matrix(truth[:3, :3].T@r_acc).magnitude()))
            compare(actual['fixed_pair_path']['position_m'], measure(pos), 'path position')
            compare(actual['fixed_pair_path']['orientation_deg'], measure(angles), 'path rotation')
        else:
            check(actual['fixed_pair_path'] is None, 'no gap fill')
    return count


def main():
    torch.set_num_threads(1)
    freeze = read(OUT/'freeze.json'); lock = read(OUT/'prediction-lock.json')
    assert sha(OUT/'freeze.json') == lock['freeze_sha256']
    assert sha(OUT/'predictions.json') == lock['prediction_sha256']
    for kind in ['source_pins', 'input_pins', 'reference_pins']:
        for name, digest in freeze[kind].items():
            assert sha(ROOT/name) == digest, (kind, name)
    qualification = ROOT/'results/optimization-budget-control/qualification-001/checks.json'
    assert sha(qualification) == freeze['qualification_sha256']
    q = read(qualification); assert q['passed']
    assert q['runner_sha256'] == freeze['source_pins']['tools/optimization_budget_control.py']
    assert q['protocol_sha256'] == freeze['source_pins']['docs/research/optimization-budget-control-protocol-2026-11-15.md']
    for row in q['rows']:
        assert row['parity']['finite_difference_relative_error'] < 1e-4
    predictions = read(OUT/'predictions.json'); rows = predictions['rows']; scores = read(OUT/'scores.json')
    assert scores['prediction_lock_sha256'] == sha(OUT/'prediction-lock.json')
    assert [json.loads(line) for line in (OUT/'capture.jsonl').read_text().splitlines()] == rows
    folder = ROOT/'results/longer-fixed-thermal/recovery'
    events = {}
    for line in (folder/'captures/night/rgb/arbiter-events/events.jsonl').read_text().splitlines():
        e = json.loads(line)
        if e['kind'] == 'motion_result_written' and e['first_write_for_submission']:
            key = (e['from_idx'], e['frame_idx']); assert key not in events; events[key] = e
    with np.load(folder/'captures/night/rgb/tensor-map.npz') as archive:
        saved = {k: archive[k] for k in archive.files if k.startswith(('match//', 'frames//', 'edge/match2frame'))}
    refs = {i: proper(np.loadtxt(ROOT/f'results/fixed-rotation-continuation/recovery/references/night/{i:06}.txt').reshape(4, 4)) for i in range(599, 1200)}
    checks = audit(rows, scores, events, saved, refs)
    mutations = []
    for name in ['population', 'initialization', 'rotation', 'parent', 'cost', 'reference_score', 'availability']:
        bad = copy.deepcopy(rows); bad_scores = copy.deepcopy(scores)
        if name == 'population': bad.pop()
        elif name == 'initialization': bad[0]['parity']['initial_xyzw'][0] += 1
        elif name == 'rotation': bad[0]['methods']['extra_translation']['committed_xyzw'][3] += .01
        elif name == 'parent': bad[0]['methods']['extra_full_pose']['parent_xyzw'][0] += 1
        elif name == 'cost': bad[0]['methods']['extra_full_pose']['steps'] = 11
        elif name == 'reference_score': bad_scores['summary']['native_rgb']['interval_translation_m']['rmse'] += 1
        elif name == 'availability': bad_scores['summary']['native_rgb']['available'] -= 1
        try:
            audit(bad, bad_scores, events, saved, refs)
        except AssertionError:
            mutations.append(name)
        else:
            raise AssertionError('Mutation escaped: '+name)
    receipt = {'status': 'PASS', 'independent_checks': checks, 'mutations_rejected': mutations,
               'auditor_sha256': sha(__file__), 'scores_sha256': sha(OUT/'scores.json'),
               'prediction_lock_sha256': sha(OUT/'prediction-lock.json')}
    path = OUT/'audit.json'
    with path.open('x') as stream:
        json.dump(receipt, stream, indent=2); stream.write('\n')
    print(json.dumps(receipt, indent=2))


if __name__ == '__main__':
    main()
