"""Saved-input replay of native disparity PGO with calibrated thermal factors.

No frontend or production-default change. Native graph and optimizer bodies are
imported unchanged; namespace packages avoid initializing unrelated frontends.
"""
from pathlib import Path
import sys
import types
import json
import hashlib
import datetime
import numpy as np
import torch
import pypose as pp
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
NATIVE = ROOT / 'MAC-VO-main'
sys.path.insert(0, str(NATIVE))
for name, folder in [('Module', 'Module'), ('Module.Optimization', 'Module/Optimization'),
                     ('Module.Optimization.TwoFramePGO', 'Module/Optimization/TwoFramePGO')]:
    if name not in sys.modules:
        module = types.ModuleType(name)
        module.__path__ = [str(NATIVE / folder)]
        sys.modules[name] = module

from Module.Map import MatchObs, PointNode
from Module.Optimization.TwoFramePGO.Graphs import GraphInput, Analytic_ReprojDisp_TwoFramePGO
from Module.Optimization.PyposeOptimizers import LM_analytic
from pypose.optim.corrector import FastTriggs
from pypose.optim.kernel import Huber
from pypose.optim.scheduler import StopOnPlateau
from pypose.optim.solver import PINV
from pypose.optim.strategy import TrustRegion
from longer_fixed_thermal import REC, read, write, sha, path_stats
from longer_native_control import pose
from nominal_joint_motion import geometry, bind_camera

OUT = ROOT / 'results/native-thermal-constraints'
O = np.array([[0., 1, 0], [0, 0, 1], [1, 0, 0]])


def load(condition):
    source = read(REC / 'inputs' / condition / 'source-manifest.json')
    extrinsics = geometry(read(REC / 'inputs' / condition / 'calibration.json'))
    saved, events = {}, {}
    for camera in ['rgb', 'thermal']:
        folder = REC / 'captures' / condition / camera
        with np.load(folder / 'tensor-map.npz') as archive:
            saved[camera] = {k: archive[k] for k in archive.files
                            if not k.startswith('edge/point2match') and k != 'points//color'}
        events[camera] = {}
        for line in (folder / 'arbiter-events/events.jsonl').read_text().splitlines():
            event = json.loads(line)
            if event['kind'] == 'motion_result_written' and event['first_write_for_submission']:
                key = (event['from_idx'], event['frame_idx'])
                assert key not in events[camera]
                events[camera][key] = event
    return source, extrinsics, saved, events


def rgb_input(saved, event, pair, records):
    assert event and not event['original_need_interp']
    assert [event['time0_ns'], event['time1_ns']] == [
        records[j]['cameras']['rgb']['left']['recorded_time_ns'] for j in [pair, pair + 1]]
    mask = ((saved['edge/match2frame1/mapping'] == pair) &
            (saved['edge/match2frame2/mapping'] == pair + 1))
    indices = np.flatnonzero(mask)
    point_indices = saved['edge/match2point/mapping'][mask].astype(int)
    observations = MatchObs(torch.arange(len(indices)), {
        k.removeprefix('match//'): torch.from_numpy(v[mask].copy())
        for k, v in saved.items() if k.startswith('match//')})
    points = PointNode(torch.arange(len(indices)), {
        'pos_Tw': torch.from_numpy(saved['points//pos_Tw'][point_indices].copy()),
        'cov_Tw': torch.from_numpy(saved['points//cov_Tw'][point_indices].copy())})
    parent = torch.tensor(event['parent_pose_camera_map_xyzw'], dtype=torch.float32)[None]
    data = GraphInput(torch.tensor([pair + 1]), torch.tensor([pair]), pp.SE3(parent),
                      torch.from_numpy(saved['frames//baseline'][pair + 1:pair + 2].copy()),
                      observations, points, torch.from_numpy(saved['frames//K'][pair + 1].copy()),
                      torch.zeros(len(indices), dtype=torch.long), 'cpu')
    assert len(indices) >= 3
    return data


class NativeThermalGraph(Analytic_ReprojDisp_TwoFramePGO):
    """Leave native RGB graph functions intact; append calibrated residual rows."""
    def __init__(self, data, packet=None, parent=None, thermal_precision=.1):
        super().__init__(data)
        self.has_thermal = packet is not None
        if not self.has_thermal:
            return
        if not 0 < thermal_precision <= 1:
            raise ValueError('positive fixed thermal precision required')
        # Packet source points already expressed in source RGB optical frame.
        local = packet['points'] @ O
        world = local @ parent[:3, :3].T + parent[:3, 3]
        eR, et = packet['camera_R'], packet['camera_t']
        self.register_buffer('thermal_world', torch.tensor(world, dtype=torch.double))
        self.register_buffer('thermal_R', torch.tensor(O.T @ eR @ O, dtype=torch.double))
        self.register_buffer('thermal_t', torch.tensor(et @ O, dtype=torch.double))
        self.register_buffer('thermal_Kb', torch.tensor(packet['Kb'], dtype=torch.double))
        self.register_buffer('thermal_observed', torch.tensor(packet['observations'], dtype=torch.double))
        self.register_buffer('thermal_cov', torch.tensor(packet['covariance'] / thermal_precision,
                                                        dtype=torch.double))

    def forward(self):
        rgb = super().forward()
        if not self.has_thermal:
            return rgb
        q = self.pose2opt.Inv().Act(self.thermal_world)
        self.thermal_q = q
        p = (self.thermal_R @ q.unsqueeze(-1)).squeeze(-1) + self.thermal_t
        self.thermal_p = p
        x, y, z = p.unbind(-1)
        fx, fy, cx, cy, baseline = self.thermal_Kb.unbind(-1)
        prediction = torch.stack((fx*y/x + cx, fy*z/x + cy, fx*baseline/x), -1)
        return torch.cat((rgb, prediction - self.thermal_observed), 0)

    def build_jacobian(self):
        rgb = super().build_jacobian()
        if not self.has_thermal:
            return rgb
        x, y, z = self.thermal_p.unbind(-1)
        fx, fy, _, _, baseline = self.thermal_Kb.unbind(-1)
        projection = torch.zeros((len(x), 3, 3), dtype=torch.double)
        projection[:, 0, 0] = -fx*y/x.square()
        projection[:, 0, 1] = fx/x
        projection[:, 1, 0] = -fy*z/x.square()
        projection[:, 1, 2] = fy/x
        projection[:, 2, 0] = -fx*baseline/x.square()
        rotation_inverse = self.pose2opt.rotation().matrix().transpose(-1, -2)
        body = torch.zeros((len(x), 3, 7), dtype=torch.double)
        body[:, :, :3] = -rotation_inverse
        body[:, :, 3:6] = rotation_inverse @ pp.vec2skew(self.thermal_world)
        thermal = (projection @ self.thermal_R @ body).reshape(-1, 7)
        return torch.cat((rgb, thermal), 0)

    def covariance_array(self):
        rgb = super().covariance_array()
        return torch.cat((rgb, self.thermal_cov), 0) if self.has_thermal else rgb


def optimize(graph):
    # Same native _optimize numerical body; omit only its GPU timing context
    # because this isolated runtime is CPU-only. No solver adaptation.
    graph = graph.to(device=torch.device('cpu'), dtype=torch.double)
    optimizer = LM_analytic(graph, min=1e-6, kernel=Huber(delta=.1), solver=PINV(),
                            strategy=TrustRegion(radius=1e3),
                            corrector=FastTriggs(Huber(delta=.1)), vectorize=True)
    scheduler = StopOnPlateau(optimizer, steps=10, patience=2, decreasing=1e-5, verbose=False)
    losses = []
    while scheduler.continual():
        weight = torch.block_diag(*torch.pinverse(graph.covariance_array().double()))
        loss = optimizer.step(input=(), weight=weight)
        scheduler.step(loss)
        losses.append(float(loss))
    matrix = graph.pose2opt.matrix().detach().numpy()[0]
    assert np.isfinite(matrix).all()
    return matrix, losses


def qualify():
    torch.set_num_threads(1)
    target = OUT / 'qualification-003'
    target.mkdir(parents=True, exist_ok=False)
    checks = []
    for condition in ['day', 'night', 'rain']:
        source, E, saved, events = load(condition)
        for pair in [0, 1, 2]:
            event = events['rgb'][(pair, pair + 1)]
            data = rgb_input(saved['rgb'], event, pair, source['records'])
            original = Analytic_ReprojDisp_TwoFramePGO(data).double()
            empty = NativeThermalGraph(data).double()
            assert torch.equal(original(), empty())
            assert torch.equal(original.build_jacobian(), empty.build_jacobian())
            assert torch.equal(original.covariance_array(), empty.covariance_array())
            result, losses = optimize(original)
            parent = pose(event['parent_pose_camera_map_xyzw'])
            relative = np.linalg.solve(parent, result)
            expected = pose(event['derived_camera_delta_xyzw'])
            error = np.linalg.solve(expected, relative)
            t_error = float(np.linalg.norm(error[:3, 3]))
            r_error = float(Rotation.from_matrix(error[:3, :3]).magnitude())
            packet, _ = bind_camera(saved['thermal'], events['thermal'].get((pair, pair+1)),
                                     pair, source['records'], 'thr', E['thermal'])
            combined = NativeThermalGraph(data, packet, parent).double()
            base = combined.pose2opt.detach().clone()
            residual = combined().detach().numpy()
            analytic = combined.build_jacobian().detach().numpy()[:, :6]
            columns = []
            for axis in range(6):
                delta = torch.zeros(1, 6, dtype=torch.double); delta[0, axis] = 1e-6
                with torch.no_grad():
                    combined.pose2opt.copy_(pp.se3(delta).Exp() @ base)
                plus = combined().detach().numpy().reshape(-1)
                with torch.no_grad():
                    combined.pose2opt.copy_(pp.se3(-delta).Exp() @ base)
                minus = combined().detach().numpy().reshape(-1)
                columns.append((plus-minus)/2e-6)
            with torch.no_grad():
                combined.pose2opt.copy_(base)
            numeric = np.stack(columns, axis=-1)
            jacobian_error = float(np.max(np.abs(numeric-analytic)/(1+np.abs(numeric))))
            # Audit actual unchanged float32-origin pose parameters: their
            # quaternion norm need not be exactly one after promotion to double.
            # Assuming an exact identity roundtrip is stronger than native code.
            inverse = combined.pose2opt.Inv().matrix().detach().numpy()[0]
            world = combined.thermal_world.detach().numpy()
            q = (world @ inverse[:3, :3].T + inverse[:3, 3]) @ O.T
            c = np.einsum('nij,nj->ni', packet['camera_R'], q)+packet['camera_t']
            k = packet['Kb']; optical = np.column_stack((k[:,0]*c[:,0]/c[:,2]+k[:,2],
                k[:,1]*c[:,1]/c[:,2]+k[:,3], k[:,0]*k[:,4]/c[:,2]))-packet['observations']
            assert np.allclose(residual[-len(q):], optical, atol=1e-8, rtol=0)
            checks.append({'condition': condition, 'pair': pair,
                           'native_translation_reproduction_m': t_error,
                           'native_rotation_reproduction_rad': r_error,
                           'jacobian_relative_error': jacobian_error, 'losses': losses,
                           'rgb_zero_thermal_exact': True, 'optical_projection_agrees': True})
            print(checks[-1], flush=True)
        del saved
    passed = all(c['native_translation_reproduction_m'] < 1e-4 and
                 c['native_rotation_reproduction_rad'] < 1e-4 and
                 c['jacobian_relative_error'] < 1e-4 for c in checks)
    write(target / 'checks.json', {'at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
          'checks': checks, 'passed': passed, 'torch': torch.__version__, 'pypose': pp.__version__,
          'sources': {str(p.relative_to(ROOT)): sha(p) for p in [Path(__file__),
              NATIVE/'Module/Optimization/PyposeOptimizers.py',
              NATIVE/'Module/Optimization/TwoFramePGO/Graphs.py',
              NATIVE/'Module/Optimization/TwoFramePGO/Optimizer.py']}})
    if not passed:
        raise RuntimeError('Qualification failed; no accuracy comparison authorized by protocol')


def capture():
    torch.set_num_threads(1)
    qualification = read(OUT / 'qualification-003/checks.json')
    assert qualification['passed']
    assert qualification['sources']['tools/native_thermal_constraints.py'] == sha(__file__)
    target = OUT / 'run-001'
    target.mkdir(parents=True, exist_ok=False)
    freeze = {'at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'source_sha256': sha(__file__), 'qualification_sha256': sha(OUT/'qualification-003/checks.json'),
              'conditions': ['day', 'night', 'rain'], 'pairs': [0, 598],
              'methods': ['native_rgb_replay', 'native_plus_thermal'], 'thermal_precision': .1,
              'contract': 'supplied_same_ID_no_duration_scaling', 'new_reference_scoring': False,
              'input_pins': {}}
    for c in freeze['conditions']:
        paths = [REC/'inputs'/c/'source-manifest.json', REC/'inputs'/c/'calibration.json']
        for camera in ['rgb', 'thermal']:
            paths += [REC/'captures'/c/camera/'tensor-map.npz',
                      REC/'captures'/c/camera/'arbiter-events/events.jsonl']
        freeze['input_pins'][c] = {str(p.relative_to(ROOT)): sha(p) for p in paths}
    write(target/'freeze.json', freeze)
    receipts = []
    for condition in freeze['conditions']:
        source, E, saved, events = load(condition)
        rows = []
        reproduction_errors = []
        for pair in range(599):
            event = events['rgb'].get((pair, pair+1))
            methods = {}
            binding = {}
            if event is None or event['original_need_interp'] or event['derived_camera_delta_xyzw'] is None:
                for method in freeze['methods']:
                    methods[method] = {'status': 'UNAVAILABLE_NATIVE_RGB_INPUT', 'transform': None}
            else:
                parent = pose(event['parent_pose_camera_map_xyzw'])
                data = rgb_input(saved['rgb'], event, pair, source['records'])
                matrix, losses = optimize(Analytic_ReprojDisp_TwoFramePGO(data))
                relative = np.linalg.solve(parent, matrix)
                error = np.linalg.solve(pose(event['derived_camera_delta_xyzw']), relative)
                t_error = float(np.linalg.norm(error[:3,3]))
                r_error = float(Rotation.from_matrix(error[:3,:3]).magnitude())
                reproduction_errors.append({'pair': pair, 'translation_m': t_error, 'rotation_rad': r_error})
                optical = np.eye(4); optical[:3,:3] = O
                native_delta = optical @ relative @ optical.T
                methods['native_rgb_replay'] = {'status': 'NATIVE_REPLAY', 'transform': native_delta.tolist(),
                                                'losses': losses}
                try:
                    packet, metadata = bind_camera(saved['thermal'], events['thermal'].get((pair,pair+1)),
                                                    pair, source['records'], 'thr', E['thermal'])
                    binding = {'thermal': metadata}
                except ValueError as error:
                    packet = None
                    binding = {'thermal_unavailable': str(error)}
                if packet is None:
                    methods['native_plus_thermal'] = {**methods['native_rgb_replay'],
                                                      'status': 'UNCHANGED_RGB_THERMAL_UNAVAILABLE'}
                else:
                    try:
                        # Independent graph instance: native RGB state is never mutated by the other arm.
                        fused_data = rgb_input(saved['rgb'], event, pair, source['records'])
                        matrix, losses = optimize(NativeThermalGraph(fused_data, packet, parent))
                        relative = np.linalg.solve(parent, matrix)
                        delta = optical @ relative @ optical.T
                        methods['native_plus_thermal'] = {'status': 'NATIVE_WITH_THERMAL',
                                                          'transform': delta.tolist(), 'losses': losses}
                    except (RuntimeError, ValueError, np.linalg.LinAlgError) as error:
                        methods['native_plus_thermal'] = {'status': 'FIT_FAILED', 'transform': None,
                                                          'reason': str(error)}
            rows.append({'pair': pair, 'methods': methods, 'binding': binding})
            if pair % 100 == 0:
                print('CAPTURE', condition, pair, flush=True)
        p = target/f'predictions-{condition}.json'
        write(p, {'rows': rows, 'references_read': False, 'sequence': source['sequence']})
        receipts.append({'condition': condition, 'prediction_sha256': sha(p),
                         'reproduction_errors': reproduction_errors,
                         'reproduction_passed': all(x['translation_m']<1e-4 and x['rotation_rad']<1e-4
                                                    for x in reproduction_errors)})
        del saved
    write(target/'prediction-lock.json', {'rows': receipts, 'source_sha256': sha(__file__),
                                          'references_read': False, 'freeze_sha256': sha(target/'freeze.json')})
    if not all(x['reproduction_passed'] for x in receipts):
        raise RuntimeError('Full cohort native reproduction failed; do not score fusion accuracy')


def score():
    from selected_motion_trajectory import rigid
    target = OUT/'run-001'
    lock = read(target/'prediction-lock.json')
    assert all(x['reproduction_passed'] for x in lock['rows'])
    summary = {}
    for receipt in lock['rows']:
        c = receipt['condition']; p = target/f'predictions-{c}.json'
        assert sha(p) == receipt['prediction_sha256']
        rows = read(p)['rows']
        refs = [rigid(np.loadtxt(REC/'references'/c/f'{i:06}.txt').reshape(4,4))[0] for i in range(600)]
        methods = {}
        for method in ['native_rgb_replay', 'native_plus_thermal']:
            def stats(start, end):
                result = path_stats(rows, refs, method, start, end)
                if result['status'] == 'COMPLETE':
                    result.pop('positions'); result.pop('reference')
                return result
            methods[method] = {'available': sum(r['methods'][method]['transform'] is not None for r in rows),
                               'full_path': stats(0,599),
                               'blocks': [{'start': i, **stats(i,i+99)} for i in range(0,600,100)]}
        summary[c] = methods
        print('SCORED', c, {m:x['full_path'] for m,x in methods.items()}, flush=True)
    write(target/'scores.json', {'summary': summary, 'prediction_lock_sha256': sha(target/'prediction-lock.json'),
                                'scope': 'Saved-map native optimizer replay, not full frontend/writeback VO.'})


if __name__ == '__main__':
    {'qualify': qualify, 'capture': capture, 'score': score}[sys.argv[1]]()
