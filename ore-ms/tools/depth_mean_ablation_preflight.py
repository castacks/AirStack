"""Bounded depth-label qualification and unchanged native replay; no ablation."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results/depth-mean-ablation/preflight-001'
PLAN = ROOT/'experiments/depth-mean-ablation-preflight-2026-11-09.json'


def read(p):
    return json.loads(Path(p).read_text())


def write(p, value):
    p.parent.mkdir(parents=True, exist_ok=True)
    with p.open('x') as f:
        json.dump(value, f, indent=2, allow_nan=False)


def pose_error(a, b):
    a, b = np.asarray(a), np.asarray(b)
    return {'translation_m': float(np.linalg.norm(a[:3]-b[:3])),
            'rotation_deg': float(np.rad2deg((Rotation.from_quat(a[3:]).inv()*Rotation.from_quat(b[3:])).magnitude()))}


def label_coverage(sequence, frame_ids, member_names):
    expected = [f'{sequence}/{camera}/depth/{stem}.png' for camera in ['rgb', 'thr'] for stem in frame_ids]
    # Exact sequence/path suffix identity, never match on frame number alone.
    available = [n for n in expected if any(m.endswith('/labels/'+n) for m in member_names)]
    return {'expected_own_camera_maps': len(expected), 'available_exact_maps': available,
            'missing_exact_maps': [n for n in expected if n not in available]}


def run(condition):
    import hashlib
    plan = read(PLAN)
    for rel, digest in plan['pins'].items():
        assert hashlib.sha256((ROOT/rel).read_bytes()).hexdigest() == digest, rel
    sys.path.insert(0, str(ROOT/'MAC-VO-main'))
    from Scripts.Setup.solver_repair_experiment import native_graph_class, make_graph, original_solve
    cls = native_graph_class(ROOT)
    rows = []
    for case in plan['cases']:
        if case['condition'] != condition:
            continue
        cp = ROOT/case['expert_relative']
        config = read(cp/'effective-config.json')['Odometry']
        assert config['motion']['type'] == 'StaticMotionModel'
        assert config['optimizer']['args'] == {'device': 'cpu', 'vectorize': True, 'parallel': False, 'graph_type': 'disp', 'autodiff': False}
        assert config['args']['mapping'] is False
        graph = make_graph(ROOT, case, cls)
        initial_points = graph.pos_Tw.detach().cpu().numpy().copy()
        initial_cov = graph.covariance_array().detach().cpu().numpy().copy()
        result = original_solve(graph)
        delta = pose_error(case['recorded_native_pose_xyzw'], result['pose_xyzw'])
        passed = all(delta[k] <= v for k, v in plan['parity_tolerance'].items())
        np.testing.assert_array_equal(graph.pos_Tw.detach().cpu().numpy(), initial_points)
        np.testing.assert_array_equal(graph.covariance_array().detach().cpu().numpy(), initial_cov)
        parent = np.asarray(case['initial_pose_xyzw'])
        rp = Rotation.from_quat(parent[3:]).as_matrix()
        source_points = (initial_points-parent[:3])@rp
        uv = graph.obs.data['pixel1_uv'].numpy().astype(float)
        depth = graph.obs.data['pixel1_d'].numpy().astype(float).reshape(-1)
        with np.load(cp/'tensor-map.npz', allow_pickle=False) as saved:
            k = saved['frames//K'][case['pair_ordinal']].astype(float)
        reconstructed = np.column_stack([depth, (uv[:, 0]-k[0, 2])*depth/k[0, 0], (uv[:, 1]-k[1, 2])*depth/k[1, 1]])
        rows.append({'condition': condition, 'camera': case['camera'], 'pair': case['pair_ordinal'],
                     'factor_count': len(initial_points), 'parity_pass': passed, 'parity_error': delta,
                     'source_depth_ray_max_abs_m': float(np.max(np.abs(source_points-reconstructed))),
                     'source_point_ray_scope': 'check against original source observation in native NED frame; not reference truth',
                     'native_solver_result': result})
    result = {'condition': condition, 'replays': len(rows), 'passed': sum(r['parity_pass'] for r in rows),
              'runtime_metrics': False, 'new_frontend_calls': 0, 'depth_changing_arms_executed': 0,
              'created_at': datetime.now(timezone.utc).isoformat(), 'rows': rows}
    write(OUT/f'parity-{condition}.json', result)
    print({k: v for k, v in result.items() if k != 'rows'})


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--condition', required=True, choices=['day', 'night', 'rain'])
    run(ap.parse_args().condition)
