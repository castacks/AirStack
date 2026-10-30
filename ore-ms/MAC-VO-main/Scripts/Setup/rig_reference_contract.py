"""Reconstruct nominal rig paths from frozen sources, without fitting geometry."""
import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(root):
    frozen = {}
    def load(path, kind='json'):
        frozen[str(path.relative_to(root))] = sha(path)
        return (json.loads if kind == 'json' else yaml.safe_load)(path.read_text())

    calibration = root/'data/calibration/yamaha.yaml'
    config = load(calibration, 'yaml')
    assert sha(calibration) == '031c199619916bf1466a807d9ed91896d58b1428f0bdf74951dabc3c1b59e27d'
    graph = {}; edges = []
    for entry in config['transform_params']:
        parent, child = entry['from_frame'], entry['to_frame']
        q = np.array(entry['quaternion'], dtype=float)
        assert np.isfinite(q).all() and abs(np.linalg.norm(q)-1) <= 1e-4
        matrix = np.eye(4); matrix[:3, :3] = Rotation.from_quat(q).as_matrix()
        matrix[:3, 3] = entry['translation']
        edges.append(entry)
        graph.setdefault(parent, []).append((child, matrix))
        graph.setdefault(child, []).append((parent, np.linalg.inv(matrix)))

    def path_to(target):
        queue = deque([('vehicle', np.eye(4), ['vehicle'])]); visited = {'vehicle'}
        while queue:
            frame, composed, path = queue.popleft()
            if frame == target:
                return composed, path
            for child, matrix in graph.get(frame, []):
                if child not in visited:
                    visited.add(child); queue.append((child, composed@matrix, path+[child]))
        return None, None

    rows = []
    for name in ('end-to-end-solver-control', 'solver-collapse-replication', 'solver-day-replication'):
        base = root/'results'/name/'run-001'
        protocol = load(base/'protocol.json'); report = load(base/'evaluation.json'); proof = load(base/'validation.json')
        assert proof['status'] == 'PASS' and proof['evaluation_sha256'] == sha(base/'evaluation.json')
        assert proof['execution_protocol_sha256'] == sha(base/'protocol.json')
        ancestry = protocol['evaluation_input_sha256']
        prior_path = next(p for p in ancestry if p.endswith('/protocol.json'))
        prior_declaration = load(root/prior_path)
        assert sha(root/prior_path) == ancestry[prior_path]
        prior = prior_declaration['input_sha256']
        assert prior['data/calibration/yamaha.yaml'] == sha(calibration)
        for camera, optical, info_name in (
            ('rgb', 'multisense/left_camera_optical_frame', 'multisense_left_rect_color_camera_info'),
            ('thermal', 'thermal_left/optical_frame', 'thermal_left_processed_camera_info')):
            info_path = root/protocol['dataset_relative']/'sensors'/info_name/'data.yaml'
            info = load(info_path, 'yaml')
            # Verify against the original frozen ancestry as well as current bytes.
            assert prior[str(info_path.relative_to(root))] == sha(info_path)
            raw_r = np.array(info['r'], dtype=float)
            assert raw_r.shape == (3, 3) and np.isfinite(raw_r).all()
            defect = float(np.max(np.abs(raw_r.T@raw_r-np.eye(3))))
            assert defect <= 1e-6 and np.linalg.det(raw_r) > 0
            u, _, vt = np.linalg.svd(raw_r); rect = u@vt
            correction = np.eye(4)
            correction[:3, :3] = rect.T@np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]])
            body_from_optical, path = path_to(optical)
            assert path is not None
            reconstructed = body_from_optical@correction
            originals = [r for r in report['rows'] if r['arm'] == 'original' and r['camera'] == camera]
            discrepancies = [float(np.max(np.abs(reconstructed-np.array(r['nominal_body_from_camera'])))) for r in originals]
            assert discrepancies and max(discrepancies) < 1e-12
            rows.append(dict(run=name, camera=camera, static_path=path,
                camera_info_relative=str(info_path.relative_to(root)), rectification_orthogonality_defect=defect,
                rectification_svd_max_correction=float(np.max(np.abs(raw_r-rect))),
                nominal_body_from_camera=reconstructed.tolist(), rows_checked=len(originals),
                max_saved_matrix_absolute_difference=max(discrepancies)))
    gps, gps_path = path_to('gps_frame')
    assert gps is None and gps_path is None
    imu, imu_path = path_to('novatel/imu_frame'); assert imu_path is not None
    assert all(sha(root/p) == h for p, h in frozen.items())
    return dict(status='PASS_SOURCE_RECONSTRUCTION_ONLY', input_sha256=frozen,
        source_sha256=sha(Path(__file__)), transform_convention='T_parent_from_child; xyzw quaternion normalized by scipy',
        source_edges=edges, rows=rows, gps_frame_in_supplied_graph=False,
        nominal_novatel_imu_path=imu_path, nominal_vehicle_from_novatel_imu=imu.tolist(),
        gps_equals_imu_assumed=False, physical_body_transform_verified=False,
        independent_ground_truth=False, training_membership='unknown', prospective_test_recordings=[],
        no_new_inference=True, no_fit=True, no_reserved_access=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True); p.add_argument('--output', type=Path, required=True)
    args = p.parse_args(); result = run(args.root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as f:
        json.dump(result, f, indent=2, allow_nan=False); f.write('\n')
    print(json.dumps({'status': result['status'], 'camera_window_matrices': len(result['rows']),
                      'saved_matrix_rows_checked': sum(r['rows_checked'] for r in result['rows'])}))
