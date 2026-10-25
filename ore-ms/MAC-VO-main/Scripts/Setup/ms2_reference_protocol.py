"""Audit frozen MS2 reference conventions; never choose a pose direction or fit time.

Outputs contain both direction hypotheses and source-protocol eligibility only.
They do not turn projected navigation/depth products into physical ground truth.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from Scripts.Setup.audit_ms2_pilot import read_calibration


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rotation_angle(rotation):
    # atan2 is stable near identity. No projection of a non-rigid source matrix.
    skew = np.array([rotation[2, 1] - rotation[1, 2],
                     rotation[0, 2] - rotation[2, 0],
                     rotation[1, 0] - rotation[0, 1]])
    return float(np.arctan2(np.linalg.norm(skew), np.trace(rotation) - 1))


def audit(root, manifest_path):
    root = Path(root)
    manifest = json.loads(Path(manifest_path).read_text())
    rows = manifest['records']
    assert [r['stem'] for r in rows] == [f'{i:06d}' for i in range(20)]
    pilot = root / 'results/ms2-night-pilot/pilot'
    sync = pilot / 'sync_data' / manifest['sequence']
    calib_path = sync / 'calib.npy'
    calibration = read_calibration(calib_path)
    frozen = {str(Path(manifest_path).resolve()): sha(manifest_path), str(calib_path): sha(calib_path)}
    result = {'scope': 'reference/protocol audit only; no selected pose direction or physical timing claim',
              'sequence': manifest['sequence'], 'source_ids': [r['stem'] for r in rows],
              'cameras': {}, 'pairs': [], 'frozen_inputs': frozen}
    for modality in ('rgb', 'thr'):
        poses = []
        rigidity = []
        source_times = {}
        for side in ('left', 'right'):
            path = sync / modality / f'img_{side}_timestamp.txt'
            frozen[str(path)] = sha(path)
            times = [int(x) for x in path.read_text().split()]
            assert len(times) == 10131 and all(b > a for a, b in zip(times, times[1:]))
            for i, row in enumerate(rows):
                assert times[i] == row['cameras'][modality][side]['recorded_time_ns']
            source_times[side] = times
        for row in rows:
            path = pilot / 'odom' / manifest['sequence'] / modality / (row['stem'] + '.txt')
            pose = np.loadtxt(path).reshape(4, 4)
            assert np.isfinite(pose).all() and np.array_equal(pose[3], [0, 0, 0, 1])
            orth = float(np.linalg.norm(pose[:3, :3].T @ pose[:3, :3] - np.eye(3)))
            det = float(np.linalg.det(pose[:3, :3]))
            assert orth < 1e-5 and abs(det - 1) < 1e-5
            rigidity.append({'orthogonality': orth, 'determinant': det})
            poses.append(pose)
            frozen[str(path)] = sha(path)
        R = np.asarray(calibration['R_' + modality + 'R']['values'])
        t = np.asarray(calibration['T_' + modality + 'R']['values'], dtype=float).reshape(3) / 1000
        K = np.asarray(calibration['K_' + modality + 'L']['values'])
        assert np.array_equal(R, np.eye(3)) and t[0] < 0 and np.array_equal(t[1:], [0, 0])
        assert K.tolist() == rows[0]['cameras'][modality]['K']
        assert abs(-t[0] - rows[0]['cameras'][modality]['baseline_m']) < 1e-12
        hypothesis = {}
        for name, absolute in [('camera_to_initial', poses),
                               ('initial_to_camera', [np.linalg.inv(p) for p in poses])]:
            # Under the named hypothesis, absolute is interpreted camera->initial.
            # The second case inverts the source first; neither is selected.
            steps = [np.linalg.inv(a) @ b for a, b in zip(absolute, absolute[1:])]
            hypothesis[name] = {
                'translation_step_m': [float(np.linalg.norm(p[:3, 3])) for p in steps],
                'rotation_step_rad': [rotation_angle(p[:3, :3]) for p in steps],
                'endpoint_translation_m': float(np.linalg.norm((np.linalg.inv(absolute[0]) @ absolute[-1])[:3, 3])),
                'endpoint_rotation_rad': rotation_angle((np.linalg.inv(absolute[0]) @ absolute[-1])[:3, :3]),
                'path_length_m': float(sum(np.linalg.norm(p[:3, 3]) for p in steps))}
        # Explicit support geometry only: no network outputs/error values influence eligibility.
        depth_support = {}
        import cv2
        for variant in ('depth', 'depth_multi', 'depth_filtered'):
            counts = []
            for row in rows:
                record = row['references'][modality][variant]
                path = root / record['path']
                assert sha(path) == record['sha256']
                frozen[str(path)] = sha(path)
                depth = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
                assert depth.dtype == np.uint16 and list(depth.shape) == row['cameras'][modality]['left']['shape'][:2]
                z = depth.astype(np.float64) / 256
                xx = np.arange(depth.shape[1])[None, :]
                valid = z > 0
                disparity = np.zeros_like(z)
                disparity[valid] = K[0, 0] * (-t[0]) / z[valid]
                right_x = xx - disparity
                # Bilinear sampling needs both floor/ceil neighbours in bounds.
                eligible = valid & (right_x >= 0) & (right_x < depth.shape[1] - 1)
                counts.append({'nonzero_depth': int(valid.sum()), 'right_image_in_bounds': int(eligible.sum())})
            depth_support[variant] = counts
        offsets = [b - a for a, b in zip(source_times['left'], source_times['right'])]
        result['cameras'][modality] = {
            'initial_identity_max_abs': float(np.max(np.abs(poses[0] - np.eye(4)))),
            'rigidity': rigidity, 'source_duration_ns': source_times['left'][19] - source_times['left'][0],
            'rectified_stereo_rotation_identity': True, 'right_translation_m': t.tolist(),
            'disparity_convention': 'd=u_left-u_right=fx*baseline/Z; forward flow_x=-d',
            'direction_hypotheses_not_selected': hypothesis, 'depth_support': depth_support,
            'full_stereo_within_2ms': sum(abs(x) <= 2_000_000 for x in offsets),
            'full_stereo_count': len(offsets)}
    for i, row in enumerate(rows):
        times = {m + '_' + s: row['cameras'][m][s]['recorded_time_ns'] for m in ('rgb', 'thr') for s in ('left', 'right')}
        rgb = times['rgb_right'] - times['rgb_left']
        thr = times['thr_right'] - times['thr_left']
        cross = times['thr_left'] - times['rgb_left']
        result['pairs'].append({'stem': row['stem'], 'times_ns': times,
                                'rgb_stereo_ns': rgb, 'thermal_stereo_ns': thr,
                                'cross_left_ns': cross,
                                'inherited_joint_gate': abs(rgb) <= 2_000_000 and abs(thr) <= 2_000_000 and abs(cross) <= 30_000_000})
    result['navigation_samples_present'] = len(list((sync / 'gps_imu/data').glob('*')))
    result['navigation_quality_verified'] = False
    result['physical_cross_modal_error_eligible'] = False
    result['provided_pair_depth_diagnostic_defined'] = True
    assert all(sha(path) == digest for path, digest in frozen.items())
    result['frozen_inputs_unchanged'] = True
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--manifest', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        p.error('Refuse to overwrite an audit')
    result = audit(args.root.resolve(), args.manifest.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print('Reference/protocol audit complete; physical cross-modal error eligibility remains false.')
