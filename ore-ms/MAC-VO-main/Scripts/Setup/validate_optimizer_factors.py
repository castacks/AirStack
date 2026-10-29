"""Independent forward/finite-difference and saved-factor provenance audit.

Does not import the diagnostic producer or load pose-error labels.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def rot(pose):
    x, y, z, w = np.asarray(pose, dtype=float).reshape(7)[3:]
    scale = np.sqrt(x*x+y*y+z*z+w*w); x, y, z, w = np.array([x, y, z, w])/scale
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def validate(root, run):
    report = json.loads((run/'audit.json').read_text())
    assert report['labels_loaded'] is False and report['fitted'] is False
    assert not any('proxy' in p or 'reference' in p for p in report['input_sha256'])
    for path, digest in report['input_sha256'].items(): assert sha(root/path) == digest
    checked = 0; factors = 0; terminal = 0; unavailable = 0; seen = set(); max_fd_error = 0.; max_landmark_error = 0.
    max_landmark_relative_error = 0.; max_point_norm = 0.
    cases = ('shared-arbiter-events', 'arbiter-support-collapse', 'history-day-replication')
    expected = set()
    for case in cases:
        source = root/'results'/case/'run-001'
        for camera in ('rgb', 'thermal'):
            expert = source/'experts'/camera
            with np.load(expert/'tensor-map.npz', allow_pickle=False) as data:
                expected.update((case, camera, i) for i in range(len(data['frames//time_ns'])-1))
    for row in report['rows']:
        identity = row['case'], row['camera'], row['pair_ordinal']
        assert identity not in seen; seen.add(identity)
        case, camera, i = identity; source = root/'results'/case/'run-001'; expert = source/'experts'/camera
        journal = [json.loads(line) for line in (expert/'arbiter-events/events.jsonl').read_text().splitlines()]
        timeline = [json.loads(line) for line in (source/'timeline.jsonl').read_text().splitlines()]
        d, = [e for e in timeline if e['kind'] == 'decision' and e['pair_ordinal'] == i]
        assert row['available_at_original_deadline'] == d['availability'][camera]
        with np.load(expert/'tensor-map.npz', allow_pickle=False) as a:
            times = [int(v) for v in a['frames//time_ns'][i:i+2]]
            assert row['time_ns'] == times
            h, = [e for e in journal if e['kind'] == 'pair_end' and [e['time0_ns'], e['time1_ns']] == times]
            selected = np.flatnonzero((a['edge/match2frame1/mapping'] == i) & (a['edge/match2frame2/mapping'] == i+1))
            ids = a['edge/match2point/mapping'][selected]
            assert len(ids) == row['retained_factor_count'] == h['retained_count']
            assert row['original_need_interp'] == h['original_need_interp']
            with np.load(expert/'uncertainty'/f'pair-{i:03d}.npz', allow_pickle=False) as raw:
                np.testing.assert_array_equal(raw['K'].reshape(3, 3), a['frames//K'][i])
                np.testing.assert_array_equal(a['frames//K'][i], a['frames//K'][i+1])
                take = np.flatnonzero(raw['selected_inbound'])[raw['outlier_keep_inbound']]
                for endpoint in (0, 1):
                    np.testing.assert_array_equal(raw[f'selected_uv{endpoint}'][take], a[f'match//pixel{endpoint+1}_uv'][selected])
                    np.testing.assert_array_equal(raw[f'selected_depth{endpoint}'][take], a[f'match//pixel{endpoint+1}_d'][selected].ravel())
            motions = [e for e in journal if e['kind'] == 'motion_result_written' and e['first_write_for_submission'] and e['from_idx'] == i and e['frame_idx'] == i+1]
            if not motions:
                assert row['diagnostics'] is None and row['motion_event_sequence'] is None and row['unavailable_reason'] == 'no_first_motion_write'
                assert not row['available_at_original_deadline']; unavailable += 1; continue
            m, = motions
            assert row['motion_event_sequence'] == m['sequence'] and not row['original_need_interp']
            assert row['native_pose_xyzw'] == m['optimizer_pose_before_write_native_parameterization']
            assert row['motion_observation_monotonic_ns'] == m['observation_monotonic_ns']
            assert m['sequence'] < min(e['sequence'] for e in journal if e['kind'] == 'refinement_begin')
            if row['available_at_original_deadline']:
                assert m['observation_monotonic_ns'] <= d['controller_monotonic_ns']
                assert d['evidence'][camera]['motion_event_sequence'] == m['sequence']
            else: terminal += 1
            pose = np.array(row['native_pose_xyzw']).reshape(7); parent = np.array(m['parent_pose_camera_map_xyzw'])
            points = a['points//pos_Tw'][ids].astype(float); k = a['frames//K'][i+1].astype(float)
            # Reconstruct saved landmarks from source camera observations and causal parent.
            uv0 = a['match//pixel1_uv'][selected]; depth = a['match//pixel1_d'][selected].ravel()
            local = np.column_stack((depth, (uv0[:, 0]-k[0, 2])*depth/k[0, 0], (uv0[:, 1]-k[1, 2])*depth/k[1, 1]))
            # Native point creation uses FP32 unprojection/SE3 action. Bound
            # cancellation by vector magnitude, not a nearly-zero coordinate.
            error = np.linalg.norm(local @ rot(parent).T+parent[:3]-points, axis=1)
            budget = 64*np.finfo(np.float32).eps*(1+np.linalg.norm(local, axis=1)+np.linalg.norm(parent[:3]))
            assert np.all(error <= budget)
            max_landmark_error = max(max_landmark_error, float(error.max()))
            max_landmark_relative_error = max(max_landmark_relative_error, float(np.max(error/(1+np.linalg.norm(local, axis=1)+np.linalg.norm(parent[:3])))))
            max_point_norm = max(max_point_norm, float(np.linalg.norm(local, axis=1).max()))
            uv = a['match//pixel2_uv'][selected]; disp = a['match//pixel2_disp'][selected].ravel()
            b = float(a['frames//baseline'][i+1]); q = (points-pose[:3]) @ rot(pose)
            def forward(delta):
                # A right-tangent curve with the exact differential at zero.
                p = (q-delta[:3]) @ Rotation.from_rotvec(delta[3:]).as_matrix()
                return np.column_stack((k[0, 0]*p[:, 1]/p[:, 0]+k[0, 2]-uv[:, 0],
                                        k[1, 1]*p[:, 2]/p[:, 0]+k[1, 2]-uv[:, 1], k[0, 0]*b/p[:, 0]-disp))
            residual = forward(np.zeros(6)); eps = 1e-6
            jac = np.stack([(forward(v*eps)-forward(-v*eps))/(2*eps) for v in np.eye(6)], axis=-1)
            cov = np.zeros((len(ids), 3, 3)); v = a['match//pixel2_uv_cov'][selected]
            cov[:, 0, 0] = v[:, 0]; cov[:, 1, 1] = v[:, 1]; cov[:, 0, 1] = cov[:, 1, 0] = v[:, 2]
            cov[:, 2, 2] = a['match//pixel2_disp_cov'][selected].ravel()
            weight = np.linalg.inv(cov)
            info = np.einsum('nki,nkl,nlj->ij', jac, weight, jac)
            z = row['diagnostics']; actual = np.array(z['information_right_camera_m_rad'])
            err = np.linalg.norm(actual-info)/np.linalg.norm(info); assert err < 2e-7; max_fd_error = max(max_fd_error, err)
            np.testing.assert_allclose(z['median_reprojection_px'], np.median(np.linalg.norm(residual[:, :2], axis=1)), atol=1e-8, rtol=1e-7)
            np.testing.assert_allclose(z['median_abs_disparity_px'], np.median(abs(residual[:, 2])), atol=1e-8, rtol=1e-7)
            np.testing.assert_allclose(z['whitened_sum_squares'], np.einsum('ni,nij,nj->', residual, weight, residual), rtol=1e-7, atol=1e-8)
            np.testing.assert_allclose(z['whitened_mean_per_component'], z['whitened_sum_squares']/(3*len(ids)), rtol=1e-12)
            assert z['factor_count'] == len(ids) and z['residual_dimension'] == 3*len(ids)
            assert z['pose_covariance'] is None and z['optimizer_convergence'] is None
            l = np.linalg.cholesky(cov); s = np.linalg.svd(np.linalg.solve(l, jac).reshape(-1, 6), compute_uv=False)
            np.testing.assert_allclose(z['whitened_jacobian_singular_values'], s, rtol=2e-6)
            rank = int(sum(s > max(3*len(ids), 6)*np.finfo(float).eps*s[0]))
            assert z['numerical_rank'] == rank
            assert z['mixed_units_condition_number'] is None if rank < 6 else np.isclose(z['mixed_units_condition_number'], s[0]/s[-1], rtol=2e-6)
            checked += 1; factors += len(ids)
    assert seen == expected and len(expected) == 434
    result = {'status': 'PASS', 'created_at': report['created_at'], 'audit_sha256': sha(run/'audit.json'),
              'validator_sha256': sha(Path(__file__)), 'slots': len(seen), 'motions': checked, 'factor_blocks': factors,
              'unwritten_slots': unavailable, 'motions_not_available_at_original_deadline': terminal,
              'max_relative_finite_difference_information_error': max_fd_error,
              'max_landmark_reconstruction_error_m': max_landmark_error,
              'max_landmark_relative_reconstruction_error': max_landmark_relative_error,
              'max_source_camera_point_norm_m': max_point_norm,
              'landmark_tolerance': '64*float32_eps*(1+norm(local_point)+norm(parent_translation)), vector norm',
              'scope': 'arithmetic, causal-pose and saved-factor identity validation; not calibration or pose accuracy'}
    (run/'validation.json').write_text(json.dumps(result, indent=2)+'\n'); print(json.dumps(result))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('--root', type=Path, required=True); p.add_argument('--run', type=Path, required=True)
    a = p.parse_args(); validate(a.root, a.run)
