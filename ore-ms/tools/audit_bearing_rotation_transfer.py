"""Saved-output day/night audit independent of candidate solver/scoring code."""
import json
from pathlib import Path
import numpy as np
from scipy.linalg import fractional_matrix_power
from scipy.spatial.transform import Rotation
from audit_bearing_rotation_assay import normalized, angle, stats, close

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'results/bearing-rotation-transfer/run-001'


def read(path):
    return json.loads(Path(path).read_text())


def audit():
    import bearing_rotation_transfer as transfer
    from pose_covariance_accuracy import transforms
    from score_pose_covariance_accuracy import reference_pose
    transfer.verify()
    all_checks = {}; summaries = {}; total_costs = total_motions = total_slots = 0
    global_lock = read(BASE/'global-prediction-lock.json')
    for condition in ['day', 'night']:
        transfer.verify_condition(condition)
        target = BASE/'inputs'/condition; out = BASE/'assay'/condition
        lock = read(out/'prediction-lock.json')
        assert global_lock[condition] == transfer.core.sha(out/'prediction-lock.json')
        for name, digest in lock['pins'].items(): assert transfer.core.sha(out/name) == digest
        metadata = read(target/'metadata.json'); source = read(target/'source-manifest.json')
        frame = transforms(read(ROOT/metadata['calibration']),
            ROOT/'results/pose-covariance/cohort-preflight/snapshot/ore-ms/results/provided-reference-full-record/run-001/source-001/official-loader.py')['thermal']
        optical_frame = frame[:3, :3]@np.array([[0., 0, 1], [1, 0, 0], [0, 1, 0]])
        poses = [reference_pose(ROOT/p)[0] for p in metadata['reference_rgb_poses']]
        refs = {i: None if poses[i] is None or poses[i+1] is None else (np.linalg.inv(poses[i])@poses[i+1])[:3, :3] for i in range(19)}
        clocks = [[source['records'][i]['cameras']['rgb']['left']['recorded_time_ns'],
                   source['records'][i+1]['cameras']['rgb']['left']['recorded_time_ns']] for i in range(19)]
        events = [json.loads(s) for s in (BASE/'native'/condition/'thermal/arbiter-events/events.jsonl').read_text().splitlines()]
        writes = {(e['from_idx'], e['frame_idx']): e for e in events if e['kind'] == 'motion_result_written' and e['first_write_for_submission']}
        predictions, scored = read(out/'predictions.json'), read(out/'scores.json')
        errors = {key: {} for key in scored['coverage']}; costs = motions = slots = 0
        assert len(predictions['rows']) == len(scored['rows']) == 19 and predictions['solver_calls'] <= 114
        for row, report in zip(predictions['rows'], scored['rows']):
            i = row['pair']; assert report['pair'] == i
            a, b = row['time0_ns'], row['time1_ns']; c, d = clocks[i]
            horizon = max(a-c, c-b, a-d, d-b, 0) <= b-a
            with np.load(target/'packets'/f'{i:03}.npz', allow_pickle=False) as packet, np.load(out/'support'/f'{i:03}.npz', allow_pickle=False) as support:
                selected = packet['common']&support['original']&support['replacement']
                np.testing.assert_array_equal(selected, support['common']); assert int(selected.sum()) == row['after_filter']
                f0 = normalized(packet['source_uv'][selected], packet['source_K'].reshape(3, 3))[0]
                covariance = packet['covariance'][selected]
                for mean, objectives in row['methods'].items():
                    f1, jac = normalized(packet[mean+'_uv'][selected], packet['target_K'].reshape(3, 3))
                    for name, result in objectives.items():
                        slots += 1; key = mean+'/'+name; entry = report['entries'][key]
                        if result['rotation'] is not None:
                            r, t = np.array(result['rotation']), np.array(result['translation_direction'])
                            close(r.T@r, np.eye(3)); close(np.linalg.det(r), 1); close(np.linalg.norm(t), 1)
                            cost = 0.
                            for n in range(len(f0)):
                                numerator = t@np.cross(f0[n], r@f1[n]); variance = 1.
                                if name != 'NEC':
                                    pixel = np.eye(2) if name == 'PNEC_unit_pixel_covariance' else covariance[n]
                                    j = jac[n]; u = r.T@np.cross(t, f0[n])
                                    variance = u@(j@pixel@j.T)@u+1e-12
                                cost += numerator*numerator/variance
                            close(cost, result['final_cost']); costs += 1
                        if result['status'] != 'CONVERGED' or not horizon:
                            assert entry['mapped_rotation'] is None and entry['error_deg'] is None
                            continue
                        mapped = fractional_matrix_power(optical_frame@r@optical_frame.T, (d-c)/(b-a))
                        assert np.max(np.abs(np.imag(mapped))) < 1e-8
                        mapped = np.real(mapped); close(mapped, entry['mapped_rotation'])
                        if refs[i] is None:
                            assert entry['error_deg'] is None
                        else:
                            e = angle(mapped, refs[i]); close(e, entry['error_deg']); errors[key][i] = e; motions += 1
            event = writes.get((i, i+1)); mapped = None
            if event is not None and event['derived_camera_delta_xyzw'] is not None:
                a0, b0 = event['time0_ns'], event['time1_ns']; assert [a0, b0] == [a, b]
                if max(a0-c, c-b0, a0-d, d-b0, 0) <= b0-a0:
                    quaternion = np.array(event['derived_camera_delta_xyzw'])[3:]
                    close(np.linalg.norm(quaternion), 1)
                    r = Rotation.from_quat(quaternion).as_matrix()
                    mapped = np.real(fractional_matrix_power(frame[:3, :3]@r@frame[:3, :3].T, (d-c)/(b0-a0)))
            entry = report['entries']['native']
            if mapped is None:
                assert entry['mapped_rotation'] is None and entry['error_deg'] is None
            else:
                close(mapped, entry['mapped_rotation'])
                if refs[i] is not None:
                    e = angle(mapped, refs[i]); close(e, entry['error_deg']); errors['native'][i] = e
        for key in errors:
            assert scored['coverage'][key] == len(errors[key])
            expected = stats(list(errors[key].values()))
            for metric, value in expected.items():
                if value is None: assert scored['own_support_rotation_deg'][key][metric] is None
                else: close(scored['own_support_rotation_deg'][key][metric], value)
        n, candidate, native = [errors[k] for k in ['original/NEC', 'original/PNEC_original_pixel_covariance', 'native']]
        common = sorted(set(n)&set(candidate)&set(native)); assert scored['primary']['matched_pair_ids'] == common
        primary_stats = {k: stats([errors[k][i] for i in common]) for k in ['original/NEC', 'original/PNEC_original_pixel_covariance', 'native']}
        checks = {'nonempty_matched_support': bool(common), 'no_NEC_coverage_loss': set(n) <= set(candidate)}
        for metric in ['mean', 'rmse']:
            checks['improves_NEC_'+metric] = bool(common) and primary_stats['original/PNEC_original_pixel_covariance'][metric] < primary_stats['original/NEC'][metric]
            checks['not_worse_native_'+metric] = bool(common) and primary_stats['original/PNEC_original_pixel_covariance'][metric] <= primary_stats['native'][metric]
        assert checks == scored['primary']['checks']
        assert scored['primary']['decision'] == ('RAIN_DEVELOPMENT_GATE_PASS' if all(checks.values()) else 'STOP_THIS_PROTOTYPE_CONFIGURATION')
        for key in primary_stats:
            for metric, value in primary_stats[key].items():
                if value is None: assert scored['primary']['matched_rotation_deg'][key][metric] is None
                else: close(scored['primary']['matched_rotation_deg'][key][metric], value)
        all_checks[condition] = all(checks.values())
        summaries[condition] = {'retained_slots': slots, 'reconstructed_costs': costs, 'mapped_motion_errors': motions,
                                'primary_matched_intervals': len(common), 'primary_gate_pass': all(checks.values())}
        total_costs += costs; total_motions += motions; total_slots += slots
    summary = read(BASE/'summary.json')
    assert summary['recording_gates'] == all_checks
    assert summary['decision'] == ('DAY_NIGHT_DEVELOPMENT_GATES_PASS' if all(all_checks.values()) else 'STOP_TRANSFER_PROMOTION_NO_RETUNING')
    result = {'status': 'PASS', 'recordings': summaries, 'retained_slots': total_slots,
              'reconstructed_costs': total_costs, 'mapped_motion_errors': total_motions,
              'scope': 'Independent saved objective/mapping/metric/coverage/decision checks; shared reference serialization qualification and calibration lookup; no neural or optimizer rerun'}
    with (BASE/'audit.json').open('x') as f: json.dump(result, f, indent=2)
    print(result)


if __name__ == '__main__': audit()
