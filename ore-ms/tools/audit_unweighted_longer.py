"""Independent saved-cost, rotation mapping and extension decision audit."""
import json
import numpy as np
from scipy.linalg import fractional_matrix_power
from scipy.spatial.transform import Rotation
from audit_bearing_rotation_assay import normalized, angle, stats, close
import unweighted_longer as run


def read(path): return json.loads(path.read_text())


def inside(a, b, clock):
    c, d = clock
    return max(a-c, c-b, a-d, d-b, 0) <= b-a


def power(r, ratio):
    mapped = fractional_matrix_power(r, ratio)
    assert np.max(np.abs(np.imag(mapped))) < 1e-8
    return np.real(mapped)


def audit():
    from pose_covariance_accuracy import transforms
    from score_pose_covariance_accuracy import reference_pose
    run.verify(); root, base = run.ROOT, run.BASE
    global_lock = read(base/'global-prediction-lock.json'); gates = {}; records = {}
    for condition in run.CASES:
        run.verify_condition(condition)
        out = base/'assay'/condition; target = base/'inputs'/condition
        assert global_lock[condition] == run.core.sha(out/'prediction-lock.json')
        for name, digest in read(out/'prediction-lock.json')['pins'].items(): assert run.core.sha(out/name) == digest
        metadata, source = read(target/'metadata.json'), read(target/'source-manifest.json')
        frames = transforms(read(root/metadata['calibration']), root/'results/pose-covariance/cohort-preflight/snapshot/ore-ms/results/provided-reference-full-record/run-001/source-001/official-loader.py')
        poses = [reference_pose(root/p)[0] for p in metadata['reference_rgb_poses']]
        refs = [None if a is None or b is None else (np.linalg.inv(a)@b)[:3, :3] for a, b in zip(poses, poses[1:])]
        clocks = {sensor: [[source['records'][j]['cameras'][sensor]['left']['recorded_time_ns'] for j in [i, i+1]] for i in range(99)] for sensor in ['rgb', 'thr']}
        events = {}
        for mode in ['rgb', 'thermal']:
            values = [json.loads(s) for s in (base/'native'/condition/mode/'arbiter-events/events.jsonl').read_text().splitlines()]
            events[mode] = {(e['from_idx'], e['frame_idx']): e for e in values if e['kind'] == 'motion_result_written' and e['first_write_for_submission']}
        predictions, scores = read(out/'predictions.json'), read(out/'scores.json')
        assert len(predictions['rows']) == len(scores['rows']) == 99 and predictions['solver_calls'] <= 99
        errors = {k: {} for k in ['candidate', 'native_rgb', 'native_thermal']}; costs = motions = 0
        for row, report in zip(predictions['rows'], scores['rows']):
            i = row['pair']; assert i == report['pair']
            result = row['candidate']; mapped = None
            with np.load(target/'packets'/f'{i:03}.npz', allow_pickle=False) as packet, np.load(out/'support'/f'{i:03}.npz', allow_pickle=False) as support:
                selected = packet['common']&support['original']&support['replacement']
                np.testing.assert_array_equal(selected, support['common']); assert int(selected.sum()) == row['after_filter']
                if result['rotation'] is not None:
                    r, t = np.array(result['rotation']), np.array(result['translation_direction'])
                    close(r.T@r, np.eye(3)); close(np.linalg.det(r), 1); close(np.linalg.norm(t), 1)
                    f0 = normalized(packet['source_uv'][selected], packet['source_K'].reshape(3, 3))[0]
                    f1 = normalized(packet['replacement_uv'][selected], packet['target_K'].reshape(3, 3))[0]
                    cost = sum(float(t@np.cross(a, r@b))**2 for a, b in zip(f0, f1))
                    close(cost, result['final_cost']); costs += 1
                    if result['status'] == 'CONVERGED' and inside(row['time0_ns'], row['time1_ns'], clocks['rgb'][i]):
                        f = frames['thermal'][:3, :3]@np.array([[0., 0, 1], [1, 0, 0], [0, 1, 0]])
                        mapped = power(f@r@f.T, (clocks['rgb'][i][1]-clocks['rgb'][i][0])/(row['time1_ns']-row['time0_ns']))
            predicted = {'candidate': mapped}
            for mode, sensor in [('rgb', 'rgb'), ('thermal', 'thr')]:
                e = events[mode].get((i, i+1)); mapped = None
                if e is not None and e['derived_camera_delta_xyzw'] is not None:
                    a, b = e['time0_ns'], e['time1_ns']; assert [a, b] == clocks[sensor][i]
                    if inside(a, b, clocks['rgb'][i]):
                        quaternion = np.array(e['derived_camera_delta_xyzw'])[3:]; close(np.linalg.norm(quaternion), 1)
                        r = Rotation.from_quat(quaternion).as_matrix(); f = frames[mode][:3, :3]
                        mapped = power(f@r@f.T, (clocks['rgb'][i][1]-clocks['rgb'][i][0])/(b-a))
                predicted['native_'+mode] = mapped
            for key, mapped in predicted.items():
                entry = report['entries'][key]
                if mapped is None: assert entry['mapped_rotation'] is None
                else: close(mapped, entry['mapped_rotation'])
                if mapped is None or refs[i] is None: assert entry['error_deg'] is None
                else:
                    e = angle(mapped, refs[i]); close(e, entry['error_deg']); errors[key][i] = e; motions += 1
        for field, allowed in [('primary_extension', set(range(19, 99))), ('full_prefix', set(range(99)))]:
            reported = scores[field]; populations = {k: set(v)&allowed for k, v in errors.items()}
            common = sorted(set.intersection(*populations.values())); assert reported['matched_pair_ids'] == common
            metrics = {k: stats([v[i] for i in common]) for k, v in errors.items()}
            checks = {'nonempty_common': bool(common), 'no_RGB_coverage_loss': populations['native_rgb'] <= populations['candidate'],
                      'no_thermal_coverage_loss': populations['native_thermal'] <= populations['candidate']}
            for control in ['native_rgb', 'native_thermal']:
                for metric in ['mean', 'rmse']:
                    checks['improves_'+control+'_'+metric] = bool(common) and metrics['candidate'][metric] < metrics[control][metric]
            assert reported['checks'] == checks and reported['pass'] == all(checks.values())
            assert reported['coverage'] == {k: len(v) for k, v in populations.items()}
            for key, values in metrics.items():
                for metric, value in values.items():
                    if value is None: assert reported['rotation_deg'][key][metric] is None
                    else: close(reported['rotation_deg'][key][metric], value)
        gates[condition] = scores['primary_extension']['pass']
        records[condition] = {'retained_intervals': 99, 'audited_costs': costs, 'mapped_errors': motions, 'extension_pass': gates[condition]}
    summary = read(base/'summary.json')
    assert summary['recording_gates'] == gates
    assert summary['decision'] == ('ADVANCE_TO_SEPARATE_FULL_MOTION_DESIGN' if all(gates.values()) else 'CLOSE_UNWEIGHTED_ROUTE_NO_RETUNING')
    result = {'status': 'PASS', 'recordings': records,
        'scope': 'Independent scalar NEC cost, matrix-power frame/time mapping, errors, full/extension metrics, coverage and decision; shared reference serialization qualification and calibration lookup, no network/optimizer rerun'}
    with (base/'audit.json').open('x') as f: json.dump(result, f, indent=2)
    print(result)


if __name__ == '__main__': audit()
