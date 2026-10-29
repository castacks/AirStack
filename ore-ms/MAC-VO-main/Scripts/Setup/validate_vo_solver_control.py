"""Independent event/array/backend and recorded-reference arithmetic checks."""
import argparse
from collections import Counter
import contextlib
from decimal import Decimal
import io
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from Scripts.Setup.validate_solver_repair import saved_context, values, update


def read(p): return json.loads(Path(p).read_text())
def sha(p):
    import hashlib
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def close(a, b): np.testing.assert_allclose(a, b, rtol=2e-7, atol=1e-9)


def pose(x):
    x = np.asarray(x, dtype=float).reshape(7); assert np.isfinite(x).all() and abs(np.linalg.norm(x[3:])-1) < 1e-5
    m = np.eye(4); m[:3, :3] = Rotation.from_quat(x[3:]).as_matrix(); m[:3, 3] = x[:3]; return m


def reference_reader(root):
    path = root/'data/tartandrive3/20251027/night_mac_salon_warehousefig8/super_odometry/odometry'
    times = np.array([int((Decimal(line)*1_000_000_000).to_integral_value()) for line in (path/'interp_timestamps.txt').read_text().splitlines()], dtype=np.int64)
    samples = np.loadtxt(path/'interp_data.txt')[:, :7]; assert len(samples) == len(times) and np.all(np.diff(times) > 0)
    def at(t):
        assert type(t) is int and times[0] <= t <= times[-1]
        i = min(max(int(np.searchsorted(times, t, side='right'))-1, 0), len(times)-2)
        alpha = (t-int(times[i]))/(int(times[i+1])-int(times[i])); p = (1-alpha)*samples[i, :3]+alpha*samples[i+1, :3]
        q0, q1 = samples[i, 3:].copy(), samples[i+1, 3:].copy(); q0 /= np.linalg.norm(q0); q1 /= np.linalg.norm(q1)
        dot = float(np.dot(q0, q1))
        if dot < 0: q1 = -q1; dot = -dot
        angle = math.acos(max(-1, min(1, dot)))
        q = (1-alpha)*q0+alpha*q1 if angle < 1e-8 else (math.sin((1-alpha)*angle)*q0+math.sin(alpha*angle)*q1)/math.sin(angle)
        return pose(np.r_[p, q/np.linalg.norm(q)])
    return at


def loss(prediction, truth):
    error = np.linalg.inv(truth)@prediction; r = error[:3, :3]
    sin = np.linalg.norm([r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1]])/2
    return {'translation_m': float(np.linalg.norm(error[:3, 3])), 'rotation_deg': math.degrees(math.atan2(sin, (np.trace(r)-1)/2))}


def moments(x):
    return {'count': len(x), 'mean': math.fsum(x)/len(x), 'rmse': math.sqrt(math.fsum(v*v for v in x)/len(x))} if x else {'count': 0, 'mean': None, 'rmse': None}


def npz_equal(first, second, paths):
    count = 0
    for path in paths:
        with np.load(first/path, allow_pickle=False) as a, np.load(second/path, allow_pickle=False) as b:
            assert set(a.files) == set(b.files)
            for name in a.files:
                assert a[name].shape == b[name].shape and a[name].dtype == b[name].dtype and a[name].tobytes() == b[name].tobytes(); count += 1
    return count


def repair_trace(solution, initial, context):
    # Anchor to independently checked serialized states rather than accumulating
    # repeated SE3 arithmetic differences across iterations. Near convergence,
    # dividing two tiny independently rounded reductions amplifies roundoff.
    best = np.asarray(initial).ravel(); best_loss = solution['initial_loss']; damping = .001; accepted = 0; error_max = 0.; checked = set()
    close(values(best, context)[0], best_loss)
    for a in solution['attempts']:
        assert np.array_equal(np.array(a['parameters_before'][0]).ravel(), best)
        close(a['loss_before'], best_loss); close(a['damping'], damping)
        cost, h, g, _ = values(best, context)
        # Preserve the EXP083 independent normal-equation tolerance, including
        # its absolute floor for cancellation in near-zero gradient components.
        np.testing.assert_allclose(a['H'], h, rtol=2e-7, atol=1e-8)
        np.testing.assert_allclose(a['g'], g, rtol=2e-7, atol=1e-8)
        close(cost, best_loss)
        if a['outer'] not in checked:
            checked.add(a['outer']); eps = 1e-8
            fd = [(values(update(best, v*eps, True), context)[0]-values(update(best, -v*eps, True), context)[0])/(2*eps) for v in np.eye(7)]
            error = float(np.linalg.norm(np.array(fd)-2*g.ravel())/(1+np.linalg.norm(2*g)))
            assert error < 1e-3; error_max = max(error_max, error)
        d = np.array(a['D']); reported_h, reported_g = np.array(a['H']), np.array(a['g'])
        matrix = reported_h.copy(); matrix[np.diag_indices_from(matrix)] = np.clip(np.diag(reported_h), 1e-6, 1e32)*(1+damping)
        np.testing.assert_allclose(d, np.linalg.pinv(matrix)@-reported_g, rtol=3e-6, atol=1e-8)
        if a['reason'] == 'SMALL_STEP':
            assert np.linalg.norm(d) <= 1e-10 and not a['accepted'] and solution['termination'] == 'NO_PROGRESS_SMALL_STEP'; break
        predicted = float(-(2*reported_g.T@d+d.T@reported_h@d).item())
        np.testing.assert_allclose(a['predicted_reduction'], predicted, rtol=1e-9, atol=1e-12)
        if predicted <= 0:
            assert not a['accepted'] and a['reason'] == 'NONPOSITIVE_MODEL_REDUCTION'; damping = min(1e12, damping*10); continue
        candidate = np.array(a['candidate_parameters'][0]).ravel()
        np.testing.assert_allclose(candidate, update(best, d, True), rtol=1e-8, atol=1e-9)
        close(a['candidate_loss'], values(candidate, context)[0])
        actual = a['loss_before']-a['candidate_loss']; quality = actual/a['predicted_reduction']
        np.testing.assert_allclose(a['actual_reduction'], actual, rtol=1e-10, atol=1e-13)
        np.testing.assert_allclose(a['quality'], quality, rtol=1e-10, atol=1e-12)
        assert a['accepted'] == (actual > 0 and quality >= 1e-4)
        if a['accepted']:
            best = candidate; best_loss = a['candidate_loss']; accepted += 1
            damping = max(1e-12, min(1e12, damping/2 if quality > .75 else damping*10 if quality < .25 else damping))
        else: damping = min(1e12, damping*10)
    assert np.array_equal(np.array(solution['parameters'][0]).ravel(), best)
    close(solution['returned_loss'], values(best, context)[0]); assert solution['best_loss'] == solution['returned_loss'] == best_loss <= solution['initial_loss']
    assert solution['accepted_steps'] == accepted
    if solution['termination'] == 'MAX_ITERATIONS': assert checked == set(range(10)) and solution['attempts'][-1]['accepted']
    elif solution['termination'] == 'GRADIENT_TOLERANCE': assert np.max(abs(values(best, context)[2])) <= 1e-9
    else: assert solution['termination'] == 'NO_PROGRESS_SMALL_STEP'
    return len(solution['attempts']), accepted, error_max


def validate(root, output):
    protocol = read(output/'protocol.json'); execution = read(output/'execution.json'); evaluated = read(output/'evaluation.json'); evaluation_protocol = read(output/'evaluation-protocol.json')
    assert execution['status'] == 'PASS_EXECUTION_CONTROLS_ONLY' and execution['protocol_sha256'] == sha(output/'protocol.json')
    assert evaluated['evaluation_protocol_sha256'] == sha(output/'evaluation-protocol.json') and not evaluated['fit_performed'] and not evaluated['independent_ground_truth']
    assert not evaluation_protocol['fit_performed'] and not evaluation_protocol['physical_body_transform_verified'] and not evaluation_protocol['independent_ground_truth']
    for section in ('source_sha256', 'input_sha256', 'checkpoint_sha256', 'evaluation_input_sha256'):
        for path, digest in protocol[section].items(): assert sha(root/path) == digest
    for path, digest in evaluation_protocol['input_sha256'].items(): assert sha(root/path) == digest
    old = root/'results/arbiter-pose-proxy/run-002'; old_proof = read(old/'validation.json')
    assert old_proof['status'] == 'PASS' and old_proof['report_sha256'] == sha(old/'pose-proxy.json')
    old_early = [r for r in read(old/'pose-proxy.json')['rows'] if r['window'] == 'early']; extrinsics = {c: old_early[0]['experts'][c]['nominal_body_from_camera'] for c in ('rgb', 'thermal')}
    ref = reference_reader(root); rows = {(r['arm'], r['camera'], r['pair_ordinal']): r for r in evaluated['rows']}
    assert len(rows) == len(evaluated['rows']) == 114
    assert set(evaluated['runs']) == {arm+'/'+c for arm in ('original', 'passthrough', 'repair') for c in ('rgb', 'thermal')} and set(evaluated['matched']) == {'rgb', 'thermal'}
    proof_parity = {}; frontend_parity = {}; raw_journals = {}; checked_events = repair_attempts = accepted_steps = 0; max_gradient_error = 0.
    for camera in ('rgb', 'thermal'):
        first = output/'runs/original'/camera; second = output/'runs/passthrough'/camera; changed = output/'runs/repair'/camera
        paths = sorted(p.relative_to(first) for p in first.rglob('*.npz'))
        assert paths == sorted(p.relative_to(second) for p in second.rglob('*.npz'))
        proof_parity[camera] = {'files': len(paths), 'arrays': npz_equal(first, second, paths)}
        assert execution['parity'][camera]['arrays_bitwise_equal'] == proof_parity[camera]['arrays']
        frontend = [path for path in paths if str(path) == 'first-depth.npz' or path.parts[0] == 'uncertainty']
        frontend_parity[camera] = {'files': len(frontend), 'arrays': npz_equal(first, changed, frontend)}
        assert read(first/'effective-config.json') == read(second/'effective-config.json') == read(changed/'effective-config.json')
    for spec in protocol['runs']:
        arm, camera = spec['arm'], spec['camera']; path = root/spec['output_relative']; report = read(path/'report.json'); overview = evaluated['runs'][arm+'/'+camera]
        assert report['status'] == 'PASS_EXECUTION_ONLY' and report['input_rows'] == protocol['input_rows'] and report['completed_frames'] == 20
        assert overview['tracking_diagnostics'] == report['tracking_diagnostics'] and overview['tracking'] == report['tracking']
        events = [json.loads(line) for line in (path/'arbiter-events/events.jsonl').read_text().splitlines()]; raw_journals[arm, camera] = events
        assert [e['sequence'] for e in events] == list(range(len(events)))
        clocks = [e['observation_monotonic_ns'] for e in events]; assert all(type(t) is int for t in clocks) and clocks == sorted(clocks)
        assert overview['events'] == len(events); checked_events += len(events)
        summary = read(path/'arbiter-events/summary.json'); assert summary['status'] == 'COMPLETE' and not summary['observer_errors']
        refinement = min(e['sequence'] for e in events if e['kind'] == 'refinement_begin')
        with np.load(path/'tracking-flags.npz', allow_pickle=False) as flags:
            before, after = flags['before_refinement'], flags['after_refinement']; assert before.dtype == after.dtype == np.bool_ and before.shape == after.shape == (20,)
            assert report['tracking_diagnostics']['unsupported_pairs_before_refinement'] == int(before[1:].sum())
            assert report['tracking_diagnostics']['stored_flags_after_refinement'] == int(after.sum())
        stages = []
        for i in range(19):
            row = rows[arm, camera, i]; expected_times = [protocol['input_rows'][j][camera+'_left']['time_ns'] for j in (i, i+1)]
            common_times = [protocol['input_rows'][j]['rgb_left']['time_ns'] for j in (i, i+1)]
            assert row['sensor_interval_ns'] == expected_times and row['common_interval_ns'] == common_times and row['nominal_body_from_camera'] == extrinsics[camera]
            h, = [e for e in events if e['kind'] == 'pair_end' and [e['time0_ns'], e['time1_ns']] == expected_times]
            assert row['original_need_interp'] == h['original_need_interp'] == bool(before[i+1]) and h['execution_error'] is None
            with np.load(path/'uncertainty'/f'pair-{i:03d}.npz', allow_pickle=False) as raw:
                assert int(raw['time0_ns']) == expected_times[0]
                assert h['selected_count'] == len(raw['selected_uv0']) and h['inbound_count'] == int(raw['selected_inbound'].sum()) and h['retained_count'] == int(raw['outlier_keep_inbound'].sum())
                points = raw['selected_uv0'][raw['selected_inbound']][raw['outlier_keep_inbound']]; height, width = raw['height_width']; cells = np.floor(points/[width, height]*3).astype(int)
                assert h['occupied_3x3_cells'] == len(np.unique(cells[:, 1]*3+cells[:, 0]))
            motions = [e for e in events if e['kind'] == 'motion_result_written' and e['first_write_for_submission'] and e['from_idx'] == i and e['frame_idx'] == i+1]
            if not motions or h['original_need_interp']:
                assert row['unavailable_reason'] is not None and row['common_loss'] is None; continue
            m, = motions; assert [m['time0_ns'], m['time1_ns']] == expected_times and m['sequence'] < refinement and not m['original_need_interp']
            assert m['sequence'] == row['motion_event_sequence'] and m['stage'] == row['motion_stage']; stages.append(m['stage'])
            assert m['optimizer_convergence'] is None and m['pose_covariance'] is None
            native_delta = np.linalg.inv(pose(m['parent_pose_camera_map_xyzw']))@pose(m['committed_pose_camera_map_xyzw']); close(pose(m['derived_camera_delta_xyzw']), native_delta)
            r0, r1 = [ref(t) for t in common_times]; a, b = [ref(t) for t in expected_times]
            for name, value in {'common_start': r0, 'common_end': r1, 'own_start': a, 'own_end': b}.items(): close(row['reference_poses'][name], value)
            e = np.array(extrinsics[camera]); body = e@native_delta@np.linalg.inv(e); aligned = np.linalg.inv(r0)@a@body@np.linalg.inv(b)@r1
            close(row['own_body_delta'], body); close(row['common_body_delta'], aligned)
            for name, expected in [('own_loss', loss(body, np.linalg.inv(a)@b)), ('common_loss', loss(aligned, np.linalg.inv(r0)@r1))]:
                for k, value in expected.items(): close(row[name][k], value)
        assert overview['first_motion_stages'] == dict(Counter(stages))
        if arm != 'original':
            adapter = read(path/'solver-adapter.json'); assert adapter['error'] is None and adapter['instance_method_restored'] and adapter['factory_restored']
            assert [s['ordinal'] for s in adapter['records']] == list(range(len(adapter['records'])))
            assert {(s['from_idx'], s['frame_idx']) for s in adapter['records']} == {(e['from_idx'], e['frame_idx']) for e in events if e['kind'] == 'optimization_submitted'}
            assert len(adapter['records']) == summary['optimization_submissions']
            assert overview['solver_statuses'] == dict(Counter(r['status'] for r in adapter['records']))
            assert overview['solver_elapsed_ns'] == [r['elapsed_ns'] for r in adapter['records']] and overview['cuda_peak_allocated_bytes'] == adapter['cuda_peak_allocated_bytes']
            for s in adapter['records']:
                i = s['from_idx']; assert s['frame_idx'] == i+1 and s['elapsed_ns'] == s['finished_monotonic_ns']-s['started_monotonic_ns'] >= 0
                m, = [e for e in events if e['kind'] == 'motion_result_written' and e['first_write_for_submission'] and e['from_idx'] == i and e['frame_idx'] == i+1]
                submission, = [e for e in events if e['kind'] == 'optimization_submitted' and e['from_idx'] == i and e['frame_idx'] == i+1]
                complete, = [e for e in events if e['kind'] == 'optimization_submit_returned' and e['job']['submission_id'] == submission['submission_id']]
                assert submission['observation_monotonic_ns'] <= s['started_monotonic_ns'] <= s['finished_monotonic_ns'] <= complete['observation_monotonic_ns'] <= m['observation_monotonic_ns']
                assert [s['time0_ns'], s['time1_ns']] == [m['time0_ns'], m['time1_ns']]
                close(s['pose_xyzw'], np.asarray(m['optimizer_pose_before_write_native_parameterization']).ravel())
                if arm == 'repair':
                    assert s['status'] == s['solver']['termination'] and s['status'] not in ('INVALID_INPUT', 'LINEARIZATION_ERROR', 'LINEAR_SOLVER_ERROR', 'REJECTION_LIMIT')
                    context = saved_context(root, {'expert_relative': spec['output_relative'], 'pair_ordinal': i})
                    initial = np.asarray(m['parent_pose_camera_map_xyzw'], dtype=np.float32).astype(float)
                    attempts, accepted, error = repair_trace(s['solver'], initial, context); repair_attempts += attempts; accepted_steps += accepted; max_gradient_error = max(max_gradient_error, error)
    for camera in ('rgb', 'thermal'):
        stripped = lambda events: [{k: v for k, v in e.items() if k != 'observation_monotonic_ns'} for e in events]
        assert stripped(raw_journals['original', camera]) == stripped(raw_journals['passthrough', camera])
        matched = evaluated['matched'][camera]; lookup = {(r['arm'], r['pair_ordinal']): r for r in evaluated['rows'] if r['camera'] == camera}
        ids = [i for i in range(19) if all(lookup[arm, i]['common_loss'] is not None for arm in ('original', 'repair'))]
        assert matched['original_repair_pair_ordinals'] == ids
        for arm in ('original', 'passthrough', 'repair'):
            available = [lookup[arm, i] for i in range(19) if lookup[arm, i]['common_loss'] is not None]; full = matched['full'][arm]
            assert full['available'] == len(available) and full['missing'] == 19-len(available) and full['pair_ordinals'] == [r['pair_ordinal'] for r in available]
            for target in ('own_loss', 'common_loss'):
                for k in ('translation_m', 'rotation_deg'):
                    expected = moments([r[target][k] for r in available]); assert full[target][k]['count'] == expected['count']
                    for stat in ('mean', 'rmse'): close(full[target][k][stat], expected[stat])
            if arm in ('original', 'repair'):
                for k in ('translation_m', 'rotation_deg'):
                    expected = moments([lookup[arm, i]['common_loss'][k] for i in ids]); assert matched['original_repair_matched'][arm][k]['count'] == len(ids)
                    for stat in ('mean', 'rmse'): close(matched['original_repair_matched'][arm][k][stat], expected[stat])
        assert matched['full']['original']['own_loss'] == matched['full']['passthrough']['own_loss'] and matched['full']['original']['common_loss'] == matched['full']['passthrough']['common_loss']
    result = {'status': 'PASS', 'camera_runs': 6, 'event_records_checked': checked_events, 'scored_intervals': len(rows), 'original_passthrough_bitwise': proof_parity,
        'original_repair_frontend_bitwise': frontend_parity, 'repair_attempts_checked': repair_attempts, 'repair_accepted_steps_checked': accepted_steps,
        'max_repair_relative_gradient_error': max_gradient_error, 'evaluation_sha256': sha(output/'evaluation.json'), 'execution_protocol_sha256': sha(output/'protocol.json'),
        'evaluation_protocol_sha256': sha(output/'evaluation-protocol.json'), 'validator_sha256': sha(Path(__file__)),
        'scope': 'independent numerical/event/reference-proxy arithmetic and preservation checks; not independent physical truth or real-time validation'}
    (output/'validation.json').write_text(json.dumps(result, indent=2)+'\n'); print(json.dumps(result))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('--root', type=Path, required=True); p.add_argument('--output', type=Path, required=True)
    a = p.parse_args(); validate(a.root, a.output)
