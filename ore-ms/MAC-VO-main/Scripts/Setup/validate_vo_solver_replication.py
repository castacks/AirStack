"""Independent full-population audit for the declared middle100 replication."""
import argparse
import json
from collections import Counter
from pathlib import Path
import numpy as np
from Scripts.Setup.validate_vo_solver_control import (read, sha, pose, reference_reader, loss, moments, npz_equal, repair_trace, saved_context)


def close(a, b):
    if a is None or b is None:
        assert a is None and b is None
    else:
        np.testing.assert_allclose(a, b, rtol=2e-7, atol=1e-9)


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
    old_early = [r for r in read(old/'pose-proxy.json')['rows'] if r['window'] == 'middle']; extrinsics = {c: next(r['experts'][c]['nominal_body_from_camera'] for r in old_early if r['experts'][c] is not None) for c in ('rgb', 'thermal')}
    ref = reference_reader(root); rows = {(r['arm'], r['camera'], r['pair_ordinal']): r for r in evaluated['rows']}
    assert len(rows) == len(evaluated['rows']) == 594
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
        assert report['status'] == 'PASS_EXECUTION_ONLY' and report['input_rows'] == protocol['input_rows'] and report['completed_frames'] == 100
        assert overview['tracking_diagnostics'] == report['tracking_diagnostics'] and overview['tracking'] == report['tracking']
        events = [json.loads(line) for line in (path/'arbiter-events/events.jsonl').read_text().splitlines()]; raw_journals[arm, camera] = events
        assert [e['sequence'] for e in events] == list(range(len(events)))
        clocks = [e['observation_monotonic_ns'] for e in events]; assert all(type(t) is int for t in clocks) and clocks == sorted(clocks)
        assert overview['events'] == len(events); checked_events += len(events)
        summary = read(path/'arbiter-events/summary.json'); assert summary['status'] == 'COMPLETE' and not summary['observer_errors']
        refinement = min(e['sequence'] for e in events if e['kind'] == 'refinement_begin')
        with np.load(path/'tracking-flags.npz', allow_pickle=False) as flags:
            before, after = flags['before_refinement'], flags['after_refinement']; assert before.dtype == after.dtype == np.bool_ and before.shape == after.shape == (100,)
            assert report['tracking_diagnostics']['unsupported_pairs_before_refinement'] == int(before[1:].sum())
            assert report['tracking_diagnostics']['stored_flags_after_refinement'] == int(after.sum())
        stages = []
        for i in range(99):
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
        ids = [i for i in range(99) if all(lookup[arm, i]['common_loss'] is not None for arm in ('original', 'repair'))]
        assert matched['original_repair_pair_ordinals'] == ids
        for arm in ('original', 'passthrough', 'repair'):
            available = [lookup[arm, i] for i in range(99) if lookup[arm, i]['common_loss'] is not None]; full = matched['full'][arm]
            assert full['available'] == len(available) and full['missing'] == 99-len(available) and full['pair_ordinals'] == [r['pair_ordinal'] for r in available]
            for target in ('own_loss', 'common_loss'):
                for k in ('translation_m', 'rotation_deg'):
                    expected = moments([r[target][k] for r in available]); assert full[target][k]['count'] == expected['count']
                    for stat in ('mean', 'rmse'): close(full[target][k][stat], expected[stat])
            if arm in ('original', 'repair'):
                for k in ('translation_m', 'rotation_deg'):
                    expected = moments([lookup[arm, i]['common_loss'][k] for i in ids]); assert matched['original_repair_matched'][arm][k]['count'] == len(ids)
                    for stat in ('mean', 'rmse'): close(matched['original_repair_matched'][arm][k][stat], expected[stat])
        assert matched['full']['original']['own_loss'] == matched['full']['passthrough']['own_loss'] and matched['full']['original']['common_loss'] == matched['full']['passthrough']['common_loss']
    result = {'status': 'PASS', 'camera_runs': 6, 'event_records_checked': checked_events, 'interval_slots': len(rows), 'scored_intervals': sum(r['common_loss'] is not None for r in rows.values()), 'original_passthrough_bitwise': proof_parity,
        'original_repair_frontend_bitwise': frontend_parity, 'repair_attempts_checked': repair_attempts, 'repair_accepted_steps_checked': accepted_steps,
        'max_repair_relative_gradient_error': max_gradient_error, 'evaluation_sha256': sha(output/'evaluation.json'), 'execution_protocol_sha256': sha(output/'protocol.json'),
        'evaluation_protocol_sha256': sha(output/'evaluation-protocol.json'), 'validator_sha256': sha(Path(__file__)),
        'scope': 'independent numerical/event/reference-proxy arithmetic and preservation checks; not independent physical truth or real-time validation'}
    (output/'validation.json').write_text(json.dumps(result, indent=2)+'\n'); print(json.dumps(result))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('--root', type=Path, required=True); p.add_argument('--output', type=Path, required=True)
    a = p.parse_args(); validate(a.root, a.output)
