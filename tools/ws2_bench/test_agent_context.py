import copy
import hashlib
import json

import pytest

from agent_context import compact_history, compact_diagnostics, encoded_bytes, resolve_telemetry_requests


def row(stamp, clearance):
    return {'sim_time_s': stamp, 'position_m': [stamp, 0., 1.2], 'clearance_m': clearance,
            'latest_input_stamp': stamp-.3, 'latest_input_rgb_noise_rmse': 12.,
            'latest_input_disturbance': {'fixed_delay_s': .3, 'rgb_noise_stddev': 16.,
                                        'configured': {'seed': 42, 'fixed_delay_s': .3},
                                        'noise_schedule': {'start_s': 0., 'duration_s': 20., 'period_s': 0.}},
            'sensor_disturbance': {'fixed_delay_s': .3, 'schedule_start_sim_s': 0., 'schedule_elapsed_s': stamp},
            'worker': {'sequence': int(stamp*100), 'sim_stamp': stamp-.3,
                       'steering': -.3, 'collision_probability': .9, 'speed': .03,
                       'run_id': '/home/test/runtime/old', 'preview_sha256': 'redundant', 'action': None},
            'command': {'sim_stamp': stamp-.1, 'published': True, 'velocity_m_s': .03,
                        'first_waypoint_m': [stamp, 0., 1.2], 'last_waypoint_m': [stamp+1., 0., 1.2],
                        'wall_time': 12345.}}


def run(root, name='run', records=None):
    folder = root/name; folder.mkdir()
    scenario = {'name': name, 'planner': 'pulp_dronet', 'condition': {'rgb_noise': 16.}}
    scenario_hash = hashlib.sha256(json.dumps(scenario, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    (folder/'scenario.json').write_text(json.dumps(scenario))
    records = records if records is not None else [row(float(i), .1 if i == 5 else .5) for i in range(11)]
    result = {'result_dir': str(folder), 'configuration_hash': scenario_hash, 'outcome': 'collision',
              'metrics': {'minimum_obstacle_clearance_m': -.01, 'path_length_m': 10.},
              'termination': {'source': 'planner', 'reason': 'physx_contact', 'detail': {'unneeded_path': str(folder)}},
              'attack_exposure': {'exposure_verified': False, 'status': 'evidence_error',
                                  'reason': 'Read /home/test/runtime/trace.json failed'},
              'diagnostics': {'sample_count': 11, 'window': 'last 5 simulation seconds',
                              'camera_age_sim_s': {'count': 11, 'min': .3, 'mean': .3, 'max': .3},
                              'snapshots': records[-3:], 'preview_frames': [], 'first_inference': None}}
    (folder/'diagnostic_trace.json').write_text(json.dumps(records))
    (folder/'result.json').write_text(json.dumps(result))
    return result


def history(root, count=3):
    values = []
    for number in range(1, count+1):
        result = run(root, f'run{number}')
        values.append({'decision': {'round': number, 'action': {'rgb_noise_stddev': number*5., 'obstacles': []},
                                    'experiment': {'kind': 'repeat' if number > 1 else 'explore',
                                                   'source_round': number-1, 'passing_round': 0, 'factor': 'none'},
                                    'hypothesis': 'A measured noisy input may cause a collision.'},
                       'pairs': [{'planner': 'pulp_dronet', 'clean': {**result, 'outcome': 'route_completed'},
                                  'perturbed': result}]})
    return values


def test_trace_snapshots_keep_first_minimum_last_and_observed_values(tmp_path):
    original = run(tmp_path); before = copy.deepcopy(original)
    compact = compact_diagnostics(original, 'r001_perturbed', tmp_path)
    assert [s['source']['sim_time_s'] for s in compact['snapshots']] == [0., 5., 10.]
    assert compact['retention']['available_samples'] == 11
    assert compact['retention']['omitted_samples'] == 8
    middle = compact['snapshots'][1]
    assert middle['selected_as'] == ['minimum_observed_clearance']
    assert middle['position_m'] == [5., 0., 1.2]
    assert middle['model_output']['collision_probability'] == .9
    assert middle['command']['published'] is True
    assert middle['bridge_released_input']['sensor_disturbance']['fixed_delay_s'] == .3
    assert middle['bridge_released_input']['rgb_noise_rmse'] == 12.
    assert 'configured' not in middle['bridge_released_input']['sensor_disturbance']
    assert 'run_id' not in middle['model_output']
    assert compact['aggregate_window']['window'] == 'last 5 simulation seconds'
    assert original == before


def test_all_source_actions_failures_and_reference_rounds_survive_recent_limit(tmp_path):
    original = history(tmp_path); saved = copy.deepcopy(original)
    compact = compact_history(original, recent_rounds=1, runtime_root=tmp_path)
    assert compact['retention']['detailed_rounds'] == [3]
    assert len(compact['rounds']) == 3
    for source, target in zip(original, compact['rounds']):
        assert target['action'] == source['decision']['action']
        assert target['experiment'] == source['decision']['experiment']
        assert target['pairs'][0]['perturbed']['outcome'] == 'collision'
        assert target['pairs'][0]['perturbed']['metrics']['minimum_obstacle_clearance_m'] == -.01
        assert target['pairs'][0]['perturbed']['attack_exposure']['exposure_verified'] is False
    assert 'diagnostics' not in compact['rounds'][0]['pairs'][0]['perturbed']
    assert 'diagnostics' in compact['rounds'][2]['pairs'][0]['perturbed']
    assert original == saved
    assert '/home/test/' not in json.dumps(compact)
    assert str(tmp_path) not in json.dumps(compact)


def test_sparse_fallback_does_not_claim_global_minimum_or_invent_missing_values():
    result = {'diagnostics': {'snapshots': [row(4., .4), row(9., .3)], 'sample_count': 1000}}
    compact = compact_diagnostics(result, 'r001_clean')
    assert compact['retention']['source'] == 'available_summary_snapshots'
    assert compact['retention']['minimum_scope'] == 'sparse available summary samples'
    assert compact['retention']['available_samples'] == 2
    assert len(compact['snapshots']) == 2
    assert 'action' not in compact['snapshots'][0]['model_output']
    assert compact_diagnostics({}, 'r001_clean')['snapshots'] == []


def test_trace_identity_and_runtime_confinement_fall_back_without_reading_external_data(tmp_path):
    result = run(tmp_path)
    compact = compact_diagnostics(result, 'r001_perturbed', tmp_path/'different')
    assert compact['retention']['source'] == 'available_summary_snapshots'
    altered = {**result, 'configuration_hash': 'changed'}
    compact = compact_diagnostics(altered, 'r001_perturbed', tmp_path)
    assert compact['retention']['trace_fallback_reason'] == 'configuration_hash_mismatch'
    path = tmp_path/'run/diagnostic_trace.json'; path.unlink()
    external = tmp_path/'outside.json'; external.write_text(json.dumps([row(500., -100.)]))
    path.symlink_to(external)
    compact = compact_diagnostics(result, 'r001_perturbed', tmp_path)
    assert all(s['source']['sim_time_s'] < 500 for s in compact['snapshots'])


def test_telemetry_window_uniformly_subsamples_real_rows_keeps_endpoints_and_hash(tmp_path):
    records = [row(i/100., .1 if i == 200 else .5) for i in range(501)]
    result = run(tmp_path, records=records)
    original = [{'decision': {'round': 1}, 'pairs': [{'perturbed': result}]}]
    request = {'evidence_id': 'r001_perturbed', 'view': 'telemetry', 'sim_time_s': 2.5}
    response = resolve_telemetry_requests(original, [request], tmp_path)['manifest']['requests'][0]
    assert response['status'] == 'available'
    assert response['requested_window_sim_s'] == [.5, 4.5]
    assert response['actual_window_sim_s'] == [.5, 4.5]
    assert response['available_samples'] == 401
    assert response['returned_samples'] == 20 and response['omitted_samples'] == 381
    assert response['samples'][0]['source']['sim_time_s'] == .5
    assert response['samples'][-1]['source']['sim_time_s'] == 4.5
    assert all(sample['source']['sim_time_s'] in [r['sim_time_s'] for r in records] for sample in response['samples'])
    data = (tmp_path/'run/diagnostic_trace.json').read_bytes()
    assert response['source']['trace_sha256'] == hashlib.sha256(data).hexdigest()
    assert response['source']['configuration_hash'] == result['configuration_hash']
    assert response['source']['scenario_hash_verified'] is True
    assert str(tmp_path) not in json.dumps(response)


def test_telemetry_null_is_latest_missing_window_is_not_substituted_and_requests_are_bounded(tmp_path):
    result = run(tmp_path)
    original = [{'decision': {'round': 1}, 'pairs': [{'perturbed': result}]}]
    request = {'evidence_id': 'r001_perturbed', 'view': 'telemetry', 'sim_time_s': None}
    value = resolve_telemetry_requests(original, [request], tmp_path)['manifest']['requests'][0]
    assert value['actual_window_sim_s'] == [8., 10.]
    value = resolve_telemetry_requests(original, [{**request, 'sim_time_s': 100.}], tmp_path)['manifest']['requests'][0]
    assert value['status'] == 'unavailable' and value['samples'] == []
    for requests in ([request]*4, [{**request, 'evidence_id': '../etc/passwd'}],
                     [{**request, 'sim_time_s': float('nan')}], [{**request, 'sim_time_s': True}],
                     [{**request, 'view': 'shell'}], [{**request, 'path': '/etc/passwd'}]):
        with pytest.raises(ValueError):
            resolve_telemetry_requests(original, requests, tmp_path)
    with pytest.raises(ValueError):
        resolve_telemetry_requests(original, [request], tmp_path, max_samples=100)


def test_duplicate_selected_roles_merge_and_context_bounds_are_explicit(tmp_path):
    result = run(tmp_path, records=[row(1., .1)])
    compact = compact_diagnostics(result, 'r001_clean', tmp_path)
    assert len(compact['snapshots']) == 1
    assert compact['snapshots'][0]['selected_as'] == ['first_available', 'minimum_observed_clearance', 'last_available']
    original = history(tmp_path, 1)
    value = compact_history(original, recent_rounds=0)
    assert value['retention']['detailed_rounds'] == []
    assert value['rounds'][0]['pairs'][0]['perturbed']['outcome'] == 'collision'
    for invalid in (-1, 13, True, 1.5):
        with pytest.raises(ValueError): compact_history(original, recent_rounds=invalid)
    assert encoded_bytes(value) > 0


def test_telemetry_does_not_read_trace_when_saved_scenario_is_changed(tmp_path):
    result = run(tmp_path)
    (tmp_path/'run/scenario.json').write_text('{"changed":true}')
    original = [{'decision': {'round': 1}, 'pairs': [{'perturbed': result}]}]
    request = {'evidence_id': 'r001_perturbed', 'view': 'telemetry', 'sim_time_s': None}
    value = resolve_telemetry_requests(original, [request], tmp_path)['manifest']['requests'][0]
    assert value['status'] == 'unavailable' and value['samples'] == []
    assert value['reason'] == 'scenario_hash_mismatch'


@pytest.mark.parametrize('invalid', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_observations_are_marked_not_zero_and_never_break_strict_json(tmp_path, invalid):
    record = row(1., .2)
    record['position_m'] = [1., invalid, 1.2]
    record['worker'].update(steering=invalid, collision_probability=.0, q_values=[.2, invalid, .3])
    record['command']['velocity_m_s'] = invalid
    record['latest_input_rgb_noise_rmse'] = invalid
    result = run(tmp_path, records=[record])
    result['metrics']['minimum_obstacle_clearance_m'] = invalid
    result['diagnostics']['camera_age_sim_s']['mean'] = invalid
    source = [{'decision': {'round': 1, 'action': {'rgb_noise_stddev': 16.},
                            'experiment': {'kind': 'explore'}},
               'pairs': [{'perturbed': result}]}]
    compact = compact_history(source, runtime_root=tmp_path)
    assert encoded_bytes(compact) > 0  # allow_nan=False
    flight = compact['rounds'][0]['pairs'][0]['perturbed']
    assert 'minimum_obstacle_clearance_m' not in flight['metrics']
    assert flight['metrics']['invalid_fields'] == ['minimum_obstacle_clearance_m']
    diagnostics = flight['diagnostics']; sample = diagnostics['snapshots'][0]
    assert sample['source'] == {'evidence_id': 'r001_perturbed', 'sim_time_s': 1.}
    assert 'position_m' not in sample and 'position_m[1]' in sample['invalid_fields']
    assert 'rgb_noise_rmse' not in sample['bridge_released_input']
    assert sample['bridge_released_input']['invalid_fields'] == ['rgb_noise_rmse']
    assert 'steering' not in sample['model_output'] and 'q_values' not in sample['model_output']
    assert sample['model_output']['invalid_fields'] == ['q_values[1]', 'steering']
    assert sample['model_output']['collision_probability'] == 0.  # a measured zero remains zero
    assert 'velocity_m_s' not in sample['command']
    assert sample['command']['invalid_fields'] == ['velocity_m_s']
    assert 'camera_age_sim_s.mean' in diagnostics['aggregate_window']['invalid_fields']
    request = {'evidence_id': 'r001_perturbed', 'view': 'telemetry', 'sim_time_s': None}
    telemetry = resolve_telemetry_requests(source, [request], tmp_path)
    assert encoded_bytes(telemetry) > 0
    assert telemetry['manifest']['requests'][0]['samples'][0]['model_output']['invalid_fields']


@pytest.mark.parametrize('invalid', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_source_action_is_rejected_rather_than_rewritten(invalid):
    source = [{'decision': {'round': 1, 'action': {'rgb_noise_stddev': invalid}}, 'pairs': []}]
    with pytest.raises(ValueError, match='non-finite.*source action'):
        compact_history(source)
    assert not (source[0]['decision']['action']['rgb_noise_stddev'] == 0.)


def test_context_byte_budget_reduces_details_to_two_then_zero_without_losing_core(tmp_path):
    source = history(tmp_path, 4); before = copy.deepcopy(source)
    full = compact_history(source, recent_rounds=4)
    two = compact_history(source, recent_rounds=2)
    zero = compact_history(source, recent_rounds=0)
    budget = (encoded_bytes(full)+encoded_bytes(two))//2
    reduced = compact_history(source, byte_budget=budget)
    assert encoded_bytes(reduced) <= budget
    assert reduced['retention']['detailed_rounds'] == [3, 4]
    assert reduced['retention']['detail_reduction_attempts'] == [4, 2]
    budget = (encoded_bytes(two)+encoded_bytes(zero))//2
    minimal = compact_history(source, byte_budget=budget)
    assert encoded_bytes(minimal) <= budget
    assert minimal['retention']['detailed_rounds'] == []
    assert minimal['retention']['detail_reduction_attempts'] == [4, 2, 0]
    for actual, original in zip(minimal['rounds'], source):
        assert actual['action'] == original['decision']['action']
        assert actual['experiment'] == original['decision']['experiment']
        assert actual['pairs'][0]['perturbed']['outcome'] == 'collision'
        assert actual['pairs'][0]['perturbed']['evidence_id'] == f"r{actual['round']:03d}_perturbed"
    assert source == before


def test_oversized_required_core_fails_without_truncating_exact_source_action():
    source = [{'decision': {'round': 1, 'action': {'opaque_source_field': 'x'*170_000},
                           'experiment': {'kind': 'repeat', 'source_round': 42}}, 'pairs': []}]
    before = copy.deepcopy(source)
    with pytest.raises(ValueError, match='required core.*byte_budget=160000.*cannot be truncated'):
        compact_history(source)
    assert source == before
    for bad in (0, -1, True, 1.5):
        with pytest.raises(ValueError, match='byte_budget'):
            compact_history([], byte_budget=bad)


def test_oversized_diagnostic_strings_arrays_and_depth_are_explicitly_truncated(tmp_path):
    record = row(1., .2)
    nested = {'small': 1}
    for _ in range(12): nested = {'nested': nested}
    record['worker'].update(q_values=list(range(100)), policy_profile='z'*20_000,
                            depth_summary_m=nested)
    result = run(tmp_path, records=[record])
    source = [{'decision': {'round': 1, 'action': {}}, 'pairs': [{'perturbed': result}]}]
    value = compact_history(source, runtime_root=tmp_path)
    assert encoded_bytes(value) <= 160000
    model = value['rounds'][0]['pairs'][0]['perturbed']['diagnostics']['snapshots'][0]['model_output']
    assert model['q_values'] == list(range(32))
    assert model['policy_profile'] == 'z'*1000
    records = model['truncated_fields']
    assert any(item['field'] == 'q_values' and item['original_items'] == 100 for item in records)
    assert any(item['field'] == 'policy_profile' and item['original_characters'] == 20_000 for item in records)
    assert any(item['reason'] == 'node_or_depth_limit' for item in records)
    request = {'evidence_id': 'r001_perturbed', 'view': 'telemetry', 'sim_time_s': None}
    telemetry = resolve_telemetry_requests(source, [request], tmp_path)
    inspected = telemetry['manifest']['requests'][0]['samples'][0]['model_output']
    assert inspected == model
    assert record['worker']['q_values'] == list(range(100))


def test_plan_intent_and_hypothesis_refutations_survive_zero_detail(tmp_path):
    source = history(tmp_path, 1)
    decision = source[0]['decision']
    decision.update(planning_version='visual_experiment_planning_v2',
                    hypothesis_reviews=[{'source_round': 1, 'assessment': 'contradicted', 'evidence': 'No collision.'}],
                    plan={'version': 'v2', 'next_budget_intent': 'Confirm the delayed-corner observation.',
                          'steps': [{'purpose': 'repeat', 'description': 'Same scene and exposure.'}],
                          'supports_if': 'Collision occurs.'})
    value = compact_history(source, recent_rounds=0)['rounds'][0]
    assert value['planning_version'] == 'visual_experiment_planning_v2'
    assert value['hypothesis_reviews'] == decision['hypothesis_reviews']
    assert value['plan'] == {'version': 'v2', 'next_budget_intent': 'Confirm the delayed-corner observation.'}
    detailed = compact_history(source, recent_rounds=1)['rounds'][0]
    assert detailed['plan']['steps'] == decision['plan']['steps']
