import ast
import copy
from pathlib import Path

import pytest

from consumed_input_audit import audit_consumed_inputs

RUN = '/verified/runtime/example'


def evidence(sequence=3, stamp=2., noise=16., delay=.3, revision=1):
    return {'schema_version': 1, 'sequence': sequence, 'sim_stamp': stamp,
            'disturbance_revision': revision,
            'sensor_disturbance': {'revision': revision, 'rgb_noise_stddev': noise,
                'fixed_delay_s': delay, 'noise_schedule': None, 'delay_schedule': None,
                'schedule_elapsed_s': stamp},
            'rgb_noise_rmse': 12. if noise else 0.}


def worker(consumed=None, sequence=3, stamp=2., run_id=RUN):
    value = {'run_id': run_id, 'sequence': sequence, 'sim_stamp': stamp}
    if consumed is not None: value['consumed_input'] = consumed
    return value


def released(consumed, observed=2.3, w=None):
    return {'sim_time_s': observed, 'camera_sequence': consumed['sequence'],
            'latest_input_stamp': consumed['sim_stamp'],
            'latest_input_disturbance': consumed['sensor_disturbance'],
            'latest_input_rgb_noise_rmse': consumed['rgb_noise_rmse'], 'worker': w}


@pytest.mark.parametrize('relative', [
    'MonoNav/mononav_airstack.py',
    'pulp-dronet/airstack_integration/worker.py',
    'Collision-avoidance/collision_avoidance_airstack.py'])
def test_actual_worker_metadata_helper_copies_same_frame_without_loading_model(relative):
    workspace = Path(__file__).resolve().parents[3]
    source = ast.parse((workspace/relative).read_text())
    helper = next(node for node in source.body if isinstance(node, ast.FunctionDef) and node.name == 'consumed_input_metadata')
    module = ast.Module(body=[helper], type_ignores=[]); namespace = {'copy': copy}
    exec(compile(module, relative, 'exec'), namespace)
    metadata = {'sequence': 3, 'stamp': 2., 'sensor_disturbance': {'rgb_noise_stddev': 16., 'revision': 2},
                'disturbance_revision': 2, 'rgb_noise_rmse': 12.}
    first = namespace['consumed_input_metadata'](metadata)
    metadata['sensor_disturbance']['rgb_noise_stddev'] = 0.
    metadata['sequence'] = 999  # later source/health data cannot replace first-frame evidence
    assert first['sequence'] == 3 and first['sim_stamp'] == 2.
    assert first['sensor_disturbance']['rgb_noise_stddev'] == 16.
    assert first['rgb_noise_rmse'] == 12. and first['disturbance_revision'] == 2
    legacy = namespace['consumed_input_metadata']({'sequence': 5, 'stamp': 4.})
    assert legacy['sensor_disturbance'] is None and legacy['rgb_noise_rmse'] is None
    assert any(isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and
               node.func.id == 'consumed_input_metadata' and node.args[0].id in ('meta', 'metadata')
               for node in ast.walk(source))


def test_direct_consumed_frame_remains_active_when_latest_released_frame_is_inactive():
    active = evidence(); inactive = evidence(4, 2.1, 0., 0.)
    rows = [released(inactive, 2.5, worker(active)), released(inactive, 2.6, worker(active))]
    result = audit_consumed_inputs(rows, run_id=RUN, condition={'rgb_noise': 16., 'delay': .3})
    assert result['observed_unique_consumed_frames'] == 1
    assert result['configuration_checked'] is True
    assert result['observed_released_frame_identities'] == 1
    assert result['evidence_sources']['direct_worker_metadata'] == 1
    assert result['factor_counts']['noise'] == {'active': 1, 'inactive': 0, 'unknown': 0}
    assert result['factor_counts']['delay']['active'] == 1
    frame = result['frames'][0]
    assert frame['recorded_output_observations'] == 2
    assert frame['first_output_observation_sim_s'] == 2.5
    assert frame['input_age_at_output_observation_s'] == .5
    assert frame['exact_delivery_latency_verified'] is False


def test_legacy_exact_join_matches_across_rows_not_current_health():
    active = evidence(); inactive = evidence(4, 2.1, 0., 0.)
    rows = [released(active), released(inactive, 2.5, worker())]
    result = audit_consumed_inputs(rows, run_id=RUN)
    assert result['configuration_checked'] is False
    assert result['evidence_sources']['same_run_exact_frame_join'] == 1
    assert result['frames'][0]['states'] == {'noise': 'active', 'delay': 'active'}


def test_skipped_released_frames_do_not_establish_consumed_attack_exposure():
    active = evidence(); inactive = evidence(4, 2.1, 0., 0.)
    rows = [released(active), released(inactive, 2.5, worker(sequence=4, stamp=2.1))]
    result = audit_consumed_inputs(rows, run_id=RUN)
    assert result['observed_released_frame_identities'] == 2
    assert result['observed_unique_consumed_frames'] == 1
    assert result['factor_counts']['noise'] == {'active': 0, 'inactive': 1, 'unknown': 0}
    unmatched = audit_consumed_inputs([released(active, w=worker(sequence=8, stamp=5.))], run_id=RUN)
    assert unmatched['factor_counts']['noise']['unknown'] == 1
    assert unmatched['frames'][0]['issues'] == ['no_exact_recorded_released_frame']


def test_invalid_direct_metadata_is_not_laundered_through_matching_legacy_frame():
    invalid = evidence(); invalid['sequence'] = 99
    result = audit_consumed_inputs([released(evidence(), w=worker(invalid))], run_id=RUN)
    assert result['evidence_sources']['unavailable'] == 1
    assert result['frames'][0]['states'] == {'noise': 'unknown', 'delay': 'unknown'}
    assert result['issues']['outer_inner_frame_identity_mismatch'] == 1
    a = evidence(); b = evidence(revision=2)
    result = audit_consumed_inputs([released(a), released(b, 2.5, worker())], run_id=RUN)
    assert result['frames'][0]['issues'] == ['ambiguous_exact_frame_join']
    invalid = evidence(); invalid['schema_version'] = True
    result = audit_consumed_inputs([released(evidence(), w=worker(invalid))], run_id=RUN)
    assert result['issues']['invalid_direct_consumed_schema'] == 1
    assert result['factor_counts']['noise']['unknown'] == 1


def test_recorded_sidecar_is_derived_compact_and_preserves_originals(tmp_path):
    import hashlib
    import json
    from consumed_input_audit import record_episode_consumption
    from episode import fingerprint
    folder = tmp_path/'run'; folder.mkdir()
    scenario = {'condition': {'rgb_noise': 16., 'delay': .3}}
    result = {'result_dir': str(folder), 'outcome': 'collision', 'configuration_hash': fingerprint(scenario)}
    data = evidence()
    rows = [released(data, w=worker(data, run_id=str(folder)))]
    for name, value in [('scenario.json', scenario), ('result.json', result), ('diagnostic_trace.json', rows)]:
        (folder/name).write_text(json.dumps(value))
    originals = {name: (folder/name).read_bytes() for name in ('scenario.json', 'result.json', 'diagnostic_trace.json')}
    compact = record_episode_consumption(result, tmp_path)
    assert compact['status'] == 'complete' and compact['configuration_checked'] is True
    assert 'frames' not in compact
    assert compact['factor_counts']['noise']['active'] == 1
    assert compact['source']['scenario_hash_verified'] is True
    artifact = (folder/'worker_input_exposure.json').read_bytes()
    assert hashlib.sha256(artifact).hexdigest() == compact['artifact']['sha256']
    assert len(json.loads(artifact)['frames']) == 1
    assert all((folder/name).read_bytes() == value for name, value in originals.items())
    with pytest.raises(ValueError): record_episode_consumption(result, tmp_path/'other_runtime')
    (folder/'scenario.json').write_text('{"changed":true}')
    unavailable = record_episode_consumption(result, tmp_path)
    assert unavailable['status'] == 'unavailable_evidence_identity' and unavailable['configuration_checked'] is False
    assert unavailable['reason'] == 'scenario_hash_mismatch'


def test_wrong_run_is_rejected_and_missing_or_nonfinite_pixels_are_unknown():
    result = audit_consumed_inputs([released(evidence(), w=worker(evidence(), run_id='/wrong/run'))], run_id=RUN)
    assert result['status'] == 'unavailable_no_same_run_worker_frames'
    assert result['issues']['worker_run_identity_mismatch'] == 1
    for value in (None, float('nan'), float('inf'), -1., 0.):
        data = evidence(); data['rgb_noise_rmse'] = value
        result = audit_consumed_inputs([{'sim_time_s': 1.9, 'worker': worker(data)}], run_id=RUN)
        assert result['factor_counts']['noise']['unknown'] == 1
        assert result['frames'][0]['input_age_at_output_observation_s'] is None


def test_schedule_boundary_is_source_capture_time_and_condition_mismatch_unknown():
    schedule = {'start_s': 1., 'duration_s': 1., 'period_s': 0.}
    data = evidence(stamp=2., noise=0., delay=0.)
    data['sensor_disturbance'].update(noise_schedule=schedule, delay_schedule=schedule)
    condition = {'rgb_noise': 16., 'delay': .3, 'noise_schedule': schedule, 'delay_schedule': schedule}
    result = audit_consumed_inputs([{'sim_time_s': 5., 'worker': worker(data)}], run_id=RUN, condition=condition)
    assert result['frames'][0]['states'] == {'noise': 'inactive', 'delay': 'inactive'}
    data['sensor_disturbance']['fixed_delay_s'] = .3
    result = audit_consumed_inputs([{'sim_time_s': 5., 'worker': worker(data)}], run_id=RUN, condition=condition)
    assert result['frames'][0]['states']['delay'] == 'unknown'
    assert result['frames'][0]['issues'] == ['delay_schedule_or_configuration_mismatch']
