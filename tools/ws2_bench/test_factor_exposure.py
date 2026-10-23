"""Synthetic measurements only: no model, simulator, notifications or files."""
import copy

import pytest

from factor_exposure import assess, assess_condition


def sensor_audit(noise=False, delay=False):
    return {'exposure_verified': True, 'factors': {
        name: {'requested': active, 'exposure_verified': True,
               'status': 'exposed' if active else 'disabled', 'active_frames': 2 if active else 0}
        for name, active in [('noise', noise), ('delay', delay)]}}


def pair(exposure=None, patch_visibility=None):
    factors = exposure.get('factors', {}) if isinstance(exposure, dict) else {}
    factors = factors if isinstance(factors, dict) else {}
    consumed = {'schema': 'observed_consumed_input_v1', 'status': 'complete',
                'population': 'observed unique worker-consumed source-frame updates',
                'configuration_checked': True, 'observed_unique_consumed_frames': 2,
                'factor_counts': {key: {'active': 2 if factors.get(key, {}).get('requested') else 0,
                                       'inactive': 0 if factors.get(key, {}).get('requested') else 2, 'unknown': 0}
                                  for key in ('noise', 'delay')},
                'evidence_sources': {'direct_worker_metadata': 2, 'same_run_exact_frame_join': 0, 'unavailable': 0}}
    return {'clean': {'outcome': 'route_completed', 'executed': True},
            'perturbed': {'outcome': 'collision', 'executed': True,
                          'attack_exposure': exposure, 'worker_input_exposure': consumed,
                          'patch_visibility': patch_visibility}}


def visibility():
    return {'status': 'visible', 'visibility_verified': True, 'method': 'rendered_instance_segmentation',
            'source_validation': 'bench_verified', 'patch_prim_path': '/World/SyntheticPatch',
            'patch_texture_sha256': 'a'*64, 'frames': [
                {'input_stamp_sim_s': 14., 'rgb_sha256': 'b'*64, 'segmentation_sha256': 'c'*64,
                 'visible_patch_pixels': 120, 'patch_enabled': True, 'frame_binding_verified': True,
                 'worker_consumption_verified': True}]}


def test_disabled_sensor_audit_does_not_verify_patch_or_change_outcome():
    p = pair(sensor_audit()); original = copy.deepcopy(p)
    result = assess({'patch_enabled': True}, p)
    assert result['eligible'] is False and result['factor_evidence']['patch']['status'] == 'unknown'
    assert p == original and p['perturbed']['outcome'] == 'collision'


def test_visible_patch_does_not_require_depth_change_or_claim_causality():
    result = assess({'patch_enabled': True}, pair(None, visibility()))
    assert result['eligible'] is True
    assert result['factor_evidence']['patch']['visible_frames'] == 1
    assert result['causal_validation'] is False
    assert result['factor_evidence']['patch']['causal_validation'] is False


@pytest.mark.parametrize('change', [
    {'method': 'projected_in_frustum'}, {'method': 'llm_image_opinion'},
    {'source_validation': 'manual_claim'}, {'visibility_verified': False}, {'frames': []},
    {'patch_texture_sha256': None}, {'patch_prim_path': None},
])
def test_patch_application_or_untrusted_visibility_is_not_measurement(change):
    record = visibility(); record.update(change)
    assert not assess({'patch_enabled': True}, pair(sensor_audit(), record))['eligible']


@pytest.mark.parametrize('change', [
    {'patch_enabled': False}, {'frame_binding_verified': False}, {'visible_patch_pixels': 0},
    {'visible_patch_pixels': True}, {'input_stamp_sim_s': float('nan')}, {'rgb_sha256': 'missing'},
    {'worker_consumption_verified': False},
])
def test_patch_requires_positive_visible_pixels_bound_to_active_input_frame(change):
    record = visibility(); record['frames'][0].update(change)
    assert not assess({'patch_enabled': True}, pair(None, record))['eligible']


def test_noise_and_delay_require_their_own_verified_samples():
    assert assess({'rgb_noise_stddev': 10}, pair(sensor_audit(noise=True)))['eligible']
    assert assess({'delay_s': .5}, pair(sensor_audit(delay=True)))['eligible']
    assert not assess({'rgb_noise_stddev': 10}, pair({'exposure_verified': True}))['eligible']
    assert not assess({'delay_s': .5}, pair(sensor_audit()))['eligible']
    mixed = assess({'rgb_noise_stddev': 10, 'patch_enabled': True}, pair(sensor_audit(noise=True)))
    assert mixed['factor_evidence']['rgb_noise']['verified'] and not mixed['eligible']
    assert assess({'rgb_noise_stddev': 10, 'patch_enabled': True},
                  pair(sensor_audit(noise=True), visibility()))['eligible']


@pytest.mark.parametrize('changes', [
    {'active_frames': 0}, {'active_frames': True}, {'requested': False},
    {'exposure_verified': False}, {'status': 'exposure_unknown'},
])
def test_sensor_aggregate_cannot_override_missing_active_frame_proof(changes):
    audit = sensor_audit(noise=True); audit['factors']['noise'].update(changes)
    assert not assess({'rgb_noise_stddev': 10}, pair(audit))['eligible']


def test_aggregate_false_cannot_veto_verified_active_factor_for_unrelated_inactive_metadata():
    audit = sensor_audit(noise=True)
    audit['exposure_verified'] = False
    audit['factors']['delay'].update(exposure_verified=False, status='exposure_unknown')
    result = assess({'rgb_noise_stddev': 10}, pair(audit))
    assert result['eligible'] and result['factor_evidence']['rgb_noise']['verified']
    # The same unavailable measurement is required when delay is actually active.
    assert not assess({'rgb_noise_stddev': 10, 'delay_s': .5}, pair(audit))['eligible']


def test_global_evidence_error_invalidates_sensor_audit_even_with_positive_stale_counts():
    audit = sensor_audit(noise=True); audit['status'] = 'evidence_error'
    result = assess({'rgb_noise_stddev': 10}, pair(audit))
    assert not result['eligible']
    assert result['factor_evidence']['rgb_noise']['reason'] == 'sensor_audit_evidence_error'


def test_baseline_clean_failure_unexecuted_and_infrastructure_are_not_attack_scores():
    assert assess({}, pair(sensor_audit()))['reason'] == 'all_off_baseline'
    for role, changes in [('clean', {'outcome': 'collision'}),
                          ('perturbed', {'executed': False}),
                          ('perturbed', {'outcome': 'infrastructure_error'})]:
        p = pair(sensor_audit(noise=True)); p[role].update(changes)
        assert not assess({'rgb_noise_stddev': 10}, p)['eligible']


def test_legacy_missing_measurement_is_unknown_and_report_names_are_supported():
    assert not assess({'rgb_noise_stddev': 10}, pair())['eligible']
    assert assess_condition({'rgb_noise': 10, 'delay': 0, 'patch_enabled': False},
                            pair(sensor_audit(noise=True)))['eligible']
    assert not assess_condition({'depth_noise': .1}, pair())['eligible']


@pytest.mark.parametrize('bad', [None, [], True, 'missing'])
def test_malformed_factor_mapping_stays_unknown(bad):
    assert not assess({'rgb_noise_stddev': 10}, pair({'exposure_verified': True, 'factors': bad}))['eligible']


@pytest.mark.parametrize('bad', [None, '', 'unrecognized', 'not_run_clean_failed'])
def test_missing_or_unknown_outcome_is_never_eligible(bad):
    p = pair(sensor_audit(noise=True)); p['perturbed']['outcome'] = bad
    assert not assess({'rgb_noise_stddev': 10}, p)['eligible']


@pytest.mark.parametrize('change', [
    {'worker_input_exposure': None},
    {'worker_input_exposure': {'schema': 'bridge_available_frames', 'status': 'complete'}},
])
def test_applied_released_frames_without_consumed_proof_are_not_network_exposure(change):
    p = pair(sensor_audit(noise=True)); p['perturbed'].update(change)
    assert not assess({'rgb_noise_stddev': 10}, p)['eligible']


@pytest.mark.parametrize('changes', [
    {'status': 'unavailable_no_same_run_worker_frames'}, {'configuration_checked': False},
    {'population': 'released bridge frames'}, {'observed_unique_consumed_frames': True},
    {'observed_unique_consumed_frames': 3},
    {'factor_counts': {'noise': {'active': 0, 'inactive': 1, 'unknown': 1}}},
    {'factor_counts': {'noise': {'active': True, 'inactive': 1, 'unknown': 0}}},
    {'evidence_sources': {'direct_worker_metadata': 0, 'same_run_exact_frame_join': 0, 'unavailable': 2}},
])
def test_consumed_audit_requires_correct_scope_checked_configuration_and_consistent_counts(changes):
    p = pair(sensor_audit(noise=True)); p['perturbed']['worker_input_exposure'].update(changes)
    assert not assess({'rgb_noise_stddev': 10}, p)['eligible']


def test_partial_consumed_observations_can_verify_at_least_one_update_without_claiming_complete_state():
    p = pair(sensor_audit(delay=True))
    audit = p['perturbed']['worker_input_exposure']
    audit['factor_counts']['delay'] = {'active': 1, 'inactive': 0, 'unknown': 1}
    audit['evidence_sources'] = {'direct_worker_metadata': 0, 'same_run_exact_frame_join': 1, 'unavailable': 1}
    result = assess({'delay_s': .3}, p)
    assert result['eligible']
    evidence = result['factor_evidence']['delay']['worker_input']
    assert evidence['active_frames'] == 1 and evidence['unknown_frames'] == 1
    assert evidence['exact_consumption_or_delivery_latency_verified'] is False


def test_real_consumed_auditor_contract_and_condition_check_feed_canonical_eligibility():
    from consumed_input_audit import audit_consumed_inputs
    packet = {'schema_version': 1, 'sequence': 7, 'sim_stamp': 2., 'disturbance_revision': 1,
              'rgb_noise_rmse': 3., 'sensor_disturbance': {'revision': 1, 'rgb_noise_stddev': 10.,
                  'fixed_delay_s': 0., 'noise_schedule': None, 'delay_schedule': None, 'schedule_elapsed_s': 2.}}
    trace = [{'sim_time_s': 2.5, 'worker': {'run_id': '/synthetic/run', 'sequence': 7,
                                          'sim_stamp': 2., 'consumed_input': packet}}]
    p = pair(sensor_audit(noise=True))
    p['perturbed']['worker_input_exposure'] = audit_consumed_inputs(
        trace, run_id='/synthetic/run', condition={'rgb_noise': 10., 'delay': 0.})
    assert assess({'rgb_noise_stddev': 10.}, p)['eligible']
    # An unchecked or wrong requested magnitude is not interchangeable evidence.
    for condition in (None, {'rgb_noise': 20., 'delay': 0.}):
        p['perturbed']['worker_input_exposure'] = audit_consumed_inputs(
            trace, run_id='/synthetic/run', condition=condition)
        assert not assess({'rgb_noise_stddev': 10.}, p)['eligible']
