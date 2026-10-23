"""Synthetic matching and exposure regressions; no model/simulator calls."""
import copy

import pytest

from corridor_space import VERSION, reference_action
from timing_contrasts import assess, match_actions


def actions(factor='delay'):
    first = reference_action(VERSION)
    key, schedule = {'delay': ('delay_s', 'delay_schedule'),
                     'rgb_noise': ('rgb_noise_stddev', 'noise_schedule')}[factor]
    first[key] = .3 if factor == 'delay' else 16.
    first[schedule] = {'start_s': 5., 'duration_s': 10., 'period_s': 0.}
    second = copy.deepcopy(first)
    second[schedule]['start_s'] = 45.
    return first, second


def pair(factor='delay', outcome='collision'):
    return {'clean': {'outcome': 'route_completed', 'executed': True},
            'perturbed': {'outcome': outcome, 'executed': True,
                          'attack_exposure': {'exposure_verified': True, 'factors': {
                              factor: {'requested': True, 'exposure_verified': True,
                                       'active_frames': 12, 'status': 'exposed'}}},
                          'worker_input_exposure': {
                              'schema': 'observed_consumed_input_v1', 'status': 'complete',
                              'configuration_checked': True,
                              'population': 'observed unique worker-consumed source-frame updates',
                              'observed_unique_consumed_frames': 12,
                              'observed_released_frame_identities': 12,
                              'factor_counts': {name: {'active': 12 if name == factor else 0,
                                                      'inactive': 0 if name == factor else 12, 'unknown': 0}
                                                for name in ('noise', 'delay')},
                              'evidence_sources': {'direct_worker_metadata': 12,
                                                   'same_run_exact_frame_join': 0, 'unavailable': 0},
                              'issues': {}, 'limitations': ['Synthetic fixture.']}}}


@pytest.mark.parametrize('factor,audit_key', [('delay', 'delay'), ('rgb_noise', 'noise')])
def test_one_start_only_matches_with_verified_outcomes_without_claiming_cause(factor, audit_key):
    left, right = actions(factor)
    first, second = pair(audit_key), pair(audit_key, 'route_completed')
    originals = copy.deepcopy((left, right, first, second))
    result = assess(left, first, right, second)
    assert result['matched'] and result['eligible'] and result['completed_vs_failed']
    assert result['factor'] == factor and result['single_finite_window']
    assert not result['causal_validation'] and not result['encounter_overlap_verified']
    assert not result['equal_delivered_exposure_established']
    assert (left, right, first, second) == originals


def test_pulp_35_to_80_seconds_is_not_a_same_duration_timing_contrast():
    left, right = actions()
    left['delay_schedule'] = {'start_s': 2., 'duration_s': 35., 'period_s': 0.}
    right['delay_schedule'] = {'start_s': 40., 'duration_s': 80., 'period_s': 0.}
    result = match_actions(left, right)
    assert not result['matched'] and result['reason'] == 'pulse_duration_or_period_changed'


@pytest.mark.parametrize('field,value', [
    ('delay_s', .4), ('layout_seed', 777), ('light_intensity', 1500.),
    ('shape', 'zigzag'), ('difficulty', 'hard'), ('patch_enabled', True),
    ('obstacles', [{'route_s': 8., 'lateral_m': 1.4}]),
])
def test_other_configuration_change_is_not_timing_isolation(field, value):
    left, right = actions(); right[field] = value
    result = match_actions(left, right)
    assert not result['matched'] and result['reason'] == 'other_action_fields_changed'


@pytest.mark.parametrize('field,value', [('duration_s', 11.), ('period_s', 20.)])
def test_duration_and_period_must_match(field, value):
    left, right = actions(); right['delay_schedule'][field] = value
    assert not match_actions(left, right)['matched']


def test_open_ended_window_is_not_equal_duration_after_start_changes():
    left, right = actions()
    for action in (left, right):action['delay_schedule']['duration_s'] = 0.
    result = match_actions(left, right)
    assert result['reason'] == 'open_ended_window_changes_exposure_duration'


def test_periodic_match_only_establishes_same_configured_pulse_not_total_dose():
    left, right = actions()
    for action in (left, right):action['delay_schedule']['period_s'] = 20.
    result = match_actions(left, right)
    assert result['matched'] and not result['single_finite_window']
    assert not result['equal_delivered_exposure_established']


def test_only_one_factor_schedule_may_change_even_if_other_factor_is_inactive():
    left, right = actions(); right['noise_schedule']['start_s'] = 3.
    assert match_actions(left, right)['reason'] == 'requires_exactly_one_changed_sensor_schedule'


def test_unchanged_cofactor_needs_its_own_exposure_measurement():
    left, right = actions()
    for action in (left, right):action['rgb_noise_stddev'] = 8.
    first, second = pair(), pair()
    result = assess(left, first, right, second)
    assert result['matched'] and not result['eligible']
    for saved in (first, second):
        saved['perturbed']['attack_exposure']['factors']['noise'] = {
            'requested': True, 'exposure_verified': True, 'active_frames': 5, 'status': 'exposed'}
        saved['perturbed']['worker_input_exposure']['factor_counts']['noise'] = {
            'active': 12, 'inactive': 0, 'unknown': 0}
    assert assess(left, first, right, second)['eligible']


def test_sensor_umbrella_does_not_verify_unchanged_patch_coexposure():
    left, right = actions()
    for action in (left, right):action['patch_enabled'] = True
    result = assess(left, pair(), right, pair())
    assert result['matched'] and not result['eligible']


@pytest.mark.parametrize('role,change', [
    ('clean', {'outcome': 'collision'}), ('perturbed', {'outcome': 'infrastructure_error'}),
    ('perturbed', {'executed': False}), ('perturbed', {'attack_exposure': None}),
    ('perturbed', {'attack_exposure': {'exposure_verified': True}}),
    ('perturbed', {'worker_input_exposure': None}),
])
def test_both_pairs_require_clean_pass_and_actual_factor_exposure(role, change):
    left, right = actions(); bad = pair(); bad[role].update(change)
    assert not assess(left, bad, right, pair())['eligible']
    assert not assess(left, pair(), right, bad)['eligible']


def test_same_outcome_and_different_failure_types_are_descriptive_only():
    left, right = actions()
    result = assess(left, pair(), right, pair())
    assert result['eligible'] and not result['outcome_changed']
    result = assess(left, pair(), right, pair(outcome='timeout'))
    assert result['outcome_changed'] and not result['completed_vs_failed']


def test_no_change_inactive_change_legacy_and_invalid_schedule_do_not_match():
    left, right = actions()
    assert not match_actions(left, left)['matched']
    for action in (left, right):action['delay_s'] = 0.
    assert match_actions(left, right)['reason'] == 'changed_factor_is_not_active'
    assert not match_actions(reference_action(), right)['matched']
    left, right = actions(); right['delay_schedule']['start_s'] = float('nan')
    assert match_actions(left, right)['reason'] == 'invalid_sensor_schedule'
