"""Offline policy/budget checks; these tests never run a flight or provider."""
import copy

import pytest

from baseline_proposals_v2 import Prior, VERSION as PROPOSAL_VERSION, proposal_key
from comparison_protocol import (PHASE_A_PAIRS, PhaseComplete, choose_phase_a,
                                 failure_efficiency, phase_budget)
from corridor_space import VERSION, reference_action


def action(seed=1, noise=12., delay=0.):
    value = reference_action(VERSION)
    value.update(layout_seed=seed, rgb_noise_stddev=noise, delay_s=delay)
    return value


def pair(a, outcome='collision', *, eligible=True, clean='route_completed'):
    factors = {name: {'requested': a[field] > 0, 'exposure_verified': a[field] > 0,
                      'status': 'exposed' if a[field] > 0 else 'off',
                      'active_frames': 10 if a[field] > 0 else 0}
               for name, field in (('noise', 'rgb_noise_stddev'), ('delay', 'delay_s'))}
    consumed = {'schema': 'observed_consumed_input_v1', 'status': 'complete',
                'configuration_checked': True,
                'population': 'observed unique worker-consumed source-frame updates',
                'observed_unique_consumed_frames': 10,
                'factor_counts': {name: {'active': f['active_frames'],
                                        'inactive': 10-f['active_frames'], 'unknown': 0}
                                  for name, f in factors.items()},
                'evidence_sources': {'direct_worker_metadata': 10,
                                     'same_run_exact_frame_join': 0, 'unavailable': 0}}
    return {'clean': {'outcome': clean, 'executed': True,
                      'metrics': {'route_progress_percent': 100.}},
            'perturbed': {'outcome': outcome, 'executed': True,
                          'metrics': {'route_progress_percent': 80. if outcome == 'collision' else 100.},
                          'attack_exposure': {'factors': factors},
                          'worker_input_exposure': consumed if eligible else None}}


def row(number, a=None, outcome='collision', *, decision=None, **kwargs):
    a = a or action()
    decision = copy.deepcopy(decision) if decision is not None else {'action': copy.deepcopy(a)}
    decision['round'] = number
    measured = pair(decision['action'], outcome, **kwargs)
    for role in ('clean', 'perturbed'):
        measured[role].update(result_dir=f'/synthetic/pilot/round_{number}/{role}',
                              configuration_hash=('a' if role == 'clean' else 'b')*64)
    return {'decision': decision, 'pairs': [measured]}


def choose(policy, history, **kwargs):
    kwargs.setdefault('reserved_slots_used', 2*len(history))
    return choose_phase_a(policy, 73001, 'pulp_dronet', history, initial_action=action(), **kwargs)


def test_common_start_is_fresh_identical_and_inputs_are_preserved():
    initial = action(); old = copy.deepcopy(initial)
    r = choose('random', []); s = choose('search', [])
    assert r['action'] == s['action'] == initial
    assert r['experiment'] == {'kind': 'explore', 'source_round': 0, 'passing_round': 0, 'factor': 'none'}
    assert r['proposal_audit']['requires_fresh_pair'] is True
    r['action']['layout_seed'] = 999
    assert initial == old and choose('random', [])['action'] == initial


@pytest.mark.parametrize('repeat_outcome,clean,eligible', [
    ('collision', 'route_completed', True), ('route_completed', 'route_completed', True),
    ('collision', 'collision', True), ('infrastructure_error', 'route_completed', False)])
def test_exactly_one_confirmation_then_discovery_even_if_repeat_fails(repeat_outcome, clean, eligible):
    history = [row(1)]; old = copy.deepcopy(history)
    repeated = choose('random', history)
    assert repeated['action'] == history[0]['decision']['action']
    assert repeated['experiment']['kind'] == 'repeat' and repeated['experiment']['source_round'] == 1
    history.append(row(2, outcome=repeat_outcome, clean=clean, eligible=eligible, decision=repeated))
    discovery = choose('random', history)
    assert discovery['experiment']['kind'] == 'explore'
    assert proposal_key(discovery['action']) != proposal_key(action())
    assert history[0] == old[0]


@pytest.mark.parametrize('outcome,clean,eligible', [
    ('route_completed', 'route_completed', True), ('collision', 'collision', True),
    ('collision', 'route_completed', False), ('infrastructure_error', 'route_completed', True)])
def test_only_observed_eligible_failure_triggers_confirmation(outcome, clean, eligible):
    decision = choose('random', [row(1, outcome=outcome, clean=clean, eligible=eligible)])
    assert decision['experiment']['kind'] == 'explore'


def test_known_maximum_noise_failure_receives_normal_confirmation_credit():
    a = action(noise=32.)
    repeated = choose_phase_a('random', 73001, 'pulp_dronet', [row(1, a)], initial_action=a,
                              reserved_slots_used=2)
    assert repeated['experiment']['kind'] == 'repeat' and repeated['action'] == a


def test_first_search_discovery_really_attempts_local_then_returns_to_fixed_mixture():
    # Near-certain restart still cannot suppress the predeclared first local attempt.
    prior = Prior(restart_probability=.999999)
    history = [row(1, outcome='route_completed')]
    first = choose('search', history, prior=prior)
    assert first['proposal_audit']['require_local_attempt'] is True
    assert first['proposal_audit']['mutation_attempts'] > 0
    assert first['proposal_audit']['source'] == 'elite_mutation'
    history.append(row(2, outcome='route_completed', decision=first))
    second = choose('search', history, prior=prior)
    assert second['proposal_audit']['require_local_attempt'] is False
    assert second['proposal_audit']['source'] == 'global'
    assert second['proposal_audit']['fallback'] == 'fixed_probability_restart'


def test_missing_incumbent_does_not_use_up_first_local_attempt():
    history = [row(1, outcome='collision', eligible=False)]
    global_draw = choose('search', history)
    assert global_draw['proposal_audit']['fallback'] == 'no_eligible_incumbent'
    assert global_draw['proposal_audit']['mutation_attempts'] == 0
    # Guarantee a valid executed active setting in this synthetic later round.
    later = copy.deepcopy(global_draw); later['action'] = action(seed=2)
    history.append(row(2, outcome='route_completed', decision=later))
    local = choose('search', history)
    assert local['proposal_audit']['require_local_attempt'] is True
    assert local['proposal_audit']['mutation_attempts'] > 0


def test_unidentified_runs_are_not_search_elites_even_with_plausible_metrics():
    history = [row(1, outcome='route_completed')]
    history[0]['pairs'][0]['perturbed'].pop('configuration_hash')
    proposed = choose('search', history)
    assert proposed['proposal_audit']['fallback'] == 'no_eligible_incumbent'
    assert proposed['proposal_audit']['pilot_execution_evidence_exclusions'] == [
        {'round': 1, 'reason': 'missing_saved_execution_identity'}]


def test_confirmation_does_not_advance_random_discovery_stream():
    failure_history = [row(1)]
    repeated = choose('random', failure_history)
    failure_history.append(row(2, decision=repeated))
    after_repeat = choose('random', failure_history)
    after_pass = choose('random', [row(1, outcome='route_completed')])
    assert after_repeat['action'] == after_pass['action']
    assert after_repeat['proposal_audit']['proposal_index'] == 1


def test_budget_boundary_and_default_llm_are_not_overridden():
    history = [row(n, action(seed=n), outcome='route_completed') for n in range(1, 7)]
    with pytest.raises(PhaseComplete): choose('random', history)
    with pytest.raises(ValueError, match='random or search'): choose('astra', [])
    spec = phase_budget()
    assert spec['phases']['adaptive']['pairs'] == PHASE_A_PAIRS == 6
    assert spec['phases']['prospective_diagnosis']['pairs'] == 2
    assert spec['pilot_reserved_slots'] == 96 and spec['default_product_policy'] == 'free_llm'
    assert 'strong whole-flight noise' in spec['objective']


@pytest.mark.parametrize('used', [11, 12])
def test_charged_retry_slots_can_exhaust_budget_before_six_pairs(used):
    with pytest.raises(PhaseComplete):
        choose('random', [row(1)], reserved_slots_used=used)


def test_reserved_slots_cannot_omit_already_completed_pairs():
    with pytest.raises(ValueError, match='reserved phase-A slots'):
        choose('random', [row(1)], reserved_slots_used=1)


@pytest.mark.parametrize('mutation', ['unfinished', 'duplicate_round', 'wrong_start', 'legacy'])
def test_rejects_pending_foreign_or_ambiguous_history(mutation):
    h = [row(1)]
    if mutation == 'unfinished': h[0]['pairs'] = []
    if mutation == 'duplicate_round': h.append(row(1))
    if mutation == 'wrong_start': h[0]['decision']['action']['layout_seed'] = 9
    if mutation == 'legacy':
        for key in ('action_space_version', 'noise_schedule', 'delay_schedule'):
            h[0]['decision']['action'].pop(key)
    with pytest.raises(ValueError): choose('random', h)


def test_endpoint_uses_reserved_slots_and_requires_same_outcome_twice():
    h = [row(1), row(2, outcome='timeout'), row(3)]
    result = failure_efficiency(h, pair_slot_ends=[2, 6, 8], reserved_slots_used=8)
    assert result['primary']['event_slot'] == 8
    assert result['confirmed_settings'] == result['confirmed_descriptive_families'] == 1
    assert result['explicitly_executed_flights'] == 6
    assert result['reserved_slots_used'] == 8
    assert result['primary']['right_censored'] is False and result['causal_validation'] is False
    mixed = failure_efficiency(h[:2], pair_slot_ends=[2, 6], reserved_slots_used=12)
    assert mixed['confirmed_settings'] == 0
    assert mixed['mixed_failure_repeat_settings_without_same_outcome'] == 1
    assert mixed['primary']['right_censored'] is True and mixed['primary']['censor_slot'] == 12


def test_no_attribution_for_failed_control_missing_consumption_or_missed_window():
    h = [row(1), row(2, eligible=False), row(3, clean='collision')]
    h.append(row(4)); h[-1]['pairs'][0]['perturbed']['attack_exposure']['factors']['noise']['active_frames'] = 0
    result = failure_efficiency(h, pair_slot_ends=[2, 4, 6, 8], reserved_slots_used=12)
    assert result['confirmed_settings'] == 0 and result['primary']['right_censored'] is True
    assert result['pair_eligibility_reasons']['active_factor_exposure_unverified'] == 2
    assert result['pair_eligibility_reasons']['clean_control_failed'] == 1


def test_inactive_setting_fields_do_not_create_distinct_failures_but_seeds_remain():
    a = action(); b = copy.deepcopy(a)
    b['delay_schedule']['start_s'] = 400.
    b['patch_size_m'] = .7
    h = [row(1, a), row(2, b), row(3, action(seed=2)), row(4, action(seed=2))]
    result = failure_efficiency(h, pair_slot_ends=[2, 4, 6, 8], reserved_slots_used=8)
    assert result['confirmed_settings'] == 2
    assert result['confirmed_descriptive_families'] == 1
    assert result['primary']['event_slot'] == 4


def test_pending_endpoint_does_not_claim_censoring_or_zero_cost():
    result = failure_efficiency([row(1)], pair_slot_ends=[2], reserved_slots_used=2)
    assert result['primary']['pending'] is True
    assert result['primary']['event_slot'] is result['primary']['censor_slot'] is None
    assert result['primary']['right_censored'] is False


def test_singleton_remaining_slot_cannot_keep_endpoint_pending_after_phase_closes():
    result = failure_efficiency([row(1)], pair_slot_ends=[2], reserved_slots_used=11)
    assert result['primary']['right_censored'] is True and result['primary']['censor_slot'] == 12
    assert result['primary']['pending'] is False and result['unused_reserved_slots'] == 1


def test_terminal_agent_stop_records_actual_censoring_without_claiming_full_budget():
    result = failure_efficiency([row(1)], pair_slot_ends=[2], reserved_slots_used=2, terminal=True)
    assert result['primary']['pending'] is False and result['primary']['stopped_early'] is True
    assert result['primary']['right_censored'] is True and result['primary']['censor_slot'] == 2
    assert result['primary']['event_slot'] is None and result['unused_reserved_slots'] == 10
    assert result['budget_reserved_slots'] == 12 and result['explicitly_executed_flights'] == 2


@pytest.mark.parametrize('corruption', ['copied_pair', 'same_role_path', 'missing_executed',
                                       'missing_path', 'missing_config_hash', 'path_alias'])
def test_repeated_artifacts_or_unknown_execution_never_create_confirmation(corruption):
    h = [row(1), row(2)]
    runs = h[1]['pairs'][0]
    if corruption == 'copied_pair': h[1]['pairs'] = copy.deepcopy(h[0]['pairs'])
    if corruption == 'same_role_path': runs['perturbed']['result_dir'] = runs['clean']['result_dir']
    if corruption == 'missing_executed':
        for run in runs.values(): run.pop('executed')
    if corruption == 'missing_path': runs['perturbed'].pop('result_dir')
    if corruption == 'missing_config_hash': runs['perturbed'].pop('configuration_hash')
    if corruption == 'path_alias':
        runs['perturbed']['result_dir'] = '/synthetic/pilot/round_1/unused/../perturbed'
    result = failure_efficiency(h, pair_slot_ends=[2, 4], reserved_slots_used=12)
    assert result['confirmed_settings'] == 0 and result['primary']['right_censored'] is True
    assert result['execution_identity_checks'][1]['eligible'] is False


@pytest.mark.parametrize('ends,used', [([], 2), ([1], 2), ([4], 2), ([2], 13), ([True], 2)])
def test_rejects_missing_or_inconsistent_slot_ledger(ends, used):
    with pytest.raises(ValueError):
        failure_efficiency([row(1)], pair_slot_ends=ends, reserved_slots_used=used)


def test_diagnostic_results_cannot_be_folded_back_into_discovery_endpoint():
    h = [row(i) for i in range(1, 8)]
    with pytest.raises(ValueError, match='phase-A history only'):
        failure_efficiency(h, pair_slot_ends=list(range(2, 16, 2)), reserved_slots_used=12)
