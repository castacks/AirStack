"""Offline matched-policy contracts; provider doubles never invoke an LLM."""
import copy
import json
from dataclasses import asdict

import pytest

from baseline_proposals_v2 import Prior
from comparison_protocol import VERSION, PhaseComplete
from matched_policy import (DIAGNOSIS_VERSION, MatchedPolicy, audit_diagnosis_decision, diagnosis_outcomes,
                            select_source, validate_diagnosis)
from test_comparison_protocol import action, row


def phase_a():
    return [row(1), row(2)] + [row(i, action(seed=i), outcome='route_completed') for i in range(3, 7)]


def forecast(completes=True):
    values = {'route_completed': .8 if completes else .1,
              'collision': .1 if completes else .8,
              'insufficient_progress': .04, 'timeout': .03, 'planner_stopped': .03}
    primary = max(values, key=values.get)
    return dict(primary_outcome=primary, confidence=values[primary], probabilities=values)


def proposal(history=None):
    history = history or phase_a(); source = select_source(history)
    a = copy.deepcopy(history[source['source_round']-1]['decision']['action'])
    a['rgb_noise_stddev'] /= 2
    return dict(version=DIAGNOSIS_VERSION, source_round=source['source_round'],
                hypothesis='Reducing noise restores completion in this exact scene.',
                reason='Test a measured conditional noise magnitude contrast.',
                evidence_ids=source['evidence_ids'],
                intervention=dict(kind='reduce_magnitude', factor='rgb_noise'),
                action=a, prediction=forecast(), expected_relation='completion_recovers')


class Provider:
    supports_experiment_planning = True
    supports_images = True
    last_call_id = 'mock_call'

    def __init__(self): self.calls = []

    def complete(self, purpose, system, payload, schema, validate, **kwargs):
        self.calls.append(dict(purpose=purpose, system=system, payload=copy.deepcopy(payload), kwargs=kwargs))
        return validate(proposal())


def policy(name='random'):
    provider = Provider()
    protocol = dict(version=VERSION, target='pulp_dronet', seed=73001,
                    initial_action=action(), shared_prior=asdict(Prior()),
                    phase_budgets={'adaptive_pairs': 6, 'diagnosis_pairs': 2})
    result = MatchedPolicy(name, 73001, provider, protocol)
    result.campaign_context = {'target_planner': 'pulp_dronet', 'total_pairs': 8,
                               'study_objective': 'Development-only phrase must not reach pilot selector.'}
    return result, provider


def test_source_selection_prefers_confirmed_failure_then_earliest_observation():
    h = [row(1, action(seed=1)), row(2, action(seed=2)), row(3, action(seed=2))]
    selected = select_source(h)
    assert selected['source_round'] == 2 and selected['matching_failure_rounds'] == [2, 3]
    assert selected['confirmed_same_outcome'] is True
    h[2]['pairs'][0]['perturbed']['outcome'] = 'timeout'
    assert select_source(h)['source_round'] == 1
    h[0]['pairs'][0]['clean']['executed'] = False
    assert select_source(h)['source_round'] == 2


def test_copied_episode_does_not_make_confirmed_source():
    h = [row(1), row(2)]; h[1]['pairs'] = copy.deepcopy(h[0]['pairs'])
    assert select_source(h)['confirmed_same_outcome'] is False


@pytest.mark.parametrize('change', ['geometry', 'light', 'seed', 'cofactor', 'zero', 'increase',
                                   'missing_citation', 'unknown_citation', 'source', 'observed', 'criterion'])
def test_invalid_or_already_observed_prospective_claims_are_rejected(change):
    h = phase_a(); raw = proposal(h)
    if change == 'geometry': raw['action']['shape'] = 'curve'
    if change == 'light': raw['action']['light_intensity'] += 1
    if change == 'seed': raw['action']['layout_seed'] += 1
    if change == 'cofactor': raw['action']['delay_s'] = .1
    if change == 'zero': raw['action']['rgb_noise_stddev'] = 0.
    if change == 'increase': raw['action']['rgb_noise_stddev'] = 13.
    if change == 'missing_citation': raw['evidence_ids'] = ['r001_perturbed', 'r002_perturbed']
    if change == 'unknown_citation': raw['evidence_ids'].append('r099_clean')
    if change == 'source': raw['source_round'] = 2
    if change == 'observed': h[2]['decision']['action'] = copy.deepcopy(raw['action'])
    if change == 'criterion': raw['expected_relation'] = 'failure_persists'
    with pytest.raises(ValueError): validate_diagnosis(raw, h)


def test_removal_leaves_one_active_factor_and_never_changes_its_schedule():
    h = phase_a()
    for index in (0, 1):
        h[index] = row(index+1, action(delay=.2))
    raw = proposal(h)
    raw['intervention']['kind'] = 'remove_factor'; raw['action']['rgb_noise_stddev'] = 0.
    assert validate_diagnosis(raw, h)['action']['delay_s'] == .2
    raw['action']['delay_schedule']['start_s'] = 3.
    with pytest.raises(ValueError, match='another setting'): validate_diagnosis(raw, h)
    h = phase_a(); raw = proposal(h)
    raw['intervention']['kind'] = 'remove_factor'; raw['action']['rgb_noise_stddev'] = 0.
    with pytest.raises(ValueError, match='another active sensor'): validate_diagnosis(raw, h)


def test_timing_requires_positive_duration_and_exact_configured_dose():
    h = phase_a()
    for index in (0, 1):
        h[index]['decision']['action']['noise_schedule'] = {'start_s': 20., 'duration_s': 15., 'period_s': 0.}
    raw = proposal(h); raw['action'] = copy.deepcopy(h[0]['decision']['action'])
    raw['action']['noise_schedule']['start_s'] = 50.
    raw['intervention']['kind'] = 'shift_timing'
    assert validate_diagnosis(raw, h)['action']['noise_schedule']['start_s'] == 50.
    raw['action']['noise_schedule']['duration_s'] = 20.
    with pytest.raises(ValueError, match='unchanged duration'): validate_diagnosis(raw, h)


@pytest.mark.parametrize('name', ['random', 'search', 'agent_search'])
def test_every_policy_executes_same_initial_pair_without_calling_reporter(name):
    selected, provider = policy(name)
    initial = selected.choose_next([], 8)
    assert initial['action'] == action() and initial['comparison']['phase'] == 'adaptive'
    assert initial['proposal_audit']['requires_fresh_pair'] is True
    assert provider.calls == []


def test_common_reporter_prompt_and_evidence_are_identical_across_selectors():
    payloads = []; prompts = []
    for name in ('random', 'search', 'agent_search'):
        selected, provider = policy(name)
        h = phase_a()
        for item in h:
            item['decision'].update(policy=name, hypothesis='Selector-specific prose should be removed.')
        decision = selected.choose_next(h, 2)
        assert decision['diagnosis']['status'] == 'prospective'
        assert decision['diagnosis']['reporter_mode'] == 'common_bounded_text'
        call = provider.calls[0]; payloads.append(call['payload']); prompts.append(call['system'])
        assert call['kwargs'] == {}  # No silent difference in image attachment.
        assert all('hypothesis' not in r for r in call['payload']['history'])
    assert payloads[0] == payloads[1] == payloads[2]
    assert prompts[0] == prompts[1] == prompts[2]


def test_second_diagnostic_pair_repeats_frozen_intervention_without_rewriting_forecast():
    selected, provider = policy(); h = phase_a()
    first = selected.choose_next(h, 2)
    h.append(row(7, decision=first, outcome='route_completed'))
    second = selected.choose_next(h, 1)
    assert second['experiment'] == {'kind': 'repeat', 'source_round': 7, 'passing_round': 0, 'factor': 'none'}
    assert second['action'] == first['action']
    assert second['plan']['prediction'] == first['plan']['prediction']
    assert second['diagnosis']['proposal_sha256'] == first['diagnosis']['proposal_sha256']
    assert len(provider.calls) == 1


@pytest.mark.parametrize('outcomes,classification', [
    (('route_completed', 'route_completed'), 'supported'),
    (('collision', 'collision'), 'refuted'),
    (('route_completed', 'collision'), 'inconclusive')])
def test_only_consistent_eligible_outcome_repeats_support_or_refute(outcomes, classification):
    selected, _ = policy(); h = phase_a()
    first = selected.choose_next(h, 2); seventh = row(7, decision=first, outcome=outcomes[0])
    second = selected.choose_next(h+[seventh], 1); eighth = row(8, decision=second, outcome=outcomes[1])
    result = diagnosis_outcomes(first['diagnosis']['proposal'], h, [seventh, eighth])
    assert result['classification'] == classification
    assert result['causal_validation'] is False
    assert all(item['multiclass_brier_score'] is not None for item in result['observations'])


def test_missing_exposure_stays_inconclusive_and_does_not_start_repeat_queue():
    selected, provider = policy(); h = phase_a()
    first = selected.choose_next(h, 2); seventh = row(7, decision=first, eligible=False)
    second = selected.choose_next(h+[seventh], 1)
    assert second['diagnosis']['status'] == 'unavailable'
    assert second['experiment']['kind'] == 'explore'
    result = diagnosis_outcomes(first['diagnosis']['proposal'], h, [seventh])
    assert result['classification'] == 'inconclusive' and not result['observations'][0]['eligible']
    assert len(provider.calls) == 1


def test_no_source_has_predeclared_descriptive_fallback_and_no_diagnostic_model_call():
    selected, provider = policy()
    h = [row(i, action(seed=i), outcome='route_completed') for i in range(1, 7)]
    first = selected.choose_next(h, 2)
    second = selected.choose_next(h+[row(7, decision=first)], 1)
    assert first['diagnosis']['status'] == second['diagnosis']['status'] == 'unavailable'
    assert first['diagnosis']['excluded_from_phase_a_endpoint'] is True
    assert provider.calls == []
    assert first['proposal_audit']['proposal_index'] == 101
    assert second['proposal_audit']['proposal_index'] == 102


def test_changed_source_history_or_posthoc_forecast_cannot_receive_prospective_credit():
    selected, _ = policy(); h = phase_a()
    first = selected.choose_next(h, 2); seventh = row(7, decision=first, outcome='route_completed')
    changed = copy.deepcopy(h); changed[0]['pairs'][0]['clean']['metrics']['route_progress_percent'] = 99.
    with pytest.raises(ValueError, match='history changed'): selected.choose_next(changed+[seventh], 1)
    raw = copy.deepcopy(first['diagnosis']['proposal'])
    raw['prediction']['probabilities']['route_completed'] = .85
    raw['prediction']['probabilities']['collision'] = .05
    raw['prediction']['confidence'] = .85
    result = diagnosis_outcomes(raw, h, [seventh])
    assert result['observations'][0]['ineligible_reason'] == 'missing_or_changed_frozen_diagnosis'


def test_astra_selector_keeps_full_free_planning_and_restores_scoped_context(monkeypatch):
    import agent_exploration_v2
    selected, provider = policy('agent_search'); old = copy.deepcopy(selected.campaign_context)
    seen = {}
    def planner(wrapped, history, remaining):
        seen.update(context=copy.deepcopy(wrapped.campaign_context), remaining=remaining)
        assert wrapped.provider.original is provider
        return wrapped.decision(history, action(seed=20),
                                dict(kind='explore', source_round=0, passing_round=0, factor='none'),
                                'Mock free agent choice.', 'Mock hypothesis.')
    monkeypatch.setattr(agent_exploration_v2, 'choose_next', planner)
    result = selected.choose_next([row(1)], 7)
    assert seen['remaining'] == 5 and seen['context']['total_pairs'] == 6
    assert 'Development-only' not in seen['context']['study_objective']
    assert '35% whole mission' in seen['context']['shared_proposal_prior']['schedules']
    assert result['action']['layout_seed'] == 20
    assert selected.provider is provider and selected.campaign_context == old


def test_agent_stop_is_propagated_instead_of_spending_hidden_flights(monkeypatch):
    import agent_exploration_v2
    from agent_exploration import AgentStop
    selected, provider = policy('agent_search'); old = copy.deepcopy(selected.campaign_context)
    def stopped(*args): raise AgentStop('No justified next test.')
    monkeypatch.setattr(agent_exploration_v2, 'choose_next', stopped)
    with pytest.raises(AgentStop): selected.choose_next([row(1)], 7)
    assert selected.provider is provider and selected.campaign_context == old


def test_phase_and_reserved_budget_are_enforced():
    selected, _ = policy(); selected.reserved_slots_used = 15
    with pytest.raises(PhaseComplete): selected.choose_next(phase_a(), 2)
    with pytest.raises(PhaseComplete): selected.choose_next([], 0)


def test_derived_writer_tracks_pending_repeat_completion_and_preserves_inputs(tmp_path, monkeypatch):
    import episode
    monkeypatch.setattr(episode, 'RUNTIME', tmp_path)
    selected, _ = policy(); selected.campaign_context['campaign_dir'] = str(tmp_path)
    h = phase_a(); original = copy.deepcopy(h)
    assert selected.write_diagnosis_evaluation(h[:2])['status'] == 'phase_a_in_progress'
    stopped = selected.write_diagnosis_evaluation(h[:2], terminal=True)
    assert stopped['status'] == 'unavailable_early_stop' and stopped['terminal'] is True
    assert selected.write_diagnosis_evaluation(h)['status'] == 'awaiting_prospective_intervention'
    first = selected.choose_next(h, 2); seventh = row(7, decision=first, outcome='route_completed')
    pending = selected.write_diagnosis_evaluation(h+[seventh])
    assert pending['status'] == 'awaiting_intervention_repeat' and pending['classification'] == 'inconclusive'
    second = selected.choose_next(h+[seventh], 1); eighth = row(8, decision=second, outcome='route_completed')
    completed = selected.write_diagnosis_evaluation(h+[seventh, eighth], terminal=True)
    assert completed['status'] == 'complete' and completed['classification'] == 'supported'
    assert completed['intervention_delta']['relative_magnitude_reduction'] == .5
    assert completed['diagnostic_usefulness_established'] is False
    assert json.loads((tmp_path/'matched_diagnosis_evaluation.json').read_text()) == completed
    assert h == original


def test_derived_writer_handles_unavailable_and_ineligible_fallback(tmp_path, monkeypatch):
    import episode
    monkeypatch.setattr(episode, 'RUNTIME', tmp_path)
    selected, _ = policy(); selected.campaign_context['campaign_dir'] = str(tmp_path)
    passed = [row(i, action(seed=i), outcome='route_completed') for i in range(1, 7)]
    assert selected.write_diagnosis_evaluation(passed)['status'] == 'unavailable_no_eligible_source'
    fallback = selected.choose_next(passed, 2)
    assert selected.write_diagnosis_evaluation(passed+[row(7, decision=fallback)])['status'] == 'unavailable'
    h = phase_a(); first = selected.choose_next(h, 2); seventh = row(7, decision=first, eligible=False)
    assert selected.write_diagnosis_evaluation(h+[seventh])['status'] == 'first_intervention_ineligible'
    second = selected.choose_next(h+[seventh], 1); eighth = row(8, decision=second)
    result = selected.write_diagnosis_evaluation(h+[seventh, eighth], terminal=True)
    assert result['status'] == 'inconclusive_after_fallback' and result['classification'] == 'inconclusive'
    assert result['fallback_rounds'] == [8]


def test_auditor_checks_actual_reporter_output_and_requires_no_new_repeat_call():
    selected, _ = policy(); h = phase_a(); first = selected.choose_next(h, 2)
    call = {'call_id': first['llm_call_id'], 'status': 'validated',
            'request': {'purpose': 'matched_prospective_diagnosis'},
            'attempts': [{'output': copy.deepcopy(first['diagnosis']['proposal'])}]}
    assert audit_diagnosis_decision(first, h, call=call)
    seventh = row(7, decision=first, outcome='route_completed')
    second = selected.choose_next(h+[seventh], 1)
    assert audit_diagnosis_decision(second, h, first_decision=first)
    with pytest.raises(ValueError, match='not a new LLM choice'):
        audit_diagnosis_decision(second, h, call=call, first_decision=first)
    call['attempts'][0]['output']['reason'] = 'Edited after the model call.'
    with pytest.raises(ValueError, match='actual reporter output'):
        audit_diagnosis_decision(first, h, call=call)
    first['diagnosis']['proposal_sha256'] = '0'*64
    with pytest.raises(ValueError, match='hash differs'):
        audit_diagnosis_decision(first, h, call=call)


def test_tiny_contrasts_are_described_but_do_not_create_a_new_outcome_gate():
    selected, _ = policy(); h = phase_a(); raw = proposal(h)
    raw['action']['rgb_noise_stddev'] = 11.999
    raw['expected_relation'] = 'failure_persists'; raw['prediction'] = forecast(False)
    raw = validate_diagnosis(raw, h)
    first = selected._diagnostic_decision(h, raw, select_source(h)); seventh = row(7, decision=first)
    second = selected._diagnostic_decision(h+[seventh], raw, select_source(h), repeat=True)
    result = diagnosis_outcomes(raw, h, [seventh, row(8, decision=second)])
    assert result['classification'] == 'supported'
    assert result['intervention_delta']['less_than_one_percent_change'] is True
    assert result['diagnostic_usefulness_established'] is False
