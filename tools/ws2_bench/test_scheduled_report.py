"""Scheduled exposure and quota regressions using synthetic flights only."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import agent_campaign
import codex_provider
from agent_analysis import evidence_for_report, validate_analysis, write_analysis
from codex_provider import CodexSubscriptionProvider
from codex_quota import QuotaLowError, QuotaUnknownError
from corridor_space import VERSION, action_to_episode, experiment, mission_for, reference_action
from episode import fingerprint, resolved
from vulnerability_report import analyze


def entry(number=1, exposure=True, start=10., obstacle_seed=42, clean='route_completed'):
    action = reference_action(VERSION)
    action.update(rgb_noise_stddev=16., layout_seed=obstacle_seed,
                  noise_schedule={'start_s': start, 'duration_s': 2., 'period_s': 0.})
    condition = resolved(action_to_episode(action, 'mononav', 42, 'synthetic',
                                           mission_for('mononav')))['condition']
    observed = ({'status': 'exposed', 'exposure_verified': True,
                 'factors': {'noise': {'active_frames': 5}}} if exposure is True else
                {'status': exposure, 'exposure_verified': False,
                 'factors': {'noise': {'active_frames': 0}}})
    return {'decision': {'round': number, 'action': action, 'condition': condition,
                        'experiment': experiment(), 'hypothesis': 'A brief noise pulse may alter depth.',
                        'plan': {'expected_support': 'A changed prediction coincides with failure.',
                                 'expected_refutation': 'The matched prediction and motion are unchanged.'},
                        'hypothesis_reviews': [{'round': 1, 'status': 'unresolved'}]},
            'pairs': [{'planner': 'mononav',
                       'clean': {'outcome': clean, 'metrics': {'mission_duration_sim_s': 20.}},
                       'perturbed': {'outcome': 'collision',
                                     'metrics': {'mission_duration_sim_s': 12.},
                                     'attack_exposure': observed}}]}


def report(history, phase='complete'):
    return analyze(history, [], {'planners': ['mononav'], 'action_space': 'corridor',
                                  'mission': mission_for('mononav')}, phase)


def analysis(status='candidate'):
    return {'summary': 'Synthetic report fixture.',
            'findings': [{'claim': 'A perturbation-associated failure was observed.',
                          'status': status, 'evidence_rounds': ['round_01']}],
            'limitations': ['Synthetic data; no real flight conclusion.'],
            'next_tests': [], 'defense_candidates': []}


@pytest.mark.parametrize('exposure', ['not_exposed_before_termination', 'evidence_error'])
def test_unverified_scheduled_exposure_never_becomes_attack_effect(exposure):
    history = [entry(exposure=exposure)]
    original = copy.deepcopy(history)
    result = report(history)
    assert not result['findings']
    assert result['paired_comparisons'][0]['attack_effect_evaluable'] is False
    # The actual contact remains a collision; eligibility is separate from scoring.
    assert result['outcomes'] == {'collision': 1, 'route_completed': 1}
    assert result['evaluated_flights'] == 2
    assert result['flight_results'][1]['outcome'] == 'collision'
    assert result['paired_comparisons'][0]['perturbed']['outcome'] == 'collision'
    assert history == original
    evidence = evidence_for_report(result)
    with pytest.raises(ValueError, match='clean-pass / attacked-fail'):
        validate_analysis(analysis(), evidence)
    assert validate_analysis(analysis('inconclusive'), evidence)['findings'][0]['status'] == 'inconclusive'


def test_exposed_clean_success_attack_failure_remains_candidate():
    result = report([entry()])
    assert result['paired_comparisons'][0]['attack_effect_evaluable'] is True
    assert len(result['findings']) == 1
    assert result['findings'][0]['status'] == 'candidate_needs_repeat'
    assert result['findings'][0]['failures'] == 1
    assert validate_analysis(analysis(), evidence_for_report(result))['findings'][0]['status'] == 'candidate'


def test_exposure_cannot_turn_clean_failure_into_attack_evidence():
    result = report([entry(clean='collision')])
    assert result['outcomes'] == {'collision': 2}
    assert not result['findings']
    assert result['conditions_with_clean_failure'][0]['outcome'] == 'collision'
    assert result['paired_comparisons'][0]['attack_effect_evaluable'] is False


@pytest.mark.parametrize('missing', ['absent', None, {}, 'malformed'])
def test_scheduled_missing_exposure_fails_closed_but_legacy_still_works(missing):
    history = [entry()]
    attacked = history[0]['pairs'][0]['perturbed']
    if missing == 'absent':
        attacked.pop('attack_exposure')
    else:
        attacked['attack_exposure'] = missing
    actual = report(history)
    assert not actual['findings']
    assert actual['paired_comparisons'][0]['attack_effect_evaluable'] is False
    assert actual['outcomes']['collision'] == 1
    # Historical constant-disturbance runs did not capture scheduled telemetry.
    legacy = copy.deepcopy(history)
    legacy[0]['pairs'][0]['perturbed'].pop('attack_exposure', None)
    for name in ('noise_schedule', 'delay_schedule'):
        legacy[0]['decision']['condition'].pop(name)
    result = report(legacy)
    assert result['paired_comparisons'][0]['attack_effect_evaluable'] is True
    assert result['findings'][0]['status'] == 'candidate_needs_repeat'


def test_schedule_changes_condition_identity_but_not_shared_environment():
    history = [entry(1, start=10.), entry(2, start=12.), entry(3, start=10.)]
    evidence = evidence_for_report(report(history))
    design = evidence['experimental_design']
    assert len(design['distinct_shared_environments']) == 1
    assert design['distinct_shared_environments'][0]['round_ids'] == ['round_01', 'round_02', 'round_03']
    layout = design['distinct_shared_environments'][0]['shared_environment']
    assert 'noise_schedule' not in layout and 'delay_schedule' not in layout
    assert sorted(g['pair_count'] for g in design['condition_repetitions']) == [1, 2]
    # Changing a real shared setting is still a distinct environment.
    changed = entry(4, obstacle_seed=123)
    design = evidence_for_report(report([*history, changed]))['experimental_design']
    assert len(design['distinct_shared_environments']) == 2


def test_analysis_receives_prediction_plan_and_measured_exposure(tmp_path):
    history = [entry(exposure='not_exposed_before_termination')]
    actual = report(history)
    received = []
    class Provider:
        last_call_id = 'synthetic-only'
        def metadata(self):
            return {'provider': 'synthetic_report_test'}
        def complete(self, purpose, system, payload, schema, validate):
            received.append((purpose, system, payload))
            return validate(analysis('inconclusive'))
    result = write_analysis(tmp_path, actual, Provider())
    assert result['status'] == 'complete'
    assert len(received) == 1
    purpose, system, evidence = received[0]
    assert purpose == 'analysis'
    first = evidence['rounds'][0]
    for key in ('plan', 'hypothesis', 'hypothesis_reviews'):
        assert first[key] == history[0]['decision'][key]
    assert first['attacked']['attack_exposure'] == history[0]['pairs'][0]['perturbed']['attack_exposure']
    assert first['attack_executed'] and not first['attack_effect_evaluable']
    assert 'not verified' in system and 'not measured facts' in system


@pytest.mark.parametrize('error', [QuotaLowError, QuotaUnknownError])
def test_quota_wait_neither_selects_nor_generates_final_report_and_can_resume(tmp_path, monkeypatch, error):
    monkeypatch.setattr(agent_campaign, 'RUNTIME', tmp_path)
    calls = []; flights = []
    class Provider:
        chooses_action = True
        last_call_id = 'synthetic-only'
        blocked = True
        def metadata(self):
            return {'provider': 'synthetic_quota_test'}
        def check_auth(self):
            return {'loggedIn': True}
        def check_quota(self):
            if self.blocked:
                raise error('Synthetic quota wait')
            return {'status': 'ok'}
        def complete(self, purpose, system, payload, schema, validate):
            calls.append(purpose)
            if purpose == 'analysis':
                value = analysis('observation')
                value['findings'][0]['claim'] = 'Both synthetic baseline flights completed.'
                return validate(value)
            return validate({'action': reference_action(), 'experiment': experiment(),
                             'reason': 'Synthetic quota-resume test.', 'hypothesis': 'Both controls should finish.'})
    def fly(config, folder, **kwargs):
        flights.append(config)
        folder.mkdir(parents=True)
        result = {'outcome': 'route_completed', 'metrics': {}, 'result_dir': str(folder),
                  'configuration_hash': fingerprint(config)}
        (folder / 'result.json').write_text(json.dumps(result))
        return result
    monkeypatch.setattr(agent_campaign, 'run_episode', fly)
    provider = Provider(); root = tmp_path / 'quota-campaign'
    kwargs = dict(provider=provider, action_space='corridor', clean_validation_runs=0,
                  clean_failure_policy='record', retries=0, pause_seconds=0, llm_report=True)
    agent_campaign.run_adaptive_campaign(root, 'agent_search', 2, **kwargs)
    assert not flights and not calls
    assert json.loads((root / 'presentation.json').read_text())['phase'] == 'quota_wait'
    assert not (root / 'llm_analysis.json').exists()
    provider.blocked = False
    agent_campaign.run_adaptive_campaign(root, 'agent_search', 2, **kwargs)
    assert len(flights) == 2
    assert calls == ['corridor_selection', 'analysis']
    assert json.loads((root / 'presentation.json').read_text())['phase'] == 'complete'


def test_unknown_after_response_preserves_answer_but_blocks_next_request_until_reset(tmp_path, monkeypatch):
    for name in ('OPENAI_API_KEY', 'CODEX_API_KEY', 'OPENAI_BASE_URL',
                 'AZURE_OPENAI_API_KEY', 'CODEX_OSS_BASE_URL'):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setattr(codex_provider.shutil, 'which', lambda _: '/fake/codex')
    quota = {'status': 'ok'}; calls = []
    def ensure(*args, **kwargs):
        if quota['status'] != 'ok':
            raise QuotaUnknownError('Synthetic unavailable quota')
        return quota.copy()
    def run(argv, **kwargs):
        calls.append(argv)
        Path(argv[argv.index('--output-last-message') + 1]).write_text('{"saved":true}')
        quota['status'] = 'unknown'
        return SimpleNamespace(returncode=0, stdout='{"type":"turn.completed"}', stderr='')
    monkeypatch.setattr(codex_provider, 'ensure_quota', ensure)
    monkeypatch.setattr(codex_provider, 'check_quota', lambda *a, **k: quota.copy())
    monkeypatch.setattr(codex_provider.subprocess, 'run', run)
    provider = CodexSubscriptionProvider(tmp_path / 'calls')
    assert provider.complete('test', 'system', {}, {}, lambda v: v) == {'saved': True}
    with pytest.raises(QuotaUnknownError):
        provider.complete('test', 'system', {}, {}, lambda v: v)
    assert len(calls) == 1
    quota['status'] = 'ok'
    assert provider.complete('test', 'system', {}, {}, lambda v: v) == {'saved': True}
    assert len(calls) == 2
    audits = [json.loads(path.read_text()) for path in (tmp_path / 'calls').glob('*.json')]
    assert sorted(a['status'] for a in audits) == ['quota_wait', 'validated', 'validated']
