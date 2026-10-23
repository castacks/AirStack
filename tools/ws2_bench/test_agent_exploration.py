"""Evidence inspection must remain bounded and never silently launch a test."""
import copy
import io
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from PIL import Image

import agent_exploration as planning
from corridor_space import CorridorPolicy, VERSION, reference_action, action_to_episode, experiment
from visual_evidence import digest


def proposal(operation='test', **updates):
    action = reference_action(VERSION); action['rgb_noise_stddev'] = 24.
    value = {'operation': operation, 'hypothesis': 'Noise may change obstacle-facing steering.',
             'reason': 'Inspect the observed behavior, then test the noise hypothesis.',
             'action': action if operation == 'test' else None,
             'experiment': experiment() if operation == 'test' else None,
             'evidence_requests': [], 'reviews': [],
             'plan': {'supports_if': 'Clean completes and noisy flight collides.',
                      'refuted_if': 'Both complete with similar progress.',
                      'expected_attack_outcomes': ['collision'],
                      'allocation': {'explore_pairs': 1, 'confirm_pairs': 1, 'diagnose_pairs': 0},
                      'steps': [{'purpose': 'explore', 'description': 'Test the stronger noise condition.'},
                                {'purpose': 'repeat', 'description': 'Confirm any observed failure.'}]}}
    value.update(updates)
    return value


def history_with_images(root):
    action = reference_action(VERSION); action['rgb_noise_stddev'] = 16.
    pair = {}
    for role in ('clean', 'perturbed'):
        folder = root/role; folder.mkdir()
        buffer = io.BytesIO(); Image.new('RGB', (32, 16), 'red' if role == 'perturbed' else 'blue').save(buffer, format='JPEG')
        data = buffer.getvalue(); (folder/'frame.jpg').write_bytes(data)
        result = {'result_dir': str(folder), 'outcome': 'collision' if role == 'perturbed' else 'route_completed'}
        (folder/'result.json').write_text(json.dumps(result))
        (folder/'visual_evidence.json').write_text(json.dumps({
            'run_id': str(folder), 'target': 'pulp_dronet',
            'frames': [{'file': 'frame.jpg', 'sha256': digest(data), 'capture_sim_s': 10.,
                        'model_output': {'steering': .2, 'collision_probability': .9}}]}))
        pair[role] = result
    condition = action_to_episode(action, 'pulp_dronet', 42, 'test', {})['condition']
    return [{'decision': {'round': 1, 'action': action, 'condition': condition,
                          'experiment': experiment(), 'hypothesis': 'Noise affects steering.'}, 'pairs': [pair]}]


class Provider:
    supports_images = True
    supports_experiment_planning = True
    last_call_id = None

    def __init__(self, values):
        self.values = values; self.calls = []

    def complete(self, purpose, system, payload, schema, validate, **kwargs):
        self.calls.append({'purpose': purpose, 'system': system, 'payload': copy.deepcopy(payload),
                           'images': kwargs.get('images', [])})
        self.last_call_id = f'mock_{len(self.calls)}'
        return validate(copy.deepcopy(self.values[len(self.calls)-1]))


def test_two_inspections_keep_both_images_and_plan_metadata_without_flights(tmp_path, monkeypatch):
    monkeypatch.setattr(planning, 'RUNTIME', tmp_path)
    history = history_with_images(tmp_path)
    inspect_clean = proposal('inspect', evidence_requests=[
        {'evidence_id': 'r001_clean', 'view': 'inference', 'sim_time_s': None}])
    inspect_attack = proposal('inspect', evidence_requests=[
        {'evidence_id': 'r001_perturbed', 'view': 'inference', 'sim_time_s': 9.5}])
    provider = Provider([inspect_clean, inspect_attack, proposal()])
    policy = CorridorPolicy('agent_search', 42, provider)
    policy.campaign_context = {'campaign_dir': str(tmp_path/'campaign'), 'target_planner': 'pulp_dronet'}
    decision = policy.choose_next(history, 2)
    assert len(provider.calls) == 3
    assert provider.calls[0]['images'] == []
    assert len(provider.calls[1]['images']) == 1
    assert len(provider.calls[2]['images']) == 2
    assert provider.calls[2]['payload']['inspection_requests_remaining'] == 0
    assert provider.calls[2]['payload']['remaining_pairs'] == 2
    assert decision['round'] == 2 and decision['llm_call_id'] == 'mock_3'
    assert decision['plan']['allocation']['confirm_pairs'] == 1
    assert decision['planning_version'] == planning.VERSION
    assert len(decision['evidence_inspections']) == 2
    assert all(e['requests'][0]['status'] == 'available' for e in decision['evidence_inspections'])
    audit = tmp_path/'campaign/agent_evidence/round_02'
    assert len(list(audit.glob('proposal_*.json'))) == 3


def test_unknown_evidence_id_is_reported_without_opening_model_supplied_path(tmp_path, monkeypatch):
    monkeypatch.setattr(planning, 'RUNTIME', tmp_path)
    history = history_with_images(tmp_path)
    provider = Provider([proposal('inspect', evidence_requests=[
        {'evidence_id': '/etc/passwd', 'view': 'inference', 'sim_time_s': None}]), proposal()])
    policy = CorridorPolicy('agent_search', 42, provider)
    policy.campaign_context = {'campaign_dir': str(tmp_path/'campaign'), 'target_planner': 'pulp_dronet'}
    result = planning.choose_next(policy, history, 2)
    assert provider.calls[1]['images'] == []
    assert 'Unknown recorded evidence' in result['evidence_inspections'][0]['unavailable']


def test_flight_budget_and_next_purpose_allocation_are_enforced():
    with pytest.raises(ValueError, match='remaining flight budget'):
        planning.validate_response(proposal(), 'pulp_dronet', [], 1)
    value = proposal(); value['plan']['allocation'] = {'explore_pairs': 0, 'confirm_pairs': 2, 'diagnose_pairs': 0}
    with pytest.raises(ValueError, match='allocated to its experiment purpose'):
        planning.validate_response(value, 'pulp_dronet', [], 2)
    value = proposal(); value['plan']['allocation']['explore_pairs'] = True
    with pytest.raises(ValueError, match='remaining flight budget'):
        planning.validate_response(value, 'pulp_dronet', [], 2)


def test_bounded_inspection_and_hypothesis_evidence_validation():
    request = {'evidence_id': 'r001_clean', 'view': 'inference', 'sim_time_s': None}
    with pytest.raises(ValueError, match='inspection allowance'):
        planning.validate_response(proposal('inspect', evidence_requests=[request]), 'pulp_dronet', [], 2, allow_inspect=False)
    with pytest.raises(ValueError, match='finite simulation seconds'):
        planning.validate_response(proposal('inspect', evidence_requests=[{**request, 'sim_time_s': float('nan')}]), 'pulp_dronet', [], 2)
    value = proposal(); value['reviews'] = [{'source_round': 99, 'assessment': 'supported', 'evidence': 'Unrecorded run.'}]
    with pytest.raises(ValueError, match='executed round'):
        planning.validate_response(value, 'pulp_dronet', [], 2)


def test_prediction_audit_distinguishes_prediction_from_causal_validation():
    history = []
    for number, (clean, attack) in enumerate([
            ('route_completed', 'collision'), ('route_completed', 'route_completed'),
            ('collision', 'not_run_clean_failed'), ('route_completed', 'infrastructure_error')], 1):
        history.append({'decision': {'round': number, 'plan': proposal()['plan']},
                        'pairs': [{'clean': {'outcome': clean}, 'perturbed': {'outcome': attack}}]})
    audit = planning.prediction_audit(history)
    assert [v['outcome_prediction_matched'] for v in audit] == [True, False, None, None]
    assert [v['eligible'] for v in audit] == [True, True, False, False]
    assert all('does not validate' in v['limitation'] for v in audit)


def test_explicit_stop_saves_reason_without_returning_an_action(tmp_path, monkeypatch):
    monkeypatch.setattr(planning, 'RUNTIME', tmp_path)
    provider = Provider([proposal('stop')])
    policy = CorridorPolicy('agent_search', 42, provider)
    policy.campaign_context = {'campaign_dir': str(tmp_path/'campaign'), 'target_planner': 'pulp_dronet'}
    with pytest.raises(planning.AgentStop):
        planning.choose_next(policy, [], 2)
    assert len(provider.calls) == 1
    assert json.loads((tmp_path/'campaign/agent_stop.json').read_text())['operation'] == 'stop'


def test_real_codex_provider_planning_campaign_resumes_without_calls_or_flights(tmp_path, monkeypatch):
    """Exercise the actual provider class/CLI parsing; only transport/flight are fakes."""
    import agent_campaign
    import codex_provider
    import campaign_versions
    from episode import fingerprint
    monkeypatch.setattr(planning, 'RUNTIME', tmp_path)
    monkeypatch.setattr(agent_campaign, 'RUNTIME', tmp_path)
    monkeypatch.setattr(codex_provider.shutil, 'which', lambda _: '/mock/codex')
    monkeypatch.setattr(codex_provider, 'subscription_environment', lambda: {})
    for name in ('ensure_quota', 'check_quota'):
        monkeypatch.setattr(codex_provider, name, lambda *a, **k: {'status': 'ok', 'remaining_percent': 50.})
    provider = codex_provider.CodexSubscriptionProvider(tmp_path/'calls')
    monkeypatch.setattr(provider, 'check_auth', lambda: {'loggedIn': True})
    assert provider.supports_experiment_planning and provider.supports_images
    calls = []; flights = []
    def cli(argv, **kwargs):
        data = json.loads(kwargs['input'])['data']
        assert data['remaining_pairs'] == 1 and data['evidence_catalog'] == []
        output = proposal(); output['plan']['allocation']['confirm_pairs'] = 0
        Path(argv[argv.index('--output-last-message')+1]).write_text(json.dumps(output))
        calls.append(argv)
        return SimpleNamespace(returncode=0, stdout='{"type":"turn.completed","usage":{"input_tokens":1}}', stderr='')
    monkeypatch.setattr(codex_provider.subprocess, 'run', cli)
    def fly(config, folder, **kwargs):
        folder.mkdir(); flights.append(config)
        result = {'configuration_hash': fingerprint(config), 'result_dir': str(folder),
                  'outcome': 'collision' if config['condition']['rgb_noise'] else 'route_completed',
                  'metrics': {'path_length_m': 10., 'mission_duration_sim_s': 60.}}
        (folder/'result.json').write_text(json.dumps(result))
        return result
    monkeypatch.setattr(agent_campaign, 'run_episode', fly)
    options = dict(provider=provider, planner='pulp_dronet', retries=0, clean_validation_runs=0,
                   action_space='corridor', clean_failure_policy='record', pause_seconds=0, llm_report=False)
    root = tmp_path/'campaign'
    # Create an archived, unversioned v1 fixture. Then restore real version
    # selection to verify that a historical resume never migrates its protocol.
    pin_versions = campaign_versions.apply
    monkeypatch.setattr(campaign_versions, 'apply', lambda config, previous=None, **kw: config)
    history = agent_campaign.run_adaptive_campaign(root, 'agent_search', 2, **options)
    assert len(calls) == 1 and len(flights) == 2
    assert history[0]['decision']['planning_version'] == planning.VERSION
    assert history[0]['decision']['plan']['allocation']['explore_pairs'] == 1
    assert json.loads((root/'hypothesis_predictions.json').read_text())[0]['outcome_prediction_matched'] is True
    monkeypatch.setattr(campaign_versions, 'apply', pin_versions)
    before = (root/'history.json').read_bytes()
    resumed = agent_campaign.run_adaptive_campaign(root, 'agent_search', 2, **options)
    assert resumed == history
    assert (root/'history.json').read_bytes() == before
    assert 'planning_version' not in json.loads((root/'config.json').read_text())
    assert len(calls) == 1 and len(flights) == 2


def test_operator_stop_after_inspection_prevents_another_model_call(tmp_path, monkeypatch):
    from operator_control import UserStop
    monkeypatch.setattr(planning, 'RUNTIME', tmp_path)
    history = history_with_images(tmp_path)
    provider = Provider([proposal('inspect', evidence_requests=[
        {'evidence_id': 'r001_clean', 'view': 'inference', 'sim_time_s': None}]), proposal()])
    policy = CorridorPolicy('agent_search', 42, provider)
    policy.campaign_context = {'campaign_dir': str(tmp_path/'campaign'), 'target_planner': 'pulp_dronet'}
    def checkpoint():
        if provider.calls:
            raise UserStop('operator stopped after inspection')
    policy.operator_checkpoint = checkpoint
    with pytest.raises(UserStop, match='stopped after inspection'):
        planning.choose_next(policy, history, 2)
    assert len(provider.calls) == 1
    assert (tmp_path/'campaign/agent_evidence/round_02/proposal_00.json').exists()
    assert not (tmp_path/'campaign/agent_evidence/round_02/proposal_01.json').exists()
