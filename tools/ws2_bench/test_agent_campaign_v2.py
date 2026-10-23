"""New-protocol integration: real orchestration/audits, synthetic I/O only."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import agent_campaign
import agent_exploration_v2
import codex_provider
import factor_exposure
import planning_contract
from agent_analysis import evidence_for_report, validate_analysis
from campaign_versions import CURRENT
from corridor_space import VERSION, action_to_episode, experiment, mission_for, reference_action
from episode import fingerprint, resolved
from test_factor_exposure import pair as measured_pair, sensor_audit
from test_planning_contract import proposal, step
from vulnerability_report import analyze


def test_v2_campaign_consumed_exposure_repeat_report_and_exact_resume(tmp_path, monkeypatch):
    """Four simulated results, two provider decisions and one cached analysis."""
    import audit_corridor_campaign
    import episode
    monkeypatch.setattr(agent_campaign, 'RUNTIME', tmp_path)
    monkeypatch.setattr(agent_exploration_v2, 'RUNTIME', tmp_path)
    monkeypatch.setattr(episode, 'RUNTIME', tmp_path)
    monkeypatch.setattr(codex_provider.shutil, 'which', lambda _: '/mock/codex')
    monkeypatch.setattr(codex_provider, 'subscription_environment', lambda: {})
    for key in ('ensure_quota', 'check_quota'):
        monkeypatch.setattr(codex_provider, key, lambda *a, **k: {'status': 'ok', 'remaining_percent': 50.})
    provider = codex_provider.CodexSubscriptionProvider(tmp_path/'calls')
    monkeypatch.setattr(provider, 'check_auth', lambda: {'loggedIn': True})
    calls = []; flights = []; originals = {}

    def cli(argv, **kwargs):
        data = json.loads(kwargs['input'])['data']
        calls.append(copy.deepcopy(data))
        if 'remaining_pairs' in data:
            assert data['planning_version'] == planning_contract.VERSION
            assert data['evaluation_version'] == factor_exposure.VERSION
            schema = json.loads(Path(argv[argv.index('--output-schema')+1]).read_text())
            assert schema['properties']['plan']['properties']['version']['enum'] == [planning_contract.VERSION]
            output = proposal()
            if data['remaining_pairs'] == 1:
                saved = data['history'][0]
                assert saved['pairs'][0]['factor_exposure']['eligible'] is True
                assert saved['pairs'][0]['perturbed']['worker_input_exposure']['factor_counts']['noise']['active'] == 1
                assert data['prior_prediction_checks'][0]['eligible'] is True
                output['experiment'] = experiment('repeat', 1)
                output['plan'].update(next_budget_intent='confirm',
                    allocation={'discover_pairs': 0, 'confirm_pairs': 1, 'diagnose_pairs': 0},
                    steps=[step('repeat', 'confirm')])
        else:
            assert len(data['rounds']) == 2
            assert data['evaluation_version'] == factor_exposure.VERSION
            assert data['context_retention']['version']
            assert all(row['factor_exposure']['eligible'] for row in data['rounds'])
            output = {'summary': 'Synthetic matched failures were repeated.',
                      'findings': [{'claim': 'Synthetic noisy flights collided after observed consumed noise.',
                                    'status': 'candidate', 'evidence_rounds': ['round_01', 'round_02']}],
                      'limitations': ['Synthetic fixtures; no real flight conclusion or causal claim.'],
                      'next_tests': [], 'defense_candidates': []}
        Path(argv[argv.index('--output-last-message')+1]).write_text(json.dumps(output))
        return SimpleNamespace(returncode=0, stdout='{"type":"turn.completed","usage":{"input_tokens":1}}', stderr='')

    monkeypatch.setattr(codex_provider.subprocess, 'run', cli)
    monkeypatch.setattr(audit_corridor_campaign, 'scheduled_effects',
                        lambda folder, c: sensor_audit(noise=bool(c['rgb_noise']), delay=bool(c['delay'])))

    def fly(config, folder, **kwargs):
        folder.mkdir(); flights.append(copy.deepcopy(config)); c = config['condition']
        noise = c['rgb_noise']; delay = c['delay']
        disturbance = {'revision': 1, 'rgb_noise_stddev': noise, 'fixed_delay_s': delay,
                       'noise_schedule': c.get('noise_schedule'), 'delay_schedule': c.get('delay_schedule'),
                       'schedule_elapsed_s': 2.}
        consumed = {'schema_version': 1, 'sequence': 3, 'sim_stamp': 2., 'disturbance_revision': 1,
                    'sensor_disturbance': disturbance, 'rgb_noise_rmse': 12. if noise else 0.}
        rows = [{'sim_time_s': 2.3, 'camera_sequence': 3, 'latest_input_stamp': 2.,
                 'latest_input_disturbance': disturbance, 'latest_input_rgb_noise_rmse': consumed['rgb_noise_rmse'],
                 'worker': {'run_id': str(folder), 'sequence': 3, 'sim_stamp': 2., 'consumed_input': consumed}}]
        result = {'configuration_hash': fingerprint(config), 'result_dir': str(folder),
                  'outcome': 'collision' if noise else 'route_completed',
                  'metrics': {'path_length_m': 10., 'mission_duration_sim_s': 60.}}
        for name, value in [('result.json', result), ('scenario.json', config), ('diagnostic_trace.json', rows)]:
            path = folder/name; path.write_text(json.dumps(value)); originals[path] = path.read_bytes()
        return result

    monkeypatch.setattr(agent_campaign, 'run_episode', fly)
    options = dict(provider=provider, planner='pulp_dronet', retries=0, clean_validation_runs=0,
                   action_space='corridor', clean_failure_policy='record', pause_seconds=0, llm_report=True)
    root = tmp_path/'campaign'
    history = agent_campaign.run_adaptive_campaign(root, 'agent_search', 4, **options)
    assert len(flights) == 4 and len(calls) == 3
    config = json.loads((root/'config.json').read_text())
    assert {key: config[key] for key in CURRENT} == CURRENT
    assert history[1]['decision']['experiment'] == experiment('repeat', 1)
    assert history[1]['decision']['action'] == history[0]['decision']['action']
    assert all(row['pairs'][0]['factor_exposure']['eligible'] for row in history)
    assert all(row['pairs'][0]['factor_exposure']['causal_validation'] is False for row in history)
    report = json.loads((root/'vulnerability_report.json').read_text())
    assert report['outcomes'] == {'collision': 2, 'route_completed': 2}
    assert report['findings'][0]['status'] == 'reproduced_condition'
    predictions = json.loads((root/'hypothesis_predictions.json').read_text())
    assert all(row['eligible'] and row['multiclass_brier_score'] == 0. for row in predictions)
    assert all(path.read_bytes() == data for path, data in originals.items())
    assert len(list(root.rglob('worker_input_exposure.json'))) == 4
    before = (root/'history.json').read_bytes()
    resumed = agent_campaign.run_adaptive_campaign(root, 'agent_search', 4, **options)
    assert resumed == history and (root/'history.json').read_bytes() == before
    assert len(flights) == 4 and len(calls) == 3


@pytest.mark.parametrize('attack', ['patch', 'all_off'])
def test_v2_unknown_patch_and_all_off_never_become_attack_effects_in_prompts(attack):
    action = reference_action(VERSION)
    target = 'kim' if attack == 'patch' else 'pulp_dronet'
    if attack == 'patch':
        action.update(patch_enabled=True, patch_size_m=.3, obstacles=[dict(route_s=10., lateral_m=1.4, width_m=.5,
            depth_m=.5, height_m=2., yaw_deg=0.)],
            patch_mount=dict(obstacle_index=0, face='front', height_m=1.))
    condition = resolved(action_to_episode(action, target, 42, 'synthetic', mission_for(target)))['condition']
    pair = measured_pair(sensor_audit()); pair['planner'] = target
    for role in ('clean', 'perturbed'): pair[role]['metrics'] = {'mission_duration_sim_s': 20.}
    pair['factor_exposure'] = factor_exposure.assess(action, pair)
    history = [{'decision': {'round': 1, 'action': action, 'condition': condition,
                             'experiment': experiment(), 'plan': proposal()['plan'],
                             'planning_version': planning_contract.VERSION}, 'pairs': [pair]}]
    original = copy.deepcopy(history)
    report = analyze(history, [], {'planners': [target], 'action_space': 'corridor', **CURRENT}, 'complete')
    assert report['outcomes'] == {'collision': 1, 'route_completed': 1}
    assert report['findings'] == []
    assert not report['paired_comparisons'][0]['attack_effect_evaluable']
    assert not planning_contract.prediction_audit(history)[0]['eligible']
    evidence = evidence_for_report(report)
    verdict = evidence['rounds'][0]['factor_exposure']
    assert verdict['eligible'] is False
    if attack == 'patch':
        assert verdict['factor_evidence']['patch']['reason'] == 'patch_visibility_not_verified'
    else:
        assert verdict['reason'] == 'all_off_baseline'
    wrong = {'summary': 'Synthetic fixture.', 'findings': [{'claim': 'Attack caused failure.',
        'status': 'candidate', 'evidence_rounds': ['round_01']}],
        'limitations': ['Synthetic fixture.'], 'next_tests': [], 'defense_candidates': []}
    with pytest.raises(ValueError, match='clean-pass / attacked-fail'):
        validate_analysis(wrong, evidence)
    assert history == original
