"""Real matched orchestration with fake flights/provider, never simulator I/O."""
import copy
import json
from dataclasses import asdict
from pathlib import Path

import pytest

import agent_campaign
import agent_exploration_v2
import audit_corridor_campaign
import consumed_input_audit
import episode
from baseline_proposals_v2 import Prior
from comparison_protocol import VERSION
from corridor_space import experiment
from episode import fingerprint
from matched_policy import DIAGNOSIS_VERSION, audit_diagnosis_decision
from test_comparison_protocol import action, pair as proof_pair
from test_matched_policy import forecast


class Reporter:
    supports_experiment_planning = True
    supports_images = True
    chooses_action = True

    def __init__(self, root):
        self.root = root
        self.calls = []
        self.last_call_id = None

    def check_auth(self): return {'loggedIn': True}
    def metadata(self): return {'provider': 'offline_fixture', 'model': 'synthetic', 'reasoning_effort': 'none'}

    def complete(self, purpose, system, payload, schema, validate, **kwargs):
        assert purpose == 'matched_prospective_diagnosis'
        self.calls.append(copy.deepcopy(payload))
        source = payload['source']
        source_action = next(r['action'] for r in payload['history'] if r['round'] == source['source_round'])
        selected = copy.deepcopy(source_action)
        factor = 'rgb_noise' if selected['rgb_noise_stddev'] > 0 else 'delay'
        selected['rgb_noise_stddev' if factor == 'rgb_noise' else 'delay_s'] *= .49
        raw = dict(version=DIAGNOSIS_VERSION, source_round=source['source_round'],
                   hypothesis='A lower magnitude restores completion in this synthetic fixture.',
                   reason='Test a prospective single-factor contrast.', evidence_ids=source['evidence_ids'],
                   intervention=dict(kind='reduce_magnitude', factor=factor), action=selected,
                   prediction=forecast(), expected_relation='completion_recovers')
        result = validate(raw)
        self.last_call_id = 'synthetic_diagnosis'
        folder = self.root/'llm_calls'; folder.mkdir(exist_ok=True)
        (folder/(self.last_call_id+'.json')).write_text(json.dumps(dict(
            call_id=self.last_call_id, status='validated', request=dict(purpose=purpose),
            attempts=[dict(output=raw)])))
        return result


def harness(tmp_path, monkeypatch):
    for module in (agent_campaign, agent_exploration_v2, episode):
        monkeypatch.setattr(module, 'RUNTIME', tmp_path)
    flights = []; originals = {}

    def evidence(condition):
        return proof_pair(dict(rgb_noise_stddev=condition['rgb_noise'], delay_s=condition['delay']))['perturbed']

    # Sensor application and consumption are tested independently. This suite
    # tests preservation/execution/budget/evaluation using explicit known proof.
    monkeypatch.setattr(audit_corridor_campaign, 'scheduled_effects',
                        lambda folder, condition: evidence(condition)['attack_exposure'])

    def consumed(result, runtime):
        config = json.loads((Path(result['result_dir'])/'scenario.json').read_text())
        return evidence(config['condition'])['worker_input_exposure']
    monkeypatch.setattr(consumed_input_audit, 'record_episode_consumption', consumed)

    def fly(config, folder, **kwargs):
        folder.mkdir()
        role = folder.parent.name
        number = int(folder.parents[2].name.split('_')[1])
        active = config['condition']['rgb_noise'] > 0 or config['condition']['delay'] > 0
        outcome = ('collision' if number == 1 or (role == 'perturbed' and active and number <= 6)
                   else 'route_completed')
        result = dict(configuration_hash=fingerprint(config), result_dir=str(folder), outcome=outcome,
                      metrics=dict(route_progress_percent=80. if outcome == 'collision' else 100.,
                                   path_length_m=10., mission_duration_sim_s=60., minimum_clearance_m=.2),
                      cleanup_errors=[])
        flights.append(dict(round=number, role=role, config=copy.deepcopy(config)))
        for name, value in [('scenario.json', config), ('result.json', result)]:
            path = folder/name; path.write_text(json.dumps(value)); originals[path] = path.read_bytes()
        return result
    monkeypatch.setattr(agent_campaign, 'run_episode', fly)

    def selector(policy, history, remaining):
        # Provider planning itself has separate integration coverage. Here this
        # double exercises the free-Astra selector -> matched runner boundary.
        if len(history) == 2:
            selected = copy.deepcopy(history[1]['decision']['action'])
            operation = experiment('repeat', 2)
        else:
            selected = action(seed=100+len(history))
            operation = experiment()
        return policy.decision(history, selected, operation, 'Synthetic selection.', 'Synthetic hypothesis.')
    monkeypatch.setattr(agent_exploration_v2, 'choose_next', selector)
    return flights, originals


def options(root, name):
    provider = Reporter(root)
    protocol = dict(version=VERSION, target='pulp_dronet', seed=73001,
                    initial_action=action(), shared_prior=asdict(Prior()),
                    phase_budgets=dict(adaptive_pairs=6, diagnosis_pairs=2))
    return provider, dict(provider=provider, seed=73001, planner='pulp_dronet', retries=0,
                          clean_validation_runs=0, action_space='corridor', clean_failure_policy='record',
                          pause_seconds=0, llm_report=False, matched_protocol=protocol)


@pytest.mark.parametrize('name', ['random', 'search', 'agent_search'])
def test_matched_sixteen_slots_failed_clean_repeat_diagnosis_and_exact_resume(tmp_path, monkeypatch, name):
    flights, originals = harness(tmp_path, monkeypatch)
    root = tmp_path/'campaign'; provider, opts = options(root, name)
    history = agent_campaign.run_adaptive_campaign(root, name, 16, **opts)
    assert len(history) == 8 and len(flights) == 16
    assert history[0]['decision']['action'] == opts['matched_protocol']['initial_action']
    first = history[0]['pairs'][0]
    assert first['clean']['outcome'] == 'collision' and first['perturbed']['executed'] is True
    assert first['factor_exposure']['eligible'] is False
    assert all(pair[role]['executed'] is True for row in history for pair in row['pairs']
               for role in ('clean', 'perturbed'))
    assert len(provider.calls) == 1
    prospective, repeated = history[6]['decision'], history[7]['decision']
    assert prospective['diagnosis']['status'] == 'prospective'
    assert repeated['diagnosis']['status'] == 'repeat'
    assert repeated['action'] == prospective['action'] and repeated['experiment'] == experiment('repeat', 7)
    call = json.loads((root/'llm_calls/synthetic_diagnosis.json').read_text())
    assert audit_diagnosis_decision(prospective, history[:6], call=call)
    assert audit_diagnosis_decision(repeated, history[:6], first_decision=prospective)
    efficiency = json.loads((root/'comparison_efficiency.json').read_text())
    diagnosis = json.loads((root/'matched_diagnosis_evaluation.json').read_text())
    assert efficiency['primary']['pending'] is False
    assert efficiency['reserved_slots_used'] == 12
    assert diagnosis['terminal'] is True and diagnosis['status'] == 'complete'
    assert diagnosis['classification'] == 'supported' and diagnosis['causal_validation'] is False
    assert diagnosis['completed_diagnostic_pairs'] == 2
    assert all(path.read_bytes() == value for path, value in originals.items())
    assert all('executed' not in json.loads(path.read_text()) for path in originals if path.name == 'result.json')
    if name == 'search':
        assert any(row['decision'].get('proposal_audit', {}).get('mutation_attempts', 0) > 0
                   for row in history[:6])
    saved = (root/'history.json').read_bytes()
    resumed = agent_campaign.run_adaptive_campaign(root, name, 16, **opts)
    assert resumed == history and (root/'history.json').read_bytes() == saved
    assert len(flights) == 16 and len(provider.calls) == 1


def test_matched_free_agent_early_stop_is_terminal_with_unused_budget(tmp_path, monkeypatch):
    from agent_exploration import AgentStop
    flights, _ = harness(tmp_path, monkeypatch)
    def stop(*args): raise AgentStop('Synthetic stop after the common initial pair.')
    monkeypatch.setattr(agent_exploration_v2, 'choose_next', stop)
    root = tmp_path/'campaign'; provider, opts = options(root, 'agent_search')
    history = agent_campaign.run_adaptive_campaign(root, 'agent_search', 16, **opts)
    assert len(history) == 1 and len(flights) == 2 and provider.calls == []
    primary = json.loads((root/'comparison_efficiency.json').read_text())['primary']
    assert primary['pending'] is False and primary['stopped_early'] is True
    assert primary['event_slot'] is None and primary['censor_slot'] == 2
    diagnosis = json.loads((root/'matched_diagnosis_evaluation.json').read_text())
    assert diagnosis['terminal'] is True and diagnosis['status'] == 'unavailable_early_stop'
