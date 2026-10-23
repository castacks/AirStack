"""No flights/model calls: v2 orchestration through real validators/resolvers."""
import copy
import hashlib
import json

import pytest

import agent_exploration_v2 as planning
import planning_contract as contract
from agent_exploration import AgentStop
from corridor_space import CorridorPolicy
from episode import fingerprint
from operator_control import UserStop
from test_agent_exploration import Provider, history_with_images
from test_encounter_evidence import geometry, condition, trace, attach_frame
from test_planning_contract import proposal as contract_proposal, step


def proposal():
    value = contract_proposal()
    value['action']['rgb_noise_stddev'] = 24.
    return value


def recorded_history(root):
    history = history_with_images(root)
    for role, result in history[0]['pairs'][0].items():
        from pathlib import Path
        folder = Path(result['result_dir'])
        c = condition(noise=16. if role == 'perturbed' else 0., delay=0.)
        scenario = {'planner': 'pulp_dronet', 'condition': c}
        rows = trace(c)
        attach_frame(rows, c, 3., 3., sequence=3)
        attach_frame(rows, c, 8., 8., sequence=8)
        result.update(configuration_hash=fingerprint(scenario), scene_placement_id=geometry()['placement_id'],
                      diagnostics={'window': 'last 5 simulation seconds', 'sample_count': len(rows)},
                      attack_exposure={'exposure_verified': role == 'perturbed'})
        for name, value in [('scenario.json', scenario), ('result.json', result),
                            ('corridor_geometry.json', geometry()), ('diagnostic_trace.json', rows)]:
            (folder/name).write_text(json.dumps(value))
    return history


def inspection(*requests):
    value = proposal()
    value.update(operation='inspect', action=None, experiment=None, evidence_requests=list(requests))
    value['plan'].update(prediction=None, next_budget_intent=None,
                         allocation={'discover_pairs': 1, 'confirm_pairs': 0, 'diagnose_pairs': 0},
                         steps=[step(intent='diagnose', operation='inspect'), step()])
    return value


def request(evidence_id='r001_perturbed', view='telemetry', stamp=None):
    return {'evidence_id': evidence_id, 'view': view, 'sim_time_s': stamp}


def policy(root, values):
    p = CorridorPolicy('agent_search', 42, Provider(values))
    p.campaign_context = {'campaign_dir': str(root/'campaign'), 'target_planner': 'pulp_dronet'}
    return p


def test_two_mixed_inspections_keep_images_telemetry_full_history_and_plan(tmp_path, monkeypatch):
    monkeypatch.setattr(planning, 'RUNTIME', tmp_path)
    history = recorded_history(tmp_path)
    p = policy(tmp_path, [inspection(request('r001_clean', 'inference'), request(stamp=8.)),
                          inspection(request(view='inference')), proposal()])
    original_validator = contract.validate_response; histories = []
    def validate(raw, target, original_history, *args):
        histories.append(original_history)
        return original_validator(raw, target, original_history, *args)
    monkeypatch.setattr(contract, 'validate_response', validate)
    decision = planning.choose_next(p, history, 2)
    assert len(p.provider.calls) == 3
    assert all(value is history for value in histories)
    assert [len(call['images']) for call in p.provider.calls] == [0, 1, 2]
    last = p.provider.calls[-1]['payload']
    assert last['inspection_requests_remaining'] == 0
    assert last['inspection_results'][0]['requests'][1]['status'] == 'available'
    assert last['inspection_results'][0]['requests'][1]['source']['trace_sha256']
    assert all(entry['status'] == 'available' for entry in last['sampled_encounters'])
    assert decision['planning_version'] == contract.VERSION
    assert decision['plan']['next_budget_intent'] == 'discover'
    assert decision['planning_attempt']['attempt'] == 'attempt_0001'
    assert len(decision['evidence_inspections']) == 2
    for call in p.provider.calls:
        metadata = planning._image_audit(call['images'])
        assert planning.request_bytes(call['payload'], contract.response_schema('pulp_dronet'), metadata)+planning.TRANSPORT_RESERVE_BYTES <= 200000
    folder = tmp_path/'campaign/agent_evidence/round_02/attempt_0001'
    assert len(list(folder.glob('context_*.json'))) == 3
    assert len(list(folder.glob('proposal_*.json'))) == 3
    assert (folder/'inspection_00/evidence_manifest.json').exists()


def test_unknown_or_path_evidence_id_rejected_before_inspection(tmp_path, monkeypatch):
    monkeypatch.setattr(planning, 'RUNTIME', tmp_path)
    history = recorded_history(tmp_path)
    p = policy(tmp_path, [inspection(request('/etc/passwd', 'inference'))])
    with pytest.raises(ValueError, match='executed history run'):
        planning.choose_next(p, history, 2)
    assert len(p.provider.calls) == 1
    assert not list((tmp_path/'campaign').rglob('evidence_manifest.json'))


def test_operator_stop_then_resume_preserves_immutable_attempts(tmp_path, monkeypatch):
    monkeypatch.setattr(planning, 'RUNTIME', tmp_path)
    history = recorded_history(tmp_path)
    p = policy(tmp_path, [inspection(request())])
    def stop_after_call():
        if p.provider.calls: raise UserStop('operator stopped after inspection')
    p.operator_checkpoint = stop_after_call
    with pytest.raises(UserStop): planning.choose_next(p, history, 2)
    first = tmp_path/'campaign/agent_evidence/round_02/attempt_0001/proposal_00.json'
    digest = hashlib.sha256(first.read_bytes()).hexdigest()
    assert len(p.provider.calls) == 1
    resumed = policy(tmp_path, [proposal()])
    decision = planning.choose_next(resumed, history, 2)
    assert decision['planning_attempt']['attempt'] == 'attempt_0002'
    assert decision['planning_attempt']['prior_attempts_preserved'] == ['attempt_0001']
    assert hashlib.sha256(first.read_bytes()).hexdigest() == digest
    assert resumed.provider.calls[0]['images'] == []
    assert resumed.provider.calls[0]['payload']['inspection_results'] == []


def test_explicit_stop_reuses_campaign_exception_and_preserves_reason(tmp_path, monkeypatch):
    monkeypatch.setattr(planning, 'RUNTIME', tmp_path)
    value = proposal(); value.update(operation='stop', action=None, experiment=None)
    value['plan'].update(prediction=None, next_budget_intent=None, steps=[],
                         allocation={key: 0 for key in contract.ALLOCATION_KEYS})
    p = policy(tmp_path, [value])
    with pytest.raises(AgentStop, match='Measure and then repeat'):
        planning.choose_next(p, [], 2)
    assert json.loads((tmp_path/'campaign/agent_stop.json').read_text())['operation'] == 'stop'


def test_total_request_bound_reduces_details_then_rejects_required_core(tmp_path, monkeypatch):
    monkeypatch.setattr(planning, 'RUNTIME', tmp_path)
    first = recorded_history(tmp_path)
    history = []
    for index in range(4):
        entry = copy.deepcopy(first[0]); entry['decision']['round'] = index+1; history.append(entry)
    p = policy(tmp_path, []); schema = contract.response_schema('pulp_dronet')
    encounters = planning.recent_encounters(history, tmp_path)
    full = planning.bounded_context(p, history, 2, [], encounters, [], 2, schema)
    size = planning.request_bytes(full, schema, [])+planning.TRANSPORT_RESERVE_BYTES
    bounded = planning.bounded_context(p, history, 2, [], encounters, [], 2, schema, byte_budget=size-10)
    assert planning.request_bytes(bounded, schema, [])+planning.TRANSPORT_RESERVE_BYTES <= size-10
    assert len(bounded['request_retention']['attempted_detail_levels']) > 1
    assert [r['action'] for r in bounded['history']] == [r['decision']['action'] for r in history]
    p.campaign_context['oversized_required_context'] = 'x'*210000
    with pytest.raises(ValueError, match='after bounded detail reduction'):
        planning.choose_next(p, history, 2)
    assert p.provider.calls == []
    assert list((tmp_path/'campaign').rglob('context_limit.json'))


def test_encounters_require_identity_and_propagate_unavailable_analysis(tmp_path, monkeypatch):
    from pathlib import Path
    monkeypatch.setattr(planning, 'RUNTIME', tmp_path)
    history = recorded_history(tmp_path)
    assert all(v['status'] == 'available' for v in planning.recent_encounters(history, tmp_path))
    attacked = history[0]['pairs'][0]['perturbed']; folder = Path(attacked['result_dir'])
    (folder/'diagnostic_trace.json').write_text('[]')
    values = planning.recent_encounters(history, tmp_path)
    assert values[-1]['status'] == 'unavailable' and values[-1]['reason'] == 'unavailable_no_gt'
    (folder/'scenario.json').write_text('{"tampered":true}')
    values = planning.recent_encounters(history, tmp_path)
    assert values[-1]['status'] == 'unavailable' and values[-1]['reason'] == 'scenario_hash_mismatch'


def test_image_hash_change_after_inspection_stops_before_next_model_call(tmp_path, monkeypatch):
    monkeypatch.setattr(planning, 'RUNTIME', tmp_path)
    history = recorded_history(tmp_path)
    p = policy(tmp_path, [inspection(request(view='inference')), proposal()])
    def checkpoint():
        if list((tmp_path/'campaign').rglob('evidence_manifest.json')):
            (tmp_path/'perturbed/frame.jpg').write_bytes(b'changed after verification')
    p.operator_checkpoint = checkpoint
    with pytest.raises(ValueError, match='image changed'):
        planning.choose_next(p, history, 2)
    assert len(p.provider.calls) == 1


def test_actual_codex_provider_uses_v2_schema_and_subscription_transport_mocked_only(tmp_path, monkeypatch):
    from pathlib import Path
    from types import SimpleNamespace
    import codex_provider
    monkeypatch.setattr(planning, 'RUNTIME', tmp_path)
    monkeypatch.setattr(codex_provider.shutil, 'which', lambda _: '/mock/codex')
    monkeypatch.setattr(codex_provider, 'subscription_environment', lambda: {})
    for key in ('ensure_quota', 'check_quota'):
        monkeypatch.setattr(codex_provider, key, lambda *a, **k: {'status': 'ok', 'remaining_percent': 50.})
    provider = codex_provider.CodexSubscriptionProvider(tmp_path/'calls')
    monkeypatch.setattr(provider, 'check_auth', lambda: {'loggedIn': True})
    assert provider.supports_experiment_planning and provider.supports_images
    calls = []
    def cli(argv, **kwargs):
        data = json.loads(kwargs['input'])['data']
        assert data['planning_version'] == contract.VERSION and data['history'] == []
        schema = json.loads(Path(argv[argv.index('--output-schema')+1]).read_text())
        assert schema['properties']['plan']['properties']['version']['enum'] == [contract.VERSION]
        Path(argv[argv.index('--output-last-message')+1]).write_text(json.dumps(proposal()))
        calls.append(argv)
        return SimpleNamespace(returncode=0, stdout='{"type":"turn.completed","usage":{"input_tokens":1}}', stderr='')
    monkeypatch.setattr(codex_provider.subprocess, 'run', cli)
    p = CorridorPolicy('agent_search', 42, provider)
    p.campaign_context = {'campaign_dir': str(tmp_path/'campaign'), 'target_planner': 'pulp_dronet'}
    decision = planning.choose_next(p, [], 2)
    assert len(calls) == 1
    assert decision['planning_version'] == contract.VERSION
    assert decision['plan']['prediction']['primary_outcome'] == 'collision'
    audit = json.loads(next((tmp_path/'calls').glob('corridor_planning_v2_*.json')).read_text())
    assert audit['status'] == 'validated' and audit['request']['input']['planning_version'] == contract.VERSION



def test_encounter_version_changes_details_only_and_preserves_all_history_core(tmp_path, monkeypatch):
    from encounter_context import VERSION as ENCOUNTER_VERSION
    monkeypatch.setattr(planning, 'RUNTIME', tmp_path)
    first = recorded_history(tmp_path); history = []
    for index in range(4):
        entry = copy.deepcopy(first[0]); entry['decision']['round'] = index+1; history.append(entry)
    schema = contract.response_schema('pulp_dronet'); p = policy(tmp_path, [])
    old_encounters = planning.recent_encounters(history, tmp_path)
    old = planning.bounded_context(p, history, 2, [], old_encounters, [], 2, schema)
    p.campaign_context['encounter_context_version'] = ENCOUNTER_VERSION
    new_encounters = planning.recent_encounters(history, tmp_path, encounter_context_version=ENCOUNTER_VERSION)
    new = planning.bounded_context(p, history, 2, [], new_encounters, [], 2, schema)
    assert old['request_retention']['attempted_detail_levels'][0] == {
        'recent_diagnostic_rounds': 4, 'encounter_rounds': 4, 'samples_per_requested_telemetry_window': 20}
    assert new['request_retention']['attempted_detail_levels'][0] == {
        'recent_diagnostic_rounds': 2, 'encounter_rounds': 2, 'samples_per_requested_telemetry_window': 20}
    assert 'inspection_contract_notes' not in old and 'must be null' in new['inspection_contract_notes']
    for before, after in zip(old['history'], new['history']):
        for key in ('round', 'action', 'experiment', 'pairs'):
            if key == 'pairs':
                for bp, ap in zip(before['pairs'], after['pairs']):
                    assert set(bp) == set(ap)
                    for role in ('clean', 'perturbed'):
                        assert bp[role]['evidence_id'] == ap[role]['evidence_id']
                        assert bp[role]['outcome'] == ap[role]['outcome']
                        assert bp[role]['metrics'] == ap[role]['metrics']
            else:
                assert before.get(key) == after.get(key)
    assert all('summary' not in value and value['details_omitted_from_prompt'] for value in new['sampled_encounters'][:4])
    assert all(value['summary']['schema'] == ENCOUNTER_VERSION for value in new['sampled_encounters'][4:])
    assert all(value['summary']['retention']['all_entity_phase_clock_rows_retained'] for value in new['sampled_encounters'][4:])
    with pytest.raises(ValueError, match='Unsupported encounter_context_version'):
        planning.recent_encounters(history, tmp_path, encounter_context_version='unknown')
