"""Resource accounting uses saved synthetic records only, never provider APIs."""
import copy
import json
from pathlib import Path
import sys

import pytest

from agent_resource_audit import audit_resources, main


def put(root, name, value):
    path = root/name; path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value)); return path


def record(call_id='selection_one', provider='codex_subscription', purpose='selection', **updates):
    value = {'call_id': call_id, 'provider': provider, 'requested_model': 'requested-alias',
             'request_sha256': 'same-request',
             'request': {'purpose': purpose, 'system': 'PRIVATE PROMPT NOT EXPORTED', 'images': [], 'input': {}},
             'status': 'validated', 'attempts': [attempt()]}
    value.update(updates); return value


def attempt(**updates):
    value = {'elapsed_s': 2., 'completed': True, 'resolved_model': None,
             'usage': {'input_tokens': 100, 'output_tokens': 20, 'cached_input_tokens': 80,
                       'reasoning_output_tokens': 5}, 'output': {'operation': 'test'}}
    value.update(updates); return value


def test_codex_repairs_copies_and_real_recovery_costs_count_once_per_call_id(tmp_path):
    original = record(attempts=[attempt(), attempt(elapsed_s=3.)], validation_error='invalid proposal')
    first = put(tmp_path, 'one/llm_calls/a.json', original)
    duplicate = put(tmp_path, 'copied/llm_calls/a.json', original)
    recovery = copy.deepcopy(original); recovery.update(call_id='selection_recovery', attempts=[attempt(elapsed_s=4.)])
    put(tmp_path, 'recovery/llm_calls/b.json', recovery)
    # A report wrapper may embed a full provider record; it is not another call.
    put(tmp_path, 'llm_analysis.json', {'provider_record': original})
    before = first.read_bytes(); result = audit_resources(tmp_path)
    assert result['provider_requests'] == 2 and result['duplicate_call_copies'] == 1
    assert result['recorded_retry_attempts'] == 1
    assert result['totals']['recorded_attempts'] == 3
    assert result['totals']['elapsed_s']['known_sum'] == 9.
    assert result['totals']['tokens']['input_tokens_total']['known_sum'] == 300
    assert result['totals']['tokens']['total_tokens']['known_sum'] == 360  # cache/reasoning are already included
    assert result['totals']['outcomes'] == {'accepted': 2, 'rejected_before_retry': 1}
    assert all(a['resolved_model_from_event'] is None for c in result['calls'] for a in c['attempts'])
    assert first.read_bytes() == duplicate.read_bytes() == before
    assert 'PRIVATE PROMPT' not in json.dumps(result)


def test_claude_cache_components_are_added_once_and_model_alias_is_not_resolution(tmp_path):
    usage = {'input_tokens': 2, 'cache_read_input_tokens': 100, 'cache_creation_input_tokens': 30,
             'output_tokens': 10, 'output_tokens_details': {'thinking_tokens': 3},
             'iterations': [{'input_tokens': 99999}], 'cache_creation': {'ephemeral_1h_input_tokens': 30}}
    a = attempt(usage=usage, model_usage={'claude-reported-model': {'inputTokens': 99999}}, reported_cost_usd=.01)
    put(tmp_path, 'llm_calls/claude.json', record(provider='claude_subscription', attempts=[a]))
    result = audit_resources(tmp_path); saved = result['calls'][0]['attempts'][0]
    assert saved['tokens']['input_tokens_total'] == 132 and saved['tokens']['total_tokens'] == 142
    assert saved['tokens']['reasoning_output_tokens'] == 3
    assert saved['model_ids_from_model_usage'] == ['claude-reported-model']
    assert saved['resolved_model_from_event'] is None
    assert result['calls'][0]['requested_model'] == 'requested-alias'
    assert result['totals']['reported_cost_usd_estimate']['known_sum'] == .01


def test_failed_reports_timeout_and_missing_usage_remain_visible_unknown(tmp_path):
    put(tmp_path, 'llm_calls/error.json', record('analysis_error', purpose='analysis', status='provider_error',
        attempts=[attempt(elapsed_s=27., completed=False, usage=None, output=None)]))
    put(tmp_path, 'llm_calls/timeout.json', record('analysis_timeout', provider='claude_subscription',
        purpose='analysis', status='timeout', attempts=[]))
    result = audit_resources(tmp_path)
    assert result['provider_requests'] == 2 and result['totals']['recorded_attempts'] == 1
    assert result['by_category']['final_report']['provider_requests'] == 2
    assert result['totals']['elapsed_s']['known_sum'] == 27.
    assert result['totals']['tokens']['total_tokens'] == {
        'known_sum': None, 'recorded_values': 0, 'missing_values': 1, 'complete': False}
    assert result['calls_with_possible_unrecorded_attempts'] == 1
    assert result['audit_coverage']['saved_attempt_ledger_complete'] is False
    assert result['audit_coverage']['elapsed_complete_for_saved_requests'] is False
    assert result['call_statuses'] == {'provider_error': 1, 'timeout': 1}


def test_conflicting_same_id_excluded_not_silently_chosen_or_doublecounted(tmp_path):
    first = record(); other = copy.deepcopy(first); other['attempts'][0]['elapsed_s'] = 8.
    put(tmp_path, 'one/llm_calls/a.json', first); put(tmp_path, 'two/llm_calls/a.json', other)
    result = audit_resources(tmp_path)
    assert result['provider_requests'] == 0 and result['excluded_conflicting_request_ids'] == 1
    assert len(result['conflicting_calls'][0]['sources']) == 2
    assert result['totals']['elapsed_s']['known_sum'] is None


def test_inspection_image_transmissions_and_missing_manifest_are_separate(tmp_path):
    first = record('planning_inspect', purpose='corridor_planning_v2',
                   attempts=[attempt(output={'operation': 'inspect'}), attempt(output={'operation': 'inspect'})])
    first['request']['images'] = [{'path': '/DO_NOT_OPEN/secret.png', 'sha256': 'a'*64}]
    put(tmp_path, 'llm_calls/images.json', first)
    second = record('legacy', provider='claude_subscription'); second['request'].pop('images')
    put(tmp_path, 'llm_calls/legacy.json', second)
    result = audit_resources(tmp_path)
    assert result['by_category']['inspection']['attached_image_transmissions']['known_sum'] == 2
    assert result['by_category']['selection']['attached_image_transmissions']['missing_values'] == 1
    assert result['calls'][1]['distinct_image_sha256'] == ['a'*64]
    assert '/DO_NOT_OPEN' not in json.dumps(result)


def test_episode_copies_deduplicated_and_sim_seconds_not_mixed_with_wall_seconds(tmp_path):
    result = {'result_dir': '/recorded/episode/one', 'configuration_hash': 'x', 'outcome': 'route_completed',
              'wall_duration_s': 80., 'metrics': {'mission_duration_sim_s': 30., 'planner_wall_duration_s': 60.}}
    put(tmp_path, 'round_01/pulp/clean/attempt_0/result.json', result)
    put(tmp_path, 'archived_copy/result.json', result)
    put(tmp_path, 'infrastructure/result.json', {'result_dir': '/recorded/episode/error',
        'outcome': 'infrastructure_error', 'wall_duration_s': 2.})
    audit = audit_resources(tmp_path); episodes = audit['episodes']
    assert episodes['recorded_attempts'] == 2
    assert episodes['episode_wall_duration_s']['known_sum'] == 82.
    assert episodes['mission_duration_sim_s']['known_sum'] == 30.
    assert episodes['mission_duration_sim_s']['missing_values'] == 1
    assert episodes['outcomes'] == {'infrastructure_error': 1, 'route_completed': 1}


def test_invalid_usage_and_unknown_provider_are_not_reported_as_zero(tmp_path):
    put(tmp_path, 'llm_calls/a.json', record(attempts=[attempt(usage={'input_tokens': True, 'output_tokens': -1})]))
    put(tmp_path, 'llm_calls/other.json', record('other', provider='unrecognized'))
    result = audit_resources(tmp_path)
    assert result['provider_requests'] == 1 and len(result['issues']) == 1
    assert result['totals']['tokens']['total_tokens']['known_sum'] is None
    assert result['calls'][0]['attempts'][0]['tokens']['missing_fields']


def test_old_claude_unlogged_repair_is_flagged_without_inventing_cost(tmp_path):
    a = attempt(usage={'input_tokens': 1, 'output_tokens': 2,
                      'cache_read_input_tokens': 0, 'cache_creation_input_tokens': 0})
    put(tmp_path, 'llm_calls/a.json', record(provider='claude_subscription', attempts=[a],
                                          validation_error='earlier envelope parse failure'))
    result = audit_resources(tmp_path)
    assert result['totals']['tokens']['total_tokens']['known_sum'] == 3
    assert result['totals']['tokens']['total_tokens']['complete'] is True  # only recorded entries
    assert result['audit_coverage']['token_totals_complete_for_saved_requests'] is False
    assert result['recorded_retry_attempts'] == 0  # no invented duration/tokens for missing attempt


def test_pending_requests_and_explicit_quota_wait_do_not_invent_transport_attempts(tmp_path):
    pending = record('pending', attempts=[]); pending.pop('status')
    put(tmp_path, 'llm_calls/pending.json', pending)
    put(tmp_path, 'llm_calls/quota.json', record('quota', status='quota_wait', attempts=[]))
    result = audit_resources(tmp_path)
    assert result['provider_requests'] == 2 and result['totals']['recorded_attempts'] == 0
    assert result['calls_with_possible_unrecorded_attempts'] == 1
    assert result['totals']['elapsed_s']['known_sum'] is None


def test_symlink_outside_root_is_not_read(tmp_path):
    outside = tmp_path/'outside.json'; outside.write_text('{"sensitive":"DO NOT EXPORT"}')
    root = tmp_path/'campaign'; (root/'llm_calls').mkdir(parents=True)
    (root/'llm_calls/a.json').symlink_to(outside)
    result = audit_resources(root)
    assert result['provider_requests'] == 0 and len(result['issues']) == 1
    assert 'DO NOT EXPORT' not in json.dumps(result)


def test_cli_output_is_explicit_new_derived_file_and_never_clobbers(tmp_path, monkeypatch):
    put(tmp_path, 'llm_calls/a.json', record())
    destination = tmp_path/'resources_derived.json'
    monkeypatch.setattr(sys, 'argv', ['audit', str(tmp_path), '--output', str(destination)])
    main(); assert json.loads(destination.read_text())['derived_only'] is True
    with pytest.raises(FileExistsError): main()
    monkeypatch.setattr(sys, 'argv', ['audit', str(tmp_path), '--output', str(tmp_path/'result.json')])
    with pytest.raises(SystemExit): main()
    assert not (tmp_path/'result.json').exists()
