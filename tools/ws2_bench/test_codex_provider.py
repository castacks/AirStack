import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import codex_provider
import codex_quota
from codex_provider import CodexSubscriptionProvider
from codex_quota import QuotaLowError, QuotaUnknownError, weekly_snapshot


def limits(used=90, reset=2000000000, weekly_slot='secondary'):
    return {'rateLimitsByLimitId': {'codex': {
        weekly_slot: {'usedPercent': used, 'windowDurationMins': 10080, 'resetsAt': reset}}}}


def test_weekly_limit_is_detected_by_window_not_slot():
    for slot in ('primary', 'secondary'):
        result = weekly_snapshot(limits(99, weekly_slot=slot), now=100)
        assert result['status'] == 'low'
        assert result['remaining_percent'] == 1
    response = limits(20)
    response['rateLimitsByLimitId']['codex']['primary'] = {
        'usedPercent': 99, 'windowDurationMins': 300, 'resetsAt': 2000000000}
    assert weekly_snapshot(response, now=100)['remaining_percent'] == 80


@pytest.mark.parametrize('response', [{}, {'rateLimits': {'limitId': 'other'}},
                                     limits(float('nan')), limits(101), limits(20, reset=50)])
def test_missing_or_invalid_weekly_limits_are_unknown(response):
    result = weekly_snapshot(response, now=100)
    assert result['status'] == 'unknown' and result['blocked']
    assert result['remaining_percent'] is None


def test_alert_three_times_and_deduplicate_with_unknown_poll(tmp_path):
    state = tmp_path / 'quota.json'; notifications = []
    def send(message):
        notifications.append(message)
        return True
    low = weekly_snapshot(limits(99), now=100)
    for _ in range(2):
        codex_quota.record_quota(low, state, notifier=send)
    codex_quota.record_quota(weekly_snapshot({}, now=100), state, notifier=send)
    codex_quota.record_quota(low, state, notifier=send)
    assert len(notifications) == 3
    assert all(f'{i}/3' in message for i, message in enumerate(notifications, 1))
    assert json.loads(state.read_text())['alerts_sent'] == 3
    # A manual reset is observable even when the service retains the end time.
    codex_quota.record_quota(weekly_snapshot(limits(0), now=100), state, notifier=send)
    codex_quota.record_quota(low, state, notifier=send)
    assert len(notifications) == 6


def test_failed_slack_send_is_not_marked_delivered(tmp_path):
    path = tmp_path / 'quota.json'; low = weekly_snapshot(limits(100), now=100)
    result = codex_quota.record_quota(low, path, notifier=lambda _: False)
    assert result['alerts_sent'] == 0 and result['notification_failures'] == 1
    sent = []
    result = codex_quota.record_quota(low, path, notifier=lambda m: sent.append(m) or True)
    assert result['alerts_sent'] == len(sent) == 3


def test_quota_guard_blocks_low_and_unknown(monkeypatch):
    monkeypatch.setattr(codex_quota, 'check_quota', lambda *a, **k: {'status': 'low'})
    with pytest.raises(QuotaLowError):
        codex_quota.ensure_quota()
    monkeypatch.setattr(codex_quota, 'check_quota', lambda *a, **k: {'status': 'unknown'})
    with pytest.raises(QuotaUnknownError):
        codex_quota.ensure_quota()


@pytest.fixture
def provider_env(monkeypatch):
    for name in ('OPENAI_API_KEY', 'CODEX_API_KEY', 'OPENAI_BASE_URL',
                 'AZURE_OPENAI_API_KEY', 'CODEX_OSS_BASE_URL'):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setattr(codex_provider.shutil, 'which', lambda _: '/fake/codex')
    quota = {'status': 'ok', 'remaining_percent': 50, 'resets_at': 2000000000}
    monkeypatch.setattr(codex_provider, 'ensure_quota', lambda *a, **k: quota)
    monkeypatch.setattr(codex_provider, 'check_quota', lambda *a, **k: quota)


def test_subscription_rejects_api_key_fallback(provider_env, monkeypatch):
    monkeypatch.setenv('OPENAI_API_KEY', 'synthetic-key')
    with pytest.raises(ValueError, match='OPENAI_API_KEY'):
        CodexSubscriptionProvider().complete('test', 'system', {}, {}, lambda v: v)


def test_schema_image_and_usage_are_recorded_without_tools(tmp_path, monkeypatch, provider_env):
    image = tmp_path / 'input.png'; image.write_bytes(b'fake-image-for-cli-mock')
    commands = []
    def run(argv, **kwargs):
        commands.append(argv)
        Path(argv[argv.index('--output-last-message') + 1]).write_text('{"choice": 2}')
        instructions = Path(kwargs['cwd']) / 'instructions.txt'
        assert 'English' in instructions.read_text()
        return SimpleNamespace(returncode=0, stdout=json.dumps({
            'type': 'turn.completed', 'usage': {'input_tokens': 25, 'output_tokens': 7}}), stderr='')
    monkeypatch.setattr(codex_provider.subprocess, 'run', run)
    provider = CodexSubscriptionProvider(tmp_path / 'calls')
    assert provider.complete('selection', 'Pick one.', {}, {}, lambda v: v,
                             images=[image]) == {'choice': 2}
    argv = commands[0]
    assert argv[argv.index('--model') + 1] == 'gpt-6-astra'
    assert argv[argv.index('--sandbox') + 1] == 'read-only'
    assert argv[argv.index('--image') + 1] == str(image)
    assert 'model_reasoning_effort="high"' in argv
    assert 'forced_login_method="chatgpt"' in argv
    assert 'features.shell_tool=false' in argv and 'features.apps=false' in argv
    assert '--ignore-user-config' in argv and '--ephemeral' in argv
    audit = json.loads(next((tmp_path / 'calls').glob('*.json')).read_text())
    assert audit['status'] == 'validated'
    assert audit['attempts'][0]['usage']['output_tokens'] == 7
    assert audit['request']['images'][0]['sha256']
    assert audit['reasoning_effort'] == 'high'


def test_quota_failure_never_starts_model(tmp_path, monkeypatch, provider_env):
    def blocked(*a, **k):
        raise QuotaLowError('quota low')
    monkeypatch.setattr(codex_provider, 'ensure_quota', blocked)
    monkeypatch.setattr(codex_provider.subprocess, 'run', lambda *a, **k: pytest.fail('model was called'))
    provider = CodexSubscriptionProvider(tmp_path / 'calls')
    with pytest.raises(QuotaLowError):
        provider.complete('selection', 'system', {}, {}, lambda v: v)
    assert json.loads(next((tmp_path / 'calls').glob('*.json')).read_text())['status'] == 'quota_wait'


def test_invalid_json_repairs_once_and_checks_quota_before_repair(tmp_path, monkeypatch, provider_env):
    checks = []; calls = []
    def guard(*a, **k):
        checks.append(True)
        return {'status': 'ok'}
    def run(argv, **kwargs):
        calls.append(kwargs['input'])
        Path(argv[argv.index('--output-last-message') + 1]).write_text('bad' if len(calls) == 1 else '{}')
        return SimpleNamespace(returncode=0, stdout='{"type":"turn.completed"}', stderr='')
    monkeypatch.setattr(codex_provider, 'ensure_quota', guard)
    monkeypatch.setattr(codex_provider.subprocess, 'run', run)
    assert CodexSubscriptionProvider().complete('test', 'system', {}, {}, lambda v: v) == {}
    assert len(checks) == len(calls) == 2
    assert json.loads(calls[1])['validation_feedback']


def test_unexpected_tool_usage_does_not_become_a_test(tmp_path, monkeypatch, provider_env):
    def run(argv, **kwargs):
        Path(argv[argv.index('--output-last-message') + 1]).write_text('{}')
        return SimpleNamespace(returncode=0, stdout='\n'.join([
            '{"type":"item.started","item":{"type":"command_execution"}}',
            '{"type":"turn.completed"}']), stderr='')
    monkeypatch.setattr(codex_provider.subprocess, 'run', run)
    with pytest.raises(RuntimeError, match='tool action'):
        CodexSubscriptionProvider().complete('test', 'system', {}, {}, lambda v: v)


def test_rpc_client_rejects_mutating_or_inference_methods():
    client = codex_quota.AccountClient('/fake/codex')
    with pytest.raises(ValueError, match='read-only'):
        client.request('turn/start')
    with pytest.raises(ValueError, match='read-only'):
        client.request('account/rateLimitResetCredit/consume')


def test_transport_schema_removes_only_schema_keyword_and_preserves_original():
    schema = {'type': 'object', 'properties': {
        'uniqueItems': {'type': 'string', 'enum': ['uniqueItems']},
        'citations': {'type': 'array', 'minItems': 1, 'uniqueItems': True,
                      'items': {'type': 'string'}}}}
    original = json.loads(json.dumps(schema))
    wire, changes = codex_provider.wire_schema(schema)
    assert schema == original
    assert wire['properties']['uniqueItems'] == schema['properties']['uniqueItems']
    assert wire['properties']['citations'] == {'type': 'array', 'minItems': 1, 'items': {'type': 'string'}}
    assert changes == [{'path': 'schema.properties.citations.uniqueItems', 'keyword': 'uniqueItems',
                        'value': True, 'enforcement': 'local_response_validator'}]


def test_actual_analysis_unique_evidence_validation_survives_wire_adaptation(tmp_path, monkeypatch, provider_env):
    from agent_analysis import analysis_schema, validate_analysis
    evidence = {'rounds': [{'id': 'r001', 'attack_effect_evaluable': True,
                            'attacked': {'outcome': 'collision'}}]}
    schema = analysis_schema(evidence)
    calls = []
    def run(argv, **kwargs):
        wire = json.loads(Path(argv[argv.index('--output-schema') + 1]).read_text())
        assert 'uniqueItems' not in wire['properties']['findings']['items']['properties']['evidence_rounds']
        assert 'uniqueItems' not in wire['properties']['defense_candidates']['items']['properties']['evidence_rounds']
        calls.append(kwargs['input'])
        value = {'summary': 'Synthetic provider validation test.',
                 'findings': [{'claim': 'A recorded failure.', 'status': 'observation',
                               'evidence_rounds': ['r001', 'r001'] if len(calls) == 1 else ['r001']}],
                 'limitations': ['A synthetic test is not a flight.'], 'next_tests': [], 'defense_candidates': []}
        Path(argv[argv.index('--output-last-message') + 1]).write_text(json.dumps(value))
        return SimpleNamespace(returncode=0, stdout='{"type":"turn.completed"}', stderr='')
    monkeypatch.setattr(codex_provider.subprocess, 'run', run)
    result = CodexSubscriptionProvider(tmp_path / 'calls').complete(
        'analysis', 'Test.', evidence, schema, lambda raw: validate_analysis(raw, evidence))
    assert len(calls) == 2 and result['findings'][0]['evidence_rounds'] == ['r001']
    assert 'unique' in json.loads(calls[1])['validation_feedback']
    audit = json.loads(next((tmp_path / 'calls').glob('*.json')).read_text())
    assert audit['request']['schema'] == schema
    assert len(audit['wire_schema']['adaptations']) == 2
    assert audit['wire_schema']['sha256'] == codex_provider.digest(codex_provider.wire_schema(schema)[0])


def test_provider_error_keeps_sanitized_schema_diagnosis(tmp_path, monkeypatch, provider_env):
    detail = {'type': 'error', 'error': {'message': 'Unsupported uniqueItems. token=synthetic-secret Bearer synthetic-auth',
                                      'code': 'invalid_json_schema'}, 'status': 400}
    stdout = '\n'.join([json.dumps({'type': 'error', 'message': json.dumps(detail)}),
                         json.dumps({'type': 'turn.failed', 'error': {'message': json.dumps(detail)}})])
    monkeypatch.setattr(codex_provider.subprocess, 'run', lambda *a, **k:
                        SimpleNamespace(returncode=1, stdout=stdout, stderr='DO NOT PERSIST RAW LOGS'))
    with pytest.raises(RuntimeError, match='invalid_json_schema'):
        CodexSubscriptionProvider(tmp_path / 'calls').complete('test', 'system', {}, {}, lambda raw: raw)
    audit = json.loads(next((tmp_path / 'calls').glob('*.json')).read_text())
    errors = audit['attempts'][0]['errors']
    assert len(errors) == 2 and all(error['code'] == 'invalid_json_schema' for error in errors)
    assert errors[0]['status'] == '400' and 'Unsupported uniqueItems' in errors[0]['message']
    assert 'synthetic-secret' not in json.dumps(audit) and 'synthetic-auth' not in json.dumps(audit)
    assert 'DO NOT PERSIST' not in json.dumps(audit)
