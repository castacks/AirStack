"""Read saved provider/episode audits; never call a provider or open credentials.

Usage: python3 agent_resource_audit.py CAMPAIGN [--output NEW_DERIVED_JSON]
The default prints JSON. An output path must be new; originals are never changed.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re

VERSION = 'recorded_agent_resources_v1'
MAX_JSON_BYTES = 16 * 1024 * 1024
TOKEN_FIELDS = ('input_tokens_total', 'output_tokens', 'total_tokens',
                'cached_input_tokens', 'cache_creation_input_tokens', 'reasoning_output_tokens')
TERMINAL = {'validated', 'invalid_response', 'provider_error', 'timeout', 'unexpected_tool_use', 'unexpected_model'}


def _number(value, integer=False):
    return (type(value) is int if integer else type(value) in (int, float)) and math.isfinite(value) and value >= 0


def _id(value):
    return value if isinstance(value, str) and re.fullmatch(r'[A-Za-z0-9_.:/ -]{1,160}', value) else None


def _hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def _nested(value, *path):
    for key in path:
        value = value.get(key) if isinstance(value, dict) else None
    return value


def _tokens(provider, usage):
    """Do not add modelUsage/iterations, cache subcategories or reasoning twice."""
    usage = usage if isinstance(usage, dict) else {}
    def get(*candidates):
        for path in candidates:
            value = _nested(usage, *path)
            if _number(value, True): return value
        return None
    inp = get(('input_tokens',)); out = get(('output_tokens',))
    if provider == 'claude_subscription':
        cached = get(('cache_read_input_tokens',))
        created = get(('cache_creation_input_tokens',))
        total_in = inp+cached+created if None not in (inp, cached, created) else None
        reasoning = get(('output_tokens_details', 'thinking_tokens'))
        semantics = 'Claude input_tokens excludes separately recorded cache reads and writes.'
    else:
        cached = get(('cached_input_tokens',), ('input_tokens_details', 'cached_tokens'))
        created = get(('cache_write_input_tokens',))
        total_in = inp
        reasoning = get(('reasoning_output_tokens',), ('output_tokens_details', 'reasoning_tokens'))
        semantics = 'Codex input_tokens already includes cached tokens; cache/reasoning fields are not added again.'
    values = dict(input_tokens_total=total_in, output_tokens=out,
                  total_tokens=total_in+out if None not in (total_in, out) else None,
                  cached_input_tokens=cached, cache_creation_input_tokens=created,
                  reasoning_output_tokens=reasoning)
    return dict(values, missing_fields=[key for key, value in values.items() if value is None], semantics=semantics)


def _category(purpose, output):
    if purpose == 'analysis': return 'final_report'
    operation = output.get('operation') if isinstance(output, dict) else None
    if operation in ('inspect', 'test', 'stop'):
        return {'inspect': 'inspection', 'test': 'selection', 'stop': 'agent_stop'}[operation]
    if 'planning' in purpose: return 'planning_unresolved'
    if 'selection' in purpose: return 'selection'
    return 'other'


def _sum(values):
    known = [v for v in values if _number(v)]
    return dict(known_sum=sum(known) if known else None, recorded_values=len(known),
                missing_values=len(values)-len(known), complete=len(known) == len(values))


def _summary(attempts):
    return dict(scope='Recorded attempts only; see audit_coverage for missing ledger entries.', recorded_attempts=len(attempts),
                elapsed_s=_sum([a['elapsed_s'] for a in attempts]),
                tokens={key: _sum([a['tokens'][key] for a in attempts]) for key in TOKEN_FIELDS},
                attached_image_transmissions=_sum([a['attached_images'] for a in attempts]),
                reported_cost_usd_estimate=_sum([a['reported_cost_usd_estimate'] for a in attempts]),
                outcomes=dict(sorted(Counter(a['outcome'] for a in attempts).items())))


def _call(record, sources):
    request = record['request']; provider = record['provider']; status = record.get('status')
    purpose = _id(request.get('purpose')) or 'unknown'
    images = request.get('images'); image_count = len(images) if isinstance(images, list) else None
    image_hashes = sorted({v['sha256'] for v in images or [] if isinstance(v, dict)
        and isinstance(v.get('sha256'), str) and re.fullmatch(r'[0-9a-f]{64}', v['sha256'])}) if isinstance(images, list) else []
    values = record['attempts']; attempts = []
    for index, raw in enumerate(values):
        final = index == len(values)-1
        if not final: outcome = 'rejected_before_retry'
        elif status == 'validated': outcome = 'accepted'
        elif status == 'invalid_response': outcome = 'validation_rejected'
        elif status == 'provider_error' and provider == 'claude_subscription':
            outcome = 'earlier_recorded_attempt_not_final_transport_error'
        elif status in ('provider_error', 'unexpected_tool_use', 'unexpected_model'): outcome = status
        else: outcome = 'not_accepted_or_status_unavailable'
        model_usage = raw.get('model_usage')
        model_ids = sorted(v for key in (model_usage or {}) if (v := _id(key))) if isinstance(model_usage, dict) else []
        attempts.append(dict(index=index+1, category=_category(purpose, raw.get('output')),
            outcome=outcome, elapsed_s=raw['elapsed_s'] if _number(raw.get('elapsed_s')) else None,
            tokens=_tokens(provider, raw.get('usage')),
            usage_recorded=isinstance(raw.get('usage'), dict), attached_images=image_count,
            resolved_model_from_event=_id(raw.get('resolved_model')),
            model_ids_from_model_usage=model_ids,
            recorded_error_events=len(raw.get('errors', [])) if isinstance(raw.get('errors'), list) else None,
            reported_cost_usd_estimate=raw['reported_cost_usd'] if _number(raw.get('reported_cost_usd')) else None))
    # Claude historically omits malformed/failed transport attempts; Codex omits
    # timeouts. A request without status may still be in progress. Never call
    # missing duration/usage zero or manufacture an exact transport-attempt count.
    unrecorded_possible = (status not in TERMINAL or status == 'timeout' or
        (provider == 'claude_subscription' and (status == 'provider_error' or
         (bool(record.get('validation_error')) and len(values) < 2))))
    if status == 'quota_wait': unrecorded_possible = False  # guards run before each recorded attempt
    category = _category(purpose, values[-1].get('output') if values else None)
    return dict(call_id=record['call_id'], provider=provider, purpose=purpose, category=category,
        status=_id(status), requested_model=_id(record.get('requested_model')),
        requested_reasoning_effort=_id(record.get('reasoning_effort')), sources=sources,
        recorded_attempt_count=len(attempts), recorded_retry_attempts=max(0, len(attempts)-1),
        validation_feedback_recorded=bool(record.get('validation_error')),
        additional_unrecorded_attempts_possible=unrecorded_possible,
        image_manifest_count=image_count, distinct_image_sha256=image_hashes,
        attempts=attempts, resources=_summary(attempts))


def audit_resources(root):
    """Deduplicate calls by provider/call_id and episodes by recorded result_dir.

    Same-content copies count once. Conflicting copies are explicitly excluded,
    so totals are incomplete rather than double-counted or silently chosen.
    Distinct recovery calls with the same request are real extra resource use.
    """
    root = Path(root).expanduser().resolve(strict=True)
    if not root.is_dir(): raise ValueError('Supply a campaign tree or llm_calls directory')
    issues = []; grouped = {}; episodes = {}; duplicates = 0
    call_paths = root.glob('*.json') if root.name == 'llm_calls' else root.rglob('llm_calls/*.json')
    def load(path):
        try:
            path.resolve().relative_to(root)
            if path.stat().st_size > MAX_JSON_BYTES: raise ValueError('json_exceeds_size_bound')
            data = path.read_bytes(); value = json.loads(data)
            if not isinstance(value, dict): raise ValueError('json_not_object')
            _hash(value)  # Reject nonfinite values without echoing source content.
            return value, {'file': str(path.relative_to(root)), 'sha256': hashlib.sha256(data).hexdigest()}
        except (OSError, ValueError, TypeError):
            issues.append({'file': str(path.relative_to(root)), 'reason': 'unreadable_invalid_or_out_of_scope_json'})
            return None, None
    for path in sorted(call_paths):
        value, source = load(path)
        if value is None: continue
        if (value.get('provider') not in ('codex_subscription', 'claude_subscription') or
            not _id(value.get('call_id')) or not isinstance(value.get('request'), dict) or
            not isinstance(value.get('attempts'), list) or not all(isinstance(a, dict) for a in value['attempts'])):
            issues.append(dict(file=source['file'], reason='unsupported_provider_audit')); continue
        key = (value['provider'], value['call_id'])
        grouped.setdefault(key, []).append((value, source))
    calls = []; conflicts = []
    for key, copies in sorted(grouped.items()):
        if len({_hash(value) for value, _ in copies}) != 1:
            conflicts.append(dict(provider=key[0], call_id=key[1], sources=[s for _, s in copies])); continue
        duplicates += len(copies)-1
        calls.append(_call(copies[0][0], [s for _, s in copies]))
    for path in sorted(root.rglob('result.json')):
        value, source = load(path)
        if value is None: continue
        identity = value.get('result_dir')
        if not isinstance(identity, str) or not identity or not _id(value.get('outcome')):
            issues.append(dict(file=source['file'], reason='missing_episode_identity')); continue
        metrics = value.get('metrics') if isinstance(value.get('metrics'), dict) else {}
        selected = dict(identity=identity, configuration_hash=value.get('configuration_hash'),
            outcome=value['outcome'], episode_wall_duration_s=value.get('wall_duration_s'),
            mission_duration_sim_s=metrics.get('mission_duration_sim_s'),
            planner_wall_duration_s=metrics.get('planner_wall_duration_s'))
        episodes.setdefault(identity, []).append((selected, source))
    flights = []; flight_conflicts = []
    for identity, copies in sorted(episodes.items()):
        if len({_hash(v) for v, _ in copies}) != 1:
            flight_conflicts.append(dict(recorded_result_dir=identity, sources=[s for _, s in copies])); continue
        item = copies[0][0]
        flights.append(dict(item, sources=[s for _, s in copies],
            **{key: item[key] if _number(item[key]) else None for key in
               ('episode_wall_duration_s', 'mission_duration_sim_s', 'planner_wall_duration_s')}))
    attempts = [a for call in calls for a in call['attempts']]
    ledger_complete = not (conflicts or issues or any(c['additional_unrecorded_attempts_possible'] for c in calls))
    return dict(schema=VERSION, root=str(root), derived_only=True,
        provider_requests=len(calls), excluded_conflicting_request_ids=len(conflicts),
        duplicate_call_copies=duplicates, recorded_retry_attempts=sum(c['recorded_retry_attempts'] for c in calls),
        calls_with_validation_feedback=sum(c['validation_feedback_recorded'] for c in calls),
        calls_with_possible_unrecorded_attempts=sum(c['additional_unrecorded_attempts_possible'] for c in calls),
        audit_coverage=dict(scope='Only the supplied saved audit tree; unlogged external calls cannot be detected.',
            saved_attempt_ledger_complete=ledger_complete,
            elapsed_complete_for_saved_requests=ledger_complete and all(a['elapsed_s'] is not None for a in attempts),
            token_totals_complete_for_saved_requests=ledger_complete and all(a['tokens']['total_tokens'] is not None for a in attempts)),
        call_statuses=dict(sorted(Counter(c['status'] or 'unavailable' for c in calls).items())),
        totals=_summary(attempts),
        by_provider={provider: dict(provider_requests=sum(c['provider'] == provider for c in calls),
            **_summary([a for c in calls if c['provider'] == provider for a in c['attempts']]))
            for provider in sorted({c['provider'] for c in calls})},
        by_category={category: dict(provider_requests=sum(c['category'] == category for c in calls),
            **_summary([a for a in attempts if a['category'] == category]))
            for category in sorted({c['category'] for c in calls}|{a['category'] for a in attempts})},
        episodes=dict(recorded_attempts=len(flights),
            outcomes=dict(sorted(Counter(f['outcome'] for f in flights).items())),
            **{key: _sum([f[key] for f in flights]) for key in
               ('episode_wall_duration_s', 'mission_duration_sim_s', 'planner_wall_duration_s')},
            records=flights, conflicting_records=flight_conflicts),
        calls=calls, conflicting_calls=conflicts, issues=issues,
        limitations=[
            'Provider requests and recorded transport attempts differ; missing attempts/usage are unavailable, never zero.',
            'Cached input is already included by Codex; Claude cache reads/writes are additional input. Reasoning is a subset of output. Iterations/modelUsage are not added again.',
            'Elapsed seconds sum recorded provider attempts, not total campaign elapsed time. Authentication, pre-call quota checks and pauses are not fully recorded; Codex attempt timing includes its post-call quota check.',
            'Image counts are recorded attachments/transmissions, not image-token counts. Missing manifests are unknown; image files are never opened.',
            'Requested model is not a resolved model. CLI model events and Claude modelUsage keys are reported separately without inventing missing identities.',
            'Reported USD is a CLI estimate, not subscription billing. Provider tokenizers and quotas are not directly interchangeable.',
            'Distinct recovery/retry requests count as real overhead; copied IDs count once. Conflicting copies are excluded and make totals incomplete.',
            'Episode results count recorded attempts, including infrastructure failures. They do not by themselves prove takeoff; wall time includes setup/cleanup while mission duration uses simulation time.',
            'Flight and LLM duration sums are not an end-to-end stopwatch, causal explanation or proof of agent superiority.'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(); report = audit_resources(args.root)
    data = json.dumps(report, indent=2, allow_nan=False)+'\n'
    if args.output:
        if 'llm_calls' in args.output.parts or args.output.name in ('result.json', 'history.json', 'config.json', 'decision.json'):
            parser.error('Use a new derived report filename outside llm_calls')
        with args.output.open('x') as stream: stream.write(data)
    else: print(data, end='')


if __name__ == '__main__': main()
