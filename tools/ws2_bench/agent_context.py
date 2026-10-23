"""Compact, evidence-preserving decision context (opt-in; no flight behavior).

All executed rounds retain their exact action, experiment references, outcomes
and metrics. Only recent rounds carry selected diagnostic snapshots. The full
original traces/results stay on disk and remain the validation authority.
This module never evaluates or reclassifies an experimental result.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import re

VERSION = 'compact_decision_context_v1'
MAX_TRACE_BYTES = 64 * 1024 * 1024
DEFAULT_RECENT_ROUNDS = 4
DEFAULT_BYTE_BUDGET = 160_000
MAX_FIELD_STRING_CHARS = 1000
MAX_FIELD_ITEMS = 32
MAX_FIELD_DEPTH = 6
MAX_FIELD_NODES = 256
MODEL_FIELDS = (
    'sequence', 'sim_stamp', 'inference_ms', 'primitive', 'blocked', 'recovery_mode',
    'action', 'q_values', 'model_variant', 'policy_profile', 'steering',
    'collision_probability', 'input_fov_deg', 'speed', 'policy_speed',
    'target_speed', 'yaw_rate', 'estimated_forward_clearance_m',
    'depth_summary_m', 'depth_summary_normalized', 'depth_source',
    'consumed_input',
)
METRIC_FIELDS = (
    'time_to_goal_s', 'path_length_m', 'path_efficiency', 'final_distance_to_goal_m',
    'mission_progress_percent', 'planner_hold_count', 'planner_recovery_count',
    'minimum_obstacle_clearance_m', 'mean_obstacle_clearance_m',
    'mean_speed_m_s', 'stationary_fraction', 'max_displacement_m',
    'mission_duration_sim_s', 'route_progress_m', 'route_progress_percent',
    'route_gates_passed', 'route_gate_count', 'route_completed',
)
AGGREGATE_FIELDS = (
    'window', 'window_start_sim_s', 'window_end_sim_s', 'mission_elapsed_sim_s',
    'sample_count', 'unique_worker_frames', 'camera_age_sim_s',
    'worker_input_age_sim_s', 'inference_ms', 'clearance_m',
    'path_length_m', 'mean_motion_speed_m_s',
)
SENSOR_FIELDS = (
    'rgb_noise_stddev', 'depth_noise_stddev_m', 'fixed_delay_s',
    'noise_window_active', 'delay_window_active', 'schedule_elapsed_s',
    'schedule_start_sim_s',
)
COMMAND_FIELDS = ('command_index', 'primitive_index', 'sim_stamp', 'published',
                  'velocity_m_s', 'first_waypoint_m', 'last_waypoint_m')
SNAPSHOT_FIELDS = (
    'position_m', 'clearance_m', 'camera_age_sim_s', 'patch_scene_enabled',
)
FILESYSTEM_PATH = re.compile(r'(?<![\w:])/(?:home|root|tmp|workspace|isaac-sim|mnt|var)/[^\s"\'<>]*')
_INVALID = object()


def _text(value):
    return FILESYSTEM_PATH.sub('[local path omitted]', value) if isinstance(value, str) else value


def _present(value, fields):
    """Do not turn missing/NaN data into numeric zero."""
    if not isinstance(value, dict):
        return {}
    invalid = []; truncated = []; nodes = [0]
    selected = {}
    for key in fields:
        if key not in value or value[key] is None:
            continue
        cleaned = _finite_observation(value[key], key, invalid, truncated, nodes)
        if cleaned is not _INVALID:
            selected[key] = cleaned
    if invalid:
        # Paths are relative to this field group; the surrounding snapshot/run
        # supplies evidence_id and timestamp. Invalid vector components cause
        # the entire vector to be omitted, never dimensionality reduction.
        selected['invalid_fields'] = invalid
    if truncated:
        selected['truncated_fields'] = truncated
    return selected


def _finite_observation(value, path, invalid, truncated, nodes, depth=0):
    # Limits apply only to observations and explanatory text, never replay
    # actions, experiment references or outcomes. Record exactly which source
    # field lost detail, and leave its full original available for inspection.
    nodes[0] += 1
    if nodes[0] > MAX_FIELD_NODES or depth > MAX_FIELD_DEPTH:
        truncated.append({'field': path, 'reason': 'node_or_depth_limit'})
        return _INVALID
    if isinstance(value, float) and not math.isfinite(value):
        invalid.append(path)
        return _INVALID
    if isinstance(value, str):
        cleaned = _text(value)
        if len(cleaned) > MAX_FIELD_STRING_CHARS:
            truncated.append({'field': path, 'reason': 'string_limit',
                              'original_characters': len(cleaned), 'retained_characters': MAX_FIELD_STRING_CHARS})
            return cleaned[:MAX_FIELD_STRING_CHARS]
        return cleaned
    if isinstance(value, dict):
        result = {}
        if len(value) > MAX_FIELD_ITEMS:
            truncated.append({'field': path, 'reason': 'mapping_limit',
                              'original_items': len(value), 'retained_items': MAX_FIELD_ITEMS})
        for index, (key, item) in enumerate(value.items()):
            if index >= MAX_FIELD_ITEMS or nodes[0] >= MAX_FIELD_NODES:
                if index < min(len(value), MAX_FIELD_ITEMS):
                    truncated.append({'field': path, 'reason': 'node_limit', 'retained_items': index})
                break
            if not isinstance(key, str) or len(key) > MAX_FIELD_STRING_CHARS:
                truncated.append({'field': path, 'reason': 'invalid_or_oversized_mapping_key', 'item_index': index})
                continue
            cleaned = _finite_observation(item, f'{path}.{key}', invalid, truncated, nodes, depth+1)
            if cleaned is not _INVALID:
                result[key] = cleaned
        return result
    if isinstance(value, (list, tuple)):
        limit = min(len(value), MAX_FIELD_ITEMS, max(0, MAX_FIELD_NODES-nodes[0]))
        if limit < len(value):
            truncated.append({'field': path, 'reason': 'array_or_node_limit',
                              'original_items': len(value), 'retained_items': limit})
        values = [_finite_observation(item, f'{path}[{index}]', invalid, truncated, nodes, depth+1)
                  for index, item in enumerate(value[:limit])]
        return _INVALID if any(item is _INVALID for item in values) else values
    return copy.deepcopy(value)


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _trace(result, runtime_root):
    rows, provenance = _trace_document(result, runtime_root)
    return rows, provenance['status']


def _trace_document(result, runtime_root):
    if runtime_root is None:
        return None, {'status': 'no_runtime_root'}
    directory = result.get('result_dir')
    if not isinstance(directory, str):
        return None, {'status': 'no_result_directory'}
    root = Path(runtime_root).resolve(); folder = Path(directory).resolve()
    try:
        folder.relative_to(root)
        path = (folder/'diagnostic_trace.json').resolve(); path.relative_to(folder)
        identity_path = (folder/'result.json').resolve(); identity_path.relative_to(folder)
        scenario_path = (folder/'scenario.json').resolve(); scenario_path.relative_to(folder)
        if path.stat().st_size > MAX_TRACE_BYTES:
            return None, {'status': 'trace_exceeds_read_bound'}
        saved = json.loads(identity_path.read_text())
        if saved.get('result_dir') != str(folder):
            return None, {'status': 'result_identity_mismatch'}
        if result.get('configuration_hash') and saved.get('configuration_hash') != result['configuration_hash']:
            return None, {'status': 'configuration_hash_mismatch'}
        if saved.get('outcome') != result.get('outcome'):
            return None, {'status': 'result_outcome_mismatch'}
        scenario = json.loads(scenario_path.read_text())
        scenario_hash = hashlib.sha256(json.dumps(scenario, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
        if saved.get('configuration_hash') != scenario_hash:
            return None, {'status': 'scenario_hash_mismatch'}
        data = path.read_bytes()
        values = json.loads(data)
        if not isinstance(values, list):
            return None, {'status': 'trace_not_list'}
        rows = [v for v in values if isinstance(v, dict) and _finite(v.get('sim_time_s'))]
        return rows, {'status': 'diagnostic_trace', 'trace_sha256': hashlib.sha256(data).hexdigest(),
                      'configuration_hash': saved.get('configuration_hash'),
                      'scenario_hash_verified': True,
                      'trace_record_count': len(values), 'invalid_timestamp_records': len(values)-len(rows)}
    except (OSError, ValueError, TypeError):
        return None, {'status': 'trace_unavailable_or_out_of_scope'}


def _available_summary_snapshots(diagnostics):
    """Fallback is explicitly sparse; never call its minimum a flight minimum."""
    rows = []
    for frame in [diagnostics.get('first_inference'), *diagnostics.get('preview_frames', [])]:
        if not isinstance(frame, dict) or not _finite(frame.get('capture_sim_s')):
            continue
        rows.append({**frame.get('capture_observation', {}), 'sim_time_s': frame['capture_sim_s'],
                     'model_output': frame.get('model_output', {})})
    for row in diagnostics.get('snapshots', []):
        if isinstance(row, dict) and _finite(row.get('sim_time_s')):
            rows.append(row)
    # Stable latest occurrence at the same timestamp; original evidence is kept.
    return sorted({r['sim_time_s']: r for r in rows}.values(), key=lambda r: r['sim_time_s'])


def _snapshot(row, evidence_id, labels):
    value = {'source': {'evidence_id': evidence_id, 'sim_time_s': row['sim_time_s']},
             'selected_as': labels, **_present(row, SNAPSHOT_FIELDS)}
    value['model_output'] = _present(row.get('worker') or row.get('model_output'), MODEL_FIELDS)
    value['command'] = _present(row.get('command'), COMMAND_FIELDS)
    released = row.get('bridge_released_input')
    if not isinstance(released, dict):
        released = {'sequence': row.get('camera_sequence'), 'sim_stamp': row.get('latest_input_stamp'),
                    'sensor_disturbance': row.get('latest_input_disturbance'),
                    'rgb_noise_rmse': row.get('latest_input_rgb_noise_rmse')}
    value['bridge_released_input'] = _present(released, ('sequence', 'sim_stamp', 'rgb_noise_rmse'))
    value['bridge_released_input']['sensor_disturbance'] = _present(released.get('sensor_disturbance'), SENSOR_FIELDS)
    consumed = (row.get('worker') or row.get('model_output') or {}).get('consumed_input')
    if isinstance(consumed, dict):
        compact_consumed = _present(consumed, ('schema_version', 'sequence', 'sim_stamp', 'disturbance_revision', 'rgb_noise_rmse'))
        compact_consumed['sensor_disturbance'] = _present(consumed.get('sensor_disturbance'), SENSOR_FIELDS)
        value['model_output']['consumed_input'] = compact_consumed
    # The live bridge clock explains asynchronous scene/input ordering. Keep
    # this separate from actual delivered-frame disturbance, not interchangeable.
    value['live_bridge_disturbance'] = _present(row.get('sensor_disturbance'), SENSOR_FIELDS)
    return {k: v for k, v in value.items() if v != {}}


def compact_diagnostics(result, evidence_id, runtime_root=None):
    diagnostics = result.get('diagnostics') or {}
    rows, source = _trace(result, runtime_root)
    fallback_reason = None
    if rows is None:
        fallback_reason = source
        rows = _available_summary_snapshots(diagnostics)
        source = 'available_summary_snapshots'
    rows = sorted(rows, key=lambda r: r['sim_time_s'])
    selected = {}
    if rows:
        selected.setdefault(0, []).append('first_available')
        candidates = [i for i, row in enumerate(rows) if _finite(row.get('clearance_m'))]
        if candidates:
            minimum = min(candidates, key=lambda i: (rows[i]['clearance_m'], rows[i]['sim_time_s'], i))
            selected.setdefault(minimum, []).append('minimum_observed_clearance')
        selected.setdefault(len(rows)-1, []).append('last_available')
    retention = {'source': source, 'available_samples': len(rows),
                 'selected_samples': len(selected), 'omitted_samples': len(rows)-len(selected),
                 'minimum_scope': 'available trace samples' if source == 'diagnostic_trace' else 'sparse available summary samples',
                 'full_original_artifacts_preserved': True}
    if fallback_reason:
        retention['trace_fallback_reason'] = fallback_reason
    return {'aggregate_window': _present(diagnostics, AGGREGATE_FIELDS),
            'snapshots': [_snapshot(rows[i], evidence_id, selected[i]) for i in sorted(selected)],
            'retention': retention,
            'alignment': 'Scene sample, live bridge clock and model input are asynchronous; no interpolated samples.'}


def _exposure(raw):
    if not isinstance(raw, dict):
        return None
    value = _present(raw, ('status', 'exposure_verified', 'captured_frames', 'reason'))
    if 'reason' in value:
        value['reason'] = _text(value['reason'])
    if isinstance(raw.get('factors'), dict):
        factors = {}
        for index, (name, factor) in enumerate(raw['factors'].items()):
            if index >= MAX_FIELD_ITEMS:
                break
            factors[name] = _present(factor, ('status', 'requested', 'exposure_verified',
                                            'active_frames', 'inactive_frames', 'unknown_frames'))
        value.update(_present({'factors': factors}, ('factors',)))
        if len(raw['factors']) > MAX_FIELD_ITEMS:
            value.setdefault('truncated_fields', []).append({'field': 'factors', 'reason': 'mapping_limit',
                'original_items': len(raw['factors']), 'retained_items': MAX_FIELD_ITEMS})
    return value


def _run(result, evidence_id, detail, runtime_root):
    result = result or {}; termination = result.get('termination') or {}
    value = {'evidence_id': evidence_id, 'outcome': result.get('outcome'),
             'executed': result.get('executed', True),
             'metrics': _present(result.get('metrics'), METRIC_FIELDS),
             'termination': _present(termination, ('source', 'reason')),
             'termination_detail_preserved_in_original': bool(termination.get('detail')),
             'cleanup_error_count': len(result.get('cleanup_errors') or []),
             'attempts': [{'outcome': item.get('outcome')} for item in result.get('attempts', [])]}
    exposure = _exposure(result.get('attack_exposure'))
    if exposure is not None:
        value['attack_exposure'] = exposure
    if isinstance(result.get('worker_input_exposure'), dict):
        value['worker_input_exposure'] = _present(result['worker_input_exposure'], (
            'schema', 'status', 'configuration_checked', 'population', 'observed_unique_consumed_frames',
            'observed_released_frame_identities', 'factor_counts', 'evidence_sources', 'issues', 'reason',
            'source', 'artifact', 'limitations'))
    if detail:
        value['diagnostics'] = compact_diagnostics(result, evidence_id, runtime_root)
    else:
        value['diagnostics_omitted_from_prompt'] = True
    return value


def _reviews(decision):
    """Keep every review's reference/assessment; detailed text is last four."""
    values = decision.get('hypothesis_reviews')
    if not isinstance(values, list):
        return {}
    selected = []
    for index, review in enumerate(values):
        if not isinstance(review, dict):
            raise ValueError('Hypothesis reviews must be objects with source round references')
        # Reference IDs and the assessment are core scientific history. Do not
        # silently repair or truncate them to meet a prompt budget.
        item = {key: copy.deepcopy(review[key]) for key in ('source_round', 'assessment') if key in review}
        if index >= len(values)-4:
            item.update(_present(review, ('evidence',)))
        elif 'evidence' in review:
            item['evidence_text_omitted_from_prompt'] = True
        selected.append(item)
    return {'hypothesis_reviews': selected}


def compact_history(history, recent_rounds=DEFAULT_RECENT_ROUNDS, runtime_root=None,
                    byte_budget=DEFAULT_BYTE_BUDGET):
    """Keep all source configurations; bound detailed snapshots to last N rounds.

    Exact original `history` remains the input to repeat/isolate/threshold
    validation. This returned object is only the LLM's decision context.
    """
    if type(recent_rounds) is not int or not 0 <= recent_rounds <= 12:
        raise ValueError('recent_rounds must be an integer in 0..12')
    if type(byte_budget) is not int or byte_budget <= 0:
        raise ValueError('byte_budget must be a positive integer')
    if not isinstance(history, list):
        raise ValueError('history must be an array of executed rounds')
    start = max(0, len(history)-recent_rounds); rounds = []
    for index, entry in enumerate(history):
        decision = entry['decision']; number = decision['round']
        if type(number) is not int or number <= 0:
            raise ValueError('Executed round IDs must be positive integers')
        # An invalid observation can be marked unavailable, but a source action
        # must remain exact for replay and scientific comparisons. Never repair
        # or omit a non-finite action/experiment parameter in the summary.
        try:
            json.dumps({'action': decision['action'], 'experiment': decision.get('experiment')}, allow_nan=False)
        except (ValueError, TypeError):
            raise ValueError(f'Round {number} has a non-finite or non-JSON source action/experiment') from None
        detail = index >= start and recent_rounds > 0
        item = {'round': number, 'action': copy.deepcopy(decision['action']),
                'experiment': copy.deepcopy(decision.get('experiment')),
                **_present(decision, ('hypothesis', 'planning_version', 'next_budget_intent')),
                **_reviews(decision), 'pairs': []}
        plan_fields = ('version', 'next_budget_intent')
        if detail:
            plan_fields += ('hypothesis_reviews', 'steps', 'supports_if', 'refuted_if',
                            'expected_attack_outcomes', 'allocation', 'prediction')
        if decision.get('plan'):
            item['plan'] = _present(decision['plan'], plan_fields)
        for pair_index, pair in enumerate(entry.get('pairs', [])):
            prefix = f'r{number:03d}' + (f'_p{pair_index+1:02d}' if pair_index else '')
            item['pairs'].append({'planner': pair.get('planner'),
                                  **({'factor_exposure': copy.deepcopy(pair['factor_exposure'])}
                                     if 'factor_exposure' in pair else {}),
                                  **{role: _run(pair.get(role), f'{prefix}_{role}', detail, runtime_root)
                                     for role in ('clean', 'perturbed')}})
        rounds.append(item)
    context = {'context_version': VERSION, 'rounds': rounds,
            'retention': {'total_rounds': len(history), 'detailed_rounds': [r['round'] for r in rounds[start:]] if recent_rounds else [],
                          'byte_budget': byte_budget, 'requested_recent_rounds': recent_rounds,
                          'detail_reduction_attempts': [recent_rounds],
                          'diagnostic_field_limits': {'string_characters': MAX_FIELD_STRING_CHARS,
                              'array_or_mapping_items': MAX_FIELD_ITEMS, 'nesting_depth': MAX_FIELD_DEPTH,
                              'nodes_per_field_group': MAX_FIELD_NODES},
                          'max_snapshots_per_detailed_run': 3,
                          'all_actions_outcomes_and_experiment_references_retained': True,
                          'history_is_summary_not_original_validation_input': True,
                          'missing_values_are_not_zero': True,
                          'full_trace_inspection': 'Preserved in original artifacts. resolve_telemetry_requests supports bounded text windows; caller must explicitly connect that inspection API.'}}
    for keep in dict.fromkeys((recent_rounds, min(recent_rounds, 2), 0)):
        if keep != recent_rounds:
            context['retention']['detail_reduction_attempts'].append(keep)
            cutoff = max(0, len(rounds)-keep)
            for index, item in enumerate(rounds[:cutoff]):
                if 'plan' in item:
                    item['plan'] = _present(history[index]['decision'].get('plan'), ('version', 'next_budget_intent'))
                for pair in item['pairs']:
                    for role in ('clean', 'perturbed'):
                        pair[role].pop('diagnostics', None)
                        pair[role]['diagnostics_omitted_from_prompt'] = True
            context['retention']['detailed_rounds'] = [r['round'] for r in rounds[cutoff:]] if keep else []
        size = encoded_bytes(context)
        if size <= byte_budget:
            return context
    raise ValueError(f'Compact decision context required core is {size} bytes, exceeding '
                     f'byte_budget={byte_budget} after removing all diagnostic details. '
                     'Exact actions, outcomes and experiment source references cannot be truncated; '
                     'stop and preserve the campaign or explicitly increase the context budget.')


def resolve_telemetry_requests(history, requests, runtime_root, max_samples=20):
    """Read at most 3 trusted run/time windows, each +/-2s and <=20 samples.

    Hashes identify the bytes observed during this request; they are not an
    assertion that a preexisting signed trace manifest was available. No path
    supplied by an LLM is opened. All original rows remain unchanged on disk.
    """
    if type(max_samples) is not int or not 2 <= max_samples <= 20:
        raise ValueError('max_samples must be an integer in 2..20')
    if not isinstance(requests, list) or len(requests) > 3:
        raise ValueError('At most three telemetry requests are allowed')
    known = {}
    for entry in history:
        number = entry['decision']['round']
        for index, pair in enumerate(entry.get('pairs', [])):
            prefix = f'r{number:03d}' + (f'_p{index+1:02d}' if index else '')
            for role in ('clean', 'perturbed'):
                if isinstance(pair.get(role), dict):
                    known[f'{prefix}_{role}'] = pair[role]
    resolved = []
    for request in requests:
        if not isinstance(request, dict) or set(request) != {'evidence_id', 'view', 'sim_time_s'}:
            raise ValueError('Telemetry request requires evidence_id, view, sim_time_s')
        evidence_id = request['evidence_id']; stamp = request['sim_time_s']
        if not isinstance(evidence_id, str) or evidence_id not in known or request['view'] != 'telemetry':
            raise ValueError('Unknown recorded evidence ID or telemetry view')
        if stamp is not None and (not _finite(stamp) or stamp < 0):
            raise ValueError('Telemetry time must be nonnegative finite simulation seconds or null')
        rows, provenance = _trace_document(known[evidence_id], runtime_root)
        response = {'request': copy.deepcopy(request), 'source': {'evidence_id': evidence_id, **provenance},
                    'status': 'unavailable', 'samples': []}
        if not rows:
            response['reason'] = provenance['status'] if rows is None else 'trace_contains_no_timestamped_samples'
            resolved.append(response); continue
        rows = sorted(rows, key=lambda row: row['sim_time_s'])
        center = rows[-1]['sim_time_s'] if stamp is None else float(stamp)
        begin, end = max(0., center-2.), center+2.
        window = [row for row in rows if begin <= row['sim_time_s'] <= end]
        response.update(requested_window_sim_s=[begin, end],
                        trace_available_window_sim_s=[rows[0]['sim_time_s'], rows[-1]['sim_time_s']],
                        available_samples=len(window), original_trace_samples=len(rows))
        if not window:
            response['reason'] = 'No recorded samples in requested window; none extrapolated or substituted'
            resolved.append(response); continue
        count = min(max_samples, len(window))
        indices = [0] if count == 1 else [round(i*(len(window)-1)/(count-1)) for i in range(count)]
        selected = [window[i] for i in indices]
        gaps = [b['sim_time_s']-a['sim_time_s'] for a, b in zip(window, window[1:])]
        selected_gaps = [b['sim_time_s']-a['sim_time_s'] for a, b in zip(selected, selected[1:])]
        response.update(status='available', actual_window_sim_s=[window[0]['sim_time_s'], window[-1]['sim_time_s']],
                        samples=[_snapshot(row, evidence_id, ['requested_telemetry_window']) for row in selected],
                        returned_samples=len(selected), omitted_samples=len(window)-len(selected),
                        sampling='Uniform deterministic indices of real recorded rows; endpoints retained, no interpolation',
                        max_original_gap_sim_s=max(gaps) if gaps else None,
                        max_returned_gap_sim_s=max(selected_gaps) if selected_gaps else None)
        resolved.append(response)
    return {'manifest': {'context_version': VERSION, 'requests': resolved,
                         'limitations': ['Text summaries preserve selected measured fields, not full ROS messages.',
                                         'Samples and clocks are asynchronous; gaps/omissions are explicit.',
                                         'Trace hashes describe the observed original file bytes; no original artifact is rewritten.']}}


def encoded_bytes(value):
    return len(json.dumps(value, separators=(',', ':'), ensure_ascii=False, allow_nan=False).encode('utf-8'))
