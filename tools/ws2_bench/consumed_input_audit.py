"""Offline audit of sampled worker-consumed source frames; launches nothing.

Current bridge availability is a different population. This audit never invents
an exact worker-consumption timestamp or a delivery-latency measurement.
"""
import copy
import hashlib
import json
import math
from pathlib import Path
import tempfile

VERSION = 'observed_consumed_input_v1'
LIMITATIONS = [
    'Counts are unique worker input updates observed by the recorder, not every model inference.',
    'RGB noise RMSE is measured on the source camera frame before JPEG, cropping and tensor preprocessing.',
    'Active delay means metadata records imposed delay on a consumed source frame; exact HTTP delivery latency is not measured.',
    'Input capture time and first observed worker output time are different; neither supplies an exact model-consumption pose.',
    'A current input update does not describe all state: Kim uses depth history and MonoNav uses temporal filters/TSDF.',
    'Matched consumed exposure is evidence for follow-up testing, not proof of an attack causing failure.',
]


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _identity(sequence, stamp):
    if type(sequence) is not int or sequence < 0 or not _finite(stamp) or stamp < 0:
        return None
    return sequence, stamp


def _signature(value):
    try:
        return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)
    except (TypeError, ValueError):
        return None


def _scheduled(schedule, elapsed):
    if schedule is None: return True
    if (not isinstance(schedule, dict) or set(schedule) != {'start_s', 'duration_s', 'period_s'} or
            any(not _finite(value) or value < 0 for value in schedule.values())):
        return None
    if schedule['period_s'] and not 0 < schedule['duration_s'] < schedule['period_s']: return None
    if elapsed is None: return False
    if not _finite(elapsed): return None
    if elapsed < schedule['start_s']: return False
    offset = elapsed-schedule['start_s']
    if schedule['period_s']: offset %= schedule['period_s']
    return schedule['duration_s'] == 0 or offset < schedule['duration_s']


def _factor_states(evidence, condition):
    states = {'noise': 'unknown', 'delay': 'unknown'}; problems = []
    disturbance = evidence.get('sensor_disturbance')
    if not isinstance(disturbance, dict): return states, ['missing_consumed_sensor_metadata']
    for factor, field, setting, schedule_key in (
            ('noise', 'rgb_noise_stddev', 'rgb_noise', 'noise_schedule'),
            ('delay', 'fixed_delay_s', 'delay', 'delay_schedule')):
        magnitude = disturbance.get(field)
        if not _finite(magnitude) or magnitude < 0:
            problems.append(factor+'_invalid_effective_magnitude'); continue
        if condition is not None:
            requested = condition.get(setting, 0.)
            active = _scheduled(condition.get(schedule_key), disturbance.get('schedule_elapsed_s'))
            if (not _finite(requested) or requested < 0 or active is None or
                    disturbance.get(schedule_key) != condition.get(schedule_key) or
                    magnitude != (requested if active else 0.)):
                problems.append(factor+'_schedule_or_configuration_mismatch'); continue
        if factor == 'noise':
            rmse = evidence.get('rgb_noise_rmse')
            if not _finite(rmse) or rmse < 0 or (rmse > 0) != (magnitude > 0):
                problems.append('noise_missing_or_inconsistent_prejpeg_rmse'); continue
        states[factor] = 'active' if magnitude > 0 else 'inactive'
    return states, problems


def audit_consumed_inputs(trace, *, run_id, condition=None):
    """Inspect a caller-verified single-run trace; preserve every observed ID.

    `run_id` must be the recorded episode's absolute ID supplied by the caller,
    not a model-provided path. No files are opened. The caller validates saved
    result/scenario identity before invoking this pure function.
    """
    if not isinstance(trace, list) or not isinstance(run_id, str) or not run_id:
        raise ValueError('A trace array and explicit recorded run_id are required')
    if condition is not None and not isinstance(condition, dict):
        raise ValueError('condition must be a mapping or null')
    released = {}; workers = {}; issues = {}
    def issue(name): issues[name] = issues.get(name, 0)+1
    for row in trace:
        if not isinstance(row, dict): issue('invalid_trace_row'); continue
        identity = _identity(row.get('camera_sequence'), row.get('latest_input_stamp'))
        if identity is not None:
            disturbance = row.get('latest_input_disturbance')
            candidate = {'sensor_disturbance': disturbance,
                         'rgb_noise_rmse': row.get('latest_input_rgb_noise_rmse'),
                         'disturbance_revision': disturbance.get('revision') if isinstance(disturbance, dict) else None}
            signature = _signature(candidate)
            if signature is not None: released.setdefault(identity, {})[signature] = candidate
            else: issue('nonfinite_or_nonjson_released_metadata')
        worker = row.get('worker')
        if not isinstance(worker, dict): continue
        if worker.get('run_id') != run_id: issue('worker_run_identity_mismatch'); continue
        identity = _identity(worker.get('sequence'), worker.get('sim_stamp'))
        if identity is None: issue('invalid_worker_frame_identity'); continue
        item = workers.setdefault(identity, {'observations': 0, 'first_output_observation_sim_s': None,
                                            'direct': {}, 'invalid_direct': False, 'has_direct': False})
        item['observations'] += 1
        observed = row.get('sim_time_s')
        if _finite(observed):
            old = item['first_output_observation_sim_s']
            item['first_output_observation_sim_s'] = observed if old is None else min(old, observed)
        if 'consumed_input' not in worker: continue
        item['has_direct'] = True
        consumed = worker['consumed_input']
        if (not isinstance(consumed, dict) or type(consumed.get('schema_version')) is not int or
                consumed['schema_version'] != 1):
            item['invalid_direct'] = True; issue('invalid_direct_consumed_schema'); continue
        if _identity(consumed.get('sequence'), consumed.get('sim_stamp')) != identity:
            item['invalid_direct'] = True; issue('outer_inner_frame_identity_mismatch'); continue
        disturbance = consumed.get('sensor_disturbance')
        revision = consumed.get('disturbance_revision')
        if (revision is not None and (type(revision) is not int or revision < 0 or
                (isinstance(disturbance, dict) and disturbance.get('revision') is not None and disturbance['revision'] != revision))):
            item['invalid_direct'] = True; issue('direct_revision_mismatch'); continue
        candidate = {key: copy.deepcopy(consumed.get(key)) for key in
                     ('sensor_disturbance', 'rgb_noise_rmse', 'disturbance_revision')}
        signature = _signature(candidate)
        if signature is None:
            item['invalid_direct'] = True; issue('nonfinite_or_nonjson_consumed_metadata'); continue
        item['direct'][signature] = candidate

    frames = []; counts = {key: {'active': 0, 'inactive': 0, 'unknown': 0} for key in ('noise', 'delay')}
    sources = {'direct_worker_metadata': 0, 'same_run_exact_frame_join': 0, 'unavailable': 0}
    for (sequence, stamp), item in sorted(workers.items(), key=lambda pair: (pair[0][1], pair[0][0])):
        source = 'unavailable'; evidence = None; reason = None
        if item['has_direct']:
            if item['invalid_direct']: reason = 'invalid_direct_metadata_not_replaced_with_legacy_join'
            elif len(item['direct']) != 1: reason = 'conflicting_direct_metadata_for_same_frame'
            else: source = 'direct_worker_metadata'; evidence = next(iter(item['direct'].values()))
        else:
            candidates = released.get((sequence, stamp), {})
            if len(candidates) == 1: source = 'same_run_exact_frame_join'; evidence = next(iter(candidates.values()))
            else: reason = 'ambiguous_exact_frame_join' if candidates else 'no_exact_recorded_released_frame'
        states, problems = _factor_states(evidence, condition) if evidence is not None else (
            {'noise': 'unknown', 'delay': 'unknown'}, [reason])
        observed = item['first_output_observation_sim_s']
        age = observed-stamp if observed is not None and observed >= stamp else None
        frame = {'sequence': sequence, 'input_capture_sim_s': stamp, 'source': source,
                 'recorded_output_observations': item['observations'],
                 'first_output_observation_sim_s': observed, 'input_age_at_output_observation_s': age,
                 'states': states, 'issues': problems, 'exact_delivery_latency_verified': False}
        if evidence is not None: frame['consumed_source_evidence'] = evidence
        frames.append(frame); sources[source] += 1
        for factor, state in states.items(): counts[factor][state] += 1
    return {'schema': VERSION, 'status': 'complete' if frames else 'unavailable_no_same_run_worker_frames',
            'configuration_checked': condition is not None,
            'population': 'observed unique worker-consumed source-frame updates',
            'observed_unique_consumed_frames': len(frames), 'observed_released_frame_identities': len(released),
            'factor_counts': counts, 'evidence_sources': sources, 'frames': frames,
            'issues': issues, 'limitations': LIMITATIONS}


def record_episode_consumption(result, runtime_root):
    """Write an explicitly derived sidecar and return only its compact counts.

    Never changes result.json, scenario.json or the original trace. Callers may
    attach the returned object to their derived campaign role/history record.
    """
    from agent_context import _trace_document
    folder = Path(result['result_dir']).resolve()
    folder.relative_to(Path(runtime_root).resolve())
    rows, provenance = _trace_document(result, runtime_root)
    if rows is None:
        full = {'schema': VERSION, 'status': 'unavailable_evidence_identity', 'configuration_checked': False,
                'population': 'observed unique worker-consumed source-frame updates',
                'reason': provenance['status'], 'frames': [], 'limitations': LIMITATIONS}
    else:
        scenario_path = (folder/'scenario.json').resolve(); scenario_path.relative_to(folder)
        scenario = json.loads(scenario_path.read_text())
        actual = hashlib.sha256(json.dumps(scenario, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
        if actual != provenance['configuration_hash']:
            raise ValueError('Scenario changed after consumed-input evidence verification')
        full = audit_consumed_inputs(rows, run_id=str(folder), condition=scenario['condition'])
    full['source'] = provenance
    full['derived_artifact'] = True
    full['original_result_and_trace_preserved'] = True
    data = (json.dumps(full, indent=2, allow_nan=False)+'\n').encode('utf-8')
    # Fresh temporary file plus rename never follows a preexisting sidecar
    # symlink. The verified episode folder is the only writable destination.
    with tempfile.NamedTemporaryFile(dir=folder, prefix='.worker_input_exposure_', suffix='.tmp', delete=False) as stream:
        stream.write(data); temporary = Path(stream.name)
    destination = folder/'worker_input_exposure.json'
    try:
        temporary.replace(destination)
    finally:
        temporary.unlink(missing_ok=True)
    compact = {key: copy.deepcopy(value) for key, value in full.items() if key != 'frames'}
    compact['artifact'] = {'file': destination.name, 'sha256': hashlib.sha256(data).hexdigest(), 'bytes': len(data)}
    return compact
