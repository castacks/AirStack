"""Inactive encounter-summary dispatch for the next campaign protocol.

Missing versions retain the exact historical projection. Callers must validate
the source trace/scenario before supplying rows; this module does not read files.
"""
from __future__ import annotations

import math

from encounter_context import VERSION, compact_encounter, encoded_bytes, DEFAULT_BYTE_BUDGET

LEGACY_FIELDS = ('schema', 'status', 'thresholds', 'route_length_m', 'trace_rows', 'alignment',
                 'overall', 'gaps', 'issues', 'obstacles', 'turns', 'limitations')
SCHEDULE_CLOCK = 'simulation_seconds_after_first_published_trajectory'
LEGACY_DETAIL_LEVELS = ((4, 4, 20), (2, 2, 10), (0, 0, 4))
DETAIL_LEVELS = ((2, 2, 20), (1, 1, 10), (0, 0, 4))
INSPECTION_NOTES = (
    'For operation inspect or stop, plan.prediction and plan.next_budget_intent must be null. '
    'Plan allocation may reserve future flight pairs; an inspect step may use budget_intent diagnose '
    'with pair_cost 0, without becoming a flight experiment. '
    'Telemetry requests return a bounded window of requested sim_time_s +/-2 simulation seconds; '
    'null requests the latest available window. Image timestamps are source evidence times, not mission elapsed time.'
)


def validate_version(version):
    if version is not None and version != VERSION:
        raise ValueError(f'Unsupported encounter_context_version: {version!r}')


def detail_levels(version):
    """New version changes automatic detail only; exact history core stays intact."""
    validate_version(version)
    return LEGACY_DETAIL_LEVELS if version is None else DETAIL_LEVELS


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value) and value >= 0


def verified_schedule_anchor(rows, run_id):
    """Find one recorded schedule anchor from identity-matched consumed metadata.

    This establishes the schedule's reference clock only. It does not turn the
    bridge-availability population into exact consumption-time observations.
    Missing, inconsistent or malformed anchors produce no relative times.
    """
    values = set(); frames = set(); invalid = []; prearmed = 0
    if not isinstance(rows, list) or not isinstance(run_id, str) or not run_id:
        return {'status': 'unavailable', 'reason': 'Missing verified same-run trace identity',
                'schedule_start_sim_s': None}
    for index, row in enumerate(rows):
        worker = row.get('worker') if isinstance(row, dict) else None
        if not isinstance(worker, dict) or worker.get('run_id') != run_id:
            continue
        consumed = worker.get('consumed_input')
        if consumed is None:
            continue
        valid = (isinstance(consumed, dict) and type(consumed.get('schema_version')) is int
                 and consumed['schema_version'] == 1 and type(worker.get('sequence')) is int
                 and worker['sequence'] >= 0 and _finite(worker.get('sim_stamp'))
                 and type(consumed.get('sequence')) is int
                 and consumed['sequence'] == worker['sequence']
                 and _finite(consumed.get('sim_stamp'))
                 and consumed['sim_stamp'] == worker['sim_stamp'])
        if not valid:
            invalid.append(f'observations[{index}].worker.consumed_input.identity'); continue
        disturbance = consumed.get('sensor_disturbance')
        revision = consumed.get('disturbance_revision')
        if (not isinstance(disturbance, dict) or type(revision) is not int or revision < 0
                or type(disturbance.get('revision')) is not int or disturbance['revision'] != revision):
            invalid.append(f'observations[{index}].worker.consumed_input.disturbance_revision'); continue
        anchor = disturbance.get('schedule_start_sim_s')
        if anchor is None:
            prearmed += 1; continue
        if disturbance.get('schedule_clock') != SCHEDULE_CLOCK or not _finite(anchor):
            invalid.append(f'observations[{index}].worker.consumed_input.sensor_disturbance.schedule_clock_or_start'); continue
        values.add(anchor); frames.add((worker['sequence'], worker['sim_stamp']))
    result = {'status': 'unavailable', 'schedule_start_sim_s': None,
              'method': 'Unique anchor recorded in exact same-run, same-frame consumed_input metadata',
              'schedule_clock': SCHEDULE_CLOCK,
              'unique_source_frame_count': len(frames), 'pre_arming_observations': prearmed,
              'invalid_observations': len(invalid), 'first_invalid_source_fields': invalid[:4]}
    if invalid:
        result['reason'] = 'Malformed same-run consumed schedule metadata; no anchor inferred'
    elif len(values) != 1:
        result['reason'] = ('No recorded consumed-input schedule anchor' if not values else
                            'Conflicting recorded schedule anchors; no anchor inferred')
    else:
        result.update(status='verified', schedule_start_sim_s=next(iter(values)))
    return result


def summarize(analysis, *, evidence_id, source, version=None, rows=None,
              run_id=None, byte_budget=DEFAULT_BYTE_BUDGET):
    """Versioned projection; missing version is byte-compatible historical output."""
    validate_version(version)
    if version is None:
        from agent_context import _present
        return _present(analysis, LEGACY_FIELDS)
    anchor = verified_schedule_anchor(rows, run_id)
    # Reserve the precise anchor provenance before compacting. Unknown anchors
    # remain explicit and cannot be inferred from takeoff or a visual timestamp.
    extra = encoded_bytes({'verified_schedule_anchor': anchor}) + 128
    if type(byte_budget) is not int or byte_budget <= extra:
        raise ValueError('Encounter byte budget cannot fit schedule provenance and complete phase rows')
    result = compact_encounter(analysis, evidence_id=evidence_id, source=source,
                               schedule_start_sim_s=anchor['schedule_start_sim_s'],
                               byte_budget=byte_budget-extra)
    result['verified_schedule_anchor'] = anchor
    if encoded_bytes(result) > byte_budget:
        raise ValueError('Encounter summary exceeds byte budget; no phase rows silently omitted')
    return result
